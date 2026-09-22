//! Pure-Rust ESP32-C6 Wi-Fi TX pipeline demonstrator, running on top of the esp-radio
//! (esp-wifi-sys) blob substrate. It transmits a beacon (SSID "CR-RUST") whose whole
//! per-frame TX path is driven by Rust on the REAL blob scheduler state, with only a
//! minimal, documented set of blob leaves left (see the `blob` extern blocks).
//!
//! Mechanism: while the modem sleeps, the Wi-Fi MAC clocks are gated (every MAC register
//! write is dropped, so the PLCP0_ENABLE launch bits 0xc0000000 never latch) and the PM
//! TX-block bits in WDEV_PM_TXBLOCK_RETENTION @ 0x600a4ca8 are set (0x00ff1000: an armed
//! frame never launches). The per-frame modem-PM wake (`pm_wake::cr_pm_on_data_tx`, a Rust
//! port of the blob's pm_on_data_tx -> pm_disconnected_wake -> ROM wifi_rf_phy_enable chain)
//! re-enables the clocks and the PHY through esp-radio's own OSI callbacks and then performs
//! the hal_mac_init store that clears the block bits. With the MAC active, our Rust drives
//! the full pipeline on the blob's own scheduler state:
//!   submit (cr_ppTxPkt) -> AC map + PM-wake (cr_ppMapTxQueue) -> enqueue onto the TxRxCxt
//!   pending list -> pop (cr_ppGetTxframe) -> schedule (cr_ppProcessTxQ) -> arm
//!   (cr_lmacTxFrame -> cr_lmacSetTxFrame -> Rust hal_mac_tx register ops) -> completion +
//!   recycle (cr_complete). State lives in the blob globals: our_instances @ *0x4004ffe0
//!   (per-AC lmac txq blocks, stride 0x34), TxRxCxt pending @ *0x4087ff80, and g_pm.
//!
//! Module map (top to bottom):
//!   `shims`       -- the libpp.a `blob_<fn>` tail-call shims for the leaves we still delegate
//!   `mmio`        -- volatile load/store helpers
//!   `mac_reg`     -- typed MMIO layer: every WIFI MAC TX register + per-slot addressing + bits
//!   `blob_layout` -- typed views over the blob's TX structures (Eb, TxInfo, DmaDesc, LmacTxq,
//!                    PendingQ) and the ROM pointer cells
//!   `hal_mac_tx`  -- the hal_mac_tx.o register ops, reimplemented (exported by symbol name)
//!   `eb_pool`     -- our own eb packet-buffer pool (no blob esf_buf on the hot path)
//!   `pm_wake`     -- the modem-PM wake FSM (pm_on_data_tx chain), reimplemented
//!   `pipeline`    -- the per-frame TX pipeline: submit / map / enqueue / pop / arm / complete
//!   `config`      -- build-time selectors (experiments, A/B, control-only)
//!   `diag`        -- out-of-band diagnostics (boot probe, PM snapshot, TX-power dump)
//!   `beacon`      -- the beacon payloads
//!   `main`        -- bring-up + the sustained TX loop + the [CR.H] oracle
//!
//! WARNING — blob-version-specific addresses. Every fixed address/offset that is NOT a
//! hardware MMIO register is specific to the esp-wifi-sys 0.3.0 blob and MUST be re-derived
//! per blob version from the linked-ELF / ROM disassembly: our_instances (*0x4004ffe0),
//! TxRxCxt pending (*0x4087ff80), wDevCtrl_ptr (*0x4087ff68) + g_if_enabled_mask (+0x31),
//! the g_pm field offsets, and the esf_buf pool list heads. A wrong base or offset does not
//! fault loudly — it silently reads/writes the wrong memory. That is exactly the bug that
//! bit us: a stale ic_interface_enabled mask address read 0, sent every frame down the drop
//! path, and the resulting double-free of the eb corrupted the heap. Re-verify all of these
//! against the disasm when bumping the blob. Blob globals that live in the linked .data/.bss
//! (g_pm, g_mesh_is_started, lmacConfMib) additionally MOVE with the link layout, so they
//! are linked as extern statics, never hardcoded.
//!
//! Full write-up (investigation, register/struct model, remaining blob surface):
//! docs/cleanroom_tx.md in the esp32c6-open-mac repo.

#![no_std]
#![no_main]

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, delay::Delay, time::Duration, timer::timg::TimerGroup};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

// ============================================================================
// shims — libpp.a interposition
// ============================================================================
/// The cross-boundary hal_mac_tx.o / lmac.o symbols were renamed in libpp.a to `blob_<fn>`.
/// The global_asm! block re-provides `<fn>` as an ABI-perfect tail-call into `blob_<fn>`, so
/// pp/lmac call OUR symbols; each `<fn>` reimplemented in Rust (`hal_mac_tx`) simply omits its
/// shim (our strong def wins). The shims that remain are blob leaves we deliberately keep.
///
/// `tail` preserves every argument register, so a shim is a perfectly transparent passthrough
/// regardless of the real signature.
mod shims {
    core::arch::global_asm!(
        r#"
        .section .text.deblob_shims,"ax",@progbits
        .macro SHIM name
        .globl \name
        .type  \name, @function
        \name:
        tail blob_\name
        .size \name, .-\name
        .endm

        // ---- hal_mac_tx.o leaves kept blob (HE/HE-TB only, never exercised by a legacy beacon,
        // plus the PHY TX-power calibration wrapper) ----
        SHIM hal_init_tx_pwr
        SHIM mac_tx_set_hesig
        SHIM mac_tx_set_mplen
        SHIM mac_tx_set_tb

        // ---- lmac.o cross-boundary shims (46 functions -> tail blob_<fn>). ----
        // NOTE: functions with a ROM *ABS* symbol at 0x40000xxx (GetAccess, is_lmac_idle, lmacIsIdle,
        // lmacIsLongFrame, lmacReachShortLimit, lmacReachLongLimit, lmacDiscardAgedMSDU,
        // lmacPostTxComplete, lmacProcessAckTimeout, lmacProcessAllTxTimeout, lmacProcessCollisions,
        // lmacProcessShortFrameSuccess, lmacProcessLongFrameSuccess, lmacRecycleMPDU, lmacRxDone) are
        // ROM-provided. Our strong Rust defs for those get --gc-sections'd in favour of the ROM copy,
        // and dropping their shim binds callers to the ROM version -- which uses different global
        // state than libpp and STALLS TX. So they MUST stay shims (-> blob_<fn> = libpp copy). Only
        // NON-ABS lmac functions are interposable; the one de-blobbed is lmac_update_tx_statistic.
        SHIM GetAccess
        SHIM is_lmac_idle
        SHIM lmacIsIdle
        SHIM lmacAdjustTimestamp
        SHIM lmacDisableTransmit
        SHIM lmacDiscardAgedMSDU
        SHIM lmacDiscardMSDU
        SHIM lmacEndFrameExchangeSequence
        SHIM lmacEndRetryAMPDUFail
        SHIM lmacInit
        SHIM lmacIsLongFrame
        SHIM lmacMSDUAged
        SHIM lmacPostTxComplete
        SHIM lmacProcessAckTimeout
        SHIM lmacProcessAllTxTimeout
        SHIM lmacProcessCollisions
        SHIM lmacProcessCollisions_task
        SHIM lmacProcessCtsTimeout
        SHIM lmacProcessLongFrameSuccess
        SHIM lmacProcessLongRetryFail
        SHIM lmacProcessModemStateRxBeacon
        SHIM lmacProcessRxSucData
        SHIM lmacProcessShortFrameSuccess
        SHIM lmacProcessShortRetryFail
        SHIM lmacProcessTxComplete
        SHIM lmacProcessTxError
        SHIM lmacProcessTxopQComplete
        SHIM lmacProcessTxRtsError
        SHIM lmacProcessTxSuccess
        SHIM lmacProcessTxTimeout
        SHIM lmacReachLongLimit
        SHIM lmacReachShortLimit
        SHIM lmac_record_txtime
        SHIM lmacRecycleMPDU
        SHIM lmacReleaseTxopQueue
        SHIM lmacRequestTxopQueue
        SHIM lmacRetryTxFrame
        SHIM lmacRxDone
        SHIM lmacSetAcParam
        SHIM lmacSetMuEDCAParam
        SHIM lmacSetTxFrame
        SHIM lmac_stop_hw_txq
        SHIM lmacTxDone
        SHIM lmacTxFrame
        SHIM lmacGetTxFrame
    "#
    );

    /// blob `lmac_update_tx_statistic`: empty in the blob (just `ret`; blob_ at 0x42025b8c). Its
    /// address is installed into the wdev funcs pointer table by wdev_funcs_init and invoked
    /// indirectly. Reimplemented as a no-op (correct by inspection: the blob body is empty).
    ///
    /// This is the ONLY lmac function that can be safely interposed. BLOCKER (evidence-backed; do
    /// not retry blindly): any lmac function that touches the `our_instances` context regresses TX
    /// to ~10-40% steady, while this no-op holds the full rate. lmacGetTxFrame was tried as safe
    /// Rust, as inline-asm, and as a #[naked] copy BYTE-IDENTICAL to blob_lmacGetTxFrame
    /// (0x420256de) at a different address -- all regress, in flash and in IRAM; the init-time
    /// lmacSetAcParam regresses too. A byte-identical copy at a DIFFERENT address killing TX rules
    /// out the code itself and points to a placement/dispatch dependency (a ROM/wdev pointer table
    /// or an i-cache co-location constraint on the timing-sensitive TX path). Combined with the
    /// ROM *ABS* wall above, lmac.o is not de-blobbable via symbol interposition beyond this
    /// no-op; the pipeline below therefore drives the lmac sequence itself instead of interposing.
    /// Full analysis + reproduction: docs/deblob_progress.md.
    #[unsafe(no_mangle)]
    pub extern "C" fn lmac_update_tx_statistic() {}

    // hal_mac_tx.o leaves still called by our Rust `hal_mac_tx_set_ppdu`: HT/HE-SIG and the
    // aggregate-TXOP fill. The legacy 1 Mbit DSSS beacon path calls but does not exercise them
    // (the HE/AMPDU branches are not taken).
    unsafe extern "C" {
        pub fn mac_tx_set_hesig(); // HE-SIG field (not taken by a legacy beacon)
        pub fn mac_tx_set_htsig(p: *mut u8, param2: i32); // HT-SIG field (not taken by a legacy beacon)
        pub fn hal_mac_fill_hwtxop(eb: u32, depth: u32, idx: u32); // per-MPDU TXOP fill (dead for single frames)
    }
}

// ============================================================================
// mmio — volatile load/store helpers
// ============================================================================
/// Raw volatile accessors (this bin cannot use esp-wifi-hal's PAC: esp-radio owns WIFI). Used for
/// both the MAC registers and the blob's SRAM structures.
mod mmio {
    #[inline(always)]
    pub unsafe fn rd(addr: u32) -> u32 {
        unsafe { core::ptr::read_volatile(addr as usize as *const u32) }
    }
    #[inline(always)]
    pub unsafe fn wr(addr: u32, val: u32) {
        unsafe { core::ptr::write_volatile(addr as usize as *mut u32, val) }
    }
    #[inline(always)]
    pub unsafe fn rd16(addr: u32) -> u16 {
        unsafe { core::ptr::read_volatile(addr as usize as *const u16) }
    }
    #[inline(always)]
    pub unsafe fn wr16(addr: u32, val: u16) {
        unsafe { core::ptr::write_volatile(addr as usize as *mut u16, val) }
    }
    #[inline(always)]
    pub unsafe fn rd8(addr: u32) -> u8 {
        unsafe { core::ptr::read_volatile(addr as usize as *const u8) }
    }
    #[inline(always)]
    pub unsafe fn wr8(addr: u32, val: u8) {
        unsafe { core::ptr::write_volatile(addr as usize as *mut u8, val) }
    }
}
use mmio::{rd, rd8, rd16, wr, wr8, wr16};

// ============================================================================
// mac_reg — typed MMIO register layer
// ============================================================================
/// The WIFI MAC TX register file (all real hardware MMIO, stable across blob versions). Per-slot
/// registers live in two blocks and the queue index maps to slots in REVERSE (queue 0 = highest
/// slot), so the address for slot `s` is `base - s*stride`: `CONF_STRIDE` (0x10) is the per-slot
/// config block at 0x600a4d6x, `SLOT_STRIDE` (0x74) the completion/TXOP block at 0x600a54xx. The
/// `*(slot)` fns compute those addresses; `mmio::rd`/`wr` do the actual access.
mod mac_reg {
    /// stride between adjacent slots in the 0x600a4d6x config block.
    pub const CONF_STRIDE: u32 = 0x10;
    /// stride between adjacent slots in the 0x600a54xx completion/TXOP block.
    pub const SLOT_STRIDE: u32 = 0x74;

    // -- config block (CONF_STRIDE) --
    pub const PLCP0_ENABLE: u32 = 0x600a_4d6c; // dma/len/format word + arm bits (SLOT_VALID|SLOT_ENABLED)
    pub const CONF0: u32 = 0x600a_4d60; // RTS/txop protect-enable + txop-valid (bit31)
    pub const CONF1: u32 = 0x600a_4d68; // EDCA aifsn/cw/timeout + PTI top nibble
    pub const CONF1_REAL: u32 = 0x600a_4d64; // slot conf1+4; clr_mplen HE-TB mplen-valid = bit3

    // -- completion/TXOP block (SLOT_STRIDE) --
    pub const RESP_DUR: u32 = 0x600a_54bc; // response-rate/duration + TXOP depth/count
    pub const TXLEN: u32 = 0x600a_54b8; // OFDM/HT TX length
    pub const PLCP1: u32 = 0x600a_5488; // rate/keyslot/HT-HE-mode/legacy-length/LDPC
    pub const PLCP_RATE_DUR: u32 = 0x600a_54ac; // PLCP rate/duration word
    pub const PROT_THRESH: u32 = 0x600a_548c; // RTS/txop-duration threshold (he_set_tx_protection)
    pub const PTI: u32 = 0x600a_5490; // BT-coex packet-traffic-indication
    pub const PMD: u32 = 0x600a_54e8; // PMD / completion result
    pub const BA_BITMAP: u32 = 0x600a_54d4; // block-ack bitmap word0 (+4 = word1, +8 = header)
    pub const AUX_54E0: u32 = 0x600a_54e0; // completion aux (HE-TB / BA info)
    pub const AUX_54D0: u32 = 0x600a_54d0; // completion aux (last-tx-is-tb flag, ...)
    pub const AUX_54EC: u32 = 0x600a_54ec; // completion aux (sub-status / tb_sent)
    pub const AUX_54F4: u32 = 0x600a_54f4; // completion aux

    // -- global TX state-machine registers (not per-slot) --
    pub const STATE_A: u32 = 0x600a_4cb0;
    pub const STATE_B: u32 = 0x600a_4cb8;
    pub const CLR_STATE_A: u32 = 0x600a_4cac;
    pub const CLR_STATE_B: u32 = 0x600a_4cb4;
    pub const TX_MIN_PWR: u32 = 0x600a_4400;
    pub const PM_TXBLOCK_RETENTION: u32 = 0x600a_4ca8; // the TX interlock (0x00ff1000 = blocked, 0 = active)
    pub const SYS_TIMER: u32 = 0x600a_d000; // free-running WDEV system timer (hal_now)

    // -- hal_attenna_init: RESP_DUR block loop bound + global antenna reg --
    pub const RESP_DUR_BLOCK_END: u32 = 0x600a_511c; // attenna_init RESP_DUR loop lower bound
    pub const ATTENNA_GLOBAL: u32 = 0x600a_42cc; // global antenna/PHY-select reg

    // -- diagnostic-only TX power / gain-mem / FE / BB registers (pwr_dump) --
    pub const GAIN_MEM_C8: u32 = 0x600a_08c8;
    pub const GAIN_MEM_CC: u32 = 0x600a_08cc;
    pub const GAIN_MEM_D0: u32 = 0x600a_08d0;
    pub const GAIN_MEM_D4: u32 = 0x600a_08d4;
    pub const FE_0910: u32 = 0x600a_0910;
    pub const FE_00C0: u32 = 0x600a_00c0;
    pub const BB_7030: u32 = 0x600a_7030;

    // -- PLCP0_ENABLE arm bits + CONF0/PLCP0 field bits --
    pub const SLOT_VALID: u32 = 0x8000_0000; // PLCP0_ENABLE bit31
    pub const SLOT_ENABLED: u32 = 0x4000_0000; // PLCP0_ENABLE bit30
    pub const SLOT_ARM: u32 = SLOT_VALID | SLOT_ENABLED; // 0xc0000000 (hal_mac_txq_enable)
    pub const SLOT_ARM_CLEAR: u32 = !SLOT_ARM; // 0x3fffffff (hal_mac_txq_disable)
    pub const CONF0_BIT31: u32 = 0x8000_0000; // CONF0 protect-enable / txop-valid
    pub const PLCP0_TXOP: u32 = 0x0040_0000; // PLCP0_ENABLE txop bit22 (set_txop_q)
    /// PM_TXBLOCK_RETENTION: the per-queue TX-block byte (bits 16-23) + bit12, set by
    /// hal_mac_deinit when the modem sleeps and cleared by hal_mac_init on wake.
    pub const PM_TXBLOCK_BITS: u32 = 0x00ff_1000;

    #[inline(always)]
    const fn conf(base: u32, slot: u32) -> u32 {
        base.wrapping_sub(slot.wrapping_mul(CONF_STRIDE))
    }
    #[inline(always)]
    const fn perslot(base: u32, slot: u32) -> u32 {
        base.wrapping_sub(slot.wrapping_mul(SLOT_STRIDE))
    }
    #[inline(always)]
    pub fn plcp0_enable(slot: u32) -> u32 {
        conf(PLCP0_ENABLE, slot)
    }
    #[inline(always)]
    pub fn conf0(slot: u32) -> u32 {
        conf(CONF0, slot)
    }
    #[inline(always)]
    pub fn conf1(slot: u32) -> u32 {
        conf(CONF1, slot)
    }
    #[inline(always)]
    pub fn conf1_real(slot: u32) -> u32 {
        conf(CONF1_REAL, slot)
    }
    #[inline(always)]
    pub fn resp_dur(slot: u32) -> u32 {
        perslot(RESP_DUR, slot)
    }
    #[inline(always)]
    pub fn txlen(slot: u32) -> u32 {
        perslot(TXLEN, slot)
    }
    #[inline(always)]
    pub fn plcp1(slot: u32) -> u32 {
        perslot(PLCP1, slot)
    }
    #[inline(always)]
    pub fn plcp_rate_dur(slot: u32) -> u32 {
        perslot(PLCP_RATE_DUR, slot)
    }
    #[inline(always)]
    pub fn prot_thresh(slot: u32) -> u32 {
        perslot(PROT_THRESH, slot)
    }
    #[inline(always)]
    pub fn pti(slot: u32) -> u32 {
        perslot(PTI, slot)
    }
    #[inline(always)]
    pub fn pmd(slot: u32) -> u32 {
        perslot(PMD, slot)
    }
}

// ============================================================================
// blob_layout — typed views over the blob's TX data structures
// ============================================================================
/// Thin newtypes over a base address. Every accessor is one volatile load/store at
/// `base + OFFSET`, so the access pattern is exactly the raw `*(base+0x..)` it replaces -- only
/// the names are new. The layouts were reversed from the 0.3.0 blob / C6 ROM and a live eb dump
/// (docs/cleanroom_tx.md, docs/deblob_progress.md); all offsets are blob-version-specific.
///
/// The per-AC lmac TX control array `our_instances` has its base in the fixed pp/ROM pointer
/// cell `our_instances_ptr` (0x4004ffe0): the blob materialises it with `lui a5,0x40050;
/// lw a5,-0x20(a5)` (verified in the LINKED blob_GetAccess @0x40805126) and indexes
/// `our_instances[ac] = base + ac*0x34` (5 ACs). `TxRxCxt` (the per-AC pending lists) hangs off
/// the `pTxRx` cell (0x4087ff80) with the same stride.
mod blob_layout {
    use super::{rd, rd8, rd16, wr, wr8, wr16};

    // -- ROM interface cells (fixed by the C6 ROM, stable across blob versions) --
    /// `our_instances_ptr`: -> the lmac per-AC TX control array (`LmacTxq`, stride 0x34).
    pub const OUR_INSTANCES_PTR: u32 = 0x4004_ffe0;
    /// `pTxRx` (TxRxCxt): -> the per-AC pending lists (`PendingQ`, stride 0x34).
    pub const TXRXCXT_PTR: u32 = 0x4087_ff80;
    /// `wDevCtrl_ptr`: -> wDevCtrl; `g_if_enabled_mask` byte at +0x31 (ROM ic_interface_enabled).
    pub const WDEVCTRL_PTR: u32 = 0x4087_ff68;
    const WDEVCTRL_IF_ENABLED_MASK: u32 = 0x31;
    /// stride of both per-AC arrays (our_instances and TxRxCxt).
    pub const AC_STRIDE: u32 = 0x34;

    #[inline(always)]
    pub fn our_instances_base() -> u32 {
        unsafe { rd(OUR_INSTANCES_PTR) }
    }
    /// ROM `ic_interface_enabled` source byte: `*(wDevCtrl_ptr) + 0x31`.
    #[inline(always)]
    pub fn if_enabled_mask() -> u8 {
        unsafe { rd8(rd(WDEVCTRL_PTR) + WDEVCTRL_IF_ENABLED_MASK) }
    }

    /// A TX DMA descriptor: `[size|ctrl, frame_ptr, next]`. `ctrl` packs the buffer size (bits 0-13),
    /// the length (bits 14-27), and owner/eof/link bits (31/30/29).
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct DmaDesc(pub u32);
    impl DmaDesc {
        pub const CTRL_LEN_SHIFT: u32 = 0xe;
        pub const CTRL_LEN_MASK: u32 = 0x3fff;
        /// bits 0-13 (buffer size) + 28-31 (owner/eof/...) -- everything but the length field.
        pub const CTRL_KEEP_MASK: u32 = 0xf000_3fff;
        pub const CTRL_OWNER: u32 = 0x8000_0000;
        pub const CTRL_EOF: u32 = 0x4000_0000;
        pub const CTRL_BIT29_CLEAR: u32 = 0xdfff_ffff;

        #[inline(always)]
        pub fn ctrl(self) -> u32 {
            unsafe { rd(self.0) }
        }
        #[inline(always)]
        pub fn set_ctrl(self, v: u32) {
            unsafe { wr(self.0, v) }
        }
        #[inline(always)]
        pub fn frame_ptr(self) -> u32 {
            unsafe { rd(self.0 + 4) }
        }
        #[inline(always)]
        pub fn set_frame_ptr(self, v: u32) {
            unsafe { wr(self.0 + 4, v) }
        }
        #[inline(always)]
        pub fn set_next(self, v: u32) {
            unsafe { wr(self.0 + 8, v) }
        }
        /// length field (bits 14-27) += delta, keeping bits 0-13 and 28-31 (owner/eof/size).
        #[inline(always)]
        pub fn add_len(self, delta: u32) {
            let w = self.ctrl();
            let len = ((w >> Self::CTRL_LEN_SHIFT) & Self::CTRL_LEN_MASK).wrapping_add(delta)
                & Self::CTRL_LEN_MASK;
            self.set_ctrl((len << Self::CTRL_LEN_SHIFT) | (w & Self::CTRL_KEEP_MASK));
        }
    }

    /// The per-frame TX descriptor the blob calls `txinfo` (`*(eb+0x34)`, 0x48 bytes).
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct TxInfo(pub u32);
    impl TxInfo {
        /// +0x00: flags word (bit1 no-ack/broadcast, bit3 data, bit8 RTS, bit10, bit11 QoS-null,
        /// bit12/13 (0x2102 group), bit16 discard, bit19 ack-type, bit20, bit22 AMPDU, bit27,
        /// bit30 HE-ext, bit31 HE).
        pub const FLAGS: u32 = 0x00;
        /// +0x04: frame category (byte; 7 = mgmt) / QoS class nibble (bits 4-7).
        pub const CAT: u32 = 0x04;
        /// +0x0c: rate index (byte; 0 = 1 Mbit DSSS).
        pub const RATE: u32 = 0x0c;
        /// +0x10: keyslot (bits 0-7), key type (bits 8-11), bit18, iface (bit19), AC (bits 20-23).
        pub const WORD10: u32 = 0x10;
        /// +0x18: submit timestamp (hal_now).
        pub const TIMESTAMP: u32 = 0x18;
        /// +0x20: coex PTI (byte); +0x22: PTI high field (u16).
        pub const PTI: u32 = 0x20;
        pub const PTI_HI: u32 = 0x22;
        /// +0x30: protection word (bits 3-12 = RTS/txop threshold, bit17).
        pub const WORD30: u32 = 0x30;
        /// +0x34: protection value passed to hal_he_set_tx_protection.
        pub const WORD34: u32 = 0x34;
        /// +0x40/+0x44: lifetime / timeout (config_timeout).
        pub const LIFETIME_LO: u32 = 0x40;
        pub const LIFETIME_HI: u32 = 0x44;

        pub const WORD10_IFACE_SHIFT: u32 = 0x13;
        pub const WORD10_AC_SHIFT: u32 = 0x14;
        pub const WORD10_AC_CLEAR: u32 = 0xff0f_ffff;

        #[inline(always)]
        pub fn flags(self) -> u32 {
            unsafe { rd(self.0 + Self::FLAGS) }
        }
        #[inline(always)]
        pub fn set_flags(self, v: u32) {
            unsafe { wr(self.0 + Self::FLAGS, v) }
        }
        #[inline(always)]
        pub fn cat_word(self) -> u32 {
            unsafe { rd(self.0 + Self::CAT) }
        }
        #[inline(always)]
        pub fn set_cat(self, v: u8) {
            unsafe { wr8(self.0 + Self::CAT, v) }
        }
        #[inline(always)]
        pub fn rate(self) -> u8 {
            unsafe { rd8(self.0 + Self::RATE) }
        }
        #[inline(always)]
        pub fn set_rate(self, v: u8) {
            unsafe { wr8(self.0 + Self::RATE, v) }
        }
        #[inline(always)]
        pub fn word10(self) -> u32 {
            unsafe { rd(self.0 + Self::WORD10) }
        }
        #[inline(always)]
        pub fn set_word10(self, v: u32) {
            unsafe { wr(self.0 + Self::WORD10, v) }
        }
        /// keyslot byte = low byte of WORD10.
        #[inline(always)]
        pub fn keyslot(self) -> u8 {
            unsafe { rd8(self.0 + Self::WORD10) }
        }
        #[inline(always)]
        pub fn iface(self) -> u32 {
            (self.word10() >> Self::WORD10_IFACE_SHIFT) & 1
        }
        #[inline(always)]
        pub fn ac(self) -> u32 {
            (self.word10() >> Self::WORD10_AC_SHIFT) & 0xf
        }
        #[inline(always)]
        pub fn set_timestamp(self, v: u32) {
            unsafe { wr(self.0 + Self::TIMESTAMP, v) }
        }
        #[inline(always)]
        pub fn pti(self) -> u8 {
            unsafe { rd8(self.0 + Self::PTI) }
        }
        #[inline(always)]
        pub fn pti_hi(self) -> u16 {
            unsafe { core::ptr::read_unaligned((self.0 + Self::PTI_HI) as *const u16) }
        }
        #[inline(always)]
        pub fn word30(self) -> u32 {
            unsafe { rd(self.0 + Self::WORD30) }
        }
        #[inline(always)]
        pub fn word34(self) -> u32 {
            unsafe { rd(self.0 + Self::WORD34) }
        }
        #[inline(always)]
        pub fn lifetime_lo(self) -> u32 {
            unsafe { rd(self.0 + Self::LIFETIME_LO) }
        }
        #[inline(always)]
        pub fn lifetime_hi(self) -> u32 {
            unsafe { rd(self.0 + Self::LIFETIME_HI) }
        }
    }

    /// An `esf_buf` (eb) packet buffer, the unit the pp/lmac scheduler passes around.
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Eb(pub u32);
    impl Eb {
        /// +0x04 / +0x08: two pointers to the (single) TX DMA descriptor.
        pub const DMA_DESC: u32 = 0x04;
        pub const SEC_DESC: u32 = 0x08;
        /// +0x14: u16 bytes prepended in front of the payload (security header shift).
        pub const PREFIX_LEN: u32 = 0x14;
        /// +0x16: u16 payload length.
        pub const PAYLOAD_LEN: u32 = 0x16;
        /// +0x24: u16 flags; bit13 = header already shifted (set by ppProcTxSecFrame).
        pub const FLAGS16: u32 = 0x24;
        pub const FLAG16_HDR_SHIFTED: u16 = 0x2000;
        /// +0x2c: rate-control context (trc); NULL for a raw frame.
        pub const TRC: u32 = 0x2c;
        /// +0x30: pending-list next link (also the aggregate MPDU chain).
        pub const PENDING_NEXT: u32 = 0x30;
        /// +0x34: -> `TxInfo`.
        pub const TXINFO: u32 = 0x34;

        #[inline(always)]
        pub fn addr(self) -> u32 {
            self.0
        }
        #[inline(always)]
        pub fn is_null(self) -> bool {
            self.0 == 0
        }
        #[inline(always)]
        pub fn dma_desc(self) -> DmaDesc {
            DmaDesc(unsafe { rd(self.0 + Self::DMA_DESC) })
        }
        #[inline(always)]
        pub fn sec_desc(self) -> DmaDesc {
            DmaDesc(unsafe { rd(self.0 + Self::SEC_DESC) })
        }
        #[inline(always)]
        pub fn prefix_len(self) -> u16 {
            unsafe { rd16(self.0 + Self::PREFIX_LEN) }
        }
        #[inline(always)]
        pub fn set_prefix_len(self, v: u16) {
            unsafe { wr16(self.0 + Self::PREFIX_LEN, v) }
        }
        #[inline(always)]
        pub fn payload_len(self) -> u16 {
            unsafe { rd16(self.0 + Self::PAYLOAD_LEN) }
        }
        #[inline(always)]
        pub fn set_payload_len(self, v: u16) {
            unsafe { wr16(self.0 + Self::PAYLOAD_LEN, v) }
        }
        #[inline(always)]
        pub fn flags16(self) -> u16 {
            unsafe { rd16(self.0 + Self::FLAGS16) }
        }
        #[inline(always)]
        pub fn set_flags16(self, v: u16) {
            unsafe { wr16(self.0 + Self::FLAGS16, v) }
        }
        #[inline(always)]
        pub fn trc(self) -> u32 {
            unsafe { rd(self.0 + Self::TRC) }
        }
        #[inline(always)]
        pub fn set_trc(self, v: u32) {
            unsafe { wr(self.0 + Self::TRC, v) }
        }
        #[inline(always)]
        pub fn pending_next(self) -> u32 {
            unsafe { rd(self.0 + Self::PENDING_NEXT) }
        }
        #[inline(always)]
        pub fn set_pending_next(self, v: u32) {
            unsafe { wr(self.0 + Self::PENDING_NEXT, v) }
        }
        /// address of the next-link field (the pending list's tail points here).
        #[inline(always)]
        pub fn pending_next_addr(self) -> u32 {
            self.0 + Self::PENDING_NEXT
        }
        #[inline(always)]
        pub fn txinfo(self) -> TxInfo {
            TxInfo(unsafe { rd(self.0 + Self::TXINFO) })
        }
    }

    /// One `our_instances[ac]` lmac TX control block (stride 0x34). Also the `ctx` argument of the
    /// hal_mac_tx.o functions (`*ctx` = cur_eb, ctx+4 = slot).
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct LmacTxq(pub u32);
    impl LmacTxq {
        pub const CUR_EB: u32 = 0x00; // armed frame ptr
        pub const SLOT: u32 = 0x04; // hardware slot / queue index (byte)
        pub const AIFSN: u32 = 0x05; // byte, low nibble
        pub const BACKOFF: u32 = 0x06; // u16 random backoff (masked to CW)
        pub const CW: u32 = 0x08; // byte, CW exponent
        pub const STATE: u32 = 0x12; // 0 idle / 1 armed / 3 / 5 success / 6 error
        pub const TXOP_COUNT: u32 = 0x1c; // TXOP burst count
        pub const TXOP_DEPTH: u32 = 0x1d; // TXOP burst depth
        pub const SKIP_FLAGS: u32 = 0x28; // byte; bit2 = skip clr_mplen in get_txq_complete
        /// +0x40/+0x41: OFDM/HT channel-width fields read by mac_tx_set_len (relative to the
        /// txq of the frame's AC inside the our_instances array).
        pub const TXLEN_CBW_SEL: u32 = 0x40;
        pub const TXLEN_CBW: u32 = 0x41;

        pub const STATE_IDLE: u8 = 0;
        pub const STATE_RELEASED: u8 = 3;

        #[inline(always)]
        pub fn for_ac(ac: u32) -> Self {
            Self::in_array(our_instances_base(), ac)
        }
        #[inline(always)]
        pub fn in_array(base: u32, ac: u32) -> Self {
            LmacTxq(base.wrapping_add(ac.wrapping_mul(AC_STRIDE)))
        }
        #[inline(always)]
        pub fn from_ctx(ctx: *mut u8) -> Self {
            LmacTxq(ctx as u32)
        }
        #[inline(always)]
        pub fn ptr(self) -> *mut u8 {
            self.0 as usize as *mut u8
        }
        #[inline(always)]
        pub fn cur_eb(self) -> Eb {
            Eb(unsafe { rd(self.0 + Self::CUR_EB) })
        }
        #[inline(always)]
        pub fn set_cur_eb(self, eb: Eb) {
            unsafe { wr(self.0 + Self::CUR_EB, eb.0) }
        }
        #[inline(always)]
        pub fn slot(self) -> u32 {
            unsafe { rd8(self.0 + Self::SLOT) as u32 }
        }
        #[inline(always)]
        pub fn aifsn(self) -> u8 {
            unsafe { rd8(self.0 + Self::AIFSN) }
        }
        #[inline(always)]
        pub fn backoff(self) -> u16 {
            unsafe { core::ptr::read_unaligned((self.0 + Self::BACKOFF) as *const u16) }
        }
        #[inline(always)]
        pub fn set_backoff(self, v: u16) {
            unsafe { core::ptr::write_unaligned((self.0 + Self::BACKOFF) as *mut u16, v) }
        }
        #[inline(always)]
        pub fn cw(self) -> u8 {
            unsafe { rd8(self.0 + Self::CW) }
        }
        #[inline(always)]
        pub fn state(self) -> u8 {
            unsafe { rd8(self.0 + Self::STATE) }
        }
        #[inline(always)]
        pub fn txop_count(self) -> u32 {
            unsafe { rd8(self.0 + Self::TXOP_COUNT) as u32 }
        }
        #[inline(always)]
        pub fn txop_depth(self) -> u32 {
            unsafe { rd8(self.0 + Self::TXOP_DEPTH) as u32 }
        }
        #[inline(always)]
        pub fn txlen_cbw_sel(self) -> u32 {
            unsafe { rd8(self.0 + Self::TXLEN_CBW_SEL) as u32 }
        }
        #[inline(always)]
        pub fn txlen_cbw(self) -> u32 {
            unsafe { rd8(self.0 + Self::TXLEN_CBW) as u32 }
        }
    }

    /// One TxRxCxt per-AC pending list (stride 0x34): a singly-linked eb list threaded through
    /// `Eb::PENDING_NEXT`; `tail` points at the link field to fill next (== &head when empty).
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct PendingQ(pub u32);
    impl PendingQ {
        pub const HEAD: u32 = 0x20;
        pub const TAIL: u32 = 0x24;
        pub const GUARD_BYTE: u32 = 0x29; // ppGetTxframe: must be 0
        pub const GUARD_WORD: u32 = 0x34; // ppGetTxframe: must be 0

        #[inline(always)]
        pub fn for_ac(ac: u32) -> Self {
            PendingQ(unsafe { rd(TXRXCXT_PTR) }.wrapping_add(ac.wrapping_mul(AC_STRIDE)))
        }
        #[inline(always)]
        pub fn head(self) -> u32 {
            unsafe { rd(self.0 + Self::HEAD) }
        }
        #[inline(always)]
        pub fn set_head(self, v: u32) {
            unsafe { wr(self.0 + Self::HEAD, v) }
        }
        #[inline(always)]
        pub fn head_addr(self) -> u32 {
            self.0 + Self::HEAD
        }
        #[inline(always)]
        pub fn tail(self) -> u32 {
            unsafe { rd(self.0 + Self::TAIL) }
        }
        #[inline(always)]
        pub fn set_tail(self, v: u32) {
            unsafe { wr(self.0 + Self::TAIL, v) }
        }
        #[inline(always)]
        pub fn guard_byte(self) -> u8 {
            unsafe { rd8(self.0 + Self::GUARD_BYTE) }
        }
        #[inline(always)]
        pub fn guard_word(self) -> u32 {
            unsafe { rd(self.0 + Self::GUARD_WORD) }
        }
    }
}
use blob_layout::{DmaDesc, Eb, LmacTxq, PendingQ, TxInfo};

// ============================================================================
// hal_mac_tx — the hal_mac_tx.o register ops, in Rust
// ============================================================================
/// The `#[unsafe(no_mangle)] extern "C"` functions here replace the hal_mac_tx.o shims by symbol
/// name: the blob's lmac/pp call them directly (e.g. for the boot CR-CTRL beacon), and our own
/// PPDU path (`pipeline::cr_lmacSetTxFrame` -> `hal_mac_tx_set_ppdu` -> set_plcp0/plcp1/txop_q/
/// len/pti) calls them too. Each was verified against the 0.3.0 blob decompilation; the HE/HE-TB
/// branches a legacy DSSS beacon never takes are delegated to the blob leaves in `shims` or left
/// unimplemented (documented per function). LESSON carried from the bring-up: the TX-setup path is
/// timing-sensitive -- extra per-call MMIO readbacks or atomics here stall the TX queue -- so the
/// bodies keep the blob's read/compute/write shape and count.
mod hal_mac_tx {
    use super::{rd, wr, mac_reg, shims, Eb, LmacTxq, TxInfo};

    /// blob `mac_tx_set_txop_q`: program the slot's TXOP burst fields. For depth>2 it just clears
    /// the RESP_DUR txop nibble; otherwise it sets RESP_DUR depth/count, CONF0 txop-valid bit
    /// (per txinfo bit8), PLCP0_ENABLE bit22 (per txinfo bits6-7==0x80), and fills each chained
    /// MPDU (dead for a single beacon: the eb+0x30 chain is empty).
    #[unsafe(no_mangle)]
    pub extern "C" fn mac_tx_set_txop_q(param_1: *mut u8) -> u32 {
        unsafe {
            let txq = LmacTxq::from_ctx(param_1);
            let slot = txq.slot();
            let eb = txq.cur_eb();
            let depth = txq.txop_depth();
            let resp_dur = mac_reg::resp_dur(slot);
            let txinfo = eb.txinfo();
            if depth > 2 {
                wr(resp_dur, rd(resp_dur) & 0xf0ff_ffff);
                return 0;
            }
            let count = txq.txop_count();
            wr(
                resp_dur,
                (rd(resp_dur) & 0xf0ff_ffff) | ((count << 0x18) & 0xf000_0000),
            );
            wr(resp_dur, (rd(resp_dur) & 0xcfff_ffff) | (depth << 0x1c));
            let conf0 = mac_reg::conf0(slot);
            if (txinfo.flags() & 0x100) != 0 {
                wr(conf0, rd(conf0) | mac_reg::CONF0_BIT31);
            } else {
                wr(conf0, rd(conf0) & !mac_reg::CONF0_BIT31);
            }
            let plcp0 = mac_reg::plcp0_enable(slot);
            if (txinfo.flags() & 0xc0) == 0x80 {
                wr(plcp0, rd(plcp0) | mac_reg::PLCP0_TXOP);
            } else {
                wr(plcp0, rd(plcp0) & !mac_reg::PLCP0_TXOP);
            }
            // Aggregate chain: fill each MPDU's HW TXOP. Empty for single frames.
            let mut mpdu = eb.pending_next();
            let mut idx: u32 = 1;
            while mpdu != 0 {
                shims::hal_mac_fill_hwtxop(mpdu, txq.txop_depth(), idx);
                if count == idx {
                    break;
                }
                idx = (idx + 1) & 0xff;
                mpdu = Eb(mpdu).pending_next();
            }
        }
        0
    }

    /// Rust mac_tx_get_rts_rate: pure rate -> RTS/response-rate lookup (faithful port of the blob's
    /// jump table). Our 1 Mbit DSSS beacon has rate 0 -> returns 0. No addresses, hardware-independent.
    pub fn cr_mac_tx_get_rts_rate(rate: i32) -> i32 {
        let p = rate;
        if (p.wrapping_sub(0x10) as u32) < 0x14 {
            let mut b = p == 0x10;
            if p > 0x19 {
                b = p == 0x1a;
            }
            if b {
                return 0xb;
            }
            let iv = if p < 0x1a { 0x12 } else { 0x1c };
            if p <= iv {
                return 10;
            }
            let b2 = if p < 0x1a { p < 0x13 } else { p < 0x1d };
            if !b2 {
                return 9;
            }
        } else if p < 8 {
            if p == 0 {
                return 0;
            }
        } else if (p.wrapping_sub(8) as u32) < 8 {
            let u = 1u32 << ((p as u32).wrapping_sub(8) & 0x1f);
            if u & 0x33 != 0 {
                return 9;
            }
            if u & 0x88 != 0 {
                return 0xb;
            }
            if u & 0x44 != 0 {
                return 10;
            }
        }
        if (p.wrapping_sub(1) as u32) < 3 {
            return 1;
        }
        if (p.wrapping_sub(5) as u32) > 2 {
            return 0;
        }
        5
    }

    /// Rust hal_he_set_tx_protection: CONF0 (0x600a4d60-slot*0x10) bit31 = protect-enable, and,
    /// when a threshold is set, the RTS/txop-dur threshold reg (0x600a548c-slot*0x74). Register
    /// writes only. (The blob leaf @0x42073f6a does NOT touch PLCP0_ENABLE's arm bits -- verified
    /// by disasm.)
    unsafe fn cr_hal_he_set_tx_protection(slot: i32, enable: i32, _p3: u32, threshold: i32, val: u32) {
        unsafe {
            let conf0 = mac_reg::conf0(slot as u32);
            let mut v = rd(conf0);
            if enable == 0 {
                v &= !mac_reg::CONF0_BIT31;
            } else {
                v |= mac_reg::CONF0_BIT31;
            }
            wr(conf0, v);
            if threshold != 0 {
                wr(mac_reg::prot_thresh(slot as u32), (val & 0xffff) | 0x1_0000);
            }
        }
    }

    /// Rust mac_tx_set_len: writes the slot RESP_DUR word (0x600a54bc-slot*0x74: response-rate
    /// bits6-13, cbw40 bit1, uVar5 bits22) and, for OFDM/HT rates, the TXLEN word
    /// (0x600a54b8-slot*0x74). For our legacy DSSS beacon (rate 0) only RESP_DUR is written (the
    /// TXLEN branch needs an OFDM rate). `param2` is the our_instances base (used only by the OFDM
    /// branches, not taken by the beacon).
    unsafe fn cr_mac_tx_set_len(ctx: *mut u8, param2: i32) {
        unsafe {
            let txq = LmacTxq::from_ctx(ctx);
            let eb = txq.cur_eb();
            let txinfo = eb.txinfo();
            let frame = eb.dma_desc().frame_ptr();
            let rate = txinfo.rate() as i32;
            let t10 = txinfo.word10();
            let rts = cr_mac_tx_get_rts_rate(rate) as u32;
            let ac = (t10 >> TxInfo::WORD10_AC_SHIFT) & 0xf;
            let ac_txq = LmacTxq::in_array(param2 as u32, ac);
            let mut uvar5 = 1u32;
            if (rate.wrapping_sub(0x10) as u32 & 0xff) < 0x14 {
                uvar5 = ac_txq.txlen_cbw_sel() & 3;
            }
            let slot = txq.slot();
            let resp_dur = mac_reg::resp_dur(slot);
            wr(
                resp_dur,
                uvar5 << 0x16 | ((t10 & 0xf000) == 0x1000) as u32 * 2 | (rts & 0xff) << 6 | 4,
            );
            let flags = txinfo.flags();
            if (flags as i32) >= 0 {
                let r2 = txinfo.rate() as u32;
                let mut uv3 = r2.wrapping_sub(0x10) & 0xff;
                if uv3 < 0x14 {
                    let len = if flags & 0x40_0000 == 0 {
                        rd(frame) & 0x3fff
                    } else {
                        eb.payload_len() as u32 + eb.prefix_len() as u32
                    };
                    if r2 > 0x19 {
                        uv3 = r2 - 0x1a;
                    }
                    let txlen = mac_reg::txlen(slot);
                    let cbw = ac_txq.txlen_cbw() & 3;
                    wr(txlen, cbw << 0x16 | len | uv3 << 0x1c);
                }
            }
        }
    }

    /// blob `mac_tx_set_plcp0`: program the slot's PLCP0_ENABLE dma/length/format word, then set
    /// RTS/txop protection via `cr_hal_he_set_tx_protection`.
    ///
    /// Decomp (verified 2026-09-11 against blob disasm + esp-wifi-hal ll.rs set_plcp0):
    ///   eb = *param_1; u1 = read(eb+4); base = (u1 & 0xfffff) | 0x600000;
    ///   flags = read(read(eb+0x34));  // txinfo word0
    ///   v = base;
    ///   if (flags & 0x402)==0 && (flags & 0x40480000)!=0x400000 {
    ///     if (flags & 0x100000)==0 {
    ///       v = base | ((((flags>>0x13)&1)+1) << 24);          // ack-type in bits 24..26
    ///       if flags bit31 { if flags bit30 clear { v=(u1&0xfffff)|0x2600000 }
    ///                        else { iv = if (u16@(eb+0x24) & 0x1000)==0 {1} else {5}; v=base|(iv<<24) }
    ///     } else { v = (u1&0xfffff)|0x3600000 }
    ///   }
    ///   write PLCP0_ENABLE = v;    // full write; arm bits30/31 land here as 0
    ///   hal_he_set_tx_protection(slot, (flags>>8)&1, _, (txinfo[0x30]>>3)&0x3ff, txinfo[0x34])
    /// For the legacy 1 Mbit DSSS beacon: flags has none of {bit1,bit10,bit19,bit20,bit30,bit31}
    /// set, so v = base | 0x1000000 = dma | 0x1600000 (ack_type=1), and threshold = 0.
    ///
    /// NOTE ON INSTRUMENTATION: an earlier bring-up added per-call volatile-MMIO readbacks + atomic
    /// counters here; those extra bus accesses on this timing-sensitive TX-setup path stalled the TX
    /// queue (arm-without-complete), even when the register logic was byte-identical to the blob.
    /// So this body is kept minimal -- the same read/compute/write/protect shape and count as the
    /// blob.
    #[unsafe(no_mangle)]
    pub extern "C" fn mac_tx_set_plcp0(param_1: *mut u8) -> u32 {
        unsafe {
            let txq = LmacTxq::from_ctx(param_1);
            let eb = txq.cur_eb();
            let u1 = eb.dma_desc().0;
            let txinfo = eb.txinfo();
            let flags = txinfo.flags();
            let base = (u1 & 0xfffff) | 0x60_0000;
            let mut v = base;
            if (flags & 0x402) == 0 && (flags & 0x4048_0000) != 0x40_0000 {
                if (flags & 0x10_0000) == 0 {
                    v = base | ((((flags >> 0x13) & 1) + 1) << 0x18);
                    if (flags as i32) < 0 {
                        if (flags & 0x4000_0000) == 0 {
                            v = (u1 & 0xfffff) | 0x260_0000;
                        } else {
                            let iv: u32 = if (eb.flags16() & 0x1000) == 0 { 1 } else { 5 };
                            v = base | (iv << 0x18);
                        }
                    }
                } else {
                    v = (u1 & 0xfffff) | 0x360_0000;
                }
            }
            let slot = txq.slot();
            wr(mac_reg::plcp0_enable(slot), v);
            // RTS/txop protection, exactly as the blob leaf is called.
            let enable = ((flags >> 8) & 1) as i32;
            let threshold = ((txinfo.word30() >> 3) & 0x3ff) as i32;
            let tval = txinfo.word34();
            cr_hal_he_set_tx_protection(slot as i32, enable, 0, threshold, tval);
        }
        0
    }

    /// blob `hal_mac_get_txq_complete` (TX completion critical path). Called from
    /// lmacProcessTxComplete (and by our `pipeline::cr_complete`). Reads the per-slot
    /// completion/PMD/aux registers and fills two caller structs, then calls hal_mac_tx_clr_mplen.
    ///
    /// Decomp (verified 2026-09-11 against blob disasm @0x42078c76 + the caller):
    ///   C ABI: (int *param_1, int param_2, u8 *param_3, u32 *param_4) -> u32(0)
    ///     param_1 = our_instances[ac] ctx; *param_1 = eb; byte@0x28 bit2 = "skip".
    ///     param_2 = queue index q; every slot reg = base - q*0x74.
    ///     param_3 = 6-byte result struct (memset 6). param_4 = 8-byte aux/BA struct (memset 8).
    /// param_3 (the completion result the caller dispatches on) -- legacy / non-HE-TB fill:
    ///   [0] = PMD & 0xff                    (sub-status)
    ///   [1] = ((PMD>>12)&0xf)<<4 | (PMD>>8)&0xf   (high nibble = STATE/match nibble the caller
    ///         switches a jump table on: value>5 -> lmac hangs; 0 = success; low nibble = error)
    ///   [2] = (PMD>>16)&0xff  [3] = (PMD>>25)&3  [5] = (aux54ec>>16)&0xff
    /// param_4 (aux/BA info, from 54e0/54d0/54f4):
    ///   [0] = ((54e0>>16)&0xf)<<28 | (54d0 & 0xfe000) | (54d0 & 0x100000) | ((54d0>>25)<<21)
    ///   [1] = (54e0>>20)&1 | (54e0>>20)&2 | ((54f4>>5)&0x1fc)
    ///   ((54d0 bit20) is the "last_tx_is_tb"/HE-TB flag -> selects the HE-TB param_3 fill.)
    /// HE-TB fill (param_1!=0 && param_4!=0 && param_4[0] bit20 set): fills param_3 from aux54ec
    ///   and [4]=tb_sent. The blob's muedca/HE-TB wifi_log diagnostics are SKIPPED (logging only).
    /// Tail: if param_1!=0: if (byte@param_1+0x28 & 4) return; else if eb!=0 clr_mplen(eb,q).
    ///
    /// Minimal body (LESSON): each MMIO reg is read ONCE (the blob re-reads several) -> strictly
    /// fewer bus accesses than the blob, no atomics/instrumentation, so no extra TX-path traffic.
    #[unsafe(no_mangle)]
    pub extern "C" fn hal_mac_get_txq_complete(
        param_1: *mut i32,
        param_2: i32,
        param_3: *mut u8,
        param_4: *mut u32,
    ) -> u32 {
        let off = (param_2 as u32).wrapping_mul(mac_reg::SLOT_STRIDE);
        unsafe {
            // memset(param_3, 0, 6)
            core::ptr::write_bytes(param_3, 0, 6);
            if !param_4.is_null() {
                // memset(param_4, 0, 8)
                core::ptr::write_bytes(param_4 as *mut u8, 0, 8);
                let r54e0 = rd(mac_reg::AUX_54E0.wrapping_sub(off));
                let r54d0 = rd(mac_reg::AUX_54D0.wrapping_sub(off));
                let r54f4 = rd(mac_reg::AUX_54F4.wrapping_sub(off));
                let w0 = ((r54e0 >> 0x10) << 0x1c)
                    | (r54d0 & 0xfe000)
                    | (r54d0 & 0x10_0000)
                    | ((r54d0 >> 0x19) << 0x15);
                let w1 = ((r54e0 >> 0x14) & 2) | ((r54e0 >> 0x14) & 1) | ((r54f4 >> 5) & 0x1fc);
                *param_4 = w0;
                *param_4.add(1) = w1;
            }
            let pmd = rd(mac_reg::PMD.wrapping_sub(off));
            let aux = rd(mac_reg::AUX_54EC.wrapping_sub(off));
            *param_3.add(2) = (pmd >> 0x10) as u8;
            *param_3.add(3) = ((pmd >> 0x19) & 3) as u8;

            let mut eb: i32 = 0;
            let mut he_tb = false;
            if !param_1.is_null() {
                eb = *param_1;
                if !param_4.is_null() && (*param_4 & 0x10_0000) != 0 {
                    // HE-TB completion fill (never taken by the legacy beacon). Logging skipped.
                    he_tb = true;
                    *param_3.add(1) = (*param_3.add(1) & 0x0f) | (((aux >> 0xc) as u8) << 4);
                    *param_3.add(1) = (*param_3.add(1) & 0xf0) | ((aux >> 8) as u8 & 0xf);
                    *param_3 = aux as u8;
                    let b = (aux >> 0x18) as u8 & 0x7f;
                    *param_3.add(4) = if (aux >> 0x18) & 0x40 == 0 {
                        b
                    } else {
                        b + 0x80
                    };
                }
            }
            if !he_tb {
                let hi = ((pmd >> 0xc) as u8) << 4;
                *param_3.add(1) = hi | ((pmd >> 8) as u8 & 0xf);
                *param_3 = pmd as u8;
            }
            *param_3.add(5) = (aux >> 0x10) as u8;

            // Tail: honor the "skip" flag, else tear down the mplen bitmap for this frame.
            if !param_1.is_null() {
                if (*(param_1 as *const u8).add(LmacTxq::SKIP_FLAGS as usize) & 4) != 0 {
                    return 0;
                }
                if eb != 0 {
                    hal_mac_tx_clr_mplen(eb, param_2);
                }
            }
        }
        0
    }

    /// blob `mac_tx_set_plcp1`: program the PLCP1 word (rate/keyslot/HT-HE-mode/legacy-length/LDPC)
    /// for the slot from the txinfo. Self-contained register I/O (no blob callees).
    #[unsafe(no_mangle)]
    pub extern "C" fn mac_tx_set_plcp1(param_1: *mut u8) -> u32 {
        unsafe {
            let txq = LmacTxq::from_ctx(param_1);
            let eb = txq.cur_eb();
            let txinfo = eb.txinfo();
            let lensrc = eb.dma_desc().frame_ptr();
            let rate = txinfo.rate() as u32;
            let flags = txinfo.flags();
            let uv3 = rate.wrapping_sub(0x10) & 0xff;
            let mut v: u32 = 0;
            if uv3 <= 0x13 {
                v = if (flags as i32) < 0 {
                    0x400_0000
                } else {
                    0x200_0000
                };
            }
            let keyslot = txinfo.keyslot() as u32;
            v = (v & 0xfe01_ffff) | (keyslot << 0x11);
            let ratefield = if rate > 0x28 { uv3 & 0x1f } else { rate & 0x1f };
            v = (v & 0xfffe_0fff) | (ratefield << 0xc);
            if uv3 > 0x13 {
                v = (v & 0xffff_f000) | (rd(lensrc) & 0xfff);
            }
            if (flags & 0x4000) != 0 && (txinfo.word10() & 0x40000) != 0 && (flags as i32) >= 0 {
                v |= 0x2000_0000;
            }
            let slot = txq.slot();
            wr(mac_reg::plcp1(slot), v);
        }
        0
    }

    /// blob `hal_mac_txq_enable`: arm the slot (slot_valid|slot_enabled = 0xc000_0000).
    /// The blob additionally does muedca bookkeeping (a byte-clear via GetAccess() and, for
    /// HE-TB frames only, MU-EDCA state) — that is an HE feature irrelevant to legacy TX.
    /// For safety we replicate only the arm here and delegate HE-TB frames to the blob.
    #[unsafe(no_mangle)]
    pub extern "C" fn hal_mac_txq_enable(ac: i32) {
        unsafe {
            let addr = mac_reg::plcp0_enable(ac as u32);
            wr(addr, rd(addr) | mac_reg::SLOT_ARM);
        }
    }

    /// blob `hal_mac_txq_disable`: clear slot_valid|slot_enabled.
    #[unsafe(no_mangle)]
    pub extern "C" fn hal_mac_txq_disable(ac: i32) {
        unsafe {
            let addr = mac_reg::plcp0_enable(ac as u32);
            wr(addr, rd(addr) & mac_reg::SLOT_ARM_CLEAR);
        }
    }

    /// blob `hal_mac_rate_autoack_init`: empty in the blob (no-op).
    #[unsafe(no_mangle)]
    pub extern "C" fn hal_mac_rate_autoack_init() {}

    /// blob `hal_mac_get_txq_state`: read the TX-queue state nibble for an AC.
    /// (The blob's `cRam00000000`-gated esp_test/wifi_log debug branch is skipped.)
    #[unsafe(no_mangle)]
    pub extern "C" fn hal_mac_get_txq_state(ac: i32) -> u32 {
        let state = unsafe {
            match ac {
                1 => (rd(mac_reg::STATE_A) >> 0x10) & 0xff,
                2 => rd(mac_reg::STATE_B) & 0x7ff,
                0 => rd(mac_reg::STATE_A) & 0x7ff,
                _ => 0,
            }
        };
        state & 0xf
    }

    /// blob `hal_mac_clr_txq_state`: clear a queue's state bit in the SM clear registers.
    #[unsafe(no_mangle)]
    pub extern "C" fn hal_mac_clr_txq_state(kind: i32, bit: u32) -> u32 {
        unsafe {
            match kind {
                1 => wr(mac_reg::CLR_STATE_A, 1u32 << ((bit.wrapping_add(0x10)) & 0x1f)),
                2 => wr(
                    mac_reg::CLR_STATE_B,
                    rd(mac_reg::CLR_STATE_B) | (1u32 << (bit & 0x1f)),
                ),
                0 => wr(mac_reg::CLR_STATE_A, 1u32 << (bit & 0x1f)),
                _ => {}
            }
        }
        0
    }

    /// blob `hal_mac_tx_is_cbw40`: true if the slot is configured for 40 MHz (PMD bits 25-26).
    #[unsafe(no_mangle)]
    pub extern "C" fn hal_mac_tx_is_cbw40(q: i32) -> bool {
        let addr = mac_reg::pmd(q as u32);
        unsafe { (rd(addr) >> 0x19) & 3 != 0 }
    }

    /// blob `hal_mac_tx_get_blockack`: copy the slot's block-ack bitmap registers into `out`.
    /// `out` points to caller-provided storage: [+0]=u8 frag, [+2]=u16 seq, [+4]=u32, [+8]=u32.
    #[unsafe(no_mangle)]
    pub extern "C" fn hal_mac_tx_get_blockack(q: i32, out: *mut u8) -> u32 {
        let off = (q as u32).wrapping_mul(mac_reg::SLOT_STRIDE);
        unsafe {
            let hdr = rd(mac_reg::BA_BITMAP.wrapping_add(8).wrapping_sub(off)); // 0x600a54dc - off
            core::ptr::write_unaligned(out.add(2) as *mut u16, (hdr as u16) >> 4);
            *out = ((hdr >> 0x10) & 0xf) as u8;
            let w1 = rd(mac_reg::BA_BITMAP.wrapping_add(4).wrapping_sub(off)); // 0x600a54d8 - off
            core::ptr::write_unaligned(out.add(4) as *mut u32, w1);
            let w0 = rd(mac_reg::BA_BITMAP.wrapping_sub(off)); // 0x600a54d4 - off
            core::ptr::write_unaligned(out.add(8) as *mut u32, w0);
        }
        0
    }

    /// blob `hal_set_tx_min_pwr`: program the 6-bit TX minimum power field (bits 4-9).
    #[unsafe(no_mangle)]
    pub extern "C" fn hal_set_tx_min_pwr(pwr: u32) {
        unsafe {
            wr(
                mac_reg::TX_MIN_PWR,
                ((pwr & 0x3f) << 4) | (rd(mac_reg::TX_MIN_PWR) & 0xffff_fc0f),
            )
        }
    }

    /// _LANCHOR0: baked TX-power table in blob flash (signed bytes, stride 2), read by
    /// hal_get_tx_pwr / hal_mac_tx_set_ppdu.
    const TX_PWR_TABLE: u32 = 0x4208_3134;

    /// blob `hal_get_tx_pwr`: look up the signed max-TX-power byte for a rate index from the
    /// baked flash table (indices > 0x19 are folded down by 0xa).
    #[unsafe(no_mangle)]
    pub extern "C" fn hal_get_tx_pwr(idx: u32) -> i32 {
        let i = if idx > 0x19 { idx - 0xa } else { idx };
        unsafe { *((TX_PWR_TABLE.wrapping_add(i.wrapping_mul(2))) as usize as *const i8) as i32 }
    }

    #[inline(always)]
    unsafe fn pwr_byte(off: u32) -> i32 {
        unsafe { *((TX_PWR_TABLE.wrapping_add(off)) as usize as *const i8) as i32 }
    }

    /// blob `hal_mac_tx_set_ppdu`: program the TX slot for the armed frame. Sets PLCP0/PLCP1
    /// (via helpers), clears conf1 bit3, then writes the rate/duration word. Legacy (DSSS) frames
    /// take the `mac_tx_set_len`/`mac_tx_set_txop_q` path; OFDM/HT/HE rates take the SIG path with
    /// its HT/HE-only SIG helpers left to the blob (not exercised by legacy beacons).
    /// `param_1` is the our_instances[ac] context; `*param_1` = eb, eb+0x34 = txinfo.
    #[unsafe(no_mangle)]
    pub extern "C" fn hal_mac_tx_set_ppdu(param_1: *mut u8, param_2: i32) -> u32 {
        unsafe {
            let txq = LmacTxq::from_ctx(param_1);
            let eb = txq.cur_eb();
            // (blob's `*(*(eb+4)+4) & 3` wifi_log debug branch omitted.)
            mac_tx_set_plcp0(param_1);
            mac_tx_set_plcp1(param_1);
            let slot = txq.slot();
            let conf1 = mac_reg::conf1_real(slot);
            wr(conf1, rd(conf1) & 0xffff_fff7); // clear bit3
            let txinfo = eb.txinfo();
            let rate = txinfo.rate();
            let rtsidx = cr_mac_tx_get_rts_rate(rate as i32) as u32;
            let s2 = (pwr_byte(rtsidx.wrapping_mul(2)) << 16)
                | (pwr_byte(rtsidx.wrapping_mul(2).wrapping_add(1)) << 24);
            let s3 = rate.wrapping_sub(0x10) as u32; // (rate-0x10)&0xff
            let plcp_rate_dur = mac_reg::plcp_rate_dur(slot);
            if s3 <= 0x13 {
                // OFDM/HT/HE rate. HT/HE SIG programming delegated to the blob leaves.
                let flags0 = txinfo.flags();
                if (flags0 as i32) < 0 {
                    shims::mac_tx_set_hesig();
                } else {
                    shims::mac_tx_set_htsig(param_1, param_2);
                }
                let r2 = eb.txinfo().rate() as u32;
                let idx = if r2 > 0x19 { r2 - 0xa } else { r2 };
                let c1 = pwr_byte(idx.wrapping_mul(2));
                let c2 = pwr_byte(idx.wrapping_mul(2).wrapping_add(1));
                wr(plcp_rate_dur, ((c2 << 8) | (c1 | s2)) as u32);
            } else {
                // DSSS / legacy path (our beacon).
                cr_mac_tx_set_len(param_1, param_2);
                mac_tx_set_txop_q(param_1);
                let r = txinfo.rate() as u32;
                wr(plcp_rate_dur, (pwr_byte(r.wrapping_mul(2)) | s2) as u32);
            }
            cr_mac_tx_set_pti(param_1);
        }
        0
    }

    /// Rust `mac_tx_set_pti` -> `hal_set_tx_pti` (0x4080ff56 / 0x4080febe). Programs the BT-coex
    /// packet-traffic-indication fields from the frame's pti (txinfo+0x20) and txinfo+0x22.
    /// Faithful to the 0.3.0 disasm: clears the CONF1 top nibble and packs pti into the PTI
    /// register bits 4-19 with txinfo+0x22 in bits 20-31. The blob additionally runs a coex
    /// callback that clamps pti = min(pti, coex_demand); with no BT coex active the frame's own
    /// pti governs (our disconnected beacon has pti=txinfo+0x22=0, so this reduces to clearing
    /// those fields -- proven irrelevant to emission: skipping it radiates identically).
    unsafe fn cr_mac_tx_set_pti(ctx: *mut u8) {
        unsafe {
            let txq = LmacTxq::from_ctx(ctx);
            let txinfo = txq.cur_eb().txinfo();
            let pti = txinfo.pti() as u32;
            let hw22 = txinfo.pti_hi() as u32;
            let slot = txq.slot();
            let a1 = pti; // coex clamp skipped (no BT); min(pti, coex_demand) == pti here
            let conf1 = mac_reg::conf1(slot);
            wr(conf1, (rd(conf1) & 0x0fff_ffff) | (a1 << 0x1c));
            let ptir = mac_reg::pti(slot);
            wr(ptir, (rd(ptir) & 0xffff_0fff) | ((pti << 0xc) & 0xf000));
            wr(ptir, (rd(ptir) & 0xffff_f0ff) | ((pti << 8) & 0xf00));
            wr(ptir, (rd(ptir) & 0xffff_ff0f) | ((pti << 4) & 0xf0));
            wr(ptir, (rd(ptir) & 0xfff0_ffff) | ((pti << 0x10) & 0xf_0000));
            wr(ptir, (rd(ptir) & 0x000f_ffff) | (hw22 << 0x14));
        }
    }

    /// blob `hal_mac_tx_clr_mplen`: the blob reads CONF1 (0x600a4d64 - q*0x10) and, only if bit3
    /// is set (an HE-TB aggregate), tears down the HE mplen bitmap. For a legacy frame (bit3
    /// clear) it is a no-op. Our DSSS beacon never builds an HE PPDU, so bit3 is never set and
    /// this is always the no-op path -- fully Rust. The HE-TB teardown is intentionally not
    /// implemented (unexercised). Verified against the 0.3.0 disasm.
    #[unsafe(no_mangle)]
    pub extern "C" fn hal_mac_tx_clr_mplen(_param1: i32, _q: i32) {}

    /// blob `hal_mac_tx_config_edca`: program the slot's CONF1 EDCA parameters (AIFSN, CW,
    /// iface bit) from the our_instances[ac] context.
    #[unsafe(no_mangle)]
    pub extern "C" fn hal_mac_tx_config_edca(txq: *mut u8) -> u32 {
        unsafe {
            let txq = LmacTxq::from_ctx(txq);
            let ac = txq.slot();
            let conf1 = mac_reg::conf1(ac);
            let aifsn = (txq.aifsn() & 0xf) as u32;
            wr(conf1, (rd(conf1) & 0xf0ff_ffff) | (aifsn << 24));
            let cw = (txq.backoff() & 0x3ff) as u32;
            wr(conf1, (rd(conf1) & 0xffc0_0fff) | (cw << 12));
            // iface bit: txq[0]=eb ptr -> eb+0x34 = txinfo -> txinfo+0x10 word, bit 19.
            let ifb = txq.cur_eb().txinfo().iface();
            wr(conf1, (rd(conf1) & 0xff3f_ffff) | (ifb << 0x16));
        }
        0
    }

    /// blob `hal_mac_tx_config_timeout`: program the slot's CONF1 low-12 lifetime/timeout field
    /// from the txinfo (+0x40/+0x44), clamped to 0xfff and floored by `param2`. Fully Rust: the
    /// blob's only other path is the `esp_test` per-AC EDCA-disable bypass (test_disable_edca_tx),
    /// never enabled in normal operation -- the blob itself takes this register-write path.
    /// Verified against the 0.3.0 disasm.
    #[unsafe(no_mangle)]
    pub extern "C" fn hal_mac_tx_config_timeout(txq: *mut u8, param2: i32) -> u32 {
        unsafe {
            let txq = LmacTxq::from_ctx(txq);
            let ac = txq.slot();
            let conf1 = mac_reg::conf1(ac);
            let txinfo = txq.cur_eb().txinfo();
            let u1 = txinfo.lifetime_hi();
            let mut v3 = (txinfo.lifetime_lo() >> 10) | (u1 << 0x16);
            if (u1 >> 10) != 0 || v3 > 0xfff {
                v3 = 0xfff;
            }
            let result: u32 = if param2 < 0 {
                param2 as u32
            } else if v3 >= param2 as u32 {
                v3
            } else {
                param2 as u32
            };
            wr(conf1, (rd(conf1) & 0xffff_f000) | (result & 0xfff));
        }
        0
    }

    /// blob `hal_mac_get_txq_pmd`: read the slot PMD/completion word into `*out` (bit24 cleared).
    /// Caller uses `*out >> 28` as the TXOP subframe-complete count.
    #[unsafe(no_mangle)]
    pub extern "C" fn hal_mac_get_txq_pmd(q: i32, out: *mut u32) -> u32 {
        let addr = mac_reg::pmd(q as u32);
        unsafe { *out = rd(addr) & 0xfeff_ffff }
        0
    }

    /// blob `hal_attenna_init`: reset per-slot antenna/PHY select fields across the RESP_DUR
    /// slot block (0x600a54bc down to 0x600a511c, stride 0x74) plus the global reg 0x600a42cc.
    #[unsafe(no_mangle)]
    pub extern "C" fn hal_attenna_init() {
        unsafe {
            // Pass 1: clear low 3 bits of each slot's RESP_DUR reg.
            let mut a = mac_reg::RESP_DUR;
            loop {
                wr(a, rd(a) & 0xffff_fff8);
                a = a.wrapping_sub(mac_reg::SLOT_STRIDE);
                if a == mac_reg::RESP_DUR_BLOCK_END {
                    break;
                }
            }
            // Pass 2: clear bit3, set bit5, clear bit4 of each slot's RESP_DUR reg.
            let mut a = mac_reg::RESP_DUR;
            loop {
                wr(a, rd(a) & 0xffff_fff7);
                wr(a, rd(a) | 0x20);
                wr(a, rd(a) & 0xffff_ffef);
                a = a.wrapping_sub(mac_reg::SLOT_STRIDE);
                if a == mac_reg::RESP_DUR_BLOCK_END {
                    break;
                }
            }
            wr(
                mac_reg::ATTENNA_GLOBAL,
                (rd(mac_reg::ATTENNA_GLOBAL) & 0xffff_fff8) | 0x20,
            );
        }
    }
}

// ============================================================================
// config — build-time selectors
// ============================================================================
/// Compile-time selectors (env at build time; unset == the production reference build). They
/// exist so the characterization runs documented in docs/cleanroom_tx.md are reproducible from
/// source. None of them is read on the hot TX path except as a `const` branch.
mod config {
    const fn env_digit(v: Option<&'static str>) -> u8 {
        match v {
            Some(s) => {
                let b = s.as_bytes();
                let mut i = 0;
                let mut n: u8 = 0;
                while i < b.len() {
                    n = n.wrapping_mul(10).wrapping_add(b[i].wrapping_sub(b'0'));
                    i += 1;
                }
                n
            }
            None => 0,
        }
    }

    /// The C6 MAC only accepts register writes to a TX slot whose bank the scheduler has
    /// activated; only slot 0 (the raw beacon's AC 0) is live. So we drive slot 0 ourselves.
    pub const MY_AC: i32 = 0;

    /// Opt-in diagnostics: the boot slot-writability probe and the periodic [CR.H] health / [CR.F]
    /// progress prints (out-of-band, never on the hot TX path).
    pub const DIAG: bool = true;

    /// The 10 blob-scheduled CR-CTRL beacons emitted ONCE at boot (before our loop, so they do not
    /// contend) as an RF baseline.
    ///
    /// SLOT-0 CONTENTION (root cause of the weak/erratic TX): `faithful_tx` drives AC0/slot0
    /// directly on the assumption that the blob pp/ppTask is idle. A `send_raw_frame` (the CR-CTRL
    /// beacon) KICKS the blob ppTask, which then also schedules onto slot 0 and races our direct arm
    /// + completion on the same slot and shared TxRxCxt pending list -- collapsing BOTH CR-RUST and
    /// CR-CTRL to near-zero (measured: our pipeline alone 131 frames/39s @ -70 dBm; the blob beacon
    /// alone 96-146 frames @ -68 dBm; the two interleaved every round -> 1-3 frames each). It also
    /// arms the blob's disconnected-sleep timer, thrashing the modem sleep/wake per round. So the
    /// per-round CR-CTRL burst is NOT interleaved by default (`CONTROL_BURST`).
    pub const CONTROL_BEACON: bool = true;
    /// CR_BURST=1: re-enable the per-round CR-CTRL burst (contends with our slot-0 pipeline, see
    /// above) -- only for the deliberately-contended wake A/B. CR_TX_MODE=1 (control-only) is the
    /// non-contended way to run a continuous blob reference.
    pub const CONTROL_BURST: bool =
        env_digit(option_env!("CR_BURST")) == 1 || env_digit(option_env!("CR_AB")) != 0;
    /// CR_PM_EXP=<n>: bitmask applied inside the Rust wake: 1 = skip _wifi_clock_enable,
    /// 2 = skip _phy_enable, 4 = skip the hal_mac_init store (the PM_TXBLOCK_RETENTION clear),
    /// 8 = call the ROM wifi_rf_phy_enable, 16 = call the blob pm_disconnected_wake.
    pub const PM_WAKE_EXPERIMENT: u8 = env_digit(option_env!("CR_PM_EXP"));
    /// CR_AB=1: same-RF A/B -- alternate the wake per round (even rounds Rust, odd rounds blob), so
    /// the two variants can be compared by round parity (seq/4) inside ONE capture.
    pub const AB_WAKE: bool = env_digit(option_env!("CR_AB")) != 0;
    /// CR_TX_MODE=1: control-only -- skip the Rust faithful_tx pipeline entirely and emit only the
    /// blob CR-CTRL beacon (send_raw_frame), so the firmware behaves like the pure-blob example.
    pub const TX_MODE_CONTROL_ONLY: bool = env_digit(option_env!("CR_TX_MODE")) == 1;
    /// Crown-jewel test: when false, skip the PM-wake entirely (no-op) to see if the MAC stays
    /// active without it (proven: it does not -- the block sticks).
    pub const PM_ON_DATA_TX: bool = true;
    /// When true (default), the modem-PM wake is the Rust `cr_pm_on_data_tx`; CR_BLOB_WAKE=1
    /// selects the blob `pm_on_data_tx` (kept only as the compile-time-disabled fallback).
    pub const PM_WAKE_RUST: bool = env_digit(option_env!("CR_BLOB_WAKE")) == 0;
    /// When true, use the Rust ppProcTxSecFrame (open/no-key beacon path); else the blob.
    pub const CR_SECFRAME: bool = true;
    /// When true, use our own eb pool; else the blob esf_buf pool.
    pub const CR_POOL_ENABLED: bool = true;
}

// ============================================================================
// eb_pool — our own eb packet-buffer pool
// ============================================================================
/// A fixed pool of correctly-laid-out eb structures our loop owns end-to-end -- NO blob esf_buf
/// in the hot path. Layout reversed from a live blob type-1 eb (esf_buf_alloc): a single
/// contiguous block, all internal pointers self-relative.
///   +0x04, +0x08 -> dma_desc (both point to the same 3-word desc at +0x3c)
///   +0x0c = 1                 +0x10 -> aux (+0x90)
///   +0x16 = u16 payload len   +0x1a = u8 pool type (1)
///   +0x30 = pending next-link (used by the pipeline's enqueue -- NOT our free-list)
///   +0x34 -> txinfo (+0x48, 0x48 bytes)     +0x3c dma_desc [size|ctrl, frame_ptr, next]
///   +0x48 txinfo (txinfo[0]=0x2000)         +0x90 aux (0x28 bytes)   +0xb8 frame bytes
/// The pool lives in .bss (HP SRAM, DMA-reachable like the blob ebs at 0x4087xxxx). Our free-list
/// is a separate Rust index stack (eb+0x30 is reserved for the blob pending list). The blob's
/// callback-mediated two-list esf_buf pool is kept only as a compile-time-disabled fallback
/// (`config::CR_POOL_ENABLED = false`): a bare-freelist recycle cannot feed it (it drained after
/// ~31 buffers).
mod eb_pool {
    use super::{wr, wr8, DmaDesc, Eb, TxInfo};

    unsafe extern "C" {
        fn esf_buf_alloc(payload: *const u8, pool_type: i32, len: u32) -> u32;
        fn esf_buf_recycle(eb: u32);
    }

    const POOL_N: usize = 12;
    // Offsets of the sub-objects inside one of OUR eb blocks (the blob's are laid out the same).
    const EB_ONE: u32 = 0x0c; // always 1 in a live eb
    const EB_AUX_PTR: u32 = 0x10;
    const EB_POOL_TYPE: u32 = 0x1a; // u8; 1 = static TX pool
    const EB_OWN_DESC_OFF: u32 = 0x3c;
    const EB_OWN_TXINFO_OFF: u32 = 0x48;
    const EB_AUX_OFF: u32 = 0x90;
    const EB_FRAME_OFF: usize = 0xb8;
    const EB_FRAME_MAX: usize = 0x200;
    const EB_DESC_SIZE_SEED: u32 = 0x68; // dma_desc[0] buffer-size seed, as esf_buf_alloc sets it
    const TXINFO_FLAGS_SEED: u32 = 0x2000; // txinfo[0] as esf_buf_alloc hands it out
    const EB_SIZE: usize = EB_FRAME_OFF + EB_FRAME_MAX; // header/desc/txinfo/aux + 512B frame
    #[repr(C, align(16))]
    struct EbBuf([u8; EB_SIZE]);
    static mut CR_POOL: [EbBuf; POOL_N] = [const { EbBuf([0u8; EB_SIZE]) }; POOL_N];
    static mut CR_FREE: [u16; POOL_N] = [0; POOL_N];
    static mut CR_FREE_LEN: usize = 0;

    #[inline(always)]
    fn eb_base(i: usize) -> u32 {
        unsafe { core::ptr::addr_of!(CR_POOL[i]) as u32 }
    }

    /// Copy the beacon payload into an eb's frame region and set its payload length.
    #[inline(always)]
    unsafe fn eb_load_payload(b: u32, payload: &[u8]) {
        unsafe {
            let n = payload.len().min(EB_FRAME_MAX);
            core::ptr::copy_nonoverlapping(
                payload.as_ptr(),
                (b + EB_FRAME_OFF as u32) as *mut u8,
                n,
            );
            Eb(b).set_payload_len(n as u16);
        }
    }

    /// Build the pool: initialise every eb exactly as esf_buf_alloc hands it out (self-relative
    /// pointers, type, dma descriptor, txinfo seed, payload copied into the frame region), and fill
    /// the free-list. Called once at boot.
    pub fn init(payload: &[u8]) {
        unsafe {
            for i in 0..POOL_N {
                let b = eb_base(i);
                // zero the whole block first
                core::ptr::write_bytes(b as *mut u8, 0, EB_SIZE);
                wr(b + Eb::DMA_DESC, b + EB_OWN_DESC_OFF); // dma_desc ptr
                wr(b + Eb::SEC_DESC, b + EB_OWN_DESC_OFF); // dma_desc ptr (same desc)
                wr(b + EB_ONE, 1);
                wr(b + EB_AUX_PTR, b + EB_AUX_OFF); // aux ptr
                wr8(b + EB_POOL_TYPE, 1); // pool type 1
                wr(b + Eb::TXINFO, b + EB_OWN_TXINFO_OFF); // txinfo ptr
                // dma descriptor at +0x3c: [0]=size/ctrl (low 14b = buffer size), [1]=frame ptr,
                // [2]=next. faithful_tx re-derives the owner/eof/length bits each frame.
                let desc = DmaDesc(b + EB_OWN_DESC_OFF);
                desc.set_ctrl(EB_DESC_SIZE_SEED);
                desc.set_frame_ptr(b + EB_FRAME_OFF as u32);
                desc.set_next(0);
                // txinfo seed (faithful_tx sets the rest per frame)
                TxInfo(b + EB_OWN_TXINFO_OFF).set_flags(TXINFO_FLAGS_SEED);
                // payload -> frame, and the payload length at +0x16
                eb_load_payload(b, payload);
                CR_FREE[i] = i as u16;
            }
            CR_FREE_LEN = POOL_N;
        }
    }

    /// Pop a free eb from our pool (Eb(0) if exhausted). Re-copies the payload + resets the length so
    /// a re-used eb is handed out clean, matching esf_buf_alloc semantics.
    pub fn alloc(payload: &[u8]) -> Eb {
        unsafe {
            if CR_FREE_LEN == 0 {
                return Eb(0);
            }
            CR_FREE_LEN -= 1;
            let i = CR_FREE[CR_FREE_LEN] as usize;
            let b = eb_base(i);
            let eb = Eb(b);
            eb_load_payload(b, payload);
            eb.set_prefix_len(0);
            eb.set_flags16(0); // clear the sec-hdr-shifted flag ppProcTxSecFrame sets
            eb.set_pending_next(0);
            // restore dma descriptor (ppProcTxSecFrame shifted frame ptr / lengths last time)
            let desc = DmaDesc(b + EB_OWN_DESC_OFF);
            desc.set_ctrl(EB_DESC_SIZE_SEED);
            desc.set_frame_ptr(b + EB_FRAME_OFF as u32);
            eb
        }
    }

    /// Return an eb to our pool. `eb` must be one of ours; maps back to its index.
    pub fn recycle(eb: Eb) {
        unsafe {
            let base0 = eb_base(0);
            if eb.0 < base0 {
                return;
            }
            let idx = ((eb.0 - base0) as usize) / EB_SIZE;
            if idx >= POOL_N || eb_base(idx) != eb.0 {
                return; // not one of ours
            }
            if CR_FREE_LEN < POOL_N {
                CR_FREE[CR_FREE_LEN] = idx as u16;
                CR_FREE_LEN += 1;
            }
        }
    }

    /// Free-list depth (diagnostic).
    pub fn free_len() -> usize {
        unsafe { CR_FREE_LEN }
    }

    /// Blob-pool fallback: allocate ONE eb from the live static-TX esf_buf pool with our beacon
    /// payload copied in.
    pub fn blob_alloc(payload: &[u8]) -> Eb {
        Eb(unsafe { esf_buf_alloc(payload.as_ptr(), 1, payload.len() as u32) })
    }

    /// Blob-pool fallback: recycle an eb to the esf_buf pool (also used by the pipeline's drop paths).
    pub fn blob_recycle(eb: Eb) {
        unsafe { esf_buf_recycle(eb.0) }
    }
}

// ============================================================================
// pm_wake — the modem-PM wake FSM (pm_on_data_tx), in Rust
// ============================================================================
/// What the blob does per frame (0.3.0 disasm, pm_on_data_tx @0x4080ef66 -> pm_tx_data_process
/// @0x4080ece8), reduced to the disconnected-STA path our raw beacon always takes:
///   - TWT / mesh guards (never active here), `g_pm+2 == iface` (the PM's own interface),
///   - pm_check_state (no-op while g_pm+1 == 0, which the disconnected FSM keeps it at),
///   - g_pm+0xe == 0 (not connected/PS-enabled)  =>  pm_is_in_wifi_slice_threshold(now, 5000)
///     (== 1 with no coex, since _coex_status_get() == 0), an optional _coex_wifi_request if `now`
///     is before the coex slice start (g_pm+0x70/0x74, never armed without coex),
///   - pm_disconnected_wake @0x42020838:  if g_pm+0x121 == 3 (modem asleep) && !mesh:
///         wifi_rf_phy_enable(0); g_pm+0x121 = 2; pm_set_state(0) (g_pm+1 = 0).
/// wifi_rf_phy_enable is the ROM function @0x40016a68 (a jump-table stub at 0x40000ba8), and it is
/// just a dispatcher:  mask = g_ic->rf_phy_enabled_mask (byte @ *(g_ic_ptr 0x4087ffa0) + 0x24e);
///   if mask == 0: osi->_wifi_pm_sleep_lock_acquire (+0xc8), osi->_wifi_clock_enable (+0xf8),
///                 [if *(*g_mac_sleep_en_ptr 0x4087ff00): pp_wdev_funcs[0x94] = pm_mac_wakeup],
///                 if osi->_env_is_chip (+4): osi->_phy_enable (+0xd4),
///                 pp_wdev_funcs[0x95] = ic_mac_init -> hal_mac_init @0x4080bc6a:
///                     WDEV_PM_TXBLOCK_RETENTION &= ~((pm_get_tx_blocks_retention_mask() &
///                     0xff0000) | 0x1000)     <-- THE CPU STORE THAT CLEARS THE INTERLOCK
///   mask |= 1 << mode.
/// So the "hardware consequence" model was wrong: the 0x00ff1000 -> 0 transition IS a plain
/// register write (hal_mac_init), it just has to come AFTER the Wi-Fi MAC clocks are re-enabled
/// (_wifi_clock_enable: MODEM_SYSCON/MODEM_LPCON) -- a write to the clock-gated MAC is dropped,
/// which is what made a bare strobe of the register look like a hardware interlock -- and the
/// PHY must be back up (_phy_enable) for the launched frame to actually radiate. Every OSI slot
/// the wake dispatches through is esp-radio Rust (`wifi_pm_sleep_lock_acquire` no-op,
/// `wifi_clock_enable` -> radio_clocks::enable_wifi, `env_is_chip` -> true, `phy_enable` ->
/// esp_phy::enable_phy_with_wifi_rx), reached exactly as the ROM reaches them (through the
/// g_osi_funcs_p table, which keeps esp-radio's clock/PHY refcounts balanced against the blob's
/// later wifi_rf_phy_disable from the disconnected-sleep timer). The sleep side (the blob's
/// pm_on_data_tx_done -> 1 ms disconnected-sleep-delay timer -> pm_disconnected_sleep ->
/// wifi_rf_phy_disable -> hal_mac_deinit |= 0xff1000, g_pm+0x121 = 3) is background PM state and
/// stays blob; it is armed only by the blob's own TX (the CR-CTRL beacon).
///
/// The PM globals are linked, NOT hardcoded: `g_pm` (.bss) and `g_mesh_is_started` (.data) are
/// exported blob symbols and their addresses move with the link layout -- adding this very code
/// shifted g_pm from 0x40820720 to 0x40820740 (and an earlier layout had it at 0x4081ea98), which
/// a hardcoded address would have silently mis-read. The FIELD OFFSETS below are 0.3.0-specific
/// (re-derive per blob version; see the WARNING at the top). The 0x4087ffxx cells are ROM
/// interface cells (fixed by the C6 ROM).
mod pm_wake {
    use super::{config, mac_reg, rd, rd8, wr, wr8};
    use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

    unsafe extern "C" {
        static mut g_pm: u8;
        static mut g_mesh_is_started: u8;
        /// The blob pm_on_data_tx, kept only as the compile-time-disabled fallback
        /// (`config::PM_WAKE_RUST = false`) and the A/B control.
        pub fn pm_on_data_tx(iface: u32, p2: i32) -> i32;
        /// esp-radio's own (Rust) esp_timer_get_time, the target of the OSI `_esp_timer_get_time`
        /// slot the blob PM code reads through g_osi_funcs_p+0x108.
        fn __esp_radio_esp_timer_get_time() -> i64;
    }
    #[inline(always)]
    fn g_pm_base() -> u32 {
        core::ptr::addr_of_mut!(g_pm) as u32
    }
    #[inline(always)]
    fn g_mesh_is_started_addr() -> u32 {
        core::ptr::addr_of_mut!(g_mesh_is_started) as u32
    }
    const G_OSI_FUNCS_P: u32 = 0x4087_ff6c; // ROM cell -> esp-radio's wifi_osi_funcs_t
    const G_IC_PTR: u32 = 0x4087_ffa0; // ROM cell -> g_ic; rf_phy_enabled_mask byte @ +0x24e
    const G_MAC_SLEEP_EN_PTR: u32 = 0x4087_ff00; // ROM cell -> &g_mac_sleep_en (0x4081f0dc)
    const WDEV_PM_TXBLOCK_RETENTION: u32 = mac_reg::PM_TXBLOCK_RETENTION;
    const OSI_ENV_IS_CHIP: u32 = 0x004;
    const OSI_WIFI_PM_SLEEP_LOCK_ACQUIRE: u32 = 0x0c8;
    const OSI_PHY_ENABLE: u32 = 0x0d4;
    const OSI_WIFI_CLOCK_ENABLE: u32 = 0x0f8;
    const OSI_COEX_STATUS_GET: u32 = 0x190;
    const OSI_COEX_WIFI_REQUEST: u32 = 0x198;

    /// Frames on which the Rust wake found the modem asleep (g_pm+0x121 == 3) and ran the full
    /// clock/PHY/MAC re-enable.
    pub static PM_WAKES: AtomicU32 = AtomicU32::new(0);
    /// Frames on which the block bits (0x00ff1000) were set before / after the wake.
    pub static PM_PRE_BLOCKED: AtomicU32 = AtomicU32::new(0);
    pub static PM_POST_BLOCKED: AtomicU32 = AtomicU32::new(0);
    /// PM states the reduced Rust FSM does not model (connected-PS states, TWT, mesh, coex active,
    /// g_mac_sleep_en). Must stay 0; if it ever ticks the reduction is not valid for that run.
    pub static PM_ODD: AtomicU32 = AtomicU32::new(0);
    /// Packed (pre>>8)<<16 | (post>>8) of the block register around the last wake ([CR.H] pm_blk).
    pub static PM_BLK: AtomicU32 = AtomicU32::new(0);
    /// Experiment-only readback of the block register right after the hal_mac_init store (and, for
    /// the clock-gated variant, whether that store stuck). Not read on the production build.
    pub static PM_BLK_AFTER_STORE: AtomicU32 = AtomicU32::new(0xffff_ffff);

    #[inline(always)]
    unsafe fn pm_u8(off: u32) -> u8 {
        unsafe { rd8(g_pm_base() + off) }
    }
    #[inline(always)]
    unsafe fn pm_set_u8(off: u32, v: u8) {
        unsafe { wr8(g_pm_base() + off, v) }
    }
    #[inline(always)]
    unsafe fn pm_u32(off: u32) -> u32 {
        unsafe { rd(g_pm_base() + off) }
    }
    #[inline(always)]
    unsafe fn mesh_started() -> bool {
        unsafe { rd8(g_mesh_is_started_addr()) != 0 }
    }
    #[inline(always)]
    unsafe fn osi_slot(off: u32) -> u32 {
        unsafe { rd(rd(G_OSI_FUNCS_P) + off) }
    }
    #[inline(always)]
    fn odd() {
        PM_ODD.fetch_add(1, Relaxed);
    }

    /// Rust pm_is_twt_start (0.3.0 @0x4080c2ba): g_pm+0x1c2 || g_pm+0x2e4.
    #[inline(always)]
    unsafe fn cr_pm_is_twt_start() -> bool {
        unsafe { pm_u8(0x1c2) != 0 || pm_u8(0x2e4) != 0 }
    }

    /// Rust pm_get_tx_blocks_retention_mask (0.3.0 @0x42022cbe).
    unsafe fn cr_pm_get_tx_blocks_retention_mask() -> u32 {
        unsafe {
            if pm_u8(0xe) != 0 && (pm_u8(0xf) == 0 || pm_u8(0x46a) != 0) {
                0xfff1_ffff
            } else {
                0xffff_ffff
            }
        }
    }

    /// Rust hal_mac_init (0.3.0 @0x4080bc6a, via ic_mac_init @0x4080a6c8): the store that clears
    /// the WDEV_PM_TXBLOCK_RETENTION block bits. Disconnected: mask == 0xffffffff -> clears 0xff1000.
    pub unsafe fn cr_hal_mac_init() {
        unsafe {
            let m = cr_pm_get_tx_blocks_retention_mask();
            let v = rd(WDEV_PM_TXBLOCK_RETENTION);
            wr(WDEV_PM_TXBLOCK_RETENTION, v & !((m & 0x00ff_0000) | 0x1000));
        }
    }

    /// Rust wifi_rf_phy_enable(mode) -- the ROM dispatcher @0x40016a68, 1:1.
    pub unsafe fn cr_wifi_rf_phy_enable(mode: u32) {
        unsafe {
            let maskp = rd(G_IC_PTR) + 0x24e;
            if rd8(maskp) == 0 {
                let lock: extern "C" fn() =
                    core::mem::transmute(osi_slot(OSI_WIFI_PM_SLEEP_LOCK_ACQUIRE));
                lock(); // esp-radio: no-op
                if config::PM_WAKE_EXPERIMENT & 1 == 0 {
                    let clk_en: extern "C" fn() =
                        core::mem::transmute(osi_slot(OSI_WIFI_CLOCK_ENABLE));
                    clk_en(); // esp-radio radio_clocks::enable_wifi(true): MODEM_SYSCON/LPCON gates
                }
                if rd8(rd(G_MAC_SLEEP_EN_PTR)) != 0 {
                    // g_mac_sleep_en (modem-sleep MAC retention) is never set in this build
                    // (set_power_saving(None)); pm_mac_wakeup is not modelled.
                    odd();
                }
                let is_chip: extern "C" fn() -> u32 = core::mem::transmute(osi_slot(OSI_ENV_IS_CHIP));
                if is_chip() != 0 && config::PM_WAKE_EXPERIMENT & 2 == 0 {
                    let phy_en: extern "C" fn() = core::mem::transmute(osi_slot(OSI_PHY_ENABLE));
                    phy_en(); // esp-radio: esp_phy::enable_phy_with_wifi_rx()
                }
                if config::PM_WAKE_EXPERIMENT & 4 == 0 {
                    cr_hal_mac_init(); // ic_mac_init -> hal_mac_init: unblock the MAC
                }
                if config::PM_WAKE_EXPERIMENT != 0 {
                    PM_BLK_AFTER_STORE.store(rd(WDEV_PM_TXBLOCK_RETENTION), Relaxed);
                }
            }
            wr8(maskp, rd8(maskp) | (1u8 << mode));
        }
    }

    /// Rust pm_set_state (0.3.0 @0x4202040a): g_pm+1 = s. (wifi_gpio_debug is a null-checked debug
    /// hook, *(0x40811e08) == 0 -> no-op.)
    #[inline(always)]
    unsafe fn cr_pm_set_state(s: u8) {
        unsafe { pm_set_u8(1, s) }
    }

    /// Rust pm_disconnected_wake (0.3.0 @0x42020838).
    pub unsafe fn cr_pm_disconnected_wake() {
        unsafe {
            if pm_u8(0x121) == 3 && !mesh_started() {
                PM_WAKES.fetch_add(1, Relaxed);
                if config::PM_WAKE_EXPERIMENT & 8 != 0 {
                    // bisect: the real ROM wifi_rf_phy_enable instead of the Rust port
                    unsafe extern "C" {
                        fn wifi_rf_phy_enable(mode: u32);
                    }
                    wifi_rf_phy_enable(0);
                } else {
                    cr_wifi_rf_phy_enable(0);
                }
                pm_set_u8(0x121, 2);
                cr_pm_set_state(0);
            }
        }
    }

    /// Rust pm_check_state (0.3.0 @0x4080971e), disconnected reduction. The blob calls pm_dream +
    /// pm_set_state(0) if the PS state (g_pm+1) is non-zero; the disconnected FSM never leaves 0
    /// (only pm_sleep/pm_dream move it, and they need g_pm+0xe), so that is an invariant we count.
    #[inline(always)]
    unsafe fn cr_pm_check_state() {
        unsafe {
            if pm_u8(1) != 0 {
                odd();
            }
        }
    }

    /// Rust pm_on_data_tx(iface, 0) == pm_tx_data_process(iface, 0), disconnected-STA reduction.
    /// The per-frame cost when the modem is already awake is a handful of byte reads.
    pub fn cr_pm_on_data_tx(iface: u32) {
        unsafe {
            if iface == 0 && cr_pm_is_twt_start() {
                odd(); // TWT session: the blob returns a status without waking
                return;
            }
            if mesh_started() {
                odd(); // mesh PS hook path not modelled
                return;
            }
            if pm_u8(2) as u32 != iface {
                return; // not the PM's interface
            }
            cr_pm_check_state();
            if pm_u8(0xe) != 0 {
                odd(); // connected / PS-enabled FSM (pm_go_to_wake, pm_dream, ...) not modelled
                return;
            }
            if config::PM_WAKE_EXPERIMENT & 16 != 0 {
                // bisect: the blob pm_disconnected_wake instead of the Rust one
                unsafe extern "C" {
                    fn pm_disconnected_wake();
                }
                if pm_u8(0x121) == 3 {
                    PM_WAKES.fetch_add(1, Relaxed);
                }
                pm_disconnected_wake();
                return;
            }
            // pm_is_in_wifi_slice_threshold(now, 5000): 1 unless coex is active.
            let coex_status: extern "C" fn() -> u32 =
                core::mem::transmute(osi_slot(OSI_COEX_STATUS_GET));
            if coex_status() != 0 {
                odd(); // coex time-slicing not modelled (esp-radio without `coex` returns 0)
            }
            let now = __esp_radio_esp_timer_get_time() as u64;
            let slice_start = ((pm_u32(0x74) as u64) << 32) | pm_u32(0x70) as u64;
            if now < slice_start {
                let req: extern "C" fn(u32, u32, u32) -> i32 =
                    core::mem::transmute(osi_slot(OSI_COEX_WIFI_REQUEST));
                req(1, 0, pm_u32(0x70).wrapping_sub(now as u32));
            }
            cr_pm_disconnected_wake();
        }
    }

    /// The per-frame PM-wake as the pipeline calls it: run the Rust wake (or the blob fallback /
    /// A-B control), and keep the [CR.H] block-register counters.
    pub fn on_data_tx(iface: u32, use_blob: bool) {
        unsafe {
            let blk_pre = rd(WDEV_PM_TXBLOCK_RETENTION);
            if config::PM_ON_DATA_TX {
                if !use_blob {
                    cr_pm_on_data_tx(iface); // PM-wake (Rust)
                } else {
                    pm_on_data_tx(iface, 0); // PM-wake (blob fallback / A-B control)
                }
            }
            let blk_post = rd(WDEV_PM_TXBLOCK_RETENTION);
            if (blk_pre & mac_reg::PM_TXBLOCK_BITS) != 0 {
                PM_PRE_BLOCKED.fetch_add(1, Relaxed);
            }
            if (blk_post & mac_reg::PM_TXBLOCK_BITS) != 0 {
                PM_POST_BLOCKED.fetch_add(1, Relaxed);
            }
            // pack: high byte-ish of pre and post so we see the 0xff1000/0x2000 region
            PM_BLK.store(((blk_pre >> 8) << 16) | ((blk_post >> 8) & 0xffff), Relaxed);
        }
    }

    /// Boot-time snapshot of the PM state the Rust wake relies on (diagnostic).
    pub fn snapshot() {
        unsafe {
            let ic = rd(G_IC_PTR);
            esp_println::println!(
                "[CR.PM] g_pm={:#x} st={} disc={} conn={} iface={} twt={}/{} mesh={} osi={:#x} ic={ic:#x} rfmask={} mac_sleep_en={} blk={:#010x}",
                g_pm_base(),
                pm_u8(1),
                pm_u8(0x121),
                pm_u8(0xe),
                pm_u8(2),
                pm_u8(0x1c2),
                pm_u8(0x2e4),
                mesh_started() as u8,
                rd(G_OSI_FUNCS_P),
                rd8(ic + 0x24e),
                rd8(rd(G_MAC_SLEEP_EN_PTR)),
                rd(WDEV_PM_TXBLOCK_RETENTION)
            );
        }
    }
}

// ============================================================================
// pipeline — the per-frame TX pipeline (submit / map / enqueue / pop / arm / complete)
// ============================================================================
/// Runs the blob's per-frame TX sequence in Rust on the REAL blob scheduler state:
///   cr_ppTxPkt (submit + enable-gate + AC map/PM-wake + enqueue onto TxRxCxt pending)
///   -> cr_ppProcessTxQ (pop) -> cr_lmacTxFrame/cr_lmacSetTxFrame (arm via `hal_mac_tx`)
///   -> cr_complete (poll completion, read result, recycle the eb).
/// It is called DIRECTLY from our task (not by symbol interposition into the blob's
/// ppProcessTxQ->lmacTxFrame call graph, which the lmac placement wall forbids), which requires
/// the blob pp/ppTask to be idle -- see `config::CONTROL_BEACON` for what happens otherwise. The
/// scheduler structures are the typed views in `blob_layout`. Function names keep the blob's
/// (`cr_` = clean-room) so each body can be checked against its decompilation.
#[allow(non_snake_case)]
mod pipeline {
    use super::{
        config, eb_pool, hal_mac_tx, mac_reg, pm_wake, rd, rd8, rd16, wr, wr16, blob_layout,
        DmaDesc, Eb, LmacTxq, PendingQ, TxInfo,
    };
    use core::sync::atomic::{AtomicBool, AtomicU32, Ordering::Relaxed};

    unsafe extern "C" {
        /// blob ppProcTxSecFrame: kept only as the compile-time-disabled fallback
        /// (`config::CR_SECFRAME = false`).
        fn ppProcTxSecFrame(eb: u32) -> i32;
    }

    /// CR_AB round parity: when set, this round's wake goes through the blob (see config::AB_WAKE).
    pub static AB_USE_BLOB: AtomicBool = AtomicBool::new(false);
    /// Completion-status histogram (pmd>>12 nibble from hal_mac_get_txq_complete; 0 = success).
    pub static TX_STATUS: [AtomicU32; 16] = [const { AtomicU32::new(0) }; 16];

    /// Rust `hal_now`: the blob reads the free-running WDEV system timer at 0x600ad000 (a single
    /// `lw`, verified against the 0.3.0 disasm of hal_now @0x4202c6e2). We use it only as the TX
    /// submit timestamp (txinfo+0x18); a direct volatile read reproduces it exactly.
    #[inline(always)]
    pub fn cr_hal_now() -> u32 {
        unsafe { rd(mac_reg::SYS_TIMER) }
    }

    /// Recycle an eb on a drop path (before it was ever enqueued): to our pool, or the blob pool.
    #[inline(always)]
    fn recycle(eb: Eb) {
        if config::CR_POOL_ENABLED {
            eb_pool::recycle(eb);
        } else {
            eb_pool::blob_recycle(eb);
        }
    }

    /// Rust ppTxPkt (submit), kick=0. Gate on the per-VIF enable, run the security-header step,
    /// map the AC (which runs the PM-wake), then enqueue the eb onto the REAL TxRxCxt per-AC
    /// pending list (threaded via eb+0x30). Returns the blob convention: 0 = enqueued,
    /// 1 = dropped+recycled. On any drop path the eb is recycled here, so the caller must NOT also
    /// recycle it (doing both is a double-free -- the bug that the wrong ic_interface_enabled masked).
    pub fn cr_ppTxPkt(eb: Eb, _kick: i32) -> i32 {
        unsafe {
            let txinfo = eb.txinfo();
            let iface = txinfo.iface();
            if cr_ic_interface_enabled(iface) == 0 {
                recycle(eb);
                return 1;
            }
            cr_ppTxProtoProc(eb);
            let sec = if config::CR_SECFRAME {
                cr_ppProcTxSecFrame(eb)
            } else {
                ppProcTxSecFrame(eb.0)
            };
            if sec == 1 {
                recycle(eb);
                return 1;
            }
            cr_rcGetSched(eb.trc(), eb.txinfo()); // trc==0 -> no-op for raw beacon
            let map = cr_ppMapTxQueue(eb); // Rust AC mapping + PM-wake
            if map == 0 {
                let ti = eb.txinfo();
                ti.set_timestamp(cr_hal_now()); // tsf submit stamp (blob: _WDEV_TSF0_TIMER_LO)
                // Pending-list base is pTxRx (TxRxCxt) -- per-AC lists at +ac*0x34, head +0x20 /
                // tail +0x24 (threaded via eb+0x30). Verified from the linked ppTxPkt disasm
                // (`lui 0x40880; lw -0x80` = *(0x4087ff80)). NOT our_instances.
                let q = PendingQ::for_ac(ti.ac());
                eb.set_pending_next(0); // next-link = NULL
                let tail = q.tail(); // pending tail = pointer to the slot to fill
                wr(tail, eb.0); // *(tail) = eb
                q.set_tail(eb.pending_next_addr()); // tail = &eb.next
                // kick==0: the blob would pp_post(ac) here if idle; we drive the schedule
                // ourselves.
                0
            } else {
                recycle(eb); // map fail / deferred-to-hmac not expected for our beacon
                1
            }
        }
    }

    /// Rust ppTxProtoProc for the legacy beacon: reads the on-air FC byte and sets the protocol
    /// flags. For a broadcast mgmt beacon (FC 0x80, addr1[0]&1) only txinfo bit1 (0x2) is set; the
    /// data (FC&0xc==8) and null/QoS (FC&0xf0 0x50/0x40) branches are faithfully replicated but not
    /// taken by the beacon.
    pub fn cr_ppTxProtoProc(eb: Eb) {
        unsafe {
            let mut frame = eb.dma_desc().frame_ptr();
            if (eb.flags16() & Eb::FLAG16_HDR_SHIFTED) != 0 {
                frame += 8; // FTM offset (not our beacon)
            }
            let txinfo = eb.txinfo();
            if (rd8(frame + 4) & 1) != 0 {
                txinfo.set_flags(txinfo.flags() | 2); // broadcast/multicast -> no-ack
            }
            let fc = rd8(frame);
            if (fc & 0xc) == 8 {
                let f = txinfo.flags();
                txinfo.set_flags(f | 8);
                if (txinfo.word30() & 0x2_0000) == 0 && (fc & 0x70) == 0x40 {
                    txinfo.set_flags(txinfo.flags() & 0xffff_fff7);
                }
            } else if (fc & 0xc) == 0 {
                if (fc & 0xf0) == 0x50 {
                    if (txinfo.flags() & 2) == 0 {
                        txinfo.set_flags(txinfo.flags() | 0x800_0000);
                    }
                } else if (fc & 0xf0) == 0x40 && (txinfo.flags() & 2) == 0 {
                    txinfo.set_flags(txinfo.flags() | 0x800);
                }
            }
        }
    }

    /// Rust ppProcTxSecFrame, OPEN (no-key) broadcast-beacon path only -- a faithful port of the
    /// 0.3.0 blob ppProcTxSecFrame (0x40804cd0), verified byte-for-byte against the disasm. It
    /// reserves the security header:
    ///   key type = ((*(txinfo+0x10)>>8)&0xf)-1; for no-key (>8) the sec-hdr len is 4.
    ///   eb+0x16 += 4; dma_desc[0] length (bits14-27) += 4; dma_desc[0] |= 0x40000000 (eof).
    ///   Then (guarded by eb+0x24 bit13, cleared each alloc): frame ptr (dma_desc[1]) -= 8;
    ///   eb+0x14 += 8; eb+0x24 |= 0x2000; dma_desc[0] length += 8.
    ///   memset the 8 reserved header bytes at the shifted frame ptr to 0, then write
    ///   (eb+0x14 + eb+0x16 - 8) & 0x3fff into that first word (a length the MAC reads).
    /// eb+4 and eb+8 are the SAME descriptor. HE (flags bit31) / AMPDU-CCMP (flags & 0x1040000)
    /// branches are not our path and are skipped. Returns 0 (proceed; the blob returns non-1 here).
    pub fn cr_ppProcTxSecFrame(eb: Eb) -> i32 {
        unsafe {
            let txinfo = eb.txinfo();
            // key-type -> sec-hdr length (open/no-key => 4; blob table _LANCHOR32 @ 0x4200b100).
            let ktype = ((txinfo.word10() >> 8) & 0xf).wrapping_sub(1) & 0xff;
            let seclen: u32 = if ktype <= 8 {
                rd8(0x4200_b100u32 + ktype) as u32
            } else {
                4
            };
            // eb+0x16 += seclen ; dma length += seclen
            let v16 = eb.payload_len() as u32 + seclen;
            eb.set_payload_len(v16 as u16);
            let dma = eb.sec_desc();
            dma.add_len(seclen);
            let flags = txinfo.flags();
            // HE / AMPDU-CCMP are separate blob branches; the open beacon has neither.
            if (flags & 0x8000_0000) != 0 || (flags & 0x0104_0000) != 0 {
                return 0;
            }
            dma.set_ctrl(dma.ctrl() | DmaDesc::CTRL_EOF); // eof
            let dma4 = eb.dma_desc(); // same descriptor as eb+8
            if (eb.flags16() & Eb::FLAG16_HDR_SHIFTED) == 0 {
                dma4.set_frame_ptr(dma4.frame_ptr().wrapping_sub(8)); // frame ptr -= 8
                let v14 = eb.prefix_len() as u32 + 8;
                eb.set_prefix_len(v14 as u16);
                eb.set_flags16(eb.flags16() | Eb::FLAG16_HDR_SHIFTED);
                dma4.add_len(8);
            }
            // zero the 8 reserved header bytes at the shifted frame ptr, then write the length word.
            let hdr = dma4.frame_ptr();
            core::ptr::write_bytes(hdr as *mut u8, 0, 8);
            let l14 = eb.prefix_len() as u32;
            let l16 = eb.payload_len() as u32;
            let lenword = (l14 + l16 - 8) & 0x3fff;
            wr(hdr, lenword | (rd(hdr) & 0xffff_c000));
            0
        }
    }

    /// ic_interface_enabled: the per-VIF enable check that gates ppTxPkt, a faithful port of the
    /// ROM function (0x40012c34, the one the blob's ppTxPkt jalrs): base = *(wDevCtrl_ptr @
    /// 0x4087ff68); mask = byte at base+0x31 (g_if_enabled_mask); return (mask >> iface) & 1.
    /// Validated against the ROM at runtime: rust(0)==rom(0)==1, rust(1)==rom(1)==0. A prior reimpl
    /// guessed the mask byte at the STATIC wDevCtrl+0x29; both the base (must come from the pointer)
    /// and the offset were wrong, so it read 0 on 0.3.0 -> the double-free bug.
    pub fn cr_ic_interface_enabled(iface: u32) -> i32 {
        let mask = blob_layout::if_enabled_mask() as u32;
        ((mask >> (iface & 0x1f)) & 1) as i32
    }

    /// Rust lmacIsLongFrame: MPDU length vs the RTS/long-frame threshold (lmacConfMib+0x16 on the
    /// 0.3.0 blob). `lmacConfMib` is an exported .data object whose address moves with the link
    /// layout (0x40811ca8 in one build, 0x408117d8 in another), so it is linked, not hardcoded. For
    /// our short broadcast beacon this is false; and in cr_lmacTxFrame the RTS it would gate is
    /// additionally suppressed by txinfo bit1 (broadcast), so the result is not on the beacon's
    /// critical path.
    pub fn cr_lmacIsLongFrame(eb: Eb) -> i32 {
        unsafe extern "C" {
            static mut lmacConfMib: u8;
        }
        const LMACCONFMIB_LONG_FRAME_THRESHOLD: u32 = 0x16;
        unsafe {
            let mib = core::ptr::addr_of_mut!(lmacConfMib) as u32;
            let threshold = rd16(mib + LMACCONFMIB_LONG_FRAME_THRESHOLD) as i32;
            let len = eb.prefix_len() as i32 + eb.payload_len() as i32;
            (threshold < len) as i32
        }
    }

    /// Rust ppMapTxQueue for the legacy beacon: choose the EDCA AC and write it into txinfo+0x10
    /// bits20-23, then run the PM-wake (the ingredient that makes the MAC active). For our raw beacon
    /// trc==0, so it takes the simple branch: txinfo+4=7, AC=iface. The QoS-data/TWT branches
    /// (ppSearchTxQueue / pm_on_twt_force_tx) are not exercised by the beacon and are omitted.
    /// ppProcessWaitingQueue (the per-iface hmac WAITING-queue drain) is a no-op on our path: our
    /// beacon is submitted straight to the pending list (proven: skipping it radiates with a healthy
    /// pool). Returns the blob convention: 0 = mapped.
    pub fn cr_ppMapTxQueue(eb: Eb) -> i32 {
        unsafe {
            let txinfo = eb.txinfo();
            let t4 = txinfo.cat_word();
            if (t4 & 0xf0) == 0x40 {
                txinfo.set_word10((txinfo.word10() & TxInfo::WORD10_AC_CLEAR) | 0x20_0000);
            } else {
                let iface = txinfo.iface();
                let trc = eb.trc();
                if trc == 0 || (rd16(trc + 0xc) & 0x80) != 0 {
                    txinfo.set_cat(7);
                    txinfo.set_word10(
                        (txinfo.word10() & TxInfo::WORD10_AC_CLEAR)
                            | (iface << TxInfo::WORD10_AC_SHIFT),
                    );
                    let use_blob =
                        !config::PM_WAKE_RUST || (config::AB_WAKE && AB_USE_BLOB.load(Relaxed));
                    pm_wake::on_data_tx(iface, use_blob);
                }
                // (QoS-data / TWT mapping branches not exercised by the legacy beacon.)
            }
            0
        }
    }

    /// Rust ppGetTxframe: pop the head eb from the per-AC pending list in TxRxCxt, faithful to the
    /// blob (head +0x20, tail +0x24, threaded via eb+0x30; empty -> tail=&head), with the blob's
    /// guard (+0x29==0 && +0x34==0). Our single beacon is enqueued to this AC, so this directly
    /// dequeues it (the blob's multi-queue ppSearchTxframe selection/bitmap is unnecessary for our
    /// controlled single-AC submit). Returns Eb(0) when empty.
    pub fn cr_ppGetTxframe(ac: i32) -> Eb {
        let q = PendingQ::for_ac(ac as u32);
        if q.guard_byte() == 0 && q.guard_word() == 0 {
            let head = Eb(q.head());
            if !head.is_null() {
                let next = head.pending_next();
                q.set_head(next);
                if next == 0 {
                    q.set_tail(q.head_addr()); // empty -> tail = &head
                }
                head.set_pending_next(0);
                // (blob calls lmacAdjustTimestamp() here -- a beacon-timestamp fixup that
                // derefs an AP/beacon context null in our raw path; our beacon uses timestamp=0
                // so it is unnecessary and omitted.)
                return head;
            }
        }
        Eb(0)
    }

    /// Rust completion: the lmacProcessTxComplete + lmacTxDone essentials, driven from our own
    /// loop (NOT the blob ISR). The MAC clears PLCP0_ENABLE's arm bits when the TX finishes; we poll
    /// that (bounded, out-of-band), read the completion result via our hal_mac_get_txq_complete,
    /// clear the txq_state bit, and recycle the eb. Returns (completed, status_nibble).
    pub fn cr_complete(ac: i32, eb: Eb) -> (bool, u8) {
        unsafe {
            let a = mac_reg::plcp0_enable(ac as u32);
            let mut done = false;
            for _ in 0..4000 {
                if (rd(a) & mac_reg::SLOT_ARM) == 0 {
                    done = true;
                    break;
                }
            }
            // lmacProcessTxComplete-equivalent read of the completion result for our AC.
            let txq = LmacTxq::for_ac(ac as u32);
            let mut res6 = [0u8; 8];
            let mut aux8 = [0u32; 2];
            hal_mac_tx::hal_mac_get_txq_complete(
                txq.ptr() as *mut i32,
                ac,
                res6.as_mut_ptr(),
                aux8.as_mut_ptr(),
            );
            let status = (res6[1] >> 4) & 0xf; // (pmd>>12)&0xf : 0=success
            if config::DIAG {
                TX_STATUS[(status & 0xf) as usize].fetch_add(1, Relaxed);
            }
            hal_mac_tx::hal_mac_clr_txq_state(2, ac as u32); // clear completed-state bit (as the blob does)
            // NOTE: do NOT hal_mac_txq_disable here -- the MAC auto-clears the arm bits on
            // completion; forcing a disable leaves slot 0 in a state that breaks the shared control
            // beacon (AC 0). The blob completion never disables the slot.
            // Single recycle of the eb: cr_ppTxPkt's drop-path is not taken (the ic_interface_enabled
            // fix), and the blob MAC-complete ISR skips our AC because we never set
            // our_instances[ac].state=1. (Before the fix this line double-freed the eb and corrupted
            // the heap.) Verified over sustained runs: recycles==arms, dblfree==0, allocfail==0.
            recycle(eb);
            (done, status)
        }
    }

    /// Rust hal_random: the blob's is just g_wifi_osi_funcs._rand(); we only use it for the EDCA
    /// backoff (masked to the CW window), so a self-contained xorshift PRNG is equivalent.
    static CR_RNG: AtomicU32 = AtomicU32::new(0x1234_5678);
    pub fn cr_hal_random() -> u32 {
        let mut x = CR_RNG.load(Relaxed);
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        CR_RNG.store(x, Relaxed);
        x
    }

    /// Rust rcGetSched: for raw frames trc==NULL the blob returns immediately (rate comes from the
    /// descriptor). Our beacon always has trc==0, so this is a no-op. (trc!=0 rate-control
    /// selection is not exercised by the legacy beacon and stays unimplemented.)
    pub fn cr_rcGetSched(trc: u32, _txinfo: TxInfo) {
        if trc == 0 {
            return;
        }
    }

    /// Rust lmacSetTxFrame (PPDU build) for the raw beacon. The TXOP-queue request + TSF-lifetime
    /// sequencing is reduced to the beacon path: no aggregation (trc==0), and a fixed lifetime (the
    /// timeout is not the radiate gate). The actual slot programming goes through our
    /// `hal_mac_tx` (hal_mac_tx_config_timeout + hal_mac_tx_set_ppdu).
    pub fn cr_lmacSetTxFrame(txq: LmacTxq) {
        let _cur_eb = txq.cur_eb(); // mode 0 -> eb = *txq (cur_eb)
        let ours = blob_layout::our_instances_base(); // hal_mac_tx_set_ppdu param2
        hal_mac_tx::hal_mac_tx_config_timeout(txq.ptr(), 0x7ff);
        hal_mac_tx::hal_mac_tx_set_ppdu(txq.ptr(), ours as i32);
    }

    /// Rust lmacTxFrame (the ARM) for the legacy DSSS beacon, on the REAL our_instances[ac] state:
    /// cur_eb, long-frame RTS flag, PPDU build (cr_lmacSetTxFrame), random EDCA backoff,
    /// config_edca, and the slot arm (txq_enable).
    pub fn cr_lmacTxFrame(eb: Eb, ac: i32) {
        let txq = LmacTxq::for_ac(ac as u32);
        let txinfo = eb.txinfo();
        let flags = txinfo.flags();
        // Discard path (txinfo bit16 & !offchan) is not taken by a normal beacon -> omitted.
        let state = txq.state();
        if state == LmacTxq::STATE_IDLE || state == LmacTxq::STATE_RELEASED {
            txq.set_cur_eb(eb);
            if (flags & 0x2102) == 0x2000 {
                txinfo.set_flags(flags | 0x1000);
            }
            // long-frame -> RTS (beacon is short; lmacIsLongFrame returns 0 -> no-op, but
            // faithful)
            if cr_lmacIsLongFrame(eb) != 0 && (txinfo.flags() & 2) == 0 {
                txinfo.set_flags((txinfo.flags() & 0xffff_efff) | 0x100);
            }
            // state==3 retry-RTS and FTM (0x20000000) branches skipped (not taken by the
            // beacon).
            cr_lmacSetTxFrame(txq); // PPDU build (Rust; slot programming via our Rust hal)
        }
        // EDCA random backoff masked by CW exponent at txq+8, exactly as the blob.
        let r = cr_hal_random();
        let cw = txq.cw() as u32;
        let backoff = (!(0xffff_ffffu32 << (cw & 0x1f)) & r) as u16;
        txq.set_backoff(backoff);
        hal_mac_tx::hal_mac_tx_config_edca(txq.ptr());
        // NOTE: we deliberately do NOT set our_instances[ac].state(+0x12)=1 here. With state!=1
        // the blob MAC ISR's lmacProcessTxComplete skips our AC (only clears the completed
        // bit), so it does NOT recycle our eb -- our Rust cr_complete() owns the
        // completion + recycle.
        hal_mac_tx::hal_mac_txq_enable(txq.slot() as i32);
    }

    /// Rust ppProcessTxQ for the legacy DSSS beacon path, operating on the REAL our_instances[ac]
    /// state.
    pub fn cr_ppProcessTxQ(ac: i32) -> i32 {
        let txq = LmacTxq::for_ac(ac as u32);
        // lmacIsIdle(ac): our_instances[ac].state(+0x12) must be 0 (idle). pm/twt/mesh guards
        // are permissive for a non-connected beacon-only build -> skipped.
        if txq.state() != LmacTxq::STATE_IDLE {
            return -1;
        }
        let eb = cr_ppGetTxframe(ac); // Rust pop from the real TxRxCxt pending list
        if eb.is_null() {
            return -2;
        }
        // Legacy DSSS beacon: txinfo flags have no HE(bit31)/AMPDU(0x400000)/0x1040000 bit and
        // trc(eb+0x2c)==0, so the blob's AMPDU-reorder and RTS/fragment branches are NOT taken
        // (verified against the decompile) -> go straight to the arm. pp_coex_tx_request is a
        // coex-signaling no-op on our path (proven: skipping it keeps the MAC waking, latching and
        // radiating with a healthy pool) -> Rust no-op.
        cr_lmacTxFrame(eb, ac);
        0
    }

    /// FAITHFUL submit+schedule+arm on REAL scheduler state, from our task (blob pp idle).
    /// Returns (ac, txpkt_ret, blk_before, blk_after, plcp0_before, plcp0_after).
    pub fn faithful_tx(eb: Eb, seq: u16) -> (i32, i32, u32, u32, u32, u32) {
        unsafe {
            // Mirror ieee80211_output_raw_process's eb/dma/txinfo setup BEFORE ppTxPkt (the fields
            // the blob's raw-submit fills), then run the Rust ppTxPkt (proto/sec/rate/map/enqueue,
            // kick=0). ppMapTxQueue will set the AC; do NOT pre-set AC bits here.
            let dma = eb.dma_desc();
            let frame = dma.frame_ptr();
            eb.set_prefix_len(0);
            let l16 = eb.payload_len() as u32;
            let mut w0 = dma.ctrl();
            w0 |= DmaDesc::CTRL_OWNER;
            w0 |= DmaDesc::CTRL_EOF;
            w0 &= DmaDesc::CTRL_BIT29_CLEAR;
            w0 = ((l16 & DmaDesc::CTRL_LEN_MASK) << DmaDesc::CTRL_LEN_SHIFT)
                | (w0 & DmaDesc::CTRL_KEEP_MASK);
            dma.set_ctrl(w0);
            let txinfo = eb.txinfo();
            txinfo.set_cat(7); // cat = mgmt
            txinfo.set_timestamp(cr_hal_now());
            let mut w10 = txinfo.word10();
            w10 &= 0xfff7_ffff; // iface 0
            txinfo.set_word10(w10);
            if (rd8(frame + 4) & 1) != 0 {
                txinfo.set_flags(txinfo.flags() | 0x402);
            }
            txinfo.set_rate(0); // rate 1M DSSS
            wr16(frame + 0x16, seq << 4); // seq ctrl
            eb.set_trc(0); // trc = NULL (raw frame -> rcGetSched no-op)

            // Faithful submit onto the REAL pending list (no kick).
            let ret = cr_ppTxPkt(eb, 0);
            // AC that ppMapTxQueue assigned into txinfo+0x10 bits 20-23.
            let ac = txinfo.ac() as i32;
            let blk_before = rd(mac_reg::PM_TXBLOCK_RETENTION);
            let plcp0_before = rd(mac_reg::plcp0_enable(ac as u32));
            // Faithful schedule + arm on real state (pop + lmacTxFrame -> our Rust hal).
            let _ = cr_ppProcessTxQ(ac);
            let plcp0_after = rd(mac_reg::plcp0_enable(ac as u32));
            let blk_after = rd(mac_reg::PM_TXBLOCK_RETENTION);
            (ac, ret, blk_before, blk_after, plcp0_before, plcp0_after)
        }
    }
}

// ============================================================================
// diag — out-of-band diagnostics
// ============================================================================
/// Boot-time probes and dumps. Never on the hot TX path.
mod diag {
    use super::{mac_reg, rd, wr};

    /// One-time write/read probe across all 8 slots: which slot config banks are writable?
    /// (Writes to slots whose bank the scheduler has not activated are dropped on the C6 MAC.)
    pub fn probe() {
        unsafe {
            for s in 0..8u32 {
                let a = mac_reg::plcp0_enable(s);
                let e = mac_reg::conf1(s);
                let p0 = rd(a);
                let e0 = rd(e);
                // write-test the EDCA-conf reg (non-arm, safe to restore) unless it's live
                let writable = if e0 == 0 {
                    wr(e, 0x0000_0abc);
                    let ok = rd(e) == 0x0000_0abc;
                    wr(e, 0);
                    ok
                } else {
                    true // already holds scheduler-written bits
                };
                esp_println::println!(
                    "[CR.probe] slot{s} plcp0={p0:#010x} edca={e0:#010x} writable={writable}"
                );
            }
        }
    }

    /// One-shot dump of the PHY TX-power / gain-mem state. 0x600a4400 min-pwr
    /// (hal_get_tx_min_pwr), the gain-mem words 0x600a08c8..d4 (set_tx_gain_mem; verified to change
    /// during TX in prior sessions), 0x600a0910 / 0x600a00c0 FE config, 0x600a7030 BB. Comparing
    /// the post-init dump (== the blob's, since both run the same WifiController::new) against a
    /// post-loop dump shows whether our path lowers TX power/gain over a run.
    pub fn pwr_dump(tag: &str) {
        unsafe {
            esp_println::println!(
                "[CR.PWR] {tag} minpwr={:#010x} gain[c8/cc/d0/d4]={:#010x} {:#010x} {:#010x} {:#010x} fe0910={:#010x} fe00c0={:#010x} bb7030={:#010x}",
                rd(mac_reg::TX_MIN_PWR),
                rd(mac_reg::GAIN_MEM_C8),
                rd(mac_reg::GAIN_MEM_CC),
                rd(mac_reg::GAIN_MEM_D0),
                rd(mac_reg::GAIN_MEM_D4),
                rd(mac_reg::FE_0910),
                rd(mac_reg::FE_00C0),
                rd(mac_reg::BB_7030),
            );
        }
    }
}

// ============================================================================
// beacon — the beacon payloads
// ============================================================================
/// The two beacons: `CR-CTRL` goes through the blob's own send_raw_frame path (an RF baseline at
/// boot), `CR-RUST` through our pipeline. Both are open 1 Mbit DSSS beacons on channel 1.
mod beacon {
    use core::marker::PhantomData;
    use ieee80211::{
        common::{CapabilitiesInformation, FCFFlags},
        element_chain,
        elements::{DSSSParameterSetElement, RawIEEE80211Element, SSIDElement},
        mgmt_frame::{BeaconFrame, body::BeaconBody, header::ManagementFrameHeader},
        scroll::Pwrite,
        supported_rates,
    };

    pub const SSID_CTRL: &str = "CR-CTRL";
    pub const SSID_RUST: &str = "CR-RUST";
    pub const MAC_CTRL: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0xda, 0xb0];
    pub const MAC_RUST: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0xc6, 0x00];

    pub fn build(buf: &mut [u8], ssid: &str, mac: [u8; 6]) -> usize {
        buf.pwrite(
            BeaconFrame {
                header: ManagementFrameHeader {
                    fcf_flags: FCFFlags::new(),
                    duration: 0,
                    receiver_address: [0xff; 6].into(),
                    transmitter_address: mac.into(),
                    bssid: mac.into(),
                    ..Default::default()
                },
                body: BeaconBody {
                    timestamp: 0,
                    beacon_interval: 100,
                    capabilities_info: CapabilitiesInformation::new().with_is_ess(true),
                    elements: element_chain! {
                        SSIDElement::new(ssid).unwrap(),
                        supported_rates![1 B, 2 B, 5.5 B, 11 B, 6, 9, 12, 18],
                        DSSSParameterSetElement { current_channel: 1 },
                        RawIEEE80211Element {
                            tlv_type: 5,
                            slice: [0x01, 0x02, 0x00, 0x00].as_slice(),
                            _phantom: PhantomData
                        }
                    },
                    _phantom: PhantomData,
                },
            },
            0,
        )
        .unwrap()
    }
}

// ============================================================================
// main — bring-up + the sustained pure-Rust TX loop
// ============================================================================
#[esp_hal::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    use core::sync::atomic::Ordering::Relaxed;

    esp_println::logger::init_logger_from_env();
    let hal_config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(hal_config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);

    let delay = Delay::new();
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let mut controller =
        esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default()).unwrap();
    controller
        .set_power_saving(esp_radio::wifi::PowerSaveMode::None)
        .unwrap();
    let mut sniffer = controller.sniffer();

    // Control beacon (blob scheduler path, radiates) + our Rust-armed beacon payload.
    let mut ctrl_buf = [0u8; 300];
    let ctrl_len = beacon::build(&mut ctrl_buf, beacon::SSID_CTRL, beacon::MAC_CTRL);
    let ctrl = &ctrl_buf[..ctrl_len];

    let mut rust_buf = [0u8; 300];
    let rust_len = beacon::build(&mut rust_buf, beacon::SSID_RUST, beacon::MAC_RUST);
    let rust = &rust_buf[..rust_len];

    println!(
        "[CR] booting; control SSID '{}', Rust-arm SSID '{}' slot AC{}",
        beacon::SSID_CTRL,
        beacon::SSID_RUST,
        config::MY_AC
    );

    // Let the blob bring-up settle and prove RF with a few control beacons first.
    if config::CONTROL_BEACON {
        for _ in 0..10 {
            let _ = sniffer.send_raw_frame(true, ctrl, false);
            delay.delay(Duration::from_millis(100));
        }
    }

    // Which TX slot config banks the scheduler has activated (diagnostic).
    if config::DIAG {
        diag::probe();
        pm_wake::snapshot();
        diag::pwr_dump("post-init");
    }
    // Initialise the independent Rust eb pool (no blob esf_buf in the hot path).
    eb_pool::init(rust);

    // Sustained pure-Rust TX: each round allocates a fresh eb, runs the full Rust submit->schedule
    // ->arm on the REAL blob scheduler state (blob pp idle), then Rust completion+recycle. The
    // [CR.H] oracle (out-of-band) should read arms==latched==completed, allocfail==0, and
    // last_plcp0==0xc0...... (our frame armed on slot 0).
    println!("[CR.F] pure-Rust TX pipeline on real scheduler state (submit/schedule/arm/complete)");
    let mut round: u32 = 0;
    let mut seq: u16 = 0;
    let (mut n_arm, mut n_latch, mut n_done, mut n_allocfail) = (0u32, 0u32, 0u32, 0u32);
    let mut last_pa: u32 = 0;
    loop {
        if config::AB_WAKE {
            pipeline::AB_USE_BLOB.store(round % 2 == 1, Relaxed);
        }
        // ----- TX phase: one full submit+schedule+arm+complete per iteration (no send_raw_frame).
        for _ in 0..4u32 {
            if config::TX_MODE_CONTROL_ONLY {
                break; // control-only: emit only the blob CR-CTRL beacon below
            }
            let feb = if config::CR_POOL_ENABLED {
                eb_pool::alloc(rust)
            } else {
                eb_pool::blob_alloc(rust)
            };
            if feb.is_null() {
                n_allocfail += 1;
            } else {
                let (ac, _ret, _bb, _ba, _pb, pa) = pipeline::faithful_tx(feb, seq);
                seq = seq.wrapping_add(1);
                n_arm += 1;
                if pa & mac_reg::SLOT_ARM != 0 {
                    n_latch += 1;
                }
                last_pa = pa;
                delay.delay(Duration::from_millis(40)); // let the frame win the medium + TX + complete
                // Rust-driven completion + recycle (single owner of the eb).
                let (completed, _status) = pipeline::cr_complete(ac, feb);
                if completed {
                    n_done += 1;
                }
            }
            delay.delay(Duration::from_millis(60));
        }

        // ----- Control phase: a few blob-scheduled CR-CTRL beacons as a same-RF reference.
        if config::CONTROL_BEACON && config::CONTROL_BURST {
            for _ in 0..6 {
                let _ = sniffer.send_raw_frame(true, ctrl, false);
                delay.delay(Duration::from_millis(100));
            }
        }
        round += 1;
        if config::DIAG && round == 8 {
            diag::pwr_dump("post-loop");
        }
        // Periodic health + oracle summary (out-of-band; catchable in any monitor window).
        if config::DIAG && round % 8 == 0 {
            let pmblk = pm_wake::PM_BLK.load(Relaxed);
            println!(
                "[CR.H] rounds={round} arms={n_arm} latched={n_latch} completed={n_done} allocfail={n_allocfail} last_plcp0={last_pa:#010x} pm_blk={:#06x}->{:#06x} poolfree={} pm_wakes={} pre_blk={} post_blk={} pm_odd={} blk_now={:#010x}",
                pmblk >> 16,
                pmblk & 0xffff,
                eb_pool::free_len(),
                pm_wake::PM_WAKES.load(Relaxed),
                pm_wake::PM_PRE_BLOCKED.load(Relaxed),
                pm_wake::PM_POST_BLOCKED.load(Relaxed),
                pm_wake::PM_ODD.load(Relaxed),
                unsafe { rd(mac_reg::PM_TXBLOCK_RETENTION) }
            );
            let st = &pipeline::TX_STATUS;
            println!(
                "[CR.S] status ok={} s1={} s2={} s3={} s4={} s5={} s6={} s7={} s8+={}",
                st[0].load(Relaxed),
                st[1].load(Relaxed),
                st[2].load(Relaxed),
                st[3].load(Relaxed),
                st[4].load(Relaxed),
                st[5].load(Relaxed),
                st[6].load(Relaxed),
                st[7].load(Relaxed),
                st[8..].iter().map(|a| a.load(Relaxed)).sum::<u32>()
            );
            if config::PM_WAKE_EXPERIMENT != 0 || !config::CONTROL_BURST || !config::PM_WAKE_RUST {
                println!(
                    "[CR.X] exp={} burst={} rust_wake={} blk_after_store={:#010x}",
                    config::PM_WAKE_EXPERIMENT,
                    config::CONTROL_BURST,
                    config::PM_WAKE_RUST,
                    pm_wake::PM_BLK_AFTER_STORE.load(Relaxed)
                );
            }
        }
        if config::DIAG && round % 5 == 0 {
            println!("[CR.F] completed {round} rounds");
        }
    }
}
