//! Pure-Rust ESP32-C6 Wi-Fi TX pipeline demonstrator, running on top of the esp-radio
//! (esp-wifi-sys) blob substrate. It transmits a beacon (SSID "CR-RUST") whose whole
//! per-frame TX path is driven by Rust on the REAL blob scheduler state, with only a
//! minimal, documented set of blob leaves left (see the `blob` extern blocks).
//!
//! Mechanism: the ESP32-C6 MAC holds an interlock, WDEV_PM_TXBLOCK_RETENTION @ 0x600a4ca8
//! (0x00ff1000 = blocked, 0 = active); while blocked, the PLCP0_ENABLE launch bits
//! (0xc0000000) never latch and nothing radiates. The blob `pm_on_data_tx` PM-wake FSM
//! clears that interlock (a hardware consequence of the FSM reaching the wake state +
//! wifi_rf_phy_enable, not a single register write). With the MAC active, our Rust drives
//! the full pipeline on the blob's own scheduler state:
//!   submit (cr_ppTxPkt) -> AC map + PM-wake (cr_ppMapTxQueue) -> enqueue onto the TxRxCxt
//!   pending list -> pop (cr_ppGetTxframe) -> schedule (cr_ppProcessTxQ) -> arm
//!   (cr_lmacTxFrame -> cr_lmacSetTxFrame -> Rust hal_mac_tx register ops) -> completion +
//!   recycle (cr_complete). State lives in the blob globals: our_instances @ *0x4004ffe0
//!   (per-AC lmac txq blocks, stride 0x34) and TxRxCxt pending @ *0x4087ff80.
//!
//! WARNING — blob-version-specific addresses. Every fixed address/offset that is NOT a
//! hardware MMIO register is specific to the esp-wifi-sys 0.3.0 blob and MUST be re-derived
//! per blob version from the linked-ELF / ROM disassembly: our_instances (*0x4004ffe0),
//! TxRxCxt pending (*0x4087ff80), wDevCtrl_ptr (*0x4087ff68) + g_if_enabled_mask (+0x31),
//! lmacConfMib, and the esf_buf pool list heads. A wrong base or offset does not fault
//! loudly — it silently reads/writes the wrong memory. That is exactly the bug that bit us:
//! a stale ic_interface_enabled mask address read 0, sent every frame down the drop path,
//! and the resulting double-free of the eb corrupted the heap. Re-verify all of these
//! against the disasm when bumping the blob.
//!
//! Full write-up (investigation, register/struct model, remaining blob surface):
//! docs/cleanroom_tx.md in the esp32c6-open-mac repo.

#![no_std]
#![no_main]

use core::marker::PhantomData;

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, delay::Delay, time::Duration, timer::timg::TimerGroup};
use esp_println::println;
use ieee80211::{
    common::{CapabilitiesInformation, FCFFlags},
    element_chain,
    elements::{DSSSParameterSetElement, RawIEEE80211Element, SSIDElement},
    mgmt_frame::{BeaconFrame, body::BeaconBody, header::ManagementFrameHeader},
    scroll::Pwrite,
    supported_rates,
};

esp_bootloader_esp_idf::esp_app_desc!();

// ============================================================================
// SECTION 1 — Blob interposition shims
// ============================================================================
// The cross-boundary hal_mac_tx.o / lmac.o symbols were renamed in libpp.a to `blob_<fn>`.
// The global_asm! block re-provides `<fn>` as an ABI-perfect tail-call into `blob_<fn>`, so
// pp/lmac call OUR symbols; each `<fn>` we have reimplemented in Rust below simply omits its
// shim (our strong def wins). The shims that remain are blob leaves we deliberately keep.
//
// ---- Phase 1 interposition shims: `<fn>` -> tail blob_<fn> (all 23 cross-boundary
// symbols of hal_mac_tx.o). `tail` preserves every argument register, so this is a
// perfectly transparent passthrough regardless of the real signature. ----
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

    SHIM hal_init_tx_pwr
    SHIM mac_tx_set_hesig
    SHIM mac_tx_set_mplen
    SHIM mac_tx_set_tb

    // ---- lmac.o cross-boundary shims (Phase 1: 46 functions -> tail blob_<fn>). ----
    // NOTE: functions with a ROM *ABS* symbol at 0x40000xxx (GetAccess, is_lmac_idle, lmacIsIdle,
    // lmacIsLongFrame, lmacReachShortLimit, lmacReachLongLimit, lmacDiscardAgedMSDU,
    // lmacPostTxComplete, lmacProcessAckTimeout, lmacProcessAllTxTimeout, lmacProcessCollisions,
    // lmacProcessShortFrameSuccess, lmacProcessLongFrameSuccess, lmacRecycleMPDU, lmacRxDone) are
    // ROM-provided. Our
    // strong Rust defs for those get --gc-sections'd in favour of the ROM copy, and dropping their
    // shim binds callers to the ROM version -- which uses different global state than libpp and
    // STALLS TX. So they MUST stay shims (-> blob_<fn> = libpp copy). Only NON-ABS lmac functions
    // are interposable; the one de-blobbed here is lmac_update_tx_statistic (below).
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

// ============================================================================
// lmac.o Phase 2: real Rust reimplementations of lmac.o cross-boundary functions.
// ============================================================================
// The lmac scheduler keeps its per-AC TX control state in the `our_instances` array, whose base
// pointer lives in the fixed pp/ROM pointer-table slot `our_instances_ptr` (address 0x4004ffe0 in
// the linked firmware). The blob materialises the base with `lui a5,0x40050; lw a5,-0x20(a5)`
// (verified in the LINKED blob_GetAccess @0x40805126), i.e. base = `*(u32*)0x4004ffe0`, then
// indexes `our_instances[ac] = base + ac*0x34` (5 ACs, stride 0x34). We reference the exported
// `our_instances_ptr` symbol and read it the same way (in the Ghidra analysis elf this symbol
// relocated to address 0, which is why the decompiler renders the base as `iRam00000000`).
//
// lmac_txq_c6 block layout (base + ac*0x34), from GetAccess/txsm_map + disasm:
//   +0x00 cur_eb (armed frame ptr)      +0x05 aifsn        +0x08 cw (clamped)
//   +0x09 cwmin  +0x0a cwmax            +0x12 state{0 idle,1 armed,5 success,6 error}
//   +0x18 rate2ampdu (i16)             +0x1d txop_depth
//   +0x20 pending_head  +0x24 pending_tail   +0x2d.. completion result
//
// lmac config globals live in the exported `lmacConfMib` .data object (0x40811b68 in this
// link). Verified from the LINKED firmware disasm of blob_lmacIsLongFrame/Reach*Limit:
//   lmacConfMib[0x14] = u8 long-retry-limit   lmacConfMib[0x15] = u8 short-retry-limit
//   lmacConfMib[0x16] = u16 RTS / long-frame threshold (read via `lhu`)
// We reference the `lmacConfMib` symbol (+offset) so the read is link-stable and sees the same
// runtime value the blob's lmacInit/config wrote (Ghidra's analysis-elf DAT_ram_42080dd0.. are the
// pre-relocation flash copies of these same fields). Only NON-ABS (interposable) lmac symbols are
// reimplemented here; the ~15 ROM *ABS* ones (see the shim NOTE above) must stay shims. Full model
// + blocker log: docs/deblob_progress.md in the research repo.
mod lmac_deblob {
    /// blob `lmac_update_tx_statistic`: empty in the blob (just `ret`; blob_ at 0x42025b8c). Its
    /// address is installed into the wdev funcs pointer table by wdev_funcs_init and invoked
    /// indirectly. Reimplemented as a no-op (correct by inspection: the blob body is empty).
    /// This is the ONLY lmac function that can be safely interposed -- see the blocker note below.
    #[unsafe(no_mangle)]
    pub extern "C" fn lmac_update_tx_statistic() {}
}

// BLOCKER -- lmac context functions are NOT interposable (evidence-backed; do not retry blindly).
// Any lmac function that touches the `our_instances` context regresses TX to ~10-40% steady, while
// the empty no-op above holds the full 100/10s beacon rate (control-verified: update_stat-only =
// 307/30s). Tested exhaustively:
//   * lmacGetTxFrame -- reimplemented as safe Rust (read_volatile), inline-asm (blob-exact `lw`),
//     and a #[naked] copy BYTE-IDENTICAL to blob_lmacGetTxFrame (0x420256de): all regress (241/60s,
//     then 5/60s for the naked copy at a different address). Both flash and IRAM (#[esp_hal::ram])
//     placements regress (9/30s).
//   * lmacSetAcParam -- init-time only (not a per-frame path) -- also regresses (9/30s).
// A byte-identical machine-code copy at a DIFFERENT address killing TX rules out the code itself
// and points to a placement/dispatch dependency: the blob lmac functions must live at their
// original libpp addresses (likely a ROM/wdev function-pointer table, or an i-cache/co-location
// constraint on the timing-sensitive TX path). The no-op works precisely because it reads nothing.
// Combined with the ROM *ABS* wall (the ~15 symbols above), lmac.o is not meaningfully de-blobbable
// via symbol interposition beyond this no-op. Full analysis + reproduction:
// docs/deblob_progress.md.

// ---- Phase 2: real Rust reimplementations of hal_mac_tx.o functions ----
// Each replaces its global_asm shim above. Verified against the blob decompilation.
// PLCP0_ENABLE for blob queue 0 (highest slot) is 0x600a_4d6c; higher queue index
// steps DOWN by 0x10 (the blob's reversed slot numbering), so addr = 0x600a_4d6c - ac*0x10.
mod blob {
    unsafe extern "C" {
        // config_timeout: our Rust hal_mac_tx_config_timeout delegates the non-beacon (RTS/CTS)
        // timeout programming to the blob; clr_mplen: aggregate mplen-bitmap teardown on completion.
        pub fn blob_hal_mac_tx_config_timeout(txq: *mut u8, param2: i32) -> u32;
        pub fn blob_hal_mac_tx_clr_mplen(param1: i32, q: i32);
    }
}

// hal_mac_tx.o leaves still called by our Rust `hal_mac_tx_set_ppdu`. plcp0/plcp1/txop_q/len/
// rts_rate are Rust now; these four are HT/HE-SIG, coex-PTI and aggregate-TXOP leaves that the
// legacy 1 Mbit DSSS beacon path calls but does not exercise (HE/AMPDU branches are not taken).
unsafe extern "C" {
    fn mac_tx_set_pti(p: *mut u8); // coex packet-traffic-indication (CONF-region writes)
    fn mac_tx_set_hesig(); // HE-SIG field (not taken by a legacy beacon)
    fn mac_tx_set_htsig(p: *mut u8, param2: i32); // HT-SIG field (not taken by a legacy beacon)
    fn hal_mac_fill_hwtxop(eb: u32, depth: u32, idx: u32); // per-MPDU TXOP fill (dead for single frames)
}

// ============================================================================
// SECTION 2 — Rust hal_mac_tx register ops
// ============================================================================
// The MAC TX register file (all real hardware MMIO). Queue index maps to slots in REVERSE
// (queue 0 = highest slot); per-queue address = base - q*stride. These #[unsafe(no_mangle)]
// functions replace the hal_mac_tx.o shims: the blob's lmac/pp calls them directly, and our
// Rust PPDU path (cr_lmacSetTxFrame -> hal_mac_tx_set_ppdu -> set_plcp0/plcp1/txop_q/len)
// calls them too. All addresses here are hardware registers (stable across blob versions).
//
// TXOP slot registers: RESP_DUR @0x600a54bc-slot*0x74, CONF0 @0x600a4d60-slot*0x10.
const WDEV_TXQ0_RESP_DUR: u32 = 0x600a_54bc;
const WDEV_TXQ0_CONF0: u32 = 0x600a_4d60;
// PLCP0_ENABLE base reused from txq_plcp0_enable_addr (0x600a_4d6c).
const WDEV_TXQ0_PLCP0: u32 = 0x600a_4d6c;

/// blob `mac_tx_set_txop_q`: program the slot's TXOP burst fields. For depth>2 it just clears
/// the RESP_DUR txop nibble; otherwise it sets RESP_DUR depth/count, CONF0 txop-valid bit
/// (per txinfo bit8), PLCP0_ENABLE bit22 (per txinfo bits6-7==0x80), and fills each chained
/// MPDU (dead for a single beacon: eb+0x30 chain is empty).
#[unsafe(no_mangle)]
pub extern "C" fn mac_tx_set_txop_q(param_1: *mut u8) -> u32 {
    unsafe {
        let slot = *param_1.add(4) as u32;
        let eb = core::ptr::read_unaligned(param_1 as *const u32);
        let depth = *param_1.add(0x1d) as u32;
        let resp_dur = WDEV_TXQ0_RESP_DUR.wrapping_sub(slot.wrapping_mul(0x74));
        let txinfo = rd_at(eb.wrapping_add(0x34));
        if depth > 2 {
            wr(resp_dur, rd(resp_dur) & 0xf0ff_ffff);
            return 0;
        }
        let s3 = *param_1.add(0x1c) as u32;
        wr(
            resp_dur,
            (rd(resp_dur) & 0xf0ff_ffff) | ((s3 << 0x18) & 0xf000_0000),
        );
        wr(resp_dur, (rd(resp_dur) & 0xcfff_ffff) | (depth << 0x1c));
        let conf0 = WDEV_TXQ0_CONF0.wrapping_sub(slot.wrapping_mul(0x10));
        if (rd_at(txinfo) & 0x100) != 0 {
            wr(conf0, rd(conf0) | 0x8000_0000);
        } else {
            wr(conf0, rd(conf0) & 0x7fff_ffff);
        }
        let plcp0 = WDEV_TXQ0_PLCP0.wrapping_sub(slot.wrapping_mul(0x10));
        if (rd_at(txinfo) & 0xc0) == 0x80 {
            wr(plcp0, rd(plcp0) | 0x40_0000);
        } else {
            wr(plcp0, rd(plcp0) & 0xffbf_ffff);
        }
        // Aggregate chain: fill each MPDU's HW TXOP. Empty for single frames.
        let mut s2 = rd_at(eb.wrapping_add(0x30));
        let mut s1: u32 = 1;
        while s2 != 0 {
            hal_mac_fill_hwtxop(s2, *param_1.add(0x1d) as u32, s1);
            if s3 == s1 {
                break;
            }
            s1 = (s1 + 1) & 0xff;
            s2 = rd_at(s2.wrapping_add(0x30));
        }
    }
    0
}

// ---- mac_tx_set_plcp0 (blocker #1) reimplementation + instrumentation ----
// Decomp (verified 2026-09-11 against blob disasm + esp-wifi-hal ll.rs set_plcp0):
//   eb = *param_1; u1 = read(eb+4); base = (u1 & 0xfffff) | 0x600000;
//   flags = read(read(eb+0x34));  // txinfo word0
//   v = base;
//   if (flags & 0x402)==0 && (flags & 0x40480000)!=0x400000 {
//     if (flags & 0x100000)==0 {
//       v = base | ((((flags>>0x13)&1)+1) << 24);          // ack-type in bits 24..26
//       if flags bit31 { if flags bit30 clear { v=(u1&0xfffff)|0x2600000 }
//                        else { iv = if (u16@(eb+0x24) & 0x1000)==0 {1} else {5}; v=base|(iv<<24) }
// }     } else { v = (u1&0xfffff)|0x3600000 }
//   }
//   write PLCP0_ENABLE(0x600a4d6c-slot*0x10) = v;    // full write; arm bits30/31 land here as 0
//   hal_he_set_tx_protection(slot, (flags>>8)&1, _, (txinfo[0xc]>>3)&0x3ff, txinfo[0xd])
// hal_he_set_tx_protection (blob leaf @0x42073f6a, NOT renamed) writes CONF0 (0x600a4d60-slot*0x10)
// bit31 (protect-enable, per arg2) and, when threshold!=0, 0x600a548c-slot*0x74 = val|0x10000.
// It does NOT touch PLCP0_ENABLE's arm bits (verified by disasm: a5=(0x600a4d6-slot)<<4=CONF0).
//
// For the legacy 1 Mbit DSSS beacon: flags has none of {bit1,bit10,bit19,bit20,bit30,bit31} set,
// so v = base | 0x1000000 = dma | 0x1600000 (matches esp-wifi-hal wait_for_ack=false: ack_type=1).
// arg2 = (flags>>8)&1, threshold = (txinfo+0x30 >>3)&0x3ff (0 for a beacon -> no threshold write).


#[inline(always)]
unsafe fn rd16(addr: u32) -> u16 {
    unsafe { core::ptr::read_volatile(addr as usize as *const u16) }
}

/// blob `mac_tx_set_plcp0`: program the slot's PLCP0_ENABLE dma/length/format word, then set
/// RTS/txop protection via the blob leaf. See the block comment above for the decomp.
///
/// NOTE ON INSTRUMENTATION: an earlier bring-up added per-call volatile-MMIO readbacks + atomic
/// counters here; those extra bus accesses on this timing-sensitive TX-setup path stalled the TX
/// queue (arm-without-complete), even when the register logic was byte-identical to the blob. So
/// this body is kept minimal -- the same read/compute/write/protect shape and count as the blob.
/// Rust mac_tx_get_rts_rate: pure rate -> RTS/response-rate lookup (faithful port of the blob's
/// jump table). Our 1 Mbit DSSS beacon has rate 0 -> returns 0. No addresses, hardware-independent.
fn cr_mac_tx_get_rts_rate(rate: i32) -> i32 {
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

/// Rust hal_he_set_tx_protection: CONF0 (0x600a4d60-slot*0x10) bit31 = protect-enable, and, when a
/// threshold is set, the RTS/txop-dur threshold reg (0x600a548c-slot*0x74). Register writes only.
#[allow(dead_code)]
unsafe fn cr_hal_he_set_tx_protection(slot: i32, enable: i32, _p3: u32, threshold: i32, val: u32) {
    unsafe {
        let conf0 = 0x600a_4d60u32.wrapping_sub((slot as u32).wrapping_mul(0x10));
        let mut v = rd(conf0);
        if enable == 0 {
            v &= 0x7fff_ffff;
        } else {
            v |= 0x8000_0000;
        }
        wr(conf0, v);
        if threshold != 0 {
            wr(
                0x600a_548cu32.wrapping_sub((slot as u32).wrapping_mul(0x74)),
                (val & 0xffff) | 0x1_0000,
            );
        }
    }
}

/// Rust mac_tx_set_len: writes the slot RESP_DUR word (0x600a54bc-slot*0x74: response-rate bits6-13,
/// cbw40 bit1, uVar5 bits22) and, for OFDM/HT rates, the TXLEN word (0x600a54b8-slot*0x74). For our
/// legacy DSSS beacon (rate 0) only RESP_DUR is written (the TXLEN branch needs an OFDM rate).
/// `param2` is the our_instances base (used only by the OFDM branches, not taken by the beacon).
unsafe fn cr_mac_tx_set_len(ctx: *mut u8, param2: i32) {
    unsafe {
        let eb = core::ptr::read_unaligned(ctx as *const u32);
        let txinfo = rd_at(eb + 0x34);
        let frame = rd_at(rd_at(eb + 4) + 4); // dma_desc[1]
        let rate = core::ptr::read_volatile((txinfo + 0xc) as *const u8) as i32;
        let t10 = rd(txinfo + 0x10);
        let rts = cr_mac_tx_get_rts_rate(rate) as u32;
        let ac = (t10 >> 0x14) & 0xf;
        let mut uvar5 = 1u32;
        if (rate.wrapping_sub(0x10) as u32 & 0xff) < 0x14 {
            uvar5 = (core::ptr::read_volatile(
                (ac.wrapping_mul(0x34).wrapping_add(param2 as u32).wrapping_add(0x40)) as *const u8,
            ) as u32)
                & 3;
        }
        let slot = *ctx.add(4) as u32;
        let resp_dur = 0x600a_54bcu32.wrapping_sub(slot.wrapping_mul(0x74));
        wr(
            resp_dur,
            uvar5 << 0x16 | ((t10 & 0xf000) == 0x1000) as u32 * 2 | (rts & 0xff) << 6 | 4,
        );
        let flags = rd(txinfo);
        if (flags as i32) >= 0 {
            let r2 = core::ptr::read_volatile((txinfo + 0xc) as *const u8) as u32;
            let mut uv3 = r2.wrapping_sub(0x10) & 0xff;
            if uv3 < 0x14 {
                let len = if flags & 0x40_0000 == 0 {
                    rd(frame) & 0x3fff
                } else {
                    rd16(eb + 0x16) as u32 + rd16(eb + 0x14) as u32
                };
                if r2 > 0x19 {
                    uv3 = r2 - 0x1a;
                }
                let txlen = 0x600a_54b8u32.wrapping_sub(slot.wrapping_mul(0x74));
                let cbw = core::ptr::read_volatile(
                    (param2 as u32).wrapping_add(ac.wrapping_mul(0x34)).wrapping_add(0x41)
                        as *const u8,
                ) as u32
                    & 3;
                wr(txlen, cbw << 0x16 | len | uv3 << 0x1c);
            }
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn mac_tx_set_plcp0(param_1: *mut u8) -> u32 {
    unsafe {
        let eb = core::ptr::read_unaligned(param_1 as *const u32);
        let u1 = rd_at(eb.wrapping_add(4));
        let txinfo = rd_at(eb.wrapping_add(0x34));
        let flags = rd_at(txinfo);
        let base = (u1 & 0xfffff) | 0x60_0000;
        let mut v = base;
        if (flags & 0x402) == 0 && (flags & 0x4048_0000) != 0x40_0000 {
            if (flags & 0x10_0000) == 0 {
                v = base | ((((flags >> 0x13) & 1) + 1) << 0x18);
                if (flags as i32) < 0 {
                    if (flags & 0x4000_0000) == 0 {
                        v = (u1 & 0xfffff) | 0x260_0000;
                    } else {
                        let iv: u32 = if (rd16(eb.wrapping_add(0x24)) & 0x1000) == 0 {
                            1
                        } else {
                            5
                        };
                        v = base | (iv << 0x18);
                    }
                }
            } else {
                v = (u1 & 0xfffff) | 0x360_0000;
            }
        }
        let slot = *param_1.add(4) as u32;
        wr(0x600a_4d6c_u32.wrapping_sub(slot.wrapping_mul(0x10)), v);
        // RTS/txop protection, exactly as the blob leaf is called.
        let enable = ((flags >> 8) & 1) as i32;
        let threshold = ((rd_at(txinfo.wrapping_add(0x30)) >> 3) & 0x3ff) as i32;
        let tval = rd_at(txinfo.wrapping_add(0x34));
        // RTS/txop protection via the Rust cr_hal_he_set_tx_protection (CONF0 bit31 + threshold reg).
        cr_hal_he_set_tx_protection(slot as i32, enable, 0, threshold, tval);
    }
    0
}

// ---- hal_mac_get_txq_complete (blocker #1, TX completion critical path) ----
// Called from lmacProcessTxComplete (and esp_test). Reads the per-slot completion/PMD/aux
// registers and fills two caller structs, then calls hal_mac_tx_clr_mplen.
//
// Decomp (verified 2026-09-11 against blob disasm @0x42078c76 + the caller lmacProcessTxComplete):
//   C ABI: hal_mac_get_txq_complete(int *param_1, int param_2, u8 *param_3, u32 *param_4) -> u32(0)
//     param_1 = g_ic.txq[ac] context (int*); *param_1 = eb (tx_frame); byte@0x28 bit2 = "skip".
//     param_2 = queue index q; every slot reg = base - q*0x74.
//     param_3 = 6-byte result struct (memset 6). param_4 = 8-byte aux/BA struct (memset 8, if !=0).
//   Registers (verified map): PMD 0x600a54e8, aux 54ec, aux 54e0/54d0/54f4 (all - q*0x74).
//
// param_3 (the completion result the caller dispatches on) -- legacy / non-HE-TB fill:
//   [0] = PMD & 0xff                    (sub-status)
//   [1] = ((PMD>>12)&0xf)<<4 | (PMD>>8)&0xf   (high nibble = STATE/match nibble the caller
//         switches a jump table on: value>5 -> lmac hangs; 0 = success; low nibble = error)
//   [2] = (PMD>>16)&0xff  [3] = (PMD>>25)&3  [5] = (aux54ec>>16)&0xff
// param_4 (aux/BA info, from 54e0/54d0/54f4):
//   [0] = ((54e0>>16)&0xf)<<28 | (54d0 & 0xfe000) | (54d0 & 0x100000) | ((54d0>>25)<<21)
//   [1] = (54e0>>20)&1 | (54e0>>20)&2 | ((54f4>>5)&0x1fc)
//   ((54d0 bit20) is the "last_tx_is_tb"/HE-TB flag -> selects the HE-TB param_3 fill below.)
// HE-TB fill (param_1!=0 && param_4!=0 && param_4[0] bit20 set): fills param_3 from aux54ec and
//   [4]=tb_sent. The muedca/HE-TB wifi_log diagnostics of the blob (is_use_muedca, context-struct
//   derefs) are SKIPPED -- pure logging, gated, and never taken by the legacy DSSS beacon.
// Tail: if param_1!=0: if (byte@param_1+0x28 & 4) return; else if eb!=0 clr_mplen(eb,q). All the
//   blob's "(complete)..." wifi_log branches are logging-only (gated off for the beacon) ->
// skipped.
//
// Minimal body (LESSON): each MMIO reg is read ONCE (the blob re-reads several) -> strictly fewer
// bus accesses than the blob, no atomics/instrumentation, so no extra TX-path traffic.
#[unsafe(no_mangle)]
pub extern "C" fn hal_mac_get_txq_complete(
    param_1: *mut i32,
    param_2: i32,
    param_3: *mut u8,
    param_4: *mut u32,
) -> u32 {
    let off = (param_2 as u32).wrapping_mul(0x74);
    unsafe {
        // memset(param_3, 0, 6)
        core::ptr::write_bytes(param_3, 0, 6);
        if !param_4.is_null() {
            // memset(param_4, 0, 8)
            core::ptr::write_bytes(param_4 as *mut u8, 0, 8);
            let r54e0 = rd(0x600a_54e0_u32.wrapping_sub(off));
            let r54d0 = rd(0x600a_54d0_u32.wrapping_sub(off));
            let r54f4 = rd(0x600a_54f4_u32.wrapping_sub(off));
            let w0 = ((r54e0 >> 0x10) << 0x1c)
                | (r54d0 & 0xfe000)
                | (r54d0 & 0x10_0000)
                | ((r54d0 >> 0x19) << 0x15);
            let w1 = ((r54e0 >> 0x14) & 2) | ((r54e0 >> 0x14) & 1) | ((r54f4 >> 5) & 0x1fc);
            *param_4 = w0;
            *param_4.add(1) = w1;
        }
        let pmd = rd(0x600a_54e8_u32.wrapping_sub(off));
        let aux = rd(0x600a_54ec_u32.wrapping_sub(off));
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
            if (*(param_1 as *const u8).add(0x28) & 4) != 0 {
                return 0;
            }
            if eb != 0 {
                hal_mac_tx_clr_mplen(eb, param_2);
            }
        }
    }
    0
}

// PLCP1 word: queue0 @ 0x600a5488, stride -0x74.
const WDEV_TXQ0_PLCP1: u32 = 0x600a_5488;

/// blob `mac_tx_set_plcp1`: program the PLCP1 word (rate/keyslot/HT-HE-mode/legacy-length/LDPC)
/// for the slot from the txinfo. Self-contained register I/O (no blob callees).
#[unsafe(no_mangle)]
pub extern "C" fn mac_tx_set_plcp1(param_1: *mut u8) -> u32 {
    unsafe {
        let eb = core::ptr::read_unaligned(param_1 as *const u32);
        let txinfo = rd_at(eb.wrapping_add(0x34));
        let lensrc = rd_at(rd_at(eb.wrapping_add(4)).wrapping_add(4));
        let rate = *((txinfo.wrapping_add(0xc)) as usize as *const u8) as u32;
        let flags = rd_at(txinfo);
        let uv3 = rate.wrapping_sub(0x10) & 0xff;
        let mut v: u32 = 0;
        if uv3 <= 0x13 {
            v = if (flags as i32) < 0 {
                0x400_0000
            } else {
                0x200_0000
            };
        }
        let keyslot = *((txinfo.wrapping_add(0x10)) as usize as *const u8) as u32;
        v = (v & 0xfe01_ffff) | (keyslot << 0x11);
        let ratefield = if rate > 0x28 { uv3 & 0x1f } else { rate & 0x1f };
        v = (v & 0xfffe_0fff) | (ratefield << 0xc);
        if uv3 > 0x13 {
            v = (v & 0xffff_f000) | (rd_at(lensrc) & 0xfff);
        }
        if (flags & 0x4000) != 0
            && (rd_at(txinfo.wrapping_add(0x10)) & 0x40000) != 0
            && (flags as i32) >= 0
        {
            v |= 0x2000_0000;
        }
        let slot = *param_1.add(4) as u32;
        wr(WDEV_TXQ0_PLCP1.wrapping_sub(slot.wrapping_mul(0x74)), v);
    }
    0
}

#[inline(always)]
fn txq_plcp0_enable_addr(ac: i32) -> *mut u32 {
    (0x600a_4d6c_i32.wrapping_sub(ac.wrapping_mul(0x10))) as usize as *mut u32
}

/// blob `hal_mac_txq_enable`: arm the slot (slot_valid|slot_enabled = 0xc000_0000).
/// The blob additionally does muedca bookkeeping (a byte-clear via GetAccess() and, for
/// HE-TB frames only, MU-EDCA state) — that is an HE feature irrelevant to legacy TX.
/// For safety we replicate only the arm here and delegate HE-TB frames to the blob.
#[unsafe(no_mangle)]
pub extern "C" fn hal_mac_txq_enable(ac: i32) {
    unsafe {
        let addr = txq_plcp0_enable_addr(ac);
        core::ptr::write_volatile(addr, core::ptr::read_volatile(addr) | 0xc000_0000);
    }
}

/// blob `hal_mac_txq_disable`: clear slot_valid|slot_enabled.
#[unsafe(no_mangle)]
pub extern "C" fn hal_mac_txq_disable(ac: i32) {
    unsafe {
        let addr = txq_plcp0_enable_addr(ac);
        core::ptr::write_volatile(addr, core::ptr::read_volatile(addr) & 0x3fff_ffff);
    }
}

// ---- raw MMIO helpers (deblob bin can't use esp-wifi-hal; esp-radio owns WIFI) ----
#[inline(always)]
unsafe fn rd(addr: u32) -> u32 {
    unsafe { core::ptr::read_volatile(addr as usize as *const u32) }
}
#[inline(always)]
unsafe fn wr(addr: u32, val: u32) {
    unsafe { core::ptr::write_volatile(addr as usize as *mut u32, val) }
}

// The WIFI MAC TX register file. Queue index maps to slots in REVERSE (queue0 = highest
// slot); per-queue address = base - q * stride. Verified against the blob disassembly.
// 0x74-stride block (completion/PMD/BA/txop): base 0x600a54xx, stride 0x74.
// 0x10-stride block (per-slot config): base 0x600a4d6x, stride 0x10.
const TXQ_PMD_Q0: u32 = 0x600a_54e8; // PMD/completion result
const TXQ_BA_BITMAP_Q0: u32 = 0x600a_54d4; // block-ack bitmap word0 (d8=word1, dc=header)
const TXQ_CONF1_Q0: u32 = 0x600a_4d68; // EDCA/AIFSN/CW/timeout
// Global TX state-machine registers (not per-queue).
const TXQ_STATE_A: u32 = 0x600a_4cb0;
const TXQ_STATE_B: u32 = 0x600a_4cb8;
const TXQ_CLR_STATE_A: u32 = 0x600a_4cac;
const TXQ_CLR_STATE_B: u32 = 0x600a_4cb4;
const TX_MIN_PWR: u32 = 0x600a_4400;

/// blob `hal_mac_rate_autoack_init`: empty in the blob (no-op).
#[unsafe(no_mangle)]
pub extern "C" fn hal_mac_rate_autoack_init() {}

/// blob `hal_mac_get_txq_state`: read the TX-queue state nibble for an AC.
/// (The blob's `cRam00000000`-gated esp_test/wifi_log debug branch is skipped.)
#[unsafe(no_mangle)]
pub extern "C" fn hal_mac_get_txq_state(ac: i32) -> u32 {
    let state = unsafe {
        match ac {
            1 => (rd(TXQ_STATE_A) >> 0x10) & 0xff,
            2 => rd(TXQ_STATE_B) & 0x7ff,
            0 => rd(TXQ_STATE_A) & 0x7ff,
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
            1 => wr(TXQ_CLR_STATE_A, 1u32 << ((bit.wrapping_add(0x10)) & 0x1f)),
            2 => wr(
                TXQ_CLR_STATE_B,
                rd(TXQ_CLR_STATE_B) | (1u32 << (bit & 0x1f)),
            ),
            0 => wr(TXQ_CLR_STATE_A, 1u32 << (bit & 0x1f)),
            _ => {}
        }
    }
    0
}

/// blob `hal_mac_tx_is_cbw40`: true if the slot is configured for 40 MHz (PMD bits 25-26).
#[unsafe(no_mangle)]
pub extern "C" fn hal_mac_tx_is_cbw40(q: i32) -> bool {
    let addr = TXQ_PMD_Q0.wrapping_sub((q as u32).wrapping_mul(0x74));
    unsafe { (rd(addr) >> 0x19) & 3 != 0 }
}

/// blob `hal_mac_tx_get_blockack`: copy the slot's block-ack bitmap registers into `out`.
/// `out` points to caller-provided storage: [+0]=u8 frag, [+2]=u16 seq, [+4]=u32, [+8]=u32.
#[unsafe(no_mangle)]
pub extern "C" fn hal_mac_tx_get_blockack(q: i32, out: *mut u8) -> u32 {
    let off = (q as u32).wrapping_mul(0x74);
    unsafe {
        let hdr = rd(TXQ_BA_BITMAP_Q0.wrapping_add(8).wrapping_sub(off)); // 0x600a54dc - off
        core::ptr::write_unaligned(out.add(2) as *mut u16, (hdr as u16) >> 4);
        *out = ((hdr >> 0x10) & 0xf) as u8;
        let w1 = rd(TXQ_BA_BITMAP_Q0.wrapping_add(4).wrapping_sub(off)); // 0x600a54d8 - off
        core::ptr::write_unaligned(out.add(4) as *mut u32, w1);
        let w0 = rd(TXQ_BA_BITMAP_Q0.wrapping_sub(off)); // 0x600a54d4 - off
        core::ptr::write_unaligned(out.add(8) as *mut u32, w0);
    }
    0
}

/// blob `hal_set_tx_min_pwr`: program the 6-bit TX minimum power field (bits 4-9).
#[unsafe(no_mangle)]
pub extern "C" fn hal_set_tx_min_pwr(pwr: u32) {
    unsafe {
        wr(
            TX_MIN_PWR,
            ((pwr & 0x3f) << 4) | (rd(TX_MIN_PWR) & 0xffff_fc0f),
        )
    }
}

// esp_test_is_disable_edca[]: per-AC test flag array in blob flash (memory-mapped, readable).
const ESP_TEST_IS_DISABLE_EDCA: u32 = 0x4208_318c;
// _LANCHOR0: baked TX-power table in blob flash (signed bytes, stride 2), read by hal_get_tx_pwr.
const TX_PWR_TABLE: u32 = 0x4208_3134;
// Real conf1 register (slot base + 4); clr_mplen's HE-TB mplen-valid bit is bit 3.
const TXQ_CONF1_REAL_Q0: u32 = 0x600a_4d64;

/// blob `hal_get_tx_pwr`: look up the signed max-TX-power byte for a rate index from the
/// baked flash table (indices > 0x19 are folded down by 0xa).
#[unsafe(no_mangle)]
pub extern "C" fn hal_get_tx_pwr(idx: u32) -> i32 {
    let i = if idx > 0x19 { idx - 0xa } else { idx };
    unsafe { *((TX_PWR_TABLE.wrapping_add(i.wrapping_mul(2))) as usize as *const i8) as i32 }
}

// PLCP rate/duration word: queue0 @ 0x600a54ac, stride -0x74.
const TXQ_PLCP_RATE_DUR_Q0: u32 = 0x600a_54ac;

#[inline(always)]
unsafe fn pwr_byte(off: u32) -> i32 {
    unsafe { *((TX_PWR_TABLE.wrapping_add(off)) as usize as *const i8) as i32 }
}

/// blob `hal_mac_tx_set_ppdu`: program the TX slot for the armed frame. Sets PLCP0/PLCP1
/// (via helpers), clears conf1 bit3, then writes the rate/duration word. Legacy (DSSS) frames
/// take the `mac_tx_set_len`/`mac_tx_set_txop_q` path; OFDM/HT/HE rates take the SIG path with
/// its HT/HE-only SIG helpers left to the blob (not exercised by legacy beacons).
/// `param_1` is the g_ic.txq[ac] context; `*param_1` = eb, eb+0x34 = txinfo.
#[unsafe(no_mangle)]
pub extern "C" fn hal_mac_tx_set_ppdu(param_1: *mut u8, param_2: i32) -> u32 {
    unsafe {
        let eb = core::ptr::read_unaligned(param_1 as *const u32);
        // (blob's `*(*(eb+4)+4) & 3` wifi_log debug branch omitted.)
        mac_tx_set_plcp0(param_1);
        mac_tx_set_plcp1(param_1);
        let slot = *param_1.add(4) as u32;
        let conf1 = TXQ_CONF1_REAL_Q0.wrapping_sub(slot.wrapping_mul(0x10));
        wr(conf1, rd(conf1) & 0xffff_fff7); // clear bit3
        let txinfo = rd_at(eb.wrapping_add(0x34));
        let rate = *((txinfo.wrapping_add(0xc)) as usize as *const u8);
        let rtsidx = cr_mac_tx_get_rts_rate(rate as i32) as u32;
        let s2 = (pwr_byte(rtsidx.wrapping_mul(2)) << 16)
            | (pwr_byte(rtsidx.wrapping_mul(2).wrapping_add(1)) << 24);
        let s3 = rate.wrapping_sub(0x10) as u32; // (rate-0x10)&0xff
        let plcp_rate_dur = TXQ_PLCP_RATE_DUR_Q0.wrapping_sub(slot.wrapping_mul(0x74));
        if s3 <= 0x13 {
            // OFDM/HT/HE rate. HT/HE SIG programming delegated to the blob leaves.
            let flags0 = rd_at(txinfo);
            if (flags0 as i32) < 0 {
                mac_tx_set_hesig();
            } else {
                mac_tx_set_htsig(param_1, param_2);
            }
            let r2 =
                *((rd_at(eb.wrapping_add(0x34)).wrapping_add(0xc)) as usize as *const u8) as u32;
            let idx = if r2 > 0x19 { r2 - 0xa } else { r2 };
            let c1 = pwr_byte(idx.wrapping_mul(2));
            let c2 = pwr_byte(idx.wrapping_mul(2).wrapping_add(1));
            wr(plcp_rate_dur, ((c2 << 8) | (c1 | s2)) as u32);
        } else {
            // DSSS / legacy path (our beacon).
            cr_mac_tx_set_len(param_1, param_2);
            mac_tx_set_txop_q(param_1);
            let r = *((txinfo.wrapping_add(0xc)) as usize as *const u8) as u32;
            wr(plcp_rate_dur, (pwr_byte(r.wrapping_mul(2)) | s2) as u32);
        }
        mac_tx_set_pti(param_1);
    }
    0
}

/// blob `hal_mac_tx_clr_mplen`: for legacy frames (conf1 bit3 clear) this is a no-op; the
/// HE-TB mplen-bitmap teardown (bit3 set) is delegated to the blob (TODO: reimplement when
/// HE-TB support is added — it touches _LANCHOR8/9 he bitmap state).
#[unsafe(no_mangle)]
pub extern "C" fn hal_mac_tx_clr_mplen(param1: i32, q: i32) {
    let conf1 = TXQ_CONF1_REAL_Q0.wrapping_sub((q as u32).wrapping_mul(0x10));
    unsafe {
        if rd(conf1) & 8 != 0 {
            blob::blob_hal_mac_tx_clr_mplen(param1, q);
        }
    }
}

#[inline(always)]
unsafe fn rd_at(addr: u32) -> u32 {
    unsafe { core::ptr::read_volatile(addr as usize as *const u32) }
}

/// blob `hal_mac_tx_config_edca`: program the slot's CONF1 EDCA parameters (AIFSN, CW,
/// iface bit) from the txq[ac] context. `txq` is the g_ic.txq[ac] entry.
#[unsafe(no_mangle)]
pub extern "C" fn hal_mac_tx_config_edca(txq: *mut u8) -> u32 {
    unsafe {
        let ac = *txq.add(4) as u32;
        let conf1 = TXQ_CONF1_Q0.wrapping_sub(ac.wrapping_mul(0x10));
        let aifsn = (*txq.add(5) & 0xf) as u32;
        wr(conf1, (rd(conf1) & 0xf0ff_ffff) | (aifsn << 24));
        let cw = (core::ptr::read_unaligned(txq.add(6) as *const u16) & 0x3ff) as u32;
        wr(conf1, (rd(conf1) & 0xffc0_0fff) | (cw << 12));
        // iface bit: txq[0]=eb ptr -> eb+0x34 = txinfo -> txinfo+0x10 word, bit 19.
        let eb = core::ptr::read_unaligned(txq as *const u32);
        let txinfo = rd_at(eb.wrapping_add(0x34));
        let ifb = (rd_at(txinfo.wrapping_add(0x10)) >> 0x13) & 1;
        wr(conf1, (rd(conf1) & 0xff3f_ffff) | (ifb << 0x16));
    }
    0
}

/// blob `hal_mac_tx_config_timeout`: program the slot's CONF1 low-12 lifetime/timeout field
/// from the txinfo (+0x40/+0x44), clamped and floored by `param2`. Delegates to the blob only
/// when the per-AC EDCA test bypass is active (never in normal operation).
#[unsafe(no_mangle)]
pub extern "C" fn hal_mac_tx_config_timeout(txq: *mut u8, param2: i32) -> u32 {
    unsafe {
        let ac = *txq.add(4) as u32;
        if *((ESP_TEST_IS_DISABLE_EDCA + ac) as usize as *const u8) != 0 {
            return blob::blob_hal_mac_tx_config_timeout(txq, param2);
        }
        let conf1 = TXQ_CONF1_Q0.wrapping_sub(ac.wrapping_mul(0x10));
        let eb = core::ptr::read_unaligned(txq as *const u32);
        let txinfo = rd_at(eb.wrapping_add(0x34));
        let u1 = rd_at(txinfo.wrapping_add(0x44));
        let mut v3 = (rd_at(txinfo.wrapping_add(0x40)) >> 10) | (u1 << 0x16);
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
    let addr = TXQ_PMD_Q0.wrapping_sub((q as u32).wrapping_mul(0x74));
    unsafe { *out = rd(addr) & 0xfeff_ffff }
    0
}

/// blob `hal_attenna_init`: reset per-slot antenna/PHY select fields across the RESP_DUR
/// slot block (0x600a54bc down to 0x600a511c, stride 0x74) plus the global reg 0x600a42cc.
#[unsafe(no_mangle)]
pub extern "C" fn hal_attenna_init() {
    unsafe {
        // Pass 1: clear low 3 bits of each slot's RESP_DUR reg.
        let mut a = 0x600a_54bc_u32;
        loop {
            wr(a, rd(a) & 0xffff_fff8);
            a = a.wrapping_sub(0x74);
            if a == 0x600a_511c {
                break;
            }
        }
        // Pass 2: clear bit3, set bit5, clear bit4 of each slot's RESP_DUR reg.
        let mut a = 0x600a_54bc_u32;
        loop {
            wr(a, rd(a) & 0xffff_fff7);
            wr(a, rd(a) | 0x20);
            wr(a, rd(a) & 0xffff_ffef);
            a = a.wrapping_sub(0x74);
            if a == 0x600a_511c {
                break;
            }
        }
        wr(0x600a_42cc, (rd(0x600a_42cc) & 0xffff_fff8) | 0x20);
    }
}

// ============================================================================
// CLEAN-ROOM Rust TX arm path (session 9).
// ============================================================================
// Goal: drive the full lmacTxFrame-equivalent ARM sequence from OUR OWN code on a
// DEDICATED slot, reusing the proven Rust hal_mac_tx functions above, rather than
// interposing into the blob's ppProcessTxQ->lmacTxFrame call graph (which failed).
//
// The blob control beacon (send_raw_frame, SSID CR-CTRL) still runs on the blob
// scheduler's slot (AC 0 per ppMapTxQueue) as a same-RF positive control. Our frame
// goes on AC/slot MY_AC. We obtain a correctly DMA-placed eb from the live libpp pool
// via esf_buf_alloc (resolves to libpp 0x4080acf4 in this link, NOT ROM), then fill
// the dma_desc + txinfo fields EXACTLY as ieee80211_output_raw_process does (minus the
// ppTxPkt submit), so our eb is structurally identical to a blob-built beacon eb. We
// then run: config_timeout -> set_ppdu (plcp0/1 + rate/dur/len/txop/pti) -> config_edca
// -> txq_enable(slot). We never touch our_instances, so the blob's lmacProcessTxComplete
// (guarded on state==1) skips our slot and never tries to recycle our eb.
// ============================================================================
// SECTION 3 — The cr_* pure-Rust TX pipeline
// ============================================================================
// Runs the blob's per-frame TX sequence in Rust on the REAL blob scheduler state:
//   cr_ppTxPkt (submit + enable-gate + AC map/PM-wake + enqueue onto TxRxCxt pending)
//   -> cr_ppProcessTxQ (pop) -> cr_lmacTxFrame/cr_lmacSetTxFrame (arm via the Rust hal)
//   -> cr_complete (poll completion, read result, recycle the eb).
// Fixed non-hardware addresses used here (ALL 0.3.0-blob-specific — re-derive per version):
//   our_instances base   = *(u32*)0x4004ffe0   (per-AC lmac txq, +ac*0x34; state @+0x12,
//                                                cur_eb @+0x00, cw @+0x08)
//   TxRxCxt pending base  = *(u32*)0x4087ff80   (per-AC list, +ac*0x34; head @+0x20, tail @+0x24,
//                                                threaded via eb+0x30)
//   wDevCtrl base         = *(u32*)0x4087ff68   (g_if_enabled_mask byte @ +0x31)
//   lmacConfMib           = 0x40811ca8          (RTS/long-frame threshold @ +0x16)
mod cleanroom_tx {
    use super::{rd, rd_at, wr};

    // The C6 MAC only accepts register writes to a TX slot whose bank the scheduler has
    // activated; only slot 0 (the raw beacon's AC 0) is live. So we drive slot 0 ourselves,
    // time-multiplexed with a quiesced blob beacon.
    pub const MY_AC: i32 = 0;

    /// Opt-in diagnostics. `DIAG` gates the boot slot-writability probe and the periodic [CR.H]
    /// health / [CR.F] progress prints (out-of-band, never on the hot TX path). `CONTROL_BEACON`
    /// gates the blob-scheduled CR-CTRL beacon that runs alongside CR-RUST as a same-RF reference.
    /// Both default true so the reference build behaves exactly as verified. The sustained TX path
    /// (alloc -> faithful_tx -> cr_complete) carries no diagnostics regardless.
    pub const DIAG: bool = true;
    pub const CONTROL_BEACON: bool = true;

    // The minimal blob surface the live TX path still calls. Everything else on the per-frame path
    // (submit/map/pop/enqueue/schedule/arm/complete + the hal register ops) is Rust below.
    unsafe extern "C" {
        // esf_buf pool: a callback-mediated two-list eb pool (alloc pops one head, recycle pushes
        // another; the OSI callbacks reconcile them). type 1 = static TX. See docs for why it stays.
        fn esf_buf_alloc(payload: *const u8, pool_type: i32, len: u32) -> u32;
        fn esf_buf_recycle(eb: u32);
        fn hal_now() -> u32; // WDEV TSF timer read (submit timestamp / EDCA backoff seed)
        // modem-PM wake FSM: clears the WDEV_PM_TXBLOCK_RETENTION interlock so the MAC goes active
        // (a hardware consequence of the g_pm FSM + wifi_rf_phy_enable, not a single register write).
        fn pm_on_data_tx(iface: u32, p2: i32) -> i32;
        // security-header handling: dual-descriptor (eb+8 vs eb+4) length adjustment, required for
        // reliable emit (a no-op regresses the on-air frame).
        fn ppProcTxSecFrame(eb: u32) -> i32;
    }

    /// Rust ppTxPkt (submit), kick=0. Gate on the per-VIF enable, run the blob security-header leaf,
    /// map the AC (which runs pm_on_data_tx, the PM-wake), then enqueue the eb onto the REAL TxRxCxt
    /// per-AC pending list (threaded via eb+0x30). Returns the blob convention: 0 = enqueued,
    /// 1 = dropped+recycled. On any drop path the eb is recycled here, so the caller must NOT also
    /// recycle it (doing both is a double-free -- the bug that the wrong ic_interface_enabled masked).
    pub fn cr_ppTxPkt(eb: u32, _kick: i32) -> i32 {
        unsafe {
            let txinfo = rd_at(eb + 0x34);
            let iface = (rd(txinfo + 0x10) >> 0x13) & 1;
            if cr_ic_interface_enabled(iface) == 0 {
                esf_buf_recycle(eb);
                return 1;
            }
            cr_ppTxProtoProc(eb);
            if ppProcTxSecFrame(eb) == 1 {
                esf_buf_recycle(eb);
                return 1;
            }
            cr_rcGetSched(rd(eb + 0x2c), rd_at(eb + 0x34)); // trc==0 -> no-op for raw beacon
            let map = cr_ppMapTxQueue(eb); // Rust AC mapping; keeps blob pm_on_data_tx (PM-wake)
            if map == 0 {
                let ti = rd_at(eb + 0x34);
                wr(ti + 0x18, hal_now()); // tsf submit stamp (blob: _WDEV_TSF0_TIMER_LO)
                // Pending-list base is pTxRx (TxRxCxt), *(0x4087ff80) -- per-AC lists at +ac*0x34,
                // head +0x20 / tail +0x24 (threaded via eb+0x30). Verified from the linked ppTxPkt
                // disasm (`lui 0x40880; lw -0x80` = *(0x4087ff80)). NOT our_instances (0x4087f840).
                let pend = rd(0x4087_ff80);
                let ac = (rd(ti + 0x10) >> 0x14) & 0xf;
                let q = pend.wrapping_add(ac.wrapping_mul(0x34));
                wr(eb + 0x30, 0); // next-link = NULL
                let tail = rd(q + 0x24); // pending tail = pointer to the slot to fill
                wr(tail, eb); // *(tail) = eb
                wr(q + 0x24, eb + 0x30); // tail = &eb.next
                // kick==0: the blob would pp_post(ac) here if idle; we drive the schedule
                // ourselves.
                0
            } else {
                esf_buf_recycle(eb); // map fail / deferred-to-hmac not expected for our beacon
                1
            }
        }
    }

    /// Rust ppTxProtoProc for the legacy beacon: reads the on-air FC byte and sets the protocol
    /// flags. For a broadcast mgmt beacon (FC 0x80, addr1[0]&1) only txinfo bit1 (0x2) is set; the
    /// data (FC&0xc==8) and null/QoS (FC&0xf0 0x50/0x40) branches are faithfully replicated but not
    /// taken by the beacon.
    pub fn cr_ppTxProtoProc(eb: u32) {
        unsafe {
            let mut frame = rd_at(rd_at(eb + 4) + 4); // dma_desc[1]
            if (core::ptr::read_volatile((eb + 0x24) as *const u16) & 0x2000) != 0 {
                frame += 8; // FTM offset (not our beacon)
            }
            let txinfo = rd_at(eb + 0x34);
            if (core::ptr::read_volatile((frame + 4) as *const u8) & 1) != 0 {
                wr(txinfo, rd(txinfo) | 2); // broadcast/multicast -> no-ack
            }
            let fc = core::ptr::read_volatile(frame as *const u8);
            if (fc & 0xc) == 8 {
                let f = rd(txinfo);
                wr(txinfo, f | 8);
                if (rd(txinfo + 0x30) & 0x2_0000) == 0 && (fc & 0x70) == 0x40 {
                    wr(txinfo, rd(txinfo) & 0xffff_fff7);
                }
            } else if (fc & 0xc) == 0 {
                if (fc & 0xf0) == 0x50 {
                    if (rd(txinfo) & 2) == 0 {
                        wr(txinfo, rd(txinfo) | 0x800_0000);
                    }
                } else if (fc & 0xf0) == 0x40 && (rd(txinfo) & 2) == 0 {
                    wr(txinfo, rd(txinfo) | 0x800);
                }
            }
        }
    }

    /// Rust ppMapTxQueue for the legacy beacon: choose the EDCA AC and write it into txinfo+0x10
    /// bits20-23, keeping the blob pm_on_data_tx (the PM-wake that makes the MAC active -- the
    /// breakthrough ingredient) and ppProcessWaitingQueue (hmac drain). For our raw beacon trc==0,
    /// so it takes the simple branch: txinfo+4=7, AC=iface. The QoS-data/TWT branches
    /// (ppSearchTxQueue / pm_on_twt_force_tx) are not exercised by the beacon and are omitted.
    /// Returns the blob convention: 0 = mapped.
    /// ic_interface_enabled: the per-VIF enable check that gates ppTxPkt, reimplemented in Rust as a
    /// faithful port of the ROM function (which the blob's ppTxPkt jalrs). Validated against the ROM
    /// at runtime: rust(0)==rom(0)==1, rust(1)==rom(1)==0. A prior reimpl guessed the mask byte at the
    /// STATIC wDevCtrl+0x29 (0x408120b9); both the base (must come from the pointer, runtime 0x40812050
    /// not the static 0x40812090) and the offset (+0x31, not +0x29) were wrong, so it read 0 on 0.3.0.
    pub fn cr_ic_interface_enabled(iface: u32) -> i32 {
        // Faithful port of the ROM ic_interface_enabled (0x40012c34, the one the blob's ppTxPkt
        // jalrs): base = *(wDevCtrl_ptr @ 0x4087ff68); mask = byte at base+0x31 (g_if_enabled_mask);
        // return (mask >> iface) & 1. The earlier reimpl used the STATIC wDevCtrl (0x40812090) plus a
        // wrong offset 0x29 (= 0x408120b9), which reads 0 on the 0.3.0 blob -> the double-free bug.
        // Both the base-via-pointer and the +0x31 offset are verified against the 0.3.0 ROM disasm.
        unsafe {
            let wdevctrl = rd(0x4087_ff68);
            let mask = core::ptr::read_volatile((wdevctrl + 0x31) as *const u8) as u32;
            ((mask >> (iface & 0x1f)) & 1) as i32
        }
    }

    /// Rust lmacIsLongFrame: MPDU length vs the RTS/long-frame threshold (lmacConfMib+0x16 on the
    /// 0.3.0 blob, lmacConfMib=0x40811ca8). For our short broadcast beacon this is false; and in
    /// cr_lmacTxFrame the RTS it would gate is additionally suppressed by txinfo bit1 (broadcast),
    /// so the result is not on the beacon's critical path.
    pub fn cr_lmacIsLongFrame(eb: u32) -> i32 {
        unsafe {
            let threshold = core::ptr::read_volatile((0x4081_1ca8u32 + 0x16) as *const u16) as i32;
            let len = core::ptr::read_volatile((eb + 0x14) as *const u16) as i32
                + core::ptr::read_volatile((eb + 0x16) as *const u16) as i32;
            (threshold < len) as i32
        }
    }

    pub static PM_BLK: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
    /// Crown-jewel test: when false, skip the blob pm_on_data_tx entirely (Rust no-op) to see if the
    /// MAC stays active without it in our sustained-TX steady state.
    pub const PM_ON_DATA_TX: bool = true;
    pub fn cr_ppMapTxQueue(eb: u32) -> i32 {
        unsafe {
            // ppProcessWaitingQueue drains the per-iface hmac WAITING queue (frames deferred to the
            // mgmt queue). Our beacon is submitted straight to the pending list, so there is nothing
            // to drain -> Rust no-op on our path (proven: skipping it radiates with a healthy pool).
            let txinfo = rd_at(eb + 0x34);
            let t4 = rd(txinfo + 4);
            if (t4 & 0xf0) == 0x40 {
                wr(txinfo + 0x10, (rd(txinfo + 0x10) & 0xff0f_ffff) | 0x20_0000);
            } else {
                let iface = (rd(txinfo + 0x10) >> 0x13) & 1;
                let trc = rd(eb + 0x2c);
                if trc == 0 || (core::ptr::read_volatile((trc + 0xc) as *const u16) & 0x80) != 0 {
                    core::ptr::write_volatile((txinfo + 4) as *mut u8, 7);
                    wr(
                        txinfo + 0x10,
                        (rd(txinfo + 0x10) & 0xff0f_ffff) | (iface << 0x14),
                    );
                    let blk_pre = rd(0x600a_4ca8);
                    if PM_ON_DATA_TX {
                        pm_on_data_tx(iface, 0); // PM-wake (blob)
                    }
                    let blk_post = rd(0x600a_4ca8);
                    // pack: high byte-ish of pre and post so we see the 0xff1000/0x2000 region
                    PM_BLK.store(((blk_pre >> 8) << 16) | ((blk_post >> 8) & 0xffff), core::sync::atomic::Ordering::Relaxed);
                }
                // (QoS-data / TWT mapping branches not exercised by the legacy beacon.)
            }
            0
        }
    }

    /// Rust ppGetTxframe: pop the head eb from the per-AC pending list in TxRxCxt (*0x4087ff80),
    /// faithful to the blob (head +0x20, tail +0x24, threaded via eb+0x30; empty -> tail=&head),
    /// with the blob's guard (+0x29==0 && +0x34==0) and the lmacAdjustTimestamp fixup. Our single
    /// beacon is enqueued to this AC, so this directly dequeues it (the blob's multi-queue
    /// ppSearchTxframe selection/bitmap is unnecessary for our controlled single-AC submit).
    pub fn cr_ppGetTxframe(ac: i32) -> u32 {
        unsafe {
            let base = rd(0x4087_ff80);
            let q = base.wrapping_add((ac as u32).wrapping_mul(0x34));
            if core::ptr::read_volatile((q + 0x29) as *const u8) == 0 && rd(q + 0x34) == 0 {
                let head = rd(q + 0x20);
                if head != 0 {
                    let next = rd(head + 0x30);
                    wr(q + 0x20, next);
                    if next == 0 {
                        wr(q + 0x24, q + 0x20); // empty -> tail = &head
                    }
                    wr(head + 0x30, 0);
                    // (blob calls lmacAdjustTimestamp() here -- a beacon-timestamp fixup that
                    // derefs an AP/beacon context null in our raw path; our
                    // beacon uses timestamp=0 so it is unnecessary and
                    // omitted.)
                    return head;
                }
            }
            0
        }
    }

    /// Rust completion (PART B): the lmacProcessTxComplete + lmacTxDone essentials, driven from our
    /// own loop (NOT the blob ISR / NOT symbol interposition). The MAC clears PLCP0_ENABLE's arm
    /// bits when the TX finishes; we poll that (bounded, out-of-band), read the completion
    /// result via our Rust hal_mac_get_txq_complete, clear the txq_state bit, disarm the slot,
    /// and recycle the eb (esf_buf_recycle -- the pool allocator stays blob). Returns
    /// (completed, status_nibble).
    pub fn cr_complete(ac: i32, eb: u32) -> (bool, u8) {
        unsafe {
            let a = 0x600a_4d6c - (ac as u32) * 0x10;
            let mut done = false;
            for _ in 0..4000 {
                if (rd(a) & 0xc000_0000) == 0 {
                    done = true;
                    break;
                }
            }
            // lmacProcessTxComplete-equivalent read of the completion result for our AC.
            let txq = rd(0x4004_ffe0).wrapping_add((ac as u32).wrapping_mul(0x34));
            let mut res6 = [0u8; 8];
            let mut aux8 = [0u32; 2];
            super::hal_mac_get_txq_complete(
                txq as *mut i32,
                ac,
                res6.as_mut_ptr(),
                aux8.as_mut_ptr(),
            );
            let status = (res6[1] >> 4) & 0xf; // (pmd>>12)&0xf : 0=success
            super::hal_mac_clr_txq_state(2, ac as u32); // clear completed-state bit (as the blob does)
            // NOTE: do NOT hal_mac_txq_disable here -- the MAC auto-clears the arm bits on
            // completion; forcing a disable leaves slot 0 in a state that breaks the shared control
            // beacon (AC 0). The blob completion never disables the slot.
            // eb ownership: with cr_ic_interface_enabled fixed (ROM call), cr_ppTxPkt no longer
            // takes its drop-path (which recycled the eb), so this is the SINGLE recycle of the eb
            // -- one alloc_eb per iteration, one recycle here. (Before the fix, cr_ppTxPkt's wrong
            // interface-disabled path recycled the eb first, and this line double-freed it, which
            // corrupted the LLA heap backing the esf_buf pool -> "hole list out of order".) Verified
            // over sustained runs: recycles==arms, dblfree==0, allocfail==0.
            let _txq = txq;
            esf_buf_recycle(eb);
            (done, status)
        }
    }

    /// Rust hal_random: the blob's is just g_wifi_osi_funcs._rand(); we only use it for the EDCA
    /// backoff (masked to the CW window), so a self-contained xorshift PRNG is equivalent.
    pub static CR_RNG: core::sync::atomic::AtomicU32 =
        core::sync::atomic::AtomicU32::new(0x1234_5678);
    pub fn cr_hal_random() -> u32 {
        let mut x = CR_RNG.load(core::sync::atomic::Ordering::Relaxed);
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        CR_RNG.store(x, core::sync::atomic::Ordering::Relaxed);
        x
    }

    /// Rust rcGetSched: for raw frames trc==NULL the blob returns immediately (rate comes from the
    /// descriptor). Our beacon always has trc==0, so this is a no-op. (trc!=0 rate-control
    /// selection is not exercised by the legacy beacon and stays unimplemented.)
    pub fn cr_rcGetSched(trc: u32, _txinfo: u32) {
        if trc == 0 {
            return;
        }
    }

    /// Rust reimplementation of lmacTxFrame (the ARM) for the legacy DSSS beacon, on the REAL
    /// our_instances[ac] state. lmacSetTxFrame (PPDU build) stays the blob shim, which itself
    /// routes through our proven Rust hal (config_timeout/set_ppdu). The arm sequencing +
    /// real-state bookkeeping (cur_eb, random backoff, config_edca, state=ARMED, txq_enable) is
    /// Rust here.
    /// Rust lmacSetTxFrame (PPDU build) for the raw beacon. The TXOP-queue request + TSF-lifetime
    /// sequencing is reduced to the beacon path: no aggregation (trc==0), and a fixed lifetime (the
    /// timeout is not the radiate gate). The actual slot programming goes through our Rust hal
    /// (hal_mac_tx_config_timeout + hal_mac_tx_set_ppdu, both already Rust).
    pub fn cr_lmacSetTxFrame(txq: u32) {
        unsafe {
            let _cur_eb = rd(txq); // mode 0 -> eb = *txq (cur_eb)
            let ours = rd(0x4004_ffe0); // our_instances base (hal_mac_tx_set_ppdu param2)
            super::hal_mac_tx_config_timeout(txq as *mut u8, 0x7ff);
            super::hal_mac_tx_set_ppdu(txq as *mut u8, ours as i32);
        }
    }

    pub fn cr_lmacTxFrame(eb: u32, ac: i32) {
        unsafe {
            let base = rd(0x4004_ffe0);
            let txq = base.wrapping_add((ac as u32).wrapping_mul(0x34));
            let txinfo = rd_at(eb + 0x34);
            let flags = rd(txinfo);
            // Discard path (txinfo bit16 & !offchan) is not taken by a normal beacon -> omitted.
            let state = core::ptr::read_volatile((txq + 0x12) as *const u8);
            if state == 0 || state == 3 {
                core::ptr::write_volatile(txq as *mut u32, eb); // cur_eb
                if (flags & 0x2102) == 0x2000 {
                    wr(txinfo, flags | 0x1000);
                }
                // long-frame -> RTS (beacon is short; lmacIsLongFrame returns 0 -> no-op, but
                // faithful)
                if cr_lmacIsLongFrame(eb) != 0 && (rd(txinfo) & 2) == 0 {
                    wr(txinfo, (rd(txinfo) & 0xffff_efff) | 0x100);
                }
                // state==3 retry-RTS and FTM (0x20000000) branches skipped (not taken by the
                // beacon).
                cr_lmacSetTxFrame(txq); // PPDU build (Rust; slot programming via our Rust hal)
            }
            // EDCA random backoff masked by CW exponent at txq+8, exactly as the blob.
            let r = cr_hal_random();
            let cw = core::ptr::read_volatile((txq + 8) as *const u8) as u32;
            let backoff = (!(0xffff_ffffu32 << (cw & 0x1f)) & r) as u16;
            core::ptr::write_unaligned((txq + 6) as *mut u16, backoff);
            super::hal_mac_tx_config_edca(txq as *mut u8);
            // NOTE: we deliberately do NOT set our_instances[ac].state(+0x12)=1 here. With state!=1
            // the blob MAC ISR's lmacProcessTxComplete skips our AC (only clears the completed
            // bit), so it does NOT recycle our eb -- our Rust cr_complete() owns the
            // completion + recycle.
            super::hal_mac_txq_enable(core::ptr::read_volatile((txq + 4) as *const u8) as i32);
        }
    }

    /// Rust reimplementation of ppProcessTxQ for the legacy DSSS beacon path, operating on the REAL
    /// our_instances[ac] state. Called DIRECTLY by faithful_tx (NOT symbol interposition), so the
    /// lmac placement/timing wall does not apply. Leaf helpers (ppSearchTxframe/pp_coex_tx_request)
    /// stay blob for now.
    pub fn cr_ppProcessTxQ(ac: i32) -> i32 {
        unsafe {
            let base = rd(0x4004_ffe0);
            let txq = base.wrapping_add((ac as u32).wrapping_mul(0x34));
            // lmacIsIdle(ac): our_instances[ac].state(+0x12) must be 0 (idle). pm/twt/mesh guards
            // are permissive for a non-connected beacon-only build -> skipped.
            if core::ptr::read_volatile((txq + 0x12) as *const u8) != 0 {
                return -1;
            }
            let eb = cr_ppGetTxframe(ac); // Rust pop from the real TxRxCxt pending list
            if eb == 0 {
                return -2;
            }
            // Legacy DSSS beacon: txinfo flags have no HE(bit31)/AMPDU(0x400000)/0x1040000 bit and
            // trc(eb+0x2c)==0, so the blob's AMPDU-reorder and RTS/fragment branches are NOT taken
            // (verified against the decompile) -> go straight to the coex request + arm.
            // pp_coex_tx_request is a coex-signaling no-op on our path (proven: skipping it keeps the
            // MAC waking, latching and radiating with a healthy pool) -> reimplemented as a Rust no-op.
            let _ = eb; // (coex request would classify the frame + call the coex OSI cbs; none needed)
            cr_lmacTxFrame(eb, ac);
            0
        }
    }

    /// FAITHFUL submit+schedule+arm on REAL scheduler state, from our task (blob pp idle).
    /// Returns (ac, txpkt_ret, blk_before, blk_after, plcp0_before, plcp0_after).
    pub fn faithful_tx(eb: u32, seq: u16) -> (i32, i32, u32, u32, u32, u32) {
        unsafe {
            // Mirror ieee80211_output_raw_process's eb/dma/txinfo setup BEFORE ppTxPkt (the fields
            // the blob's raw-submit fills), then let the blob's real ppTxPkt do proto/sec/rate/map/
            // enqueue with kick=0. ppMapTxQueue will set the AC; do NOT pre-set AC bits here.
            let dma = rd_at(eb + 4);
            let frame = rd_at(dma + 4);
            core::ptr::write_volatile((eb + 0x14) as *mut u16, 0);
            let l16 = core::ptr::read_volatile((eb + 0x16) as *const u16) as u32;
            let mut w0 = rd(dma);
            w0 |= 0x8000_0000;
            w0 |= 0x4000_0000;
            w0 &= 0xdfff_ffff;
            w0 = ((l16 & 0x3fff) << 0xe) | (w0 & 0xf000_3fff);
            wr(dma, w0);
            let txinfo = rd_at(eb + 0x34);
            core::ptr::write_volatile((txinfo + 4) as *mut u8, 7); // cat = mgmt
            wr(txinfo + 0x18, hal_now());
            let mut w10 = rd(txinfo + 0x10);
            w10 &= 0xfff7_ffff; // iface 0
            wr(txinfo + 0x10, w10);
            if (core::ptr::read_volatile((frame + 4) as *const u8) & 1) != 0 {
                wr(txinfo, rd(txinfo) | 0x402);
            }
            core::ptr::write_volatile((txinfo + 0xc) as *mut u8, 0); // rate 1M DSSS
            core::ptr::write_volatile((frame + 0x16) as *mut u16, seq << 4); // seq ctrl
            wr(eb + 0x2c, 0); // trc = NULL (raw frame -> rcGetSched no-op)

            // Faithful submit onto REAL our_instances pending (no kick).
            let ret = cr_ppTxPkt(eb, 0);
            // AC that ppMapTxQueue assigned into txinfo+0x10 bits 20-23.
            let ac = ((rd(txinfo + 0x10) >> 0x14) & 0xf) as i32;
            let blk_before = rd(0x600a_4ca8);
            let plcp0_before = rd(0x600a_4d6c - (ac as u32) * 0x10);
            // Faithful schedule + arm on real state (pop + coex + lmacTxFrame -> our Rust hal).
            let _ = cr_ppProcessTxQ(ac);
            let plcp0_after = rd(0x600a_4d6c - (ac as u32) * 0x10);
            let blk_after = rd(0x600a_4ca8);
            (ac, ret, blk_before, blk_after, plcp0_before, plcp0_after)
        }
    }

    /// Allocate ONE eb from the live static-TX pool with our beacon payload copied in.
    pub fn alloc_eb(payload: &[u8]) -> u32 {
        unsafe { esf_buf_alloc(payload.as_ptr(), 1, payload.len() as u32) }
    }

    /// One-time write/read probe across all 8 slots: which slot config banks are writable?
    /// (Writes to slots whose bank the scheduler has not activated are dropped on the C6 MAC.)
    pub fn probe() {
        unsafe {
            for s in 0..8u32 {
                let a = 0x600a_4d6c - s * 0x10;
                let e = 0x600a_4d68 - s * 0x10;
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
                super::println!(
                    "[CR.probe] slot{s} plcp0={p0:#010x} edca={e0:#010x} writable={writable}"
                );
            }
        }
    }
}

// ============================================================================
// SECTION 4 — Beacon payloads
// ============================================================================
const SSID_CTRL: &str = "CR-CTRL";
const SSID_RUST: &str = "CR-RUST";
const MAC_CTRL: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0xda, 0xb0];
const MAC_RUST: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0xc6, 0x00];

fn build_beacon(buf: &mut [u8], ssid: &str, mac: [u8; 6]) -> usize {
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

// ============================================================================
// SECTION 5 — main: bring-up + the sustained pure-Rust TX loop
// ============================================================================
#[esp_hal::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

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
    let ctrl_len = build_beacon(&mut ctrl_buf, SSID_CTRL, MAC_CTRL);
    let ctrl = &ctrl_buf[..ctrl_len];

    let mut rust_buf = [0u8; 300];
    let rust_len = build_beacon(&mut rust_buf, SSID_RUST, MAC_RUST);

    println!(
        "[CR] booting; control SSID '{SSID_CTRL}', Rust-arm SSID '{SSID_RUST}' slot AC{}",
        cleanroom_tx::MY_AC
    );

    // Let the blob bring-up settle and prove RF with a few control beacons first.
    if cleanroom_tx::CONTROL_BEACON {
        for _ in 0..10 {
            let _ = sniffer.send_raw_frame(true, ctrl, false);
            delay.delay(Duration::from_millis(100));
        }
    }

    // Which TX slot config banks the scheduler has activated (diagnostic).
    if cleanroom_tx::DIAG {
        cleanroom_tx::probe();
    }

    // Sustained pure-Rust TX: each round allocates a fresh eb, runs the full Rust submit->schedule
    // ->arm on the REAL blob scheduler state (blob pp idle), then Rust completion+recycle. The
    // [CR.H] oracle (out-of-band) should read arms==latched==completed, allocfail==0, and
    // last_plcp0==0xc067a6f0 (our frame armed on slot 0).
    println!("[CR.F] pure-Rust TX pipeline on real scheduler state (submit/schedule/arm/complete)");
    let mut round: u32 = 0;
    let mut seq: u16 = 0;
    let (mut n_arm, mut n_latch, mut n_done, mut n_allocfail) = (0u32, 0u32, 0u32, 0u32);
    let mut last_pa: u32 = 0;
    loop {
        // ----- TX phase: one full submit+schedule+arm+complete per iteration (no send_raw_frame).
        for _ in 0..4u32 {
            let feb = cleanroom_tx::alloc_eb(&rust_buf[..rust_len]);
            if feb == 0 {
                n_allocfail += 1;
            } else {
                let (ac, _ret, _bb, _ba, _pb, pa) = cleanroom_tx::faithful_tx(feb, seq);
                seq = seq.wrapping_add(1);
                n_arm += 1;
                if pa & 0xc000_0000 != 0 {
                    n_latch += 1;
                }
                last_pa = pa;
                delay.delay(Duration::from_millis(40)); // let the frame win the medium + TX + complete
                // Rust-driven completion + recycle (single owner of the eb).
                let (completed, _status) = cleanroom_tx::cr_complete(ac, feb);
                if completed {
                    n_done += 1;
                }
            }
            delay.delay(Duration::from_millis(60));
        }

        // ----- Control phase: a few blob-scheduled CR-CTRL beacons as a same-RF reference.
        if cleanroom_tx::CONTROL_BEACON {
            for _ in 0..6 {
                let _ = sniffer.send_raw_frame(true, ctrl, false);
                delay.delay(Duration::from_millis(100));
            }
        }
        round += 1;
        // Periodic health + oracle summary (out-of-band; catchable in any monitor window).
        if cleanroom_tx::DIAG && round % 8 == 0 {
            let pmblk = cleanroom_tx::PM_BLK.load(core::sync::atomic::Ordering::Relaxed);
            println!(
                "[CR.H] rounds={round} arms={n_arm} latched={n_latch} completed={n_done} allocfail={n_allocfail} last_plcp0={last_pa:#010x} pm_blk={:#06x}->{:#06x}",
                pmblk >> 16, pmblk & 0xffff
            );
        }
        if cleanroom_tx::DIAG && round % 5 == 0 {
            println!("[CR.F] completed {round} rounds");
        }
    }
}
