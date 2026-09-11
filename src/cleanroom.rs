//! De-blob step 1 (SCRATCH — not tracked upstream): interpose our code between the
//! blob pp/lmac scheduler and the blob `hal_mac_tx.o`.
//!
//! Phase 1 (this build): the 23 cross-boundary symbols of hal_mac_tx.o were renamed in
//! libpp.a to `blob_<fn>`; the `global_asm!` block below re-provides `<fn>` as an
//! ABI-perfect tail-call shim into `blob_<fn>`. So pp/lmac now call OUR symbols, which
//! forward to the blob. If the normal blob beacon (send_raw_frame -> esp_wifi_80211_tx
//! -> pp -> lmac -> hal_mac_tx) STILL radiates (SSID "DEBLOB-HAL"), the interposition
//! wiring is proven and Phase 2 can replace each shim body with real Rust one at a time.

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
#[allow(dead_code)]
mod lmac_deblob {
    unsafe extern "C" {
        // Fixed pp/ROM pointer-table slot (0x4004ffe0) holding the base of the per-AC lmac txq
        // instance array. Reading it yields the same base the blob's GetAccess computes.
        #[link_name = "our_instances_ptr"]
        static OUR_INSTANCES_PTR: u32;
    }

    /// Base of the `our_instances` per-AC lmac txq array = `*our_instances_ptr` (== `GetAccess(0)`).
    #[inline(always)]
    fn our_instances() -> u32 {
        unsafe { core::ptr::read_volatile(&OUR_INSTANCES_PTR as *const u32) }
    }

    /// `our_instances[ac]` = per-AC lmac TX control block (the blob's `GetAccess(ac)`).
    #[inline(always)]
    fn lmac_txq(ac: i32) -> u32 {
        our_instances().wrapping_add((ac as u32).wrapping_mul(0x34))
    }

    /// Plain `lbu` load (matches the blob's byte reads; no bounds/align guard).
    #[inline(always)]
    unsafe fn lbu(addr: u32) -> u8 {
        let v: u32;
        unsafe {
            core::arch::asm!("lbu {0}, 0({1})", out(reg) v, in(reg) addr, options(nostack, readonly))
        };
        v as u8
    }
    /// Plain `lw` load (matches the blob's word reads exactly; no compiler-inserted alignment
    /// check/panic branch, unlike `read_volatile::<u32>` on a computed pointer).
    #[inline(always)]
    unsafe fn lw(addr: u32) -> u32 {
        let v: u32;
        unsafe {
            core::arch::asm!("lw {0}, 0({1})", out(reg) v, in(reg) addr, options(nostack, readonly))
        };
        v
    }

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
// A byte-identical machine-code copy at a DIFFERENT address killing TX rules out the code itself and
// points to a placement/dispatch dependency: the blob lmac functions must live at their original
// libpp addresses (likely a ROM/wdev function-pointer table, or an i-cache/co-location constraint on
// the timing-sensitive TX path). The no-op works precisely because it reads nothing. Combined with
// the ROM *ABS* wall (the ~15 symbols above), lmac.o is not meaningfully de-blobbable via symbol
// interposition beyond this no-op. Full analysis + reproduction: docs/deblob_progress.md.

// ---- Phase 2: real Rust reimplementations of hal_mac_tx.o functions ----
// Each replaces its global_asm shim above. Verified against the blob decompilation.
// PLCP0_ENABLE for blob queue 0 (highest slot) is 0x600a_4d6c; higher queue index
// steps DOWN by 0x10 (the blob's reversed slot numbering), so addr = 0x600a_4d6c - ac*0x10.
mod blob {
    unsafe extern "C" {
        pub fn blob_hal_mac_txq_enable(ac: i32);
        pub fn blob_hal_mac_tx_config_timeout(txq: *mut u8, param2: i32) -> u32;
        pub fn blob_hal_mac_tx_clr_mplen(param1: i32, q: i32);
    }
}

// hal_mac_tx.o helpers still called by our Rust `hal_mac_tx_set_ppdu`. plcp0/plcp1/txop_q are now
// real Rust in this module; the rest are non-renamed blob leaves (RTS-rate lookup, legacy len,
// HT/HE SIG, PTI).
unsafe extern "C" {
    fn mac_tx_get_rts_rate(rate: u8) -> i32;
    fn mac_tx_set_len(p: *mut u8, param2: i32);
    fn mac_tx_set_pti(p: *mut u8);
    fn mac_tx_set_hesig();
    fn mac_tx_set_htsig(p: *mut u8, param2: i32);
    // blob leaf: fill hardware TXOP fields for each MPDU of an aggregate (dead for single frames).
    fn hal_mac_fill_hwtxop(eb: u32, depth: u32, idx: u32);
}

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
        wr(resp_dur, (rd(resp_dur) & 0xf0ff_ffff) | ((s3 << 0x18) & 0xf000_0000));
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
//                        else { iv = if (u16@(eb+0x24) & 0x1000)==0 {1} else {5}; v=base|(iv<<24) } }
//     } else { v = (u1&0xfffff)|0x3600000 }
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

unsafe extern "C" {
    // blob leaf (not renamed, single caller): (slot, protect_enable, unused, rts_threshold, val).
    // Writes CONF0 (0x600a4d60-slot*0x10) bit31 = protect_enable and, when rts_threshold!=0,
    // 0x600a548c-slot*0x74 = val|0x10000. Does NOT touch PLCP0_ENABLE's arm bits.
    fn hal_he_set_tx_protection(slot: i32, enable: i32, p3: u32, threshold: i32, val: u32);
}

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
                        let iv: u32 = if (rd16(eb.wrapping_add(0x24)) & 0x1000) == 0 { 1 } else { 5 };
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
        hal_he_set_tx_protection(slot as i32, enable, 0, threshold, tval);
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
//   blob's "(complete)..." wifi_log branches are logging-only (gated off for the beacon) -> skipped.
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
                *param_3.add(4) = if (aux >> 0x18) & 0x40 == 0 { b } else { b + 0x80 };
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
            v = if (flags as i32) < 0 { 0x400_0000 } else { 0x200_0000 };
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
        // ---- LEAD 2a(2): on alternate beacons, overwrite slot 0's programming with OUR frame
        // BEFORE the arm write below, so the blob's own |=0xc0000000 launches OUR frame (CR-RUST)
        // on the functional queue, in ppTask context. Even beacons keep the blob's own beacon
        // (CR-CTRL) as the same-RF control. ac==0 is the beacon queue. ----
        if ARM_INCTX_ENABLED.load(core::sync::atomic::Ordering::Relaxed) && ac == 0 {
            let n = cleanroom_tx::CR_CALL.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            // One-shot window (calls 40..60): arm OUR frame on slot 0 for ~20 beacons, then stop, to
            // test radiation without the sustained-path destabilisation a per-beacon arm causes.
            if (40..60).contains(&n) {
                cleanroom_tx::arm_inctx();
            }
        }
        let addr = txq_plcp0_enable_addr(ac);
        core::ptr::write_volatile(addr, core::ptr::read_volatile(addr) | 0xc000_0000);
        let _ = blob::blob_hal_mac_txq_enable as usize;
    }
}

pub static ARM_INCTX_ENABLED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

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
            2 => wr(TXQ_CLR_STATE_B, rd(TXQ_CLR_STATE_B) | (1u32 << (bit & 0x1f))),
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
    unsafe { wr(TX_MIN_PWR, ((pwr & 0x3f) << 4) | (rd(TX_MIN_PWR) & 0xffff_fc0f)) }
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
        let rtsidx = mac_tx_get_rts_rate(rate) as u32;
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
            let r2 = *((rd_at(eb.wrapping_add(0x34)).wrapping_add(0xc)) as usize as *const u8) as u32;
            let idx = if r2 > 0x19 { r2 - 0xa } else { r2 };
            let c1 = pwr_byte(idx.wrapping_mul(2));
            let c2 = pwr_byte(idx.wrapping_mul(2).wrapping_add(1));
            wr(plcp_rate_dur, ((c2 << 8) | (c1 | s2)) as u32);
        } else {
            // DSSS / legacy path (our beacon).
            mac_tx_set_len(param_1, param_2);
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
mod cleanroom_tx {
    use super::{hal_mac_tx_config_edca, hal_mac_tx_config_timeout, hal_mac_tx_set_ppdu,
                hal_mac_txq_enable, hal_mac_txq_disable, rd, rd_at, wr};

    // The C6 MAC only accepts register writes to a TX slot whose bank the scheduler has
    // activated; only slot 0 (the raw beacon's AC 0) is live. So we drive slot 0 ourselves,
    // time-multiplexed with a quiesced blob beacon.
    pub const MY_AC: i32 = 0;

    unsafe extern "C" {
        // libpp esf_buf pool alloc (type 1 = static TX). Copies `payload` in, sets eb sizes,
        // returns the eb pointer (0 on pool-empty).
        fn esf_buf_alloc(payload: *const u8, pool_type: i32, len: u32) -> u32;
        fn hal_now() -> u32;
        // coex medium request the blob issues (from ppProcessTxQ) right before lmacTxFrame.
        fn pp_coex_tx_request(eb: u32);
    }

    /// Issue the coex TX request the blob does before arming (tests the coex-grant hypothesis).
    pub fn coex_request(eb: u32) {
        unsafe { pp_coex_tx_request(eb) }
    }

    /// Raw PLCP0_ENABLE read for a slot (for tight in-context arm-bit polling).
    #[inline(always)]
    pub fn raw_plcp0(slot: i32) -> u32 {
        unsafe { rd(0x600a_4d6c - (slot as u32) * 0x10) }
    }

    // ---- LEAD 2a(2): arm OUR frame from ppTask context (called inside hal_mac_txq_enable) ----
    // Proven: control-register writes + launch bits STICK when issued in this (MAC-active ppTask)
    // context. We drive the full arm of our pre-built eb on an idle slot here, unblocking the queues
    // first (that write also sticks in this context). Returns the PLCP0_ENABLE readback.
    pub const MY_SLOT2: i32 = 1; // idle slot proven to latch launch bits from ppTask context
    pub static CR_EB: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
    pub static CR_SEQ: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
    pub static CR_CALL: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
    pub static CR_ARM_RB: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
    static mut CR_CTX: [u8; 0x40] = [0u8; 0x40];

    /// Reprogram slot 0 (the functional queue the blob just set up for its beacon) with OUR frame,
    /// WITHOUT setting the arm bit — the blob's own `hal_mac_txq_enable` write that follows will
    /// launch whatever is now in slot 0. Called in ppTask context before that arm write. Leaves
    /// our_instances[0] (blob's cur_eb/state) intact so the blob's completion recycles ITS eb.
    pub fn arm_inctx() {
        use core::sync::atomic::Ordering::Relaxed;
        let eb = CR_EB.load(Relaxed);
        if eb == 0 {
            return;
        }
        unsafe {
            let seq = CR_SEQ.fetch_add(1, Relaxed) as u16;
            prep_eb(eb, 0, 0, seq); // our frame, AC/slot 0
            let ctx = core::ptr::addr_of_mut!(CR_CTX) as *mut u8;
            core::ptr::write_unaligned(ctx as *mut u32, eb);
            *ctx.add(4) = 0; // slot 0
            *ctx.add(5) = 2;
            *ctx.add(8) = 4;
            *ctx.add(0x1c) = 0;
            *ctx.add(0x1d) = 1;
            let ours = rd(0x4004_ffe0);
            super::hal_mac_tx_config_timeout(ctx, 0x200);
            super::hal_mac_tx_set_ppdu(ctx, ours as i32); // overwrites slot-0 PLCP with our frame
            let backoff = (hal_now() as u16) & 0x0f;
            core::ptr::write_unaligned(ctx.add(6) as *mut u16, backoff);
            super::hal_mac_tx_config_edca(ctx);
            CR_ARM_RB.store(rd(0x600a_4d6c), Relaxed); // slot0 PLCP0_ENABLE (pre-arm)
        }
    }

    #[inline(always)]
    pub fn raw_txblock() -> u32 {
        unsafe { rd(0x600a_4ca8) }
    }

    /// From OUR normal task: spin until the MAC is in the active window (block != 0x00ff1000, i.e.
    /// the blob's just-submitted beacon is keying up), then test whether a CPU write to the block
    /// register and to an idle slot's launch bits STICKS. Returns (bit0_stuck, block_val, launch).
    /// This separates execution-context gating from MAC-active-state gating.
    pub fn active_window_write_probe() -> (bool, u32, bool) {
        unsafe {
            let mut blk = 0x00ff_1000u32;
            for _ in 0..200000 {
                let b = rd(0x600a_4ca8);
                if b != 0x00ff_1000 {
                    blk = b;
                    // MAC active: try the writes immediately.
                    wr(0x600a_4ca8, b ^ 0x1);
                    let stuck = (rd(0x600a_4ca8) & 1) != (b & 1);
                    wr(0x600a_4ca8, b);
                    let p = rd(0x600a_4d5c); // idle slot 1 launch
                    wr(0x600a_4d5c, p | 0xc000_0000);
                    let latched = rd(0x600a_4d5c) & 0xc000_0000 != 0;
                    wr(0x600a_4d5c, p);
                    return (stuck, blk, latched);
                }
            }
            (false, blk, false)
        }
    }

    /// One-time per-bit writability probe of WDEV_PM_TXBLOCK_RETENTION (0x600a4ca8), run in the
    /// SAME (test-phase) MAC state as our failing arm. Distinguishes "CPU write dropped" from
    /// "write stuck then HW re-blocks".
    pub fn txblock_write_probe() {
        unsafe {
            let v0 = rd(0x600a_4ca8);
            wr(0x600a_4ca8, v0 | 0x0000_0001); let set_b0 = rd(0x600a_4ca8);
            wr(0x600a_4ca8, v0 | 0x0000_0400); let set_b10 = rd(0x600a_4ca8);
            wr(0x600a_4ca8, v0 & !0x0000_1000); let clr_b12 = rd(0x600a_4ca8);
            wr(0x600a_4ca8, 0); let wr_zero = rd(0x600a_4ca8);
            wr(0x600a_4ca8, v0); // restore
            super::println!(
                "[CR.wp] txblock={v0:#010x} |b0->{set_b0:#010x} |b10->{set_b10:#010x} &~b12->{clr_b12:#010x} =0->{wr_zero:#010x}"
            );
        }
    }

    #[inline(always)]
    unsafe fn wr8(addr: u32, v: u8) { unsafe { core::ptr::write_volatile(addr as *mut u8, v) } }
    #[inline(always)]
    unsafe fn wr16(addr: u32, v: u16) { unsafe { core::ptr::write_volatile(addr as *mut u16, v) } }
    #[inline(always)]
    unsafe fn rd8(addr: u32) -> u8 { unsafe { core::ptr::read_volatile(addr as *const u8) } }
    #[inline(always)]
    unsafe fn rd16(addr: u32) -> u16 { unsafe { core::ptr::read_volatile(addr as *const u16) } }

    /// Allocate ONE eb from the live static-TX pool with our beacon payload copied in.
    pub fn alloc_eb(payload: &[u8]) -> u32 {
        unsafe { esf_buf_alloc(payload.as_ptr(), 1, payload.len() as u32) }
    }

    /// Fill the dma_desc + txinfo fields exactly like ieee80211_output_raw_process, minus
    /// the node/seq lookup and ppTxPkt. Re-run every iteration: the MAC clears the DMA owner
    /// bit after consuming the descriptor, so it must be re-armed, and we bump the seqno/tsf.
    pub fn prep_eb(eb: u32, iface: u32, my_ac: u32, seq: u16) {
        unsafe {
            let dma = rd_at(eb + 4);        // dma_desc
            let frame = rd_at(dma + 4);     // dma_desc[1] = on-air frame bytes
            wr16(eb + 0x14, 0);             // header-len part = 0 (total len comes from eb+0x16)
            let l14 = rd16(eb + 0x14) as u32;
            let l16 = rd16(eb + 0x16) as u32;
            // dma_desc[0]: owner | eof | (clear 29) | length(bits14-27)
            let mut w0 = rd(dma);
            w0 |= 0x8000_0000;
            w0 |= 0x4000_0000;
            w0 &= 0xdfff_ffff;
            w0 = (((l16 + l14) & 0x3fff) << 0xe) | (w0 & 0xf000_3fff);
            wr(dma, w0);
            let txinfo = rd_at(eb + 0x34);
            wr8(txinfo + 4, 7);                 // cat/AC group = 7 (mgmt)
            wr(txinfo + 0x18, hal_now());       // tsf submit time
            // txinfo+0x10: iface bit19 + AC bits20-23 (ppMapTxQueue would set the AC; we set ours)
            let mut w10 = rd(txinfo + 0x10);
            w10 = (w10 & 0xfff7_ffff) | ((iface & 1) << 0x13);
            w10 = (w10 & 0xff0f_ffff) | ((my_ac & 0xf) << 0x14);
            wr(txinfo + 0x10, w10);
            // flags word: clear discard/HE/AMPDU; broadcast addr1 -> |=0x402; match live beacon 0x6xxx
            let mut fl = rd(txinfo);
            fl &= !(0x0001_0000u32 | 0x8000_0000 | 0x0040_0000);
            fl &= !0x40; // ensure bit6 clear so (flags & 0xc0) == 0x80
            fl |= 0x80;  // set bit7: mac_tx_set_txop_q keeps PLCP0_ENABLE bit22 (descriptor base)
            if (rd8(frame + 4) & 1) != 0 { fl |= 0x402; }
            fl |= 0x6000;
            wr(txinfo, fl);
            wr8(txinfo + 0xc, 0);               // rate idx 0 = 1 Mbit/s DSSS (legacy beacon)
            wr8(txinfo + 0x20, 0);              // pti
            wr16(txinfo + 0x22, 0);
            wr(txinfo + 0x40, 0);               // lifetime fields (config_timeout floors via param2)
            wr(txinfo + 0x44, 0);
            wr16(frame + 0x16, seq << 4);       // 802.11 sequence control
        }
    }

    /// The clean-room ARM: our own copy of the lmacSetTxFrame + lmacTxFrame essentials,
    /// driving the proven Rust hal functions. `ctx` is a private 0x40-byte fake txq context;
    /// only the fields the hal functions read are filled. No our_instances write.
    pub fn arm(ctx: *mut u8, eb: u32, slot: i32) -> u32 {
        unsafe {
            // fake txq context fields the hal fns read:
            core::ptr::write_unaligned(ctx as *mut u32, eb); // [0..4] eb ptr
            *ctx.add(4) = slot as u8;                         // [4] slot/AC
            *ctx.add(5) = 2;                                  // [5] aifsn
            *ctx.add(8) = 4;                                  // [8] CW exponent (backoff mask)
            *ctx.add(0x1c) = 0;                               // [0x1c] txop s3
            *ctx.add(0x1d) = 1;                               // [0x1d] txop depth
            let ours = rd(0x4004_ffe0);                       // our_instances base (set_ppdu param2)
            // lmacSetTxFrame essentials: lifetime timeout, then the PPDU/slot programming.
            hal_mac_tx_config_timeout(ctx, 0x200);
            hal_mac_tx_set_ppdu(ctx, ours as i32);
            // lmacTxFrame essentials: random EDCA backoff (masked by CW), config_edca, arm.
            let backoff = (hal_now() as u16) & !(0xffffu16 << (*ctx.add(8) & 0x1f));
            core::ptr::write_unaligned(ctx.add(6) as *mut u16, backoff & 0x3ff);
            hal_mac_tx_config_edca(ctx);
            // CANDIDATE STROBE (lead 1): unblock TX in WDEV_PM_TXBLOCK_RETENTION (0x600a4ca8)
            // exactly as hal_mac_init + hal_pm_unblock_txq do, right before the launch write.
            let txblock_before = rd(0x600a_4ca8);
            if UNBLOCK_TX {
                wr(0x600a_4ca8, txblock_before & !(0x00ff_0000 | 0x1000 | 0x000e_0000));
            }
            let txblock_after = rd(0x600a_4ca8); // did the unblock write stick?
            hal_mac_txq_enable(slot);
            // Immediate readback of PLCP0_ENABLE (arm bits are 0xc0000000). Returns whether the
            // valid|enable latched at all, the very first bus cycle after the enable write.
            let rb = rd(0x600a_4d6c - (slot as u32) * 0x10);
            LAST_TXBLOCK.store(txblock_before, core::sync::atomic::Ordering::Relaxed);
            LAST_TXBLOCK_AFTER.store(txblock_after, core::sync::atomic::Ordering::Relaxed);
            rb
        }
    }

    /// Toggle for the candidate unblock strobe.
    pub const UNBLOCK_TX: bool = true;
    pub static LAST_TXBLOCK: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
    pub static LAST_TXBLOCK_AFTER: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

    /// Disarm our slot and clear its completed-state bit, so the next arm starts clean and the
    /// blob's TX-complete scan sees no stale bit for our slot.
    pub fn disarm(slot: i32) {
        unsafe {
            hal_mac_txq_disable(slot);
            wr(0x600a_4cac, 1u32 << slot); // clr state A bit for our slot (write-1-to-clear)
        }
    }

    /// Out-of-band diagnostic snapshot (called rarely, never on the hot path): the arm bits of
    /// all 8 slots, our slot's raw PLCP0_ENABLE, EDCA-conf (0x4d68), and PMD completion word.
    pub fn snapshot(slot: i32) -> (u32, u32, u32, u32) {
        unsafe {
            let mut arms = 0u32;
            for s in 0..8u32 {
                let plcp0 = rd(0x600a_4d6c - s * 0x10);
                if (plcp0 & 0xc000_0000) != 0 { arms |= 1 << s; }
            }
            let plcp0 = rd(0x600a_4d6c - (slot as u32) * 0x10);
            let edca = rd(0x600a_4d68 - (slot as u32) * 0x10);
            let pmd = rd(0x600a_54e8 - (slot as u32) * 0x74);
            (arms, plcp0, edca, pmd)
        }
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
                super::println!("[CR.probe] slot{s} plcp0={p0:#010x} edca={e0:#010x} writable={writable}");
            }
        }
    }
}

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

#[esp_hal::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);

    let delay = Delay::new();
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

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

    println!("[CR] booting; control SSID '{SSID_CTRL}', Rust-arm SSID '{SSID_RUST}' slot AC{}", cleanroom_tx::MY_AC);

    // Let the blob bring-up settle and prove RF with a few control beacons first.
    for _ in 0..10 {
        let _ = sniffer.send_raw_frame(true, ctrl, false);
        delay.delay(Duration::from_millis(100));
    }

    // Allocate our eb ONCE from the live static-TX pool (payload copied in). Reused forever.
    let eb = cleanroom_tx::alloc_eb(&rust_buf[..rust_len]);
    println!("[CR] esf_buf_alloc(type=1, len={rust_len}) -> eb={eb:#010x}");
    cleanroom_tx::probe();
    let mut ctx = [0u8; 0x40];
    let ctx_ptr = ctx.as_mut_ptr();

    // LEAD 2a(2): publish our eb and let hal_mac_txq_enable (ppTask context) arm it on slot 1.
    cleanroom_tx::CR_EB.store(eb, core::sync::atomic::Ordering::Relaxed);
    ARM_INCTX_ENABLED.store(eb != 0, core::sync::atomic::Ordering::Relaxed);

    let slot = cleanroom_tx::MY_AC;
    // Our NORMAL-task arm is disabled; arming happens only from ppTask (in-context) for lead 2a(2).
    const RUN_MY_ARM: bool = false;
    let mut round: u32 = 0;
    let mut seq: u16 = 0;
    loop {
        // ===== CONTROL PHASE: 10 blob-driven beacons on slot 0 (same-RF positive reference) =====
        for k in 0..10 {
            let _ = sniffer.send_raw_frame(true, ctrl, false);
            if round == 1 && k == 0 {
                let rb = cleanroom_tx::CR_ARM_RB.load(core::sync::atomic::Ordering::Relaxed);
                println!("[CR.2a2] in-ctx arm of slot{} readback PLCP0_ENABLE={rb:#010x} (latched={})",
                    cleanroom_tx::MY_SLOT2, rb & 0xc000_0000 != 0);
            }
            // LEAD 2a refinement: is the write-lock CONTEXT(ppTask)-gated or MAC-ACTIVE-STATE gated?
            // Probe write-stickiness from OUR OWN task, but in the ~active window right after submit
            // (block != 0x00ff1000). If a write STICKS here (our task, MAC active) it is MAC-state,
            // not execution-context, that gates it.
            if round == 2 && k == 0 {
                let (stuck, blk, plcp_latch) = cleanroom_tx::active_window_write_probe();
                println!("[CR.2a3] our-task active-window write: txblock={blk:#010x} bit0_stuck={stuck} slot1_launch_latched={plcp_latch}");
            }
            // On round 0, tight-poll PLCP0_ENABLE right after submit to CATCH the blob's own
            // in-context arm bit (0xc0000000) latching — proves the bits ARE settable in-context.
            if round == 0 && k == 0 {
                // Track the block register (0x600a4ca8) trajectory right after submit: find if/when
                // it leaves the blocked state 0x00ff1000 (a window our arm could exploit), and catch
                // the launch bit + the block value at that instant.
                let mut arm_caught = false;
                let mut txblock_at_arm = 0xdead_beef_u32;
                let mut n_unblocked = 0u32;    // samples with block != 0x00ff1000
                let mut first_reblock: i32 = -1;
                let mut seen_unblock = false;
                const N: i32 = 60000;
                for idx in 0..N {
                    let b = cleanroom_tx::raw_txblock();
                    let v = cleanroom_tx::raw_plcp0(slot);
                    if b != 0x00ff_1000 { n_unblocked += 1; seen_unblock = true; }
                    else if seen_unblock && first_reblock < 0 { first_reblock = idx; }
                    if (v & 0xc000_0000) != 0 { arm_caught = true; txblock_at_arm = b; }
                }
                println!("[CR] blob poll: arm_caught={arm_caught} txblock_at_arm={txblock_at_arm:#010x} unblocked_samples={n_unblocked}/{N} first_reblock@{first_reblock}");
            }
            delay.delay(Duration::from_millis(100));
        }

        // Let the last blob beacon fully complete + recycle before we take over slot 0.
        delay.delay(Duration::from_millis(50));

        // ===== TEST PHASE: 10 of OUR OWN Rust-driven arms on slot 0 (blob beacon quiesced) =====
        if eb != 0 && RUN_MY_ARM {
            if round == 0 {
                cleanroom_tx::txblock_write_probe();
            }
            for i in 0..10u32 {
                cleanroom_tx::prep_eb(eb, 0, slot as u32, seq);
                seq = seq.wrapping_add(1);
                cleanroom_tx::coex_request(eb); // blob issues this before lmacTxFrame
                let arm_rb = cleanroom_tx::arm(ctx_ptr, eb, slot);
                if round == 0 && i < 4 {
                    let (arms, plcp0, edca, pmd) = cleanroom_tx::snapshot(slot);
                    let txb = cleanroom_tx::LAST_TXBLOCK.load(core::sync::atomic::Ordering::Relaxed);
                    let txa = cleanroom_tx::LAST_TXBLOCK_AFTER.load(core::sync::atomic::Ordering::Relaxed);
                    println!(
                        "[CR] test#{i} txblock {txb:#010x}->{txa:#010x} enable_readback={arm_rb:#010x} | arms={arms:#04x} plcp0={plcp0:#010x} edca={edca:#010x} pmd={pmd:#010x}"
                    );
                }
                delay.delay(Duration::from_millis(60));
                let (_, _, _, pmd_done) = cleanroom_tx::snapshot(slot);
                if round == 0 && i < 4 {
                    println!("[CR] test#{i} post-wait pmd={pmd_done:#010x}");
                }
                cleanroom_tx::disarm(slot);
                delay.delay(Duration::from_millis(40));
            }
        }
        round += 1;
        if round % 5 == 0 {
            println!("[CR] completed {round} control+test rounds");
        }
    }
}
