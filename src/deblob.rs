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
// We reference the symbol (+offset) so the read is link-stable and sees the same runtime
// value the blob's lmacInit/config wrote. This model (also in docs/deblob_progress.md) is the
// pre-relocation flash copies of these same fields).
// De-blobbed lmac function (interposable, non-ABS). The full context model + the list of
// blockers (ROM *ABS* symbols; the non-deterministic PM-sleep TX stall that also hits the
// all-shim control and makes the radiate invariant unreliable) live in the research repo's
// docs/deblob_progress.md.
//
/// blob `lmac_update_tx_statistic`: empty in the blob (just `ret`; blob_ at 0x42025b8c). Its
/// address is installed into the wdev funcs pointer table by wdev_funcs_init and invoked
/// indirectly. Reimplemented as a no-op (correct by inspection: the blob body is empty). Confirmed
/// to reach full beacon rate (100/10s), matching the all-shim control's pre-stall behaviour.
#[unsafe(no_mangle)]
pub extern "C" fn lmac_update_tx_statistic() {}

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
        let addr = txq_plcp0_enable_addr(ac);
        core::ptr::write_volatile(addr, core::ptr::read_volatile(addr) | 0xc000_0000);
        // Note: legacy (non HE-TB) frames need nothing more for the arm. HE-TB muedca
        // bookkeeping is left as a TODO (delegate to blob::blob_hal_mac_txq_enable when
        // HE-TB support is added). Keep the symbol referenced so it stays linked.
        let _ = blob::blob_hal_mac_txq_enable as usize;
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

const SSID: &str = "DEBLOB-HAL";
const MAC_ADDRESS: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0xda, 0xb0];

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
    // Disable modem sleep. A sniffer-only build never hits the STA-start path that applies
    // PowerSaveMode::None, so the modem inherits IDF's WIFI_PS_MIN_MODEM default and the pp
    // power-management path (ppCheckTxConnTrafficIdle) sleeps the radio after a while, making
    // the beacon TX decay to zero. Forcing WIFI_PS_NONE keeps the modem awake so the radiate
    // invariant stays reliable for de-blob validation.
    controller
        .set_power_saving(esp_radio::wifi::PowerSaveMode::None)
        .unwrap();
    let mut sniffer = controller.sniffer();

    let mut beacon = [0u8; 300];
    let length = beacon
        .pwrite(
            BeaconFrame {
                header: ManagementFrameHeader {
                    fcf_flags: FCFFlags::new(),
                    duration: 0,
                    receiver_address: [0xff; 6].into(),
                    transmitter_address: MAC_ADDRESS.into(),
                    bssid: MAC_ADDRESS.into(),
                    ..Default::default()
                },
                body: BeaconBody {
                    timestamp: 0,
                    beacon_interval: 100,
                    capabilities_info: CapabilitiesInformation::new().with_is_ess(true),
                    elements: element_chain! {
                        SSIDElement::new(SSID).unwrap(),
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
        .unwrap();
    let beacon = &beacon[..length];

    println!("[DEBLOB] hal_mac_tx.o interposed via shims; TXing SSID '{SSID}'");
    let mut n: u32 = 0;
    loop {
        match sniffer.send_raw_frame(true, beacon, false) {
            Ok(()) => {
                if n % 20 == 0 {
                    println!("[DEBLOB] beacon #{n} tx ok (plcp0 real Rust)");
                }
            }
            Err(e) => println!("[DEBLOB] tx err: {e:?}"),
        }
        n += 1;
        delay.delay(Duration::from_millis(100));
    }
}
