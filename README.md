# ESP32-C6 open-MAC de-blob

Research toward an open (pure-Rust) Wi-Fi TX path on the **ESP32-C6** (Wi-Fi 6 MAC),
by incrementally replacing the proprietary `libpp.a` object files with Rust
reimplementations **one function at a time**, verifying on real hardware that TX still
radiates after every step.

This repo is the lab notebook: the method, the reverse-engineering digests, the tooling,
and the current de-blob source of truth (`src/deblob.rs` / `src/cleanroom.rs`). It is
deliberately separate from the clean pure-Rust HAL work (the `esp-wifi-hal` / `FoA` forks)
because the approach here **runs the blob pp/lmac scheduler** and cuts it down from the
working side.

## ✅ Breakthrough: our own Rust code radiates a frame on the C6

A Rust-driven transmit path (`src/cleanroom.rs`, `faithful_tx`) puts **our own beacon on
the air** on the C6 — verified on an external monitor (SSID/MAC `CR-RUST`, 1 Mbit DSSS),
steady and reproducible alongside a blob-beacon control (`CR-CTRL`). The arm fires through
our proven Rust `hal_mac_tx` layer on the **real** `our_instances` scheduler state.

The barrier that stalled every earlier attempt turned out **not** to be fundamental. The
MAC's transmit-enable interlock (the `PLCP0_ENABLE[31:30]` launch bits, gated by the PM
TX-block register `0x600a4ca8`) only opens once the MAC enters its active/txing state — and
that transition is triggered by **`pm_on_data_tx`, called from `ppMapTxQueue` inside
`ppTxPkt`** on the submit path, *not* by the arm. Every earlier shortcut (direct slot poke,
per-symbol interposition, the fake-context clean-room arm) skipped `ppTxPkt`, so the PM wake
never ran and the launch bits could never latch. Faithfully running the real
`ppTxPkt → ppProcessTxQ → lmacTxFrame` sequence on real scheduler state restores the PM wake
and the arm then keys the PHY. **Pure-Rust C6 TX is an engineering reverse-and-port, not a
hardware wall.** (The current `faithful_tx` still calls the blob `ppTxPkt`/`ppProcessTxQ`;
the remaining work is reimplementing those in Rust — see Status.)

## The core finding (why this approach)

On the older Wi-Fi-4 MACs (ESP32 / S2 / S3 / C3, base `0x60033000`) you can transmit by
poking the TX-slot registers directly — that is how `esp-wifi-hal` does open TX on the C3.

On the **C6 Wi-Fi-6 MAC** (base `0x600a4000`) that does **not** work, and we proved it
exhaustively (7 register-hunt passes + two decisive hardware experiments):

- Under a direct-slot poke the MAC completes every frame with `tx_success`/`Ok()` but the
  baseband **never keys up** — 0 frames on air — and **no writable register** reproduces RF.
- On a **fully blob-initialised** C6, a direct poke won't even *arm* the slot: the
  `slot_valid|slot_enabled` bits refuse to latch and `CTRL(0x600a4ca8)` shows the MAC-ready
  bits (`0x6000`) clear; forcing them by a register write is **rejected by hardware**.
- The blob **tears the TX datapath down between frames** and the running **pp/lmac
  scheduler** brings it ready→armed→transmit transiently, per frame. That live sequence is
  what keys the PHY — not any static register state.

Conclusion (confirmed by the breakthrough above): open C6 TX requires faithfully
**reproducing the pp/lmac submit→schedule flow on real scheduler state** — specifically the
`pm_on_data_tx` PM-wake on the submit path plus the real `eb`/pending/DMA linkage — after
which the arm (our Rust `hal_mac_tx`) keys the PHY. We pursued this from two directions:
(a) cut the blob down from the working side, replacing its functions with Rust leaf-up under
a "still radiates" invariant; and (b) drive a faithful Rust submit→schedule→arm ourselves,
which is what achieved the breakthrough.

Structural fact that makes this tractable: `libpp.a` keeps symbols and splits cleanly into
objects — `pp.o` (ppTxPkt/ppProcessTxQ/ppTask/pp_post), `lmac.o` (lmacTxFrame/…),
`hal_mac_tx.o` (the MAC-TX register ops). The whole pp/lmac machine is in the RAM blob (no
ROM pp code, no hidden hardware ops — the "op table" is just `g_wifi_osi_funcs`, FreeRTOS
glue). See `docs/txsm_map.md` (TX state machine) and `docs/pp_init_map.md` (init/bring-up).

## The method (per object)

1. **Rename** the object's cross-boundary symbols in `libpp.a` to `blob_<fn>`
   (`tools/patch_libpp.sh`). Callers in the other objects (`lmac.o`, `wdev.o`, …) still
   reference the plain `<fn>`, so those references become ours to satisfy.
2. **Interpose (Phase 1)**: re-provide each `<fn>` as an ABI-perfect `global_asm!`
   tail-call shim into `blob_<fn>`. If the blob beacon still radiates through the shims,
   the wiring is proven end-to-end (blob pp/lmac → **our** symbols → blob bodies).
3. **Replace (Phase 2)**: swap each shim body for real Rust, one at a time, decompiling the
   blob (ReVa/Ghidra) and cross-checking `esp-wifi-hal/src/ll.rs`; rebuild, flash, and
   confirm sustained radiation before moving on.

The interposition unit is the **cross-boundary** symbol (called from another object).
Object-internal helpers become dead once their callers are Rust. HE / MU-EDCA / HE-TB
bookkeeping is skipped for the legacy DSSS test beacon (guarded out or delegated to
`blob_<fn>`) because the radiate invariant cannot validate paths the beacon never takes.

## Status

**The whole legacy-beacon TX pipeline is now Rust**, driving the real scheduler state, with
`CR-RUST` radiating and the eb pool healthy over sustained runs (health oracle
`arms=latched=completed`, `allocfail=0`). What is Rust vs. the remaining intentional blob
leaves:

| Stage | Status |
|-------|--------|
| Register/arm ops (`hal_mac_tx.o`: plcp0/1, config_edca/timeout, txq_enable, set_ppdu, get_txq_complete…) | **Rust** (19/23; rest HE-only / low-value) |
| Submit `cr_ppTxPkt` (proto flags, AC map, enqueue onto real TxRxCxt pending list) | **Rust** |
| Schedule `cr_ppProcessTxQ` (idle guard, pop, coex, arm) | **Rust** |
| Arm `cr_lmacTxFrame` (cur_eb, backoff, EDCA, state, txq_enable) | **Rust** |
| Pending-list pop `cr_ppGetTxframe`, AC map `cr_ppMapTxQueue`, `cr_hal_random`, `cr_rcGetSched` | **Rust** |
| Completion `cr_complete` (poll arm-clear → decode → clr state → recycle eb; our loop owns it, ISR skips our AC) | **Rust** |

Remaining **intentional** blob leaves (substrate-level, documented in `docs/cleanroom_tx.md`):
`esf_buf_alloc`/`recycle` (the eb pool allocator — substrate boundary), `pm_on_data_tx`
(the PM-wake FSM — the breakthrough ingredient, validated), `ppProcTxSecFrame` (security
header), `pp_coex_tx_request` (coex OSI), `lmacSetTxFrame`'s TSF-lifetime/TXOP sequencing
(its PPDU programming already routes through our Rust hal), and a few constant-returning
guards. Per-symbol interposition of individual `lmac.o` functions is **not viable** (a
functionally-identical copy stalls — a placement/timing coupling); the faithful-reproduction
route (our code calling our Rust directly on real state) supersedes it and is what works.

Milestones: positive control (blob radiates here) → direct-poke / init-state ruled out →
interlock localized to `PLCP0_ENABLE[31:30]` → then to the PM TX-block reg `0x600a4ca8` →
proved it is MAC-active-state gated (not context) → faithful Rust submit→schedule→arm on
real state radiates `CR-RUST` (root cause: the `pm_on_data_tx` PM-wake in `ppTxPkt`) →
**full pipeline incl. completion reimplemented in Rust, pool-healthy over sustained runs.**

See `docs/deblob_progress.md` (per-function de-blob log + register map) and
`docs/cleanroom_tx.md` (the clean-room arm path, the interlock investigation, and the
faithful-reproduction breakthrough, sessions 9a–9d).

## Hard-won lessons (read before touching the TX path)

- The TX slot-programming / completion path is **timing-sensitive**: adding per-call
  volatile-MMIO readbacks or atomic counters as instrumentation **itself causes stalls**,
  even when the blob does the work. Diagnostics on this path must be zero-overhead.
- The serial `beacon #N` counter is **not** a radiation signal — `send_raw_frame` returns
  `Ok` even while the queue is stalled. Only mon0 frame counts over 30 s+ windows prove
  sustained TX; validate with a **control** (known-good shim vs candidate, same RF) and the
  stall **signature** (steady distribution vs progressive decay to zero), not an absolute
  count (ambient RF varies a lot).
- A stall is usually **arm-without-complete**: a frame arms but never completes → the AC
  state stays "armed" → `lmacIsIdle` is false → the next frame is never scheduled → the
  queue decays. (E.g. wrongly setting the PLCP0 ACK-expected bit on a broadcast beacon.)
- Do **not** blindly replicate context-struct pointer derefs from the decompiler
  (`GetAccess()`-style) — they have faulted. Verify offsets or delegate that sub-part.

## Build / flash / validate

`src/deblob.rs` is an **esp-hal example bin** — it only builds inside the esp-hal
workspace. To build it:

1. Patch the vendored `libpp.a` (see `tools/patch_libpp.sh`; edit the checkout path for
   your `esp-wifi-sys` rev), then `cargo clean -p esp-wifi-sys-esp32c6` (a `touch` is not
   enough — the copy to `OUT_DIR` is cached).
2. Copy `src/deblob.rs` to `<esp-hal>/examples/wifi/80211_tx/src/bin/deblob.rs`.
3. Build / flash:
   ```
   cd <esp-hal>/examples/wifi/80211_tx
   cargo build --release --bin deblob --target riscv32imac-unknown-none-elf --features esp32c6
   espflash flash --chip esp32c6 --port /dev/ttyACM0 \
     target/riscv32imac-unknown-none-elf/release/deblob         # needs the stub loader
   ```
4. Validate (laptop AX210 in monitor mode via `tools/mon_up.sh 1`):
   ```
   sudo -n timeout 30 tcpdump -i mon0 -n -w d.pcap
   tshark -r d.pcap -Y 'wlan.ssid=="DEBLOB-HAL"' | wc -l
   ```
   Expect a steady stream of `DEBLOB-HAL` beacons; compare against a pure-shim control build
   under the same RF.

## Revert the blob patch

```
cp <checkout>/esp-wifi-sys-esp32c6/libs/libpp.a.orig <checkout>/.../libpp.a
cargo clean -p esp-wifi-sys-esp32c6
```

## Layout

- `src/deblob.rs` — current de-blob source of truth (the interposition shims + Rust reimpls).
- `tools/patch_libpp.sh` — rename cross-boundary symbols of an object to `blob_<fn>`.
- `tools/retools.py`, `tools/reva.py`, `docs/REVA-README.md` — ReVa/Ghidra RE toolkit.
- `tools/mon_up.sh`, `tools/mon_down.sh` — AX210 monitor helpers (dedicated `mon0` vif).
- `docs/deblob_progress.md` — per-function de-blob log + verified register map.
- `docs/txsm_map.md` — reversed C6 TX state machine.
- `docs/pp_init_map.md` — reversed pp/lmac init / bring-up path + the bring-up recipe.
- `docs/hal_mac_tx_cross.txt` — the 23 cross-boundary symbols of `hal_mac_tx.o`.
- `re/` — raw reverse-engineering artifacts (symbol maps, call graph, OSI usage).

## Hardware

ESP32-C6 (rev 0.2) on USB-JTAG `/dev/ttyACM0`; laptop Intel AX210 as the OTA monitor
(`mon0`, ch 1). The blob is `esp-wifi-sys` rev `2ea8e3e`, chip variant `esp32c6`.
