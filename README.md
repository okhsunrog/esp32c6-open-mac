# ESP32-C6 open-MAC — pure-Rust Wi-Fi TX

Open (pure-Rust) Wi-Fi **transmit** on the **ESP32-C6** (Wi-Fi 6 MAC): the entire per-frame
TX MAC path — submit, AC mapping, enqueue, schedule, arm, the modem-PM wake, and completion —
runs as our own Rust on the real scheduler state, with no blob `pp`/`lmac`/`hal_mac_tx` call on
the exercised path. Verified over the air, at the **same TX power and rate as the full blob
stack** (`CR-RUST` ≈ −69 dBm vs the blob's ≈ −68 dBm on the same monitor). The reference lives
in `src/cleanroom.rs`.

C6 TX was an open problem — `esp32-open-mac` had not done it, and the direct-register method
that opens TX on the Wi-Fi-4 MACs (ESP32 / S2 / S3 / C3) does **not** work on the C6 Wi-Fi-6
MAC. This repo is the lab notebook: the reverse-engineering, the tooling, the working Rust
pipeline, and the honest boundary of what stays substrate.

## What is Rust, and where the boundary is

The whole per-frame TX MAC logic is Rust (`cr_*` in `src/cleanroom.rs`), driving the real
`our_instances` / `TxRxCxt` scheduler state and our own eb pool:

| Stage | Function |
|-------|----------|
| Submit | `cr_ppTxPkt` (proto flags, AC map, enqueue onto the real `TxRxCxt` pending list) |
| Modem-PM wake | `cr_pm_on_data_tx` (g_pm FSM → clocks/PHY via esp-radio OSI → the `hal_mac_init` unblock store) |
| Schedule | `cr_ppProcessTxQ` (idle guard, pop, coex, arm) |
| Arm | `cr_lmacTxFrame` / `cr_lmacSetTxFrame` (cur_eb, backoff, EDCA, state, PPDU build, slot enable) |
| Register/arm ops | the `hal_mac_tx.o` layer (plcp0/1, config_edca/timeout, txq_enable, set_ppdu, get_txq_complete, set_len/pti, he_set_tx_protection, …) |
| eb pool | `cr_pool_*` — an independent Rust eb pool the TX loop owns end-to-end |
| Security header | `cr_ppProcTxSecFrame` (the required no-key header-length adjustment) |
| Completion | `cr_complete` (poll arm-clear → decode → clear state → recycle eb; our loop owns it) |
| Helpers | `cr_hal_now`, `cr_hal_random`, `cr_rcGetSched`, `cr_ppMapTxQueue`, `cr_ppGetTxframe`, the guards |

**What stays blob is substrate — the same boundary the open C3 stack already accepts:**
the **PHY / RF** (`libphy`, reached through `esp-phy`), and chip/clock init + the FreeRTOS OS
adapter (via `esp-radio`'s `WifiController::new` / `esp_wifi_start`). Our modem-PM wake calls
`wifi_rf_phy_enable`'s work through esp-radio's **own Rust** OSI callbacks
(`radio_clocks::enable_wifi`, `esp_phy::enable_phy_with_wifi_rx`) plus a Rust store that clears
the PM-block bits — so even the wake is Rust down to the `libphy`/init substrate. The only
`libpp` symbols still linked are on branches the legacy DSSS beacon never takes (HE/HT SIG,
AMPDU aggregation) and the blob's own RX/init/ISR — not our TX path.

In short: **C6 TX is now as open as C3 TX at the MAC layer**, leaning only on `libphy` + init,
which is exactly what `esp-wifi-hal` leans on today.

## Why the C6 is hard (the mechanism, corrected)

On the Wi-Fi-4 MACs you transmit by poking the TX-slot registers directly (how `esp-wifi-hal`
does open TX on the C3). On the C6 that produces `tx_success` but **zero RF**, because two
gates sit in front of the launch:

1. **While the modem sleeps the MAC clock is gated** — every MAC register write is silently
   dropped, including the `PLCP0_ENABLE[31:30]` launch-enable bits (they read back unchanged).
2. **The PM TX-block register** `WDEV_PM_TXBLOCK_RETENTION` (`0x600a4ca8`) reads `0x00ff1000`
   and blocks any armed frame from launching.

Both are undone by the submit path's **`pm_on_data_tx` modem-PM wake**: it re-enables the
clocks and PHY, then `hal_mac_init` does an ordinary CPU store `0x600a4ca8 &= ~((mask&0xff0000)
| 0x1000)` that clears the block. Earlier passes wrongly read this as "a hardware consequence,
no CPU store clears it" and called it near-fundamental — the store is simply *dropped while the
clock is still gated*, so you must enable clocks **and** do the store **and** bring the PHY up,
in order. All three are now reproduced in Rust, so the interlock is an ordinary, reproducible
sequence — not a wall. (See `docs/cleanroom_tx.md`, "The Rust modem-PM wake".)

The consequence: open C6 TX cannot skip the `pp/lmac` submit→schedule flow (as every shortcut —
direct poke, per-symbol interposition, a fake-context arm — did). It must faithfully run that
flow on real scheduler state; then the arm keys the PHY.

## How we got here (two approaches)

- **Interposition (early):** patch `libpp.a` to rename an object's cross-boundary symbols to
  `blob_<fn>` (`tools/patch_libpp.sh`), re-provide each as a Rust function, and replace bodies
  one at a time under a "still radiates" invariant. This works for the placement/timing-
  insensitive **register** layer (`hal_mac_tx.o`, fully de-blobbed this way) but **not** for
  the `lmac.o` scheduler functions: a functionally-identical copy at a different address stalls
  TX (a placement/timing coupling). `src/deblob.rs` is that earlier vehicle.
- **Faithful reproduction (the result):** drive the full submit→schedule→arm→complete ourselves
  from Rust, calling our `cr_*` functions directly on the **real** `our_instances`/`TxRxCxt`
  state — no symbol interposition, so the placement wall doesn't apply. This is what radiates,
  and what the whole MAC path was then built out in. `src/cleanroom.rs` is the reference.

`libpp.a` keeps symbols and splits cleanly into objects — `pp.o`, `lmac.o`, `hal_mac_tx.o`,
`esf_buf.o` — which made both approaches tractable. See `docs/txsm_map.md` (TX state machine),
`docs/pp_init_map.md` (init/bring-up), `docs/deblob_progress.md` (per-function log + register
map), `docs/cleanroom_tx.md` (the interlock investigation and the faithful pipeline).

## Integration: how this lands in the open-MAC project

The reusable deliverables: the full reversed TX state machine + register/struct model
(`docs/`), and pure-Rust reimplementations of the entire TX MAC logic path driving real
scheduler state — portable code, cross-referenced with `esp-wifi-hal/src/ll.rs`.

The MAC-layer TX is fully Rust at the **same substrate boundary as the open C3 stack**
(`libphy` + chip/clock init + the OS adapter). So a C6 TX path can be contributed to
`esp-wifi-hal` / FoA on that footing. It currently runs on `esp-radio`'s init/OSI substrate;
slimming that to `esp-wifi-hal`'s own minimal init (which already does C6 **RX**) is follow-up
work, not a new unknown.

Worth sharing upstream with `esp32-open-mac`: the mechanism above answers their open "why no
C6 TX" and shows the path.

## Hard-won lessons (read before touching the TX path)

- **Version-specific addresses are the #1 hazard.** The blob moves data statics between builds
  (`g_pm` moved `0x40820720→40→30→70`; `lmacConfMib` moved between `2ea8e3e` and `0.3.0`). A
  wrong address once caused a double-free that corrupted the heap and only paniced under later
  layout shifts ("adding dead code crashes"). Reference blob data as **linked `extern` statics**
  where possible, and verify every hardcoded address/offset against the linked-ELF disasm.
- **Don't interleave a blob control beacon with our slot-0 pipeline.** `send_raw_frame` kicks
  the blob `ppTask`, which then schedules onto slot 0 and races our direct arm/completion on the
  same slot + shared pending list — collapsing both to a weak trickle. Use a boot-time baseline
  or an opt-in mode; keep the sustained loop slot-0-exclusive. (This — not any RF/power problem —
  was behind a "TX degraded to −82 dBm" scare; slot-0-exclusive, our TX is ≈ −69 dBm, blob-equal.)
- **The TX slot-programming / completion path is timing-sensitive:** diagnostics must be
  zero-overhead. Per-call volatile-MMIO readback loops or atomic counters injected into the
  sustained path themselves cause stalls, even when the blob does the work.
- **The serial `beacon #N` counter is not a radiation signal** (`send_raw_frame` returns `Ok`
  during a stall). Validate by mon0 frame counts over 30 s+ windows; RF varies, so compare
  ratios / the on-device `[CR.H]` oracle (`arms==latched==completed`) and *in-capture* controls,
  not absolute counts across runs.
- **Don't blindly replicate context-struct pointer derefs** from the decompiler; verify offsets
  against the disasm or delegate that sub-part.

## Build / flash / validate

`src/cleanroom.rs` is an **esp-hal example bin** — it builds inside the esp-hal workspace.

1. Patch the vendored `libpp.a` (`tools/patch_libpp.sh hal_mac_tx.o lmac.o`; edit the checkout
   path for your `esp-wifi-sys` rev), then `cargo clean -p esp-wifi-sys-esp32c6` (a `touch` is
   not enough — the copy to `OUT_DIR` is cached).
2. Copy `src/cleanroom.rs` to `<esp-hal>/examples/wifi/80211_tx/src/bin/cleanroom.rs`.
3. Build / flash (needs the stub loader):
   ```
   cd <esp-hal>/examples/wifi/80211_tx
   cargo build --release --bin cleanroom --target riscv32imac-unknown-none-elf --features esp32c6
   espflash flash --chip esp32c6 --port /dev/ttyACM0 \
     target/riscv32imac-unknown-none-elf/release/cleanroom
   ```
4. Validate (laptop AX210 in monitor mode via `tools/mon_up.sh 1`):
   ```
   sudo -n timeout 30 tcpdump -i mon0 -n -w d.pcap
   tshark -r d.pcap -Y 'wlan.sa==02:00:00:00:c6:00' -e radiotap.dbm_antsignal -T fields | ...
   ```
   Expect a steady stream of `CR-RUST` beacons at ≈ −69 dBm. The firmware also prints an on-device
   `[CR.H]` oracle (`arms/latched/completed/allocfail/poolfree`); `latched==arms` with
   `last_plcp0 = 0xc0......` means the MAC went active and the frame armed. Diagnostics and the
   opt-in blob control are behind `const DIAG` / `CONTROL_BEACON` / env toggles.

## Revert the blob patch

```
cp <checkout>/esp-wifi-sys-esp32c6/libs/libpp.a.orig <checkout>/.../libpp.a
cargo clean -p esp-wifi-sys-esp32c6
```

## Layout

- `src/cleanroom.rs` — the pure-Rust C6 TX pipeline (the reference, the working result).
- `src/deblob.rs` — the earlier per-symbol interposition vehicle (`hal_mac_tx.o` de-blob).
- `tools/patch_libpp.sh` — rename an object's cross-boundary symbols to `blob_<fn>`.
- `tools/retools.py`, `tools/reva.py`, `docs/REVA-README.md` — ReVa/Ghidra RE toolkit.
- `tools/mon_up.sh`, `tools/mon_down.sh` — AX210 monitor helpers (dedicated `mon0` vif).
- `docs/cleanroom_tx.md` — the interlock investigation, the faithful pipeline, the Rust PM wake.
- `docs/deblob_progress.md` — per-function de-blob log + verified register map.
- `docs/txsm_map.md` — reversed C6 TX state machine.
- `docs/pp_init_map.md` — reversed pp/lmac init / bring-up path.
- `re/` — raw reverse-engineering artifacts (symbol maps, call graph, OSI usage).

## Hardware

ESP32-C6 (rev 0.2) on USB-JTAG `/dev/ttyACM0`; laptop Intel AX210 as the OTA monitor (`mon0`,
ch 1). Reversed against `esp-wifi-sys` rev `2ea8e3e` (Ghidra project); the working build runs on
`esp-wifi-sys` `0.3.0` — data-static addresses differ between the two and are re-derived from
the live linked ELF (see the version-address lesson above).
