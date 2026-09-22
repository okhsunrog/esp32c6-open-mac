# ESP32-C6 clean-room Rust Wi-Fi TX pipeline

A pure-Rust demonstrator of the ESP32-C6 Wi-Fi MAC transmit path, built on top of the working
esp-radio blob substrate. Our Rust code drives the whole per-frame TX pipeline — submit, AC-map,
modem-PM wake, pending enqueue/pop, schedule, arm, and completion — on the **real** blob scheduler
state, and the frame radiates (SSID `CR-RUST`, captured on mon0 ch1 alongside the blob's own
`CR-CTRL` control beacon). The reference implementation is `src/cleanroom.rs`. **The exercised
per-frame TX path no longer calls the blob at all** (see "The Rust modem-PM wake").

## Mechanism in one paragraph

The C6 MAC sits behind two gates that the modem power-management FSM operates. (1) While the modem
is asleep the Wi-Fi MAC/BB clocks are gated (`MODEM_SYSCON`/`MODEM_LPCON`, esp-radio's
`wifi_clock_enable(false)`): every MAC register write is silently dropped, so the `PLCP0_ENABLE`
valid|enable launch bits (`0xc0000000`) never latch — this is what looked like a "hardware
interlock". (2) `WDEV_PM_TXBLOCK_RETENTION` (0x600a4ca8) holds the PM TX-block bits: the blob's
`hal_mac_deinit` sets `|= 0x00ff1000` when the modem goes to sleep, and while they are set an armed
frame never launches (it latches but never completes). The 0x00ff1000 -> 0 transition is a plain
CPU store (`hal_mac_init`: `reg &= ~((mask & 0xff0000) | 0x1000)`) that only sticks once the clocks
are back, and the frame only completes/radiates once the PHY is re-enabled. The blob's
`pm_on_data_tx` (invoked from the AC-mapping step) is the per-frame FSM that does all three — clocks,
PHY, unblock store — and it is now Rust (`cr_pm_on_data_tx`). Our Rust reproduces the blob's
`ppTxPkt -> ppMapTxQueue(+pm wake) -> ppGetTxframe -> ppProcessTxQ -> lmacTxFrame -> hal_mac_tx`
sequence on the live scheduler structures — `our_instances` (per-AC lmac TX control, base
`*(0x4004ffe0)`) and `TxRxCxt` (per-AC pending lists, base `*(0x4087ff80)`); the eb packet pool,
`ppProcTxSecFrame` and the PM wake are Rust; the blob's own PM *sleep* side (a background timer)
and the PHY/clock substrate behind esp-radio's OSI callbacks are the remaining non-Rust pieces.

## WARNING — the fixed addresses are blob-version-specific (and some are link-layout-specific)

Every non-hardware address/offset in this file (the `our_instances` and `TxRxCxt` pointer slots,
`wDevCtrl_ptr` @ `*(0x4087ff68)` + the `g_if_enabled_mask` byte at `+0x31`, the `g_pm` field
offsets, the `esf_buf` pool list heads) is specific to the exact esp-wifi-sys blob build in use
(currently **0.3.0**) and to the on-chip ROM. They were re-derived from the 0.3.0 linked-ELF and ROM
disassembly and **must be re-derived per blob version**. A wrong base or offset does not fail loudly
— it silently reads/writes the wrong memory. This is exactly what bit us: `ic_interface_enabled` was
reimplemented against a stale address (the static `wDevCtrl` 0x40812090 + offset 0x29 instead of the
runtime pointer `*(0x4087ff68)` + 0x31), read 0, mis-classified every frame as "interface disabled",
and drove a drop-path double-free of the eb that corrupted the allocator heap. Verify addresses
against the disasm; do not port them blindly.

There are three kinds of address, and only two of them may be hardcoded:
- MMIO registers (`0x600aXXXX`): hardware, stable.
- ROM interface cells (`0x4087ffXX`: `g_osi_funcs_p`, `pp_wdev_funcs`, `g_ic_ptr`,
  `g_mac_sleep_en_ptr`, the `our_instances`/`TxRxCxt`/`wDevCtrl` pointer slots): fixed by the C6 ROM
  (`esp32c6.rom.*.ld`), stable across blob versions and link layouts.
- Blob globals that live in the linked `.data`/`.bss` (`g_pm`, `g_mesh_is_started`, `lmacConfMib`,
  `wDevCtrl`, the esf_buf list heads): their addresses move whenever the link layout changes —
  adding the Rust PM wake itself moved `g_pm` from 0x40820720 to 0x40820740 and then 0x40820730
  between three builds of the same source, and an older layout had it at 0x4081ea98. Hardcoding
  these is a silent-misread trap; they must be **linked** (`unsafe extern "C" { static mut g_pm: u8; }`
  + `addr_of_mut!`), which is what `cr_pm_on_data_tx` and `cr_lmacIsLongFrame` now do. Only the
  field *offsets* inside them are hardcoded (and those are blob-version-specific).

## The TX pipeline (Rust)

Per frame, `faithful_tx` mirrors the blob's raw-submit field setup on a freshly `esf_buf_alloc`'d eb,
then runs the Rust pipeline on the real scheduler state:

- `cr_ppTxPkt(eb)` — the submit. Guards with the Rust `cr_ic_interface_enabled` (per-VIF enable),
  runs `cr_ppTxProtoProc` (protocol flags), `cr_ppProcTxSecFrame` (Rust security-header reservation),
  `cr_rcGetSched` (no-op for a raw trc==0 beacon), then `cr_ppMapTxQueue`.
- `cr_ppMapTxQueue(eb)` — assigns the EDCA AC into `txinfo+0x10`, and runs the Rust
  `cr_pm_on_data_tx` (the PM-wake that makes the MAC active — the load-bearing ingredient; the blob
  `pm_on_data_tx` is kept only as a compile-time-disabled fallback). On success `cr_ppTxPkt`
  threads the eb onto the per-AC `TxRxCxt` pending list (`base + ac*0x34`, head `+0x20`, tail
  `+0x24`, linked via `eb+0x30`).
- `cr_ppProcessTxQ(ac)` — the schedule. Pops the head eb from the pending list (`cr_ppGetTxframe`),
  then `cr_lmacTxFrame`.
- `cr_lmacTxFrame` / `cr_lmacSetTxFrame` — the arm. Sets `our_instances[ac].cur_eb`, builds the PPDU
  via the Rust hal (`hal_mac_tx_config_timeout` + `hal_mac_tx_set_ppdu`, which run the Rust
  `mac_tx_set_plcp0/plcp1/txop_q` + `cr_mac_tx_get_rts_rate` + `cr_mac_tx_set_len` register ops),
  programs the EDCA backoff (`hal_mac_tx_config_edca`), and arms the slot (`hal_mac_txq_enable`
  sets `PLCP0_ENABLE |= 0xc0000000`). It deliberately does NOT set `our_instances[ac].state=1`, so
  the blob MAC-complete ISR skips our AC and we own the completion.
- `cr_complete(ac, eb)` — the completion. Polls `PLCP0_ENABLE` arm-clear, reads the completion
  result via the Rust `hal_mac_get_txq_complete`, clears the txq-state bit, and recycles the eb via
  the blob `esf_buf_recycle`. Exactly one recycle per alloc (the ic_interface_enabled fix removed the
  drop-path double-free).

The `mac_tx_set_plcp0` step also calls the Rust `cr_hal_he_set_tx_protection` (CONF0 protect bit) —
promoted from blob after the heap-corruption fix.

## Register / struct model (0.3.0 blob + C6 ROM)

Scheduler state (re-derive per blob version):
- `our_instances` per-AC lmac TX control block: base `*(u32*)0x4004ffe0`, stride `0x34`. Fields:
  `+0x00` cur_eb, `+0x05` aifsn, `+0x08` cw, `+0x12` state (0 idle / 1 armed / 5 success / 6 error),
  `+0x20` pending_head, `+0x24` pending_tail.
- `TxRxCxt` (pTxRx) per-AC pending lists: base `*(u32*)0x4087ff80`, per-AC `+ac*0x34`, head `+0x20`,
  tail `+0x24`, threaded via `eb+0x30`.
- `wDevCtrl`: base `*(u32*)0x4087ff68` (NOT the static link address); `g_if_enabled_mask` byte at
  `+0x31`, returns `(mask >> iface) & 1`.
- `lmacConfMib` (linked symbol; layout-dependent address): `+0x16` = RTS/long-frame threshold (u16).
- `g_pm` (linked symbol; layout-dependent address) field offsets on 0.3.0: `+1` PS state (0 active /
  1 / 2 sleeping, `pm_set_state`), `+2` PM interface, `+0xe` connected/PS-enabled, `+0xf`, `+0x24`,
  `+0x70/+0x74` coex slice start (u64), `+0x121` disconnected-modem state (2 awake / 3 asleep),
  `+0x1c2`/`+0x2e4` TWT flags, `+0x46a` (in `pm_get_tx_blocks_retention_mask`).
- ROM interface cells (fixed by the ROM): `g_osi_funcs_p` 0x4087ff6c (esp-radio's
  `wifi_osi_funcs_t`: `+4 _env_is_chip`, `+0xc8 _wifi_pm_sleep_lock_acquire`, `+0xd4 _phy_enable`,
  `+0xf8 _wifi_clock_enable`, `+0x108 _esp_timer_get_time`, `+0x190 _coex_status_get`,
  `+0x198 _coex_wifi_request`), `pp_wdev_funcs` 0x4087ff70 (`[0x94] pm_mac_wakeup`,
  `[0x95] ic_mac_init`), `g_ic_ptr` 0x4087ffa0 (`g_ic+0x24e` = rf_phy_enabled_mask byte),
  `g_mac_sleep_en_ptr` 0x4087ff00.
- `esf_buf` type-1 pool: alloc pops per-type head `0x40811b90 + type*0x14`; recycle pushes a
  different head (`g_eb_list_desc` 0x40811bc8 + type*0x14) reconciled by OSI callbacks.

WIFI MAC TX register file (hardware, stable): the `0x74`-stride block (completion/PMD/BA/txop) at
base `0x600a54xx`, and the `0x10`-stride per-slot config block at base `0x600a4d6x`. Queue index maps
to slots in REVERSE (queue 0 = highest slot), so per-queue addr = base - q*stride. Key regs:
`PLCP0_ENABLE 0x600a4d6c - ac*0x10` (arm bits `0xc0000000`, dma/length/format word), `PLCP1
0x600a5488 - q*0x74`, `RESP_DUR 0x600a54bc - slot*0x74`, `CONF0 0x600a4d60 - slot*0x10`, `PMD
0x600a54e8 - q*0x74`, and the `WDEV_PM_TXBLOCK_RETENTION` interlock at `0x600a4ca8`.

## The investigation (how we got here)

1. **The interlock.** The Rust arm programmed slot 0 correctly but the `PLCP0_ENABLE` valid|enable
   bits would not latch — CPU writes to them were dropped. The gate was localised to
   `WDEV_PM_TXBLOCK_RETENTION` (0x600a4ca8): `0x00ff1000` when blocked, `0` when active.
2. **MAC-active-state gating.** The launch write only sticks when the MAC is in the active state; a
   bare CPU strobe of the interlock register is not enough. The write-lock is near-fundamental —
   gated by the PM state machine, not by execution context alone.
3. **The `pm_on_data_tx` breakthrough.** Faithfully reproducing the blob's submit->schedule->arm
   *including* `pm_on_data_tx` (inside `ppMapTxQueue`) transitions the MAC active, the interlock
   clears, and the frame radiates. This is the load-bearing blob ingredient.
4. **Reimplementing the pipeline in Rust.** Each orchestrator (`ppTxPkt`, `ppMapTxQueue`,
   `ppGetTxframe`/enqueue, `ppProcessTxQ`, `lmacTxFrame`/`lmacSetTxFrame`, `cr_complete`) and the hal
   register ops were ported to Rust, driven directly on the real scheduler state (NOT via symbol
   interposition, which the placement-locked libpp lmac copies resist).
5. **The heap-corruption root cause.** A `linked_list_allocator` "hole list out of order" panic fired
   whenever unrelated code was added — a layout-sensitive symptom of a real double-free. Root cause:
   `cr_ic_interface_enabled` read a wrong 0.3.0 address (returned 0), so `cr_ppTxPkt` took its
   interface-disabled drop path and recycled the eb, then `cr_complete` recycled the same eb again.
   Fixing the address (call the value the ROM `ic_interface_enabled` computes: `*(0x4087ff68)+0x31`)
   made our real frame flow through and arm, and left `cr_complete` as the single recycle owner. This
   also un-blocked promoting `hal_he_set_tx_protection` to Rust.

## The independent Rust eb pool

`esf_buf` is de-blobbed: instead of the blob's callback-mediated two-list pool (which a bare Rust
recycle cannot feed — it drained after ~31 buffers), our loop owns a fixed pool of eb structures
end-to-end. `cr_pool_alloc` pops from our own free-list, `cr_pool_recycle` pushes back; the blob
`esf_buf` lists are never touched on the hot path.

The eb layout was reversed from a live blob type-1 eb (a single contiguous block, all internal
pointers self-relative), verified against the 0.3.0 disasm and a runtime dump:
- `+0x04` and `+0x08` both point to the same 3-word DMA descriptor at `+0x3c`
  (`[size|ctrl, frame_ptr, next]`); `ppProcTxSecFrame` reads eb+8 and eb+4 as the "two" descriptors —
  they are the same one.
- `+0x0c` = 1, `+0x10` -> aux (`+0x90`), `+0x16` = u16 payload length, `+0x1a` = u8 pool type (1),
  `+0x30` = pending next-link (the blob pending list uses this, so our free-list is a *separate* Rust
  index stack), `+0x34` -> txinfo (`+0x48`, 0x48 bytes), frame bytes at `+0xb8`.

`cr_pool_init` builds N=12 ebs in a `#[repr(C, align(16))]` static array in `.bss` (HP SRAM, DMA-
reachable like the blob ebs at `0x4087xxxx`), initialising each exactly as `esf_buf_alloc` does
(self-relative pointers, type, DMA descriptor, txinfo seed, payload copied into the frame region).
`cr_pool_alloc` re-copies the payload and restores the descriptor/length fields (which
`ppProcTxSecFrame` mutates in place) so a reused eb is handed out clean. Feasibility rested on one
check: nothing outside our loop references our eb — `ppProcTxSecFrame` and the hal use only stored
eb-field pointers (no `(eb - pool_base)/size` index arithmetic), and the blob MAC-complete ISR skips
our AC (we never set `our_instances[ac].state=1`), confirmed by `dblfree==0`.

Validated from a fresh flash: no drain over **1056 arms** (`arms==latched==completed==1056`,
`allocfail=0`, `poolfree=12` constant at every health print — the previous blob-fed pool drained at
31), `last_plcp0=0xc061d1fc` (our pool eb armed and radiating), no panic; OTA-confirmed on mon0 ch1
(binned beacons: CR-RUST present alongside CR-CTRL).

## The Rust ppProcTxSecFrame

`ppProcTxSecFrame` reserves a security header on the frame; it is REQUIRED for reliable emit (a
short/underlength frame latches but the baseband keys up only marginally, so the latch oracle alone is
not sufficient to validate it). `cr_ppProcTxSecFrame` is a faithful port of the 0.3.0 blob open/no-key
broadcast-beacon path (0x40804cd0), verified byte-for-byte against the disasm, feasible now that we
own the eb layout and know eb+4 and eb+8 are the same single descriptor:
- key type = `((*(txinfo+0x10) >> 8) & 0xf) - 1`; for the open no-key frame this is > 8, so the
  security-header length is 4 (the blob's `_LANCHOR32` table at 0x4200b100 is only read for keyed
  types).
- `eb+0x16 += 4`; the DMA descriptor length field (bits 14-27 of `dma[0]`) `+= 4`; `dma[0] |=
  0x40000000` (eof).
- guarded by `eb+0x24` bit13 (cleared on each `cr_pool_alloc`): frame pointer (`dma[1]`) `-= 8`;
  `eb+0x14 += 8`; `eb+0x24 |= 0x2000`; `dma[0]` length `+= 8`.
- the memset (args were unresolved before, resolved here): `memset(dma[1], 0, 8)` — zero the 8
  reserved header bytes at the shifted frame pointer; then write `(eb+0x14 + eb+0x16 - 8) & 0x3fff`
  into that first word (a length field the MAC reads). HE (flags bit31) and AMPDU-CCMP (flags &
  0x1040000) are separate blob branches the open beacon never takes and are skipped.

Validation (the critical OTA check, not just the latch oracle): from a fresh flash the on-device
oracle is perfect (`arms==latched==completed`, `allocfail=0`, `poolfree=12`, no panic), and a binned
mon0 capture shows CR-RUST radiating at a healthy steady rate — 26 CR-RUST vs 35 CR-CTRL over 72 s
(ratio ~0.74, matching the 4:6 attempt share), not a degraded trickle. The blob-`ppProcTxSecFrame`
control under the same RF gave the same-order ratio (a short direct comparison read 1:3 for both
builds), confirming the Rust version emits as reliably as the blob.

## The Rust modem-PM wake (pm_on_data_tx)

`pm_on_data_tx` was the last blob call on the exercised per-frame path. It is now Rust
(`cr_pm_on_data_tx`), and the earlier characterization ("the block clears as a hardware consequence,
no CPU store clears it") was wrong — the clear IS a CPU store, it is just the last step of a
clock/PHY re-enable sequence, and a bare store fails for a different reason (clock gating).

The decompiled chain (0.3.0 linked ELF; every offset verified in the disasm, Ghidra's program is a
different build and its prologue differs slightly):

- `pm_on_data_tx` @0x4080ef66 is `j pm_tx_data_process` @0x4080ece8. For our disconnected raw beacon
  it takes: TWT guard (`pm_is_twt_start` @0x4080c2ba: `g_pm+0x1c2 || g_pm+0x2e4`), mesh guard
  (`g_mesh_is_started`), `g_pm+2 == iface`, `pm_check_state` @0x4080971e (no-op while `g_pm+1 == 0`,
  which the disconnected FSM keeps it at), `g_pm+0xe == 0` (not connected/PS) ->
  `pm_is_in_wifi_slice_threshold(now, 5000)` (== 1 with no coex: `_coex_status_get` is 0) -> an
  optional `_coex_wifi_request(1, 0, ...)` if `now < g_pm+0x70/0x74` (never armed without coex) ->
  `pm_disconnected_wake`.
- `pm_disconnected_wake` @0x42020838: `if g_pm+0x121 == 3 && !mesh { wifi_rf_phy_enable(0);
  g_pm+0x121 = 2; pm_set_state(0) }`. `pm_set_state` @0x4202040a is `g_pm+1 = s` plus
  `wifi_gpio_debug`, a null-checked debug hook (`*(0x40811e08) == 0` -> `ret`).
- `wifi_rf_phy_enable` is the ROM routine @0x40016a68 (jump-table stub 0x40000ba8), a dispatcher:
  `mask = *(g_ic + 0x24e)` (`g_ic = *g_ic_ptr(0x4087ffa0)`); if `mask == 0`:
  `osi->_wifi_pm_sleep_lock_acquire` (+0xc8), `osi->_wifi_clock_enable` (+0xf8),
  [`pp_wdev_funcs[0x94]` = `pm_mac_wakeup` if `*(*g_mac_sleep_en_ptr(0x4087ff00))`, never set here],
  if `osi->_env_is_chip` (+4): `osi->_phy_enable` (+0xd4), then `pp_wdev_funcs[0x95]` = `ic_mac_init`
  @0x4080a6c8 -> `hal_mac_init` @0x4080bc6a: `WDEV_PM_TXBLOCK_RETENTION &=
  ~((pm_get_tx_blocks_retention_mask() & 0xff0000) | 0x1000)` (mask @0x42022cbe = 0xffffffff while
  disconnected, so this clears exactly 0x00ff1000); finally `mask |= 1 << mode`.
- Every OSI slot in that dispatcher is esp-radio **Rust**: `wifi_pm_sleep_lock_acquire` (no-op),
  `wifi_clock_enable` (`radio_clocks::enable_wifi(true)`: the `MODEM_SYSCON.clk_conf1` /
  `MODEM_LPCON.clk_conf` Wi-Fi clock gates), `env_is_chip` (true), `phy_enable`
  (`esp_phy::enable_phy_with_wifi_rx`). `cr_wifi_rf_phy_enable` dispatches through the same
  `g_osi_funcs_p` table (it is esp-radio's own static), which also keeps esp-radio's clock and PHY
  refcounts balanced against the blob's later `wifi_rf_phy_disable`.

The sleep side stays blob and is background PM, not per-frame: the blob's `pm_on_data_tx_done ->
pm_tx_data_done_process -> pm_enable_disconnected_sleep_delay_timer` (1 ms) ->
`pm_disconnected_sleep -> wifi_rf_phy_disable` (ROM) -> `ic_mac_deinit -> hal_mac_deinit`
(`|= 0x00ff1000`), `_phy_disable`, `_wifi_clock_disable`; `g_pm+0x121 = 3`. It is armed by the blob's
OWN tx-done (the CR-CTRL burst), never by our Rust completion, so in the reference build the modem is
asleep at the start of every round and `cr_pm_on_data_tx` wakes it exactly once per round
(`pm_wakes == rounds == pre_blk`, `post_blk == 0`), and with the burst disabled it wakes exactly once
per boot and the block stays 0 for the rest of the run.

What each gate does (fresh-flash experiments, on-device oracle, `CR_NO_BURST=1 CR_PM_EXP=<n>`):
- full wake: block clears, `latched == completed == arms`, radiates.
- no `hal_mac_init` store (`EXP=4`): block stays 0x00ff1000, the launch bits still LATCH
  (`last_plcp0=0xc061cd5c`) but `completed == 0` and nothing radiates -> the block bits gate the
  launch, not the register write.
- store only, MAC clock still gated (`EXP=3`): the store reads back 0x00ff1000 and the slot's launch
  bits do not latch either (`last_plcp0=0x0067a5f0`, `latched == 0`) -> every MAC write is dropped
  while the clock is gated; this is the "interlock" the earlier sessions saw when strobing the
  register.
- clocks + store, no `_phy_enable` (`EXP=2`): block clears (0), launch latches, `completed == 0`, no
  RF -> the PHY must be up for the MAC to finish a launch.

Validation of the Rust wake (fresh flash, reference build): boot snapshot `disc=3 blk=0x00ff1000`
(asleep); then `arms == latched == completed` (672+ arms), `allocfail=0`, `poolfree=12`,
`pm_wakes == rounds == pre_blk`, `post_blk=0`, `pm_odd=0` (no unmodelled PM state ever seen), no
panic; OTA on mon0 ch1 over 60 s: **90 CR-RUST vs 143 CR-CTRL** (0.63, against the 4:6 attempt
share = 0.67), both at -58.5 dBm, present in every 10 s bin. Same-RF A/B (`CR_AB=1`, the wake
alternating per round between Rust and blob inside one capture) shows the two indistinguishable in
count and RSSI. NOTE on OTA methodology: the absolute C6->sniffer level swung by >15 dB between
back-to-back fresh-flash runs in this session while the third-party AP reference stayed at
-53..-56 dBm and the blob's CR-CTRL path was hit identically — treat absolute counts across runs as
noise and compare only inside one capture (CR-CTRL reference or the per-round A/B).

## Slot-0 contention: why the CR-CTRL beacon must not be interleaved

`faithful_tx` drives AC0/slot0 directly (arm + own completion) on the assumption that the blob
pp/`ppTask` is idle. That assumption holds only while nothing else submits to the blob TX path. A
`send_raw_frame` — which is exactly what the `CR-CTRL` reference beacon is — KICKS the blob `ppTask`,
which then also schedules onto slot 0 and races our direct arm + completion on the same slot and the
same shared `TxRxCxt` pending list. The result collapses BOTH beacons.

Measured (same RF, same session, device confirmed healthy — pure blob 146 frames/20s @ -68 dBm):
- our Rust pipeline alone (no interleaved CR-CTRL): **131–236 frames @ -69 dBm, steady** — matches
  the blob.
- the blob CR-CTRL beacon alone (`CR_TX_MODE=1` control-only, our pipeline off): 96 frames @ -69 dBm.
- the two interleaved every round (the old default): **1–3 frames each @ -82 dBm** — both collapse.
- the collapse is independent of our PM wake (it happens with `CR_BLOB_WAKE=1` too), so it is the
  slot-0 / ppTask race, not the wake. The interleaving also arms the blob's disconnected-sleep timer,
  so the modem additionally thrashes sleep/wake every round (`pm_wakes == rounds`), whereas with the
  pipeline exclusive the modem wakes once and stays awake (`pm_wakes == 1`, block stays 0).

This was the "weak and erratic TX" symptom (our firmware transmitting at -81 dBm / 2-3 frames while
the pure blob was healthy): it was NOT device state and NOT TX power — both firmwares run the same
`WifiController::new` (which ends with `esp_wifi_set_max_tx_power(20)`), so the PHY TX-power/gain
state is identical, and the on-device oracle showed `arms == latched == completed` at full rate
throughout (the modem was never rate-collapsing). It was our own pipeline contending with the
interleaved blob beacon on slot 0.

Fix: the per-round CR-CTRL burst is OFF by default — the 10 CR-CTRL beacons are emitted only once at
boot, before our loop starts (so `ppTask` has gone idle by the time we drive slot 0), as an RF
baseline. A continuous same-RF blob reference is fundamentally incompatible with our direct-slot
pipeline (they share slot 0), so it is available only as `CR_TX_MODE=1` (control-only: our pipeline
disabled, blob beacon only) for a clean blob-vs-blob RF check, or `CR_BURST=1` / `CR_AB=1` for the
deliberately-contended wake A/B. Default validated from a fresh flash: 236 CR-RUST frames over 58 s
at -69.3 dBm, present in every 10 s bin, `pm_wakes=1`, `arms==latched==completed`, no collapse.

## Minimal remaining blob surface

The exercised per-frame TX path is entirely Rust. Everything below is either Rust now, kept blob
only as a compile-time-disabled fallback, background (not per-frame) PM state, or in a branch the
1 Mbit DSSS beacon never reaches.

Substrate leaves:
- `pm_on_data_tx` — NOW RUST (`cr_pm_on_data_tx`, see "The Rust modem-PM wake"). The blob extern is
  kept only as the compile-time-disabled fallback (`PM_WAKE_RUST = false` / `CR_BLOB_WAKE=1`). What
  remains non-Rust behind it is substrate reached through esp-radio's own OSI callbacks: the libphy
  `phy_wakeup_init`/calibration inside `esp_phy::enable_phy`, and the blob's background PM sleep
  timer (`pm_disconnected_sleep`) that re-blocks the MAC after the blob's own TX.
- `esf_buf_alloc` / `esf_buf_recycle` — NOW RUST (see "The independent Rust eb pool" below). The blob
  esf_buf is a callback-mediated two-list pool that a bare-freelist Rust recycle cannot feed (it
  drained after ~31 buffers). Instead our loop uses its OWN fixed pool of correctly-laid-out eb
  structures end-to-end; the blob esf_buf externs are kept only as a compile-time-disabled fallback.

hal_mac_tx.o leaves:
- `ppProcTxSecFrame` — NOW RUST (`cr_ppProcTxSecFrame`, open/no-key beacon path; see "The Rust
  ppProcTxSecFrame" below). Proven required for reliable emit, and OTA-confirmed to emit as reliably
  as the blob. The blob extern is kept only as a compile-time-disabled fallback.
- `mac_tx_set_pti` — NOW RUST (`cr_mac_tx_set_pti`). It is called unconditionally by
  `hal_mac_tx_set_ppdu` (so it IS on the exercised path), and reduces to `hal_set_tx_pti` clearing the
  CONF1 top nibble (0x600a4d68 - slot*0x10) and packing pti (txinfo+0x20) / txinfo+0x22 into the PTI
  register (0x600a5490 - slot*0x74); the blob's coex callback only clamps pti downward and is inert
  with no BT active. OTA-confirmed unchanged. HE-only leaves `mac_tx_set_hesig`, `mac_tx_set_htsig`,
  `hal_mac_fill_hwtxop` stay blob but are genuinely never reached by the 1 Mbit DSSS beacon (they sit
  in the OFDM/HT/HE and aggregate branches).

lmac.o: the ~15 ROM *ABS* lmac symbols are not interposable, and the libpp lmac copies are
placement-locked (a byte-identical copy at a different address stalls TX); only the empty
`lmac_update_tx_statistic` is safely reimplemented (as a no-op). See the appendix.

`hal_now` is now Rust: the blob reads the free-running WDEV system timer at 0x600ad000 (a single
`lw`, verified against the 0.3.0 disasm of hal_now @0x4202c6e2); `cr_hal_now` is a direct volatile
read used as the TX submit timestamp. The two former hal config delegations are also gone:
`hal_mac_tx_config_timeout` no longer branches to the blob (its only other path is the `esp_test`
EDCA-disable bypass, never enabled — the blob itself takes the register-write path our Rust
reproduces; the old code even read a wrong 0.3.0 address, 0x4208318c vs the blob's 0x408216c8, for
that dead check), and `hal_mac_tx_clr_mplen` is a pure no-op for legacy frames (the blob only acts
when CONF1 bit3 = HE-TB, which our DSSS beacon never sets). Everything on the per-frame TX path —
submit, AC-map, PM wake, enqueue/pop, schedule, arm (plcp0/plcp1/txop_q/rts_rate/set_len/
he_protection), and completion — is Rust.

## Appendix — session log

Condensed timeline of the investigation (full blow-by-blow is in git history):
- 9a/9b: clean-room arm built; radiation blocked; interlock localised to WDEV_PM_TXBLOCK_RETENTION.
- 9c: write-lock shown to be MAC-active-state gated (near-fundamental).
- 9d: BREAKTHROUGH — faithful reproduction incl. pm_on_data_tx radiates CR-RUST.
- 9e/9f: orchestrators + completion path reimplemented in Rust; pool stays healthy.
- 9g: env recovery onto the 0.3.0 blob; pm_on_data_tx identified as the crown-jewel substrate.
- 9h: guard leaves in Rust; ppProcTxSecFrame confirmed required (must stay blob).
- 9i: hal register helpers (rts_rate, set_len) in Rust; he_protection/pti deferred.
- 9j: root-caused + fixed the layout-sensitive heap corruption (the ic_interface_enabled double-free).
- 9n: hal_now, the two hal config delegations, and mac_tx_set_pti reimplemented/removed in Rust;
  OTA-confirmed. The only exercised-path blob call left is pm_on_data_tx (the modem-PM wake).
- 9m: ppProcTxSecFrame reimplemented in Rust (open/no-key beacon path, memset resolved); OTA-confirmed
  to emit as reliably as the blob.
- 9l: esf_buf de-blobbed — an independent Rust eb pool (own free-list, blob esf_buf untouched on the
  hot path); no drain over 1056 arms, OTA-confirmed.
- 9k: ic_interface_enabled reimplemented in Rust with the correct 0.3.0 address; esf_buf pool and
  pm_on_data_tx characterized and kept blob; consolidation + refactor into this reference.
- 9o: pm_on_data_tx reimplemented in Rust (cr_pm_on_data_tx): the wake chain decompiled to the ROM
  wifi_rf_phy_enable dispatcher + hal_mac_init store; the two gates (clock gating drops MAC writes;
  the PM block bits stop the launch) separated by experiment; g_pm/g_mesh_is_started/lmacConfMib
  switched from hardcoded to linked symbols after the layout shift bit. OTA-confirmed. The exercised
  per-frame TX path is now blob-call-free.
