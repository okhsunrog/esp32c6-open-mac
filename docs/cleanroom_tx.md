# ESP32-C6 clean-room Rust Wi-Fi TX pipeline

A pure-Rust demonstrator of the ESP32-C6 Wi-Fi MAC transmit path, built on top of the working
esp-radio blob substrate. Our Rust code drives the whole per-frame TX pipeline — submit, AC-map,
pending enqueue/pop, schedule, arm, and completion — on the **real** blob scheduler state, and the
frame radiates (SSID `CR-RUST`, captured on mon0 ch1 alongside the blob's own `CR-CTRL` control
beacon). The reference implementation is `src/cleanroom.rs`.

## Mechanism in one paragraph

The C6 MAC gates TX launch behind a hardware interlock, `WDEV_PM_TXBLOCK_RETENTION` (0x600a4ca8):
while the modem is in power-save it reads `0x00ff1000` and the `PLCP0_ENABLE` valid|enable launch
bits (`0xc0000000`) will not latch, so a CPU-programmed slot never keys up the PHY. The blob
`pm_on_data_tx` PM-wake FSM (invoked from the AC-mapping step) transitions the MAC to the active
state; only then does the interlock clear and the launch bits latch. Our Rust reproduces the blob's
`ppTxPkt -> ppMapTxQueue -> ppGetTxframe -> ppProcessTxQ -> lmacTxFrame -> hal_mac_tx` sequence on the
live scheduler structures — `our_instances` (per-AC lmac TX control, base `*(0x4004ffe0)`) and
`TxRxCxt` (per-AC pending lists, base `*(0x4087ff80)`) — calling the blob only for a small,
documented set of substrate leaves (chiefly `pm_on_data_tx` itself and the `esf_buf` packet pool).

## WARNING — the fixed addresses are blob-version-specific

Every non-hardware address/offset in this file (the `our_instances` and `TxRxCxt` pointer slots,
`wDevCtrl_ptr` @ `*(0x4087ff68)` + the `g_if_enabled_mask` byte at `+0x31`, `lmacConfMib`, the
`esf_buf` pool list heads) is specific to the exact esp-wifi-sys blob build in use (currently
**0.3.0**) and to the on-chip ROM. They were re-derived from the 0.3.0 linked-ELF and ROM
disassembly and **must be re-derived per blob version**. A wrong base or offset does not fail loudly
— it silently reads/writes the wrong memory. This is exactly what bit us: `ic_interface_enabled` was
reimplemented against a stale address (the static `wDevCtrl` 0x40812090 + offset 0x29 instead of the
runtime pointer `*(0x4087ff68)` + 0x31), read 0, mis-classified every frame as "interface disabled",
and drove a drop-path double-free of the eb that corrupted the allocator heap. Verify addresses
against the disasm; do not port them blindly.

## The TX pipeline (Rust)

Per frame, `faithful_tx` mirrors the blob's raw-submit field setup on a freshly `esf_buf_alloc`'d eb,
then runs the Rust pipeline on the real scheduler state:

- `cr_ppTxPkt(eb)` — the submit. Guards with the Rust `cr_ic_interface_enabled` (per-VIF enable),
  runs `cr_ppTxProtoProc` (protocol flags), calls the blob `ppProcTxSecFrame` (security-header
  length work — kept blob), `cr_rcGetSched` (no-op for a raw trc==0 beacon), then `cr_ppMapTxQueue`.
- `cr_ppMapTxQueue(eb)` — assigns the EDCA AC into `txinfo+0x10`, and calls the blob `pm_on_data_tx`
  (the PM-wake that makes the MAC active — the load-bearing ingredient). On success `cr_ppTxPkt`
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
- `lmacConfMib` @ 0x40811ca8 (0.3.0): `+0x16` = RTS/long-frame threshold (u16).
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

## Minimal remaining blob surface

The per-frame TX *logic* is entirely Rust. What remains blob, with reasons:

Substrate leaves (the fully-blob-free frontier — characterized, kept blob):
- `pm_on_data_tx` — the modem-PM wake FSM. `WDEV_PM_TXBLOCK_RETENTION` is only ever SET by the PM
  code; no CPU store clears `0x00ff1000 -> 0`. The wake clear is a hardware consequence of
  `wifi_rf_phy_enable` + the `g_pm` FSM reaching the wake state, so it cannot be replayed as a
  register write — it needs the full g_pm state machine + PHY enable in order.
- `esf_buf_alloc` / `esf_buf_recycle` — a callback-mediated two-list eb pool (alloc pops one per-type
  head, recycle pushes a different one; the two are reconciled by OSI callbacks). A bare-freelist
  Rust recycle radiates and latches fine but drains the pool after ~31 buffers, so the buffer return
  depends on the callback machinery. Kept blob.

hal_mac_tx.o leaves:
- `ppProcTxSecFrame` — the security-header dual-descriptor length adjustment; proven required for
  reliable emit.
- `mac_tx_set_pti`, `mac_tx_set_hesig`, `mac_tx_set_htsig`, `hal_mac_fill_hwtxop` — PTI/HE/aggregate
  helpers on the PPDU-build path that the legacy 1 Mbit DSSS beacon does not exercise.

lmac.o: the ~15 ROM *ABS* lmac symbols are not interposable, and the libpp lmac copies are
placement-locked (a byte-identical copy at a different address stalls TX); only the empty
`lmac_update_tx_statistic` is safely reimplemented (as a no-op). See the appendix.

`hal_now` (WDEV TSF timer read) is the one remaining trivial blob/ROM helper on the hot path. Two
Rust hal ops also still delegate a config path to the blob (`blob_hal_mac_tx_config_timeout` for the
non-beacon timeout case, `blob_hal_mac_tx_clr_mplen` for the mplen-bitmap teardown); the beacon path
itself is Rust. Everything else on the per-frame TX path — submit, AC-map, enqueue/pop, schedule,
arm (plcp0/plcp1/txop_q/rts_rate/set_len/he_protection), and completion — is Rust.

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
- 9k: ic_interface_enabled reimplemented in Rust with the correct 0.3.0 address; esf_buf pool and
  pm_on_data_tx characterized and kept blob; consolidation + refactor into this reference.
