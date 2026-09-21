# ESP32-C6 clean-room Rust TX arm path — session 9 (2026-09-11)

First pass at a SELF-CONTAINED Rust TX path: rather than interposing our code between the
blob's ppProcessTxQ->lmacTxFrame call graph (session-8 finding: that regresses/stalls), we drive
the full lmacTxFrame-equivalent ARM sequence from OUR OWN task, on top of the working esp-radio
substrate (chip/PHY/clock init, OS adapter, WifiController::new + start — all radiate). Reuses the
proven Rust `hal_mac_tx` functions (BATCH1-8, all validated radiating) as the HAL layer.

Bin: `src/cleanroom.rs` (built in esp-hal examples/wifi/80211_tx as `src/bin/cleanroom.rs`).
Build: `cargo build --release --bin cleanroom --target riscv32imac-unknown-none-elf --features esp32c6`.

## What the clean-room path does (milestone 1 — BUILT)

1. esp-radio brings everything up (same as deblob.rs); `set_power_saving(None)` keeps the modem awake.
2. A CONTROL beacon (SSID `CR-CTRL`, blob path via `send_raw_frame`) runs as a same-RF positive
   reference — it goes through esp_wifi_80211_tx -> ieee80211 -> ppTxPkt -> ppTask -> ppProcessTxQ
   -> lmacTxFrame -> (our Rust hal) -> MAC, i.e. the proven radiating path.
3. OUR OWN arm path (SSID `CR-RUST`), driven from the main task, NOT via the blob scheduler:
   - `eb = esf_buf_alloc(payload, 1, len)` — a correctly DMA-placed eb from the LIVE libpp static-TX
     pool. In the de-blob link `esf_buf_alloc` resolves to libpp `0x4080acf4` (NOT the ROM *ABS*
     copy), so the eb is consistent with the running pools. CRITICAL: `set_plcp0` feeds the
     dma_desc's low-20 bits into PLCP0_ENABLE, so the eb MUST live in the MAC-DMA region — hand-
     rolling one in DRAM would mis-map; esf_buf_alloc places it correctly.
   - Fill dma_desc + txinfo EXACTLY as `ieee80211_output_raw_process` does (owner|eof|length; cat=7;
     iface bit19; AC bits20-23; tsf=hal_now; broadcast 0x402; rate idx 0 = 1 Mbit DSSS; seqno),
     minus the ppTxPkt submit — so our eb is structurally identical to a blob-built beacon eb.
   - A private 0x40-byte fake `txq` context (only the fields the hal fns read: [0]=eb, [4]=slot,
     [5]=aifsn, [6]=backoff, [8]=cw, [0x1d]=depth). We NEVER touch `our_instances`, so the blob's
     `lmacProcessTxComplete` (guarded on state==1) skips our slot and never recycles our eb.
   - Arm = the lmacSetTxFrame + lmacTxFrame essentials, calling the proven Rust hal fns:
     `hal_mac_tx_config_timeout` -> `hal_mac_tx_set_ppdu` (plcp0/plcp1/rate/dur/len/txop/pti) ->
     `hal_mac_tx_config_edca` -> `hal_mac_txq_enable` (PLCP0_ENABLE |= 0xc0000000).
   Control and test run in ALTERNATING phases (10 blob beacons, then 10 of our arms) so they never
   contend for the slot and the pcap bins cleanly by SSID.

## Hardware result (milestone 2 — CHARACTERISED, does NOT radiate; blocker localised)

### The C6 MAC exposes exactly ONE writable TX slot bank
Write/read probe of all 8 slot config banks (PLCP0_ENABLE 0x600a4d6c-slot*0x10, EDCA 0x600a4d68-
slot*0x10):
```
slot0 plcp0=0x0067a4c8 edca=0x020023ff writable=true    <- the blob beacon's slot (AC 0)
slot1..7 plcp0=0x00000000 edca=0x00000000 writable=false <- writes DROPPED, reads 0
```
So a "dedicated free slot" is not available: the C6 MAC only accepts register writes to the slot
whose bank the scheduler has ACTIVATED. (`ppMapTxQueue` maps the raw beacon to AC 0 — confirmed.)
=> we must drive slot 0 itself, time-multiplexed with a quiesced blob beacon.

### On slot 0, our arm programs everything correctly EXCEPT the valid|enable latch
Driving our arm on slot 0 (blob beacon quiesced), immediate readback after `hal_mac_txq_enable`:
```
enable_readback=0x0067a5c8 | arms=0x00 plcp0=0x0067a5c8 edca=0x020023ff pmd=0x00000000
```
- The CONFIG bits latch perfectly: plcp0 low = 0x0067a5c8 and edca = 0x020023ff are essentially
  identical to the blob's own (0x0067a4c8 / 0x020023ff) — our PLCP0/PLCP1/EDCA/rate programming from
  the eb matches the blob byte-for-byte (differences are just the seqno/length of our frame).
- The ARM bits DO NOT LATCH: even the FIRST bus cycle after `PLCP0_ENABLE |= 0xc0000000` reads back
  WITHOUT bits 30/31. No completion is ever recorded (pmd = 0). The MAC silently drops the write of
  the transmit-enable bits (30/31) while accepting the config bits (0-29) of the SAME register.

### Proof the enable bits ARE settable on this exact register — in the blob's context
Tight-polling PLCP0_ENABLE right after `send_raw_frame` (the blob's own in-context arm) CATCHES it:
```
in-context blob arm poll: arm_bit_caught=true max_plcp0=0xc067a5c8
```
i.e. when the BLOB scheduler executes the identical single write (`PLCP0_ENABLE |= 0xc0000000`,
verified as the ONLY hardware write in blob `hal_mac_txq_enable` @disasm — the rest is HE-TB/muedca
software bookkeeping), bits 30/31 latch (0xc067a5c8) and the frame radiates. The low bits it latches
(0x67a5c8) are the SAME value our arm writes.

### Radiation ground truth (mon0 ch1, same firmware, same RF, same slot 0)
`CR-CTRL` (blob path): radiates steadily. `CR-RUST` (our arm path): 0 frames — never radiates.
120s alternating-phase binned capture (30s bins):
```
           0-30s  30-60s  60-90s  90-120s   total
CR-CTRL      135     138     132      132      537   (steady, same-RF positive control)
CR-RUST        0       0       0        0        0   (our Rust-driven arm, never radiates)
```

## What this rules in / out (the "timing/runtime coupling", localised)

RULED OUT as the gate for the valid|enable latch:
- Missing register writes — blob `hal_mac_txq_enable` does ONLY `|=0xc0000000`; our reimpl matches.
- Config programming — our PLCP0/PLCP1/EDCA/rate/timeout all latch and match the blob's values.
- The coex grant — calling `pp_coex_tx_request(eb)` (the request the blob issues from ppProcessTxQ
  before lmacTxFrame) BEFORE our arm made NO difference (enable still drops). Confirms the annotation
  that coex fns return 0 / never block on this WiFi-only build.
- CCA / TSF / EDCA-config (session-7 negatives) — independently re-confirmed here.

REMAINING HYPOTHESIS (the coupling): PLCP0_ENABLE[31:30] is a write-gated "transmit-launch" latch
that the MAC only opens transiently as part of the scheduler's arming, driven from ppTask/MAC-ISR
context after the submit path (ppTxPkt -> pp_post -> ppProcessTxQ) has put the MAC TX state machine
into a "tx-requested" phase. A byte-identical CPU write of bits 30/31 from an independent task, with
the config bank fully and correctly programmed, is silently dropped. This is a HARDWARE write-enable
interlock on the launch bits, not a software/register-value gap — it extends the 7-session verdict
("BB only keys up under the running scheduler") to the finest granularity: it is not that the BB
ignores a set enable bit, it is that the enable bit itself will not LATCH outside the scheduler's
live arming context.

## Session 9b: lead 1 pursued — the interlock is WDEV_PM_TXBLOCK_RETENTION (hardware/PM-driven)

Hunted for a "tx-requested" strobe that opens the PLCP0_ENABLE[31:30] write-enable. Result: there
is NO CPU-writable strobe on our path; the gate is a hardware PM TX-block register we cannot write.

STATIC (full submit->arm MMIO enumeration):
- `ppTxPkt` and `ppProcessTxQ` write NO WDEV MMIO on the per-frame path — only SRAM queue
  bookkeeping, ppMapTxQueue (AC bits in txinfo), pp_coex_tx_request, and pp_post. The only MMIO read
  is `_WDEV_TSF0_TIMER_LO` into txinfo+0x18. So the ENTIRE submit->arm MMIO write set is exactly
  `lmacSetTxFrame` + `lmacTxFrame` + hal callees — which our cleanroom already replicates. There is
  no extra per-frame register write we omit.
- `hal_mac_txq_enable`'s only hardware write (disasm) is `PLCP0_ENABLE |= 0xc0000000`; rest is
  HE-TB/muedca software bookkeeping.

DYNAMIC — the differentiator is WDEV_PM_TXBLOCK_RETENTION (0x600a4ca8):
- Reading it right before our (failing) arm: `txblock_before = 0x00ff1000` — TX is BLOCKED
  (bit12 0x1000 + per-queue block 0xff0000). `hal_mac_init` clears `~(pm mask & 0xff0000 | 0x1000)`;
  PM paths (pm_off_channel, pm_coex_slice_timeout) SET 0xe0000; so these bits are the TX-block gate.
- Catching the blob's own in-context arm (tight poll of PLCP0_ENABLE right after send_raw_frame):
  `arm_bit_caught=true plcp0=0xc067a5c8 txblock_at_arm=0x00002000` — when the blob launches, the
  block bits are CLEARED and bit13 (0x2000 = "txing", per hal_mac_deinit's `>>0xd&1`) is set.
- We CANNOT clear the block from our context. Writing `0` (or any pattern) to 0x600a4ca8 and reading
  back: `0x00ff1000->0x00ff1000`. Per-bit probe in the same MAC state: set-b0, set-b10, clear-b12,
  write-0 ALL read back `0x00ff1000` unchanged. The register is fully WRITE-LOCKED from our task —
  exactly like the PLCP0_ENABLE launch bits. (Same RMW `hal_mac_init` uses, so calling it would be
  identical and futile.)
- The unblocked state is not a usable idle window: polling the block register for 60000 samples
  right after a submit, `unblocked_samples=12172/60000 first_reblock@12172` — it is unblocked only
  during the blob's own in-flight beacon (~the TX duration, slot 0 busy), then RE-BLOCKS and stays
  `0x00ff1000`. The unblock is a CONSEQUENCE of the scheduled TX (HW/PM sets "txing"), not a
  precondition we can assert or exploit on an idle slot.

Verdict for lead 1: the PLCP0_ENABLE launch-enable interlock is gated by the MAC's PM TX-block FSM
(WDEV_PM_TXBLOCK_RETENTION). Both the block bits and the launch bits reject CPU writes from our
non-scheduler context, and the block is un-set only by the hardware/PM as part of the in-context
submit->schedule->TX flow (coinciding with the scheduler owning the slot). No reproducible CPU-side
strobe opens the interlock. This is concrete, register-level evidence for the lead-2 branch: a
clean-room TX must reproduce the hardware "TX-allowed/txing" state the scheduler+PM chain sets (and
the context in which these control-register writes are accepted), not merely issue the enable write.
CR-RUST remains 0 on mon0; CR-CTRL steady (same firmware/RF).

## Session 9c: leads 2a + 2b — the write-lock is MAC-ACTIVE-STATE gated (near-fundamental)

2a asked: is the control-register write-lock gated by execution CONTEXT (ppTask/privilege) or by a
MAC-STATE precondition? Answer, with a sharp register oracle: it is MAC-ACTIVE-STATE gated.

- In ppTask context (probe embedded in our interposed hal_mac_txq_enable, which the blob calls on
  ppTask during its OWN slot-0 beacon arm) the writes that our normal task could not make now STICK:
  WDEV_PM_TXBLOCK_RETENTION bit-flip 0x0000->0x0001 stuck; an idle slot's EDCA 0x0000->0x0abc stuck
  (dead from our task); idle slot 1 PLCP0_ENABLE[31:30] LATCHED. Driving our full arm there set
  slot-0 PLCP0_ENABLE to 0xc067a4c8 (launch bits latched, descriptor base correct).
- DECISIVE disambiguation (from our OWN normal task, no interposition): probe write-stickiness in the
  ~active window right after send_raw_frame (WDEV_PM_TXBLOCK_RETENTION != 0x00ff1000, i.e. the MAC is
  keying up the blob's just-submitted beacon): `txblock=0x00000000 bit0_stuck=true
  slot1_launch_latched=true`. So writes STICK from our normal task too, as long as the MAC is ACTIVE.
  => it is NOT execution-context / privilege that gates the control-register writes; it is the MAC TX
  clock/active state. When the MAC is idle (block 0x00ff1000) the writes are dropped from ANY context;
  when it is active (block cleared, txing) they stick from ANY context.

2b (the PM-unblock path): the block register is written (cleared) by CPU only in hal_mac_init and
hal_pm_unblock_txq (0xe0000); NO software on the per-beacon submit->arm path clears the full
0x00ff1000. So the per-beacon 0x00ff1000 -> 0x00002000 transition (block cleared, bit13 "txing" set)
is a HARDWARE-FSM consequence of the MAC entering active TX for a scheduled frame, not a standalone
CPU write. Once the FSM has made the MAC active, CPU writes to the control registers (block bits AND
launch bits) are accepted — that is exactly what 2a measured.

Did CR-RUST radiate? No. We could LATCH the launch bits in-context/in-window (slot 1: latched;
slot 0 overwrite: 0xc067a4c8 with correct descriptor base after matching the blob's txinfo so
mac_tx_set_txop_q keeps PLCP0_ENABLE bit22), but: (i) slot 1 is not a serviceable TX queue (launched,
never emitted); (ii) overwriting slot 0 in-context either re-sent the blob's own frame (wrong bit22
-> doubled CR-CTRL) or went silent (correct bit22 -> our frame's DMA still not emitted), and a
per-beacon in-context arm destabilised the blob's TX entirely (timing-sensitive-path lesson); a
one-shot 20-beacon window stayed stable (CR-CTRL 421/healthy) but still produced no CR-RUST. Getting
an independent frame onto the air needs a correctly-staged DMA on a serviceable slot, which slot 0
only is — and slot 0 is owned by the scheduler whose TX is what makes the MAC active in the first
place.

FINAL VERDICT on clean-room C6 TX feasibility: the PLCP0_ENABLE launch-enable (and the PM TX-block)
are writable ONLY while the MAC TX clock/active state is up, and that state is entered ONLY as a
hardware-FSM consequence of a frame scheduled through the blob's submit->schedule flow (which also
owns the one serviceable slot). This is a chicken-and-egg / near-fundamental barrier for an
idle-start clean-room arm: there is no CPU-reachable write or sequence from an idle MAC that opens
the interlock. A clean-room TX would have to reproduce the hardware path that brings the MAC to the
active/txing state (the full scheduler+PM+queue bring-up that makes the block clear and the launch
bits latchable), not merely issue the register writes. The write-lock being MAC-state (not context)
gated is the key new datum: once the MAC is active, our own task CAN write the launch bits — the
remaining wall is bringing the MAC to that active state without the blob scheduler, and getting a
frame staged on the single serviceable slot.

## Session 9d: BREAKTHROUGH — faithful reproduction radiates CR-RUST

The honest test: reproduce the blob's submit->schedule->arm on the REAL our_instances state (not a
fake context), with a faithfully-built eb, and let the MAC decide. It WORKS — our frame radiates.

The path that radiates (from our own task, blob pp idle / no send_raw_frame during the faithful
phase):
- `eb = esf_buf_alloc(rust_beacon, 1, len)` — real static-TX pool eb.
- Faithful eb setup mirroring ieee80211_output_raw_process: dma_desc owner|eof|length, txinfo cat=7,
  tsf=hal_now, iface 0, broadcast 0x402, rate idx 0 (1 Mbit DSSS), seqno, and eb+0x2c (trc)=0.
- `ppTxPkt(eb, 0)` — the blob's REAL submit: ppTxProtoProc / ppProcTxSecFrame / rcGetSched /
  ppMapTxQueue + ENQUEUE onto the REAL our_instances[ac] pending list. kick=0 so pp_post is NOT
  issued and the blob's ppTask stays parked — WE drive the schedule.
- `ppProcessTxQ(ac)` (ac=0, the AC ppMapTxQueue assigned) — the blob's REAL schedule: pop from the
  pending list + pp_coex_tx_request + lmacTxFrame (arm) on the REAL our_instances[0] state, driving
  the arm through our proven Rust hal_mac_tx layer (config_edca / set_ppdu / hal_mac_txq_enable).
- Completion is serviced by the blob MAC ISR (wDev_ProcessFiq -> lmacProcessTxComplete): our
  faithful_cleanup finds state already back to 0 (completed=true), no wedge.

Register oracles at each faithful arm (every iteration, steady):
```
ac=0 txpkt_ret=0  block 0x00000000->0x00000000  plcp0 0x0067a4c8 -> 0xc067a5c8  completed=true
```
i.e. the MAC is ACTIVE (0x600a4ca8 = 0, not the idle 0x00ff1000), the launch bits LATCH
(PLCP0_ENABLE bit31|bit30 set), and the frame COMPLETES. (It re-blocks to 0x00ff1000 between
iterations and is re-activated by the next ppTxPkt.)

Radiation (mon0 ch1, faithful phase 4 arms/round + occasional CR-CTRL control 6/round, 30s bins):
```
          bin0 bin1 bin2 bin3   total   source MAC
CR-RUST     6    7    8    0       21    02:00:00:00:c6:00  (our faithful Rust path, 1 Mbit DSSS)
CR-CTRL    11   13    9    3       36    02:00:00:00:da:b0  (blob beacon control)
```
CR-RUST radiates steadily, proportional to its attempt share (4:6 vs the control). (bin3 taper is the
AX210 monitor vif degrading, not TX; rebuilt via tools/mon_up.sh.)

Why the earlier cleanroom arm never emitted, now explained: the MAC-active transition (block
0x00ff1000 -> 0, which unlocks the launch/PM-block writes — the session-9c finding) is triggered by
the PM wake on the SUBMIT path — `pm_on_data_tx`, called from `ppMapTxQueue` inside `ppTxPkt` — NOT
by the arm itself. Our fake-context cleanroom arm skipped ppTxPkt, so pm_on_data_tx never ran, the
MAC stayed blocked, and the launch bits could not latch. The faithful ppTxPkt restores the PM-wake +
the real eb/DMA/our_instances linkage, the MAC goes active, and the arm (through our Rust hal) fires
the PHY.

VERDICT: clean-room C6 TX from Rust IS feasible on top of the working esp-radio substrate. The
missing ingredient was faithfulness of the submit+schedule on REAL scheduler state (the PM-wake and
the real eb/pending linkage), not the arm register writes (those were already our proven Rust hal).
Remaining work to a pure-Rust path: reimplement ppTxPkt (incl. the pm_on_data_tx PM-wake + the
our_instances enqueue) and ppProcessTxQ/lmacTxFrame in Rust rather than calling the blob copies; the
hardware behaviour is now known to follow once the real state + PM-wake are reproduced.

## Next leads (for a future pass)
- Whether the MAC active/txing state can be forced/held independent of a scheduled frame (a clock or
  PM-wake write that sets bit13 and clears the block and STAYS), which 2a shows would then make the
  launch bits writable from our own task; and whether a second serviceable TX slot can be activated.
- Find what makes the PM FSM accept control-register writes and clear the TX-block in-context:
  trace the PM path from frame-submit/pp_post through the tbtt/wake hooks to the 0x600a4ca8 clear;
  the clear is NOT a plain per-frame CPU write (none exists on the path), so it is either a HW
  response to a scheduled-TX request or a PM-wake write accepted only in a specific MAC/PM state.
- Test issuing the whole arm from ppTask/MAC-ISR context (lead 2) — whether the control-register
  write-lock is keyed to execution context vs a pure MAC/PM state precondition.
- Find what OPENS the launch-latch write window: diff the WDEV MMIO the blob touches between the
  submit (ppTxPkt/pp_post) and the arm, looking for a "tx request/ready" strobe (candidate regions
  0x600a4c00 datapath control, 0x600a4308) written on the ppProcessTxQ path but not on ours.
- Try issuing the enable write from MAC-ISR/ppTask context (e.g. from a pp_post handler) to test the
  "must originate in scheduler context" half of the hypothesis vs a pure MAC-state precondition.
- The only mechanism proven to latch bits 30/31 remains the blob's full submit->schedule->arm chain;
  a clean-room TX therefore needs to reproduce whatever hardware "tx-requested" state that chain sets
  before the enable write, not just the enable write itself.

## Session 9e: pure-Rust reimplementation of the orchestrators (called by faithful_tx)

Replacing the blob ppTxPkt/ppProcessTxQ/lmacTxFrame that faithful_tx calls with Rust `cr_*`
reimplementations, operating on the REAL our_instances state. These are called DIRECTLY by our
driving code (not symbol interposition), so the lmac placement/timing wall does not apply. Leaf
helpers stay blob for now. Radiation held at every step (binned mon0 vs CR-CTRL control).

- cr_ppProcessTxQ (Rust): lmacIsIdle guard (our_instances[ac].state+0x12==0), ppSearchTxframe pop
  (blob leaf), pp_coex_tx_request (blob leaf), lmacTxFrame. Legacy-beacon AMPDU/RTS/fragment branches
  skipped (flags not HE/AMPDU, trc==0). Oracles: block 0x00000000 (active), PLCP0_ENABLE ->
  0xc067a5c8 (latched), completed=true. Radiation: CR-RUST 260 (bins 98/118/44) vs CR-CTRL 400.
- cr_lmacTxFrame (Rust): the ARM on real our_instances[ac] state -- cur_eb=eb, lmacSetTxFrame (blob
  shim -> our Rust hal builds PPDU), hal_random backoff masked by CW (txq+8), hal_mac_tx_config_edca,
  state(+0x12)=ARMED, hal_mac_txq_enable(slot=txq+4). long-frame/FTM/retry branches skipped (beacon).
  Radiation: CR-RUST 241 (bins 102/119/20) vs CR-CTRL 349; oracles unchanged.
- cr_ppTxPkt (Rust): the submit -- ic_interface_enabled guard, ppTxProtoProc, ppProcTxSecFrame,
  rcGetSched (all blob leaves), ppMapTxQueue (blob leaf, runs pm_on_data_tx = the PM-wake), then the
  Rust ENQUEUE onto the real per-AC pending list. KEY FIX: the pending-list base is pTxRx/TxRxCxt
  = *(0x4087ff80), NOT our_instances (0x4087f840) -- verified from the linked ppTxPkt disasm
  (`lui 0x40880; lw -0x80`). Enqueue at TxRxCxt+ac*0x34: *( *(q+0x24) ) = eb; *(q+0x24) = eb+0x30
  (tail append, threaded via eb+0x30). (our_instances holds the EDCA state that cr_lmacTxFrame uses;
  the pending lists are a separate per-AC array in TxRxCxt -- the session-8 map conflated the two.)
  The cat-sanity-check + g_lmac_cnt stats and the kick/pp_post path are omitted (diagnostic/kick==0).
  Radiation: CR-RUST 9 vs CR-CTRL 11 (parity; low absolute counts = a congested channel, 820
  competing beacons, hitting both equally), no panic, oracles unchanged.

All three orchestrators (ppTxPkt, ppProcessTxQ, lmacTxFrame) are now Rust, called directly by
faithful_tx, and CR-RUST radiates. Remaining blob leaf-helpers = the next de-blob frontier:
  esf_buf_alloc, ppTxProtoProc, ppProcTxSecFrame, rcGetSched, ppMapTxQueue (+ its pm_on_data_tx),
  pp_coex_tx_request, ppSearchTxframe/ppGetTxframe (the pending-list pop + queue selection),
  lmacSetTxFrame (PPDU-build orchestration; its hal leaves are already our Rust), hal_random,
  ic_interface_enabled/lmacIsLongFrame, esf_buf_recycle, and the completion/ISR path
  (wDev_ProcessFiq -> lmacProcessTxComplete -> lmacTxDone).

## Session 9f: remaining leaf helpers + completion path in Rust

Continuing the pure-Rust reimplementation; same rules (called directly by our code, real state,
radiation + pool-health verified, committed per step). Added a periodic [CR.H] health line
(arms/latched/completed/allocfail + last PLCP0_ENABLE) as the per-step oracle: latched==arms and
completed==arms means every faithful frame armed (MAC went active, launch latched 0xc067a5c8) and
completed with a healthy pool.

- cr_hal_random (Rust xorshift) + cr_rcGetSched (Rust, trc==0 no-op). Health: arms=64 latched=64
  completed=64 allocfail=0 last_plcp0=0xc067a5c8; radiation CR-RUST 76 vs CR-CTRL 147.

### PART B: completion path in Rust (polling in our loop, pool-healthy)
cr_lmacTxFrame no longer sets our_instances[ac].state(+0x12)=1. With state!=1 the blob MAC ISR
(lmacProcessTxComplete) takes its _L559 branch for our AC -- it clears the completed-state bit and
logs, but does NOT recycle our eb -- so OUR code owns the completion. cr_complete (called from
faithful_tx's loop) is the lmacProcessTxComplete+lmacTxDone essentials in Rust:
  - poll PLCP0_ENABLE[ac] until the arm bits (0xc0000000) clear (the MAC auto-clears them when the
    TX finishes) -- bounded, out-of-band;
  - hal_mac_get_txq_complete(our_instances[ac], ac, res6, aux8) [our Rust] -> status nibble res6[1]>>4
    (0 = success);
  - hal_mac_clr_txq_state(2, ac) [our Rust];
  - esf_buf_recycle(eb) -- the lmacTxDone-equivalent recycle (ppProcTxCallback/rcUpdateTxDone not
    needed for a raw trc==0 no-callback beacon). The esf_buf POOL allocator stays blob (intentional).
CRITICAL: do NOT hal_mac_txq_disable in completion -- the MAC already clears the arm bits, and
forcing a disable desyncs slot 0 (shared with the control beacon) and kills BOTH CR-RUST and CR-CTRL
(observed: CR-RUST 0 / CR-CTRL 5). Without the disable: pool healthy over 96+ frames (arms=latched=
completed, allocfail=0), radiation CR-RUST 111 vs CR-CTRL 194. allocfail=0 with no crash proves
exactly one recycle per frame (ours) -- the blob ISR is not double-recycling.
- cr_ppGetTxframe (Rust pending-list pop): dequeue the head eb from TxRxCxt[ac] (*0x4087ff80 +
  ac*0x34; head +0x20, tail +0x24, next eb+0x30; empty -> tail=&head) with the blob guard
  (+0x29==0 && +0x34==0). Replaces blob ppSearchTxframe in cr_ppProcessTxQ (our single-AC submit
  doesn't need the blob's multi-queue selection/bitmap). DROPPED the blob's lmacAdjustTimestamp()
  call -- it derefs an AP/beacon context that is null in our raw path (Load access fault at
  0x4080519c); our beacon uses timestamp=0 so the fixup is unnecessary. Health: 96/96/96 allocfail=0;
  radiation CR-RUST 4 == CR-CTRL 4 (parity; short mid-flash window + congested channel).
- cr_ppMapTxQueue (Rust AC mapping): for the raw beacon (trc==0) -> txinfo+4=7, AC=iface<<0x14 in
  txinfo+0x10, and KEEP blob pm_on_data_tx (the PM-wake) + ppProcessWaitingQueue (hmac drain). The
  QoS-data/TWT mapping branches (ppSearchTxQueue/pm_on_twt_force_tx) aren't exercised by the beacon.
  Health 96/96/96 allocfail=0; radiation CR-RUST present vs CR-CTRL (both steady under congestion).
- cr_ppTxProtoProc (Rust): reads the on-air FC and sets the protocol flags; for our broadcast mgmt
  beacon only txinfo bit1 (no-ack) is set. Sustained health arms=latched=completed=160, allocfail=0;
  radiation over 120s CR-RUST 9 (bins 3/3/1/2) vs CR-CTRL 19 (7/3/3/6) on a heavily congested channel
  (1279 competing beacons) -- both steady across all bins.

## Session 9f summary: Rust now covers the whole TX pipeline; remaining blob leaves

NOW RUST (called directly by faithful_tx / our loop, real scheduler state, radiation + pool-health
verified, each committed): cr_ppTxPkt, cr_ppProcessTxQ, cr_lmacTxFrame (orchestrators, session 9e);
cr_ppMapTxQueue (AC mapping), cr_ppGetTxframe (pending-list pop), cr_ppTxProtoProc (proto flags),
cr_hal_random, cr_rcGetSched, and cr_complete (the completion path -- poll + hal_mac_get_txq_complete
decode + clr_txq_state + esf_buf_recycle, pool-healthy over 160+ frames). The hal_mac_tx register
layer (set_plcp0/1, config_edca/timeout, set_ppdu, txq_enable, get_txq_complete/state/pmd,
clr_txq_state) was already Rust from the de-blob work.

REMAINING BLOB LEAVES (intentional, with reasons):
- esf_buf_alloc / esf_buf_recycle: the eb POOL allocator is deeply tied to the blob memory pools
  (g_eb_list_desc, the static/dynamic pools, g_wifi_global_lock). Rewriting it is a large side-quest
  with no radiation benefit; kept as the allocator boundary.
- ppProcTxSecFrame: real security-header work (adjusts the MPDU length/seqno by the key/IV size and,
  for encrypted frames, does the crypto). Not a no-op even for our open beacon; kept.
- pm_on_data_tx: the PM-wake that makes the MAC active -- the breakthrough ingredient. Deep PM FSM
  (connection/sleep/coex-slice state); reimplementing risks the MAC-active transition, so kept and
  validated (the MAC still goes active: block 0x600a4ca8 -> 0, launch latches).
- pp_coex_tx_request: coex signaling via the OSI coex callbacks; no effect on our interlock (tested
  earlier) and reimplementing needs the OSI coex layer; kept.
- lmacSetTxFrame: its PPDU programming already routes through our Rust hal (config_timeout/set_ppdu);
  only the TSF-based lifetime calc + TXOP-token sequencing is blob. Kept (low incremental value).
- ppProcessWaitingQueue (hmac drain), lmacAdjustTimestamp (AP-beacon TSF fixup -- DROPPED from our
  pop since it faults on our raw path), ic_interface_enabled / lmacIsLongFrame (trivial guards; ROM
  / uncertain runtime DAT addresses, return the expected constant for our beacon) -- kept as guards.

Completion-ISR note: the blob MAC ISR (wDev_ProcessFiq -> lmacProcessTxComplete) still fires on TX-
done, but because cr_lmacTxFrame leaves our_instances[ac].state!=1 it takes the skip branch for our
AC (clears the completed bit + logs, no recycle). Our cr_complete owns the decode + recycle; allocfail
stays 0 over sustained runs with no crash, proving exactly one recycle per frame (no ISR double-free).
Replacing the shared MAC ISR itself was deemed unnecessary and risky (it also serves RX/beacon/timers).

## Session 9g: env recovery (blob 0.3.0) + more leaves; pm_on_data_tx is the crown-jewel substrate

Environment note: an esp-hal workspace update bumped esp-radio to esp-wifi-sys 0.3.0, whose libpp.a
blob DIFFERS from the reversed one (git checkout 2ea8e3e; hal_mac_tx.o/lmac.o md5 differ). The build
broke (esp_rtos::start API changed to one arg; and duplicate lmac symbols from the unpatched 0.3.0
blob). Fixes (no tracked esp-hal source changed): esp_rtos::start(timg0.timer0); re-ran
tools/patch_libpp.sh (BR=registry 0.3.0) -> 23 hal_mac_tx + 46 lmac cross-boundary symbols renamed
to blob_ (SAME counts as 2ea8e3e -> module structure unchanged), copied the patched libpp.a over the
build's out copy. VALIDATED the 0.3.0 blob is compatible with our reversed offsets: the whole Rust
pipeline still arms (PLCP0_ENABLE latches 0xc067a6f0), completes, pool healthy (arms=latched=
completed, allocfail=0) and radiates -- so all prior offsets/addresses hold on 0.3.0.

New Rust this session:
- pp_coex_tx_request -> Rust no-op. Proven a no-op on our path: skipping it keeps the MAC waking,
  latching (0xc067a6f0) and radiating with a healthy pool (arms=latched=completed=96, allocfail=0).
  Coex is signaling-only when coex is off; nothing on our beacon path depends on it.
- cr_lmacSetTxFrame (Rust): the PPDU-build sequencing -- reduced to the raw-beacon path (no TXOP
  aggregation, trc==0) with a fixed lifetime (the timeout is not the radiate gate, session 7); the
  slot programming goes through our Rust hal (hal_mac_tx_config_timeout + hal_mac_tx_set_ppdu). With
  it, cr_lmacTxFrame is now FULLY Rust. Health arms=latched=completed=96, allocfail=0; radiation
  CR-RUST 11 vs CR-CTRL 21.

CROWN JEWEL -- pm_on_data_tx STAYS BLOB (evidence-backed, as anticipated):
- It is the essential per-frame PM-wake. DECISIVE test: replacing it with a Rust no-op ->
  latched=0/128 (the launch bits NEVER latch), WDEV_PM_TXBLOCK_RETENTION stuck at 0x00ff1000
  (blocked), zero radiation. With it -> block clears to 0, launch latches, radiates. So pm_on_data_tx
  is what transitions the MAC to the active/TX-allowed state.
- What it does (from the decompile) that we could not reproduce: a deep PM finite-state machine --
  pm_check_state (modem sleep/dream state), and on the disconnected/awake path pm_disconnected_wake
  (which calls wifi_rf_phy_enable(0) to re-enable the modem/PHY and pm_set_state), plus coex-slice
  scheduling and active timers, all gated on a dozen PM-state globals (DAT_420824b1/b2/be,
  420825d1, ...). The WDEV_PM_TXBLOCK_RETENTION block-clear is a CONSEQUENCE of this modem-wake, not
  a single reproducible register write (no per-beacon CPU write clears the full 0x00ff1000). This is
  substrate modem power-management, on par with PHY/clock init and the OS adapter that we
  intentionally leave to the blob/esp-radio -- reimplementing it would mean reimplementing the modem
  PM subsystem, with no RF benefit over the blob call. Kept as blob and validated it still wakes the
  MAC on the 0.3.0 blob.

Remaining blob leaves (final): pm_on_data_tx (the PM-wake, above), esf_buf_alloc/recycle (pool
allocator substrate), ppProcTxSecFrame (real security-header length/seqno work), ppProcessWaitingQueue
(hmac drain -- iterates pm_allow_tx/ppMapWaitTxq over interfaces; empty on our path but reads
version-specific static index), and the trivial guards ic_interface_enabled / lmacIsLongFrame (ROM /
uncertain runtime DAT addresses; always the beacon constant on our path). All are substrate/PM/pool
or blob-version-address-dependent; the per-frame TX critical path (submit -> map -> pop -> arm ->
complete) is Rust except for the pm_on_data_tx PM-wake.
