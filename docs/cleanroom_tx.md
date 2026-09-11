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
