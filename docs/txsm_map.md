# ESP32-C6 pp/lmac TX state machine — reversed map (session 7, 2026-09-11)

Built on 6 prior passes. All conclusions below are also persisted as comments/labels/structs
in the live Ghidra project (see `retools.comment_map()`), this file is the readable digest.

## The loop (states + transitions)

```
esp_wifi_80211_tx / ieee80211_output_do   (public entry, runs on caller/wifi task)
  -> ieee80211_alloc_tx_buf  -> eb (esf_buf) allocated, txinfo(eb+0x34) filled
  -> ieee80211_post_hmac_tx  -> ic_tx_pkt(eb) -> ppTxPkt(eb,1)

ppTxPkt(eb, kick)                                  [SUBMIT]
  - ic_interface_enabled? else discard
  - ppTxProtoProc + ppProcTxSecFrame (crypto/proto)
  - rcGetSched (rate control picks rate for eb+0x2c trc)
  - ppMapTxQueue -> chooses AC, writes txinfo+0x10 bits20-23
  - ENQUEUE eb onto g_ic.txq[ac].pending  (tail = +0x24, threaded via eb+0x30)
  - if kick && lmacIsIdle(ac):  pp_post(ac, 0)     -> wakes ppTask

ppTask  (the pp event loop; blocks on g_ic->queue_recv +0x74)   [SCHEDULER THREAD]
  switch(sig):
    0..4 -> ppProcessTxQ(ac)          <- TX schedule
    0x10 -> ppProcTxDone(1)           <- TX done housekeeping
    0x16 -> lmacProcessTxTimeout
    0x17 -> lmacProcessTxComplete     <- TX complete bottom half
    0x11/0x0d/0x19 -> RX; 0x12 -> AMPDU resort; 8 -> timers

ppProcessTxQ(ac)                                   [SCHEDULE]
  - guard: lmacIsIdle(ac) && !pm/twt/mesh-block
  - ppSearchTxframe -> pop eb from g_ic.txq[ac].pending
  - AMPDU aggregation (ppCalTxAMPDULength / ppAssembleAMPDU / ppHEAMPDU2Normal)
  - -> lmacTxFrame(eb, ac)

lmacTxFrame(eb, ac)                                [ARM]
  - g_ic.txq[ac].cur_eb (+0) = eb
  - discard flag (txinfo bit16)? recycle + pp_post, return
  - long frame -> RTS (txinfo |= 0x100)
  - random EDCA backoff = hal_random() -> txq[ac]+6
  - hal_mac_tx_config_edca -> WDEV_TXQ_CONF1 (AIFSN | CW/backoff | iface)
  - g_ic.txq[ac].state (+0x12) = 1  (ARMED)
  - hal_mac_txq_enable(slot)  -> WDEV_TXQ0_PLCP0_ENABLE |= 0xc0000000 (valid|enable)
      (lmacSetTxFrame runs first inside lmacTxFrame's callee path:
       hal_mac_tx_config_timeout -> CONF1 low12 = lifetime;
       hal_mac_tx_set_ppdu -> mac_tx_set_plcp0/1 + HT/HE-SIG + rate/duration slot regs)

--- hardware transmits (or does not — see handshake) ---

wDev_ProcessFiq  (FIQ dispatcher)                  [IRQ bottom half]
  - hal_mac_interrupt_get_event bitmap:
      0x80  -> lmacPostTxComplete  == pp_post(0x17)   <- TX DONE irq
      0x4000-> RX, 0x100 -> collisions, 0x80000 -> tx timeout, 0x8000 -> beacon filter
  - PWR bitmap: 0xf tbtt, 0xf0 tsf timer, 0x80000 beacon miss

lmacProcessTxComplete  (sig 0x17, ppTask)          [COMPLETE]
  - hal_mac_get_txq_state(2) = completed-queue bitmap
  - per queue: state must be 1 (armed); hal_mac_get_txq_complete -> result regs
  - status nibble = result>>12 & 0xf:
      0 success -> lmacProcessTxSuccess   (state=5)
      1 rts err, 2 cts timeout, 4 tx err, 5 ack timeout -> lmacProcessTx*/retry
  - hal_mac_clr_txq_state

lmacProcessTxSuccess -> lmacEndFrameExchangeSequence -> lmacTxDone      [DONE -> NEXT]
  - ppProcTxCallback + ppEnqueueTxDone (recycle eb to caller)
  - rcUpdateTxDone (rate control feedback)
  - pp_post(0x10); lmacReleaseTxopQueue
  - ppProcessTxQ(ac)  <- launches the NEXT queued frame  ==> loop closes
```

## Data structures (registered as Ghidra types in category /wifi_txsm)

- **g_ic** = `*(void**)0` (Ghidra `iRam00000000`/`pcRam00000000`): the ic/wdev control block.
  First 0x104 bytes = 5 `lmac_txq_c6` blocks (stride 0x34). Higher offsets = op-pointer table
  (see OS surface). Also holds per-iface HMAC/mgmt queues at g_ic+0x378+iface*8.
- **lmac_txq_c6** (per-AC, g_ic+ac*0x34): +0 cur_eb, +0x12 state{0 idle,1 armed,5 success,6 error},
  +0x1d txop_depth, +0x20 pending_head, +0x24 pending_tail, +0x2d..0x31 completion result.
- **eb_c6** (esf_buf): +0x08 dma_desc, +0x1a hmac_waiting, +0x1c flags, +0x2c trc (rate/node),
  +0x30 next-link, +0x34 -> tx_desc_c6.
- **tx_desc_c6** (*(eb+0x34)): +0 flags0 (bit31 HE, 0x400000 AMPDU, 0x100 long, 0x10000 discard,
  0x20000000 FTM), +4 catq (cat/AC), +0xc rate_idx, +0x10 ifaceac (bit19 iface, b20-23 AC),
  +0x18 tsf_submit, +0x28 duration, +0x2f gi_ltf, +0x40/44 lifetime.

## OS-adapter surface the TX path needs (g_ic op pointers)

+0x28 enter_critical, +0x2c exit_critical, +0x30 yield_from_isr, +0x40 task_init,
+0x54/+0x58 mutex lock/unlock, +0x64 queue_send, +0x68 queue_send_from_isr,
+0x74 queue_recv (blocking), +0x78 msgs_waiting, +0xa0 ms->ticks, +0x148 rtc_time, +0x174 malloc.
=> minimum to run pp for TX: one task (ppTask) + a message queue (send/send_isr/recv) +
recursive mutex/critical section + a timer service + malloc + yield_from_isr.

## The pp<->BB handshake candidate (item 4)

Established empirically over 6 passes: the per-frame path writes ONLY MAC slot regs
(0x600a4d6x + 0x600a54xx); there is no per-frame modem/RF write, no one-time ic_enable_tx,
and forcing every BB bit esp-radio asserts during real TX produces no RF (those bits are
EFFECTS of a real transmit, not a settable arm). This session adds:

- `hal_init` (which open-mac DOES call) already runs `mac_txrx_init`, so the one-time MAC
  TX-datapath control writes (0x600a4c00 region, 0x600a4c9c|=3, 0x600a4c1c|=0xc0000000,
  0x600a4308|=2, EDCA/AIFS defaults 0x600a4c20/24) ARE present in open-mac. Not the gap.
- The remaining mechanistic difference is the **live EDCA/contention cycle**: `hal_mac_tx_config_edca`
  programs CONF1 with AIFSN + a *random backoff* and `hal_mac_tx_config_timeout` a lifetime;
  the slot valid|enable bit only ARMS the slot. On the C6 Wi-Fi-6 MAC the PHY key-up appears to be
  triggered by the MAC's internal end-of-backoff "transmit now" edge (a MAC->BB hardware handshake),
  NOT by the enable bit. A one-shot slot poke (open-mac) arms the slot but the MAC completes the
  cycle (tx_success) without the BB engaging — consistent with the observation that the BB TX-active
  status never asserts and no writable register reproduces RF.
- **Cheap untested probe**: the blob has `esp_test_is_disable_edca[ac]` -> `test_disable_edca_tx()`
  (an immediate/no-EDCA send path). Reproducing that path, or ensuring CONF1 carries a valid
  AIFS/CW/lifetime + the EDCA engine is clocked, is the most direct hardware test of this hypothesis.

## Reversing-surface size (for the estimate)

- ppProcessTxQ callees (depth 5): ~72 core pp/lmac/hal functions (100 incl helpers).
- ppTxPkt submit subtree: ~19 core functions.
- completion/retry side (lmacProcessTxSuccess/Error/AckTimeout/ShortRetryFail/
  EndFrameExchangeSequence + rc*): ~another 40-60.
- Total TX-relevant surface ~120-180 distinct functions (names are free — blob keeps symbols —
  the work is semantics + hardware effect), plus the MAC register model and RX/ACK + timers.

---

## Session 7 Phase-1 hardware result: EDCA/CCA ruled out

- Static compare: our esp-wifi-hal C6 path programs EDCA to the SAME registers as the blob.
  PAC layout of tx_slot_config (base 0x600a4d20, stride 0x10): conf0@+0, conf1@+4, config@+8, plcp0@+0xc.
  Our `set_channel_access_parameters` writes `config()` (=0x600a4d68 for slot4=blob q0) with
  TIMEOUT(0-11)/BACKOFF_TIME(12-21)/AIFSN(24-27) — identical fields to blob hal_mac_tx_config_edca
  + hal_mac_tx_config_timeout. Our set_plcp0 clears bit3 of `conf1()` (=0x600a4d64) — matches the
  blob's `&WDEV_TXQ0_CONF1` clear (label confirmed at 0x600a4d64). We DO set a randomized per-frame
  backoff (edca.rs random_backoff_slot_count) for EDCA queues. => no register mismatch.
- CCA control register = 0x600a4c5c (WDEV_MAC_CCA_CTRL), top nibble: phy_enable_cca_new clears
  (CCA on), phy_disable_cca_new sets 0xa0000000 (ignore medium), test_disable_edca_tx sets
  0xc0000000. open-mac never touches it (inherits libphy state).
- EXPERIMENT: added ll.force_disable_cca() (0x600a4c5c|=0xa0000000) before start_tx_queue, built
  beacon CHANNEL=11, flashed C6 (58:e6:c5:17:35:7c), captured AX210 mon0 ch11. Result: 767 frames,
  strong positive control (349 JustANet + others) but ZERO from our SSID "The cake is a lie." /
  our MAC. => forcing CCA off does NOT make the C6 radiate. EDCA/CCA/medium-sensing RULED OUT.
  Experiment reverted (working tree clean); knowledge kept here + memory + Ghidra comments.

## Session 7 Phase-2: full subtree annotated

Every function reachable from esp_wifi_80211_tx / ppProcessTxQ down to the hal_* leaves, plus the
completion/retry cluster and the RX/ACK deferral, is now annotated in Ghidra (147 in scope; 129
persisted analysis comments total). Struct types lmac_txq_c6 / eb_c6 / tx_desc_c6 in /wifi_txsm
refined with the retry-count/CW-index/muedca fields discovered this pass.

## Remaining opacity -> concrete open questions (exit criterion b)

1. The MAC->BB internal transmit trigger. Everything writable is now mapped and none of it (slot
   arm, EDCA, CCA, mac_txrx_init datapath enables) makes the BB engage under direct-slot poke.
   QUESTION: is the "transmit now" edge produced only when the MAC's EDCA slot-time counter is
   actively clocked by a running TSF/timebase? -> test: bring up TSF/TBTT timers before TX on C6
   (the C3 works without this, the C6 may not) and re-check RF. This is the top unresolved lead.
2. RESOLVED: the HE-TB "((slot&3)+0x1802936e)*4" / "0x1802957c*4" accesses are just the
   decompiler showing pre-scaled word pointers; *4 = 0x600a4db8 and 0x600a55f0 respectively, i.e.
   ordinary WDEV MMIO (HE-TB per-queue mplen chain registers). There is NO hidden SRAM window; the
   whole TX register surface is 0x600a4xxx/0x600a5xxx MMIO and is now mapped. This hardens the
   verdict that no writable register is missing.
3. g_ic (=*(void**)0) op-pointer table: offsets +0xc8/+0xd4/+0xf8/+0x104/+0x188/+0x148/+0x1a8/
   +0x19c/+0x394/+0x404 are called but their concrete handlers are set at runtime (not static).
   QUESTION: which come from g_wifi_osi_funcs vs g_ic hardware ops? Resolve by finding the init
   that populates g_ic (the ic-ops registration), not yet located.
4. FUN_ram_420755f0 (bit-scan in lmacProcessTxComplete) and FUN_ram_4207428e (in ppProcTxCallback)
   are unnamed helpers — trivial (find-first-set) but unlabelled.

## Session 7 second hardware experiment: TSF timebase ruled out

Hypothesis: the C6 MAC EDCA/slot sequencer needs a running TSF timebase to advance backoff and fire
the PHY (blob always runs the MAC timer; the open beacon example never enables TSF). Test: added
wifi.set_tsf_time(0,..) + wifi.set_tsf_enabled(0,true) before the beacon loop, rebuilt, flashed,
captured AX210 mon0 ch11 -> 532 frames, strong positive control (272 JustANet), ZERO of ours.
Firmware verified healthy (no panic on serial). => TSF timebase is NOT the blocker either. Reverted.

NET RESULT of session 7 hardware work: three independent writable-state levers tested and falsified
(EDCA-config already matches; CCA-disable; TSF-enable). With the entire TX register surface now
mapped as MMIO and no writable state reproducing RF across 7 passes, the verdict is final: the C6 BB
TX datapath engages ONLY under the running pp/lmac firmware; the open direct-slot approach cannot
drive C6 TX. The register-hunt is exhausted.

## Liveness confirmation (session 7)

To validate the negative results were not a dead/panicked loop, added a serial heartbeat logging each
transmit_oneshot result (ESP_LOG=info, esp-println UART). Observed: "beacon #1..#N tx_result=Ok(())"
incrementing every ~100ms. => the firmware is alive, the TX loop iterates, and the MAC COMPLETES every
frame with Ok(()) (tx_success, no MAC error) -- yet 0 frames reach the air. This is the crux stated
concretely: MAC completes, BB never emits. Confirms the CCA/TSF negatives are real, not artefacts.
