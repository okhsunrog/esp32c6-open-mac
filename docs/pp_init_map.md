# ESP32-C6 pp/lmac INITIALIZATION / bring-up — reversed map (session 8, 2026-09-11)

Companion to `txsm_map.md` (TX loop). Everything here is also persisted in the live Ghidra project
(comments on every function named below, `ROM_*` labels, `menuconfig_*`/state labels, and the
1540 `ROMCALL_<sym>__in_<fn>` renames). Method notes at the end.

## 0. FIRST: what `*(void**)0` / `iRam00000000` really is (corrects txsm_map)

The blob ELF was linked by rust-lld with **269 undefined symbols resolved to address 0**. Ghidra
collapses all of them into `Ram00000000`. They were recovered exactly from the vendor archive
relocations (`relocmap2.py` -> `zsym_map.json`, relaxation-aware) and classified against the IDF
ROM linker scripts (`undef_classes.txt`). Three different things were being called "g_ic":

| what txsm_map called it | real symbol (ROM cell) | points to | role |
|---|---|---|---|
| `g_ic + 0x28/0x2c/0x30/0x40/0x54/0x58/0x64/0x68/0x74/0x78/0xa0/0x148/0x174 ...` | `g_osi_funcs_p` @0x4087ff6c | **APP `g_wifi_osi_funcs`** (wifi_osi_funcs_t, version 8, magic 0xDEADBEAF @+0x1e4) | FreeRTOS/OS glue table |
| `g_ic.txq[ac]` (5 x lmac_txq_c6, stride 0x34) | `our_instances_ptr` = ROM **const** @0x4004ffe0 -> `our_instances` @0x4087f840 (ROM .bss, 260 B) | ROM RAM | per-AC lmac queue array |
| `g_ic + 0x378 + iface*8`, `+0x394/+0x404`, `+0x3fc`, `+0x400`, `+(0xd8+ac)*4` | `pTxRx` @0x4087ff80 -> blob `TxRxCxt` @0x420808cc (0x408 B) | blob .bss | pp context: hmac/wait queues, tx-cb bitmaps+table, rx-pending count, rx_buf_size, BAR ebs |
| first arg of `_mutex_lock/_unlock` | `g_wifi_global_lock` @0x4087ff48 | recursive mutex handle | esf_buf pools, api lock |
| first arg of `_wifi_int_disable/_restore` | `g_intr_lock_mux` @0x4087ff4c | spinlock handle | pp_post / ppTask |
| queue handle in ppTask/pp_post | `s_wifi_queue` @0x4087ff44 | `*(void**)q` = FreeRTOS queue | ppTask mailbox |
| — | `g_ic_ptr` @0x4087ffa0 -> blob `g_ic` @0x420836f0 (0x2b8 B) | net80211 `ieee80211com` | NOT used by the pp TX path |

Other ROM cells written by the blob at init (all in 0x4087f5a8..0x40880000, i.e. **above esp-hal's
RAM end 0x4087e610 — preserved, never touched by our linker**): `pp_task_hdl`, `s_pp_task_create_sem`,
`s_pp_task_del_sem`, `pp_wdev_funcs` (0x408 hook table for ROM-resident pp copies), `net80211_funcs`
(0xec), `lmacConfMib_ptr`, `wDevCtrl_ptr`, `wDevMacSleep_ptr`, `g_lmac_cnt_ptr`, `pp_sig_cnt_ptr`,
`g_wifi_menuconfig_ptr`, `g_eb_list_desc_ptr`, `s_fragment_ptr`, `if_ctrl_ptr`, `ap_no_lr_ptr`,
`rc*SchedTbl_ptr`, `BasicOFDMSched_ptr`, `trc_ctl_ptr`, `g_pm_cfg_ptr`, `g_pm_ptr`,
`g_txop_queue_status_ptr`, `g_pm_cnt_ptr` (all by `wdev_data_init`), `g_scan/g_chm/g_ic_ptr/
g_hmac_cnt_ptr/g_tx_cacheq_ptr/g_mac_sleep_en_ptr/g_mesh_*` (by `net80211_data_ptr_init`),
`g_per_conn_trc[2]` (trc_init), `g_config_func`, `g_timer_func`, `g_net80211_tx_func`,
`g_tx_done_cb_func`, `s_encap_amsdu_func` (registration fns), `our_tx_eb/our_wait_eb` (lmac retry).
ROM .data (`our_controls` 0x4087f6e4 = RC ack/rts/duration tables, the `*_ptr` consts) is
initialised by ROM boot — valid as long as nothing overwrites 0x4087f5a8+.

**The blob executes NO ROM pp/net80211 code.** Undefined *functions* it calls are only ROM libc
(memset/memcpy/memcmp/strlen/strncmp/strnlen/sprintf/puts), libgcc (`__ctzsi2`, `__divsf3`, ...),
`ets_delay_us`, `roundup2`, `esp_*_rom_version_get`, libphy ROM leaves, and app symbols
(`pp_printf`, `net80211_printf`, `phy_printf`, `misc_nvs_init/deinit`, `rtc_clk_xtal_freq_get`,
`g_log_level`, `WIFI_EVENT`...). The ROM's own `ppTask/lmacTxFrame/...` copies are dead code for us.
The two "unnamed helpers" from txsm_map Q4 are ROM `__ctzsi2` call sites (find-first-set), now
`ROMCALL___ctzsi2__in_lmacProcessTxComplete` / `ROMCALL___ctzsi2__in_ppProcTxCallback`.

## 1. Init call tree (exact order)

```
esp_wifi_init(cfg)  ->  esp_wifi_init_internal(cfg)                        [caller task]
  wifi_osi_funcs_register(cfg->osi_funcs)   g_osi_funcs_p = cfg->osi_funcs (checks _version==8, _magic)
  wifi_api_lock()                           s_wifi_api_lock = _recursive_mutex_create() (lazy); _mutex_lock
  pm_funcs_init()                           ptr_beacon_offset_funcs = _calloc_internal(1,0x44); fill
  wdev_funcs_init(caps_lo, caps_hi)         pp_printf("pp rom version"); wdev_data_init()  <- ROM ptr cells
                                            pp_wdev_funcs = _calloc_internal(1,0x408); fill 258 hooks
  net80211_funcs_init()                     net80211_data_ptr_init()  <- ROM ptr cells (once)
                                            net80211_funcs = _calloc_internal(1,0xec); fill; softap funcs
  wifi_init_in_caller_task(cfg)
     g_intr_lock_mux      = _spin_lock_create()          (ROM cell 0x4087ff4c)
     g_wifi_global_lock   = _recursive_mutex_create()    (ROM cell 0x4087ff48)
     mac_list_lock        = _mutex_create()
     wifi_menuconfig_init(cfg)             validate magic 0x1f2f3f4f / wpa_crypto size 0x2c ver 1; copy -> g_wifi_menuconfig
     misc_nvs_init()                       (app symbol)
     ic_create_wifi_task() == pp_create_task()           <- ppTask + queue (section 2)
     ieee80211_ioctl_init()                g_config_func = ieee80211_ioctl_process; g_timer post cb; s_wifi_task_hdl = pp_task_hdl
  msg = _wifi_zalloc(0x18) {type=0x26, fn=wifi_init_process, arg=cfg}; ieee80211_ioctl(msg)
        -> pp_post(6, msg) -> ppTask sig 6 -> ieee80211_ioctl_process -> msg->fn:

  wifi_init_process(msg)                                                    [ppTask]
     memset(g_ic,0,0x2b8); defaults; _read_mac(g_sta_mac_addr,0); _read_mac(g_ap_mac_addr,1)
     wifi_nvs_init()
     wifi_lmac_init(cfg):  ic_set_beacon_int(nvs_bcn_int/100*0x19000)
        ic_init(rx_buf_num, static_tx_num, 1):
           trc_init(1)          g_per_conn_trc[0],[1] = _wifi_zalloc(0x90); rate table ptrs (+0x64..+0x74), +0xc=0x80
           lmacInit()           lmacConfMib defaults; lmacInitAc(2,3,4,10,0) (3,7,4,10,0) (1,2,3,4,0xbc0)
                                (0,2,2,3,0x5e0) (4,1,0,0,0) -> our_instances[]; RC_SetBasicRate(0x15f,1); rcAttach()
           pp_attach(static_tx) pp_sig_cnt[0x24]=0; bars[0..3]=_malloc_internal(0x28); memset(TxRxCxt,0,0x408);
                                TxRxCxt+0x400=0x6a4; ppInitTxq(); TxRxCxt hmac/wait queues +0x374..0x390;
                                esf_buf_setup(static_tx) [eb pools]; TxRxCxt+(0xd8+ac)*4 = ppPrepareBarFrame(ac)
           wDev_Rxbuf_Init(n)   rx dscr ring: _zalloc_internal(n*0xc) + n x _malloc_internal(rx_buf_size+4); wDevCtrl
           pm_attach()          memset(g_pm,0,0x470)+defaults; ppRegisterTxCallback(pm_send_sleep_null_cb,5)/(wake,6);
                                11 x _timer_setfn; _wifi_clock_enable; pm_mac_try_enable_modem_state;
                                hal_set_sta_tsf_wakeup(0); pm_clear_wakeup_signal; _wifi_clock_disable; g_pm=1
           _coex_register_start_cb(esp_wifi_internal_on_coex_start); _coex_schm_register_cb(0, ..._on_coex_schm_phase)
     wifi_hmac_init():   ieee80211_ifattach() [crypto/proto/ht/he/scan/ftm attach on g_ic];
                         ieee80211_output_init() [g_net80211_tx_func = ieee80211_output_process; s_encap_amsdu_func];
                         ic_register_pm_tx_null_cb; ic_register_timer_cb(ieee80211_timer_do_process); ieee80211_cnx_attach
     wifi_crypto_init(cfg)
     g_wifi_state = 1 (inited)

esp_wifi_start()  -> msg{type 1, fn wifi_start_process} -> ieee80211_ioctl -> ppTask:
  wifi_start_process()                                                      [ppTask]
     adc2_wifi_acquire(); ieee80211_set_hmac_stop(); mode = g_wifi_nvs[0]
     wifi_hw_start(idx)      *** THE ONE-TIME HW BRING-UP (first mode only) ***
        ieee80211_set_hmac_stop(0)
        _wifi_apb80m_request()                       OSI +0xc8
        _wifi_clock_enable()                         OSI +0xf8   (MODEM_SYSCON wifi clocks)
        if g_mac_sleep_en: _wifi_rtc_disable_iso()   OSI +0x104
        _phy_enable()                                OSI +0xd4   (libphy: register_chipv7_phy, cal, RF on)
        _coex_enable()                               OSI +0x188
        wifi_reset_mac():  _wifi_reset_mac() (+0xf4, MODEM_SYSCON MAC reset pulse); g_wdev_last_desc_reset=1; hal_mac_rx_disable()
        ic_mac_init() -> hal_mac_init():  WDEV 0x600a4ca8 &= ~((pm_txblock_mask<<16)|0x1000)
        chm_init(&g_ic)
        ic_set_interrupt_handler():
             hal_init()                              <- mac_txrx_init + all one-time MAC regs (txsm_map)
                                                        + _slowclk_cal_get(+0x148) -> hal_timer_update_by_rtc
                                                        + _coex_pti_get(+0x1a8) x14 -> PTI regs
             _set_intr(core, 2 /*WIFI_PWR*/, 1, 1);  _set_intr(core, 0 /*WIFI_MAC*/, 1, 1)
             _set_isr(1, wDev_ProcessFiq, NULL);      _ints_on(1<<1)
        chip_enable():  wifi_set_rx_policy(0) [ic_set_mac(0/1)=0x600a405c.., rx policy regs]; ic_enable_rx() [0x600a4080 |= 1<<31]
        pm_noise_check_enable()
     wifi_mode_set(mode)     STA: wifi_create_sta(): vap=_wifi_zalloc(0x244), node=_wifi_zalloc(0x4f8), ieee80211_phy_init,
                             ht/he attach, ic_register_rx_cb(sta_rx_cb), ic_set_interface(0,vap).  AP: net80211_funcs[+0xd0]
     _do_wifi_start(mode)    STA: wifi_station_start(): hal_enable_sta_tsf() (0x600ad050|=0x88080000);
                             ic_set_vif(0, 0, g_sta_mac_addr, 0, pmf, cfg)  <- ENABLES IFACE 0 (section 3)
                             wifi_event_post(2)
     g_wifi_state = 2 (started); ieee80211_update_phy_country()
```

## 2. ppTask + its queue (facts)

- Created by `pp_create_task` (`ic_create_wifi_task` @0x4200358c is a byte-identical copy):
  - `s_wifi_queue = _wifi_create_queue(200, 8)`; the blob dereferences `*(void**)ret` as the
    FreeRTOS handle (esp-radio's `wifi_static_queue_t { handle; storage }`). Item = `{u32 sig; u32 par}`.
  - `s_pp_task_create_sem = _semphr_create(1, 0)`.
  - `_task_create_pinned_to_core(ppTask, "wifi", stack, NULL, _task_get_max_priority()-2, &pp_task_hdl, core_id)`
    stack = 0xc00 (0x1800 if feature_caps bit0) + 0x200 if `!nano_enable`; core = `menuconfig.wifi_task_core_id`.
  - creator blocks on `_semphr_take(create_sem, -1)`, then `_task_delay(1)`, `_semphr_delete`.
- ppTask body: `_semphr_give(s_pp_task_create_sem)`; loop `_queue_recv(handle, &msg, 0xffffffff)`;
  for `sig < 0x24 && sig != 0xd`: `_wifi_int_disable(g_intr_lock_mux)` / `pp_sig_cnt[sig]--` /
  `_wifi_int_restore`. Dispatch:

| sig | handler | notes |
|---|---|---|
| 0..4 | `ppProcessTxQ(ac)` | TX schedule per AC (4 = beacon/mgmt q) |
| 5 | `g_net80211_tx_func` = `ieee80211_output_process` | drains hmac cache queue -> ppTxPkt |
| 6, 7 | `g_config_func` = `ieee80211_ioctl_process(msg)` | init/start/ioctl run here |
| 8 | `pp_timer_do_process(par)` | ets timers |
| 0xd | `ppProcessRxPktHdr` | RX hdr (throttled) |
| 0xe | assert -> hang | |
| 0xf | task delete path | drains queue, gives `s_pp_task_del_sem`, `_wifi_delete_queue`, `_task_delete(NULL)` |
| 0x10 | `ppProcTxDone(1)` | TX done housekeeping / eb recycle |
| 0x11 | `ppRxPkt` | |
| 0x12 | `ppResortTxAMPDU` | |
| 0x16 | `lmacProcessTxTimeout` | |
| 0x17 | `lmacProcessTxComplete` | TX complete bottom half |
| 0x18 | `lmacProcessCollisions_task` | |
| 0x19 | `wdevProcessRxSucDataAll` | |
| 0x1a/0x1b/0x1d/0x1f/0x20 | pm_on_tbtt / pm_on_tsf_timer / pm_on_beacon_rx / beacon miss / modem-state beacon | |
| 0x1e | bss color collision | |

- `pp_post(sig, par)`: `sig < 0x12` (task ctx) -> spinlock, `pp_sig_cnt` de-dup (sigs 6..8 may
  stack; others coalesce to one pending), `_queue_send(handle, &msg, _task_ms_to_tick(10))`;
  `sig >= 0x12` (ISR ctx: 0x16/0x17/0x18/0x19/0x1a..) -> `_queue_send_from_isr(handle, &msg, &hptw)`
  then `_task_yield_from_isr()` if hptw. sig 0xd is additionally throttled by
  `_queue_msg_waiting > 0x95` or `TxRxCxt+0x3fc > 0x30`.
- ISR: `wDev_ProcessFiq` on CPU interrupt 1 fed by sources 0 (WIFI_MAC) and 2 (WIFI_PWR),
  priority 1. It only posts to ppTask (0x17 on MAC event bit 0x80 = TX complete).

## 3. What the TX submit path needs from init (the dependency list)

`ppTxPkt(eb, 1)` (and everything below it, see txsm_map) reads:
- `g_if_enabled_mask` (blob 0x420812bd, via `ic_interface_enabled`) -> set by **`ic_set_vif(iface, 0, mac, ..)`**
  (also programs MAC-addr regs 0x600a405c/60, rx policy, `if_ctrl[iface]`, RX enable bit31).
  Without it: "lmac if%d stop, discard packet".
- `our_instances[ac]` (ROM RAM) initialised by **`lmacInit`/`lmacInitAc`** (AIFS/CW/TXOP per AC,
  state=0, pending list empty).
- `TxRxCxt` (via pTxRx) initialised by **`wdev_data_init` + `pp_attach`** (ppInitTxq, hmac queues, tx-cb table).
- eb pools `g_eb_list_desc` from **`esf_buf_setup`** (inside pp_attach); `esf_buf_alloc/recycle` lock
  `g_wifi_global_lock` -> needs **`g_wifi_global_lock = _recursive_mutex_create()`**.
- `pp_post` needs **`g_intr_lock_mux = _spin_lock_create()`**, `s_wifi_queue`, `pp_sig_cnt` (zeroed in pp_attach).
- Rate control: `rcGetSched(eb+0x2c trc, txdesc)` **returns immediately if trc == NULL**. Raw
  (`esp_wifi_80211_tx`) frames have trc = NULL and carry the rate in the descriptor:
  `txdesc[7] = ic_get_default_sched()` (= 0x42080f34 = `rc11BSchedTbl+0x24`),
  `txdesc+0xc = ic_get_80211_tx_rate(iface)` (table `_LANCHOR50`, set by `esp_wifi_config_80211_tx_rate`).
  BAR frames instead use `rc_get_trc_by_index(0,0)` -> `g_per_conn_trc[0]` from **`trc_init`**.
- `pp_coex_tx_request` (before lmacTxFrame) calls `_coex_event_duration_get`/`_coex_wifi_request`/
  `_coex_pti_get`; `pm_coex_reconnect_policy`/`chm_get_current_band` read `g_pm`/`gChmCxt`.
  `ppProcessTxQ` guards read `g_pm` (`pm_is_twt_start`, `pm_twt_disallow_tx`, `pm_is_waked`) and
  `g_mesh_is_started` -> **`pm_attach`** (or a zeroed g_pm) keeps them permissive.
- `lmacTxFrame` -> `hal_random` = `_rand()`; `hal_mac_tx_config_edca`... (pure MAC regs);
  `mac_tx_set_pti` -> `_coex_pti_get`.
- Completion: `wDev_ProcessFiq` (needs the ISR installed) -> `pp_post(0x17)` -> `lmacProcessTxComplete`
  -> `lmacTxDone` -> `ppProcTxCallback` (TxRxCxt cb table) / `ppEnqueueTxDone` -> `pp_post(0x10)` ->
  `ppProcTxDone` -> `esf_buf_recycle` (mutex) ; `rcUpdateTxDone` tolerates trc NULL? (see Q2).

## 4. g_wifi_osi_funcs fields the pp path calls (offset -> field -> signature -> who)

Derived by `osi_usage.py` + `callgraph.osi_union` over the reachable set; signatures from
esp-wifi-sys `wifi_osi_funcs_t` (all fields are 4-byte fn pointers; `_version` at 0, `_magic` at 0x1e4).
Everything here is FreeRTOS/OS glue (**no hardware ops live in this table except the
modem-clock/phy/coex shims, which esp-radio already implements**).

TX runtime (ppTask + pp_post + ISR + submit + completion):

| off | field | signature | called from |
|---|---|---|---|
| 0x28 | `_wifi_int_disable` | `fn(mux:*mut void)->u32` | ppTask, pp_post, wDev_AppendRxBlocks |
| 0x2c | `_wifi_int_restore` | `fn(mux:*mut void, tmp:u32)` | ppTask, pp_post |
| 0x30 | `_task_yield_from_isr` | `fn()` | pp_post (ISR sigs) |
| 0x40 | `_semphr_give` | `fn(sem)->i32` | ppTask (create handshake) |
| 0x54/0x58 | `_mutex_lock/_mutex_unlock` | `fn(mutex)->i32` | esf_buf_alloc/recycle, wifi_api_lock, esp_wifi_80211_tx, ppCalTxHEAMPDULength |
| 0x64 | `_queue_send` | `fn(queue, item:*mut void, block_ticks:u32)->i32` | pp_post |
| 0x68 | `_queue_send_from_isr` | `fn(queue, item, hptw:*mut void)->i32` | pp_post |
| 0x74 | `_queue_recv` | `fn(queue, item, block_ticks:u32)->i32` (returns 1 on success) | ppTask |
| 0x78 | `_queue_msg_waiting` | `fn(queue)->u32` | pp_post (sig 0xd), ppTask exit |
| 0xa0 | `_task_ms_to_tick` | `fn(ms:u32)->i32` | pp_post |
| 0xa4 | `_task_get_current_task` | `fn()->*mut void` | current_task_is_wifi_task |
| 0xb0 | `_free` | `fn(p)` | esf_buf_recycle (dynamic ebs) |
| 0xbc | `_rand` | `fn()->u32` | hal_random (EDCA backoff) |
| 0x108 | `_esp_timer_get_time` | `fn()->i64` | pm_* (only if pm paths run) |
| 0x198/0x19c | `_coex_wifi_request/_release` | `fn(event,latency,duration)->i32` / `fn(event)->i32` | pp_coex_tx_request/_release |
| 0x1a4 | `_coex_event_duration_get` | `fn(event, dur:*mut u32)->i32` | pp_coex_tx_request |
| 0x1a8 | `_coex_pti_get` | `fn(event, pti:*mut u8)->i32` | mac_tx_set_pti, pp_coex_tx_request, hal_init |
| 0xe0/0xe4/0xf0 | `_timer_arm/_timer_disarm/_timer_arm_us` | ets_timer API | pm_* timers (only via pm hooks) |
| 0x98/0x17c | `_task_delete/_wifi_delete_queue` | | ppTask exit path only |

Init chain (esp_wifi_init + esp_wifi_start, minus net80211/nvs/crypto):

| off | field | signature | called from |
|---|---|---|---|
| 0x004 | `_env_is_chip` | `fn()->bool` | hal_he_set_mac_delay, hal_init_bf, wifi_hw_start |
| 0x008 | `_set_intr` | `fn(cpu:i32, src:u32, intr_num:u32, prio:i32)` | ic_set_interrupt_handler |
| 0x010 | `_set_isr` | `fn(n:i32, f:*mut void, arg:*mut void)` | ic_set_interrupt_handler |
| 0x014 | `_ints_on` | `fn(mask:u32)` | ic_set_interrupt_handler |
| 0x020 | `_spin_lock_create` | `fn()->*mut void` | wifi_init_in_caller_task (g_intr_lock_mux) |
| 0x034/0x038/0x03c | `_semphr_create(max,init)/_delete/_take(sem,ticks)` | | pp_create_task |
| 0x048/0x04c | `_mutex_create/_recursive_mutex_create` | `fn()->*mut void` | wifi_init_in_caller_task, wifi_api_lock |
| 0x090 | `_task_create_pinned_to_core` | `fn(func, name:*const c_char, stack:u32, param, prio:u32, handle_out:*mut void, core:u32)->i32` (returns 1) | pp_create_task |
| 0x09c/0x0a8 | `_task_delay(ticks)/_task_get_max_priority()->i32` | | pp_create_task |
| 0x0b8 | `_get_free_heap_size` | `fn()->u32` | pp_attach (log only) |
| 0x0c8 | `_wifi_apb80m_request` | `fn()` | wifi_hw_start |
| 0x0d4 | `_phy_enable` | `fn()` | wifi_hw_start |
| 0x0dc | `_read_mac` | `fn(mac:*mut u8, type:u32)->i32` | wifi_init_process |
| 0x0ec | `_timer_setfn` | `fn(timer, fn, arg)` | pm_attach (11x) |
| 0x0f4 | `_wifi_reset_mac` | `fn()` | wifi_reset_mac |
| 0x0f8/0x0fc | `_wifi_clock_enable/_disable` | `fn()` | wifi_hw_start, pm_attach |
| 0x104 | `_wifi_rtc_disable_iso` | `fn()` | wifi_hw_start (only if mac sleep) |
| 0x148 | `_slowclk_cal_get` | `fn()->u32` | hal_init |
| 0x158/0x160/0x164 | `_malloc_internal(sz)/_calloc_internal(n,sz)/_zalloc_internal(sz)` | | pp_attach, wDev_Rxbuf_Init, *_funcs_init |
| 0x168/0x174 | `_wifi_malloc/_wifi_zalloc` | `fn(sz)->*mut void` | esf_buf_alloc_dynamic, trc_init, ioctl msgs |
| 0x178/0x17c | `_wifi_create_queue(len,item)/_wifi_delete_queue` | | pp_create_task |
| 0x188 | `_coex_enable` | `fn()->i32` | wifi_hw_start |
| 0x1c8/0x1cc | `_coex_schm_register_cb/_coex_register_start_cb` | | ic_init |
| 0x14c/0x150 | `_log_write/_log_writev` | | wifi_log (everything) |

Not-OSI "op pointers" from txsm_map's unresolved list: `+0x394/+0x404` = TxRxCxt tx-callback
bitmaps (`ppRegisterTxCallback`), `+0x3fc` TxRxCxt rx-pending count, `+0x400` TxRxCxt rx_buf_size,
`+0xc8/+0xd4/+0xf8/+0x104/+0x148/+0x188/+0x19c/+0x1a8` are the OSI fields listed above.

## 5. BRING-UP RECIPE (pp/lmac on top of our Rust hal_init) — ordered

Two levels. **A** is the positive control that we know radiates (it is exactly what esp-radio does);
**B** is the surgical pp-only subset that this pass established is sufficient to arm one frame.

### A. Full-blob positive control (lowest risk)
Link the vendor archives as esp-radio does, provide `g_wifi_osi_funcs` (esp-radio's `os_adapter`
already implements every field above, version 8 + magic), call `esp_wifi_init_internal(&cfg)`,
`esp_wifi_start()`, then `esp_wifi_80211_tx(0, buf, len, false)`. Confirm RF on the AX210. This
proves the toolchain/link/OSI layer before any surgery.

### B. pp-only bring-up (what our Rust must do, in this order)
0. Memory: keep 0x4087f5a8..0x40880000 untouched (esp-hal already stops at 0x4087e610). Provide
   the 33 app symbols the blob imports (`pp_printf`, `net80211_printf`, `phy_printf`, `misc_nvs_init`
   -> return 0, `misc_nvs_deinit`, `misc_nvs_restore`, `g_log_level/g_log_mod`, `WIFI_EVENT`,
   `rtc_clk_xtal_freq_get`, `puts/putchar/free/floor`, `hexstr2bin`, `regdomain_table/regulatory_data`,
   mesh_* stubs) — esp-radio already has all of them.
1. `wifi_osi_funcs_register(&g_wifi_osi_funcs)` — sets `g_osi_funcs_p`.
   Minimal fields for B: 0x20, 0x28, 0x2c, 0x30, 0x34, 0x38, 0x3c, 0x40, 0x48, 0x4c, 0x54, 0x58,
   0x64, 0x68, 0x74, 0x78, 0x90, 0x9c, 0xa0, 0xa4, 0xa8, 0xb0, 0xb8, 0xbc, 0xdc, 0xec, 0x14c/0x150,
   0x158, 0x160, 0x164, 0x168, 0x174, 0x178, 0x17c, 0x198, 0x19c, 0x1a4, 0x1a8 (coex_* may return 0
   / pti 0), plus the HW shims if we let the blob do HW start (0xc8, 0xd4, 0xf4, 0xf8, 0xfc, 0x104,
   0x148, 0x188, 0x8, 0x10, 0x14, 0x4).
2. `wdev_data_init()` and `net80211_data_ptr_init()` — ROM pointer cells (no alloc). (Optionally the
   full `wdev_funcs_init(caps_lo,caps_hi)` / `net80211_funcs_init()`; the blob TX path never reads
   those tables.)
3. `g_intr_lock_mux = _spin_lock_create()`; `g_wifi_global_lock = _recursive_mutex_create()`
   — write the ROM cells 0x4087ff4c / 0x4087ff48 (extern statics) or just call
   `wifi_init_in_caller_task(&cfg)` which does 3+4+5+6 (but also `misc_nvs_init` + `ieee80211_ioctl_init`).
4. `wifi_menuconfig_init(&cfg)` — needed for tx_buf_type/static_tx_buf_num/mgmt_sbuf_num/
   feature_caps/wifi_task_core_id (esf_buf_setup + task stack/core).
5. `pp_create_task()` — creates `s_wifi_queue` (200 x 8 B) + ppTask; returns after ppTask's first
   `_semphr_give`. ppTask is now parked in `_queue_recv`.
6. `trc_init(1)`; `lmacInit()`; `pp_attach(static_tx_num)`; `pm_attach()` (== the TX-relevant part of
   `ic_init`; skip `wDev_Rxbuf_Init` unless you want blob RX, and skip the coex register cbs).
   All of this is pure SRAM state — it can run before or after HW init (in the blob it runs before).
7. HW bring-up = the body of `wifi_hw_start` with `hal_init` replaced by ours:
   `_wifi_apb80m_request`; `_wifi_clock_enable`; `_phy_enable` (**libphy phy init — the blob never
   touches RF itself; our open-mac path already runs this**); `_coex_enable` (may be a no-op);
   `wifi_reset_mac()`; `hal_mac_init()` (or `ic_mac_init`); **our Rust `hal_init`** (must contain
   the `mac_txrx_init` writes + `hal_timer_update_by_rtc(1, slowclk_cal)` + PTI defaults — compare
   register-for-register with blob `hal_init`, section 3 of txsm_map); then install
   `wDev_ProcessFiq` on the WIFI_MAC(0)+WIFI_PWR(2) interrupt sources (what `ic_set_interrupt_handler`
   does after hal_init); then `chip_enable()` (`wifi_set_rx_policy(0)` + `ic_enable_rx`).
8. Interface: `ic_set_vif(0, 0, sta_mac, 0, 0, 0)` (+ `hal_enable_sta_tsf()` as `wifi_station_start`
   does) — sets `g_if_enabled_mask` bit0 so `ppTxPkt` accepts the frame.
9. Submit one frame (BAR-style, no net80211 needed):
   `eb = esf_buf_alloc(payload_ptr, 1 /*static tx pool*/, len)` (memcpy's payload, sets eb+0x16 len);
   fill `tx_desc = *(eb+0x34)` like `esp_wifi_80211_tx` + `ieee80211_output_raw_process` do:
   `eb+0x14 = 0x18` (hdr len), `eb+0x16 = len-0x18`; `desc[0] |= 0x4000` (raw); `desc[7] =
   ic_get_default_sched()`; `desc+0xc = rate idx` (use `ic_get_80211_tx_rate(0)` table or a fixed
   11b/g index); `desc[4] = (iface&1)<<19 | 0`; `desc[5] = 0x100`; `desc+4 (byte) = 7` (cat/AC ->
   ppMapTxQueue), `desc+0x18 = hal_now()`; DMA desc (`*(eb+4)`): owner|eof bits and length as in
   `ieee80211_output_raw_process`; set sequence number; `eb+0x2c = 0` (trc NULL -> rcGetSched no-op)
   or `rc_get_trc_by_index(0,0)`; then **`ppTxPkt(eb, 1)`** -> `pp_post(ac)` -> ppTask ->
   `ppProcessTxQ(ac)` -> `lmacTxFrame` -> MAC. (Or use `ieee80211_output_raw_process(0, eb)`
   directly: it does the DMA/seq fill and calls `ppTxPkt` — it only needs `g_ic_vap_sta_ptr`
   non-NULL for the node lookup, i.e. a `wifi_create_sta()` first.)
10. Completion: IRQ -> `pp_post(0x17)` -> `lmacProcessTxComplete` -> `lmacTxDone` -> `pp_post(0x10)`
    -> `ppProcTxDone` -> `esf_buf_recycle(eb)`. Watch the serial log for "pp task q full"/"lmac if0
    stop" (both indicate a missing step above).

## 6. Still open (concrete questions)

Q1. RESOLVED: `ic_get_80211_tx_rate` = `trc_get_80211_tx_rate` reads `g_80211_tx_rate_per_iface[iface]`
    (blob .bss 0x42082cc4, labelled) — zero at boot = wifi_phy_rate_t 0 = WIFI_PHY_RATE_1M_L, a valid
    index for hal_mac_tx_set_ppdu; `esp_wifi_config_80211_tx_rate` overrides it.
Q2. Does `rcUpdateTxDone(trc=NULL,...)` on completion tolerate the NULL trc of raw frames on the
    ISR-path (`lmacTxDone` -> `rcUpdateTxDone`)? The raw path in the blob works with NULL trc, so it
    must, but the guard was not decompiled this pass.
Q3. RESOLVED: task name at 0x4205e0c4 is "wifi" (pp_create_task -> _task_create_pinned_to_core(ppTask, "wifi", ...)).
Q4. The remaining 25 functions `relocmap2` could not align (`zsym_unaligned.txt`, mostly libphy/coex
    such as `rc_cal_new`) may have a few un-annotated `Ram00000000` sites — none are on the pp init
    or TX path.

## 7. Method notes / files (for the next session)
- `relocmap2.py` -> `zsym_map.json` (5010 sites), `undef_classes.txt`, `osi_offsets.txt`,
  `osi_usage.py` -> `osi_usage.{txt,json}`, `callgraph.py` -> `callgraph.json`.
- `retools.dcz / disz / zsyms_in / dumpz` — always read this blob with `dcz`.
- Raw dumps used this pass: `init_dumpz1.txt`, `init_dumpz2.txt`, `init_dumpz3.txt`, `tx_dumpz.txt`.
- Ghidra: comments on all functions in sections 1-3; labels `ROM_*` (ROM cells), `g_wifi_state_*`,
  `g_sta_mac_addr/g_ap_mac_addr`, `g_if_enabled_mask`, `g_ic_vap_sta_ptr/g_ic_vap_ap_ptr`,
  `menuconfig_*`; 1540 `ROMCALL_*` renames; the two txsm Q4 helpers resolved.
