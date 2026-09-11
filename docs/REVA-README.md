# ReVa (Ghidra) via Bash — cheatsheet for the C6 pp reverse

Ghidra GUI is running with the C6 blob open. ReVa serves MCP on localhost:8080.
Drive it from Bash with the client at scratchpad/reva.py (no MCP setup needed):

  python3 scratchpad/reva.py list-tools
  python3 scratchpad/reva.py call <tool> '<json-args>'

Program path in the project: "/esp32c6-wifi.elf" (4655 funcs, RISCV RV32IMC).
Renames/comments PERSIST in the Ghidra project (assistant mode) — they accumulate
and the user can open the GUI to inspect. Use meaningful names + comments as you go.

Key tools (exact arg names):
- Decompile: get-decompilation {"programPath":"/esp32c6-wifi.elf","functionNameOrAddress":"ppTxPkt","includeCallees":true,"includeCallers":true,"limit":120}
- Rename a FUNCTION: set-function-prototype {"programPath":"/esp32c6-wifi.elf","location":"0x420779e4","signature":"int pp_tx_pkt(int eb, int a2)","createIfNotExists":false}
- Rename a data global / label (e.g. DAT_ram_600axxxx): create-label {"programPath":"/esp32c6-wifi.elf","addressOrSymbol":"0x600a4d6c","labelName":"WDEV_TXQ_CONF2","setAsPrimary":true}
- Rename local vars: rename-variables {"programPath":"/esp32c6-wifi.elf","functionNameOrAddress":"pp_tx_pkt","variableMappings":{"iVar2":"iface_enabled"}}
- Annotate reasoning: set-decompilation-comment {"programPath":"/esp32c6-wifi.elf","functionNameOrAddress":"pp_tx_pkt","lineNumber":15,"commentType":"pre","comment":"..."}
- Navigate: get-callers-decompiled, get-referencers-decompiled, find-cross-references,
  get-call-tree, trace-data-flow-backward/forward, read-memory, find-constant-uses.

The client is non-blocking. If a call hangs >90s, the server may be mid-analysis;
retry. Ghidra GUI + the ReVa server must stay up (do not kill them).

---

# retools.py — reusable RE toolkit (added session 7, 2026-09-11)

`scratchpad/retools.py` is a growing, importable toolkit over this same ReVa server.
Prefer it over ad-hoc reva.py calls; extend it instead of writing one-off scripts.

Import:  `import retools as rt`   (PROG defaults to "/esp32c6-wifi.elf")
CLI:     `python3 retools.py help`  then e.g. `python3 retools.py dc ppTxPkt`

High-level helpers (each has a docstring):
- decompile / decompile_many (batch) — get C, includes our persisted comments
- callers / referencers / xref / call_tree / trace_back / trace_fwd — navigation
- read_mem / search / const_uses / funcs(substr) — lookup
- comment / proto / rename_vars / label — WRITE annotations (persist in Ghidra)
- comment_map(start,end) — list all persisted comments in a range (default 0x42000000-0x420b0000);
  NOTE: decompilation comments live at internal instruction addresses, so query by RANGE not by symbol
- struct_layout(fn) — reconstruct field offsets from `base + 0xNN` accesses in a function
- reg_touch(reg) — which functions touch a register/DAT_ram_600aXXXX
- branches(fn) — branches gated on memory/register reads (find HW-status decision points)
- diff(fnA, fnB) — unified diff of two decompilations
- export_map(outfile) — dump the persisted comment map to a file

Gotchas learned:
- get-decompilation arg is `functionNameOrAddress`; comment tool is `set-decompilation-comment`
  with `functionNameOrAddress`+`lineNumber` (pre/post/eol/plate). A comment on a function not yet
  decompiled THIS session may fail with "read the decompilation first" — call rt.decompile(fn) first.
- get-comments needs `addressRange` as an OBJECT {"start":"0x..","end":"0x.."} (not a string).
- parse-c-structure `category` must be a path like "/wifi_txsm".
- Struct types for the TX machine are registered in Ghidra category /wifi_txsm:
  lmac_txq_c6 (per-AC block), eb_c6 (esf_buf), tx_desc_c6 (*(eb+0x34)). Not force-applied.

Reversed TX state machine digest: scratchpad/txsm_map.md.

## retools additions (session 7 Phase 2)
- decompile(fn) now returns CLEAN C by default (parses the JSON) -> struct_layout/branches/diff work
  directly; decompile_raw(fn) for the full JSON.
- func_addr_map(substr) -> {name: address}; callee_names(addr) -> set of names in a call tree.
- comment_map(start,end) -> [(addr,type,text)]; commented_addrs() -> set of int addrs with comments.
- note(fn, text) primes the decompilation then comments (handles the "read decompilation first" case).
- note_many([(fn,text),...]) -> bulk-annotate a whole cluster (the Phase-2 workhorse).
- dump(fns, path, limit) -> decompile many to one file for reading, then note_many to annotate.
- is_annotated(fn) heuristic (address-window; getters/thunks can false-negative).
The full TX subtree from esp_wifi_80211_tx to hal_* leaves is annotated; digest + open questions in
scratchpad/txsm_map.md; structs lmac_txq_c6/eb_c6/tx_desc_c6 in Ghidra category /wifi_txsm.

## Session 8 additions: address-0 symbol resolution (READ THIS FIRST)

The linked blob has 269 UNDEFINED symbols that rust-lld resolved to address 0. Ghidra shows
every one of them as `*Ram00000000` / `FUN_ram_xxxx()` — they are NOT one global. They are the
ESP32-C6 ROM interface cells (g_osi_funcs_p, pTxRx, our_instances_ptr, g_ic_ptr, net80211_funcs,
g_wifi_global_lock, s_wifi_queue, pp_task_hdl, ...) plus ROM libc/libgcc/phy functions
(memset/memcpy/__ctzsi2/...) and a few app symbols (misc_nvs_init, pp_printf, ...).
Classification: scratchpad/undef_classes.txt (ROM addr from IDF esp32c6.rom.*.ld).

Tooling (all in scratchpad):
- relocmap2.py  -> zsym_map.json : ELF addr -> symbol for every such site, rebuilt from the
  vendor archive relocations (esp-wifi-sys 2ea8e3e, fw 4df78f2), relaxation-aware
  (lld turned local auipc+jalr into jal/c.jal, shifting later offsets). relocmap.py = old naive.
- retools: `dcz <fn>` decompile with `// symbol` per line (USE INSTEAD OF dc), `zsyms <fn>`,
  `disz(fn)` llvm-objdump disassembly with symbols (needed where the decompiler
  dead-store-eliminates consecutive stores to address 0, e.g. wdev_data_init).
- osi_usage.py -> osi_usage.txt/json : which wifi_osi_funcs_t field (offset->name from
  osi_offsets.txt, derived from esp-wifi-sys include.rs) each blob function calls.
- callgraph.py : local static call graph; `osi_union(roots, stop)` = OSI fields reachable.
- The 1540 `FUN_ram_xxxx` fragments that were really ROM call sites are now named
  `ROMCALL_<sym>__in_<function>` in Ghidra (they are split artifacts, not functions).
- Labels ROM_* were added at the ROM cell addresses (0x4087fxxx / 0x4004ffxx).

Init/bring-up digest: scratchpad/pp_init_map.md (session 8). TX loop digest: txsm_map.md
(note its "g_ic = *(void**)0 op table" is really g_wifi_osi_funcs / our_instances / TxRxCxt —
see pp_init_map.md section 0).
