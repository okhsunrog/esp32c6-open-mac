#!/usr/bin/env python3
"""retools -- reusable high-level RE toolkit over the ReVa (Ghidra) MCP-over-HTTP server.

Grows over sessions. Import it (`import retools as rt`) or use the CLI
(`python3 retools.py <cmd> ...`). Talks to the ReVa server on localhost:8080
against the C6 Wi-Fi blob by default (PROG = "/esp32c6-wifi.elf").

One HTTP session is created per process and reused across calls (fast batches).
All annotation helpers PERSIST into the live Ghidra project.

CLI quick reference (see `python3 retools.py help`):
  dc  <fn> [--callees] [--callers]     decompile one function (with comments)
  dcm <fn1,fn2,...>                    batch-decompile, one blob per function
  cmt <fn> <line> <text> [--type pre]  set a decompilation comment
  proto <addr> "<signature>"           set a function prototype / rename it
  rv  <fn> iVar2=name,iVar3=name2      rename local variables
  lbl <addr> <name>                    create a primary label (register / DAT / field)
  xref <loc> [--dir to|from|both]      cross references
  callers <fn>                         decompiled callers
  refs <sym>                           referencers of a data symbol / address
  tree <addr> [--depth N] [--dir ...]  call tree
  back <addr> / fwd <addr>             data-flow trace
  mem <addr> [--len N] [--fmt hex]     read memory
  search <regex>                       search decompilation across program
  regtouch <0x600aXXXX>                functions that touch a register/DAT
  layout <fn> [--base VAR]             reconstruct struct field offsets from a fn
  branches <fn>                        branches gated on register/memory reads
  diff <fnA> <fnB>                     unified diff of two decompilations
  exportmap [outfile]                  dump renamed funcs + comments to a file
  funcs [--filter substr]             list functions
  dcz <fn>                             decompile with address-0 symbols resolved (USE THIS)
  zsyms <fn>                           list address-0 symbol refs in a function
"""
import sys, json, re, http.client, difflib

PROG = "/esp32c6-wifi.elf"
HOST, PORT = "localhost", 8080

# ---------------------------------------------------------------- transport
_sid = None
_id = 0

def _rpc(method, params, want=True):
    """Low-level JSON-RPC over the MCP HTTP/SSE endpoint. Reuses a session id."""
    global _id, _sid
    _id += 1; myid = _id
    body = json.dumps({"jsonrpc": "2.0", "id": myid, "method": method, "params": params})
    h = {"Content-Type": "application/json", "Accept": "application/json, text/event-stream"}
    if _sid: h["Mcp-Session-Id"] = _sid
    c = http.client.HTTPConnection(HOST, PORT, timeout=120)
    c.request("POST", "/mcp/message", body, h)
    r = c.getresponse()
    _sid = r.getheader("Mcp-Session-Id") or _sid
    if not want:
        c.close(); return None
    buf = b""; out = None
    while True:
        ch = r.read(1)
        if not ch: break
        buf += ch
        if ch == b"\n":
            line = buf.decode("utf-8", "replace").strip(); buf = b""
            if line.startswith("data:"): line = line[5:].strip()
            if line.startswith("{"):
                try:
                    o = json.loads(line)
                    if o.get("id") == myid: out = o; break
                except Exception: pass
    c.close(); return out

def _ensure_session():
    global _sid
    if _sid: return
    o = _rpc("initialize", {"protocolVersion": "2024-11-05", "capabilities": {},
                            "clientInfo": {"name": "retools", "version": "2"}})
    _rpc("notifications/initialized", {}, want=False)

def call(tool, args=None, raw=False):
    """Call any ReVa tool by name with a dict of args. Returns joined text (or raw dict)."""
    _ensure_session()
    o = _rpc("tools/call", {"name": tool, "arguments": args or {}})
    if raw: return o
    if isinstance(o, dict) and "result" in o:
        return "\n".join(it.get("text", json.dumps(it)) for it in o["result"].get("content", []))
    return json.dumps(o)

# ---------------------------------------------------------------- read helpers
def decompile_raw(fn, callees=False, callers=False, comments=True, disasm=False, limit=200):
    """Raw JSON string from get-decompilation (has metadata, refs, comments, decompilation)."""
    return call("get-decompilation", {"programPath": PROG, "functionNameOrAddress": fn,
        "includeCallees": callees, "includeCallers": callers, "includeComments": comments,
        "includeDisassembly": disasm, "limit": limit})

def decompile(fn, callees=False, callers=False, comments=True, disasm=False, limit=200, clean=True):
    """Decompiled C for a function name/address (includes our persisted comments).
    clean=True (default) returns just the readable C listing; clean=False returns raw JSON."""
    txt = decompile_raw(fn, callees, callers, comments, disasm, limit)
    if not clean:
        return txt
    try:
        o = json.loads(txt)
        d = o.get("decompilation")
        if d:
            return d
    except Exception:
        m = re.search(r'"decompilation":"(.*?)","parameters"', txt, re.S)
        if m:
            return m.group(1).encode().decode("unicode_escape")
    return txt

def decompile_many(fns, **kw):
    """Batch-decompile; returns {fn: text}. Reuses the session so it is cheap."""
    return {fn: decompile(fn, **kw) for fn in fns}

def callers(fn, maxn=30, context=True):
    """Decompiled callers of a function (who calls it, with call-site context)."""
    return call("get-callers-decompiled", {"programPath": PROG, "functionNameOrAddress": fn,
        "maxCallers": maxn, "includeCallContext": context})

def referencers(sym, maxn=40, data=True, context=True):
    """Referencers of a data symbol / address (who reads/writes this global/register)."""
    return call("get-referencers-decompiled", {"programPath": PROG, "addressOrSymbol": sym,
        "maxReferencers": maxn, "includeDataRefs": data, "includeRefContext": context})

def xref(loc, direction="both", context=True, ctxlines=2, limit=60):
    """Cross references to/from a location. direction in {to,from,both}."""
    return call("find-cross-references", {"programPath": PROG, "location": loc,
        "direction": direction, "includeContext": context, "contextLines": ctxlines,
        "includeData": True, "limit": limit})

def call_tree(addr, depth=3, direction="callees"):
    """Call tree from a function address. direction in {callees,callers}."""
    return call("get-call-tree", {"programPath": PROG, "functionAddress": addr,
        "maxDepth": depth, "direction": direction})

def trace_back(addr):
    """Backward data-flow from an instruction address (where did this value come from)."""
    return call("trace-data-flow-backward", {"programPath": PROG, "address": addr})

def trace_fwd(addr):
    """Forward data-flow from an instruction address (where does this value go)."""
    return call("trace-data-flow-forward", {"programPath": PROG, "address": addr})

def read_mem(addr, length=64, fmt="hex"):
    """Read program memory. fmt in {hex,bytes,...}."""
    return call("read-memory", {"programPath": PROG, "addressOrSymbol": addr,
        "length": length, "format": fmt})

def search(pattern, maxn=60, case=False):
    """Regex search across the whole decompiled program."""
    return call("search-decompilation", {"programPath": PROG, "pattern": pattern,
        "maxResults": maxn, "caseSensitive": case})

def const_uses(value, maxn=60):
    """Find uses of a constant value across the program."""
    return call("find-constant-uses", {"programPath": PROG, "value": value, "maxResults": maxn})

def funcs(substr=None, maxn=6000):
    """List function names (optionally filtered by substring)."""
    t = call("get-functions", {"programPath": PROG, "maxCount": maxn})
    if substr:
        return "\n".join(l for l in t.splitlines() if substr.lower() in l.lower())
    return t

def comments(addr_range=None, sym=None):
    """Dump comments. addr_range = (start,end) tuple/list, or 'start-end' string;
    sym = a symbol/address. NOTE decompilation comments live at internal instruction
    addresses, so query by range (not by function symbol) to see them all."""
    a = {"programPath": PROG}
    if addr_range:
        if isinstance(addr_range, str):
            s, e = addr_range.split("-")
        else:
            s, e = addr_range
        a["addressRange"] = {"start": s, "end": e}
    if sym: a["addressOrSymbol"] = sym
    return call("get-comments", a)

def func_addr_map(substr=None):
    """Return {name: address} for all functions (optionally filtered)."""
    import json as _j
    r = call("get-functions", {"programPath": PROG, "maxCount": 6000}, raw=True)
    txt = "".join(it.get("text", "") for it in r["result"]["content"])
    o = _j.loads(txt)
    out = {}
    for f in o.get("functions", []):
        if substr is None or substr.lower() in f["name"].lower():
            out[f["name"]] = f["address"]
    return out

def callee_names(fn_addr, depth=5, direction="callees", exclude_prefixes=("wifi_log",)):
    """Set of unique function names in a call tree from an address (default callees)."""
    t = call_tree(fn_addr, depth=depth, direction=direction)
    names = set(re.findall(r'"name":"([^"]+)"', t))
    return {n for n in names if not any(n.startswith(p) for p in exclude_prefixes)}

def commented_addrs(start="0x42000000", end="0x420b0000"):
    """Set of int addresses that already carry a comment."""
    return {int(a, 16) for a, _, _ in comment_map(start, end) if a}

def note(fn, text, line=3):
    """Prime the decompilation (so line->addr mapping exists) then set a pre-comment.
    Uses the 'nearest addressable line' fallback, so an approximate line is fine."""
    decompile(fn)
    return comment(fn, line, text)

def note_many(pairs, line=3):
    """Annotate many functions at once: pairs = [(fn, text), ...]. Returns count applied.
    The workhorse for systematic call-tree annotation (Phase-2 style)."""
    n = 0
    for fn, text in pairs:
        r = note(fn, text, line)
        n += ("success" in r) or ("nearest" in r)
    return n

def dump(fns, path, limit=40):
    """Decompile a list of functions (clean C) into one text file with headers.
    Read the file, then annotate with note_many -- avoids re-hitting the server per read."""
    with open(path, "w") as f:
        for fn in fns:
            f.write("\n===== %s =====\n" % fn)
            f.write(decompile(fn, limit=limit))
    return f"wrote {path} ({len(fns)} functions)"

def is_annotated(fn, window=0x600):
    """True if a comment exists within [entry, entry+window) of the function.
    Note: getters/thunks can store their comment at a shared address and read False here
    even when the comment shows in the listing; use as a heuristic, not gospel."""
    a = func_addr_map(fn).get(fn)
    if not a:
        return False
    ai = int(a, 16)
    return any(ai <= c < ai + window for c in commented_addrs())

def comment_map(start="0x42000000", end="0x420b0000"):
    """Return [(address, type, text)] of all comments in a code range, sorted by address."""
    import json as _j
    txt = comments(addr_range=(start, end))
    try:
        o = _j.loads(txt)
    except Exception:
        return []
    out = [(c.get("address"), c.get("commentType"), c.get("comment", "")) for c in o.get("comments", [])]
    return sorted(out, key=lambda x: int(x[0], 16) if x[0] else 0)

# ---------------------------------------------------------------- write helpers
def comment(fn, line, text, ctype="pre"):
    """Set a decompilation comment (ctype: pre|post|eol|plate) -- persists in Ghidra."""
    return call("set-decompilation-comment", {"programPath": PROG, "functionNameOrAddress": fn,
        "lineNumber": line, "comment": text, "commentType": ctype})

def proto(location, signature, create=False):
    """Set a function prototype (also renames the function) -- persists in Ghidra."""
    return call("set-function-prototype", {"programPath": PROG, "location": location,
        "signature": signature, "createIfNotExists": create})

def rename_vars(fn, mapping):
    """Rename local variables: mapping = {"iVar2":"queue_idx", ...} -- persists in Ghidra."""
    return call("rename-variables", {"programPath": PROG, "functionNameOrAddress": fn,
        "variableMappings": mapping})

def label(addr, name, primary=True):
    """Create a label at an address (register / DAT / struct field) -- persists in Ghidra."""
    return call("create-label", {"programPath": PROG, "addressOrSymbol": addr,
        "labelName": name, "setAsPrimary": primary})

# ---------------------------------------------------------------- analysis (pure python over decompiled text)
_OFF_RE = re.compile(r"\b(\w+)\s*\+\s*(0x[0-9a-fA-F]+|\d+)\b")

def struct_layout(fn, base=None):
    """Reconstruct struct field offsets from a function's decompilation.

    Scans for `<var> + 0xNN` patterns and groups the offsets per base variable,
    which reconstructs the field layout of pointers threaded through the code.
    If `base` is given, only that variable is reported. Returns a text report.
    """
    txt = decompile(fn)
    groups = {}
    for m in _OFF_RE.finditer(txt):
        var, off = m.group(1), m.group(2)
        val = int(off, 16) if off.startswith("0x") else int(off)
        groups.setdefault(var, set()).add(val)
    lines = [f"# struct-offset map for {fn} (base var -> sorted offsets)"]
    for var in sorted(groups):
        if base and var != base: continue
        offs = sorted(groups[var])
        lines.append(f"{var}: " + ", ".join(hex(o) for o in offs))
    return "\n".join(lines)

def reg_touch(reg):
    """Which functions touch a register / DAT_ram_600aXXXX. Accepts 0x... or a symbol.

    Tries referencers first (data xrefs), then a decompilation text search as a
    fallback for the DAT_ram_<addr> spelling.
    """
    out = [f"== referencers of {reg} =="]
    out.append(referencers(reg))
    if reg.startswith("0x"):
        datname = "DAT_ram_" + reg[2:].lower()
        out.append(f"\n== text search for {datname} ==")
        out.append(search(datname))
    return "\n".join(out)

def branches(fn):
    """Extract branches gated on register/memory reads in a function.

    Heuristic: lines containing an `if`/`while`/`?` whose condition dereferences
    memory (`*(...)`, `DAT_ram_`, `_DAT_ram_`, or `[...]`). Useful to find the
    hardware-status-polled decision points in a state machine.
    """
    txt = decompile(fn)
    hot = []
    for i, ln in enumerate(txt.splitlines(), 1):
        s = ln.strip()
        if re.search(r"\b(if|while)\b|\?", s) and re.search(r"\*\(|DAT_ram_|\]\s*[=!<>&|]|0x600a", s):
            hot.append(f"{i:4}: {s}")
    return "\n".join([f"# memory/register-gated branches in {fn}"] + hot)

def diff(fn_a, fn_b, **kw):
    """Unified diff between two functions' decompilations (structure/behaviour compare)."""
    a = decompile(fn_a, **kw).splitlines()
    b = decompile(fn_b, **kw).splitlines()
    return "\n".join(difflib.unified_diff(a, b, fn_a, fn_b, lineterm=""))

def export_map(outfile=None, name_filter=None):
    """Export accumulated map: non-default function names + all plate/pre comments.

    Writes a readable snapshot of what has been annotated so a future session can
    pick up the map without re-reading the whole project.
    """
    lines = ["# ===== persisted analysis comments (address :: type :: text) ====="]
    for addr, ctype, text_ in comment_map():
        for a in (addr, addr):  # keep address then wrapped text
            pass
        lines.append(f"{addr}  [{ctype}]  {text_}")
    text = "\n".join(lines)
    if outfile:
        with open(outfile, "w") as f: f.write(text)
        return f"wrote {outfile} ({len(text)} bytes)"
    return text

# ---------------------------------------------------------------- CLI
def _main(argv):
    if not argv or argv[0] in ("help", "-h", "--help"):
        print(__doc__); return
    cmd, a = argv[0], argv[1:]
    def opt(name, default=None):
        if name in a:
            i = a.index(name)
            return a[i+1] if i+1 < len(a) else True
        return default
    def flag(name):
        return name in a
    if cmd == "dc":
        print(decompile(a[0], callees=flag("--callees"), callers=flag("--callers")))
    elif cmd == "dcm":
        for fn, txt in decompile_many(a[0].split(",")).items():
            print(f"\n// ===== {fn} =====\n{txt}")
    elif cmd == "cmt":
        print(comment(a[0], int(a[1]), a[2], ctype=opt("--type", "pre")))
    elif cmd == "proto":
        print(proto(a[0], a[1]))
    elif cmd == "rv":
        mp = dict(kv.split("=", 1) for kv in a[1].split(","))
        print(rename_vars(a[0], mp))
    elif cmd == "lbl":
        print(label(a[0], a[1]))
    elif cmd == "xref":
        print(xref(a[0], direction=opt("--dir", "both")))
    elif cmd == "callers":
        print(callers(a[0]))
    elif cmd == "refs":
        print(referencers(a[0]))
    elif cmd == "tree":
        print(call_tree(a[0], depth=int(opt("--depth", 3)), direction=opt("--dir", "callees")))
    elif cmd == "back":
        print(trace_back(a[0]))
    elif cmd == "fwd":
        print(trace_fwd(a[0]))
    elif cmd == "mem":
        print(read_mem(a[0], length=int(opt("--len", 64)), fmt=opt("--fmt", "hex")))
    elif cmd == "search":
        print(search(a[0]))
    elif cmd == "regtouch":
        print(reg_touch(a[0]))
    elif cmd == "layout":
        print(struct_layout(a[0], base=opt("--base")))
    elif cmd == "branches":
        print(branches(a[0]))
    elif cmd == "diff":
        print(diff(a[0], a[1]))
    elif cmd == "exportmap":
        print(export_map(a[0] if a else None))
    elif cmd == "funcs":
        print(funcs(substr=opt("--filter")))
    elif cmd == "dcz":
        print(dcz(a[0]))
    elif cmd == "zsyms":
        for x in zsyms_in(a[0]): print(*x)
    elif cmd == "call":
        print(call(a[0], json.loads(a[1]) if len(a) > 1 else {}))
    else:
        print(f"unknown cmd {cmd}; run `python3 retools.py help`")

if __name__ == "__main__":
    _main(sys.argv[1:])

# ---------------------------------------------------------------- address-0 symbol resolution (session 8)
# The linked blob has ~269 UNDEFINED symbols resolved to 0 (ROM pointer globals such as
# g_osi_funcs_p, pTxRx, our_instances_ptr, g_ic_ptr, net80211_funcs, ...). Ghidra shows all of
# them as `*Ram00000000`. relocmap.py rebuilds addr->symbol from the archive relocations into
# zsym_map.json; these helpers inline that knowledge into decompilations.
import os as _os
_ZSYM = None
def zsym_map():
    """{int addr: (symbol, rtype, fn, off)} from zsym_map.json (run relocmap.py to build)."""
    global _ZSYM
    if _ZSYM is None:
        p = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "zsym_map.json")
        _ZSYM = {int(k, 16): tuple(v) for k, v in json.load(open(p)).items()}
    return _ZSYM

def zsyms_in(fn, window=None):
    """[(addr, symbol)] of address-0 symbol references inside a function (by address range)."""
    o = json.loads(decompile_raw(fn, limit=1))
    s, e = int(o["startAddress"], 16), int(o["endAddress"], 16)
    return [(hex(a), v[0]) for a, v in sorted(zsym_map().items()) if s <= a <= e]

def dcz(fn, limit=400):
    """Decompile with every `Ram00000000` access resolved: appends `// {sym}` to each line whose
    instructions carry a relocation against an undefined (address-0) symbol. Also rewrites
    `FUN_ram_xxxxxxxx(` calls that are really calls to undefined ROM functions (memset, memcpy,
    pp_printf ...) into their names. THE way to read this blob."""
    raw = decompile_raw(fn, limit=limit, disasm=True)
    try:
        o = json.loads(raw)
    except Exception:
        return f"// dcz: could not decompile {fn}: {raw[:200]}"
    zm = zsym_map(); out = []
    for e in o.get("synchronizedContent", []):
        txt = e["decompilation"]; syms = []
        for ins in e.get("assembly", []):
            a = int(ins.split(":")[0], 16)
            for cand in (a, a - 2, a + 2, a - 4, a + 4):
                if cand in zm and zm[cand][0] not in syms and abs(cand - a) <= 4:
                    syms.append(zm[cand][0]); break
        # resolve FUN_ram_<addr>( that are jumps to address 0 (undefined fns)
        for m in re.finditer(r"FUN_ram_([0-9a-f]{8})\(", txt):
            a = int(m.group(1), 16)
            for cand in (a, a - 4, a + 4, a - 2, a + 2):
                if cand in zm:
                    txt = txt.replace(m.group(0), zm[cand][0] + "("); break
        out.append(f"{e['lineNumber']:4}\t{txt}" + (f"   // {', '.join(syms)}" if syms else ""))
    return "\n".join(out)

def dumpz(fns, path, limit=400):
    """dcz() a list of functions into one file."""
    with open(path, "w") as f:
        for fn in fns:
            f.write(f"\n===== {fn} =====\n"); f.write(dcz(fn, limit=limit))
    return f"wrote {path} ({len(fns)} functions)"

def disz(fn, elf=None):
    """Disassembly (llvm-objdump) of a function with address-0 symbols resolved per instruction.
    Needed where the decompiler dead-store-eliminates consecutive stores to `*Ram00000000`
    (e.g. wdev_data_init / net80211_data_ptr_init pointer-registration functions)."""
    import subprocess
    elf = elf or _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "blobs", "esp32c6-wifi.elf")
    o = json.loads(decompile_raw(fn, limit=1))
    s, e = int(o["startAddress"], 16), int(o["endAddress"], 16) + 1
    txt = subprocess.check_output(["llvm-objdump", "-d", "--no-show-raw-insn", "-M", "no-aliases",
        f"--start-address={s:#x}", f"--stop-address={e:#x}", elf]).decode()
    zm = zsym_map(); out = []
    for ln in txt.splitlines():
        m = re.match(r"\s*([0-9a-f]+):\s*(.*)", ln)
        if not m: continue
        a = int(m.group(1), 16); tag = zm.get(a)
        out.append(f"{a:#x}: {m.group(2):40s}" + (f" ; {tag[0]}" if tag else ""))
    return "\n".join(out)
