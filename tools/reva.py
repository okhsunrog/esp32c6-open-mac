#!/usr/bin/env python3
"""Minimal ReVa (Ghidra) MCP-over-HTTP client for Bash/agent use. Non-blocking on SSE.
Usage:
  reva.py list-tools
  reva.py call <tool-name> '<json-args>'
Examples:
  reva.py call list-open-programs '{}'
  reva.py call get-decompilation '{"programPath":"/esp32c6-wifi.elf","functionName":"ppTxPkt"}'
  reva.py call create-label '{"programPath":"/esp32c6-wifi.elf","address":"0x600a4d6c","name":"WDEV_TXQ_CONF2"}'
"""
import sys, json, http.client
_id=0
def rpc(method, params, sid=None, want_resp=True):
    global _id; _id+=1; myid=_id
    body=json.dumps({"jsonrpc":"2.0","id":myid,"method":method,"params":params})
    h={"Content-Type":"application/json","Accept":"application/json, text/event-stream"}
    if sid: h["Mcp-Session-Id"]=sid
    c=http.client.HTTPConnection("localhost",8080,timeout=120)
    c.request("POST","/mcp/message",body,h)
    r=c.getresponse()
    newsid=r.getheader("Mcp-Session-Id") or sid
    if not want_resp:
        c.close(); return newsid, None
    out=None; buf=b""
    while True:
        chunk=r.read(1)
        if not chunk: break
        buf+=chunk
        if chunk==b"\n":
            line=buf.decode("utf-8","replace").strip(); buf=b""
            if line.startswith("data:"): line=line[5:].strip()
            if line.startswith("{"):
                try:
                    o=json.loads(line)
                    if o.get("id")==myid: out=o; break
                except: pass
    c.close()
    return newsid, out
def session():
    sid,_=rpc("initialize",{"protocolVersion":"2024-11-05","capabilities":{},"clientInfo":{"name":"bash","version":"1"}})
    rpc("notifications/initialized",{},sid,want_resp=False)
    return sid
def main():
    a=sys.argv[1:]; sid=session()
    if not a or a[0]=="list-tools":
        _,r=rpc("tools/list",{},sid)
        names=[t["name"] for t in r.get("result",{}).get("tools",[])]
        print(json.dumps(names,indent=1)); return
    if a[0]=="call":
        args=json.loads(a[2]) if len(a)>2 else {}
        _,r=rpc("tools/call",{"name":a[1],"arguments":args},sid)
        if isinstance(r,dict) and "result" in r:
            for item in r["result"].get("content",[]):
                print(item.get("text", json.dumps(item)))
        else:
            print(json.dumps(r,indent=1))
        return
main()
