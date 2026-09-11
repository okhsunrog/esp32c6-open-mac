#!/bin/sh
# Rename the CROSS-BOUNDARY symbols (those referenced from other objects) of one or more
# libpp.a objects to blob_<fn>, so a de-blob build can re-provide <fn> (shim or Rust) and
# have the blob's callers bind to it. Re-derives from libpp.a.orig each run, so the set of
# patched objects is exactly the arguments (idempotent, deterministic).
#
# Usage: patch_libpp.sh <object.o> [<object.o> ...]     e.g.  patch_libpp.sh hal_mac_tx.o lmac.o
# Edit BR for your esp-wifi-sys checkout / chip variant. After running: cargo clean -p esp-wifi-sys-esp32c6
set -e
BR=${BR:-/home/okhsunrog/.cargo/git/checkouts/esp-wifi-sys-6a72a2ac9439b52d/2ea8e3e/esp-wifi-sys-esp32c6}
LIB="$BR/libs/libpp.a"
[ -f "$LIB.orig" ] || cp "$LIB" "$LIB.orig"
[ "$#" -ge 1 ] || { echo "usage: $0 <object.o> [<object.o> ...]"; exit 2; }
work=$(mktemp -d); cd "$work"; cp "$LIB.orig" libpp.a; ar x libpp.a
cp "$LIB.orig" out.a
for obj in "$@"; do
  [ -f "$obj" ] || { echo "no such object: $obj"; exit 1; }
  nm --defined-only "$obj" | awk '$2 ~ /^[TtWw]$/ && $3 !~ /^\.L/ {print $3}' | sort -u > defs.txt
  : > extU.txt
  for o in *.o; do [ "$o" = "$obj" ] && continue; nm -u "$o" 2>/dev/null | awk '{print $NF}'; done | sort -u >> extU.txt
  sort -u extU.txt -o extU.txt
  comm -12 defs.txt extU.txt > cross.txt
  echo "$obj: $(wc -l < cross.txt) cross-boundary symbols -> blob_"
  awk '{print $1" blob_"$1}' cross.txt > redef.txt
  rust-objcopy --redefine-syms=redef.txt "$obj" "$obj"
  ar r out.a "$obj" >/dev/null 2>&1
  cp cross.txt "/tmp/${obj%.o}_cross.txt"
done
ranlib out.a 2>/dev/null || true
cp out.a "$LIB"
echo "installed patched libpp.a ($(stat -c%s "$LIB") bytes; orig $(stat -c%s "$LIB.orig"))"
cd /; rm -rf "$work"
