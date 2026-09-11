set -e
BR=/home/okhsunrog/.cargo/git/checkouts/esp-wifi-sys-6a72a2ac9439b52d/2ea8e3e/esp-wifi-sys-esp32c6
LIB="$BR/libs/libpp.a"
work=$(mktemp -d); cd "$work"; cp "$LIB.orig" libpp.a; ar x libpp.a
nm --defined-only hal_mac_tx.o | awk '$2 ~ /^[TtRrDdBbVvWw]$/ && $3 !~ /^\.L/ {print $3}' | sort -u > defs.txt
: > extU.txt
for o in *.o; do [ "$o" = hal_mac_tx.o ] && continue; nm -u "$o" 2>/dev/null | awk '{print $NF}'; done | sort -u > extU.txt
comm -12 defs.txt extU.txt > cross.txt
awk '{print $1" blob_"$1}' cross.txt > redef.txt
rust-objcopy --redefine-syms=redef.txt hal_mac_tx.o hal_mac_tx.o
cp "$LIB.orig" libpp_new.a
ar r libpp_new.a hal_mac_tx.o >/dev/null 2>&1
ranlib libpp_new.a 2>/dev/null || true
cp libpp_new.a "$LIB"
cp cross.txt /tmp/hal_mac_tx_cross.txt
echo "INSTALLED patched libpp.a: $(stat -c%s "$LIB") vs orig $(stat -c%s "$LIB.orig")"
echo "--- how lmac.o references it (nm on extracted member) ---"
nm lmac.o | grep -E 'hal_mac_txq_enable' || echo "(lmac uses it? checking all callers of blob targets)"
echo "--- any object still referencing the UNrenamed cross symbols (should be the callers we want to hit our shims) ---"
grep -l . /dev/null >/dev/null; 
for s in hal_mac_txq_enable hal_mac_tx_set_ppdu mac_tx_set_plcp0; do
  printf "%s <- " "$s"; for o in *.o; do nm -u "$o" 2>/dev/null | grep -qE " $s$" && printf "%s " "${o%.o}"; done; echo
done
cd /; rm -rf "$work"
