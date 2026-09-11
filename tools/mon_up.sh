#!/bin/sh
# Put the AX210 into monitor mode via a DEDICATED mon0 vif on phy0 (channel default 1).
# Switching wlan0's own type does NOT deliver frames on iwlwifi/AX210 - a separate
# monitor vif is required. Wired uplink stays up.
set -e
CH=${1:-1}
nmcli dev set wlan0 managed no >/dev/null 2>&1 || true
sudo -n ip link set wlan0 down 2>/dev/null || true
sudo -n iw dev wlan0 set type managed 2>/dev/null || true
sudo -n iw dev mon0 del 2>/dev/null || true
sudo -n iw phy phy0 interface add mon0 type monitor
sudo -n ip link set mon0 up
sudo -n iw dev mon0 set channel "$CH"
iw dev mon0 info | grep -E 'type|channel'
