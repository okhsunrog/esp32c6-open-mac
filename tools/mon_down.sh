#!/bin/sh
# Remove mon0 and restore wlan0 to managed (NetworkManager reconnects).
sudo -n iw dev mon0 del 2>/dev/null || true
sudo -n ip link set wlan0 down 2>/dev/null || true
sudo -n iw dev wlan0 set type managed 2>/dev/null || true
sudo -n ip link set wlan0 up 2>/dev/null || true
nmcli dev set wlan0 managed yes >/dev/null 2>&1 || true
nmcli dev connect wlan0 >/dev/null 2>&1 || true
sleep 2; nmcli -t -f DEVICE,STATE,CONNECTION dev status | grep wlan0 || true
