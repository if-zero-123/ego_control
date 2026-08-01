#!/usr/bin/env bash
set -Eeuo pipefail

ssid="${D_TASK_WIFI_SSID:-316}"
iface="${D_TASK_WIFI_IFACE:-wlan0}"

if [[ "${1:-}" == "--check" ]]; then
  command -v nmcli >/dev/null
  nmcli -g connection.id connection show "$ssid" | grep -Fxq "$ssid"
  exit 0
fi

command -v nmcli >/dev/null
until nmcli -g GENERAL.CONNECTION device show "$iface" 2>/dev/null | grep -Fxq "$ssid"; do
  nmcli radio wifi on || true
  nmcli -w 30 connection up id "$ssid" ifname "$iface" || true
  sleep 3
done
