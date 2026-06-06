#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SERVICE_SRC="$PKG_DIR/systemd/craic-autostart.service"
ENV_SRC="$PKG_DIR/config/craic-autostart.env.example"
SERVICE_DST="/etc/systemd/system/craic-autostart.service"
ENV_DST="/etc/default/craic-autostart"

sudo install -m 0644 "$SERVICE_SRC" "$SERVICE_DST"
if [[ ! -e "$ENV_DST" ]]; then
  sudo install -m 0644 "$ENV_SRC" "$ENV_DST"
  echo "created $ENV_DST"
else
  echo "kept existing $ENV_DST"
fi

sudo systemctl daemon-reload
sudo systemctl enable craic-autostart.service

echo "enabled craic-autostart.service"
echo "edit config: sudo nano $ENV_DST"
echo "start now: sudo systemctl restart craic-autostart.service"
echo "watch logs: journalctl -u craic-autostart.service -f"
