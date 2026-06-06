#!/usr/bin/env bash
set -Eeuo pipefail

SERVICE_NAME="craic-autostart.service"

if systemctl list-unit-files "$SERVICE_NAME" >/dev/null 2>&1; then
  sudo systemctl disable --now "$SERVICE_NAME"
  sudo systemctl reset-failed "$SERVICE_NAME" >/dev/null 2>&1 || true
else
  echo "$SERVICE_NAME is not installed"
  exit 0
fi

echo
systemctl --no-pager --lines=20 status "$SERVICE_NAME" || true
