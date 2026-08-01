#!/usr/bin/env bash
set -Eeuo pipefail

package_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

for file in \
  scripts/wait_for_316.sh \
  scripts/d_task_uav_autostart.sh \
  systemd/d-task-uav-wifi.service \
  systemd/d-task-uav-autostart.service; do
  test -f "$package_dir/$file"
done

bash -n "$package_dir/scripts/wait_for_316.sh"
bash -n "$package_dir/scripts/d_task_uav_autostart.sh"
grep -Fxq 'Requires=d-task-uav-wifi.service' \
  "$package_dir/systemd/d-task-uav-autostart.service"
grep -Fxq 'After=d-task-uav-wifi.service' \
  "$package_dir/systemd/d-task-uav-autostart.service"
