#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

chmod +x "${REPO_DIR}/scripts/navibot_auto_update.sh"
sudo install -m 0644 "${REPO_DIR}/deploy/systemd/navibot-auto-update.service" "/etc/systemd/system/navibot-auto-update.service"
sudo install -m 0644 "${REPO_DIR}/deploy/systemd/navibot-auto-update.timer" "/etc/systemd/system/navibot-auto-update.timer"
sudo systemctl daemon-reload
sudo systemctl enable navibot-auto-update.timer
sudo systemctl restart navibot-auto-update.timer
sudo systemctl --no-pager --full status navibot-auto-update.timer
