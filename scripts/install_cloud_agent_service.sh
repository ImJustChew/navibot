#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="navibot-cloud-agent.service"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_SRC="${REPO_DIR}/deploy/systemd/${SERVICE_NAME}"
SERVICE_DST="/etc/systemd/system/${SERVICE_NAME}"
ENV_DIR="/etc/navibot"
ENV_FILE="${ENV_DIR}/cloud-agent.env"

BACKEND_URL="${NAVIBOT_BACKEND_URL:-${1:-}}"
ROBOT_ID="${NAVIBOT_ROBOT_ID:-${2:-devbot}}"
ROBOT_TOKEN="${NAVIBOT_ROBOT_TOKEN:-${3:-}}"

if [[ ! -f "${SERVICE_SRC}" ]]; then
  echo "Missing service file: ${SERVICE_SRC}" >&2
  exit 1
fi

if [[ -z "${BACKEND_URL}" ]]; then
  echo "Missing backend URL. Pass it as arg 1 or set NAVIBOT_BACKEND_URL." >&2
  exit 1
fi

if [[ -z "${ROBOT_TOKEN}" ]]; then
  echo "Missing robot token. Pass it as arg 3 or set NAVIBOT_ROBOT_TOKEN." >&2
  exit 1
fi

python3 -m pip install --user -e "${REPO_DIR}[rpi]"

sudo install -d -m 0750 "${ENV_DIR}"
sudo tee "${ENV_FILE}" >/dev/null <<EOF
NAVIBOT_BACKEND_URL=${BACKEND_URL}
NAVIBOT_ROBOT_ID=${ROBOT_ID}
NAVIBOT_ROBOT_TOKEN=${ROBOT_TOKEN}
EOF
sudo chmod 0640 "${ENV_FILE}"
sudo chown root:vroom "${ENV_FILE}"

sudo install -m 0644 "${SERVICE_SRC}" "${SERVICE_DST}"
sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}"
sudo systemctl restart "${SERVICE_NAME}"
sudo systemctl --no-pager --full status "${SERVICE_NAME}"
