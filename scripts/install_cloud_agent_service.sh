#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="navibot-cloud-agent.service"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_SRC="${REPO_DIR}/deploy/systemd/${SERVICE_NAME}"
SERVICE_DST="/etc/systemd/system/${SERVICE_NAME}"
ENV_DIR="/etc/navibot"
ENV_FILE="${ENV_DIR}/cloud-agent.env"
VENV_DIR="${REPO_DIR}/.venv"

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

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  if ! python3 -m venv "${VENV_DIR}"; then
    echo "python3 venv support is missing; installing python3-venv and retrying." >&2
    sudo apt-get update
    sudo apt-get install -y python3-venv
    python3 -m venv "${VENV_DIR}"
  fi
fi

missing_apt_packages=()
command -v swig >/dev/null 2>&1 || missing_apt_packages+=("swig")
command -v gcc >/dev/null 2>&1 || missing_apt_packages+=("build-essential")
[[ -f /usr/lib/aarch64-linux-gnu/liblgpio.so || -f /usr/lib/arm-linux-gnueabihf/liblgpio.so || -f /usr/lib/arm-linux-gnueabi/liblgpio.so ]] || missing_apt_packages+=("liblgpio-dev")
[[ -f "/usr/include/python$(python3 - <<'PY'
import sys
print(f"{sys.version_info.major}.{sys.version_info.minor}")
PY
)/Python.h" ]] || missing_apt_packages+=("python3-dev")

if (( ${#missing_apt_packages[@]} > 0 )); then
  echo "Installing native Python build prerequisites: ${missing_apt_packages[*]}" >&2
  sudo apt-get update
  sudo apt-get install -y "${missing_apt_packages[@]}"
fi

"${VENV_DIR}/bin/python" -m pip install --upgrade pip
"${VENV_DIR}/bin/python" -m pip install -e "${REPO_DIR}[rpi]"

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
