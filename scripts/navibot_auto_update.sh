#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="/home/vroom/navibot"
BRANCH="${NAVIBOT_AUTO_UPDATE_BRANCH:-main}"
LOCK_FILE="/tmp/navibot-auto-update.lock"
RUN_AS_USER="${NAVIBOT_AUTO_UPDATE_USER:-vroom}"
VENV_DIR="${REPO_DIR}/.venv"

exec 9>"${LOCK_FILE}"
flock -n 9 || exit 0

cd "${REPO_DIR}"

sudo -u "${RUN_AS_USER}" git fetch --prune origin "${BRANCH}"

LOCAL_HEAD="$(sudo -u "${RUN_AS_USER}" git rev-parse HEAD)"
REMOTE_HEAD="$(sudo -u "${RUN_AS_USER}" git rev-parse "origin/${BRANCH}")"

if [[ "${LOCAL_HEAD}" == "${REMOTE_HEAD}" ]]; then
  echo "Navibot already up to date at ${LOCAL_HEAD}"
  exit 0
fi

sudo -u "${RUN_AS_USER}" git pull --ff-only origin "${BRANCH}"
if [[ -x "${VENV_DIR}/bin/python" ]] && ! grep -q "include-system-site-packages = true" "${VENV_DIR}/pyvenv.cfg"; then
  rm -rf "${VENV_DIR}"
fi
if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  sudo -u "${RUN_AS_USER}" python3 -m venv --system-site-packages "${VENV_DIR}"
fi
sudo -u "${RUN_AS_USER}" "${VENV_DIR}/bin/python" -m pip install -e "${REPO_DIR}[rpi]"

systemctl restart navibot-cloud-agent.service
echo "Navibot updated from ${LOCAL_HEAD} to ${REMOTE_HEAD}"
