#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="/home/vroom/navibot"
BRANCH="${NAVIBOT_AUTO_UPDATE_BRANCH:-main}"
LOCK_FILE="/tmp/navibot-auto-update.lock"
RUN_AS_USER="${NAVIBOT_AUTO_UPDATE_USER:-vroom}"

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
sudo -u "${RUN_AS_USER}" python3 -m pip install --user -e "${REPO_DIR}[rpi]"

systemctl restart navibot-cloud-agent.service
echo "Navibot updated from ${LOCAL_HEAD} to ${REMOTE_HEAD}"
