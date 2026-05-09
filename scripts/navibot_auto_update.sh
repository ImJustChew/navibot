#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="/home/vroom/navibot"
BRANCH="${NAVIBOT_AUTO_UPDATE_BRANCH:-main}"
LOCK_FILE="/tmp/navibot-auto-update.lock"

exec 9>"${LOCK_FILE}"
flock -n 9 || exit 0

cd "${REPO_DIR}"

git fetch --prune origin "${BRANCH}"

LOCAL_HEAD="$(git rev-parse HEAD)"
REMOTE_HEAD="$(git rev-parse "origin/${BRANCH}")"

if [[ "${LOCAL_HEAD}" == "${REMOTE_HEAD}" ]]; then
  echo "Navibot already up to date at ${LOCAL_HEAD}"
  exit 0
fi

git pull --ff-only origin "${BRANCH}"
python3 -m pip install --user -e "${REPO_DIR}[rpi]"

systemctl restart navibot-cloud-agent.service
echo "Navibot updated from ${LOCAL_HEAD} to ${REMOTE_HEAD}"
