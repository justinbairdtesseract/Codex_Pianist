#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEFAULT_STAGE="${ROOT_DIR}/simulation/scenes/pianist_lab.usda"
STAGE_PATH="${1:-${DEFAULT_STAGE}}"

if [[ "${STAGE_PATH}" != /* ]]; then
  STAGE_PATH="${ROOT_DIR}/${STAGE_PATH}"
fi
STAGE_PATH="$(realpath -m "${STAGE_PATH}")"

if [[ ! -f "${STAGE_PATH}" ]]; then
  echo "[ERROR] Stage not found: ${STAGE_PATH}"
  exit 1
fi

if ! command -v isaacsim >/dev/null 2>&1; then
  echo "[ERROR] 'isaacsim' command not found. Activate your Isaac environment first."
  exit 1
fi

export OMNI_KIT_ACCEPT_EULA=YES
export LD_LIBRARY_PATH="${ROOT_DIR}/.compat-libs:${LD_LIBRARY_PATH:-}"
export PIANIST_STAGE_PATH="${STAGE_PATH}"
export PIANIST_ARM_CENTER_DEG="${PIANIST_ARM_CENTER_DEG:--12.9,-18.1,-25.5,-15.1,73.2,-45.0}"

echo "[INFO] Launching Isaac Sim with stage: ${STAGE_PATH}"
echo "[INFO] Applying center pose (deg): ${PIANIST_ARM_CENTER_DEG}"

exec isaacsim \
  --/app/file="${STAGE_PATH}" \
  --exec "${ROOT_DIR}/simulation/open_stage_center_pose_on_startup.py"
