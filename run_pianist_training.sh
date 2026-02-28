#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

source "$ROOT_DIR/pianist_robot_v1/.venv_isaac/bin/activate"
export OMNI_KIT_ACCEPT_EULA=YES

python train.py --headless "$@"
