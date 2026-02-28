#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_DIR="${ROOT_DIR}/simulation"
SOURCES_DIR="${SIM_DIR}/sources"
GENERATED_DIR="${SIM_DIR}/generated"
ISAACLAB_DIR="${SOURCES_DIR}/IsaacLab"
XARM_ROS2_DIR="${SOURCES_DIR}/xarm_ros2"
DEX_URDF_DIR="${SOURCES_DIR}/dex-urdf"
VENV_ISAAC="${ROOT_DIR}/.venv_isaac"
COMPAT_LIBS="${ROOT_DIR}/.compat-libs"

XARM_XACRO_SRC="${XARM_ROS2_DIR}/xarm_description/urdf/xarm_device.urdf.xacro"
XARM_URDF_OUT="${GENERATED_DIR}/uf850.urdf"
XARM_URDF_ISAAC="${GENERATED_DIR}/uf850_for_isaac.urdf"
XARM_USD_OUT="${SIM_DIR}/uf850.usd"
HAND_URDF="${DEX_URDF_DIR}/robots/hands/inspire_hand/inspire_hand_right.urdf"
HAND_USD_OUT="${SIM_DIR}/inspire_rh56dfx_2r_hand.usd"
HAND_URDF_ISAAC="${GENERATED_DIR}/inspire_hand_right_for_isaac.urdf"
ADAPTER_STL="${SIM_DIR}/assets/adapter/Inspire-adapter.stl"
ADAPTER_USD_OUT="${SIM_DIR}/assets/adapter/Inspire-adapter.usda"
ASSEMBLY_CONFIG="${SIM_DIR}/assembly_config.json"

if [[ ! -x "${VENV_ISAAC}/bin/python" ]]; then
  echo "[ERROR] Missing Isaac venv at ${VENV_ISAAC}"
  exit 1
fi
if [[ ! -d "${ISAACLAB_DIR}" ]]; then
  echo "[ERROR] Missing IsaacLab repo at ${ISAACLAB_DIR}"
  exit 1
fi
if [[ ! -f "${XARM_XACRO_SRC}" ]]; then
  echo "[ERROR] Missing xArm xacro file at ${XARM_XACRO_SRC}"
  exit 1
fi
if [[ ! -f "${HAND_URDF}" ]]; then
  echo "[ERROR] Missing Inspire hand URDF at ${HAND_URDF}"
  exit 1
fi
if [[ ! -f "${ASSEMBLY_CONFIG}" ]]; then
  echo "[ERROR] Missing assembly config at ${ASSEMBLY_CONFIG}"
  exit 1
fi

# Prefer high-performance host policy before conversion.
if command -v powerprofilesctl >/dev/null 2>&1; then
  CURRENT_PROFILE="$(powerprofilesctl get || true)"
  if [[ "${CURRENT_PROFILE}" != "performance" ]]; then
    echo "[INFO] Switching system power profile to performance..."
    powerprofilesctl set performance || true
  fi
  echo "[INFO] Active power profile: $(powerprofilesctl get || echo unknown)"
fi

mkdir -p "${GENERATED_DIR}"

echo "[INFO] Installing xacro tooling into .venv_isaac (idempotent)..."
"${VENV_ISAAC}/bin/python" -m pip install -q xacro rospkg

echo "[INFO] Preparing patched xacro tree for non-ROS environment..."
PATCH_DIR="${GENERATED_DIR}/xacro_patched"
PATCH_XDESC="${PATCH_DIR}/xarm_description"
rm -rf "${PATCH_XDESC}"
mkdir -p "${PATCH_DIR}"
cp -a "${XARM_ROS2_DIR}/xarm_description" "${PATCH_XDESC}"

find "${PATCH_XDESC}" -type f -name '*.xacro' -print0 | xargs -0 sed -i "s|\$(find xarm_description)|${PATCH_XDESC}|g"
find "${PATCH_XDESC}" -type f -name '*.xacro' -print0 | xargs -0 sed -i "s|\$(find xarm_controller)|${XARM_ROS2_DIR}/xarm_controller|g"
sed -i 's/load_gazebo_plugin="true"/load_gazebo_plugin="false"/' "${PATCH_XDESC}/urdf/xarm_device.urdf.xacro"

echo "[INFO] Generating uf850 URDF from xacro..."
"${VENV_ISAAC}/bin/xacro" \
  "${PATCH_XDESC}/urdf/xarm_device.urdf.xacro" \
  robot_type:=uf850 dof:=6 \
  add_gripper:=false add_vacuum_gripper:=false add_bio_gripper:=false \
  > "${XARM_URDF_OUT}"

echo "[INFO] Rewriting package:// mesh URIs to absolute file:// paths for Isaac importer..."
sed "s|package://xarm_description|file://${XARM_ROS2_DIR}/xarm_description|g" "${XARM_URDF_OUT}" > "${XARM_URDF_ISAAC}"

export TERM=xterm
export OMNI_KIT_ACCEPT_EULA=YES
export LD_LIBRARY_PATH="${COMPAT_LIBS}:${LD_LIBRARY_PATH:-}"

source "${VENV_ISAAC}/bin/activate"

echo "[INFO] Converting uf850 URDF -> USD (with --fix-base)..."
(
  cd "${ISAACLAB_DIR}"
  ./isaaclab.sh -p scripts/tools/convert_urdf.py \
    "${XARM_URDF_ISAAC}" \
    "${XARM_USD_OUT}" \
    --fix-base \
    --headless
)

echo "[INFO] Converting Inspire RH56DFX-2R hand URDF -> USD (without --fix-base)..."
echo "[INFO] Preparing Isaac-safe Inspire hand URDF..."
"${VENV_ISAAC}/bin/python" "${SIM_DIR}/prepare_hand_urdf_for_isaac.py" \
  "${HAND_URDF}" \
  "${HAND_URDF_ISAAC}" \
  --absolute-paths

echo "[INFO] Converting patched Inspire RH56DFX-2R hand URDF -> USD (without --fix-base)..."
(
  cd "${ISAACLAB_DIR}"
  ./isaaclab.sh -p "${SIM_DIR}/convert_hand_urdf.py" \
    "${HAND_URDF_ISAAC}" \
    "${HAND_USD_OUT}" \
    --headless
)

if [[ -f "${ADAPTER_STL}" ]]; then
  echo "[INFO] Converting Inspire adapter STL -> USD..."
  (
    cd "${ISAACLAB_DIR}"
    ./isaaclab.sh -p scripts/tools/convert_mesh.py \
      "${ADAPTER_STL}" \
      "${ADAPTER_USD_OUT}" \
      --collision-approximation convexHull \
      --mass 0.12 \
      --headless
  )
else
  echo "[WARN] Adapter STL not found at ${ADAPTER_STL}; skipping adapter conversion."
fi

echo "[INFO] Building combined pianist assembly USD..."
"${VENV_ISAAC}/bin/python" "${SIM_DIR}/build_pianist_assembly.py" --config "${ASSEMBLY_CONFIG}"

echo "[INFO] Done."
echo "       xArm USD : ${XARM_USD_OUT}"
echo "       Hand USD : ${HAND_USD_OUT}"
echo "       Adapter  : ${ADAPTER_USD_OUT}"
echo "       Assembly : ${SIM_DIR}/pianist_robot.usda"
