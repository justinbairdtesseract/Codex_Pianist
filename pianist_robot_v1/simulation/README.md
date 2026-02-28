# simulation

Simulation assets for `pianist_robot_v1`.

## Version
- Current simulation version: `1.0.0`
- Frozen on: `2026-02-27`
- Scope: arm + adapter + hand assembly in `simulation/pianist_robot.usda` and scene launch via `simulation/scenes/pianist_lab.usda`

## Source Repositories
- `simulation/sources/xarm_ros2` (UFactory)
- `simulation/sources/dex-urdf` (community dex hand URDFs)
- `simulation/sources/IsaacLab` (URDF -> USD conversion tool)

## Key Inputs
- xArm source xacro: `simulation/sources/xarm_ros2/xarm_description/urdf/uf850/uf850.urdf.xacro`
- Generated xArm URDF: `simulation/generated/uf850.urdf`
- Isaac-ready xArm URDF: `simulation/generated/uf850_for_isaac.urdf`
- Hand URDF: `simulation/sources/dex-urdf/robots/hands/inspire_hand/inspire_hand_right.urdf`
- Isaac-safe hand URDF: `simulation/generated/inspire_hand_right_for_isaac.urdf`

## Generated USD Outputs
- `simulation/uf850.usd` (converted with `--fix-base`)
- `simulation/inspire_rh56dfx_2r_hand.usd` (converted without `--fix-base`)
- `simulation/assets/adapter/Inspire-adapter.usda` (STL converted mesh)
- `simulation/pianist_robot.usda` (arm + adapter + hand fixed-joint assembly)

## Scene Assembly
- Scene file: `simulation/scenes/pianist_lab.usda`
- Includes:
  - Physics scene + gravity
  - Ground, table, keyboard proxy colliders
  - Referenced assembled robot at `/World/PianistRobot`

## GUI Launch
Use the wrapper so Isaac picks up local compatibility libs on Ubuntu 25.10:

```bash
cd /home/justin-baird/Codex_Pianist/pianist_robot_v1
source .venv_isaac/bin/activate
./simulation/run_isaac_gui.sh simulation/scenes/pianist_lab.usda
```

## Joint Introspection
Generate canonical joint map from the scene:

```bash
cd /home/justin-baird/Codex_Pianist/pianist_robot_v1
source .venv_isaac/bin/activate
OMNI_KIT_ACCEPT_EULA=YES python simulation/generate_joint_map.py --headless --stage simulation/scenes/pianist_lab.usda --output simulation/joint_map.yaml
```

Output:
- `simulation/joint_map.yaml` (arm joint order, hand finger groups, joint limits, default play-center pose)

## Smoke Test (Piano Motion)
Run a simulation smoke test that:
- moves arm to play-center pose
- fully opens the hand
- cycles index -> middle -> ring -> pinky press/release sequence

```bash
cd /home/justin-baird/Codex_Pianist/pianist_robot_v1
source .venv_isaac/bin/activate
OMNI_KIT_ACCEPT_EULA=YES python simulation/smoke_test_piano_motion.py --headless --stage simulation/scenes/pianist_lab.usda --joint-map simulation/joint_map.yaml --loops 2
```

## Re-run Conversion
From the project root:

```bash
cd /home/justin-baird/Codex_Pianist/pianist_robot_v1
./simulation/convert_assets.sh
```

The script handles:
- xacro preprocessing for non-ROS environments
- xArm URDF generation
- mesh URI normalization for Isaac importer
- hand URDF visual patching (collision `OBJ` visuals first, with optional baked `GLB` -> `OBJ` fallback)
- Isaac Lab conversion flags (`--fix-base` for xArm only)
