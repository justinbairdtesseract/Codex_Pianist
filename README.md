# Codex Pianist

Codex Pianist is a robotic piano project built around:

- a UFactory `UF850` arm
- an `Inspire RH56DFX-2R` robotic hand
- a custom 3D-printed adapter that mounts the hand to the arm
- an Isaac Sim / Isaac Lab simulation workflow for assembly, scene validation, and reinforcement-learning preparation
- a direct hardware-control workflow for simple scripted arm and hand motion outside the simulator

This repository contains the first working public snapshot of that system.

## Built In Codex

This repository was built end-to-end through iterative engineering work in Codex.

That includes:

- direct hardware-control scripts for the arm and hand
- URDF preprocessing and conversion flow for Isaac
- STL import and scaling for the printed adapter
- assembly generation for arm + adapter + hand
- Isaac GUI launch tooling
- scene setup for the piano lab environment
- export packaging for a minimal hardware-only bundle
- repository preparation for public release

This is not a manually curated example repo that was later summarized in AI. The implementation work itself was done through Codex-driven development.

## Version

- Current repository snapshot: `v1.0.0`
- Simulation version marker: [pianist_robot_v1/simulation/VERSION](pianist_robot_v1/simulation/VERSION)
- Frozen simulation date: `2026-02-27`

Version `1.0.0` captures the first assembled simulation with:

- UF850 arm
- Inspire RH56DFX-2R hand
- custom printed hand adapter
- working Isaac scene launch
- working direct arm/hand command scripts

## What Is In This Repo

The repository currently has two main tracks.

### 1. Full Project

[pianist_robot_v1](pianist_robot_v1) contains:

- direct hardware control for the real arm and hand
- arm calibration and note-test routines
- simulation asset conversion scripts
- generated USD and URDF outputs
- assembled Isaac scene files
- piano-lab simulation environment used as the starting point for RL work

### 2. Portable Hardware Export

[Export_Codex_Pianist](Export_Codex_Pianist) contains:

- only the files needed to connect to and command the real arm and hand
- no simulation assets
- no Isaac setup
- no modeling pipeline

That folder is intended for moving simple hardware control to another machine quickly.

## Hardware Stack

The current physical system uses:

- `UFactory UF850` arm
- `Inspire RH56DFX-2R` hand
- RS485 / Modbus RTU hand communication
- Ethernet arm communication
- custom `Inspire-adapter.stl` printed adapter for mechanical mounting

## Software Stack

The current project uses:

- Python
- `xarm-python-sdk` for arm control
- `minimalmodbus` and `pyserial` for hand control
- NVIDIA Isaac Sim
- Isaac Lab tooling for URDF to USD conversion workflows
- USD / USDA assets for robot assembly and scene composition
- imported upstream robot-description sources as pinned submodules

## Simulation Stack

The simulation path is centered in [pianist_robot_v1/simulation](pianist_robot_v1/simulation).

Key outputs in `v1.0.0`:

- [uf850.usd](pianist_robot_v1/simulation/uf850.usd)
- [inspire_rh56dfx_2r_hand.usd](pianist_robot_v1/simulation/inspire_rh56dfx_2r_hand.usd)
- [assets/adapter/Inspire-adapter.usda](pianist_robot_v1/simulation/assets/adapter/Inspire-adapter.usda)
- [pianist_robot.usda](pianist_robot_v1/simulation/pianist_robot.usda)
- [scenes/pianist_lab.usda](pianist_robot_v1/simulation/scenes/pianist_lab.usda)

The assembled scene includes:

- the UF850 arm
- the Inspire hand
- the custom adapter
- a ground plane
- a table
- a keyboard proxy for motion testing

Detailed simulation notes are in [pianist_robot_v1/simulation/README.md](pianist_robot_v1/simulation/README.md).

## Direct Hardware Control

The direct-control path is implemented with these scripts:

- [check_hardware.py](pianist_robot_v1/check_hardware.py)
- [startup_test.py](pianist_robot_v1/startup_test.py)
- [four_finger_note_test.py](pianist_robot_v1/four_finger_note_test.py)
- [continuous_practice.py](pianist_robot_v1/continuous_practice.py)
- [play_center_calibration.py](pianist_robot_v1/play_center_calibration.py)
- [main.py](pianist_robot_v1/main.py)

Supporting modules:

- [drivers/xarm_driver.py](pianist_robot_v1/drivers/xarm_driver.py)
- [drivers/hand_modbus_driver.py](pianist_robot_v1/drivers/hand_modbus_driver.py)
- [intelligence/piano_logic.py](pianist_robot_v1/intelligence/piano_logic.py)

## Upstream Sources

This repository uses submodules for external source dependencies under `pianist_robot_v1/simulation/sources`:

- `IsaacLab`
- `dex-urdf`
- `xarm_ros2`

These are pinned to the exact commits used for `v1.0.0`.

Clone with submodules:

```bash
git clone --recurse-submodules https://github.com/justinbairdtesseract/Codex_Pianist.git
```

Or, after clone:

```bash
git submodule update --init --recursive
```

## Quick Start

### Hardware-Only Export

If you only want to move the real arm and hand from another machine, use:

- [Export_Codex_Pianist](Export_Codex_Pianist)

That bundle has its own [README.md](Export_Codex_Pianist/README.md).

### Isaac Simulation

From the main project:

```bash
cd pianist_robot_v1
source .venv_isaac/bin/activate
./simulation/run_isaac_gui.sh simulation/scenes/pianist_lab.usda
```

### Direct Hardware Sanity Check

```bash
cd pianist_robot_v1
source .venv/bin/activate
python check_hardware.py --show-all-usb-serial
python startup_test.py --hand-port auto
```

## Current Status

At `v1.0.0`, the project has:

- working hardware-control scripts
- a working first assembled simulation
- working GUI launch into Isaac
- a portable direct-control export
- a clean public repository baseline for hackathon development

The next major phase is reinforcement learning on top of the Isaac simulation environment.

## Repository Notes

- Local Isaac virtual environments are intentionally excluded from Git.
- The repo keeps generated assembly assets that are part of the working `v1.0.0` snapshot.
- Upstream robot-description dependencies are tracked as submodules rather than copied vendor trees.

## License / Attribution

This repository includes original project code plus references to upstream open-source dependencies through submodules.

Please review the licenses in the upstream projects for:

- `IsaacLab`
- `dex-urdf`
- `xarm_ros2`
