# Export_Codex_Pianist

Minimal hardware-control export for the Codex Pianist project.

This bundle contains only the code needed to:

- detect the xArm and the Inspire hand hardware
- send direct xArm joint commands
- send direct Inspire hand Modbus commands
- run simple startup, calibration, and note-test routines

It intentionally does not include:

- Isaac Sim
- USD/URDF assets
- simulation scenes
- modeling or assembly files

## Contents

- `main.py`: minimal bringup check
- `check_hardware.py`: xArm ping + USB serial scan
- `startup_test.py`: move arm and cycle all hand motors
- `four_finger_note_test.py`: repeated 4-finger note test
- `continuous_practice.py`: continuous practice loop until Ctrl+C
- `play_center_calibration.py`: center-pose calibration for the arm
- `play_center_calibration_results.json`: latest saved calibration result
- `drivers/`: xArm and Inspire hand drivers
- `intelligence/`: small logic helper used by `main.py`

## Python Requirements

Install these packages in a fresh virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Hardware Inputs

Set these values for your machine:

- `XARM_IP`: IP address of the arm
- `HAND_MODBUS_PORT`: serial device for the RS485 adapter, for example `/dev/ttyUSB0`
- `HAND_MODBUS_SLAVE_ID`: usually `1`

You can export them manually:

```bash
export XARM_IP=192.168.0.203
export HAND_MODBUS_PORT=/dev/ttyUSB0
export HAND_MODBUS_SLAVE_ID=1
```

## Recommended First Run

From inside this folder:

```bash
source .venv/bin/activate
python check_hardware.py --xarm-ip "$XARM_IP" --show-all-usb-serial
python startup_test.py --xarm-ip "$XARM_IP" --hand-port auto
```

## Common Commands

Minimal bringup:

```bash
python main.py
```

Calibrate play center:

```bash
python play_center_calibration.py --xarm-ip "$XARM_IP" --execute
```

Run the 4-finger note test:

```bash
python four_finger_note_test.py --xarm-ip "$XARM_IP" --hand-port auto --use-calibration-center
```

Run continuous practice:

```bash
python continuous_practice.py --xarm-ip "$XARM_IP" --hand-port auto --use-calibration-center
```

## Notes

- Run commands from this folder so `play_center_calibration_results.json` is found correctly.
- If the serial device exists but access fails, add your user to the correct serial-access group or use the appropriate device permissions for that machine.
- The hand driver is a direct Modbus wrapper. Verify the slave ID and serial adapter before sending motion commands.
