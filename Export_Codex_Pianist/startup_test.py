from __future__ import annotations

import argparse
import logging
import os
import sys
from typing import Sequence

from check_hardware import DEFAULT_KEYWORDS, find_rs485_ports, ping_host
DEFAULT_XARM_IP = "192.168.0.203"
DEFAULT_HAND_PORT = "/dev/ttyUSB0"
DEFAULT_ARM_HOME_JOINTS_DEG = (0.0, 0.0, 0.0, 0.0, 90.0, -45.0)
DEFAULT_ARM_TEST_JOINTS_DEG = (0.0, 0.0, -15.0, 0.0, 110.0, -45.0)


def configure_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )


def _parse_joint_targets(text: str) -> list[float]:
    parts = [p.strip() for p in text.split(",") if p.strip()]
    if len(parts) != 6:
        raise ValueError(f"Expected 6 comma-separated joint values, got {len(parts)}")
    return [float(p) for p in parts]


def _resolve_hand_port(requested_port: str) -> str:
    if requested_port != "auto":
        return requested_port

    matches, usb_ports = find_rs485_ports(DEFAULT_KEYWORDS)
    if matches:
        return matches[0].device
    if usb_ports:
        return usb_ports[0].device
    return DEFAULT_HAND_PORT


def _run_precheck(xarm_ip: str, hand_port: str, logger: logging.Logger) -> bool:
    matches, _ = find_rs485_ports(DEFAULT_KEYWORDS)
    ping_ok, ping_output = ping_host(xarm_ip, timeout_seconds=1, count=2)

    device_present = any(getattr(port, "device", "") == hand_port for port in matches) or os.path.exists(hand_port)
    device_access = os.access(hand_port, os.R_OK | os.W_OK) if os.path.exists(hand_port) else False
    hand_ok = device_present and device_access

    logger.info("Precheck xArm ping: %s", "PASS" if ping_ok else "FAIL")
    logger.info("Precheck hand port: %s (%s)", hand_port, "PASS" if hand_ok else "FAIL")
    if not ping_ok:
        logger.warning("xArm ping output: %s", ping_output.replace("\n", " | "))
    if device_present and not device_access:
        logger.warning(
            "Hand serial device exists but current user lacks read/write permissions: %s",
            hand_port,
        )
    return ping_ok and hand_ok


def run_startup_test(
    xarm_ip: str,
    hand_port: str,
    hand_slave_id: int,
    arm_test_joint_targets: Sequence[float],
    arm_home_joint_targets: Sequence[float],
    arm_speed: float,
    arm_acceleration: float,
    hand_open_value: int,
    hand_close_value: int,
    hand_hold_seconds: float,
) -> bool:
    from drivers import InspireHandConfig, InspireHandDriver, XArmConfig, XArmDriver

    logger = logging.getLogger("startup_test")
    arm = XArmDriver(XArmConfig(robot_ip=xarm_ip, speed=arm_speed, acceleration=arm_acceleration))
    hand = InspireHandDriver(InspireHandConfig(port=hand_port, slave_id=hand_slave_id))

    arm_ok = False
    hand_ok = False
    arm_home_reached = False
    try:
        logger.info("Connecting xArm at %s", xarm_ip)
        arm_ok = arm.connect()
        if not arm_ok:
            logger.error("xArm connection failed.")
            return False

        logger.info("Moving xArm to default home pose first: %s", list(arm_home_joint_targets))
        if not arm.move_joints(arm_home_joint_targets, speed=arm_speed, acceleration=arm_acceleration, wait=True):
            logger.error("xArm failed to reach default home pose.")
            return False

        logger.info("Connecting Inspire hand on %s (slave %s)", hand_port, hand_slave_id)
        hand_ok = hand.connect()
        if not hand_ok:
            logger.error("Inspire hand connection failed.")
            return False

        logger.info("Ensuring hand starts fully open (all motors -> %s)", hand_open_value)
        if not hand.set_all_motors(hand_open_value, hold_seconds=hand_hold_seconds):
            logger.error("Failed to command initial full-hand open.")
            return False

        logger.info("Moving xArm to startup test pose: %s", list(arm_test_joint_targets))
        if not arm.move_joints(arm_test_joint_targets, speed=arm_speed, acceleration=arm_acceleration, wait=True):
            logger.error("xArm failed to reach startup test pose.")
            return False

        logger.info("Cycling all hand motors (open=%s, close=%s)", hand_open_value, hand_close_value)
        if not hand.cycle_all_motors(
            open_value=hand_open_value,
            close_value=hand_close_value,
            hold_seconds=hand_hold_seconds,
        ):
            logger.error("Hand motor cycle test failed.")
            return False

        logger.info("Opening hand fully (all motors -> %s)", hand_open_value)
        if not hand.set_all_motors(hand_open_value, hold_seconds=hand_hold_seconds):
            logger.error("Failed to command full hand open.")
            return False

        logger.info("Returning xArm to default home pose: %s", list(arm_home_joint_targets))
        if not arm.move_joints(arm_home_joint_targets, speed=arm_speed, acceleration=arm_acceleration, wait=True):
            logger.error("xArm failed to return to default home pose.")
            return False
        arm_home_reached = True

        logger.info("Startup test completed successfully.")
        return True
    finally:
        if arm_ok and not arm_home_reached:
            logger.warning("Attempting safe return to home pose: %s", list(arm_home_joint_targets))
            arm.move_joints(arm_home_joint_targets, speed=arm_speed, acceleration=arm_acceleration, wait=True)
        if hand_ok:
            hand.disconnect()
        if arm_ok:
            arm.disconnect()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Startup hardware routine: move xArm to default pose and cycle all Inspire hand motors."
    )
    parser.add_argument("--xarm-ip", default=DEFAULT_XARM_IP, help=f"xArm IP (default: {DEFAULT_XARM_IP})")
    parser.add_argument(
        "--hand-port",
        default="auto",
        help=f"Hand serial port (default: auto; fallback {DEFAULT_HAND_PORT})",
    )
    parser.add_argument("--hand-slave-id", type=int, default=1, help="Modbus slave ID for Inspire hand")
    parser.add_argument(
        "--test-arm-joints",
        default=",".join(str(v) for v in DEFAULT_ARM_TEST_JOINTS_DEG),
        help="Comma-separated 6 joint angles in degrees for startup test pose",
    )
    parser.add_argument(
        "--home-arm-joints",
        default=",".join(str(v) for v in DEFAULT_ARM_HOME_JOINTS_DEG),
        help="Comma-separated 6 joint angles in degrees for default home pose",
    )
    parser.add_argument("--arm-speed", type=float, default=40.0, help="xArm joint move speed")
    parser.add_argument("--arm-acc", type=float, default=400.0, help="xArm joint move acceleration")
    parser.add_argument("--hand-open", type=int, default=1000, help="Hand open target value")
    parser.add_argument("--hand-close", type=int, default=120, help="Hand close target value")
    parser.add_argument("--hand-hold", type=float, default=0.35, help="Hold time per hand move (seconds)")
    parser.add_argument(
        "--skip-precheck",
        action="store_true",
        help="Skip connectivity precheck (ping + hand port sanity)",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Continue startup test even if precheck fails",
    )
    args = parser.parse_args()

    configure_logging()
    logger = logging.getLogger("startup_test")

    try:
        arm_test_targets = _parse_joint_targets(args.test_arm_joints)
        arm_home_targets = _parse_joint_targets(args.home_arm_joints)
    except ValueError as exc:
        logger.error("%s", exc)
        return 2

    hand_port = _resolve_hand_port(args.hand_port)
    logger.info("Resolved hand port: %s", hand_port)

    precheck_ok = True
    if not args.skip_precheck:
        precheck_ok = _run_precheck(args.xarm_ip, hand_port, logger)
        if not precheck_ok and not args.force:
            logger.error("Precheck failed. Aborting startup routine.")
            logger.error("Use --force to run anyway, or fix connectivity and retry.")
            return 1

    ok = run_startup_test(
        xarm_ip=args.xarm_ip,
        hand_port=hand_port,
        hand_slave_id=args.hand_slave_id,
        arm_test_joint_targets=arm_test_targets,
        arm_home_joint_targets=arm_home_targets,
        arm_speed=args.arm_speed,
        arm_acceleration=args.arm_acc,
        hand_open_value=args.hand_open,
        hand_close_value=args.hand_close,
        hand_hold_seconds=args.hand_hold,
    )
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
