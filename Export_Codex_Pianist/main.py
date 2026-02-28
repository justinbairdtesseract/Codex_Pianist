from __future__ import annotations

import logging
import os

from drivers import InspireHandConfig, InspireHandDriver, XArmConfig, XArmDriver
from intelligence import PianoLogicEngine


def configure_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )


def main() -> None:
    configure_logging()
    logger = logging.getLogger("pianist_robot_v1")

    arm_ip = os.getenv("XARM_IP", "192.168.0.203")
    hand_port = os.getenv("HAND_MODBUS_PORT", "/dev/ttyUSB0")
    hand_slave = int(os.getenv("HAND_MODBUS_SLAVE_ID", "1"))

    arm = XArmDriver(XArmConfig(robot_ip=arm_ip))
    hand = InspireHandDriver(InspireHandConfig(port=hand_port, slave_id=hand_slave))
    logic = PianoLogicEngine()

    logger.info("Starting pianist_robot_v1 bringup")
    logger.info("xArm target IP: %s", arm_ip)
    logger.info("Hand Modbus port/slave: %s / %s", hand_port, hand_slave)

    arm_ok = arm.connect()
    hand_ok = hand.connect()
    logger.info("Arm connected: %s | Hand connected: %s", arm_ok, hand_ok)

    logger.info("Startup note plan: %s", logic.get_startup_notes())

    hand.disconnect()
    arm.disconnect()
    logger.info("Bringup complete")


if __name__ == "__main__":
    main()
