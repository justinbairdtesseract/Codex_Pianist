from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional, Sequence

try:
    from xarm.wrapper import XArmAPI
    _XARM_IMPORT_ERROR = None
except Exception as exc:  # pragma: no cover - import availability depends on environment
    XArmAPI = None
    _XARM_IMPORT_ERROR = exc


LOGGER = logging.getLogger(__name__)


@dataclass
class XArmConfig:
    robot_ip: str
    speed: float = 100.0
    acceleration: float = 1000.0


class XArmDriver:
    """Thin connection wrapper for a UFactory xArm."""

    def __init__(self, config: XArmConfig) -> None:
        self.config = config
        self.api: Optional[XArmAPI] = None

    def connect(self) -> bool:
        if XArmAPI is None:
            LOGGER.error("xarm-python-sdk is unavailable: %s", _XARM_IMPORT_ERROR)
            return False

        try:
            self.api = XArmAPI(self.config.robot_ip, do_not_open=False, is_radian=False)
            self.api.motion_enable(enable=True)
            self.api.set_mode(0)
            self.api.set_state(0)
            # xarm-python-sdk methods vary across versions; keep connect tolerant.
            if hasattr(self.api, "set_tcp_maxacc"):
                try:
                    self.api.set_tcp_maxacc(self.config.acceleration)
                except Exception as exc:
                    LOGGER.warning("set_tcp_maxacc failed (continuing): %s", exc)
            LOGGER.info("Connected to xArm at %s", self.config.robot_ip)
            return True
        except Exception as exc:
            LOGGER.exception("Failed to connect to xArm at %s: %s", self.config.robot_ip, exc)
            self.api = None
            return False

    def disconnect(self) -> None:
        if self.api is None:
            return

        try:
            self.api.disconnect()
            LOGGER.info("Disconnected from xArm")
        except Exception as exc:
            LOGGER.warning("xArm disconnect raised: %s", exc)
        finally:
            self.api = None

    def move_joints(
        self,
        joint_angles_deg: Sequence[float],
        speed: Optional[float] = None,
        acceleration: Optional[float] = None,
        wait: bool = True,
    ) -> bool:
        if self.api is None:
            LOGGER.error("xArm is not connected.")
            return False
        if len(joint_angles_deg) != 6:
            LOGGER.error("Expected 6 joint angles for xArm, got %s", len(joint_angles_deg))
            return False

        cmd_speed = self.config.speed if speed is None else speed
        cmd_acc = self.config.acceleration if acceleration is None else acceleration
        try:
            code = self.api.set_servo_angle(
                angle=list(joint_angles_deg),
                speed=cmd_speed,
                mvacc=cmd_acc,
                wait=wait,
                is_radian=False,
            )
            if code != 0:
                LOGGER.error("xArm set_servo_angle failed with code %s", code)
                return False
            LOGGER.info("xArm moved to joint target: %s", list(joint_angles_deg))
            return True
        except Exception as exc:
            LOGGER.exception("xArm joint move failed: %s", exc)
            return False
