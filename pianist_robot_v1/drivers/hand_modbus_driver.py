from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

try:
    import minimalmodbus
    _MINIMALMODBUS_IMPORT_ERROR = None
except Exception as exc:  # pragma: no cover - import availability depends on environment
    minimalmodbus = None
    _MINIMALMODBUS_IMPORT_ERROR = exc


LOGGER = logging.getLogger(__name__)


@dataclass
class InspireHandConfig:
    port: str
    slave_id: int = 1
    baudrate: int = 115200
    timeout_seconds: float = 0.2


class InspireHandDriver:
    """Placeholder Modbus RTU wrapper for Inspire RH56DFX-2R hand."""

    # Register addresses from Inspire hand protocol docs.
    # Some firmware uses byte-style addresses (e.g., 1486), while others use register-style
    # addresses (e.g., 1486 // 2). We probe both for compatibility.
    ANGLE_SET_BASE_ADDR = 1486
    MOTOR_COUNT = 6
    NO_ACTION_VALUE = 0xFFFF  # signed -1 in uint16 representation

    def __init__(self, config: InspireHandConfig) -> None:
        self.config = config
        self.instrument: Optional[minimalmodbus.Instrument] = None

    def connect(self) -> bool:
        if minimalmodbus is None:
            LOGGER.error("minimalmodbus is unavailable: %s", _MINIMALMODBUS_IMPORT_ERROR)
            return False

        try:
            instrument = minimalmodbus.Instrument(self.config.port, self.config.slave_id)
            instrument.serial.baudrate = self.config.baudrate
            instrument.serial.timeout = self.config.timeout_seconds
            instrument.serial.bytesize = 8
            instrument.serial.parity = "N"
            instrument.serial.stopbits = 1
            instrument.mode = minimalmodbus.MODE_RTU
            self.instrument = instrument
            LOGGER.info(
                "Initialized Inspire hand Modbus on %s (slave %s)",
                self.config.port,
                self.config.slave_id,
            )
            return True
        except Exception as exc:
            LOGGER.exception("Failed to initialize Inspire hand Modbus: %s", exc)
            self.instrument = None
            return False

    def disconnect(self) -> None:
        if self.instrument is None:
            return

        try:
            self.instrument.serial.close()
            LOGGER.info("Closed Inspire hand serial connection")
        except Exception as exc:
            LOGGER.warning("Inspire hand disconnect raised: %s", exc)
        finally:
            self.instrument = None

    @staticmethod
    def _address_candidates(base_addr: int) -> tuple[int, int]:
        if base_addr % 2 == 0:
            return base_addr, base_addr // 2
        return base_addr, base_addr

    def _write_registers_with_fallback(self, base_addr: int, values: Sequence[int]) -> bool:
        if self.instrument is None:
            LOGGER.error("Inspire hand is not connected.")
            return False

        normalized = [int(v) & 0xFFFF for v in values]
        for candidate in self._address_candidates(base_addr):
            try:
                self.instrument.write_registers(candidate, normalized)
                LOGGER.debug(
                    "Wrote %s values to hand registers starting at %s",
                    len(normalized),
                    candidate,
                )
                return True
            except Exception as exc:
                LOGGER.debug("Hand register write failed at %s: %s", candidate, exc)
        LOGGER.error("Failed to write hand registers for base address %s", base_addr)
        return False

    def set_motor_targets(self, motor_targets: Sequence[int]) -> bool:
        """Set target angle for all 6 motors (0-1000; 65535 means no action)."""
        if len(motor_targets) != self.MOTOR_COUNT:
            LOGGER.error(
                "Expected %s motor targets, got %s",
                self.MOTOR_COUNT,
                len(motor_targets),
            )
            return False

        return self._write_registers_with_fallback(self.ANGLE_SET_BASE_ADDR, motor_targets)

    def move_single_motor(
        self,
        motor_index: int,
        target_value: int,
        hold_seconds: float = 0.25,
        no_action_value: int = NO_ACTION_VALUE,
    ) -> bool:
        if motor_index < 0 or motor_index >= self.MOTOR_COUNT:
            LOGGER.error("Motor index out of range: %s", motor_index)
            return False

        cmd = [int(no_action_value) & 0xFFFF] * self.MOTOR_COUNT
        cmd[motor_index] = int(target_value) & 0xFFFF
        ok = self.set_motor_targets(cmd)
        if not ok:
            return False
        time.sleep(max(0.0, hold_seconds))
        return True

    def cycle_all_motors(
        self,
        open_value: int = 120,
        close_value: int = 800,
        hold_seconds: float = 0.35,
        settle_seconds: float = 0.2,
        motor_order: Optional[Iterable[int]] = None,
    ) -> bool:
        """Move each hand motor open->close->open in sequence."""
        order = list(range(self.MOTOR_COUNT)) if motor_order is None else list(motor_order)
        all_ok = True
        for idx in order:
            LOGGER.info("Cycling hand motor %s", idx)
            all_ok &= self.move_single_motor(idx, open_value, hold_seconds=hold_seconds)
            all_ok &= self.move_single_motor(idx, close_value, hold_seconds=hold_seconds)
            all_ok &= self.move_single_motor(idx, open_value, hold_seconds=hold_seconds)
            time.sleep(max(0.0, settle_seconds))
        return all_ok

    def set_all_motors(self, target_value: int, hold_seconds: float = 0.35) -> bool:
        """Set all motor targets to a single value."""
        ok = self.set_motor_targets([int(target_value) & 0xFFFF] * self.MOTOR_COUNT)
        if ok:
            time.sleep(max(0.0, hold_seconds))
        return ok
