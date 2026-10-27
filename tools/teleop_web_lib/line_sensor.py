"""
Bottom line sensor (TCRT5000) for the browser tool.
"""

import logging
from pathlib import Path

log = logging.getLogger("teleop_web")

# DO value over the black tape: the usual module gives 1 on black, 0 on white.
DEFAULT_BLACK_VALUE = 1


class LineSensor:
    """
    Bottom line sensor (TCRT5000, digital DO output) read with gpiozero.

    Power it from 3.3 V: at 5 V the DO output puts 5 V on the Pi's GPIO.
    """

    def __init__(self, cfg: dict):
        """
        Open the GPIO pin of the sensor.
        """
        # Only available on the Pi.
        from gpiozero import DigitalInputDevice

        self._dev = DigitalInputDevice(cfg["pin"], pull_up=False)
        self._black_value = int(cfg.get("black_value", DEFAULT_BLACK_VALUE))

    def is_black(self) -> bool:
        """
        True when the sensor is over the black line.
        """
        return self._dev.value == self._black_value

    def raw_value(self) -> int:
        """
        Raw digital output (0 or 1) from the DO pin.
        """
        return int(self._dev.value)

    def close(self) -> None:
        """
        Release the GPIO pin.
        """
        self._dev.close()


def build_line_sensor(
    cfg: dict | None,
    fake: bool,
    config_path: Path,
) -> LineSensor | None:
    """
    The line sensor, or None when running fake or when no pin is set.
    """
    if fake:
        return None
    cfg = cfg or {}
    if cfg.get("pin") is None:
        log.warning(
            "line_sensor.pin is empty in %s: black line not detected",
            config_path,
        )
        return None
    return LineSensor(cfg)
