"""Klipper plugin to create eight virtual input pins.

This module checks for an [input_pins] section in the given configuration
file and, if present, loads VirtualMCU to provide ams:pin1-ams:pin8.

This is a simplified example intended to illustrate how one might hook a
custom module into Klipper's configuration system. It does not implement
the full Klipper MCU protocol.
"""

import configparser
from typing import Optional

from virtual_mcu import VirtualMCU


class InputPins:
    """Manage a VirtualMCU when [input_pins] is present."""

    def __init__(self) -> None:
        self.mcu: Optional[VirtualMCU] = None

    def load_from_file(self, cfg_path: str) -> None:
        """Load configuration and initialize VirtualMCU if needed."""
        parser = configparser.ConfigParser()
        parser.read(cfg_path)
        if "input_pins" in parser:
            self.mcu = VirtualMCU()
            print(
                "InputPins: Virtual MCU loaded with pins:",
                ", ".join(self.mcu.list_pins()),
            )
        else:
            print("InputPins: [input_pins] not defined; module not loaded")


def load_config(cfg_path: str) -> InputPins:
    """Convenience function mirroring Klipper's load_config interface."""
    ip = InputPins()
    ip.load_from_file(cfg_path)
    return ip
