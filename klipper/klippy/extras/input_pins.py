"""Klipper plugin providing eight virtual input pins.

When the configuration file contains an ``[input_pins]`` section, this
module creates a small mock MCU named ``ams`` that exposes the pins
``ams:pin1`` through ``ams:pin8``.  They can be used anywhere a regular
input pin is expected (for example ``filament_switch_sensor`` or
``endstop`` sections).  Only a tiny portion of Klipper's MCU protocol is
implemented here, just enough for basic input pin handling.
"""

import configparser
from typing import Optional
import importlib

klipper_pins = None
for _name in ("klippy.pins", "pins"):
    try:
        klipper_pins = importlib.import_module(_name)
        break
    except Exception:
        pass

# Import VirtualMCU from the same package when running as a Klipper
# extras module.  Fallback to a direct import so the file can still be
# executed or compiled standalone during development.
try:
    from .virtual_mcu import VirtualMCU
except ImportError:  # pragma: no cover - fallback when not in package
    from virtual_mcu import VirtualMCU


# Shared MCU instance registered as soon as the module is imported
MCU_PREFIX = "ams"
MODULE_MCU = VirtualMCU(prefix=MCU_PREFIX)
MODULE_MCU.register_chip()



class InputPins:
    """Manage the global VirtualMCU when [input_pins] is present."""

    def __init__(self) -> None:
        self.mcu: Optional[VirtualMCU] = None

    def load_from_file(self, cfg_path: str) -> None:
        """Load configuration and bind to the global MCU if needed."""
        parser = configparser.ConfigParser()
        parser.read(cfg_path)
        if "input_pins" in parser:
            MODULE_MCU.register_chip()
            self.mcu = MODULE_MCU
            print(
                "InputPins: Virtual MCU loaded with pins:",
                ", ".join(self.mcu.list_pins()),
            )
        else:
            print("InputPins: [input_pins] not defined; module not loaded")


def load_config_file(cfg_path: str) -> InputPins:
    """Convenience wrapper for standalone testing."""
    ip = InputPins()
    ip.load_from_file(cfg_path)
    return ip


def load_config_prefix(config):  # pragma: no cover - used by Klipper
    """Nothing to do here as the chip is registered on import."""
    return config


def load_config(config):  # pragma: no cover - used by Klipper
    """Entry point used by Klipper when [input_pins] is present."""
    ip = InputPins()
    MODULE_MCU.register_chip()
    ip.mcu = MODULE_MCU
    config.get_printer().add_object('virtual_mcu', ip.mcu)
    return ip


# For backward compatibility with earlier revisions that expected
# ``load_config_prefix`` to create the object directly.
load_config_legacy = load_config
