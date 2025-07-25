'''Klipper plugin providing eight virtual input pins.

When the configuration file contains an ``[input_pins]`` section, this
module creates a small mock MCU named ``ams`` that exposes the pins
``ams:pin1`` through ``ams:pin8``.  They can be used anywhere a regular
input pin is expected (for example ``filament_switch_sensor`` or
``endstop`` sections).  Only a tiny portion of Klipper's MCU protocol is
implemented here, just enough for basic input pin handling.

This file contains both the pin definitions and the configuration loader
so only one file needs to be copied into Klipper's ``klippy/extras``
directory.  No additional imports are required.
'''

import configparser
from dataclasses import dataclass
from typing import Optional
import importlib


@dataclass
class VirtualPin:
    """A simple digital pin with configurable mode and state."""

    name: str
    mode: str = "input"
    state: int = 0

    def configure(self, mode: str) -> None:
        self.mode = mode

    def read(self) -> int:
        return self.state

    def set_state(self, value: bool) -> None:
        self.state = 1 if value else 0


class VirtualMCU:
    """A minimal MCU emulator exposing 8 input pins."""

    def __init__(self, prefix: str = "ams"):
        self.prefix = prefix
        self.pins = {
            f"pin{i}": VirtualPin(f"{prefix}:pin{i}") for i in range(1, 9)
        }
        self._registered = False

    def read_pin(self, pin: str) -> int:
        return self.pins[pin].read()

    def set_pin(self, pin: str, value: bool) -> None:
        self.pins[pin].set_state(value)

    def configure_pin(self, pin: str, mode: str) -> None:
        self.pins[pin].configure(mode)

    def list_pins(self) -> list[str]:
        """Return pin names including the prefix."""
        return [pin.name for pin in self.pins.values()]

    def register_chip(self) -> None:
        """Register this MCU with Klipper's pin subsystem when available."""
        if self._registered:
            return
        try:
            pins_mod = importlib.import_module("pins")
        except Exception:
            return
        register_fn = getattr(pins_mod, "register_chip", None)
        if register_fn is None:
            return
        register_fn(self.prefix, lambda config=None: VirtualPinChip(self))
        self._registered = True


class VirtualPinRef:
    """Adapter used when Klipper requests a pin."""

    def __init__(self, vpin: VirtualPin) -> None:
        self.vpin = vpin

    def setup_input(self, pull_up: bool = False, invert: bool = False) -> None:
        self.vpin.configure("input")

    def setup_output(self, value: int) -> None:
        self.vpin.configure("output")
        self.vpin.set_state(bool(value))

    def read(self) -> int:
        return self.vpin.read()

    def get_mcu(self):  # pragma: no cover - required by Klipper API
        return None


class VirtualPinChip:
    """Pin chip registering pins under a prefix."""

    def __init__(self, mcu: VirtualMCU) -> None:
        self._mcu = mcu

    def setup_pin(self, pin: str) -> VirtualPinRef:
        try:
            vpin = self._mcu.pins[pin]
        except KeyError as exc:
            raise ValueError(f"Invalid virtual pin '{pin}'") from exc
        return VirtualPinRef(vpin)


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
