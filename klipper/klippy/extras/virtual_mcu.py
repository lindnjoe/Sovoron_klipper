"""Minimal virtual MCU used for Klipper testing.

This module provides a very small mock MCU that exposes eight pins.  It is
not a full implementation of Klipper's MCU protocol, but it implements enough
of the pin interface for Klipper to treat the pins as valid inputs.
"""

from dataclasses import dataclass

try:  # Klipper provides a ``pins`` module when running as an extra
    from .. import pins as klipper_pins  # type: ignore
except Exception:  # pragma: no cover - running standalone
    klipper_pins = None


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
        """Register this MCU with Klipper's pin subsystem if available."""
        if klipper_pins is not None:
            klipper_pins.register_chip(self.prefix, lambda config=None: VirtualPinChip(self))


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


if __name__ == "__main__":
    mcu = VirtualMCU()
    mcu.register_chip()
    print("Virtual MCU initialized with pins:", ", ".join(mcu.list_pins()))
    print(
        "Use mcu.set_pin(name, value) to change pin state "
        "and mcu.read_pin(name) to read it."
    )

