"""Virtual input pins for Klipper."""

import logging

CHIP_NAME = "ams"


def _norm(name):
    return str(name).strip().lower()


class VirtualPinChip:
    """Registry for virtual input pins."""

    def __init__(self, printer):
        self.printer = printer
        self.pins = {}
        self._next_oid = 0
