"""Software-defined input pins for Klipper.

This module implements the ``ams_pin`` chip which creates virtual input
pins that behave like endstop-style pins.  Other subsystems can watch
the pin for state changes just as they would a physical MCU pin.  Each
``[ams_pin]`` section defines one pin, accessible elsewhere in the
configuration via ``ams_pin:<name>``.

The pin value may be changed or queried at runtime using the
``SET_AMS_PIN`` and ``QUERY_AMS_PIN`` gcode commands.
"""

import logging

# ---------------------------------------------------------------------------
# Chip management helpers
# ---------------------------------------------------------------------------

CHIP_NAME = "ams_pin"


class AmsPinChip:
    """Registry for all virtual pins attached to the ``ams_pin`` chip."""

    def __init__(self, printer):
        self.printer = printer
        self.pins = {}

    def register_pin(self, vpin):
        self.pins[vpin.name] = vpin

    def setup_pin(self, pin_type, pin_params):
        ppins = self.printer.lookup_object("pins")
        pin_name = pin_params["pin"]
        vpin = self.pins.get(pin_name)
        if vpin is None:
            raise ppins.error("%s %s not configured" % (CHIP_NAME, pin_name))
<<<<