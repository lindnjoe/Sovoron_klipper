"""Initialization helpers for Klipper modules."""

import sys

# Ensure the local pins module is available under the short name 'pins'.  Some
# scripts import ``pins`` before adding ``klippy`` to ``sys.path`` which can
# resolve to an unrelated package.  Register the proper module so references to
# ``pins.error`` and other attributes succeed regardless of import order.
from . import pins as _pins
sys.modules.setdefault("pins", _pins)

import klippy.extras.afc_sensor
