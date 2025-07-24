"""Minimal virtual MCU used for Klipper testing.

This module provides a very small mock MCU that exposes eight pins.  It is
not a full implementation of Klipper's MCU protocol, but it implements enough
of the pin interface for Klipper to treat the pins as valid inputs.
"""

