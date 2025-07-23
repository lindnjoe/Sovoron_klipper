"""Legacy wrapper for virtual input pins.

This module simply re-exports the implementation from ``ams_pin.py`` so
existing configurations referencing ``virtual_input_pin`` continue to
work.
"""

from .ams_pin import *
