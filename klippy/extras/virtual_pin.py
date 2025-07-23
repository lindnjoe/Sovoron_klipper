"""Legacy wrapper for virtual pins and filament sensors.

This module re-exports the implementation from ``ams_pin.py`` and
``filament_switch_sensor.py`` so existing configurations referencing
``virtual_pin`` continue to work without modification.
"""

from .ams_pin import *
from .filament_switch_sensor import VirtualSwitchSensor as VirtualFilamentSensor
