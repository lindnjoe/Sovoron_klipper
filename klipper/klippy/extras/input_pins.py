"""Klipper plugin to create eight virtual input pins.

This module checks for an [input_pins] section in the given configuration
file and, if present, loads VirtualMCU to provide ams:pin1-ams:pin8.

This is a simplified example intended to illustrate how one might hook a
custom module into Klipper's configuration system. It does not implement
the full Klipper MCU protocol.
"""

import configparser
