"""Klipper plugin providing eight virtual input pins.

When the configuration file contains an ``[input_pins]`` section, this
module creates a small mock MCU named ``ams`` that exposes the pins
``ams:pin1`` through ``ams:pin8``.  They can be used anywhere a regular
input pin is expected (for example ``filament_switch_sensor`` or
``endstop`` sections).  Only a tiny portion of Klipper's MCU protocol is
implemented here, just enough for basic input pin handling.
