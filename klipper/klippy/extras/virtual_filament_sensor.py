"""Emulated filament sensor triggered by a virtual pin."""

import logging
from . import filament_switch_sensor as fil_sensor


class VirtualFilamentSensor:
    """Emulated filament sensor triggered by a virtual pin."""

    def __init__(self, config):
        self.printer = config.get_printer()
        pin = config.get('pin').strip()
        if pin.startswith('virtual_pin:'):
            self.vpin_name = pin.split('virtual_pin:', 1)[1].strip()
        else:
            self.vpin_name = pin
        self.vpin = None
        self.reactor = self.printer.get_reactor()
        self.runout_helper = fil_sensor.RunoutHelper(config)
        self.printer.register_event_handler('klippy:ready', self._bind_pin)
        self.get_status = self.runout_helper.get_status

    def _bind_pin(self, eventtime=None):
        if self.vpin is not None:
            return
        vpin = self.printer.lookup_object('virtual_pin ' + self.vpin_name, None)
        if vpin is None:
            logging.error('virtual pin %s not configured', self.vpin_name)
            return
        self.vpin = vpin
        self.vpin.register_watcher(self._pin_changed)
        self.runout_helper.note_filament_present(
            self.reactor.monotonic(), bool(self.vpin.state))

    def _pin_changed(self, val):
        self.runout_helper.note_filament_present(
            self.reactor.monotonic(), bool(val))


def load_config(config):
    return VirtualFilamentSensor(config)
