# Helper module to mirror OAMS feeder status to virtual pins
#
# For each `[oams <name>]` section found in the given configuration file,
# eight virtual input pins are created: `<name>_lane0_prep`,
# `<name>_lane0_load`, ... `<name>_lane3_prep`, `<name>_lane3_load`.
# These pins reflect the real-time value of
# `printer['oams <name>'].f1s_hes_value` and can be referenced
# elsewhere as `ams_pin:<name>_lane#_prep` or `ams_pin:<name>_lane#_load`.
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import os, configparser
from .ams_pin import VirtualInputPin

class _FakeConfig:
    def __init__(self, printer, name, initial=False):
        self._printer = printer
        self._name = 'ams_pin ' + name
        self._initial = initial
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def getboolean(self, option, default=False):
        if option == 'initial_value':
            return self._initial
        return default
    def error(self, msg):
        raise configparser.Error(msg)

class OAMSVirtualPins:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        cfgpath = config.get('oams_config', 'oamsc.cfg')
        if not os.path.isabs(cfgpath):
            base = None
            try:
                start_args = self.printer.get_start_args()
                if isinstance(start_args, dict):
                    base = os.path.dirname(start_args.get('config_file', ''))
                else:
                    base = os.path.dirname(getattr(start_args, 'config_file', ''))
            except Exception:
                base = None
            if not base:
                base = '.'
            cfgpath = os.path.join(base, cfgpath)
        cp = configparser.ConfigParser(interpolation=None)
        try:
            cp.read(cfgpath)
        except Exception:
            cp.read_dict({})
        self.vpins = {}
        for sec in cp.sections():
            if not sec.startswith('oams '):
                continue
            name = sec.split(None, 1)[1]
            for i in range(4):
                for suffix in ('_prep', '_load'):
                    pin_name = f'{name}_lane{i}{suffix}'
                    fc = _FakeConfig(self.printer, pin_name)
                    vpin = VirtualInputPin(fc)
                    # expose the pin like a normal config object so other
                    # modules can look it up via printer.lookup_object()
                    try:
                        self.printer.objects[f'ams_pin {pin_name}'] = vpin
                    except Exception:
                        pass
                    self.vpins[(name, i, suffix)] = vpin
        self.poll = config.getfloat('poll_interval', 1.0, above=0.05)
        self.printer.register_event_handler('klippy:ready', self._on_ready)

    def _on_ready(self, eventtime=None):
        self.reactor.register_timer(self._poll_timer)

    def _poll_timer(self, eventtime):
        for (name, idx, suffix), vpin in self.vpins.items():
            obj = self.printer.lookup_object(f'oams {name}', None)
            if obj is None:
                continue
            vals = getattr(obj, 'f1s_hes_value', None)
            if not isinstance(vals, (list, tuple)):
                continue
            if idx < len(vals):
                vpin.set_value(bool(vals[idx]))
        return eventtime + self.poll

def load_config(config):
    """Config handler for a bare `[oams_virtual_pins]` section."""
    return OAMSVirtualPins(config)


def load_config_prefix(config):
    """Config handler for `[oams_virtual_pins <name>]` sections."""
    return OAMSVirtualPins(config)
