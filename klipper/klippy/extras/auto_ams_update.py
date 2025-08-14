import logging
from . import virtual_input_pin

SYNC_INTERVAL = 2


class _VPinConfig:
    """Minimal config wrapper used to create virtual_input_pin objects."""

    def __init__(self, printer, name):
        self._printer = printer
        self._name = f'virtual_input_pin {name}'

    def get_printer(self):
        return self._printer

    def get_name(self):
        return self._name

    def getboolean(self, key, default=False):
        return default

    def get(self, key, default=None):
        return default

    def error(self, msg):
        raise Exception(msg)


class AutoAMSUpdate:
    """Periodically update virtual pins from AMS lane status."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.interval = config.getfloat('interval', SYNC_INTERVAL, above=0.)
        # Find all oams#: options and sort by numeric suffix
        oams_opts = config.get_prefix_options('oams')
        if oams_opts:
            def sort_key(opt):
                suffix = opt[4:]
                return int(suffix) if suffix.isdigit() else opt
            oams_opts = sorted(oams_opts, key=sort_key)
            self.oams_names = [config.get(opt).strip() for opt in oams_opts]
        else:
            oams_opts = []
            self.oams_names = ['oams1']
        # Expect eight pins (4 lane + 4 hub) per AMS unit
        expected = 8 * len(self.oams_names)
        pins_opt = config.get('pins', None)
        if pins_opt is not None:
            self.pin_names = config.getlist('pins')
            if len(self.pin_names) != expected:
                raise config.error(
                    f'pins option must contain {expected} pin names')
        else:
            lane_names = []
            hub_names = []
            src_opts = oams_opts or [f'oams{i+1}' for i in range(len(self.oams_names))]
            for idx, opt in enumerate(src_opts, start=1):
                suffix = opt[4:]
                if not suffix.isdigit():
                    suffix = str(idx)
                lane_names.extend(
                    f'ams{suffix}lane{i}pl' for i in range(4))
                hub_names.extend(
                    f'ams{suffix}hub{i}' for i in range(4))
            self.pin_names = lane_names + hub_names
        # automatically create virtual pins for any missing entries
        virtual_input_pin.add_printer_objects(config)
        chip = self.printer.lookup_object('virtual_pin_chip')
        for name in self.pin_names:
            if name in chip.pins:
                continue
            vcfg = _VPinConfig(self.printer, name)
            virtual_input_pin.VirtualInputPin(vcfg)
        self.timer = self.reactor.register_timer(self._sync_event)
        self.printer.register_event_handler('klippy:ready', self.handle_ready)

    def handle_ready(self, eventtime=None):
        self.reactor.update_timer(self.timer, self.reactor.NOW)

    def _sync_event(self, eventtime):
        try:
            # Lookup all configured AMS objects
            oams_objs = [
                self.printer.lookup_object('oams ' + name, None)
                for name in self.oams_names
            ]

            def update_pin(name, value):
                cmdline = f"SET_VIRTUAL_PIN PIN={name} VALUE={int(value)}"
                self.gcode.run_script_from_command(cmdline)

            num = len(oams_objs)
            lane_offset = 0
            hub_offset = 4 * num
            for idx, oams in enumerate(oams_objs):
                vals = getattr(oams, 'f1s_hes_value', [0, 0, 0, 0]) if oams else [0, 0, 0, 0]
                hubs = getattr(oams, 'hub_hes_value', [0, 0, 0, 0]) if oams else [0, 0, 0, 0]
                for i in range(4):
                    update_pin(self.pin_names[lane_offset + idx * 4 + i], vals[i])
                for i in range(4):
                    update_pin(self.pin_names[hub_offset + idx * 4 + i], hubs[i])
        except Exception:
            logging.exception('auto AMS update error')
        return eventtime + self.interval


def load_config(config):
    return AutoAMSUpdate(config)