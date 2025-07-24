"""Virtual input pin support for Klipper."""

__all__ = [
    "VirtualInputPin", "SimpleFilamentSensor", "load_config_prefix",
    "AmsPins", "load_config", "CHIP_NAME", "_norm",
]

import logging

CHIP_NAME = "ams"


def _norm(name):
    return str(name).strip().lower()


class VirtualPinChip:
    """Registry and minimal MCU interface for AMS pins."""

    def __init__(self, printer):
        self.printer = printer
        self.pins = {}
        self._next_oid = 0
        self._config_cbs = []
        printer.register_event_handler('klippy:ready', self._run_config_cbs)
        self._register_gcode()

    def _register_gcode(self):
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('SET_AMS_PIN', self.cmd_SET_AMS_PIN,
                               desc='Set the value of a virtual input pin')
        gcode.register_command('QUERY_AMS_PIN', self.cmd_QUERY_AMS_PIN,
                               desc='Report the value of a virtual input pin')

    def cmd_SET_AMS_PIN(self, gcmd):
        name = _norm(gcmd.get('PIN'))
        val = gcmd.get_int('VALUE', 1)
        vpin = self.pins.get(name)
        if vpin is None:
            raise gcmd.error(f'AMS pin {name} not configured')
        vpin.set_value(val)

    def cmd_QUERY_AMS_PIN(self, gcmd):
        name = _norm(gcmd.get('PIN'))
        vpin = self.pins.get(name)
        if vpin is None:
            raise gcmd.error(f'AMS pin {name} not configured')
        gcmd.respond_info(f'{name}: {int(vpin.state)}')

    def setup_pin(self, pin_type, pin_params):
        ppins = self.printer.lookup_object('pins')
        if pin_type != 'endstop':
            raise ppins.error('ams pins only support endstop type')
        name = _norm(pin_params['pin'])
        vpin = self.pins.get(name)
        if vpin is None:
            raise ppins.error(f'ams pin {name} not configured')
        pin_params['chip'] = vpin
        pin_params['mcu'] = vpin
        return vpin.setup_pin(pin_type, pin_params)

    def register_pin(self, vpin):
        key = _norm(vpin.name)
        if key in self.pins:
            logging.warning('Duplicate AMS pin %s ignored', vpin.name)
            return
        self.pins[key] = vpin

    # minimal MCU methods -------------------------------------------------
    def register_config_callback(self, cb):
        self._config_cbs.append(cb)

    def _run_config_cbs(self, eventtime=None):
        for cb in self._config_cbs:
            try:
                cb()
            except Exception:
                logging.exception('Virtual pin config callback error')
        self._config_cbs = []

    def create_oid(self):
        oid = self._next_oid
        self._next_oid += 1
        return oid

    def add_config_cmd(self, *args, **kwargs):
        pass

    class _DummyCmd:
        def send(self, params):
            pass

    def alloc_command_queue(self):
        return None

    def lookup_command(self, template, cq=None):
        return self._DummyCmd()

    def get_query_slot(self, oid):
        return 0

    def seconds_to_clock(self, time):
        return 0

    def register_response(self, handler, resp_name=None, oid=None):
        pass


def _ensure_chip(printer):
    ppins = printer.lookup_object('pins')
    chip = ppins.chips.get(CHIP_NAME)
    if chip is None:
        chip = VirtualPinChip(printer)
        ppins.register_chip(CHIP_NAME, chip)
        # Also register under "ams_pin" for legacy configs
        if 'ams_pin' not in ppins.chips:
            ppins.register_chip('ams_pin', chip)
