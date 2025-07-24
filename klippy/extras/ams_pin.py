"""Virtual input pins for Klipper."""

import logging

CHIP_NAME = "ams"


def _norm(name):
    return str(name).strip().lower()


class VirtualPinChip:
    """Registry for virtual input pins."""

    def __init__(self, printer):
        self.printer = printer
        self.pins = {}
        self._next_oid = 0
        gcode = printer.lookup_object('gcode')
        gcode.register_command('SET_AMS_PIN', self.cmd_SET_AMS_PIN,
                               desc='Set the value of a virtual input pin')
        gcode.register_command('QUERY_AMS_PIN', self.cmd_QUERY_AMS_PIN,
                               desc='Report the value of a virtual input pin')

    def register_pin(self, vpin):
        key = _norm(vpin.name)
        if key in self.pins:
            logging.warning("Duplicate AMS pin %s ignored", vpin.name)
            return
        self.pins[key] = vpin

    def setup_pin(self, pin_type, pin_params):
        ppins = self.printer.lookup_object('pins')
        if pin_type != 'endstop':
            raise ppins.error('ams pins only support endstop type')
        name = _norm(pin_params['pin'])
        vpin = self.pins.get(name)
        if vpin is None:
            raise ppins.error(f'ams pin {name} not configured')
        return VirtualEndstop(vpin, pin_params['invert'])

    def create_oid(self):
        oid = self._next_oid
        self._next_oid += 1
        return oid

    # gcode helpers -----------------------------------------------------
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


def _ensure_chip(printer):
    ppins = printer.lookup_object('pins')
    chip = ppins.chips.get(CHIP_NAME)
    if chip is None:
        chip = VirtualPinChip(printer)
        ppins.register_chip(CHIP_NAME, chip)
    return chip


class VirtualEndstop:
    def __init__(self, vpin, invert):
        self._vpin = vpin
        self._invert = invert
        self._reactor = vpin.printer.get_reactor()

    def get_mcu(self):
        return self._vpin

    def add_stepper(self, stepper):
        pass

    def get_steppers(self):
        return []

    def home_start(self, print_time, sample_time, sample_count, rest_time, triggered=True):
        comp = self._reactor.completion()
        comp.complete(self.query_endstop(print_time))
        return comp

    def home_wait(self, home_end_time):
        if self.query_endstop(home_end_time):
            return home_end_time
        return 0.

    def query_endstop(self, print_time):
        return bool(self._vpin.state) ^ bool(self._invert)


class VirtualInputPin:
    """Single software-defined input pin."""

    def __init__(self, printer, name, initial_value=False):
        self.printer = printer
        self.name = name
        self.state = bool(initial_value)
        self._watchers = []
        self.oid = _ensure_chip(printer).create_oid()
        _ensure_chip(printer).register_pin(self)
        # expose as printer object so lookup works
        printer.add_object(CHIP_NAME + ' ' + self.name, self)

    def setup_pin(self, pin_type, pin_params):
        return VirtualEndstop(self, pin_params['invert'])

    def register_watcher(self, cb):
        self._watchers.append(cb)
        try:
            cb(self.printer.get_reactor().monotonic(), self.state)
        except Exception:
            logging.exception('virtual pin callback error')

    def set_value(self, val):
        val = bool(val)
        if self.state == val:
            return
        self.state = val
        et = self.printer.get_reactor().monotonic()
        for cb in list(self._watchers):
            try:
                cb(et, val)
            except Exception:
                logging.exception('virtual pin callback error')


class SimpleFilamentSensor:
    """Minimal filament sensor bound to a virtual pin."""

    def __init__(self, printer, name, vpin):
        self.printer = printer
        self.name = name
        self.vpin = vpin
        self.filament_present = bool(vpin.state)
        self.vpin.register_watcher(self._pin_changed)
        printer.add_object('filament_switch_sensor ' + name, self)

    def _pin_changed(self, eventtime, state):
        self.filament_present = bool(state)

    def get_status(self, eventtime):
        return {'filament_detected': bool(self.filament_present)}


def load_config_prefix(config):
    printer = config.get_printer()
    name = config.get_name().split()[-1]
    initial_value = config.getboolean('initial_value', False)
    return VirtualInputPin(printer, name, initial_value)


class AmsPins:
    def __init__(self, config):
        printer = config.get_printer()
        self.pins = []
        self.sensors = []
        for i in range(1, 9):
            name = f'pin{i}'
            vpin = VirtualInputPin(printer, name, False)
            self.pins.append(vpin)
            self.sensors.append(SimpleFilamentSensor(printer, name, vpin))


def load_config(config):
    return AmsPins(config)
