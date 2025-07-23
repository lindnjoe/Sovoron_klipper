"""Virtual input pin support for Klipper."""

import logging

CHIP_NAME = "ams_pin"


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

    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.state = config.getboolean('initial_value', False)
        self._watchers = set()
        self._button_handlers = []
        self._ack = 0
        self._callbacks = []

        # Make pin discoverable like other config objects
        self.printer.add_object('ams_pin ' + self.name, self)
        self.printer.register_event_handler('klippy:ready', self._run_callbacks)
        chip = _ensure_chip(self.printer)
        chip.register_pin(self)

    # pins framework -----------------------------------------------------
    def setup_pin(self, pin_type, pin_params):
        return VirtualEndstop(self, pin_params['invert'])

    # watcher helpers ----------------------------------------------------
    def register_watcher(self, cb):
        # Determine the callback signature once and store it
        try:
            argc = cb.__code__.co_argcount
        except Exception:
            argc = 2
        if argc >= 2:
            mode = 2
        elif argc == 1:
            mode = 1
        else:
            mode = 0
        self._watchers.add((cb, mode))
        et = self.printer.get_reactor().monotonic()
        try:
            if mode == 2:
                cb(et, self.state)
            elif mode == 1:
                cb(self.state)
            else:
                cb(et)
        except Exception:
            logging.exception('Virtual pin callback error')

    def set_value(self, val):
        val = bool(val)
        if self.state == val:
            return
        self.state = val
        et = self.printer.get_reactor().monotonic()
        for cb, mode in list(self._watchers):
            try:
                if mode == 2:
                    cb(et, val)
                elif mode == 1:
                    cb(val)
                else:
                    cb(et)
            except Exception:
                logging.exception('Virtual pin callback error')
        for handler, oid in list(self._button_handlers):
            params = {'ack_count': self._ack & 0xFF,
                      'state': bytes([int(val)]),
                      '#receive_time': et}
            if oid is not None:
                params['oid'] = oid
            self._ack += 1
            try:
                handler(params)
            except Exception:
                logging.exception('Virtual button handler error')

    def get_status(self, eventtime):
        return {'value': int(self.state)}

    # minimal MCU style methods -----------------------------------------
    def register_config_callback(self, cb):
        self._callbacks.append(cb)

    def _run_callbacks(self, eventtime=None):
        for cb in self._callbacks:
            try:
                cb()
            except Exception:
                logging.exception('Virtual pin config callback error')
        self._callbacks = []

    def create_oid(self):
        return _ensure_chip(self.printer).create_oid()

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
        if resp_name == 'buttons_state':
            self._button_handlers.append((handler, oid))
            params = {'ack_count': self._ack & 0xFF,
                      'state': bytes([int(self.state)]),
                      '#receive_time': self.printer.get_reactor().monotonic()}
            if oid is not None:
                params['oid'] = oid
            self._ack += 1
            try:
                handler(params)
            except Exception:
                logging.exception('Virtual button handler error')


def load_config_prefix(config):
    """Entry point for `[ams_pin]` sections."""
    return VirtualInputPin(config)
