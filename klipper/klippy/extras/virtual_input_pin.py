"""Virtual input pin module for Klipper.

Provides a software-based input pin that can be used anywhere a normal
endstop pin would be referenced. The pin state may be changed at runtime
via gcode commands.
"""

import logging
from . import filament_switch_sensor as fil_sensor
import pins


class VirtualEndstop:
    """Simple endstop object backed by a virtual pin."""

    def __init__(self, vpin, invert):
        self._vpin = vpin
        self._invert = invert
        self._reactor = vpin.printer.get_reactor()

    def get_mcu(self):
        return None

    def add_stepper(self, stepper):
        pass

    def get_steppers(self):
        return []

    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        comp = self._reactor.completion()
        comp.complete(self.query_endstop(print_time))
        return comp

    def home_wait(self, home_end_time):
        if self.query_endstop(home_end_time):
            return home_end_time
        return 0.0

    def query_endstop(self, print_time):
        return bool(self._vpin.state) ^ bool(self._invert)


class VirtualPinChip:
    """Chip manager emulating MCU behavior for virtual pins."""

    def __init__(self, printer):
        self.printer = printer
        self.pins = {}
        # store (handler, [vpin objects]) for buttons_state callbacks
        self._response_handlers = []
        self._ack_count = 0
        self._config_callbacks = []
        self.printer.register_event_handler('klippy:ready',
                                            self._run_config_callbacks)

    def add_pin(self, vpin):
        if vpin.name in self.pins:
            raise pins.error(f"Duplicate virtual_pin {vpin.name}")
        index = len(self.pins)
        self.pins[vpin.name] = (vpin, index)
        if self._response_handlers:
            for handler, vpins in self._response_handlers:
                if vpin in vpins:
                    vpin.register_watcher(
                        lambda val, h=handler, p=vpins: self._pin_changed(h, p))
                    self._pin_changed(handler, vpins)

    def _pin_changed(self, handler, vpins):
        state = 0
        for i, vpin in enumerate(vpins):
            if vpin.state:
                state |= 1 << i
        params = {
            'ack_count': self._ack_count & 0xff,
            'state': bytes([state]),
            '#receive_time': self.printer.get_reactor().monotonic(),
        }
        self._ack_count += 1
        try:
            handler(params)
        except Exception:
            logging.exception('Virtual pin chip handler error')

    def setup_pin(self, pin_type, pin_params):
        pin_name = pin_params['pin']
        entry = self.pins.get(pin_name)
        if entry is None:
            raise pins.error('virtual_pin %s not configured' % (pin_name,))
        vpin, idx = entry
        return vpin._setup_pin(pin_type, pin_params)

    # --------------------------------------------------------------
    # Minimal MCU interface used by modules like buttons.py
    # --------------------------------------------------------------
    def register_config_callback(self, cb):
        self._config_callbacks.append(cb)

    def _run_config_callbacks(self, eventtime=None):
        for cb in self._config_callbacks:
            try:
                cb()
            except Exception:
                logging.exception('Virtual pin chip config callback error')
        self._config_callbacks = []

    def create_oid(self):
        return 0

    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
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
        if resp_name != 'buttons_state':
            return
        # obtain pin order from MCU_buttons object if available
        pin_list = getattr(getattr(handler, '__self__', None), 'pin_list', [])
        if not pin_list:
            pin_list = getattr(handler, 'pin_list', [])
        vpins = []
        for pin_name, _pullup in pin_list:
            entry = self.pins.get(pin_name)
            if entry is None:
                logging.error('virtual pin %s not configured', pin_name)
                continue
            vpins.append(entry[0])
        if not vpins:
            return
        self._response_handlers.append((handler, vpins))
        for vpin in vpins:
            vpin.register_watcher(
                lambda val, h=handler, p=vpins: self._pin_changed(h, p))
        self._pin_changed(handler, vpins)


class VirtualInputPin:
    """Manage a single virtual input pin."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.state = config.getboolean('initial_value', False)
        self._watchers = set()
        # track button handlers for compatibility with modules that expect
        # MCU-style callbacks (eg, buttons.py)
        self._button_handlers = []
        self._ack_count = 0
        self._config_callbacks = []

        # run deferred config callbacks after Klipper is ready
        self.printer.register_event_handler('klippy:ready',
                                            self._run_config_callbacks)

        ppins = self.printer.lookup_object('pins')
        chip = self.printer.lookup_object('virtual_pin_chip', None)
        if chip is None:
            chip = VirtualPinChip(self.printer)
            add_obj = getattr(self.printer, 'add_object', None)
            if add_obj is not None:
                add_obj('virtual_pin_chip', chip)
            else:
                self.printer.objects['virtual_pin_chip'] = chip
            try:
                ppins.register_chip('virtual_pin', chip)
            except pins.error:
                pass
        chip.add_pin(self)

        gcode = self.printer.lookup_object('gcode')
        cname = self.name
        gcode.register_mux_command('SET_VIRTUAL_PIN', 'PIN', cname,
                                   self.cmd_SET_VIRTUAL_PIN,
                                   desc=self.cmd_SET_VIRTUAL_PIN_help)
        gcode.register_mux_command('QUERY_VIRTUAL_PIN', 'PIN', cname,
                                   self.cmd_QUERY_VIRTUAL_PIN,
                                   desc=self.cmd_QUERY_VIRTUAL_PIN_help)

    # ------------------------------------------------------------------
    # public helper methods
    # ------------------------------------------------------------------

    def _setup_pin(self, pin_type, pin_params):
        ppins = self.printer.lookup_object('pins')
        if pin_type != 'endstop':
            raise ppins.error('virtual_pin pins only support endstop type')
        return VirtualEndstop(self, pin_params['invert'])

    def register_watcher(self, callback):
        """Register a callback for state changes and invoke with current state."""
        self._watchers.add(callback)
        try:
            callback(self.state)
        except Exception:
            logging.exception('Virtual pin callback error')

    def set_value(self, val):
        val = bool(val)
        if self.state == val:
            return
        self.state = val
        for cb in list(self._watchers):
            try:
                cb(val)
            except Exception:
                logging.exception('Virtual pin callback error')
        if self._button_handlers:
            params = {
                'ack_count': self._ack_count & 0xff,
                'state': bytes([int(val)]),
                '#receive_time': self.printer.get_reactor().monotonic(),
            }
            self._ack_count += 1
            for handler in list(self._button_handlers):
                try:
                    handler(params)
                except Exception:
                    logging.exception('Virtual button handler error')

    def get_status(self, eventtime):
        return {'value': int(self.state)}

    # --------------------------------------------------------------
    # Minimal MCU interface for compatibility with modules that expect
    # MCU objects, such as buttons.py
    # --------------------------------------------------------------
    def register_config_callback(self, cb):
        """Store configuration callbacks to run when Klipper is ready."""
        self._config_callbacks.append(cb)

    def _run_config_callbacks(self, eventtime=None):
        for cb in self._config_callbacks:
            try:
                cb()
            except Exception:
                logging.exception('Virtual pin config callback error')
        self._config_callbacks = []

    def create_oid(self):
        self._ack_count = 0
        return 0

    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
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
            self._button_handlers.append(handler)
            params = {
                'ack_count': self._ack_count & 0xff,
                'state': bytes([int(self.state)]),
                '#receive_time': self.printer.get_reactor().monotonic(),
            }
            self._ack_count += 1
            try:
                handler(params)
            except Exception:
                logging.exception('Virtual button handler error')

    cmd_SET_VIRTUAL_PIN_help = 'Set the value of a virtual input pin'

    def cmd_SET_VIRTUAL_PIN(self, gcmd):
        val = gcmd.get_int('VALUE', 1)
        self.set_value(val)

    cmd_QUERY_VIRTUAL_PIN_help = 'Report the value of a virtual input pin'

    def cmd_QUERY_VIRTUAL_PIN(self, gcmd):
        gcmd.respond_info('virtual_pin %s: %d' % (self.name, self.state))


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


def load_config_prefix(config):
    """Config handler for [virtual_input_pin] sections."""
    prefix = config.get_name().split()[0]
    if prefix != 'virtual_input_pin':
        raise config.error('Unknown prefix %s' % prefix)
    return VirtualInputPin(config)


def add_printer_objects(config):
    """Register the virtual_pin chip before other modules parse pins."""
    printer = config.get_printer()
    if printer.lookup_object('virtual_pin_chip', None) is not None:
        return
    chip = VirtualPinChip(printer)
    add_obj = getattr(printer, 'add_object', None)
    if add_obj is not None:
        add_obj('virtual_pin_chip', chip)
    else:
        printer.objects['virtual_pin_chip'] = chip
    try:
        printer.lookup_object('pins').register_chip('virtual_pin', chip)
    except pins.error:
        pass
