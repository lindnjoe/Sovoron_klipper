"""Software-defined input pins for Klipper.

This module implements the ``ams_pin`` chip which creates virtual input
pins that behave like endstop-style pins.  Other subsystems can watch
the pin for state changes just as they would a physical MCU pin.  Each
``[ams_pin]`` section defines one pin, accessible elsewhere in the
configuration via ``ams_pin:<name>``.

The pin value may be changed or queried at runtime using the
``SET_AMS_PIN`` and ``QUERY_AMS_PIN`` gcode commands.

This file previously contained merge conflict markers that caused a
syntax error.  Those markers have been removed so the module can load
properly.
"""

import logging

# ---------------------------------------------------------------------------
# Chip management helpers
# ---------------------------------------------------------------------------

CHIP_NAME = "ams_pin"

def _norm(name):
    """Normalize a pin name for consistent lookups."""
    return str(name).strip().lower()


class AmsPinChip:
    """Registry for all virtual pins attached to the ``ams_pin`` chip."""

    def __init__(self, printer):
        self.printer = printer
        self.pins = {}
        self._config_callbacks = []
        self._button_handlers = []
        self.printer.register_event_handler('klippy:ready',
                                            self._run_config_callbacks)
        self._gcode_registered = False

    def register_pin(self, vpin):
        self._register_gcode()
        # Store the pin only under its normalized name so lookups are
        # unambiguous.  Previous revisions attempted to store multiple
        # aliases here which caused later pins to overwrite earlier ones.
        key = _norm(getattr(vpin, 'raw_name', vpin.name))
        if key in self.pins:
            logging.warning('Duplicate ams_pin %s ignored', key)
            return
        self.pins[key] = vpin
        for handler in self._button_handlers:
            vpin.register_response(handler, 'buttons_state')

    # G-code command registration --------------------------------------------
    def _register_gcode(self):
        if self._gcode_registered:
            return
        self._gcode_registered = True
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('SET_AMS_PIN', self.cmd_SET_AMS_PIN,
                               desc='Set the value of an ams_pin')
        gcode.register_command('QUERY_AMS_PIN', self.cmd_QUERY_AMS_PIN,
                               desc='Report the value of an ams_pin')

    def cmd_SET_AMS_PIN(self, gcmd):
        pin_name = gcmd.get('PIN')
        if pin_name is None:
            raise gcmd.error('PIN parameter required')
        pin_name = str(pin_name).strip()
        if pin_name.lower().startswith(CHIP_NAME + ':'):
            pin_name = pin_name.split(':', 1)[1]
        pin_name = _norm(pin_name)
        val = gcmd.get_int('VALUE', 1)
        vpin = self.pins.get(pin_name)
        if vpin is None:
            raise gcmd.error(f'Unknown {CHIP_NAME} {pin_name}')
        vpin.set_value(val)

    def cmd_QUERY_AMS_PIN(self, gcmd):
        pin_name = gcmd.get('PIN')
        if pin_name is None:
            raise gcmd.error('PIN parameter required')
        pin_name = str(pin_name).strip()
        if pin_name.lower().startswith(CHIP_NAME + ':'):
            pin_name = pin_name.split(':', 1)[1]
        pin_name = _norm(pin_name)
        vpin = self.pins.get(pin_name)
        if vpin is None:
            raise gcmd.error(f'Unknown {CHIP_NAME} {pin_name}')
        gcmd.respond_info(f'{CHIP_NAME} {pin_name}: {int(vpin.state)}')

    def setup_pin(self, pin_type, pin_params):
        ppins = self.printer.lookup_object("pins")
        pin_name = _norm(pin_params["pin"])
        vpin = self.pins.get(pin_name)
        if vpin is None:
            raise ppins.error("%s %s not configured" % (CHIP_NAME, pin_name))
        # Present the virtual pin itself as the MCU object so other modules
        # can register callbacks directly on it.
        pin_params["chip"] = vpin
        pin_params["mcu"] = vpin
        return vpin.setup_pin(pin_type, pin_params)

    # Minimal MCU interface ----------------------------------------------------
    def register_config_callback(self, cb):
        self._config_callbacks.append(cb)

    def _run_config_callbacks(self, eventtime=None):
        for cb in self._config_callbacks:
            try:
                cb()
            except Exception:
                logging.exception('Virtual chip config callback error')
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
        if resp_name == 'buttons_state':
            self._button_handlers.append(handler)
            for pin in self.pins.values():
                pin.register_response(handler, resp_name, oid)


def _ensure_chip(printer):
    ppins = printer.lookup_object("pins")
    chip = ppins.chips.get(CHIP_NAME)
    if chip is None:
        chip = AmsPinChip(printer)
        ppins.register_chip(CHIP_NAME, chip)
        # Expose chip via `printer.ams_pins` for convenience
        try:
            setattr(printer, "ams_pins", chip)
        except Exception:
            pass
    return chip


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


class VirtualInputPin:
    """Manage a single virtual input pin."""

    def __init__(self, config):
        self.printer = config.get_printer()
        raw_name = config.get_name().split()[-1]
        self.raw_name = raw_name
        self.name = _norm(raw_name)
        self.state = config.getboolean("initial_value", False)
        self._watchers = set()
        self._button_handlers = []
        self._ack_count = 0
        self._config_callbacks = []

        # Defer config callbacks until Klipper is ready
        self.printer.register_event_handler(
            "klippy:ready", self._run_config_callbacks
        )

        chip = _ensure_chip(self.printer)
        chip.register_pin(self)
        # Expose under several keys so lookups succeed regardless of
        # capitalization or if the caller includes the chip prefix.
        for key in {f'{CHIP_NAME} {self.name}', f'{CHIP_NAME} {self.raw_name}',
                    f'{CHIP_NAME}:{self.raw_name}'}:
            try:
                self.printer.objects[key] = self
            except Exception:
                pass

    # Pins framework interface -------------------------------------------------
    def setup_pin(self, pin_type, pin_params):
        if pin_type != "endstop":
            ppins = self.printer.lookup_object("pins")
            raise ppins.error("%s pins only support endstop type" % CHIP_NAME)
        return VirtualEndstop(self, pin_params["invert"])

    # Watchers ----------------------------------------------------------------
    def register_watcher(self, callback):
        """Register a callback and immediately invoke it with the state."""

        self._watchers.add(callback)
        try:
            callback(self.state)
        except Exception:
            logging.exception("Virtual pin callback error")

    def set_value(self, val):
        val = bool(val)
        if self.state == val:
            return
        self.state = val
        for cb in list(self._watchers):
            try:
                cb(val)
            except Exception:
                logging.exception("Virtual pin callback error")
        if self._button_handlers:
            params = {
                "ack_count": self._ack_count & 0xFF,
                "state": bytes([int(val)]),
                "#receive_time": self.printer.get_reactor().monotonic(),
            }
            self._ack_count += 1
            for handler in list(self._button_handlers):
                try:
                    handler(params)
                except Exception:
                    logging.exception("Virtual button handler error")

    def get_status(self, eventtime):
        return {"value": int(self.state)}

    # Minimal MCU interface ----------------------------------------------------
    def register_config_callback(self, cb):
        self._config_callbacks.append(cb)

    def _run_config_callbacks(self, eventtime=None):
        for cb in self._config_callbacks:
            try:
                cb()
            except Exception:
                logging.exception("Virtual pin config callback error")
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
        if resp_name == "buttons_state":
            self._button_handlers.append(handler)
            params = {
                "ack_count": self._ack_count & 0xFF,
                "state": bytes([int(self.state)]),
                "#receive_time": self.printer.get_reactor().monotonic(),
            }
            self._ack_count += 1
            try:
                handler(params)
            except Exception:
                logging.exception("Virtual button handler error")



def load_config_prefix(config):
    """Config handler for [ams_pin] sections."""

    prefix = config.get_name().split()[0]
    if prefix != "ams_pin":
        raise config.error("Unknown prefix %s" % prefix)
    return VirtualInputPin(config)


def load_config(config):
    """Alias for load_config_prefix for compatibility."""

    return load_config_prefix(config)

