# Global virtual input pins for Klipper
#
# This module provides eight software-based input pins accessible via the
# ``ams_pin:`` prefix. Pins are named ``pin1`` through ``pin8`` and may be
# used anywhere an endstop style input is expected. The pins are created on
# demand when the prefix is first used, so a bare ``[ams_pin]`` section is
# optional.  G-code commands allow software to update or query the stored
# state.

import logging


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

    def home_start(
        self, print_time, sample_time, sample_count, rest_time, triggered=True
    ):
        comp = self._reactor.completion()
        comp.complete(self.query_endstop(print_time))
        return comp

    def home_wait(self, home_end_time):
        if self.query_endstop(home_end_time):
            return home_end_time
        return 0.0

    def query_endstop(self, print_time):
        return bool(self._vpin.state) ^ bool(self._invert)


class VirtualPin:
    """Store state and callbacks for a single virtual pin."""

    def __init__(self, printer, name):
        self.printer = printer
        self.name = name
        self.state = False
        self._watchers = set()

    def register_watcher(self, callback):
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

    def get_status(self, eventtime):
        return {"value": int(self.state)}

    # --------------------------------------------------------------
    # Minimal MCU interface required by modules such as buttons.py
    # --------------------------------------------------------------
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
        pass


class VirtualPins:
    """Manage eight global virtual pins."""

    PIN_NAMES = [f"pin{i}" for i in range(1, 9)]

    def __init__(self, printer):
        self.printer = printer
        self.pins = {}

        for name in self.PIN_NAMES:
            pin = VirtualPin(self.printer, name)
            self.pins[name] = pin
            try:
                self.printer.add_object("ams_pin " + name, pin)
            except Exception:
                # FakePrinter in tests stores objects in a dict
                self.printer.objects["ams_pin " + name] = pin

        ppins = self.printer.lookup_object("pins")
        try:
            ppins.register_chip("ams_pin", self)
        except Exception:
            # Chip may already be registered
            pass

        gcode = self.printer.lookup_object("gcode")
        for name in self.PIN_NAMES:
            gcode.register_mux_command(
                "SET_AMS_PIN",
                "PIN",
                name,
                self._set_factory(name),
                desc=self.cmd_SET_AMS_PIN_help,
            )
            gcode.register_mux_command(
                "QUERY_AMS_PIN",
                "PIN",
                name,
                self._query_factory(name),
                desc=self.cmd_QUERY_AMS_PIN_help,
            )

    def _set_factory(self, name):
        def handler(gcmd, pin=name):
            val = gcmd.get_int("VALUE", 1)
            self.pins[pin].set_value(val)

        return handler

    def _query_factory(self, name):
        def handler(gcmd, pin=name):
            state = self.pins[pin].state
            gcmd.respond_info("ams_pin %s: %d" % (pin, state))

        return handler

    def setup_pin(self, pin_type, pin_params):
        ppins = self.printer.lookup_object("pins")
        if pin_type != "endstop":
            raise ppins.error("ams_pin pins only support endstop type")
        pin_name = pin_params["pin"]
        pin = self.pins.get(pin_name)
        if pin is None:
            raise ppins.error("ams_pin %s not configured" % (pin_name,))
        return VirtualEndstop(pin, pin_params["invert"])

    cmd_SET_AMS_PIN_help = "Set the value of a virtual input pin"
    cmd_QUERY_AMS_PIN_help = "Report the value of a virtual input pin"


def get_virtual_pins(printer):
    """Create or return the global VirtualPins object."""
    chip = printer.lookup_object("ams_pin_chip", None)
    if chip is not None:
        return chip
    chip = VirtualPins(printer)
    try:
        printer.add_object("ams_pin_chip", chip)
    except Exception:
        printer.objects["ams_pin_chip"] = chip
    return chip


def _load_config_impl(config):
    """Internal helper that returns the global pin manager."""
    # Access the name so the section is considered valid even though it has
    # no options.
    config.get_name()
    return get_virtual_pins(config.get_printer())


def load_config(config):
    """Entry point for a bare `[ams_pin]` section."""
    return _load_config_impl(config)


def load_config_prefix(config):
    """Prefix handler allowing `[ams_pin pinX]` sections."""
    name = config.get_name().split()[-1]
    if name == "ams_pin":
        return _load_config_impl(config)
    chip = get_virtual_pins(config.get_printer())
    pin = chip.pins.get(name)
    if pin is None:
        raise config.error("Unknown ams pin %s" % name)
    return pin


# Only expose the entry point to Klipper
__all__ = ["load_config", "load_config_prefix"]
