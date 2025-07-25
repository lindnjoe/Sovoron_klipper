import pytest

import ams_pin


class FakePins:
    class error(Exception):
        pass

    def __init__(self):
        self.chips = {}

    def register_chip(self, name, chip):
        self.chips[name] = chip


class FakeGCode:
    def __init__(self):
        self.commands = {}
        self.responses = []

    def register_mux_command(self, name, group, cname, func, desc=None):
        self.commands[(name, cname)] = func

    def respond_info(self, msg):
        self.responses.append(msg)


class FakePrinter:
    def __init__(self):
        self.objects = {
            "pins": FakePins(),
            "gcode": FakeGCode(),
        }
        self.reactor = type("R", (), {"monotonic": lambda self: 0.0})()

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)

    def add_object(self, name, obj):
        self.objects[name] = obj

    def get_reactor(self):
        return self.reactor


class FakeConfig:
    def __init__(self, printer, name):
        self.printer = printer
        self._name = name

    def get_printer(self):
        return self.printer

    def get_name(self):
        return self._name

    def error(self, msg):
        raise Exception(msg)


class FakeGcmd:
    def __init__(self, params=None):
        self.params = params or {}
        self.responses = []

    def get_int(self, name, default=None):
        return int(self.params.get(name, default))

    def respond_info(self, msg):
        self.responses.append(msg)


@pytest.fixture
def printer():
    return FakePrinter()


@pytest.fixture
def chip(printer):
    cfg = FakeConfig(printer, "ams_pin")
    return ams_pin.load_config(cfg)


def test_watcher_and_gcode(printer, chip):
    pin = chip.pins["pin1"]
    triggered = []
    pin.register_watcher(lambda v: triggered.append(v))
    gcode = printer.lookup_object("gcode")
    set_cmd = gcode.commands[("SET_AMS_PIN", "pin1")]
    query_cmd = gcode.commands[("QUERY_AMS_PIN", "pin1")]

    set_cmd(FakeGcmd({"VALUE": 1}))
    assert pin.state
    assert triggered == [False, True]

    gcmd = FakeGcmd()
    query_cmd(gcmd)
    assert "ams_pin pin1: 1" in gcmd.responses[0]
