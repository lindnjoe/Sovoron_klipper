import os
import sys
import pytest

repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.join(repo_root, "klipper", "klippy"))
from extras import virtual_input_pin as virtual_pin

class FakeReactor:
    NEVER = float('inf')
    def __init__(self):
        self.callbacks = []
        self.paused_at = None
        self._time = 0.0
    def completion(self):
        class Completion:
            def __init__(self):
                self._result = None
            def complete(self, res):
                self._result = res
            def result(self):
                return self._result
        return Completion()
    def monotonic(self):
        self._time += 0.001
        return self._time
    def register_callback(self, cb):
        self.callbacks.append(cb)
    def pause(self, when):
        self.paused_at = when

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
    def run_script(self, script):
        self.last_script = script

class FakeGcmd:
    def __init__(self, params=None):
        self.params = params or {}
        self.responses = []
    def get_int(self, name, default=None):
        return int(self.params.get(name, default))
    def respond_info(self, msg):
        self.responses.append(msg)

class FakeIdleTimeout:
    def __init__(self):
        self.state = 'Idle'
    def get_status(self, now):
        return {'state': self.state}

class FakeGcodeMacro:
    def load_template(self, config, name, default=''):
        class Tmpl:
            def render(self):
                return ''
        return Tmpl()

class FakePauseResume:
    def send_pause_command(self):
        pass

class FakePrinter:
    def __init__(self):
        self.objects = {
            'pins': FakePins(),
            'gcode': FakeGCode(),
            'gcode_macro': FakeGcodeMacro(),
            'pause_resume': FakePauseResume(),
            'idle_timeout': FakeIdleTimeout(),
        }
        self.reactor = FakeReactor()
        self.event_handlers = {}
    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)
    def load_object(self, config, name):
        return self.objects.get(name)
    def get_reactor(self):
        return self.reactor
    def register_event_handler(self, event, handler):
        self.event_handlers.setdefault(event, []).append(handler)

class FakeConfig:
    def __init__(self, printer, name, opts=None):
        self.printer = printer
        self._name = name
        self.opts = opts or {}
    def get_printer(self):
        return self.printer
    def get_name(self):
        return self._name
    def getboolean(self, key, default=False):
        return bool(self.opts.get(key, default))
    def getfloat(self, key, default=0., above=None, minval=None):
        return float(self.opts.get(key, default))
    def get(self, key, default=None):
        return self.opts.get(key, default)

class TestVirtualPin(virtual_pin.VirtualInputPin):
    def __init__(self, config):
        self.watchers = []
        super().__init__(config)
    def set_value(self, val):
        super().set_value(val)
        for cb in self.watchers:
            cb(val)

@pytest.fixture
def printer():
    return FakePrinter()

@pytest.fixture
def vpin(printer):
    cfg = FakeConfig(printer, 'virtual_pin test')
    pin = TestVirtualPin(cfg)
    printer.objects['virtual_pin ' + pin.name] = pin
    return pin

@pytest.fixture
def vpin2(printer):
    cfg = FakeConfig(printer, 'virtual_pin alt')
    pin = TestVirtualPin(cfg)
    printer.objects['virtual_pin ' + pin.name] = pin
    return pin

@pytest.fixture
def fil_sensor(printer, vpin):
    cfg = FakeConfig(printer, 'virtual_filament_sensor sensor', {'pin': vpin.name})
    sensor = virtual_pin.VirtualFilamentSensor(cfg)
    # automatically update sensor when virtual pin changes
    vpin.watchers.append(lambda val: sensor.runout_helper.note_filament_present(
        sensor.reactor.monotonic(), bool(val)))
    return sensor


def test_watchers_trigger(printer, vpin):
    triggered = []
    vpin.watchers.append(lambda val: triggered.append(val))
    vpin.set_value(1)
    assert triggered == [1]


def test_filament_sensor_updates(vpin, fil_sensor):
    assert not fil_sensor.runout_helper.filament_present
    vpin.set_value(1)
    assert fil_sensor.runout_helper.filament_present
    vpin.set_value(0)
    assert not fil_sensor.runout_helper.filament_present


def test_gcode_handlers(printer, vpin):
    gcode = printer.lookup_object('gcode')
    cmd_set = gcode.commands[('SET_VIRTUAL_PIN', vpin.name)]
    cmd_query = gcode.commands[('QUERY_VIRTUAL_PIN', vpin.name)]
    cmd_set(FakeGcmd({'VALUE': 1}))
    assert vpin.state
    gcmd = FakeGcmd()
    cmd_query(gcmd)
    assert 'virtual_pin %s: 1' % vpin.name in gcmd.responses[0]


def test_multiple_pins(printer, vpin, vpin2):
    gcode = printer.lookup_object('gcode')
    cmd1 = gcode.commands[('SET_VIRTUAL_PIN', vpin.name)]
    cmd2 = gcode.commands[('SET_VIRTUAL_PIN', vpin2.name)]
    cmd1(FakeGcmd({'VALUE': 1}))
    assert vpin.state and not vpin2.state
    cmd2(FakeGcmd({'VALUE': 1}))
    assert vpin2.state
