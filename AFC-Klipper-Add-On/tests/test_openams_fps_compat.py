from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

MODULE_PATH = Path(__file__).resolve().parents[2] / "klipper_openams" / "src" / "fps.py"
SPEC = importlib.util.spec_from_file_location("openams_fps", MODULE_PATH)
fps_mod = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = fps_mod
SPEC.loader.exec_module(fps_mod)


class _Reactor:
    def monotonic(self):
        return 123.0


class _ADCWithInterval:
    def __init__(self):
        self.args = None

    def setup_adc_sample(self, *_args):
        return None

    def setup_adc_callback(self, report_time, callback):
        self.args = (report_time, callback)


class _ADCNoInterval:
    def __init__(self):
        self.cb = None

    def setup_adc_sample(self, *_args):
        return None

    def setup_adc_callback(self, callback):
        self.cb = callback


class _Pins:
    def __init__(self, adc):
        self._adc = adc

    def setup_pin(self, *_args):
        return self._adc


class _Printer:
    def __init__(self, adc):
        self._adc = adc
        self.objects = {
            "pins": _Pins(adc),
            "oams unit1": object(),
        }

    def get_reactor(self):
        return _Reactor()

    def add_object(self, *_args):
        return None

    def lookup_object(self, name):
        return self.objects.get(name)

    def register_event_handler(self, *_args):
        return None


class _Config:
    def __init__(self, adc):
        self._printer = _Printer(adc)

    def get_printer(self):
        return self._printer

    def get_name(self):
        return "fps test"

    def get(self, key):
        values = {
            "pin": "PA0",
            "extruder": "extruder",
            "oams": "unit1",
        }
        return values[key]

    def getint(self, _key, default):
        return default

    def getfloat(self, _key, default):
        return default

    def getboolean(self, _key, default):
        return default


def test_fps_uses_two_argument_adc_callback_when_supported() -> None:
    adc = _ADCWithInterval()
    fps = fps_mod.FPS(_Config(adc))

    assert adc.args[0] == 0.1
    assert adc.args[1] == fps._adc_callback


def test_fps_falls_back_to_single_argument_adc_callback() -> None:
    adc = _ADCNoInterval()
    fps = fps_mod.FPS(_Config(adc))

    assert adc.cb == fps._adc_callback


def test_fps_adc_callback_accepts_single_value_mode() -> None:
    adc = _ADCNoInterval()
    fps = fps_mod.FPS(_Config(adc))

    fps._adc_callback(0.25)

    assert fps.fps_value == 0.25
