import importlib.util
import json
import logging
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
_EXTRA_IMPORT_PATHS = [
    REPO_ROOT / "AFC-Klipper-Add-On",
    REPO_ROOT / "klipper" / "klippy",
]

for extra_path in _EXTRA_IMPORT_PATHS:
    if extra_path.is_dir():
        path_str = str(extra_path)
        if path_str not in sys.path:
            sys.path.insert(0, path_str)

import extras

import types

if "configfile" not in sys.modules:
    configfile_stub = types.ModuleType("configfile")
    configfile_stub.error = RuntimeError
    sys.modules["configfile"] = configfile_stub

_afc_extras_path = REPO_ROOT / "AFC-Klipper-Add-On" / "extras"
if _afc_extras_path.is_dir():
    extras_path_str = str(_afc_extras_path)
    if extras_path_str not in extras.__path__:
        extras.__path__.append(extras_path_str)


MODULE_PATH = Path(__file__).resolve().parents[1] / "AFC-Klipper-Add-On" / "extras" / "openams_integration.py"
MODULE_NAME = "openams_integration_module"

_spec = importlib.util.spec_from_file_location(MODULE_NAME, MODULE_PATH)
_openams_module = importlib.util.module_from_spec(_spec)
assert _spec.loader is not None
sys.modules[MODULE_NAME] = _openams_module
_spec.loader.exec_module(_openams_module)

AFC_OPENAMS_PATH = Path(__file__).resolve().parents[1] / "AFC-Klipper-Add-On" / "extras" / "AFC_OpenAMS.py"
AFC_OPENAMS_NAME = "afc_openams_module"

_afc_spec = importlib.util.spec_from_file_location(AFC_OPENAMS_NAME, AFC_OPENAMS_PATH)
_afc_openams_module = importlib.util.module_from_spec(_afc_spec)
assert _afc_spec.loader is not None
sys.modules[AFC_OPENAMS_NAME] = _afc_openams_module
_afc_spec.loader.exec_module(_afc_openams_module)

OAMS_MANAGER_PATH = Path(__file__).resolve().parents[1] / "klipper_openams" / "src" / "oams_manager.py"
OAMS_MANAGER_NAME = "oams_manager_module"

_manager_spec = importlib.util.spec_from_file_location(OAMS_MANAGER_NAME, OAMS_MANAGER_PATH)
_oams_manager_module = importlib.util.module_from_spec(_manager_spec)
assert _manager_spec.loader is not None
sys.modules[OAMS_MANAGER_NAME] = _oams_manager_module
_manager_spec.loader.exec_module(_oams_manager_module)


@pytest.fixture(autouse=True)
def reset_openams_module_state():
    module = _openams_module
    module.AMSEventBus._instance = None
    module.AMSEventBus._lock = module.threading.RLock()
    module.LaneRegistry._instances.clear()
    module.LaneRegistry._lock = module.threading.RLock()
    module.AMSHardwareService._instances.clear()
    module.AMSRunoutCoordinator._units.clear()
    module.AMSRunoutCoordinator._monitors.clear()
    yield
    module.AMSEventBus._instance = None
    module.LaneRegistry._instances.clear()
    module.AMSHardwareService._instances.clear()
    module.AMSRunoutCoordinator._units.clear()
    module.AMSRunoutCoordinator._monitors.clear()


def test_event_bus_priority_and_history():
    module = _openams_module
    bus = module.AMSEventBus.get_instance()

    call_order = []

    def high_priority_handler(*, event_type, **kwargs):
        call_order.append(("high", event_type, kwargs["spool_index"]))

    def low_priority_handler(*, event_type, **kwargs):
        call_order.append(("low", event_type, kwargs["spool_index"]))

    bus.subscribe("spool_loaded", low_priority_handler, priority=0)
    bus.subscribe("spool_loaded", high_priority_handler, priority=5)

    handled = bus.publish("spool_loaded", unit_name="AMS_1", spool_index=2, eventtime=123.0)

    assert handled == 2
    assert call_order == [
        ("high", "spool_loaded", 2),
        ("low", "spool_loaded", 2),
    ]

    history = bus.get_history()
    assert history[-1][0] == "spool_loaded"
    assert history[-1][1] == 123.0
    assert history[-1][2]["unit_name"] == "AMS_1"

    filtered = bus.get_history(event_type="spool_loaded")
    assert filtered[-1] == history[-1]


def test_lane_registry_register_and_resolve():
    module = _openams_module
    printer = object()
    registry = module.LaneRegistry.for_printer(printer)

    info = registry.register_lane(
        lane_name="lane4",
        unit_name="AMS_1",
        spool_index=0,
        group="T4",
        extruder="extruder4",
        fps_name="fps1",
        hub_name="Hub_1",
        led_index="LED_1",
    )

    assert registry.get_by_lane("lane4") is info
    assert registry.get_by_spool("AMS_1", 0) is info
    assert registry.get_by_group("T4") is info
    assert registry.get_by_extruder("extruder4") == [info]
    assert registry.resolve_lane_name("AMS_1", 0) == "lane4"
    assert registry.resolve_group("AMS_1", 0) == "T4"
    assert registry.resolve_spool_index("lane4") == 0
    assert registry.resolve_extruder("lane4") == "extruder4"

    registry.register_lane(
        lane_name="lane5",
        unit_name="AMS_1",
        spool_index=1,
        group="T5",
        extruder="extruder4",
    )

    extruder_lanes = {lane.lane_name for lane in registry.get_by_extruder("extruder4")}
    assert extruder_lanes == {"lane4", "lane5"}


def test_hardware_service_spool_and_snapshot_events():
    module = _openams_module

    class DummyCommand:
        def __init__(self):
            self.sent = []

        def send(self, payload):
            # Store a copy to avoid accidental mutation
            self.sent.append(list(payload))

    class DummyController:
        def __init__(self):
            self.oams_load_spool_cmd = DummyCommand()
            self.oams_unload_spool_cmd = DummyCommand()
            self.oams_follower_cmd = DummyCommand()
            self.current_spool = None

        def get_status(self, eventtime):
            return {"current_spool": self.current_spool, "eventtime": eventtime}

        def set_led_error(self, idx, value):
            pass

    class DummyReactor:
        def __init__(self):
            self._now = 0.0

        def monotonic(self):
            self._now += 0.5
            return self._now

    class DummyPrinter:
        def __init__(self):
            self._reactor = DummyReactor()
            self._objects = {}

        def lookup_object(self, name, default=None):
            return self._objects.get(name, default)

        def get_reactor(self):
            return self._reactor

    printer = DummyPrinter()
    controller = DummyController()
    service = module.AMSHardwareService.for_printer(printer, name="AMS_1")
    service.attach_controller(controller)

    service.registry.register_lane(
        lane_name="lane4",
        unit_name="AMS_1",
        spool_index=0,
        group="T4",
        extruder="extruder4",
    )

    bus = module.AMSEventBus.get_instance()
    captured = []

    def capture_handler(*, event_type, **kwargs):
        captured.append((event_type, kwargs))

    for event_name in [
        "spool_loaded",
        "spool_unloaded",
        "lane_hub_loaded",
        "lane_tool_loaded",
    ]:
        bus.subscribe(event_name, capture_handler)

    service.load_spool(0)
    assert controller.oams_load_spool_cmd.sent == [[0]]
    assert captured[0][0] == "spool_loaded"
    assert captured[0][1]["spool_index"] == 0

    controller.current_spool = 0
    service.unload_spool()
    assert controller.oams_unload_spool_cmd.sent == [[]]
    assert captured[1][0] == "spool_unloaded"
    assert captured[1][1]["spool_index"] == 0

    baseline = len(captured)
    service.update_lane_snapshot(
        "AMS_1",
        "lane4",
        lane_state=True,
        hub_state=False,
        eventtime=1.0,
        spool_index=0,
        tool_state=False,
        emit_spool_event=False,
    )

    assert len(captured) == baseline

    service.update_lane_snapshot(
        "AMS_1",
        "lane4",
        lane_state=True,
        hub_state=True,
        eventtime=2.0,
        spool_index=0,
        tool_state=True,
    )

    snapshot = service.latest_lane_snapshot_for_spool("AMS_1", 0)
    assert snapshot is not None
    assert snapshot["hub_state"] is True
    assert snapshot["tool_state"] is True

    service.update_lane_snapshot(
        "AMS_1",
        "lane4",
        lane_state=False,
        hub_state=False,
        eventtime=3.0,
        spool_index=0,
        tool_state=False,
    )

    new_events = captured[baseline:]
    event_types = [event for event, _ in new_events]
    assert event_types == [
        "lane_hub_loaded",
        "lane_tool_loaded",
        "spool_unloaded",
    ]

    assert service.resolve_lane_for_spool("AMS_1", 0) == "lane4"


def test_afcams_lane_temperature_uses_saved_file(tmp_path):
    module = _afc_openams_module

    class DummyPrinter:
        def __init__(self):
            self.objects = {}

        def lookup_object(self, name, default=None):
            return self.objects.get(name, default)

    class DummyLane:
        def __init__(self, name, temp):
            self.name = name
            self.extruder_temp = temp

    helper = object.__new__(module.afcAMS)
    helper.logger = logging.getLogger("test_afcams_lane_temperature")
    helper.printer = DummyPrinter()
    helper.lanes = {}
    helper.afc = SimpleNamespace(VarFile=str(tmp_path / "AFC.var"))
    helper.name = "AMS_1"
    helper._saved_unit_cache = None
    helper._saved_unit_mtime = None
    helper._lane_temp_cache = {}
    helper._last_loaded_lane_by_extruder = {}

    helper.printer.objects["AFC_lane lane8"] = DummyLane("lane8", 220)

    unit_file = tmp_path / "AFC.var.unit"
    payload = {"AMS_1": {"lane8": {"extruder_temp": 220}}, "system": {}}
    unit_file.write_text(json.dumps(payload))

    result = module.afcAMS.get_lane_temperature(helper, "lane8", 240)

    assert result == 220
    assert helper._lane_temp_cache.get("lane8") == 220


def test_fpsstate_reset_clog_tracker_clears_restore_flags():
    module = _oams_manager_module

    state = module.FPSState()
    state.clog_active = True
    state.clog_restore_follower = True
    state.clog_restore_direction = -1
    state.clog_start_extruder = 5.0
    state.clog_start_encoder = 12

    state.reset_clog_tracker()

    assert state.clog_active is False
    assert state.clog_restore_follower is False
    assert state.clog_restore_direction == 1
    assert state.clog_start_extruder is None
    assert state.clog_start_encoder is None
