import sys
import types
import pathlib
import sys

# Ensure the 'extras' package used by AFC_AMS can be imported
sys.path.append(str(pathlib.Path(__file__).resolve().parents[2]))
import extras  # noqa: F401 - ensure package is loaded

afc_unit_mod = types.ModuleType("extras.AFC_unit")
class afcUnit:  # noqa: N801 - mimic expected class name
    pass
afc_unit_mod.afcUnit = afcUnit
sys.modules["extras.AFC_unit"] = afc_unit_mod

afc_lane_mod = types.ModuleType("extras.AFC_lane")
class AFCLaneState:  # noqa: N801 - mimic expected class name
    EJECTING = "Ejecting"
    CALIBRATING = "Calibrating"
    LOADED = "Loaded"
    NONE = "None"
afc_lane_mod.AFCLaneState = AFCLaneState
sys.modules["extras.AFC_lane"] = afc_lane_mod

from extras.AFC_AMS import afcAMS, AFCLaneState

class Dummy:
    pass

class DummyLane:
    def __init__(self, name, unit_type, extruder, runout_lane=None):
        self.name = name
        self.unit_obj = types.SimpleNamespace(type=unit_type, oams_name='oams1')
        self.extruder_obj = extruder
        self.runout_lane = runout_lane
        self.status = AFCLaneState.LOADED
        self.led_not_ready = self.led_index = 0
        self.loaded_to_hub = True
        self._performed = []
        self.hub_obj = types.SimpleNamespace(
            name=f'hub_{name}', switch_pin_callback=lambda *a, **k: None
        )
        self.load_callback = lambda *a, **k: None
        self.prep_callback = lambda *a, **k: None
        self.handle_load_runout = lambda *a, **k: None
        self.index = 1

    def _perform_infinite_runout(self):
        self._performed.append('infinite')

    def _perform_pause_runout(self):
        self._performed.append('pause')


class DummyAFC:
    def __init__(self, lanes, current_lane):
        self.lanes = {lane.name: lane for lane in lanes}
        self.function = types.SimpleNamespace(
            get_current_lane=lambda: current_lane,
            is_printing=lambda: True,
            afc_led=lambda *a, **k: None,
            get_current_extruder=lambda: 'ext1',
        )
        self.spool = types.SimpleNamespace(_clear_values=lambda lane: None)
        self.tools = {}
        self.led_not_ready = 0
        self.save_vars = lambda: None


def make_ams(lanes, current_lane, oams_manager=None):
    ams = afcAMS.__new__(afcAMS)
    ams.afc = DummyAFC(lanes, current_lane)
    ams.lanes = {lane.name: lane for lane in lanes}
    ams._last_prep_states = {}
    ams._last_load_states = {}
    ams._last_hub_states = {}
    ams.interval = 1.0
    ams.reactor = types.SimpleNamespace(
        monotonic=lambda: 0,
        NOW=0,
        register_timer=lambda *a, **k: None,
        update_timer=lambda *a, **k: None,
    )
    ams.oams = None
    ams.oams_manager = oams_manager
    return ams


def test_check_runout_suppressed_for_same_extruder():
    extruder = types.SimpleNamespace(name='ext1')
    lane = DummyLane('L1', 'AMS', extruder, runout_lane='L2')
    backup = DummyLane('L2', 'AMS', extruder)
    ams = make_ams([lane, backup], current_lane='L1')
    assert ams.check_runout(lane) is False


def test_check_runout_suppressed_when_oams_group_loaded():
    extruder = types.SimpleNamespace(name='ext1')
    lane = DummyLane('L1', 'AMS', extruder)
    fps_obj = types.SimpleNamespace(extruder_name='ext1')
    # Use a different current_oams to ensure suppression doesn't depend on it
    fps_state = types.SimpleNamespace(current_group='T0', current_oams='other')
    oams_mgr = types.SimpleNamespace(
        fpss={'fps1': fps_obj},
        current_state=types.SimpleNamespace(fps_state={'fps1': fps_state}),
    )
    ams = make_ams([lane], current_lane='L1', oams_manager=oams_mgr)
    assert ams.check_runout(lane) is False


def test_on_oams_runout_forces_afc():
    extruder = types.SimpleNamespace(name='ext1')
    lane = DummyLane('L1', 'AMS', extruder)
    ams = make_ams([lane], current_lane='L1')
    ams._on_oams_runout('fps1', 'L1', spool_idx=-1)
    assert lane._performed == ['pause']


def test_sync_event_skips_runout_for_ams_lane_with_manager():
    extruder = types.SimpleNamespace(name='ext1')
    lane = DummyLane('L1', 'AMS', extruder)
    fps_obj = types.SimpleNamespace(extruder_name='ext1')
    fps_state = types.SimpleNamespace(current_group='T0')
    oams_mgr = types.SimpleNamespace(
        fpss={'fps1': fps_obj},
        current_state=types.SimpleNamespace(fps_state={'fps1': fps_state}),
    )
    ams = make_ams([lane], current_lane='L1', oams_manager=oams_mgr)
    ams.oams = types.SimpleNamespace(
        f1s_hes_value=[0], hub_hes_value=[0], determine_current_spool=lambda: None
    )
    called = []
    ams._trigger_runout = lambda *a, **k: called.append(True)
    ams._sync_event(0)
    assert not called


def test_sync_event_triggers_runout_without_manager():
    extruder = types.SimpleNamespace(name='ext1')
    lane = DummyLane('L1', 'AMS', extruder)
    ams = make_ams([lane], current_lane='L1')
    ams.oams = types.SimpleNamespace(
        f1s_hes_value=[0], hub_hes_value=[0], determine_current_spool=lambda: None
    )
    called = []
    ams._trigger_runout = lambda *a, **k: called.append(True)
    ams._sync_event(0)
    assert called


def test_check_runout_detects_ams_without_type():
    extruder = types.SimpleNamespace(name='ext1')
    lane = DummyLane('L1', None, extruder, runout_lane='L2')
    backup = DummyLane('L2', None, extruder)
    lane.unit_obj = types.SimpleNamespace(oams_name='oams1')
    backup.unit_obj = types.SimpleNamespace(oams_name='oams1')
    ams = make_ams([lane, backup], current_lane='L1')
    assert ams.check_runout(lane) is False
