import sys
import types
from pathlib import Path

# Ensure the klipper package is importable and provide stub modules expected
# by AFC_AMS during import.
repo_root = Path(__file__).resolve().parents[4]
sys.path.append(str(repo_root / "klipper"))

# Stub out the optional AFC modules that AFC_AMS depends on so the tests do
# not require the full AFC environment.
extras = types.ModuleType("extras")
sys.modules.setdefault("extras", extras)
sys.modules["extras.AFC_unit"] = types.ModuleType("extras.AFC_unit")
sys.modules["extras.AFC_lane"] = types.ModuleType("extras.AFC_lane")
sys.modules["extras.AFC_unit"].afcUnit = object
sys.modules["extras.AFC_lane"].AFCLaneState = object

from klippy.extras.AFC_AMS import afcAMS


class DummyHub:
    def __init__(self):
        self.name = "hub1"
        self.switch_calls = []

    def switch_pin_callback(self, eventtime, state):
        self.switch_calls.append((eventtime, state))


class DummyLane:
    def __init__(self):
        self.name = "lane1"
        self.index = 1
        self.hub_obj = DummyHub()
        self.prep_calls = []
        self.load_calls = []
        self.runout_triggered = False

    def prep_callback(self, eventtime, state):
        self.prep_calls.append((eventtime, state))

    def load_callback(self, eventtime, state):
        self.load_calls.append((eventtime, state))


def _build_ams():
    ams = afcAMS.__new__(afcAMS)
    ams.lanes = {}
    ams.oams = type("DummyOAMS", (), {
        "f1s_hes_value": [True],
        "hub_hes_value": [True],
        "determine_current_spool": lambda self: None,
    })()
    ams.oams_manager = object()
    ams._last_prep_states = {}
    ams._last_load_states = {}
    ams._last_hub_states = {}
    ams._is_ams_lane = lambda lane: True
    ams.check_runout = lambda lane: True
    ams.interval = 1.0

    def trigger(lane):
        lane.runout_triggered = True

    ams._trigger_runout = trigger
    return ams


def test_prep_drop_no_runout():
    ams = _build_ams()
    lane = DummyLane()
    ams.lanes[lane.name] = lane

    # Initial sync with both sensors True
    ams._sync_event(0.0)
    initial_load_calls = list(lane.load_calls)

    # Only the prep sensor drops
    ams.oams.f1s_hes_value[0] = False
    ams._sync_event(1.0)

    assert lane.prep_calls[-1] == (1.0, False)
    assert lane.load_calls == initial_load_calls, (
        "Load callback should not fire when hub remains True"
    )
    assert not lane.runout_triggered, "Runout should not trigger on prep drop alone"

