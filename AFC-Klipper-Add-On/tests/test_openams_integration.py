from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from unittest.mock import Mock

import pytest

MODULE_PATH = Path(__file__).resolve().parents[1] / "extras" / "openams_integration.py"
SPEC = importlib.util.spec_from_file_location("openams_integration", MODULE_PATH)
openams_integration = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = openams_integration
SPEC.loader.exec_module(openams_integration)


@pytest.fixture(autouse=True)
def reset_singletons() -> None:
    openams_integration.AMSEventBus._instance = None
    openams_integration.LaneRegistry._instances = {}


def test_normalize_extruder_name_handles_aliases_and_invalid_values() -> None:
    assert openams_integration.normalize_extruder_name(" Extruder4 ") == "extruder4"
    assert openams_integration.normalize_extruder_name("ams_Extruder") == "extruder"
    assert openams_integration.normalize_extruder_name("   ") is None
    assert openams_integration.normalize_extruder_name(None) is None
    assert openams_integration.normalize_extruder_name(123) is None


def test_normalize_oams_name_defaults_and_prefix_stripping() -> None:
    assert openams_integration.normalize_oams_name("OAMS unit_1") == "unit_1"
    assert openams_integration.normalize_oams_name("   ") == "default"
    assert openams_integration.normalize_oams_name(None, default="fallback") == "fallback"


def test_event_bus_priority_order_and_unsubscribe() -> None:
    event_bus = openams_integration.AMSEventBus.get_instance(logger=Mock())
    call_order = []

    def low_priority(**kwargs):
        call_order.append(("low", kwargs["value"]))

    def high_priority(**kwargs):
        call_order.append(("high", kwargs["value"]))

    event_bus.subscribe("lane_updated", low_priority, priority=0)
    event_bus.subscribe("lane_updated", high_priority, priority=10)

    handled = event_bus.publish("lane_updated", value=42)
    assert handled == 2
    assert call_order == [("high", 42), ("low", 42)]

    event_bus.unsubscribe("lane_updated", high_priority)
    call_order.clear()

    handled = event_bus.publish("lane_updated", value=7)
    assert handled == 1
    assert call_order == [("low", 7)]


def test_event_bus_publish_without_subscribers_records_history() -> None:
    event_bus = openams_integration.AMSEventBus.get_instance()

    handled = event_bus.publish("unknown_event", lane="lane1")

    assert handled == 0
    history = event_bus.get_history(event_type="unknown_event")
    assert len(history) == 1
    assert history[0][0] == "unknown_event"
    assert history[0][2]["lane"] == "lane1"


def test_lane_registry_register_lookup_and_re_register() -> None:
    printer = object()
    logger = Mock()
    registry = openams_integration.LaneRegistry.for_printer(printer, logger=logger)

    info = registry.register_lane(
        lane_name="lane4",
        unit_name="AMS_1",
        spool_index=0,
        extruder="extruder4",
        fps_name="fps1",
    )

    assert registry.get_by_lane("lane4") == info
    assert registry.resolve_lane_token("LaNe4") == info
    assert registry.get_by_spool("AMS_1", 0) == info
    assert registry.resolve_extruder("lane4") == "extruder4"
    assert registry.resolve_spool_index("lane4") == 0

    updated = registry.register_lane(
        lane_name="lane4",
        unit_name="AMS_2",
        spool_index=1,
        extruder="extruder5",
    )
    assert updated is not info
    assert registry.get_by_spool("AMS_1", 0) is None
    assert registry.get_by_spool("AMS_2", 1) == updated
    assert registry.get_by_extruder("extruder4") == []
    assert registry.get_by_extruder("extruder5") == [updated]


def test_lane_registry_is_scoped_per_printer_instance() -> None:
    registry_a = openams_integration.LaneRegistry.for_printer(object())
    registry_b = openams_integration.LaneRegistry.for_printer(object())

    assert registry_a is not registry_b
