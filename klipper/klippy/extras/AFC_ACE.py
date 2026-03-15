# Armored Turtle Automated Filament Changer
#
# ACE Pro Unit Type - Integrates Anycubic ACE PRO hardware with AFC
# Supports both ACEPRO (Kobra-S1/ACEPRO) and DuckACE (utkabobr/DuckACE) backends.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import logging
import traceback

from configparser import Error as ConfigError
from typing import TYPE_CHECKING, Any, Dict, Optional

if TYPE_CHECKING:
    from extras.AFC_lane import AFCLane
    from configfile import ConfigWrapper

try:
    from extras.AFC_utils import ERROR_STR
except Exception:
    trace = traceback.format_exc()
    raise ConfigError(f"Error when trying to import AFC_utils.ERROR_STR\n{trace}")

try:
    from extras.AFC_unit import afcUnit
except Exception:
    raise ConfigError(ERROR_STR.format(import_lib="AFC_unit", trace=traceback.format_exc()))

try:
    from extras.AFC_lane import AFCLane, AFCLaneState
except Exception:
    raise ConfigError(ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc()))

try:
    from extras.AFC_respond import AFCprompt
except Exception:
    raise ConfigError(ERROR_STR.format(import_lib="AFC_respond", trace=traceback.format_exc()))


_module_logger = logging.getLogger(__name__)

# Sentinel for "not yet looked up"
_UNSET = object()

# Backend type constants
BACKEND_ACEPRO = "acepro"
BACKEND_DUCKACE = "duckace"


class _AceBackend:
    """Adapter that normalizes API differences between ACE backends.

    ACEPRO: AceManager with instances[] list, each AceInstance has .connected,
            .inventory (list of dicts), .get_status(), ._retract(), etc.
    DuckACE: Single DuckAce object with ._connected, ._info['slots'],
             gcode commands for tool changes.
    """

    def __init__(self, backend_type, ace_obj, instance_index, logger):
        self.backend_type = backend_type
        self._ace_obj = ace_obj
        self._instance_index = instance_index
        self.logger = logger

    @property
    def is_connected(self):
        if self.backend_type == BACKEND_ACEPRO:
            instance = self._get_acepro_instance()
            if instance is None:
                return False
            return getattr(instance, "connected", False)
        else:
            return getattr(self._ace_obj, "_connected", False)

    def get_slot_info(self, local_slot):
        """Return normalized slot info dict: {status, material, color}."""
        if self.backend_type == BACKEND_ACEPRO:
            instance = self._get_acepro_instance()
            if instance is None:
                return {}
            inventory = getattr(instance, "inventory", [])
            if local_slot < len(inventory):
                return inventory[local_slot]
            return {}
        else:
            info = getattr(self._ace_obj, "_info", {})
            slots = info.get("slots", [])
            if local_slot < len(slots):
                raw = slots[local_slot]
                return {
                    "status": raw.get("status", ""),
                    "material": raw.get("type", raw.get("sku", "")),
                    "color": raw.get("color", [0, 0, 0]),
                }
            return {}

    def get_hardware_status(self, eventtime=None):
        """Return backend-specific hardware status dict."""
        if self.backend_type == BACKEND_ACEPRO:
            instance = self._get_acepro_instance()
            if instance is not None and hasattr(instance, "get_status"):
                try:
                    return instance.get_status(eventtime)
                except Exception as e:
                    self.logger.debug(f"ACE status query error: {e}")
            return {}
        else:
            info = getattr(self._ace_obj, "_info", {})
            return dict(info)

    def has_perform_tool_change(self):
        if self.backend_type == BACKEND_ACEPRO:
            return hasattr(self._ace_obj, "perform_tool_change")
        return False

    def perform_tool_change(self, global_tool):
        """Invoke backend tool change (ACEPRO manager only)."""
        self._ace_obj.perform_tool_change(global_tool)

    def has_direct_feed(self):
        if self.backend_type == BACKEND_ACEPRO:
            instance = self._get_acepro_instance()
            return instance is not None and hasattr(instance, "_feed_to_toolhead_with_extruder_assist")
        return False

    def direct_feed(self, local_slot):
        """Feed filament via ACEPRO instance method."""
        instance = self._get_acepro_instance()
        instance._feed_to_toolhead_with_extruder_assist(local_slot)

    def has_direct_retract(self):
        if self.backend_type == BACKEND_ACEPRO:
            instance = self._get_acepro_instance()
            return instance is not None and hasattr(instance, "_retract")
        return False

    def direct_retract(self, local_slot):
        """Retract filament via ACEPRO instance method."""
        instance = self._get_acepro_instance()
        instance._retract(local_slot)

    def _get_acepro_instance(self):
        """Get the AceInstance for this unit (ACEPRO only)."""
        instances = getattr(self._ace_obj, "instances", [])
        if self._instance_index < len(instances):
            return instances[self._instance_index]
        return None


def _detect_backend(ace_obj):
    """Auto-detect whether the ace printer object is ACEPRO or DuckACE.

    ACEPRO has an 'instances' attribute (list of AceInstance objects).
    DuckACE has '_info' and '_connected' attributes.
    """
    if hasattr(ace_obj, "instances"):
        return BACKEND_ACEPRO
    if hasattr(ace_obj, "_info"):
        return BACKEND_DUCKACE
    # Fallback: check class name
    cls_name = type(ace_obj).__name__.lower()
    if "duck" in cls_name:
        return BACKEND_DUCKACE
    return BACKEND_ACEPRO


class afcACE(afcUnit):
    """AFC unit subclass that integrates Anycubic ACE PRO hardware.

    Supports both ACEPRO and DuckACE backends. The backend is auto-detected
    from the [ace] Klipper module, or can be explicitly set via the
    'ace_backend' config option.

    Each afcACE instance maps to one physical ACE PRO unit with 4 filament slots.
    """

    SLOTS_PER_UNIT = 4

    def __init__(self, config: ConfigWrapper):
        super().__init__(config)
        self.type = "ACE"
        self.logger = self.afc.logger

        # ACE instance index (0-based) - which physical ACE unit this maps to
        # For DuckACE (single-unit), this should stay 0.
        self.ace_instance_index = config.getint("ace_instance", 0)

        # Optional explicit backend selection: "acepro", "duckace", or "auto"
        self._configured_backend = config.get("ace_backend", "auto").lower().strip()

        # Cached references
        self._cached_ace_obj = _UNSET
        self._backend: Optional[_AceBackend] = None

        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self):
        """Look up ACE module objects once Klipper is fully ready."""
        ace_obj = self.printer.lookup_object("ace", None)
        self._cached_ace_obj = ace_obj

        if ace_obj is None:
            self.logger.warning(
                f"ACE unit {self.name}: [ace] module not found. "
                "Ensure ACEPRO or DuckACE is installed and [ace] section exists in config."
            )
            return

        # Detect or use configured backend
        if self._configured_backend == "auto":
            backend_type = _detect_backend(ace_obj)
        elif self._configured_backend in (BACKEND_ACEPRO, BACKEND_DUCKACE):
            backend_type = self._configured_backend
        else:
            self.logger.warning(
                f"ACE unit {self.name}: unknown ace_backend '{self._configured_backend}', "
                "falling back to auto-detection."
            )
            backend_type = _detect_backend(ace_obj)

        self._backend = _AceBackend(
            backend_type, ace_obj, self.ace_instance_index, self.logger
        )

        # Validate ACEPRO instance index
        if backend_type == BACKEND_ACEPRO:
            instances = getattr(ace_obj, "instances", [])
            if self.ace_instance_index >= len(instances):
                self.logger.error(
                    f"ACE unit {self.name}: instance index {self.ace_instance_index} "
                    f"out of range (ace_count={len(instances)})"
                )
                self._backend = None
                return

        self.logger.info(
            f"ACE unit {self.name}: using {backend_type} backend, "
            f"instance {self.ace_instance_index}"
        )

        # Sync initial slot loaded state to lanes
        self._sync_slot_loaded_state()

        # Hook runout detection into AFC's infinite spool system
        if self._backend.backend_type == BACKEND_ACEPRO:
            self._hook_acepro_runout_monitor()
        else:
            # DuckACE: use periodic slot status polling for runout detection
            self._start_slot_status_monitor()

    def _sync_slot_loaded_state(self):
        """Sync ACE slot status to lane loaded_to_hub for virtual sensor states.

        ACE lanes have no physical prep/load sensors. Instead, the lane's
        load_state and prep_state properties derive from loaded_to_hub when
        the unit type is ACE. This method queries the ACE backend for each
        lane's slot status and updates loaded_to_hub accordingly.
        """
        backend = self._backend or self._get_backend()
        if backend is None:
            return
        for lane in self.lanes.values():
            local_slot = self._get_local_slot_for_lane(lane)
            if 0 <= local_slot < self.SLOTS_PER_UNIT:
                slot_info = backend.get_slot_info(local_slot)
                slot_loaded = bool(slot_info and slot_info.get("status", "") == "ready")
                lane.loaded_to_hub = slot_loaded

    def _get_backend(self) -> Optional[_AceBackend]:
        """Get the backend adapter, initializing lazily if needed."""
        if self._backend is not None:
            return self._backend

        if self._cached_ace_obj is _UNSET:
            self._cached_ace_obj = self.printer.lookup_object("ace", None)

        ace_obj = self._cached_ace_obj
        if ace_obj is None:
            return None

        if self._configured_backend == "auto":
            backend_type = _detect_backend(ace_obj)
        elif self._configured_backend in (BACKEND_ACEPRO, BACKEND_DUCKACE):
            backend_type = self._configured_backend
        else:
            backend_type = _detect_backend(ace_obj)

        self._backend = _AceBackend(
            backend_type, ace_obj, self.ace_instance_index, self.logger
        )
        return self._backend

    def _get_local_slot_for_lane(self, lane) -> int:
        """Map an AFC lane to a local ACE slot index (0-3).

        The lane's unit field is formatted as 'UnitName:SlotIndex' where
        SlotIndex is 1-based in the config. We convert to 0-based for ACE.
        """
        unit_field = getattr(lane, "unit", "")
        if ":" in str(unit_field):
            try:
                slot_str = str(unit_field).split(":")[-1]
                return int(slot_str) - 1  # Config is 1-based, ACE is 0-based
            except (ValueError, IndexError):
                pass
        return -1

    # ---- AFC Unit Interface Implementation ----

    def handle_connect(self):
        super().handle_connect()

        self.logo = '<span class=success--text>R  _____ _____ _____\n'
        self.logo += 'E |  A  |  C  |  E  |\n'
        self.logo += 'A |  P  |  R  |  O  |\n'
        self.logo += 'D |_____|_____|_____|\n'
        self.logo += 'Y |_1_|_2_|_3_|_4_|\n'
        self.logo += '  ' + self.name + '\n'

        self.logo_error = '<span class=error--text>E  _____ _____ _____\n'
        self.logo_error += 'R |  A  |  C  |  E  |\n'
        self.logo_error += 'R | ERR | ERR | ERR |\n'
        self.logo_error += 'O |_____|_____|_____|\n'
        self.logo_error += 'R |_X_|_X_|_X_|_X_|\n'
        self.logo_error += '  ' + self.name + '</span>\n'

    def get_status(self, eventtime=None):
        """Return status dict including ACE hardware state."""
        response = super().get_status(eventtime)

        backend = self._get_backend()
        if backend is not None:
            response["ace_backend"] = backend.backend_type
            response["ace_connected"] = backend.is_connected
            try:
                response["ace_status"] = backend.get_hardware_status(eventtime)
            except Exception as e:
                self.logger.debug(f"ACE status query error: {e}")
                response["ace_status"] = {}
        else:
            response["ace_backend"] = None
            response["ace_connected"] = False
            response["ace_status"] = {}

        return response

    def load_sequence(self, cur_lane, cur_hub, cur_extruder):
        """ACE load sequence - delegates filament feeding to ACE hardware."""
        afc = self.afc
        backend = self._get_backend()

        if backend is None:
            afc.error.handle_lane_failure(
                cur_lane, f"ACE load failed: ACE module not available for {self.name}"
            )
            return False

        # Check if already loaded
        if (cur_lane.get_toolhead_pre_sensor_state()
                and hasattr(cur_lane, "tool_loaded") and cur_lane.tool_loaded):
            self.logger.debug(f"Lane {cur_lane.name} already loaded to toolhead, skipping load")
            cur_lane.set_tool_loaded()
            afc.save_vars()
            return True

        if afc._check_extruder_temp(cur_lane):
            afc.afcDeltaTime.log_with_time("Done heating toolhead")

        local_slot = self._get_local_slot_for_lane(cur_lane)
        if local_slot < 0:
            afc.error.handle_lane_failure(
                cur_lane, f"ACE load failed: cannot determine slot for {cur_lane.name}"
            )
            return False

        # Calculate global tool index for ACE
        global_tool = self.ace_instance_index * self.SLOTS_PER_UNIT + local_slot

        try:
            self.logger.info(
                f"ACE load ({backend.backend_type}): feeding tool T{global_tool} "
                f"(instance {self.ace_instance_index}, slot {local_slot}) "
                f"for lane {cur_lane.name}"
            )

            # Try backend-specific methods first, fall back to gcode command
            if backend.has_perform_tool_change():
                backend.perform_tool_change(global_tool)
            elif backend.has_direct_feed():
                backend.direct_feed(local_slot)
            else:
                # Works for both backends - ACE_CHANGE_TOOL is registered by both
                self.gcode.run_script_from_command(
                    f"ACE_CHANGE_TOOL TOOL={global_tool}"
                )
        except Exception as e:
            message = f"ACE load failed for {cur_lane.name}: {e}"
            afc.error.handle_lane_failure(cur_lane, message)
            return False

        # Buffer/ramming mode: if tool_start == "buffer", the buffer's advance_state
        # is used as the toolhead sensor. After loading, retract off the buffer sensor
        # to confirm the load and reset the buffer state.
        if cur_extruder.tool_start == "buffer" and cur_lane.buffer_obj is not None:
            try:
                cur_lane.unsync_to_extruder()
                load_checks = 0
                while cur_lane.get_toolhead_pre_sensor_state():
                    cur_lane.move_advanced(cur_lane.short_move_dis * -1, 1)
                    load_checks += 1
                    afc.reactor.pause(afc.reactor.monotonic() + 0.1)
                    if load_checks > afc.tool_max_load_checks:
                        message = (
                            f"Buffer did not decompress after {afc.tool_max_load_checks} "
                            f"retract moves. Check filament path and buffer.\n"
                            f"To resolve, set lane loaded with "
                            f"`SET_LANE_LOADED LANE={cur_lane.name}` macro."
                        )
                        afc.error.handle_lane_failure(cur_lane, message)
                        return False
                cur_lane.sync_to_extruder()
            except Exception as e:
                message = f"ACE buffer load check failed for {cur_lane.name}: {e}"
                self.logger.error(f"{message}\n{traceback.format_exc()}")
                afc.error.handle_lane_failure(cur_lane, message)
                return False
        else:
            if not cur_lane.get_toolhead_pre_sensor_state():
                message = (
                    "ACE load did not trigger toolhead sensor. CHECK FILAMENT PATH\n"
                    "To resolve, set lane loaded with "
                    f"`SET_LANE_LOADED LANE={cur_lane.name}` macro."
                )
                if afc.function.in_print():
                    message += "\nOnce filament is fully loaded click resume to continue printing"
                afc.error.handle_lane_failure(cur_lane, message)
                return False

        cur_lane.set_tool_loaded()
        cur_lane.enable_buffer(disable_fault=True)
        afc.save_vars()
        return True

    def unload_sequence(self, cur_lane, cur_hub, cur_extruder):
        """ACE unload sequence - shared toolhead steps then ACE retraction."""
        afc = self.afc
        backend = self._get_backend()

        if backend is None:
            afc.error.handle_lane_failure(
                cur_lane, f"ACE unload failed: ACE module not available for {self.name}"
            )
            return False

        cur_lane.status = AFCLaneState.TOOL_UNLOADING

        # Disable buffer before unloading (safe no-op if no buffer)
        cur_lane.disable_buffer()

        if afc._check_extruder_temp(cur_lane):
            afc.afcDeltaTime.log_with_time("Done heating toolhead")

        # Quick pull to prevent oozing
        afc.move_e_pos(-2, cur_extruder.tool_unload_speed, "Quick Pull", wait_tool=False)
        self.lane_unloading(cur_lane)
        cur_lane.sync_to_extruder()
        cur_lane.do_enable(True)
        cur_lane.select_lane()

        # Shared toolhead steps: cut, park, form tip
        if afc.tool_cut:
            cur_lane.extruder_obj.estats.increase_cut_total()
            afc.gcode.run_script_from_command(afc.tool_cut_cmd)

            if afc.park:
                afc.gcode.run_script_from_command(afc.park_cmd)

        if afc.form_tip:
            if afc.park:
                afc.gcode.run_script_from_command(afc.park_cmd)

            if afc.form_tip_cmd == "AFC":
                afc.tip = self.printer.lookup_object("AFC_form_tip")
                afc.tip.tip_form()
            else:
                afc.gcode.run_script_from_command(afc.form_tip_cmd)

        local_slot = self._get_local_slot_for_lane(cur_lane)
        global_tool = self.ace_instance_index * self.SLOTS_PER_UNIT + local_slot

        try:
            # Unsync from extruder before ACE retraction
            cur_lane.unsync_to_extruder()

            self.logger.info(
                f"ACE unload ({backend.backend_type}): retracting tool T{global_tool} "
                f"(instance {self.ace_instance_index}, slot {local_slot}) "
                f"for lane {cur_lane.name}"
            )

            # Try backend-specific retract first, fall back to gcode
            if backend.has_direct_retract():
                backend.direct_retract(local_slot)
            else:
                # ACE_RETRACT gcode command works for both backends
                self.gcode.run_script_from_command(
                    f"ACE_RETRACT INDEX={local_slot}"
                )

            cur_lane.loaded_to_hub = True
            cur_lane.set_tool_unloaded()
            cur_lane.status = AFCLaneState.LOADED
            self.lane_tool_unloaded(cur_lane)
            afc.save_vars()

        except Exception as e:
            message = f"ACE unload failed for {cur_lane.name}: {e}"
            afc.error.handle_lane_failure(cur_lane, message)
            return False

        return True

    def prep_load(self, lane):
        """No-op for ACE: hardware drives filament to sensors directly."""
        pass

    def prep_post_load(self, lane):
        """No-op for ACE: hardware handles loading internally."""
        pass

    def eject_lane(self, lane):
        """ACE units don't support stepper-based lane ejection."""
        lane_name = getattr(lane, "name", "unknown")
        message = (
            f"LANE_UNLOAD is not supported for ACE lane {lane_name}. "
            "ACE units handle filament automatically - just remove the spool physically. "
            "Use TOOL_UNLOAD if you need to unload from the toolhead."
        )
        self.logger.info(message)
        try:
            self.gcode.respond_info(message)
        except Exception:
            pass

    def lane_unload(self, cur_lane):
        """Block manual LANE_UNLOAD for ACE lanes."""
        self.eject_lane(cur_lane)
        return None

    def get_lane_reset_command(self, lane, dis) -> str:
        """ACE units use TOOL_UNLOAD for lane reset."""
        return f"TOOL_UNLOAD LANE={lane.name}"

    def system_Test(self, cur_lane, delay, assignTcmd, enable_movement):
        """Validate ACE lane state during PREP without attempting motion."""
        msg = ""
        succeeded = True

        cur_lane.unsync_to_extruder(False)
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.7)

        backend = self._get_backend()

        if backend is not None:
            local_slot = self._get_local_slot_for_lane(cur_lane)
            if 0 <= local_slot < self.SLOTS_PER_UNIT:
                slot_info = backend.get_slot_info(local_slot)
                if slot_info:
                    # Sync slot loaded state to loaded_to_hub so virtual
                    # prep_state/load_state properties reflect ACE hardware
                    slot_ready = slot_info.get("status", "") == "ready"
                    cur_lane.loaded_to_hub = slot_ready

                    # Sync RFID/spool data to AFC lane if available
                    material = slot_info.get("material", "")
                    color = slot_info.get("color", [0, 0, 0])
                    if material and hasattr(cur_lane, "material"):
                        cur_lane.material = material
                    if color and hasattr(cur_lane, "color"):
                        cur_lane.color = self._ace_color_to_hex(color)

        if not cur_lane.prep_state:
            if not cur_lane.load_state:
                self.lane_not_ready(cur_lane)
                msg += '<span class=success--text>EMPTY READY FOR SPOOL</span>'
            else:
                self.lane_fault(cur_lane)
                msg += '<span class=error--text>CHECK FILAMENT Prep: False - Load: True</span>'
                cur_lane.do_enable(False)
                succeeded = False
        else:
            self.lane_loaded(cur_lane)
            msg += '<span class=success--text>LOCKED</span>'
            if not cur_lane.load_state:
                msg += '<span class=error--text> NOT LOADED</span>'
                self.lane_not_ready(cur_lane)
                succeeded = False
            else:
                cur_lane.status = AFCLaneState.LOADED
                msg += '<span class=success--text> AND LOADED</span>'
                self.lane_illuminate_spool(cur_lane)

                if cur_lane.tool_loaded:
                    tool_ready = (
                        cur_lane.get_toolhead_pre_sensor_state()
                        or cur_lane.extruder_obj.tool_start == "buffer"
                        or cur_lane.extruder_obj.tool_end_state
                    )
                    if tool_ready and cur_lane.extruder_obj.lane_loaded == cur_lane.name:
                        cur_lane.sync_to_extruder()
                        msg += '<span class=primary--text> in ToolHead</span>'
                        if cur_lane.extruder_obj.tool_start == "buffer":
                            msg += '<span class=warning--text> Ram sensor enabled, confirm tool is loaded</span>'
                        if self.afc.function.get_current_lane() == cur_lane.name:
                            self.afc.spool.set_active_spool(cur_lane.spool_id)
                            cur_lane.unit_obj.lane_tool_loaded(cur_lane)
                            cur_lane.status = AFCLaneState.TOOLED
                        else:
                            cur_lane.unit_obj.lane_tool_loaded_idle(cur_lane)
                    elif tool_ready:
                        msg += (
                            '<span class=error--text> error in ToolHead. '
                            'Lane identified as loaded but not identified as loaded in extruder</span>'
                        )
                        succeeded = False

        if assignTcmd:
            self.afc.function.TcmdAssign(cur_lane)
        cur_lane.do_enable(False)
        self.logger.info(
            '{lane_name} tool cmd: {tcmd:3} {msg}'.format(
                lane_name=cur_lane.name, tcmd=cur_lane.map, msg=msg
            )
        )
        cur_lane.set_afc_prep_done()
        return succeeded

    def calibrate_bowden(self, cur_lane, dis, tol):
        """ACE units do not use standard AFC bowden calibration."""
        msg = (
            "ACE units do not support standard AFC bowden calibration. "
            "ACE manages tube lengths via its own configuration:\n"
            "  - parkposition_to_toolhead_length in [ace] config section\n"
            "  - toolchange_load_length in [ace] config section"
        )
        return False, msg, 0

    def calibrate_hub(self, cur_lane, tol):
        """ACE units do not use hub calibration."""
        return False, "ACE units do not support hub calibration.", 0

    def calibrate_lane(self, cur_lane, tol):
        """ACE units do not use lane calibration."""
        return False, "ACE units do not support lane calibration.", 0

    @staticmethod
    def _ace_color_to_hex(color_array):
        """Convert ACE RGB array [r, g, b] to hex color string."""
        if isinstance(color_array, (list, tuple)) and len(color_array) >= 3:
            r, g, b = int(color_array[0]), int(color_array[1]), int(color_array[2])
            return f"#{r:02x}{g:02x}{b:02x}"
        return "#000000"

    def sync_ace_inventory(self):
        """Sync RFID/spool inventory data and slot loaded state from ACE hardware to AFC lanes."""
        backend = self._get_backend()
        if backend is None:
            return

        for lane in self.lanes.values():
            local_slot = self._get_local_slot_for_lane(lane)
            if 0 <= local_slot < self.SLOTS_PER_UNIT:
                slot_info = backend.get_slot_info(local_slot)
                if slot_info:
                    # Sync slot loaded state for virtual sensors
                    lane.loaded_to_hub = slot_info.get("status", "") == "ready"

                    material = slot_info.get("material", "")
                    color = slot_info.get("color", [0, 0, 0])
                    if material and hasattr(lane, "material"):
                        lane.material = material
                    if color and hasattr(lane, "color"):
                        lane.color = self._ace_color_to_hex(color)

    # ---- Runout / Infinite Spool Integration ----

    def check_runout(self, cur_lane):
        """Override: ACE supports runout detection during printing."""
        try:
            return self.afc.function.is_printing()
        except Exception:
            return False

    def _get_lane_for_global_tool(self, global_tool):
        """Map an ACEPRO global tool index back to the AFC lane on this unit.

        Returns the lane object or None.
        """
        local_slot = global_tool - (self.ace_instance_index * self.SLOTS_PER_UNIT)
        if local_slot < 0 or local_slot >= self.SLOTS_PER_UNIT:
            return None
        for lane in self.lanes.values():
            if self._get_local_slot_for_lane(lane) == local_slot:
                return lane
        return None

    def _hook_acepro_runout_monitor(self):
        """Hook ACEPRO's RunoutMonitor to route runout events through AFC.

        ACEPRO's RunoutMonitor detects filament runout via the toolhead sensor.
        By default it handles the swap itself (pause ? tool_change ? resume).
        When AFC is managing the lanes, we intercept the runout event and let
        AFC's infinite spool system handle the unload/load/resume sequence.

        This is the same pattern OpenAMS uses: hardware detects the runout,
        AFC handles the swap.
        """
        ace_obj = self._cached_ace_obj
        if ace_obj is None:
            return

        # ACEPRO manager has a runout_monitor attribute
        monitor = getattr(ace_obj, "runout_monitor", None)
        if monitor is None:
            self.logger.debug(
                f"ACE unit {self.name}: no runout_monitor found on ACE manager, "
                "skipping ACEPRO runout hook"
            )
            return

        # Save original handler for potential fallback
        original_handler = getattr(monitor, "_handle_runout_detected", None)
        if original_handler is None:
            return

        # Already hooked by another afcACE instance
        if getattr(monitor, "_afc_runout_hooked", False):
            self.logger.debug(f"ACE unit {self.name}: runout monitor already hooked")
            return

        # Build a map of all afcACE units for tool?lane resolution
        # We need this because a single RunoutMonitor covers all ACE instances
        afc_ace_units = []
        for unit in self.afc.units.values():
            if getattr(unit, "type", None) == "ACE":
                afc_ace_units.append(unit)

        unit_ref = self  # capture for closure

        def _afc_handle_runout(tool_index):
            """AFC-aware replacement for RunoutMonitor._handle_runout_detected.

            Maps the ACEPRO tool index to an AFC lane and triggers AFC's
            infinite spool system instead of ACEPRO's own swap logic.
            """
            # Find the AFC lane for this tool
            lane = None
            for ace_unit in afc_ace_units:
                lane = ace_unit._get_lane_for_global_tool(tool_index)
                if lane is not None:
                    break

            if lane is None:
                unit_ref.logger.warning(
                    f"ACE runout on T{tool_index}: no matching AFC lane found, "
                    "falling back to ACEPRO handler"
                )
                return original_handler(tool_index)

            unit_ref.logger.info(
                f"ACE runout detected on T{tool_index} ? lane {lane.name}"
            )

            # Update virtual sensor state: slot is now empty
            lane.loaded_to_hub = False

            # Only trigger runout on the lane actively printing (TOOLED).
            # LOADED lanes are at the hub but not in the toolhead � a slot
            # going empty on a LOADED lane (e.g. user removed spool after a
            # previous infinite-spool swap) must not re-trigger the swap.
            if lane.status != AFCLaneState.TOOLED:
                unit_ref.logger.info(
                    f"ACE runout on {lane.name}: lane status is {lane.status}, "
                    "not TOOLED - ignoring"
                )
                return

            if lane.runout_lane:
                unit_ref.logger.info(
                    f"ACE infinite spool: {lane.name} ? {lane.runout_lane}"
                )
                try:
                    lane._perform_infinite_runout()
                except Exception as e:
                    unit_ref.logger.error(
                        f"ACE infinite spool failed for {lane.name}: {e}\n"
                        f"{traceback.format_exc()}"
                    )
                    # Fall back to pause
                    lane._perform_pause_runout()
                finally:
                    # unload_sequence sets loaded_to_hub = True after retract,
                    # but this lane ran out � clear it so virtual sensors
                    # reflect the empty state and the lane won't false-trigger
                    # runout again after the remap.
                    lane.loaded_to_hub = False
            else:
                unit_ref.logger.info(
                    f"ACE runout on {lane.name}: no runout_lane configured, pausing"
                )
                lane._perform_pause_runout()

        monitor._handle_runout_detected = _afc_handle_runout
        monitor._afc_runout_hooked = True

        # Disable ACEPRO's own endless spool so it doesn't also try to swap
        state = getattr(ace_obj, "state", None)
        if isinstance(state, dict):
            state["ace_endless_spool_enabled"] = False

        self.logger.info(
            f"ACE unit {self.name}: hooked ACEPRO runout monitor for AFC infinite spool"
        )

    def _start_slot_status_monitor(self):
        """Start periodic slot status monitoring for runout detection.

        This is a fallback for DuckACE (which lacks a RunoutMonitor) or when
        the ACEPRO runout hook isn't available. Polls slot status every ~1s
        during printing to detect ready?empty transitions.
        """
        self._prev_slot_states: Dict[str, bool] = {}
        self._slot_monitor_timer = self.afc.reactor.register_timer(
            self._poll_slot_status,
            self.afc.reactor.monotonic() + 5.0
        )
        self.logger.info(
            f"ACE unit {self.name}: started slot status monitor for runout detection"
        )

    def _poll_slot_status(self, eventtime):
        """Periodic callback to detect slot status changes during printing."""
        try:
            if not self.afc.function.is_printing():
                return eventtime + 2.0
        except Exception:
            return eventtime + 2.0

        backend = self._get_backend()
        if backend is None:
            return eventtime + 2.0

        for lane in self.lanes.values():
            local_slot = self._get_local_slot_for_lane(lane)
            if local_slot < 0 or local_slot >= self.SLOTS_PER_UNIT:
                continue

            slot_info = backend.get_slot_info(local_slot)
            slot_ready = bool(slot_info and slot_info.get("status", "") == "ready")
            prev_ready = self._prev_slot_states.get(lane.name, slot_ready)
            self._prev_slot_states[lane.name] = slot_ready

            # Detect ready ? not-ready transition (filament runout).
            # Only act on TOOLED lanes � LOADED lanes are at the hub, not
            # actively printing. After a previous infinite-spool swap the old
            # lane stays LOADED; the user removing that spool later must not
            # re-trigger the runout sequence.
            if prev_ready and not slot_ready:
                if lane.status == AFCLaneState.TOOLED:
                    self.logger.info(
                        f"ACE slot monitor: runout detected on {lane.name} "
                        f"(slot {local_slot} status changed)"
                    )
                    lane.loaded_to_hub = False

                    if lane.runout_lane:
                        try:
                            lane._perform_infinite_runout()
                        except Exception as e:
                            self.logger.error(
                                f"ACE infinite spool failed for {lane.name}: {e}\n"
                                f"{traceback.format_exc()}"
                            )
                            lane._perform_pause_runout()
                        finally:
                            # Clear loaded_to_hub after the swap so the old
                            # lane's virtual sensors reflect empty state and
                            # don't false-trigger on the next poll cycle.
                            lane.loaded_to_hub = False
                    else:
                        lane._perform_pause_runout()
                elif lane.status == AFCLaneState.LOADED:
                    # Slot went empty on a non-active lane � just update
                    # virtual sensor state, no runout action needed.
                    lane.loaded_to_hub = False

        return eventtime + 1.0


def load_config_prefix(config):
    return afcACE(config)