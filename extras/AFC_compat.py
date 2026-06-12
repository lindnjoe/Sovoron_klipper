# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# AFC compatibility shims.
#
# Runtime guards that let our serial-driven units (ACE, OpenAMS) run on the
# FROZEN upstream AFC core without editing upstream files. They re-express
# behaviour our deviated fork had (steppermless [AFC_lane] lanes on virtual
# hubs, two-phase unload) against upstream's classes.
#
# Each shim is idempotent (a flag on the target class) and safe to apply from
# multiple units — whichever loads first wins. Call apply_compat_patches() from
# each unit module AT IMPORT TIME, so it runs before the [AFC_lane ...] sections
# init (standard AFC config has the unit section before its lanes).
#
# Shims:
#   1. AFC_lane     - skip homing-endstop setup on a 'virtual' hub pin
#                     (upstream tries setup_pin('endstop','virtual') -> crash).
#   2. AFC_buffer   - skip fault detection for steppermless [AFC_lane] lanes
#                     (no extruder_stepper -> no movement to measure -> false fault).
#   3. AFC_hub      - virtual-hub state from the driven hub HES (switch_pin_callback),
#                     not the lane F1S/load sensor (any(lane.raw_load_state)).
#   4. AFC.unload_sequence - run the shared toolhead phase (heat/cut/park/form_tip)
#                     before a custom_unload_cmd unload, which upstream skips.

from __future__ import annotations


def _patch_afc_lane_virtual_hub():
    """1. AFC_lane: don't set up a homing endstop on a 'virtual' hub pin."""
    try:
        from extras import AFC_lane as _lane_mod
    except Exception:
        return
    LaneCls = getattr(_lane_mod, 'AFCLane', None)
    if LaneCls is None or getattr(LaneCls, '_afc_virtual_hub_patched', False):
        return
    _orig = LaneCls._set_homing_endstop

    def _guarded(self, query_endstops, ppins, pin, name):
        if pin is not None and str(pin).strip().lower() == "virtual":
            return  # virtual hub has no physical pin; no homing endstop
        return _orig(self, query_endstops, ppins, pin, name)

    LaneCls._set_homing_endstop = _guarded
    LaneCls._afc_virtual_hub_patched = True


def _patch_afc_buffer_steppermless():
    """2. AFC_buffer: skip the fault timer for lanes with no extruder_stepper."""
    try:
        from extras import AFC_buffer as _buf_mod
    except Exception:
        return
    BufCls = getattr(_buf_mod, 'AFCTrigger', None)
    if BufCls is None or getattr(BufCls, '_afc_steppermless_patched', False):
        return
    _timeout = getattr(_buf_mod, 'CHECK_RUNOUT_TIMEOUT', 0.5)
    _orig = BufCls.extruder_pos_update_event

    def _guarded(self, eventtime):
        cur = self.current_lane
        if cur is not None and getattr(cur, 'extruder_stepper', None) is None:
            return eventtime + _timeout  # no stepper to track; skip fault check
        return _orig(self, eventtime)

    BufCls.extruder_pos_update_event = _guarded
    BufCls._afc_steppermless_patched = True


def _patch_afc_hub_virtual_state():
    """3. AFC_hub: a driven virtual hub reports _state (hub HES), not F1S."""
    try:
        from extras import AFC_hub as _hub_mod
    except Exception:
        return
    HubCls = getattr(_hub_mod, 'afc_hub', None)
    if HubCls is None or getattr(HubCls, '_afc_virtual_state_patched', False):
        return
    _orig_prop = HubCls.state
    _orig_cb = HubCls.switch_pin_callback

    def _state_getter(self):
        if self.is_virtual_pin() and getattr(self, '_state_driven', False):
            return bool(self._state)
        return _orig_prop.fget(self)

    def _cb(self, eventtime, state):
        self._state_driven = True
        return _orig_cb(self, eventtime, state)

    HubCls.state = property(_state_getter)
    HubCls.switch_pin_callback = _cb
    HubCls._afc_virtual_state_patched = True


def _patch_afc_unload_shared_phase():
    """4. AFC.load_sequence / unload_sequence: run the shared toolhead ops that
    upstream's custom_load_cmd / custom_unload_cmd branches skip.

    Upstream treats a custom command as a complete replacement: the unload
    branch skips heat/cut/park/form_tip (+post_unload_macro), and the load
    branch skips _check_extruder_temp. Our serial units (ACE/OpenAMS) set those
    commands to just the hardware transport and rely on the shared phase (as our
    two-phase load/unload did) — otherwise the transport / post-load purge hits
    a cold extruder and park/form_tip/exit-bin never fire."""
    try:
        from extras import AFC as _afc_mod
    except Exception:
        return
    AFCcls = getattr(_afc_mod, 'afc', None)
    if AFCcls is None or getattr(AFCcls, '_afc_custom_unload_phase_patched', False):
        return
    _orig_unload = AFCcls.unload_sequence
    _orig_load = AFCcls.load_sequence

    def _afc_shared_toolhead_unload(self, cur_lane, cur_extruder):
        # lane_unloading: LED + (ACE/OpenAMS) follower/assist stop.
        cur_lane.unit_obj.lane_unloading(cur_lane)
        if self._check_extruder_temp(cur_lane):
            self.afcDeltaTime.log_with_time("Done heating toolhead")
        self.move_e_pos(-2, cur_extruder.tool_unload_speed, "Quick Pull",
                        wait_tool=False)
        cur_lane.disable_buffer()
        cur_lane.sync_to_extruder()
        cur_lane.select_lane()
        if self.tool_cut:
            cur_lane.extruder_obj.estats.increase_cut_total()
            self.gcode.run_script_from_command(
                "{} EXTRUDER={}".format(self.tool_cut_cmd, cur_extruder.name))
            if self.park:
                self.gcode.run_script_from_command(
                    "{} EXTRUDER={}".format(self.park_cmd, cur_extruder.name))
        if self.form_tip:
            if self.park:
                self.gcode.run_script_from_command(
                    "{} EXTRUDER={}".format(self.park_cmd, cur_extruder.name))
            # Feed the follower FORWARD during the form-tip purge so the spool
            # supplies it (units that support it, e.g. OpenAMS); no-op for units
            # without a follower (e.g. ACE). Stopped again right after, before
            # the hardware unload reverses.
            _ff = getattr(cur_lane.unit_obj, 'form_tip_follower_feed', None)
            if _ff is not None:
                _ff(cur_lane, True)
            try:
                if self.form_tip_cmd == "AFC":
                    self.printer.lookup_object('AFC_form_tip').tip_form()
                else:
                    self.gcode.run_script_from_command(self.form_tip_cmd)
            finally:
                if _ff is not None:
                    _ff(cur_lane, False)

    def _wrapped_unload(self, cur_lane, cur_hub, cur_extruder):
        is_custom = bool(cur_lane.custom_unload_cmd)
        if is_custom:
            try:
                self._afc_shared_toolhead_unload(cur_lane, cur_extruder)
            except Exception as e:
                self.logger.error(
                    "AFC shared toolhead unload phase error: %s" % e)
        result = _orig_unload(self, cur_lane, cur_hub, cur_extruder)
        # Upstream runs post_unload_macro only in the stepper branch, so a custom
        # unload skips it (e.g. SNAPMAKER_EXIT_DISCARD_BIN). Run it here, after
        # the transport. (The load side already runs post_load_macro for both.)
        if is_custom and self.post_unload_macro is not None:
            try:
                self.gcode.run_script_from_command(self.post_unload_macro)
            except Exception as e:
                self.logger.error("AFC post_unload_macro error: %s" % e)
        return result

    def _wrapped_load(self, cur_lane, cur_hub, cur_extruder):
        # Upstream's custom_load_cmd branch skips _check_extruder_temp (only the
        # stepper branch heats), so the post-load poop/purge runs cold. Heat
        # (and wait) here before the custom transport.
        if cur_lane.custom_load_cmd:
            try:
                if self._check_extruder_temp(cur_lane):
                    self.afcDeltaTime.log_with_time("Done heating toolhead")
            except Exception as e:
                self.logger.error("AFC custom-load heat error: %s" % e)
        return _orig_load(self, cur_lane, cur_hub, cur_extruder)

    AFCcls._afc_shared_toolhead_unload = _afc_shared_toolhead_unload
    AFCcls.unload_sequence = _wrapped_unload
    AFCcls.load_sequence = _wrapped_load
    AFCcls._afc_custom_unload_phase_patched = True


def apply_compat_patches():
    """Apply all AFC compatibility shims. Idempotent; safe to call repeatedly
    and from multiple unit modules."""
    _patch_afc_lane_virtual_hub()
    _patch_afc_buffer_steppermless()
    _patch_afc_hub_virtual_state()
    _patch_afc_unload_shared_phase()
