# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# AFC Power Loss Recovery (PLR)
#
# Periodically saves print state to disk during printing. After power
# loss, AFC_PLR_RESUME restores the print from the last checkpoint.
#
# Z safety: two Z values are tracked — layer_z (the actual printing
# height) and current_z (which may include a Z-hop). On resume,
# layer_z is used so the nozzle is always at or above the print
# surface, never below.
#
# Compatible with standard Klipper and Kalico.
#
# ── GCode Commands ──────────────────────────────────────────────────
#
#   AFC_PLR_RESUME  [Z_HOME=1]
#       Full power loss resume sequence. Reads saved state, homes XY,
#       restores Z, heats, primes, and resumes printing. By default Z is
#       restored from the saved layer height (no Z homing). Z_HOME=1 instead
#       physically re-homes Z at the safe corner (z_home_x/y or z_home_gcode)
#       and applies z_home_offset — use it when the saved Z can't be trusted.
#
#   AFC_PLR_SAVE
#       Manual checkpoint save (for testing/debugging).
#
#   AFC_PLR_CLEAR
#       Clear saved PLR state without resuming.
#
#   AFC_PLR_STATUS
#       Show saved PLR state info (file, position, age).
#
#   AFC_PLR_CALIBRATE_ZHOME  [SAVE=1] [APPLY=1] [CLEAR_MESH=1]
#       Calibrate the corner-vs-center Z offset (z_home_offset). See the
#       "Z-offset calibration" section below for how it works and example
#       configs for Cartographer and a normal Klipper probe.
#
# ── Configuration ───────────────────────────────────────────────────
#
#   [AFC_PLR]
#   enabled: True                    # Enable/disable PLR
#   save_interval: 5                 # Seconds between periodic checkpoints
#                                    # (min 0.1; effective rate capped by
#                                    # z_check_interval below)
#   z_check_interval: 1.0            # Seconds between Z/layer checks; also how
#                                    # often a save is evaluated (min 0.1). For
#                                    # sub-1s checkpoints, lower this too.
#   resume_z_hop: 5.0                # mm to raise Z during resume
#   pre_resume_purge_length: 30      # mm of filament to purge on resume
#   pre_resume_purge_speed: 3        # mm/s purge extrusion speed
#   save_file: <auto>                # Path to state file (auto = printer_data
#                                    # root, OUTSIDE the config dir so frequent
#                                    # rewrites don't clutter Mainsail). A
#                                    # '.static' companion holds the bed mesh
#
# ── Exclude-object recovery ──────────────────────────────────────────
#
# exclude_object state (the per-object definitions plus which objects the
# user cancelled) lives only in RAM and is wiped by the restart's
# reset_file. Without recovery, a resumed print re-prints every cancelled
# object. PLR snapshots the exclude_object status each checkpoint and, on
# resume, re-issues EXCLUDE_OBJECT_DEFINE + EXCLUDE_OBJECT after the file is
# selected (M23) and before the print is started (M24), so cancellations
# survive a power loss. No configuration needed; inert if [exclude_object]
# is not loaded. Uses stock Klipper exclude_object — no custom module.
#
# ── Z-home resume (homing_override) ──────────────────────────────────
#
# AFC_PLR_RESUME Z_HOME=1 physically re-homes Z instead of trusting the saved
# Z. For probes that home by touching the bed, it moves to a safe corner
# (z_home_x / z_home_y, clear of the print) and runs `G28 Z` with get_status
# z_home_active=True — so your homing_override must touch THERE, not at center,
# or a blind G28 Z would drive into the print. The override can read from
# printer['AFC_PLR']: z_home_active, z_home_x, z_home_y, mesh_zero_x/y.
#
#   Cartographer / eddy:  in your [homing_override] gcode, inside the Z
#   branch, move to the safe corner when a Z-home resume is active (else the
#   normal center), then run G28 Z — with cartographer as the Z endstop that
#   touches in place at the current XY. (Inside a homing_override, G28 Z runs
#   the real homing, not the override again, so there's no recursion.)
#       {% if z %}
#           G90
#           {% set plr = printer['AFC_PLR'] %}
#           {% if plr.z_home_active %}
#             G0 X{plr.z_home_x} Y{plr.z_home_y} F12000   ; safe corner
#           {% else %}
#             G0 X175 Y175 F12000                         ; normal center
#           {% endif %}
#           G28 Z
#           G0 Z10 F3000
#       {% endif %}
#
#   Regular probe that homes to the bed (BLTouch / inductive / load cell):
#   move to the safe corner before the probe touch when active, else center:
#       {% set plr = printer['AFC_PLR'] %}
#       {% if plr.z_home_active %}
#           G0 X{plr.z_home_x} Y{plr.z_home_y} F6000     ; safe corner
#       {% else %}
#           G0 X150 Y150 F6000                           ; normal center
#       {% endif %}
#       G28.1 Z                                          ; the real Z home (probe)
#       G0 Z10 F600
#
#   Plain Z endstop that homes AWAY from the bed (Z toward max): no override
#   change needed — just set z_home_standard: True. A blind G28 Z is safe
#   there (it lifts away from the print); AFC refuses it if [stepper_z] homes
#   toward the bed.
#
# ── Z-offset calibration (AFC_PLR_CALIBRATE_ZHOME) ───────────────────
#
# A Z-home resume re-homes Z at a safe corner (z_home_x / z_home_y) instead
# of bed center, so the consistent corner-vs-center bed-flatness delta must
# be applied on every resume as z_home_offset. Calibrate that value once.
# AFC_PLR_CALIBRATE_ZHOME:
#
#   1. Anchors Z=0 at the normal center reference  (z_home_calibrate_anchor)
#   2. Moves to the safe corner                    (z_home_x / z_home_y)
#   3. Probes IN PLACE there and reads the touch Z (z_home_calibrate_gcode)
#   4. Reports z_home_offset = -(corner touch Z);  SAVE=1 persists it
#
# The touch Z is parsed from the probe's console output, so any probe whose
# command prints its result works — Cartographer ("estimate contact at z=")
# or a normal Klipper PROBE ("... is z="). The probe command MUST touch at
# the current XY (the corner); it must not re-home to center. Use the SAME
# z_home_x / z_home_y you use for the resume so the measured delta matches.
#
#   Cartographer touch:
#     [AFC_PLR]
#     z_home_x: 5                            # safe corner the touch can reach
#     z_home_y: 5
#     z_home_offset: 0.0                     # written by SAVE=1
#     z_home_calibrate_anchor: CARTOGRAPHER_TOUCH_HOME   # center touch = Z=0 ref
#     z_home_calibrate_gcode: CARTOGRAPHER_TOUCH_PROBE   # touches in place
#
#   Normal Klipper probe:
#     [AFC_PLR]
#     z_home_x: 30                           # safe spot the probe can reach
#     z_home_y: 30
#     z_home_offset: 0.0                     # written by SAVE=1
#     z_home_calibrate_anchor: G28 Z         # center Z home (safe_z_home/probe)
#     z_home_calibrate_gcode: PROBE          # touches in place, "... is z="
#
#   Then run:  AFC_PLR_CALIBRATE_ZHOME SAVE=1
#   SAVE persists z_home_offset: it rewrites the existing z_home_offset line in
#   your config if present, otherwise AFC stores it in AFC_auto_vars.cfg
#   (merges on restart). APPLY=1 sets it for this session only.
#
#   CLEAR_MESH (default 1) runs BED_MESH_CLEAR first so the touches read the
#   raw bed, not mesh-compensated values — leave it on for an accurate delta.
#   It does not reload the mesh afterward (your next print/PRINT_START will).

from __future__ import annotations

import json
import logging
import os
import re
import tempfile
import threading
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    pass


Z_CHANGE_THRESHOLD = 0.01
Z_STABLE_TICKS_FOR_LAYER = 3


class AFCPLR:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()
        self.logger = logging.getLogger('AFC_PLR')

        self.enabled = config.getboolean('enabled', True)
        # Both floors are low so solid-state storage can checkpoint tightly.
        # Saves are evaluated on the Z-check timer, so the effective rate is
        # max(save_interval, z_check_interval) — lower both for sub-1s saves.
        self.save_interval = config.getfloat('save_interval', 5.0, minval=0.1)
        self.z_check_interval = config.getfloat('z_check_interval', 1.0, minval=0.1)
        self.resume_z_hop = config.getfloat('resume_z_hop', 5.0, minval=0.0)
        self.purge_length = config.getfloat('pre_resume_purge_length', 30.0, minval=0.0)
        self.purge_speed = config.getfloat('pre_resume_purge_speed', 3.0, minval=0.1)
        self.double_home = config.getboolean('double_home', False)
        # "Resume - Z Home" recovery: instead of trusting the saved Z
        # (SET_KINEMATIC_POSITION), physically re-home Z at a safe spot so we
        # never drive a fixed G28 Z into the existing print. Pick ONE backend:
        #
        #   probe that homes in/at a point (cartographer/beacon eddy, BLTouch,
        #     inductive): set z_home_x / z_home_y to a safe corner clear of the
        #     print. During a Z-home resume AFC_PLR sets get_status z_home_active
        #     True and runs `G28 Z`; your homing_override reads printer['AFC_PLR']
        #     to home at that corner instead of center. Not eddy-specific — any
        #     probe that can measure at the corner works. Override snippet:
        #
        #       [homing_override]
        #       gcode:
        #           ... (your X/Y homing) ...
        #           {% if 'Z' in params or (... home_all ...) %}
        #             {% set plr = printer['AFC_PLR'] %}
        #             {% if plr.z_home_active %}
        #               G0 X{plr.z_home_x} Y{plr.z_home_y} F6000   ; safe corner
        #             {% else %}
        #               G0 X150 Y150 F6000                         ; normal center
        #             {% endif %}
        #             G28.1 Z          ; the real Z homing (probe / BLTouch)
        #             G0 Z10 F600
        #           {% endif %}
        #
        #   plain Z endstop, no probe: set z_home_standard: True (below). Used
        #     only when Z homes away from the bed; checked automatically.
        #
        #   fully custom: set z_home_gcode to a macro that does the whole safe
        #     re-home itself (escape hatch). Takes precedence over the others.
        self.z_home_gcode = config.get('z_home_gcode', '').strip()
        self.z_home_x = config.getfloat('z_home_x', None)
        self.z_home_y = config.getfloat('z_home_y', None)
        # Fixed Z trim applied every Z-home resume via SET_GCODE_OFFSET, to
        # absorb the bed-flatness delta between the safe touch corner and the
        # mesh zero-reference. Dial in once by live-babystepping a resume.
        self.z_home_offset = config.getfloat('z_home_offset', 0.0)
        # Fixed XY trim applied on every resume via SET_GCODE_OFFSET, to absorb
        # the small consistent shift between the original print's home and the
        # independent re-home done at resume. Dial in by editing and re-testing.
        self.resume_x_offset = config.getfloat('resume_x_offset', 0.0)
        self.resume_y_offset = config.getfloat('resume_y_offset', 0.0)
        # AFC_PLR_CALIBRATE_ZHOME helpers. The anchor establishes Z=0 at the
        # normal (center) reference; the probe touches in place and prints its
        # result, which we capture from the console (cartographer doesn't
        # expose the touch Z via get_status).
        self.z_home_calibrate_anchor = config.get(
            'z_home_calibrate_anchor', 'G28 Z')
        self.z_home_calibrate_gcode = config.get(
            'z_home_calibrate_gcode', '').strip()
        # Clearance to gain before homing XY in Z-home mode: fake the current
        # Z as 0 then raise this much so sensorless XY homing can't drag the
        # nozzle across the print (real Z after a power loss is unknown).
        self.z_home_prelift = config.getfloat('z_home_prelift', 5.0, minval=0.0)
        # After a Z-home touch, re-apply the saved active tool's gcode Z offset
        # so multi-tool resumes land at the right height without per-tool macro
        # logic. Disable if your z_home_gcode already applies the offset.
        self.z_home_apply_tool_offset = config.getboolean(
            'z_home_apply_tool_offset', True)
        # Standard (non-eddy) Z re-home: run a plain `G28 Z` for the Z-home
        # resume, for machines with an ordinary Z endstop and no cartographer/
        # beacon. Only safe when Z homes AWAY from the bed (toward max) — homing
        # toward the bed after a power loss would crash into the print.
        self.z_home_standard = config.getboolean('z_home_standard', False)
        # Z homing direction from [stepper_z]: True if it homes toward max
        # (safe for a blind G28 Z), False if toward the bed (unsafe), None if
        # probe-homed/unknown (G28 Z goes to safe_z_home, not blind in place).
        self._z_homes_to_max = self._detect_z_home_to_max(config)
        # True only while a Z-home resume is mid-flight, so homing_override
        # knows to touch at the safe corner. Read live via get_status.
        self._z_home_active = False
        # Mesh zero-reference (print-area center) read from [bed_mesh], surfaced
        # so the calibration macro can compare corner-vs-center without anything
        # hardcoded.
        self.mesh_zero_x = None
        self.mesh_zero_y = None
        try:
            zrp = config.getsection('bed_mesh').get(
                'zero_reference_position', None)
            if zrp:
                coords = [float(v.strip()) for v in zrp.split(',')]
                if len(coords) >= 2:
                    self.mesh_zero_x, self.mesh_zero_y = coords[0], coords[1]
        except Exception:
            pass
        # Whether the Z-home recovery path is usable at all (custom macro,
        # configured safe corner, or standard endstop homing).
        self._z_home_available = bool(self.z_home_gcode) or (
            self.z_home_x is not None and self.z_home_y is not None) \
            or self.z_home_standard
        save_file = config.get('save_file', '')
        self._config_save_file = save_file

        self.layer_z = 0.0
        self._pending_layer_z = None
        self._z_up_ticks = 0
        self._last_save_time = 0.0
        self._timer = None
        self._has_saved_state = False
        # Cache the serialized bed mesh — it's static during a print, so we
        # avoid re-reading 900+ probe points and rebuilding the dict on every
        # checkpoint. Keyed on the ZMesh object identity; refreshed only if the
        # mesh actually changes.
        self._mesh_cache_obj = None
        self._mesh_cache_data = None
        # Names of the exclude_object definitions last written to the static
        # file. The geometry is set once when the file header plays (a beat
        # after tracking starts), so we rewrite the static file when the set
        # of defined objects changes — same pattern as the mesh cache.
        self._exclude_names_cache = None
        # The static (bed mesh + exclude-object geometry) data is written to a
        # companion file only when it changes, not every cycle.
        self.static_file = None
        self._static_dirty = False

        self._sd = None
        self._gcode_move = None
        self._afc = None
        self._print_stats = None
        self._heater_bed = None
        self.save_file = ''
        self._pending_recovery = None
        self._prompt_timer = None
        self._in_shutdown = False

        # Background writer: the periodic save's json.dump + fsync + rename
        # must not run on the reactor thread (an SD fsync can stall motion).
        # The reactor only gathers state (which must read live, non-thread-
        # safe Klipper objects on-thread) and hands the plain dict to this
        # thread, mirroring how AFC_logger offloads disk I/O via QueueListener.
        self._writer_thread = None
        self._writer_cond = threading.Condition()
        self._writer_pending = None   # latest (state, path) awaiting write
        self._writer_busy = False     # True while a disk write is in progress
        self._writer_stop = False

        if not self.enabled:
            return

        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        self.printer.register_event_handler("klippy:disconnect", self._handle_disconnect)
        self.printer.register_event_handler(
            "virtual_sdcard:reset_file", self._handle_reset_file)

        self.gcode.register_command(
            'AFC_PLR_RESUME', self.cmd_PLR_RESUME,
            desc='Resume print after power loss')
        self.gcode.register_command(
            'AFC_PLR_SAVE', self.cmd_PLR_SAVE,
            desc='Manual PLR checkpoint save')
        self.gcode.register_command(
            'AFC_PLR_CLEAR', self.cmd_PLR_CLEAR,
            desc='Clear saved PLR state')
        self.gcode.register_command(
            'AFC_PLR_STATUS', self.cmd_PLR_STATUS,
            desc='Show PLR state info')
        self.gcode.register_command(
            'AFC_PLR_TEST_ZHOME', self.cmd_PLR_TEST_ZHOME,
            desc='Dry-run the Z-home recovery sequence (no checkpoint/resume)')
        self.gcode.register_command(
            'AFC_PLR_CALIBRATE_ZHOME', self.cmd_PLR_CALIBRATE_ZHOME,
            desc='Measure corner-vs-center touch delta and report z_home_offset')

    def _detect_z_home_to_max(self, config):
        # Inspect [stepper_z] to decide whether a blind `G28 Z` is safe after a
        # power loss. Returns True if Z homes toward max (away from the bed),
        # False if toward the bed, None if probe-homed/unknown.
        try:
            zc = config.getsection('stepper_z')
        except Exception:
            return None
        endstop = (zc.get('endstop_pin', '') or '').lower()
        if 'virtual_endstop' in endstop:
            # Probe-homed (cartographer/BLTouch/inductive): G28 Z goes to
            # safe_z_home, not blindly in place — handled by the probe paths.
            return None
        pe = zc.getfloat('position_endstop', None)
        if pe is None:
            return None
        pmin = zc.getfloat('position_min', 0.0)
        pmax = zc.getfloat('position_max', None)
        if pmax is None:
            return None
        # Homes toward whichever travel limit the endstop sits nearest.
        return abs(pe - pmax) <= abs(pe - pmin)

    # ── Lifecycle ───────────────────────────────────────────────────

    def _handle_ready(self):
        self._sd = self.printer.lookup_object('virtual_sdcard', None)
        self._gcode_move = self.printer.lookup_object('gcode_move')
        self._afc = self.printer.lookup_object('AFC', None)
        self._print_stats = self.printer.lookup_object('print_stats', None)
        self._heater_bed = self.printer.lookup_object('heater_bed', None)

        # Resolve the state-file location. By default keep it OUT of the
        # Moonraker-watched config tree. PLR checkpoints are rewritten many
        # times a second via atomic rename; doing that inside config/ spams
        # Moonraker's file watcher, which makes Mainsail's file manager render
        # dozens of phantom duplicate rows of the one file (same name, "select
        # one selects all") that won't delete, and a temp file left by a
        # power-cut-mid-write clutters it too. The printer_data root is
        # persistent but not a browsable file root, so none of that reaches
        # Mainsail. A custom save_file: is honoured verbatim.
        if self._config_save_file:
            self.save_file = os.path.expanduser(self._config_save_file)
        else:
            self.save_file = os.path.join(
                self._printer_data_root(), 'afc_plr', 'AFC_PLR_state.json')

        # Companion file holding the static, write-once data (bed mesh). The
        # frequent dynamic checkpoint omits it so periodic saves stay tiny.
        base, ext = os.path.splitext(self.save_file)
        self.static_file = base + '.static' + ext

        # Remove orphaned temp files (plr_*.tmp from a power cut mid-write).
        self._cleanup_temp_orphans(os.path.dirname(self.save_file))

        # The dynamic + static files are a pair (static now holds file_path).
        # If only one survives the checkpoint is incomplete/unresumable, so
        # discard the orphan rather than offer a broken recovery or leave junk.
        if os.path.exists(self.save_file) != os.path.exists(self.static_file):
            for p in (self.save_file, self.static_file):
                try:
                    if os.path.exists(p):
                        os.unlink(p)
                except OSError:
                    pass

        self._has_saved_state = os.path.exists(self.save_file)
        if self._has_saved_state:
            state = self._load_state()
            if state:
                age = time.time() - state.get('timestamp', 0)
                fname = state.get('file_name', '?')
                self.gcode.respond_info(
                    "AFC_PLR: Saved print state found (%s, %.0fs ago). "
                    "Use AFC_PLR_RESUME to continue or AFC_PLR_CLEAR to discard."
                    % (fname, age))
                self._pending_recovery = state
                self._prompt_timer = self.reactor.register_timer(
                    self._show_recovery_prompt,
                    self.reactor.monotonic() + 10.0)

        self._timer = self.reactor.register_timer(
            self._timer_cb, self.reactor.NEVER)
        self._idle_check = self.reactor.register_timer(
            self._idle_check_cb,
            self.reactor.monotonic() + 5.0)

    def _idle_check_cb(self, eventtime):
        if self._is_printing() and self._last_save_time == 0:
            self._start_tracking()
        return eventtime + 2.0

    def _show_recovery_prompt(self, eventtime):
        state = self._pending_recovery
        if state is None:
            return self.reactor.NEVER
        self._pending_recovery = None
        age = time.time() - state.get('timestamp', 0)
        fname = state.get('file_name', '?')
        layer_z = state.get('layer_z', 0)
        ext = state.get('active_extruder', '?')
        respond = self.gcode.respond_raw
        respond("// action:prompt_begin Power Loss Recovery")
        respond("// action:prompt_text A saved print state was found.")
        respond("// action:prompt_text ")
        respond("// action:prompt_text File: %s" % fname)
        respond("// action:prompt_text Layer Z: %.2f mm" % layer_z)
        respond("// action:prompt_text Extruder: %s" % ext)
        respond("// action:prompt_text Age: %.0f seconds" % age)
        respond("// action:prompt_text ")
        respond("// action:prompt_text Resume this print?")
        respond("// action:prompt_footer_button Resume|AFC_PLR_RESUME|primary")
        # Offer the Z-home variant only when a re-home method is configured.
        if self._z_home_available:
            respond("// action:prompt_footer_button "
                    "Resume - Z Home|AFC_PLR_RESUME Z_HOME=1|secondary")
        respond("// action:prompt_footer_button Discard|AFC_PLR_CLEAR|error")
        respond("// action:prompt_show")
        return self.reactor.NEVER

    def _close_prompt(self):
        self.gcode.respond_raw("// action:prompt_end")

    def _handle_shutdown(self):
        # A shutdown (e.g. verify_heater tripping) drops virtual_sdcard /
        # print_stats out of the 'printing' state *before* this handler
        # runs, so gating the save on _is_printing() loses the checkpoint
        # exactly when we need it. _last_save_time > 0 means we were mid
        # print and actively tracking, so save a final checkpoint for PLR
        # to retry after the restart.
        self._in_shutdown = True
        # Halt the periodic timer so it can't fire after the shutdown, see
        # 'not printing', and wipe the checkpoint we're about to write.
        if self._timer is not None:
            self.reactor.update_timer(self._timer, self.reactor.NEVER)
        # Stop the background writer so it can't race (and lose to) the final
        # synchronous save below, then write the last checkpoint inline so it
        # is guaranteed on disk before klippy exits.
        self._stop_writer(join=True)
        if self._is_printing() or self._last_save_time > 0:
            try:
                self._save_state(sync=True)
                self.logger.info("Shutdown save completed")
            except Exception as e:
                self.logger.error("Shutdown save failed: %s" % e)

    def _cleanup(self):
        self._stop_tracking()
        self.layer_z = 0.0
        self._pending_layer_z = None
        self._z_up_ticks = 0
        self._last_save_time = 0.0
        self._pending_recovery = None
        self._clear_state()

    def _handle_reset_file(self):
        # Don't wipe the checkpoint if a reset_file event arrives during a
        # shutdown — that state is being preserved for PLR retry.
        if self._in_shutdown:
            return
        self._cleanup()

    # ── Timer ───────────────────────────────────────────────────────

    def _timer_cb(self, eventtime):
        if self._in_shutdown:
            return self.reactor.NEVER
        if not self._is_printing():
            self.logger.info("Print ended, clearing PLR state")
            self._cleanup()
            return self.reactor.NEVER

        current_z = self._get_current_z()
        save_triggered = False

        if abs(current_z - self.layer_z) < Z_CHANGE_THRESHOLD:
            self._z_up_ticks = 0
            self._pending_layer_z = None
        elif current_z > self.layer_z + Z_CHANGE_THRESHOLD:
            if (self._pending_layer_z is None
                    or abs(current_z - self._pending_layer_z) > Z_CHANGE_THRESHOLD):
                self._pending_layer_z = current_z
                self._z_up_ticks = 1
                save_triggered = True
            else:
                self._z_up_ticks += 1
                if self._z_up_ticks >= Z_STABLE_TICKS_FOR_LAYER:
                    self.layer_z = self._pending_layer_z
                    self._pending_layer_z = None
                    self._z_up_ticks = 0
                    save_triggered = True
        elif current_z < self.layer_z - Z_CHANGE_THRESHOLD:
            self.layer_z = current_z
            self._pending_layer_z = None
            self._z_up_ticks = 0
            save_triggered = True

        if save_triggered or (eventtime - self._last_save_time >= self.save_interval):
            try:
                self._save_state()
                self._last_save_time = eventtime
            except Exception as e:
                self.logger.error("Save failed: %s" % e)

        return eventtime + self.z_check_interval

    def _start_tracking(self):
        self.layer_z = self._get_current_z()
        self._pending_layer_z = None
        self._z_up_ticks = 0
        self._mesh_cache_obj = None     # re-capture the mesh for the new print
        self._mesh_cache_data = None
        self._exclude_names_cache = None  # re-capture exclude objects too
        self._static_dirty = True       # write the static file once for this print
        self._last_save_time = self.reactor.monotonic()
        self.reactor.update_timer(
            self._timer, self.reactor.monotonic() + self.z_check_interval)

    def _stop_tracking(self):
        self._last_save_time = 0.0
        self.reactor.update_timer(self._timer, self.reactor.NEVER)

    # ── State queries ───────────────────────────────────────────────

    def _is_printing(self):
        if self._sd is not None and self._sd.is_active():
            return True
        if self._print_stats is not None:
            ps = self._print_stats.get_status(self.reactor.monotonic())
            if ps.get('state') == 'printing':
                return True
        return False

    def _get_current_z(self):
        if self._gcode_move is None:
            return 0.0
        return self._gcode_move.last_position[2]

    # ── Save / Load ─────────────────────────────────────────────────

    def _gather_input_shaper(self, active_ext):
        # Capture the input shaper to restore. Toolchangers store it per tool
        # as params_input_shaper_* on each AFC_extruder (applied by the
        # tool-change macro, lost on a restart), so read the active tool's
        # params first; fall back to the live global [input_shaper] state.
        param_map = {
            'shaper_type_x': 'params_input_shaper_type_x',
            'shaper_freq_x': 'params_input_shaper_freq_x',
            'damping_ratio_x': 'params_input_shaper_damping_ratio_x',
            'shaper_type_y': 'params_input_shaper_type_y',
            'shaper_freq_y': 'params_input_shaper_freq_y',
            'damping_ratio_y': 'params_input_shaper_damping_ratio_y',
        }
        if active_ext:
            for _, ext in self.printer.lookup_objects('AFC_extruder'):
                if getattr(ext, 'name', None) != active_ext:
                    continue
                params = getattr(ext, 'params', None) or {}
                data = {sk: params[pk] for sk, pk in param_map.items()
                        if params.get(pk) is not None}
                if data.get('shaper_type_x') or data.get('shaper_type_y'):
                    return data
                break
        is_obj = self.printer.lookup_object('input_shaper', None)
        if is_obj is not None:
            try:
                iss = is_obj.get_status(self.reactor.monotonic())
                data = {k: iss.get(k) for k in param_map
                        if iss.get(k) is not None}
                if data:
                    return data
            except Exception:
                pass
        return None

    def _gather_state(self):
        state = {}
        state['timestamp'] = time.time()
        state['layer_z'] = self.layer_z
        state['current_z'] = self._get_current_z()

        if self._sd is not None:
            sd_status = self._sd.get_status(self.reactor.monotonic())
            state['file_path'] = sd_status.get('file_path', '')
            state['file_position'] = getattr(self._sd, 'file_position', 0)
            state['file_name'] = os.path.basename(
                sd_status.get('file_path', ''))

        gm = self._gcode_move
        if gm is not None:
            state['gcode_position'] = list(gm.last_position)
            state['base_position'] = list(gm.base_position)
            state['homing_position'] = list(gm.homing_position)
            state['speed'] = gm.speed
            # Raw gcode feedrate (mm/min) with the M220 factor backed out, so
            # it can be re-issued as an F on resume regardless of speed factor.
            # gm.speed (mm/s) == F(mm/min) * gm.speed_factor, so F = speed/factor.
            sf = getattr(gm, 'speed_factor', 0.0)
            state['feed_rate'] = (gm.speed / sf) if sf else 0.0
            gm_status = gm.get_status(self.reactor.monotonic())
            state['speed_factor'] = gm_status.get('speed_factor', 1.0)
            state['extrude_factor'] = gm_status.get('extrude_factor', 1.0)
            state['absolute_coord'] = gm.absolute_coord
            state['absolute_extrude'] = gm.absolute_extrude

        toolhead = self.printer.lookup_object('toolhead', None)
        if toolhead is not None:
            state['toolhead_position'] = list(toolhead.get_position())
            state['active_extruder'] = toolhead.get_extruder().get_name()

        # Active tool's gcode Z offset, captured now while the toolchanger
        # still knows which tool is mounted. On a Z-home resume we re-apply
        # this after the touch so the resumed tool lands at the right height
        # even though the toolchanger isn't re-initialized after a restart.
        state['active_tool_z_offset'] = 0.0
        for _, tc in self.printer.lookup_objects('AFC_Toolchanger'):
            try:
                tcs = tc.get_status(self.reactor.monotonic())
                if tcs.get('tool') is not None:
                    state['active_tool_z_offset'] = tcs.get(
                        'active_tool_gcode_z_offset', 0.0)
                    state['active_tool'] = tcs.get('tool')
                    break
            except Exception:
                pass

        if self._afc is not None:
            cur_lane = getattr(self._afc, 'current', None)
            state['current_lane'] = cur_lane

        state['extruder_temps'] = {}
        state['pressure_advance'] = {}
        state['pa_smooth_time'] = {}
        pheaters = self.printer.lookup_object('heaters', None)
        if pheaters is not None:
            for name in pheaters.available_heaters:
                heater = pheaters.lookup_heater(name)
                state['extruder_temps'][name] = heater.target_temp
                if name.startswith('extruder'):
                    ext = self.printer.lookup_object(name, None)
                    if ext is not None and hasattr(ext, 'get_status'):
                        es = ext.get_status(self.reactor.monotonic())
                        state['pressure_advance'][name] = es.get(
                            'pressure_advance', 0.0)
                        st = es.get('smooth_time', None)
                        if st is not None:
                            state['pa_smooth_time'][name] = st

        # Input shaper is NOT saved — it's a config constant (per-tool
        # params_input_shaper_*), re-derived at resume from active_extruder.

        if self._heater_bed is not None:
            state['bed_temp'] = self._heater_bed.get_status(
                self.reactor.monotonic()).get('target', 0)
        else:
            state['bed_temp'] = state['extruder_temps'].get('heater_bed', 0)

        bed_mesh = self.printer.lookup_object('bed_mesh', None)
        if bed_mesh is not None:
            z_mesh = bed_mesh.get_mesh()
            prev = self._mesh_cache_obj
            if z_mesh is None:
                state['bed_mesh_profile'] = ''
                if prev is not None:
                    self._static_dirty = True   # mesh was cleared
                self._mesh_cache_obj = None
                self._mesh_cache_data = None
            elif z_mesh is prev and self._mesh_cache_data:
                # Unchanged since the last checkpoint — reuse the serialization
                # instead of re-reading the probe matrix every save.
                state.update(self._mesh_cache_data)
            else:
                data = {
                    'bed_mesh_profile': z_mesh.get_profile_name(),
                    'bed_mesh_points': z_mesh.get_probed_matrix(),
                    'bed_mesh_params': dict(z_mesh.get_mesh_params()),
                }
                state.update(data)
                self._mesh_cache_obj = z_mesh
                self._mesh_cache_data = data
                self._static_dirty = True       # mesh appeared / changed

        state['fan_speeds'] = {}
        fan = self.printer.lookup_object('fan', None)
        if fan is not None:
            fs = fan.get_status(self.reactor.monotonic())
            state['fan_speeds']['fan'] = fs.get('speed', 0)
        for i in range(10):
            fname = 'fan_generic fan%d' % i if i > 0 else 'fan_generic fan0'
            fobj = self.printer.lookup_object(fname, None)
            if fobj is None:
                continue
            fs = fobj.get_status(self.reactor.monotonic())
            state['fan_speeds'][fname] = fs.get('speed', 0)

        # Exclude-object state. It lives only in RAM (built from the header's
        # EXCLUDE_OBJECT_DEFINE lines plus live EXCLUDE_OBJECT cancellations)
        # and is wiped on the restart's reset_file, so a resumed print would
        # otherwise re-print every object the user had cancelled. The
        # definitions (geometry) are static; the excluded set and current
        # object are dynamic (the user cancels objects mid-print).
        eo = self.printer.lookup_object('exclude_object', None)
        if eo is not None:
            eos = eo.get_status(self.reactor.monotonic())
            objects = eos.get('objects', []) or []
            state['exclude_objects'] = objects
            state['excluded_objects'] = eos.get('excluded_objects', []) or []
            state['current_object'] = eos.get('current_object')
            # Objects usually appear a beat after tracking starts (the header
            # runs the DEFINE lines), so rewrite the static file when the set
            # of defined names changes.
            names = tuple(o.get('name') for o in objects)
            if names != self._exclude_names_cache:
                self._exclude_names_cache = names
                self._static_dirty = True

        return state

    # Static, write-once-per-print fields split into the companion file so the
    # frequent dynamic checkpoint stays tiny. None of these change once a print
    # starts (the file being printed, and the bed mesh ~900 points).
    _STATIC_KEYS = ('file_path', 'file_name',
                    'bed_mesh_profile', 'bed_mesh_points', 'bed_mesh_params',
                    'exclude_objects')

    def _printer_data_root(self):
        # Derive the printer_data root (one level above 'config') from the AFC
        # vars file path so multi-instance setups resolve correctly; fall back
        # to ~/printer_data.
        afc_var = getattr(self._afc, 'VarFile', '') if self._afc else ''
        if afc_var:
            p = os.path.expanduser(afc_var)
            parts = p.split(os.sep)
            if 'config' in parts:
                root = os.sep.join(parts[:parts.index('config')])
                if root:
                    return root
            return os.path.dirname(p)
        return os.path.expanduser('~/printer_data')

    def _cleanup_temp_orphans(self, directory):
        # Delete leftover plr_*.tmp files (a write interrupted by a power cut
        # before the atomic rename). These are never the live state, so it is
        # always safe to remove them.
        if not directory or not os.path.isdir(directory):
            return
        try:
            names = os.listdir(directory)
        except OSError:
            return
        for name in names:
            if name.startswith('plr_') and name.endswith('.tmp'):
                try:
                    os.unlink(os.path.join(directory, name))
                except OSError:
                    pass

    def _save_state(self, sync=False):
        # Gather must run on the reactor thread (it reads live, non-thread-safe
        # Klipper objects). The slow disk write is then either done inline
        # (sync=True, used at shutdown / manual save where we want the result
        # to land before returning) or handed to the background writer.
        state = self._gather_state()
        static = {k: state.pop(k) for k in self._STATIC_KEYS if k in state}
        # Write the static file (bed mesh) only when it changed — once per
        # print in practice. Done inline (it's rare) and BEFORE the dynamic
        # write so the mesh is durable before anything references it.
        if self._static_dirty and self.static_file:
            try:
                self._write_state_to_disk(static, self.static_file,
                                          is_primary=False)
                self._static_dirty = False
            except Exception as e:
                self.logger.error("PLR static save failed: %s" % e)
        if sync:
            self._write_state_to_disk(state, self.save_file)
        else:
            self._queue_save(state)

    def _write_state_to_disk(self, state, path, is_primary=True):
        dir_path = os.path.dirname(path)
        if dir_path:
            os.makedirs(dir_path, exist_ok=True)
        fd, tmp_path = tempfile.mkstemp(
            dir=dir_path, suffix='.tmp', prefix='plr_')
        try:
            with os.fdopen(fd, 'w') as f:
                json.dump(state, f, indent=2)
                # Flush the file's contents all the way to disk before the
                # rename — without this the write can sit in the OS/SD-card
                # cache and be lost on a hard power cut, leaving no usable
                # checkpoint (or no file at all if it was the first save).
                f.flush()
                os.fsync(f.fileno())
            os.rename(tmp_path, path)
            # fsync the directory so the rename itself is durable across a
            # power loss, not just the file contents.
            if dir_path:
                dir_fd = os.open(dir_path, os.O_RDONLY)
                try:
                    os.fsync(dir_fd)
                finally:
                    os.close(dir_fd)
            if is_primary:
                self._has_saved_state = True
        except Exception:
            try:
                os.unlink(tmp_path)
            except OSError:
                pass
            raise

    # ── Background writer thread ─────────────────────────────────────

    def _ensure_writer(self):
        if self._writer_thread is None:
            self._writer_stop = False
            t = threading.Thread(
                target=self._writer_loop, name='AFC_PLR_writer', daemon=True)
            self._writer_thread = t
            t.start()

    def _writer_loop(self):
        while True:
            with self._writer_cond:
                while self._writer_pending is None and not self._writer_stop:
                    self._writer_cond.wait()
                if self._writer_pending is None and self._writer_stop:
                    return
                job = self._writer_pending
                self._writer_pending = None
                self._writer_busy = True
            try:
                self._write_state_to_disk(job[0], job[1])
            except Exception as e:
                self.logger.error("Background PLR save failed: %s" % e)
            finally:
                with self._writer_cond:
                    self._writer_busy = False
                    self._writer_cond.notify_all()

    def _queue_save(self, state):
        # Latest-wins: only the most recent checkpoint matters, so a pending
        # write is simply replaced rather than queued up behind a slow fsync.
        self._ensure_writer()
        with self._writer_cond:
            self._writer_pending = (state, self.save_file)
            self._writer_cond.notify_all()

    def _flush_writer(self, clear_pending=False, timeout=5.0):
        # Wait until the writer is idle (no pending job, not mid-write) so a
        # background write can't resurrect a file we're about to delete.
        if self._writer_thread is None:
            return
        deadline = time.time() + timeout
        with self._writer_cond:
            if clear_pending:
                self._writer_pending = None
            while self._writer_pending is not None or self._writer_busy:
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                self._writer_cond.wait(remaining)

    def _stop_writer(self, join=True):
        t = self._writer_thread
        if t is None:
            return
        with self._writer_cond:
            self._writer_stop = True
            self._writer_cond.notify_all()
        if join:
            t.join(timeout=5.0)
        self._writer_thread = None
        self._writer_stop = False

    def _handle_disconnect(self):
        self._stop_writer(join=True)

    def _load_state(self):
        try:
            with open(self.save_file, 'r') as f:
                state = json.load(f)
        except (IOError, json.JSONDecodeError) as e:
            self.logger.error("Failed to load PLR state: %s" % e)
            return None
        # Merge the static companion (bed mesh) if present; its absence just
        # means the resume runs without a restored mesh.
        if self.static_file:
            try:
                with open(self.static_file, 'r') as f:
                    for k, v in json.load(f).items():
                        state.setdefault(k, v)
            except (IOError, json.JSONDecodeError):
                pass
        return state

    def _clear_state(self):
        # Drop any queued write and wait for an in-flight one so the file
        # we're deleting doesn't get rewritten right after by the writer.
        self._flush_writer(clear_pending=True)
        for path in (self.save_file, self.static_file):
            try:
                if path and os.path.exists(path):
                    os.unlink(path)
            except OSError as e:
                self.logger.error("Failed to clear PLR state (%s): %s"
                                  % (path, e))
        # Also sweep any temp orphans so a finished/aborted job leaves nothing.
        self._cleanup_temp_orphans(os.path.dirname(self.save_file))
        self._has_saved_state = False

    # ── Resume ──────────────────────────────────────────────────────

    def _run_z_home(self, gcmd):
        # Re-establish a true Z at the safe spot. Custom macro wins if set;
        # otherwise flag z_home_active and run a plain G28 Z, which the
        # homing_override redirects to (z_home_x, z_home_y) via get_status.
        run = self.gcode.run_script_from_command
        if self.z_home_gcode:
            gcmd.respond_info(
                "AFC_PLR: Re-homing Z via z_home_gcode (%s)" % self.z_home_gcode)
            run(self.z_home_gcode)
        elif self.z_home_x is not None and self.z_home_y is not None:
            gcmd.respond_info(
                "AFC_PLR: Re-homing Z (G28 Z) at safe corner (%.1f, %.1f)"
                % (self.z_home_x, self.z_home_y))
            self._z_home_active = True
            try:
                run("G28 Z")
            finally:
                self._z_home_active = False
        elif self.z_home_standard:
            # Standard endstop homing (no eddy probe). Only safe when Z homes
            # away from the bed — otherwise G28 Z drives into the print.
            if self._z_homes_to_max is False:
                raise gcmd.error(
                    "AFC_PLR: z_home_standard refused — [stepper_z] homes Z "
                    "toward the bed, so G28 Z would crash into the print after "
                    "a power loss. Use the normal Resume (trusts the saved Z), "
                    "or set z_home_gcode to a macro that homes Z at a safe spot.")
            if self._z_homes_to_max is None:
                gcmd.respond_info(
                    "AFC_PLR: WARNING — could not confirm Z homes away from the "
                    "bed (probe-homed or unknown). G28 Z may move toward the "
                    "print; watch closely.")
            gcmd.respond_info("AFC_PLR: Re-homing Z via standard G28 Z")
            run("G28 Z")
        else:
            raise gcmd.error(
                "AFC_PLR: Z-home requested but no method is configured "
                "(set z_home_x/z_home_y, z_home_gcode, or z_home_standard)")

    def _format_state_summary(self, state, z_home=None):
        # Render the restore set from a checkpoint. z_home True/False describes
        # the Z handling of an actual resume; None = status preview (mode TBD).
        fmt = lambda d, f: ", ".join(f % (k, v) for k, v in d.items()) or "none"
        if z_home is True:
            z_desc = "physically re-homed (offset %.4f, tool offset %.4f)" % (
                self.z_home_offset, state.get('active_tool_z_offset', 0.0))
        elif z_home is False:
            z_desc = "trust saved layer_z"
        else:
            z_desc = "saved layer_z, or re-homed if 'Resume - Z Home'"
        if state.get('bed_mesh_points') and state.get('bed_mesh_params'):
            mesh_desc = "restored from saved data"
        elif state.get('bed_mesh_profile'):
            mesh_desc = "profile '%s'" % state.get('bed_mesh_profile')
        else:
            mesh_desc = "none saved"
        # Input shaper isn't saved — derive what resume would re-apply from the
        # active tool's config params.
        isd = self._gather_input_shaper(state.get('active_extruder')) or {}
        if isd.get('shaper_type_x') is not None:
            is_desc = "x=%s@%.1f y=%s@%.1f" % (
                isd.get('shaper_type_x'), isd.get('shaper_freq_x', 0),
                isd.get('shaper_type_y'), isd.get('shaper_freq_y', 0))
        else:
            is_desc = "config default"
        feed_rate = state.get('feed_rate', 0) or (state.get('speed', 0) * 60)
        return (
            "  File:            %s @ pos %d\n"
            "  Bed temp:        %.0fC\n"
            "  Extruder temps:  %s\n"
            "  Active extruder: %s\n"
            "  Z:               %s (layer_z=%.3f)\n"
            "  Bed mesh:        %s\n"
            "  Fans:            %s\n"
            "  Speed / Flow:    %d%% / %d%%\n"
            "  Pressure adv:    %s\n"
            "  Input shaper:    %s\n"
            "  Feedrate:        %s mm/min\n"
            "  XY recov offset: X=%.4f Y=%.4f\n"
            "  Exclude objects: %d defined, %d cancelled\n"
            "  Coord / Extrude: %s / %s"
            % (os.path.basename(state.get('file_path', '')),
               state.get('file_position', 0), state.get('bed_temp', 0),
               fmt(state.get('extruder_temps', {}), "%s=%.0f"),
               state.get('active_extruder', '?'), z_desc,
               state.get('layer_z', 0.0), mesh_desc,
               fmt(state.get('fan_speeds', {}), "%s=%.2f"),
               int(state.get('speed_factor', 1.0) * 100),
               int(state.get('extrude_factor', 1.0) * 100),
               fmt(state.get('pressure_advance', {}), "%s=%.4f"), is_desc,
               int(feed_rate) if feed_rate else "unchanged",
               self.resume_x_offset, self.resume_y_offset,
               len(state.get('exclude_objects', []) or []),
               len(state.get('excluded_objects', []) or []),
               "abs" if state.get('absolute_coord', True) else "rel",
               "abs" if state.get('absolute_extrude', True) else "rel"))

    def _resume_from_state(self, gcmd, state, z_home=False):
        file_path = state.get('file_path', '')
        file_pos = state.get('file_position', 0)
        layer_z = state.get('layer_z', 0.0)
        gcode_pos = state.get('gcode_position', [0, 0, 0, 0])
        active_ext = state.get('active_extruder', 'extruder')
        bed_temp = state.get('bed_temp', 0)
        ext_temps = state.get('extruder_temps', {})
        fan_speeds = state.get('fan_speeds', {})
        speed_factor = state.get('speed_factor', 1.0)
        extrude_factor = state.get('extrude_factor', 1.0)
        pa_values = state.get('pressure_advance', {})
        absolute_coord = state.get('absolute_coord', True)
        absolute_extrude = state.get('absolute_extrude', True)

        if not file_path:
            raise gcmd.error("No file path in saved state")

        run = self.gcode.run_script_from_command

        gcmd.respond_info("AFC_PLR: Starting resume from %s" % file_path)

        # 1. Heat bed (non-blocking)
        if bed_temp > 0:
            gcmd.respond_info("AFC_PLR: Heating bed to %.0f" % bed_temp)
            run("M140 S%.0f" % bed_temp)

        # 2. Preheat extruders to standby (non-blocking)
        for name, temp in ext_temps.items():
            if name.startswith('extruder') and temp > 0:
                run("SET_HEATER_TEMPERATURE HEATER=%s TARGET=150" % name)

        # 2b. Z-home only: gain clearance before homing XY. Real Z is unknown
        # after a power loss, so fake it as 0 and raise — otherwise sensorless
        # XY homing would drag the nozzle across the print at its last height.
        if z_home and self.z_home_prelift > 0:
            gcmd.respond_info(
                "AFC_PLR: Z-home — lifting %.1fmm to clear print before homing XY"
                % self.z_home_prelift)
            run("SET_KINEMATIC_POSITION Z=0")
            run("G91")
            run("G1 Z%.3f F600" % self.z_home_prelift)
            run("G90")

        # 3. Home XY only
        gcmd.respond_info("AFC_PLR: Homing XY")
        run("G28 X Y")

        if self.double_home:
            toolhead = self.printer.lookup_object('toolhead')
            kin = toolhead.get_kinematics()
            axes_max = kin.axes_max
            mid_x = axes_max[0] / 2.0
            mid_y = axes_max[1] / 2.0
            gcmd.respond_info(
                "AFC_PLR: Double home — moving to center (%.0f, %.0f) and re-homing"
                % (mid_x, mid_y))
            run("G90")
            run("G1 X%.1f Y%.1f F6000" % (mid_x, mid_y))
            run("G28 X Y")

        # 3b. Restore bed mesh (before Z moves so compensation is active)
        bed_mesh_points = state.get('bed_mesh_points')
        bed_mesh_params = state.get('bed_mesh_params')
        bed_mesh_profile = state.get('bed_mesh_profile', '')
        if bed_mesh_points and bed_mesh_params:
            bm = self.printer.lookup_object('bed_mesh', None)
            if bm is not None:
                gcmd.respond_info(
                    "AFC_PLR: Restoring bed mesh from saved data")
                from extras.bed_mesh import ZMesh
                z_mesh = ZMesh(bed_mesh_params, 'plr_recovery')
                z_mesh.build_mesh(bed_mesh_points)
                bm.set_mesh(z_mesh)
            else:
                gcmd.respond_info("AFC_PLR: bed_mesh not available, skipping")
        elif bed_mesh_profile:
            gcmd.respond_info(
                "AFC_PLR: Loading bed mesh '%s'" % bed_mesh_profile)
            try:
                run('BED_MESH_PROFILE LOAD="%s"' % bed_mesh_profile)
            except Exception:
                gcmd.respond_info(
                    "AFC_PLR: Bed mesh profile '%s' not found, skipping"
                    % bed_mesh_profile)

        # 3c. Apply the configured XY recovery trim (corrects the consistent
        # shift between the original home and the resume re-home).
        if self.resume_x_offset or self.resume_y_offset:
            gcmd.respond_info(
                "AFC_PLR: applying XY recovery offset X=%.4f Y=%.4f"
                % (self.resume_x_offset, self.resume_y_offset))
            run("SET_GCODE_OFFSET X_ADJUST=%.4f Y_ADJUST=%.4f MOVE=0"
                % (self.resume_x_offset, self.resume_y_offset))

        # 4. Establish Z reference
        if z_home:
            # Physically re-home Z at the safe corner (not over the print),
            # recovering a true Z zero instead of trusting the saved value.
            self._run_z_home(gcmd)
            # 4b. Re-apply the active tool's saved Z offset (the touch homes a
            # shared reference; this corrects for the specific tool mounted),
            # mirroring _ADJUST_Z_POSITION_WITH_TOOL_OFFSET but using the saved
            # value since the toolchanger isn't re-initialized after a restart.
            tool_z_off = state.get('active_tool_z_offset', 0.0)
            if self.z_home_apply_tool_offset and tool_z_off:
                th = self.printer.lookup_object('toolhead')
                cur_z = th.get_position()[2]
                gcmd.respond_info(
                    "AFC_PLR: Applying tool %s Z offset %.4f"
                    % (state.get('active_tool', '?'), tool_z_off))
                run("SET_KINEMATIC_POSITION Z=%.4f" % (cur_z + tool_z_off))
            # 4c. Fixed corner->center trim, dialed in once via z_home_offset.
            if self.z_home_offset:
                gcmd.respond_info(
                    "AFC_PLR: applying z_home_offset %.4f (corner->center trim)"
                    % self.z_home_offset)
                run("SET_GCODE_OFFSET Z_ADJUST=%.4f MOVE=0" % self.z_home_offset)
            # 5. Raise from the bed to print height + hop for safe travel.
            run("G90")
            run("G1 Z%.3f F600" % (layer_z + self.resume_z_hop))
        else:
            # Set Z without homing — trust the saved layer_z (at or below the
            # true print height, so the nozzle never starts below the surface).
            gcmd.respond_info("AFC_PLR: Setting Z position to %.3f (layer height)" % layer_z)
            run("SET_KINEMATIC_POSITION Z=%.4f" % layer_z)
            # 5. Z-hop for clearance
            if self.resume_z_hop > 0:
                run("G91")
                run("G1 Z%.3f F600" % self.resume_z_hop)
                run("G90")

        # 6. Activate the saved extruder (tool is already on shuttle)
        current_ext = self.printer.lookup_object('toolhead').get_extruder()
        if current_ext.get_name() != active_ext:
            gcmd.respond_info("AFC_PLR: Activating extruder %s" % active_ext)
            run("ACTIVATE_EXTRUDER EXTRUDER=%s" % active_ext)

        # 7. Heat extruders to target and wait
        for name, temp in ext_temps.items():
            if name.startswith('extruder') and temp >= 170:
                gcmd.respond_info(
                    "AFC_PLR: Heating %s to %.0f" % (name, temp))
                run("SET_HEATER_TEMPERATURE HEATER=%s TARGET=%.0f"
                    % (name, temp))
        if bed_temp > 0:
            gcmd.respond_info("AFC_PLR: Waiting for bed")
            run("M190 S%.0f" % bed_temp)
        for name, temp in ext_temps.items():
            if name == active_ext and temp >= 170:
                gcmd.respond_info(
                    "AFC_PLR: Waiting for %s" % name)
                run("TEMPERATURE_WAIT SENSOR=%s MINIMUM=%.0f"
                    % (name, temp - 2))

        # 8. Prime nozzle (before moving to print — purge at homing corner)
        if self.purge_length > 0:
            gcmd.respond_info(
                "AFC_PLR: Purging %.1fmm" % self.purge_length)
            run("M83")
            run("G1 E%.2f F%d" % (
                self.purge_length, int(self.purge_speed * 60)))
            run("G1 E-1.0 F1800")
            run("G4 P500")

        # 9. Restore fan speeds
        for fname, speed in fan_speeds.items():
            if fname == 'fan':
                run("M106 S%d" % int(speed * 255))
            elif fname.startswith('fan_generic'):
                run("SET_FAN_SPEED FAN=%s SPEED=%.2f"
                    % (fname.split()[-1], speed))

        # 10. Move to saved XY position
        gcmd.respond_info(
            "AFC_PLR: Moving to X%.2f Y%.2f" % (gcode_pos[0], gcode_pos[1]))
        run("G90")
        run("G1 X%.4f Y%.4f F6000" % (gcode_pos[0], gcode_pos[1]))

        # 11. Lower to layer Z
        run("G1 Z%.4f F600" % layer_z)

        # 12. Restore speed/flow factors
        run("M220 S%d" % int(speed_factor * 100))
        run("M221 S%d" % int(extrude_factor * 100))

        # 13. Restore pressure advance (and smooth time) per extruder. Restore
        # unconditionally — the saved value is the real state at checkpoint, so
        # an explicit 0 must win over any non-zero [extruder] config default.
        pa_smooth = state.get('pa_smooth_time', {})
        for name, pa in pa_values.items():
            cmd = "SET_PRESSURE_ADVANCE EXTRUDER=%s ADVANCE=%.4f" % (name, pa)
            st = pa_smooth.get(name)
            if st is not None:
                cmd += " SMOOTH_TIME=%.4f" % st
            run(cmd)

        # 13b. Restore input shaper — re-derived from the active tool's config
        # params (params_input_shaper_*), not saved, since it's a config
        # constant. A restart reverts the shaper to config defaults, so a
        # toolchanger needs the active tool's values re-applied.
        is_data = self._gather_input_shaper(active_ext)
        if is_data and self.printer.lookup_object('input_shaper', None) is not None:
            parts = []
            for axis in ('x', 'y'):
                stype = is_data.get('shaper_type_%s' % axis)
                sfreq = is_data.get('shaper_freq_%s' % axis)
                dratio = is_data.get('damping_ratio_%s' % axis)
                if stype is not None and sfreq is not None:
                    parts.append("SHAPER_TYPE_%s=%s" % (axis.upper(), stype))
                    parts.append("SHAPER_FREQ_%s=%.2f" % (axis.upper(), sfreq))
                    if dratio is not None:
                        parts.append("DAMPING_RATIO_%s=%.4f"
                                     % (axis.upper(), dratio))
            if parts:
                run("SET_INPUT_SHAPER " + " ".join(parts))

        # 14. Restore coordinate mode
        run("G90" if absolute_coord else "G91")
        run("M82" if absolute_extrude else "M83")

        # 14a. Restore the extruder's E position to what the slicer expects at
        # this point in the file. The purge left E somewhere else, so without
        # this an absolute-E file's first move (G1 ... E<big>) extrudes from the
        # wrong origin — a huge blob that trips max_extrude_cross_section and
        # aborts the resume. Harmless for relative-E files.
        run("G92 E%.5f" % gcode_pos[3])

        # 14b. Restore the print feedrate. The recovery Z move left F at 600
        # (10mm/s); on a mid-file resume every slicer move that omits F (an
        # entire feature, e.g. the wipe tower) would inherit that crawl —
        # M220 still reads 100% because it's a separate multiplier. Re-issue
        # the saved feedrate so resumed moves run at print speed (and don't
        # over-extrude at the slow rate).
        feed_rate = state.get('feed_rate', 0)
        if not feed_rate:
            feed_rate = state.get('speed', 0) * 60  # older checkpoints
        if feed_rate > 0:
            run("G1 F%d" % int(feed_rate))

        # 15. Select file, seek to saved position, then start printing
        gcmd.respond_info(
            "AFC_PLR: Resuming %s from position %d" % (file_path, file_pos))

        sd = self._sd
        if sd is None:
            raise gcmd.error("virtual_sdcard not available")

        fname = os.path.basename(file_path)
        run('M23 %s' % fname)
        run('M26 S%d' % file_pos)

        # Restore exclude_object state, wiped by the restart's reset_file.
        # Must run AFTER M23 (which fires reset_file and clears it) and BEFORE
        # M24 so the move transform and cancellations are in place before any
        # object body replays — otherwise cancelled objects print again. The
        # name match alone drives exclusion; the geometry is re-issued so the
        # front-end object map redraws too.
        eo = self.printer.lookup_object('exclude_object', None)
        if eo is not None:
            objects = state.get('exclude_objects', []) or []
            excluded = state.get('excluded_objects', []) or []
            cur_obj = state.get('current_object')
            if objects or excluded:
                gcmd.respond_info(
                    "AFC_PLR: Restoring exclude_object state "
                    "(%d defined, %d cancelled)" % (len(objects), len(excluded)))
            for obj in objects:
                name = obj.get('name')
                if not name:
                    continue
                parts = ['EXCLUDE_OBJECT_DEFINE NAME=%s' % name]
                center = obj.get('center')
                if center:
                    parts.append('CENTER=' + ','.join(str(c) for c in center))
                polygon = obj.get('polygon')
                if polygon:
                    # Compact JSON — the gcode parser splits parameters on
                    # spaces, so json.dumps' default spacing would corrupt the
                    # POLYGON value. Moonraker emits it compact for this reason.
                    parts.append('POLYGON=' + json.dumps(
                        polygon, separators=(',', ':')))
                run(' '.join(parts))
            for name in excluded:
                run('EXCLUDE_OBJECT NAME=%s' % name)
            # Power lost mid-object: re-mark the in-progress object so a resume
            # inside a cancelled object keeps excluding until its
            # EXCLUDE_OBJECT_END arrives in the replayed body.
            if cur_obj:
                run('EXCLUDE_OBJECT_START NAME=%s' % cur_obj)

        run('M24')

        # 16. Clear saved state
        self._clear_state()

        # Verification summary of everything re-applied on this resume.
        gcmd.respond_info("AFC_PLR: Resume complete — applied:\n"
                          + self._format_state_summary(state, z_home=z_home))

    # ── GCode Commands ──────────────────────────────────────────────

    def cmd_PLR_RESUME(self, gcmd):
        """
        Resume a print after power loss using saved state.

        Reads the saved PLR checkpoint, homes XY, sets Z position from
        the saved layer height, heats all extruders, primes the nozzle,
        and resumes printing from the saved file position.

        Parameters
        ----------
        Z_HOME : int, optional
            When 1, physically re-home Z via the configured ``z_home_gcode``
            (touching at a safe spot) instead of trusting the saved Z.

        Usage
        -----
        `AFC_PLR_RESUME` or `AFC_PLR_RESUME Z_HOME=1`
        """
        self._close_prompt()
        z_home = gcmd.get_int('Z_HOME', 0)
        if z_home and not self._z_home_available:
            raise gcmd.error(
                "AFC_PLR: Z_HOME requested but no Z-home method is configured "
                "(set z_home_x/z_home_y or z_home_gcode in [AFC_PLR])")
        state = self._load_state()
        if state is None:
            raise gcmd.error("No saved PLR state found at %s" % self.save_file)
        self._resume_from_state(gcmd, state, z_home=bool(z_home))

    def cmd_PLR_TEST_ZHOME(self, gcmd):
        """
        Dry-run the Z-home recovery sequence on an empty bed.

        Runs only the Z-recovery portion — pre-lift, ``G28 X Y``, the safe
        Z re-home, optional tool-offset apply, the z_home_offset trim, and a
        final hop — then STOPS. No checkpoint is needed and no file is
        resumed, so you can validate the safe touch spot and inspect the
        resulting Z without power-cutting or air-printing.

        Parameters
        ----------
        TOOL_OFFSET : float, optional
            gcode Z offset to apply after the touch (default 0), to mimic a
            specific tool and check the offset handling.
        HOP : float, optional
            mm to raise after establishing Z (default ``resume_z_hop``).

        Usage
        -----
        `AFC_PLR_TEST_ZHOME` or `AFC_PLR_TEST_ZHOME TOOL_OFFSET=-0.2 HOP=10`
        """
        if not self._z_home_available:
            raise gcmd.error(
                "AFC_PLR: no Z-home method is configured (set z_home_x/"
                "z_home_y or z_home_gcode in [AFC_PLR])")
        tool_off = gcmd.get_float('TOOL_OFFSET', 0.0)
        hop = gcmd.get_float('HOP', self.resume_z_hop, minval=0.0)
        run = self.gcode.run_script_from_command

        gcmd.respond_info(
            "AFC_PLR: TEST Z-home (dry run) — no file will be resumed")

        # Start from a clean offset like a real post-restart resume, so the
        # z_home_offset trim (applied additively below) doesn't stack across
        # repeated dry-runs in one session.
        run("SET_GCODE_OFFSET Z=0 MOVE=0")

        # Pre-lift (fake Z=0 then raise) so sensorless XY homing can't drag.
        if self.z_home_prelift > 0:
            gcmd.respond_info(
                "AFC_PLR: lifting %.1fmm (fake Z=0) before homing XY"
                % self.z_home_prelift)
            run("SET_KINEMATIC_POSITION Z=0")
            run("G91")
            run("G1 Z%.3f F600" % self.z_home_prelift)
            run("G90")

        gcmd.respond_info("AFC_PLR: homing XY")
        run("G28 X Y")

        self._run_z_home(gcmd)

        th = self.printer.lookup_object('toolhead')
        z_after = th.get_position()[2]
        gcmd.respond_info("AFC_PLR: Z after re-home = %.4f" % z_after)

        if self.z_home_apply_tool_offset and tool_off:
            new_z = z_after + tool_off
            gcmd.respond_info(
                "AFC_PLR: applying tool offset %.4f -> Z=%.4f"
                % (tool_off, new_z))
            run("SET_KINEMATIC_POSITION Z=%.4f" % new_z)

        if self.z_home_offset:
            gcmd.respond_info(
                "AFC_PLR: applying z_home_offset %.4f (corner->center trim)"
                % self.z_home_offset)
            run("SET_GCODE_OFFSET Z_ADJUST=%.4f MOVE=0" % self.z_home_offset)

        if hop > 0:
            run("G91")
            run("G1 Z%.3f F600" % hop)
            run("G90")

        final_z = th.get_position()[2]
        gcmd.respond_info(
            "AFC_PLR: TEST complete — Z after re-home=%.4f, tool offset=%.4f, "
            "z_home_offset=%.4f, final Z=%.4f. No file resumed."
            % (z_after, tool_off, self.z_home_offset, final_z))

    def _capture_touch_z(self, gcmd, touch_gcode):
        # Run a touch and scrape its Z result from the console — cartographer's
        # touch stores the value internally and does not expose it via
        # get_status, so capturing the response line is the only way to read it.
        captured = []
        handler = captured.append
        self.gcode.register_output_handler(handler)
        try:
            self.gcode.run_script_from_command(touch_gcode)
        finally:
            try:
                self.gcode.output_callbacks.remove(handler)
            except (ValueError, AttributeError):
                pass
        primary = re.compile(r'estimate contact at z\s*=\s*(-?\d+(?:\.\d+)?)')
        fallback = re.compile(r'\bz\s*=\s*(-?\d+(?:\.\d+)?)')
        val = None
        for msg in captured:
            for m in primary.finditer(msg):
                val = float(m.group(1))
        if val is None:
            for msg in captured:
                for m in fallback.finditer(msg):
                    val = float(m.group(1))
        if val is None:
            raise gcmd.error(
                "AFC_PLR: could not parse a touch Z from '%s' output"
                % touch_gcode)
        return val

    def cmd_PLR_CALIBRATE_ZHOME(self, gcmd):
        """
        Measure the corner-vs-center touch delta and report z_home_offset.

        Anchors Z at the normal (center) reference via z_home_calibrate_anchor
        (default ``G28 Z``), then touch-probes the safe corner (z_home_x/y)
        with z_home_calibrate_gcode and reads the result from the console. The
        corner's height in that center-anchored frame is exactly the delta, so
        the suggested z_home_offset is its negative.

        Parameters
        ----------
        CLEAR_MESH : int, optional (default 1)
            BED_MESH_CLEAR first so touches read raw bed, not mesh-compensated.
        APPLY : int, optional (default 0)
            Apply the measured value to z_home_offset for this session.
        SAVE : int, optional (default 0)
            Apply AND persist it to [AFC_PLR] via AFC's config rewriter (the
            'z_home_offset:' line must already exist in [AFC_PLR]).

        Usage
        -----
        `AFC_PLR_CALIBRATE_ZHOME`, `... APPLY=1`, or `... SAVE=1`
        """
        if not self.z_home_calibrate_gcode:
            raise gcmd.error(
                "AFC_PLR: set z_home_calibrate_gcode in [AFC_PLR] (e.g. "
                "CARTOGRAPHER_TOUCH_PROBE)")
        if self.z_home_x is None or self.z_home_y is None:
            raise gcmd.error("AFC_PLR: set z_home_x / z_home_y in [AFC_PLR]")
        clear_mesh = gcmd.get_int('CLEAR_MESH', 1)
        save = gcmd.get_int('SAVE', 0)
        apply_now = gcmd.get_int('APPLY', 0) or save
        run = self.gcode.run_script_from_command
        th = self.printer.lookup_object('toolhead')

        if 'z' not in th.get_status(self.reactor.monotonic())['homed_axes']:
            gcmd.respond_info("AFC_PLR: homing first")
            run("G28")
        # Measure in a clean frame: a lingering SET_GCODE_OFFSET (from a prior
        # resume/test/APPLY) would be added into the reported touch Z, inflating
        # the result. Homing doesn't clear the gcode offset, so zero it here.
        run("SET_GCODE_OFFSET Z=0 MOVE=0")
        if clear_mesh:
            run("BED_MESH_CLEAR")

        # Anchor Z=0 at the normal center reference.
        gcmd.respond_info(
            "AFC_PLR: anchoring Z at center via '%s'"
            % self.z_home_calibrate_anchor)
        run(self.z_home_calibrate_anchor)

        # Touch-probe the safe corner in that frame.
        run("G90")
        run("G0 X%.3f Y%.3f F12000" % (self.z_home_x, self.z_home_y))
        gcmd.respond_info(
            "AFC_PLR: touch-probing corner (%.1f, %.1f)"
            % (self.z_home_x, self.z_home_y))
        corner_z = self._capture_touch_z(gcmd, self.z_home_calibrate_gcode)

        offset = round(-corner_z, 4)
        gcmd.respond_info(
            "AFC_PLR: corner touch z=%.4f (center-anchored) -> "
            "suggested z_home_offset: %.4f" % (corner_z, offset))
        if apply_now:
            self.z_home_offset = offset
            gcmd.respond_info(
                "AFC_PLR: z_home_offset set to %.4f for this session" % offset)
        if save:
            self._save_z_home_offset(gcmd, offset)
        elif not apply_now:
            gcmd.respond_info(
                "AFC_PLR: add 'z_home_offset: %.4f' to [AFC_PLR], or re-run "
                "with SAVE=1 to write it automatically" % offset)

    def _save_z_home_offset(self, gcmd, offset):
        # AFC's ConfigRewrite only scans the AFC folder (cfgloc), so it can't
        # update an [AFC_PLR] kept elsewhere. Walk the whole config tree and
        # rewrite the z_home_offset line wherever it lives; if there is no
        # existing line, fall back to AFC's auto-vars store.
        val = "%.4f" % offset
        path = self._rewrite_config_value('AFC_PLR', 'z_home_offset', val)
        if path:
            gcmd.respond_info(
                "AFC_PLR: saved 'z_home_offset: %s' in %s" % (val, path))
            return
        afc = self._afc or self.printer.lookup_object('AFC', None)
        fn = getattr(afc, 'function', None) if afc is not None else None
        if fn is not None and hasattr(fn, 'ConfigRewrite'):
            try:
                fn.ConfigRewrite('AFC_PLR', 'z_home_offset', val,
                                 "AFC_PLR z_home_offset calibration")
                gcmd.respond_info(
                    "AFC_PLR: no existing z_home_offset line found — wrote it "
                    "to AFC_auto_vars.cfg (merges on next restart)")
                return
            except Exception as e:
                gcmd.respond_info("AFC_PLR: AFC rewriter error: %s" % e)
        gcmd.respond_info(
            "AFC_PLR: could not persist z_home_offset — add "
            "'z_home_offset: %s' to [AFC_PLR] manually" % val)

    def _config_roots(self):
        # Directories that may hold config files: the printer config dir and
        # the AFC config folder, de-duplicated.
        roots = []
        try:
            cf = self.printer.get_start_args().get('config_file')
            if cf:
                roots.append(os.path.dirname(os.path.abspath(cf)))
        except Exception:
            pass
        afc = self._afc or self.printer.lookup_object('AFC', None)
        cfgloc = getattr(afc, 'cfgloc', None) if afc is not None else None
        if cfgloc:
            roots.append(os.path.abspath(cfgloc))
        seen = set()
        out = []
        for r in roots:
            if r and r not in seen and os.path.isdir(r):
                seen.add(r)
                out.append(r)
        return out

    def _rewrite_config_value(self, section, key, val):
        # Find a .cfg in the config tree whose [section] holds a `key` line and
        # rewrite it in place, preserving indentation and any trailing comment.
        # Returns the file path on success, else None.
        sec_re = re.compile(r"^\s*\[\s*%s\s*\]\s*$" % re.escape(section))
        key_re = re.compile(r"^(\s*)%s\s*[:=]" % re.escape(key))
        for root in self._config_roots():
            for dirpath, _dirs, files in os.walk(root):
                for name in sorted(files):
                    if not name.endswith('.cfg'):
                        continue
                    fpath = os.path.join(dirpath, name)
                    try:
                        with open(fpath) as f:
                            lines = f.readlines()
                    except OSError:
                        continue
                    in_sec = False
                    idx = None
                    for i, line in enumerate(lines):
                        if line.lstrip().startswith('['):
                            in_sec = sec_re.match(line) is not None
                            continue
                        if in_sec and key_re.match(line):
                            idx = i
                            break
                    if idx is None:
                        continue
                    indent = key_re.match(lines[idx]).group(1)
                    comment = ''
                    if '#' in lines[idx]:
                        comment = '  ' + lines[idx][lines[idx].index('#'):].rstrip('\n')
                    lines[idx] = "%s%s: %s%s\n" % (indent, key, val, comment)
                    try:
                        with open(fpath, 'w') as f:
                            f.writelines(lines)
                        return fpath
                    except OSError:
                        continue
        return None

    def cmd_PLR_SAVE(self, gcmd):
        """
        Manually save a PLR checkpoint (for testing).

        Usage
        -----
        `AFC_PLR_SAVE`
        """
        if not self._is_printing():
            self._start_tracking()
        try:
            # Synchronous so the command reports the real on-disk result.
            self._save_state(sync=True)
            gcmd.respond_info(
                "AFC_PLR: State saved to %s (layer_z=%.3f, z=%.3f)"
                % (self.save_file, self.layer_z, self._get_current_z()))
        except Exception as e:
            raise gcmd.error("AFC_PLR save failed: %s" % e)

    def cmd_PLR_CLEAR(self, gcmd):
        """
        Clear saved PLR state without resuming.

        Usage
        -----
        `AFC_PLR_CLEAR`
        """
        self._close_prompt()
        self._cleanup()
        gcmd.respond_info("AFC_PLR: Saved state cleared")

    def cmd_PLR_STATUS(self, gcmd):
        """
        Show information about saved PLR state.

        Usage
        -----
        `AFC_PLR_STATUS`
        """
        if not self._has_saved_state:
            gcmd.respond_info("AFC_PLR: No saved state")
            return
        state = self._load_state()
        if state is None:
            gcmd.respond_info("AFC_PLR: Saved state file exists but is unreadable")
            return
        age = time.time() - state.get('timestamp', 0)
        msg = ("AFC_PLR: Saved checkpoint (%.0fs old) — would apply on resume:\n"
               % age)
        msg += self._format_state_summary(state, z_home=None)
        msg += "\n  Lane:            %s" % state.get('current_lane', '?')
        gcmd.respond_info(msg)

    # ── Moonraker status ────────────────────────────────────────────

    def get_status(self, eventtime=None):
        return {
            'enabled': self.enabled,
            'has_saved_state': self._has_saved_state,
            'layer_z': self.layer_z,
            'save_file': self.save_file,
            'is_tracking': self._last_save_time > 0,
            # Consumed by homing_override to pick the Z touch point.
            'z_home_active': self._z_home_active,
            'z_home_x': self.z_home_x if self.z_home_x is not None else 0.0,
            'z_home_y': self.z_home_y if self.z_home_y is not None else 0.0,
            # Mesh zero-reference (print center) for the calibration macro.
            'mesh_zero_x': self.mesh_zero_x if self.mesh_zero_x is not None else 0.0,
            'mesh_zero_y': self.mesh_zero_y if self.mesh_zero_y is not None else 0.0,
        }


def load_config(config):
    return AFCPLR(config)
