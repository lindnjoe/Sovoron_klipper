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
#   AFC_PLR_RESUME
#       Full power loss resume sequence. Reads saved state, homes XY,
#       restores Z without homing, heats, primes, and resumes printing.
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
# ── Configuration ───────────────────────────────────────────────────
#
#   [AFC_PLR]
#   enabled: True                    # Enable/disable PLR
#   save_interval: 30                # Seconds between periodic saves
#   z_check_interval: 1.0            # Seconds between Z position checks
#   resume_z_hop: 5.0                # mm to raise Z during resume
#   pre_resume_purge_length: 30      # mm of filament to purge on resume
#   pre_resume_purge_speed: 3        # mm/s purge extrusion speed
#   save_file: <auto>                # Path to state file (auto = next to AFC vars)

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
        self.save_interval = config.getfloat('save_interval', 30.0, minval=5.0)
        self.z_check_interval = config.getfloat('z_check_interval', 1.0, minval=0.5)
        self.resume_z_hop = config.getfloat('resume_z_hop', 5.0, minval=0.0)
        self.purge_length = config.getfloat('pre_resume_purge_length', 30.0, minval=0.0)
        self.purge_speed = config.getfloat('pre_resume_purge_speed', 3.0, minval=0.1)
        self.double_home = config.getboolean('double_home', False)
        # "Resume - Z Home" recovery: instead of trusting the saved Z
        # (SET_KINEMATIC_POSITION), physically re-home Z at a safe spot so we
        # never drive a fixed G28 Z into the existing print.
        #
        # Built-in mode: set z_home_x / z_home_y to the safe touch corner.
        # During a Z-home resume AFC_PLR raises a flag (exposed via get_status
        # as z_home_active) and runs a plain `G28 Z`; your homing_override
        # reads printer['AFC_PLR'] to touch at that corner instead of center.
        # No extra macros needed.
        #
        # Custom mode: set z_home_gcode to a macro name that does the whole
        # safe re-home itself (used when the built-in flag approach doesn't
        # fit the machine). Takes precedence over z_home_x/y when set.
        self.z_home_gcode = config.get('z_home_gcode', '').strip()
        self.z_home_x = config.getfloat('z_home_x', None)
        self.z_home_y = config.getfloat('z_home_y', None)
        # Fixed Z trim applied every Z-home resume via SET_GCODE_OFFSET, to
        # absorb the bed-flatness delta between the safe touch corner and the
        # mesh zero-reference. Dial in once by live-babystepping a resume.
        self.z_home_offset = config.getfloat('z_home_offset', 0.0)
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
        # Whether the Z-home recovery path is usable at all (custom macro or a
        # configured safe corner).
        self._z_home_available = bool(self.z_home_gcode) or (
            self.z_home_x is not None and self.z_home_y is not None)
        save_file = config.get('save_file', '')
        self._config_save_file = save_file

        self.layer_z = 0.0
        self._pending_layer_z = None
        self._z_up_ticks = 0
        self._last_save_time = 0.0
        self._timer = None
        self._has_saved_state = False

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

    # ── Lifecycle ───────────────────────────────────────────────────

    def _handle_ready(self):
        self._sd = self.printer.lookup_object('virtual_sdcard', None)
        self._gcode_move = self.printer.lookup_object('gcode_move')
        self._afc = self.printer.lookup_object('AFC', None)
        self._print_stats = self.printer.lookup_object('print_stats', None)
        self._heater_bed = self.printer.lookup_object('heater_bed', None)

        if self._config_save_file:
            self.save_file = os.path.expanduser(self._config_save_file)
        elif self._afc is not None:
            afc_var = getattr(self._afc, 'VarFile', '')
            if afc_var:
                self.save_file = os.path.join(
                    os.path.dirname(os.path.expanduser(afc_var)),
                    'AFC_PLR_state.json')
            else:
                self.save_file = os.path.expanduser(
                    '~/printer_data/config/AFC/AFC_PLR_state.json')
        else:
            self.save_file = os.path.expanduser(
                '~/printer_data/config/AFC/AFC_PLR_state.json')

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
        pheaters = self.printer.lookup_object('heaters', None)
        if pheaters is not None:
            for name in pheaters.available_heaters:
                heater = pheaters.lookup_heater(name)
                state['extruder_temps'][name] = heater.target_temp
                if name.startswith('extruder'):
                    ext = self.printer.lookup_object(name, None)
                    if ext is not None and hasattr(ext, 'get_status'):
                        es = ext.get_status(self.reactor.monotonic())
                        pa = es.get('pressure_advance', 0.0)
                        state['pressure_advance'][name] = pa

        if self._heater_bed is not None:
            state['bed_temp'] = self._heater_bed.get_status(
                self.reactor.monotonic()).get('target', 0)
        else:
            state['bed_temp'] = state['extruder_temps'].get('heater_bed', 0)

        bed_mesh = self.printer.lookup_object('bed_mesh', None)
        if bed_mesh is not None:
            z_mesh = bed_mesh.get_mesh()
            if z_mesh is not None:
                state['bed_mesh_profile'] = z_mesh.get_profile_name()
                state['bed_mesh_points'] = z_mesh.get_probed_matrix()
                state['bed_mesh_params'] = dict(z_mesh.get_mesh_params())
            else:
                state['bed_mesh_profile'] = ''

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

        return state

    def _save_state(self, sync=False):
        # Gather must run on the reactor thread (it reads live, non-thread-safe
        # Klipper objects). The slow disk write is then either done inline
        # (sync=True, used at shutdown / manual save where we want the result
        # to land before returning) or handed to the background writer.
        state = self._gather_state()
        if sync:
            self._write_state_to_disk(state, self.save_file)
        else:
            self._queue_save(state)

    def _write_state_to_disk(self, state, path):
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
                return json.load(f)
        except (IOError, json.JSONDecodeError) as e:
            self.logger.error("Failed to load PLR state: %s" % e)
            return None

    def _clear_state(self):
        # Drop any queued write and wait for an in-flight one so the file
        # we're deleting doesn't get rewritten right after by the writer.
        self._flush_writer(clear_pending=True)
        try:
            if os.path.exists(self.save_file):
                os.unlink(self.save_file)
        except OSError as e:
            self.logger.error("Failed to clear PLR state: %s" % e)
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
        else:
            raise gcmd.error(
                "AFC_PLR: Z-home requested but neither z_home_gcode nor "
                "z_home_x/z_home_y is configured")

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

        # 13. Restore pressure advance
        for name, pa in pa_values.items():
            if pa > 0:
                run("SET_PRESSURE_ADVANCE EXTRUDER=%s ADVANCE=%.4f"
                    % (name, pa))

        # 14. Restore coordinate mode
        run("G90" if absolute_coord else "G91")
        run("M82" if absolute_extrude else "M83")

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
        run('M24')

        # 16. Clear saved state
        self._clear_state()

        gcmd.respond_info("AFC_PLR: Resume complete — printing")

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
        # Persist via AFC's config rewriter: updates the z_home_offset line in
        # [AFC_PLR] in place if present, else writes it to AFC_auto_vars.cfg
        # (which merges and takes precedence on the next restart).
        afc = self._afc or self.printer.lookup_object('AFC', None)
        fn = getattr(afc, 'function', None) if afc is not None else None
        if fn is None or not hasattr(fn, 'ConfigRewrite'):
            gcmd.respond_info(
                "AFC_PLR: AFC config rewriter unavailable (AFC object %s) — "
                "add 'z_home_offset: %.4f' to [AFC_PLR] manually"
                % ("found but no .function" if afc is not None
                   else "not found", offset))
            return
        cfgloc = getattr(afc, 'cfgloc', '?')
        in_place = self._zhome_key_location(cfgloc)
        gcmd.respond_info(
            "AFC_PLR: saving z_home_offset=%.4f via AFC rewriter "
            "(config dir: %s)" % (offset, cfgloc))
        try:
            fn.ConfigRewrite('AFC_PLR', 'z_home_offset', "%.4f" % offset,
                             "AFC_PLR z_home_offset calibration")
        except Exception as e:
            gcmd.respond_info(
                "AFC_PLR: rewriter failed (%s) — add 'z_home_offset: %.4f' to "
                "[AFC_PLR] manually" % (e, offset))
            return
        if in_place:
            gcmd.respond_info(
                "AFC_PLR: updated 'z_home_offset' in place at %s" % in_place)
        else:
            gcmd.respond_info(
                "AFC_PLR: no unindented 'z_home_offset:' line found in any "
                "[AFC_PLR] under %s, so it was written to AFC_auto_vars.cfg. "
                "For in-place saves, add a top-level 'z_home_offset: 0.0' line "
                "to your [AFC_PLR] section there." % cfgloc)

    def _zhome_key_location(self, cfgloc):
        # Return the path of a .cfg in cfgloc that has an unindented
        # z_home_offset line inside an [AFC_PLR] section, else None — mirrors
        # what ConfigRewrite matches, so we can report which path it took.
        if not cfgloc or not os.path.isdir(cfgloc):
            return None
        sec = re.compile(r"^\[\s*AFC_PLR\s*\]")
        for name in os.listdir(cfgloc):
            if not name.endswith('.cfg'):
                continue
            path = os.path.join(cfgloc, name)
            try:
                in_section = False
                with open(path) as f:
                    for line in f:
                        if line.startswith('['):
                            in_section = sec.match(line) is not None
                        elif in_section and line.startswith('z_home_offset'):
                            return path
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
        msg = "AFC_PLR: Saved state:\n"
        msg += "  File: %s\n" % state.get('file_name', '?')
        msg += "  File position: %d bytes\n" % state.get('file_position', 0)
        msg += "  Layer Z: %.3f mm\n" % state.get('layer_z', 0)
        msg += "  Current Z: %.3f mm\n" % state.get('current_z', 0)
        msg += "  Active extruder: %s\n" % state.get('active_extruder', '?')
        msg += "  Lane: %s\n" % state.get('current_lane', '?')
        msg += "  Bed temp: %.0f\n" % state.get('bed_temp', 0)
        mesh_info = state.get('bed_mesh_profile', '') or 'none'
        if state.get('bed_mesh_points'):
            mesh_info += ' (mesh data saved)'
        msg += "  Bed mesh: %s\n" % mesh_info
        msg += "  Age: %.0f seconds" % age
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
