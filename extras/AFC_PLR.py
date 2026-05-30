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
import tempfile
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

        if not self.enabled:
            return

        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
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

        self._timer = self.reactor.register_timer(
            self._timer_cb, self.reactor.NEVER)
        self._idle_check = self.reactor.register_timer(
            self._idle_check_cb,
            self.reactor.monotonic() + 5.0)

    def _idle_check_cb(self, eventtime):
        if self._is_printing() and self._last_save_time == 0:
            self._start_tracking()
        return eventtime + 2.0

    def _handle_shutdown(self):
        if self._is_printing():
            try:
                self._save_state()
                self.logger.info("Shutdown save completed")
            except Exception as e:
                self.logger.error("Shutdown save failed: %s" % e)

    def _handle_reset_file(self):
        self._clear_state()
        self.layer_z = 0.0
        self._pending_layer_z = None
        self._z_up_ticks = 0
        self._last_save_time = 0.0

    # ── Timer ───────────────────────────────────────────────────────

    def _timer_cb(self, eventtime):
        if not self._is_printing():
            if self._last_save_time > 0:
                self._stop_tracking()
                ps = self._print_stats.get_status(eventtime) if self._print_stats else {}
                if ps.get('state') in ('complete', 'cancelled', 'standby'):
                    self._clear_state()
            return eventtime + self.z_check_interval

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
            state['speed_factor'] = gm.speed_factor
            state['extrude_factor'] = gm.extrude_factor
            state['absolute_coord'] = gm.absolute_coord
            state['absolute_extrude'] = gm.absolute_extrude

        toolhead = self.printer.lookup_object('toolhead', None)
        if toolhead is not None:
            state['toolhead_position'] = list(toolhead.get_position())
            state['active_extruder'] = toolhead.get_extruder().get_name()

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

    def _save_state(self):
        state = self._gather_state()
        dir_path = os.path.dirname(self.save_file)
        if dir_path:
            os.makedirs(dir_path, exist_ok=True)
        fd, tmp_path = tempfile.mkstemp(
            dir=dir_path, suffix='.tmp', prefix='plr_')
        try:
            with os.fdopen(fd, 'w') as f:
                json.dump(state, f, indent=2)
            os.rename(tmp_path, self.save_file)
            self._has_saved_state = True
        except Exception:
            try:
                os.unlink(tmp_path)
            except OSError:
                pass
            raise

    def _load_state(self):
        try:
            with open(self.save_file, 'r') as f:
                return json.load(f)
        except (IOError, json.JSONDecodeError) as e:
            self.logger.error("Failed to load PLR state: %s" % e)
            return None

    def _clear_state(self):
        try:
            if os.path.exists(self.save_file):
                os.unlink(self.save_file)
        except OSError as e:
            self.logger.error("Failed to clear PLR state: %s" % e)
        self._has_saved_state = False

    # ── Resume ──────────────────────────────────────────────────────

    def _resume_from_state(self, gcmd, state):
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

        # 3. Home XY only
        gcmd.respond_info("AFC_PLR: Homing XY")
        run("G28 X Y")

        # 4. Set Z position without homing — use layer_z (safe: at or below actual)
        gcmd.respond_info("AFC_PLR: Setting Z position to %.3f (layer height)" % layer_z)
        run("SET_KINEMATIC_POSITION Z=%.4f" % layer_z)

        # 5. Z-hop for clearance
        if self.resume_z_hop > 0:
            run("G91")
            run("G1 Z%.3f F600" % self.resume_z_hop)
            run("G90")

        # 6. Activate the saved extruder via T-command with A0 (skip full
        #    tool change — tool is still physically docked on U1)
        current_ext = self.printer.lookup_object('toolhead').get_extruder()
        if current_ext.get_name() != active_ext:
            ext_index = int(active_ext.replace('extruder', '') or '0')
            gcmd.respond_info(
                "AFC_PLR: Activating T%d (A0 — resume only)" % ext_index)
            run("T%d A0" % ext_index)

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

        # 8. Restore fan speeds
        for fname, speed in fan_speeds.items():
            if fname == 'fan':
                run("M106 S%d" % int(speed * 255))
            elif fname.startswith('fan_generic'):
                run("SET_FAN_SPEED FAN=%s SPEED=%.2f"
                    % (fname.split()[-1], speed))

        # 9. Move to saved XY position
        gcmd.respond_info(
            "AFC_PLR: Moving to X%.2f Y%.2f" % (gcode_pos[0], gcode_pos[1]))
        run("G90")
        run("G1 X%.4f Y%.4f F6000" % (gcode_pos[0], gcode_pos[1]))

        # 10. Lower to layer Z
        run("G1 Z%.4f F600" % layer_z)

        # 11. Prime nozzle
        if self.purge_length > 0:
            gcmd.respond_info(
                "AFC_PLR: Purging %.1fmm" % self.purge_length)
            run("M83")
            run("G1 E%.2f F%d" % (
                self.purge_length, int(self.purge_speed * 60)))
            run("G1 E-1.0 F1800")
            run("G4 P500")

        # 12. Restore speed/flow factors
        if speed_factor != 1.0:
            run("M220 S%d" % int(speed_factor * 100))
        if extrude_factor != 1.0:
            run("M221 S%d" % int(extrude_factor * 100))

        # 13. Restore pressure advance
        for name, pa in pa_values.items():
            if pa > 0:
                run("SET_PRESSURE_ADVANCE EXTRUDER=%s ADVANCE=%.4f"
                    % (name, pa))

        # 14. Restore coordinate mode
        run("G90" if absolute_coord else "G91")
        run("M82" if absolute_extrude else "M83")

        # 15. Seek file and start printing
        gcmd.respond_info(
            "AFC_PLR: Resuming %s from position %d" % (file_path, file_pos))

        sd = self._sd
        if sd is None:
            raise gcmd.error("virtual_sdcard not available")

        fname = os.path.basename(file_path)
        run('SDCARD_PRINT_FILE FILENAME="%s"' % fname)
        sd.file_position = file_pos

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

        Usage
        -----
        `AFC_PLR_RESUME`
        """
        state = self._load_state()
        if state is None:
            raise gcmd.error("No saved PLR state found at %s" % self.save_file)
        self._resume_from_state(gcmd, state)

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
            self._save_state()
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
        self._clear_state()
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
        }


def load_config(config):
    return AFCPLR(config)
