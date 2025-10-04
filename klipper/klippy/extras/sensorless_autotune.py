from typing import Dict, Any, Optional

AXES: Dict[str, int] = {'X': 0, 'Y': 1, 'Z': 2}


class SensorlessAutoTune:
    """Finished module: MCU-steps-based measurement, window = tolerance + optional first-approach pre-bias, clean I/O."""

    def __init__(self, config):
        self.printer  = config.get_printer()
        self.gcode    = self.printer.lookup_object('gcode')
        self.toolhead = None
        self.kin      = None

        self._p1_baseline: Dict[str, float] = {}  # axis letter -> baseline distance from Phase 1
        self.homing_timeout_info: Optional[dict] = None  # {'ax', 'stepper_names', ['start'], 'dvec'}

        rs = config.get('restore_speed', None)
        self.restore_speed: Optional[float] = (float(rs) if rs is not None else None)

        self.gcode.register_command('SENSORLESS_AUTOTUNE', self.cmd_SENSORLESS_AUTOTUNE, self.cmd_SENSORLESS_AUTOTUNE_help)
        self.gcode.register_command('FIND_AXIS_CONSTRAINTS', self.cmd_FIND_AXIS_CONSTRAINTS, self.cmd_FIND_AXIS_CONSTRAINTS_help)
        self.printer.register_event_handler('klippy:connect', self._on_connect)
        self.printer.register_event_handler('homing:homing_move_begin', self._on_hmove_begin)
        self.printer.register_event_handler('homing:homing_move_end',   self._on_hmove_end)

        self._status: Dict[str, Any] = {}

    def _on_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.kin      = self.toolhead.get_kinematics()

    def _hmove_is_intecting_arm(self, hmove) -> bool:
        info = self.homing_timeout_info
        if not info:
            return False
        try:
            es_names = {s.get_name() for es in hmove.get_mcu_endstops() for s in es.get_steppers()}
            return bool(es_names.intersection(info['stepper_names']))
        except Exception:
            return False

    def _arm_home(self, axis: str, *, window: Optional[float] = None, mode: str = 'plus') -> dict:
        ax   = AXES[axis]
        rail = self.kin.rails[ax]
        hi   = rail.get_homing_info()
        info = {'ax': ax, 'stepper_names': {s.get_name() for s in rail.get_steppers()}}
        if window is None:
            return info
        sgn    = 1.0 if hi.positive_dir else -1.0
        end    = float(hi.position_endstop)
        base   = abs(float(self._p1_baseline.get(axis, 0.0)))
        total  = abs(window) if mode == 'abs' else abs(window) + base
        info['start'] = end - sgn * total  # pre-bias FIRST approach start
        return info

    def _on_hmove_begin(self, hmove, *args):
        if not self._hmove_is_intecting_arm(hmove):
            return
        info = self.homing_timeout_info or {}
        if 'start' in info:
            cur = list(self.toolhead.get_position())
            cur[info['ax']] = float(info['start'])
            self.toolhead.set_position(cur)
        if info:
            info.pop('dvec', None)

    def _on_hmove_end(self, hmove, *args):
        if not self._hmove_is_intecting_arm(hmove):
            return
        stepper_positions = getattr(hmove, 'stepper_positions', None)
        if not stepper_positions:
            raise self.gcode.command_error("internal error: homing move did not report stepper positions")
        kin = self.kin
        kin_s0 = {s.get_name(): 0.0 for s in kin.get_steppers()}
        kin_s1 = dict(kin_s0)
        for sp in stepper_positions:
            s = sp.stepper
            n = s.get_name()
            d_mm = float(sp.trig_pos - sp.start_pos) * float(s.get_step_dist())
            kin_s1[n] = d_mm
        p0 = kin.calc_position(kin_s0)
        p1 = kin.calc_position(kin_s1)
        dvec = [(b or 0.0) - (a or 0.0) for a, b in zip(p0, p1)]
        if self.homing_timeout_info is not None:
            self.homing_timeout_info['dvec'] = dvec

    def _home_once_and_measure(self, axis: str, *, gcmd, window: Optional[float], window_mode: str = 'plus'):
        ax = AXES[axis]
        self.homing_timeout_info = self._arm_home(axis, window=window, mode=window_mode)
        err_msg = None
        st = self.toolhead.get_status(self.printer.get_reactor().monotonic())
        pre_homed_axes = set((st.get('homed_axes') or ""))
        try:
            self.gcode.run_script_from_command("G28 %s" % axis)
        except self.printer.command_error as e:
            err_msg = str(e)
        self.toolhead.wait_moves()
        info = self.homing_timeout_info or {}
        dvec = info.get('dvec')
        self.homing_timeout_info = None
        if dvec is None:
            raise gcmd.error("internal error: No distance captured from homing move")
        distance = float(dvec[ax])
        post = self.toolhead.get_position()
        if err_msg:
            self.toolhead.set_position(post, homing_axes=pre_homed_axes)

        targets = list(post) # subtract the full measured delta in tool space
        for i, di_raw in enumerate(dvec):
            if i >= len(targets):
                break
            di = float(di_raw or 0.0)
            targets[i] = float(post[i] - di)

        for i in range(min(len(targets), len(dvec))): # per-axis range check only when we actually change that axis
            if abs(targets[i] - post[i]) > 1e-9:
                rmin, rmax = self.kin.rails[i].get_range()
                if not (rmin <= targets[i] <= rmax):
                    di = float(dvec[i] or 0.0)
                    raise gcmd.error(
                        "internal error: cannot restore axis %d: %.2f <= %.2f <= %.2f (dvec: %.3f, post: %.3f)"
                        % (i, rmin, targets[i], rmax, di, post[i])
                    )
        # move back in one shot
        hi = self.kin.rails[AXES[axis]].get_homing_info()
        restore_v = self.restore_speed if (self.restore_speed is not None) else float(hi.speed)
        self.toolhead.manual_move(targets, restore_v)
        self.toolhead.wait_moves()
        return distance, err_msg

    def _run_preflight_check(self, sgh, gcmd):
        # Slam to max sensitivity then do a short, pre-biased approach; fail loudly if no trigger.
        sgh.bump(-999)
        moved, err = self._home_once_and_measure(sgh.axis, gcmd=gcmd, window=10.0, window_mode='abs')
        if err and 'No trigger' in err:
            raise gcmd.error("Pre-flight FAILED: sensorless did not trigger at max sensitivity; check DIAG wiring and mechanics.")
        self.toolhead.wait_moves()
        return abs(moved)

    cmd_SENSORLESS_AUTOTUNE_help = """SENSORLESS_AUTOTUNE AXIS=<X|Y|Z> [MIN_MOVE=<float>] [WINDOW=<float>] [START=<int>] [STOP=<int>]"""

    def cmd_SENSORLESS_AUTOTUNE(self, gcmd):
        axis = gcmd.get('AXIS', '').upper()
        if not axis or axis[0] not in AXES:
            raise gcmd.error("Specify AXIS=X|Y|Z (got %r)" % (axis,))
        axis = axis[0]
        sgh   = AxisStallGuard(self.printer, axis, gcmd=gcmd)
        field = sgh.info['field']
        more  = sgh.info['more_sensitive_step']
        vmin, vmax = sgh.info['value_min'], sgh.info['value_max']
        max_sens, min_sens = sgh.info['max_sensitive'], sgh.info['min_sensitive']
        _dwell    = gcmd.get_float('DWELL', 0.10, above=0.0) / 2.0
        window   = gcmd.get_float('WINDOW', 0.25, minval=0.0)
        start    = gcmd.get_int('START', max_sens)
        stop     = gcmd.get_int('STOP',  min_sens)
        legal_min, legal_max = min(vmin, vmax), max(vmin, vmax)
        if not (legal_min <= start <= legal_max) or not (legal_min <= stop <= legal_max):
            raise gcmd.error("START/STOP out of range (got %d, %d) — legal: [%d, %d]." % (start, stop, legal_min, legal_max))
        
        if (more > 0 and start < stop) or (more < 0 and start > stop):
            raise gcmd.error("Invalid START/STOP ordering. Begin near most sensitive (%d) ? sweep to %d." % (max_sens, min_sens))

        _min_move = self._run_preflight_check(sgh, gcmd)
        min_move  = gcmd.get_float('MIN_MOVE', _min_move * 2, above=0.0)
        gcmd.respond_info("[P1] Testing: %d >>> %d until move over %.2f" % (start, stop, _min_move * 2))

        # Phase 1 ---------- find first OK --------------------------------------------------
        cur = sgh.set(start)
        first_ok = None
        distance_to_home = None
        guard = abs(stop - start) + 2
        while guard > 0:
            guard -= 1
            moved, err = self._home_once_and_measure(axis, gcmd=gcmd, window=None)
            self.printer.get_reactor().pause(_dwell)
            if err:
                raise gcmd.error("[P1] Homing errored.")
            if abs(moved) >= min_move:
                first_ok, distance_to_home = cur, abs(moved)
                gcmd.respond_info("[P1] try: %d, moved: %.2f mm (success)" % (cur, abs(moved)))
                break
            if cur == stop:
                break
            gcmd.respond_info("[P1] try: %d, moved: %.2f mm" % (cur, abs(moved)))
            cur = sgh.bump(+1)
            self.printer.get_reactor().pause(_dwell)
        if first_ok is None:
            raise gcmd.error("No working value found within sweep.")
        self._p1_baseline[axis] = float(distance_to_home)

        # Phase 2 ---------- advance until first FAIL (late trigger = moved too much) --------
        gcmd.respond_info("[P2] Testing: (%d >>> %d)" % (first_ok, stop))
        last_ok = first_ok
        while True:
            if cur == stop:
                break
            cur = sgh.bump(+1)
            self.printer.get_reactor().pause(_dwell)
            moved, err = self._home_once_and_measure(axis, gcmd=gcmd, window=window, window_mode='plus')
            self.printer.get_reactor().pause(_dwell)
            expected_max = distance_to_home + window
            if not err and (abs(moved) <= expected_max):
                gcmd.respond_info("[P2] try: %d: moved: %.2f mm" % (cur, abs(moved)))
                last_ok = cur
                continue
            if abs(moved) > expected_max:
                raise gcmd.error("[P2] Moved too much at try %d (%.2f mm > %.2f mm)" % (cur, abs(moved), expected_max))
            break

        v_lo, v_hi = (first_ok, last_ok) if first_ok <= last_ok else (last_ok, first_ok)
        recommend = v_lo + (v_hi - v_lo) // 2
        sgh.set(recommend)
        gcmd.respond_info("Done testing %s\nRecommended value: %s=%d\nWorking range: %d <-> %d\nValues staged for SAVE_CONFIG" % (sgh._stepper, field.upper(), recommend, v_lo, v_hi))
        self._status[axis] = {'min_ok': v_lo, 'max_ok': v_hi, 'stepper': sgh._stepper}
        cfg = self.printer.lookup_object('configfile')
        option = 'driver_' + field.upper()
        cfg.set(sgh.name, option, int(recommend))

    cmd_FIND_AXIS_CONSTRAINTS_help = """FIND_AXIS_CONSTRAINTS AXIS=<X|Y|Z>\nHome both directions to infer axis min/max"""

    def cmd_FIND_AXIS_CONSTRAINTS(self, gcmd):
        axis = (gcmd.get('AXIS', '') or '').upper()
        if not axis or axis[0] not in AXES:
            raise gcmd.error("Specify AXIS=X|Y|Z (got %r)" % (axis,))
        axis = axis[0]

        ax = AXES[axis]
        rail            = self.kin.rails[ax]
        hi              = rail.get_homing_info()
        rmin, rmax      = rail.get_range()
        
        orig_dir = bool(hi.positive_dir) # True => endstop near max
        orig_end = float(hi.position_endstop) # coordinate of normal-side endstop

        rail.homing_positive_dir = (not orig_dir)
        rail.position_endstop = float(rmin if orig_dir else rmax)
        try:
            moved_A, errA = self._home_once_and_measure(axis, gcmd=gcmd, window=None)
        finally:
            rail.homing_positive_dir = orig_dir
            rail.position_endstop = float(orig_end)

        if errA:
            raise gcmd.error("Opposite-direction G28 failed: %s" % errA)
        exp_sign_A = -1.0 if orig_dir else +1.0
        if abs(moved_A) > 0.01 and moved_A * exp_sign_A <= 0.0:
            raise gcmd.error("[%s] Opposite-direction G28 moved the wrong way (?=%.3f mm)." % (axis, moved_A))

        moved_B, errB = self._home_once_and_measure(axis, gcmd=gcmd, window=None)
        if errB:
            raise gcmd.error("Normal-direction G28 failed: %s" % errB)
        exp_sign_B = +1.0 if orig_dir else -1.0
        if abs(moved_B) > 0.01 and moved_B * exp_sign_B <= 0.0:
            raise gcmd.error("[%s] Normal-direction G28 moved the wrong way (?=%.3f mm)." % (axis, moved_B))

        length = abs(moved_A) + abs(moved_B)
        span = abs(float(rmax) - float(rmin)) or 1.0
        if not (0.40 * span <= length <= 1.60 * span):
            gcmd.respond_info("[%s] Measured travel %.3f mm out of expected range (~%.3f mm)." % (axis, length, span))

        if orig_dir:
            est_max = orig_end
            est_min = orig_end - length
        else:
            est_min = orig_end
            est_max = orig_end + length

        gcmd.respond_info("[%s] travel=%.3f mm (|opposite|=%.3f + |normal|=%.3f); endstop@=%.3f => min=%.3f, max=%.3f"
                          % (axis, length, abs(moved_A), abs(moved_B), orig_end, est_min, est_max))

        # Stage for SAVE_CONFIG
        cfg = self.printer.lookup_object('configfile')
        stepper_section = rail.get_name()
        cfg.set(stepper_section, 'position_min', float(est_min))
        cfg.set(stepper_section, 'position_max', float(est_max))
        gcmd.respond_info("Staged for SAVE_CONFIG: [%s] position_min=%.3f, position_max=%.3f" % (stepper_section, est_min, est_max))

    def get_status(self, eventtime):
        return dict(self._status)

class AxisStallGuard:
    def __init__(self, printer, axis: str, stepper_name: str = None, gcmd=None):
        self.printer = printer
        self.gcode   = printer.lookup_object('gcode')
        self.axis    = axis.upper()
        self.gcmd    = gcmd
        kin = printer.lookup_object('toolhead').get_kinematics()
        self._stepper = stepper_name or self._resolve_stepper_for_axis(self.axis, kin, gcmd)
        self.name, self.tmc = self._get_tmc_for_stepper(self._stepper, gcmd)
        self.info    = self._compute_field_info(self.tmc, gcmd)

    # --- public API ---
    def set(self, value: int):
        v = self._clamp(value)
        self.gcode.run_script_from_command(
            "SET_TMC_FIELD STEPPER=%s FIELD=%s VALUE=%d" % (self._stepper, self.info['field'], int(v))
        )
        return v

    def bump(self, step: int = 1):
        cur = self.read()
        delta = -1 * self.info['more_sensitive_step'] * int(step)
        return self.set(cur + delta)

    def read(self) -> int:
        return int(self.tmc.fields.get_field(self.info['field']))

    # --- internals ---
    def _err(self, gcmd, msg: str):
        if gcmd is not None:
            raise gcmd.error(msg)
        raise self.gcode.command_error(msg)

    def _resolve_stepper_for_axis(self, axis: str, kin, gcmd=None) -> str:
        """Try rails' endstops first (tmc5160 stepper_y, etc.), else fall back to stepper_<axis>."""
        ax   = AXES[axis]
        rail = kin.rails[ax]
        for _, name in rail.get_endstops():
            try:
                self._get_tmc_for_stepper(name, gcmd)
                return name
            except Exception:
                continue
        self._get_tmc_for_stepper('stepper_' + axis.lower(), gcmd)
        return 'stepper_' + axis.lower()

    def _get_tmc_for_stepper(self, stepper_suffix: str, gcmd=None):
        for name, obj in self.printer.lookup_objects():
            if name.startswith('tmc') and name.endswith(stepper_suffix):
                return name, obj
        self._err(gcmd, "No TMC driver found for '%s'" % stepper_suffix)

    def _compute_field_info(self, tmc_obj, gcmd=None) -> Dict[str, Any]:
        fields = tmc_obj.fields
        field = ("sgthrs" if fields.lookup_register("sgthrs", None) is not None
                 else "sgt"  if fields.lookup_register("sgt",  None) is not None
                 else None)
        if field is None:
            self._err(gcmd, "Driver lacks a StallGuard threshold field")

        reg   = fields.lookup_register(field)
        mask  = fields.all_fields[reg][field]
        shift = (mask & -mask).bit_length() - 1
        width = (mask >> shift).bit_length()

        if field in fields.signed_fields:
            value_min = -(1 << (width - 1))
            value_max =  (1 << (width - 1)) - 1
            more_sensitive_step = -1
            signed = True
        else:
            value_min = 0
            value_max = (1 << width) - 1
            more_sensitive_step = +1
            signed = False

        return {
            'field': field,
            'value_min': value_min,
            'value_max': value_max,
            'more_sensitive_step': more_sensitive_step,
            'signed': signed,
            'max_sensitive': value_max if more_sensitive_step > 0 else value_min,
            'min_sensitive': value_min if more_sensitive_step > 0 else value_max,
        }

    def _clamp(self, v: int) -> int:
        if v < self.info['value_min']:
            return self.info['value_min']
        if v > self.info['value_max']:
            return self.info['value_max']
        return int(v)


def load_config(config):
    return SensorlessAutoTune(config)