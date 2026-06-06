# Averaged sensorless (stallguard) homing refinement
#
# Sensorless homing trips at a slightly different physical point each time
# (~0.05-0.1mm of scatter), so two independent homes — e.g. the original print
# and a power-loss resume — land at slightly different origins and the seam
# shifts. This module refines an already-established home by re-probing the
# stall N times, capturing each TRIGGER position (not the reset coordinate),
# rejecting outliers, and averaging — the same principle a Cartographer/Beacon
# touch uses for Z, and the Snapmaker U1 firmware uses for XY.
#
# It is a REFINEMENT: call a normal `G28 <axis>` first to establish the frame,
# then `AVERAGED_SENSORLESS_HOME AXIS=<axis>`. Each probe runs up to the stop
# and OVERSHOOTS the target (so it hits at full speed and stallguard trips like
# a real home); the gantry physically can't pass the stop, and the axis limit
# is only relaxed for the duration of the probe. If the samples don't converge
# it falls back to the existing single home — never worse than a plain G28.
#
# Example config:
#   [averaged_sensorless_home]
#   samples: 3          # samples that must agree within tolerance
#   max_samples: 10     # give up after this many probes -> keep single home
#   tolerance: 0.05     # mm; window of `samples` must fall within this
#   retract: 10         # mm run-up: back off this far, then probe back in
#   overshoot: 3        # mm past the stop the probe targets (full-speed hit)
#   probe_speed: 0      # mm/s; 0 = use the axis homing_speed
#   retract_speed: 40   # mm/s
#   settle: 0.1         # s dwell after retract so stallguard re-arms clean
#
# Usage from homing_override, after the bare G28:
#   G28 Y
#   AVERAGED_SENSORLESS_HOME AXIS=Y
import logging
from . import homing

AXIS_INDEX = {'X': 0, 'Y': 1, 'Z': 2}


class AveragedSensorlessHome:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.samples = config.getint('samples', 3, minval=1)
        self.max_samples = config.getint('max_samples', 10, minval=1)
        if self.max_samples < self.samples:
            raise config.error(
                "averaged_sensorless_home: max_samples must be >= samples")
        self.tolerance = config.getfloat('tolerance', 0.05, above=0.)
        # Hard safety clamp: a real sensorless scatter correction is well under
        # a millimetre. Anything larger means a false/early stall trigger, so
        # reject it and keep the plain G28 home rather than shift the frame.
        self.max_correction = config.getfloat('max_correction', 0.5, above=0.)
        self.retract = config.getfloat('retract', 10.0, above=0.)
        self.overshoot = config.getfloat('overshoot', 3.0, above=0.)
        self.probe_speed = config.getfloat('probe_speed', 0., minval=0.)
        self.retract_speed = config.getfloat('retract_speed', 40.0, above=0.)
        self.settle = config.getfloat('settle', 0.1, minval=0.)
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command(
            'AVERAGED_SENSORLESS_HOME', self.cmd_AVERAGED_SENSORLESS_HOME,
            desc=self.cmd_AVERAGED_SENSORLESS_HOME_help)

    cmd_AVERAGED_SENSORLESS_HOME_help = (
        "Refine an already-homed sensorless axis by averaging multiple "
        "stall-trigger positions")

    def cmd_AVERAGED_SENSORLESS_HOME(self, gcmd):
        axis_name = gcmd.get('AXIS').upper()
        if axis_name not in AXIS_INDEX:
            raise gcmd.error("AXIS must be X, Y or Z")
        axis = AXIS_INDEX[axis_name]
        samples = gcmd.get_int('SAMPLES', self.samples, minval=1)
        max_samples = gcmd.get_int('MAX_SAMPLES', self.max_samples,
                                   minval=samples)
        tolerance = gcmd.get_float('TOLERANCE', self.tolerance, above=0.)
        retract = gcmd.get_float('RETRACT', self.retract, above=0.)
        overshoot = gcmd.get_float('OVERSHOOT', self.overshoot, above=0.)

        toolhead = self.printer.lookup_object('toolhead')
        eventtime = self.printer.get_reactor().monotonic()
        if axis_name.lower() not in toolhead.get_status(eventtime)['homed_axes']:
            raise gcmd.error(
                "AVERAGED_SENSORLESS_HOME: %s must be homed first (run G28 %s)"
                % (axis_name, axis_name))

        kin = toolhead.get_kinematics()
        rail = kin.rails[axis]
        hi = rail.get_homing_info()
        endstop_pos = hi.position_endstop
        endstops = rail.get_endstops()
        speed = gcmd.get_float('SPEED', self.probe_speed or hi.speed, above=0.)
        # +1 if the endstop is at the max end, -1 if at the min end.
        sign = 1.0 if hi.positive_dir else -1.0
        start_pos = endstop_pos - sign * retract      # run-up start (off stop)
        target_pos = endstop_pos + sign * overshoot   # past the stop
        have_limits = hasattr(kin, 'limits')

        samples_list = []
        accepted = None
        for n in range(max_samples):
            # Absolute start each time so a failed probe can't walk the axis.
            self._move_to(toolhead, axis, start_pos, self.retract_speed)
            if self.settle:
                toolhead.dwell(self.settle)
            movepos = list(toolhead.get_position())
            movepos[axis] = target_pos
            saved_limit = None
            if have_limits:
                # Allow targeting past the stop just for this probe; the gantry
                # stalls at the physical stop well before reaching target_pos.
                saved_limit = kin.limits[axis]
                kin.limits[axis] = (min(saved_limit[0], target_pos),
                                    max(saved_limit[1], target_pos))
            try:
                hmove = homing.HomingMove(self.printer, endstops)
                epos = hmove.homing_move(movepos, speed, probe_pos=True)
            except self.printer.command_error as e:
                gcmd.respond_info(
                    "AVERAGED_SENSORLESS_HOME: probe %d failed (%s)"
                    % (n + 1, str(e)))
                continue
            finally:
                if saved_limit is not None:
                    kin.limits[axis] = saved_limit
            samples_list.append(epos[axis])
            if len(samples_list) >= samples:
                window = samples_list[-samples:]
                if max(window) - min(window) <= tolerance:
                    accepted = window
                    break

        if accepted is None:
            gcmd.respond_info(
                "AVERAGED_SENSORLESS_HOME %s: no %d samples within %.3f in %d "
                "probes — keeping the single G28 home (%s)"
                % (axis_name, samples, tolerance, len(samples_list),
                   "spread %.4f" % (max(samples_list) - min(samples_list))
                   if samples_list else "no triggers"))
            self._move_to(toolhead, axis, start_pos, self.retract_speed)
            return

        avg = sum(accepted) / len(accepted)
        correction = endstop_pos - avg
        if abs(correction) > self.max_correction:
            gcmd.respond_info(
                "AVERAGED_SENSORLESS_HOME %s: correction %.3f exceeds "
                "max_correction %.3f — almost certainly a false/early stall "
                "trigger. Keeping the single G28 home (NOT shifting the frame)."
                % (axis_name, correction, self.max_correction))
            self._move_to(toolhead, axis, start_pos, self.retract_speed)
            return
        pos = list(toolhead.get_position())
        pos[axis] += correction
        toolhead.set_position(pos)
        gcmd.respond_info(
            "AVERAGED_SENSORLESS_HOME %s: %d/%d samples, spread %.4f, "
            "applied correction %.4f mm"
            % (axis_name, len(accepted), len(samples_list),
               max(accepted) - min(accepted), correction))
        # Leave the axis backed off so it isn't parked against the stop.
        self._move_to(toolhead, axis, start_pos, self.retract_speed)

    def _move_to(self, toolhead, axis, pos_val, speed):
        coord = [None, None, None, None]
        coord[axis] = pos_val
        toolhead.manual_move(coord, speed)


def load_config(config):
    return AveragedSensorlessHome(config)
