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
# then `AVERAGED_SENSORLESS_HOME AXIS=<axis>`. It never drives past
# position_endstop (== the mechanical stop on these machines), and falls back
# to the existing single home if the samples don't converge — so it is never
# worse than a plain G28.
#
# Example config:
#   [averaged_sensorless_home]
#   samples: 3          # samples that must agree within tolerance
#   max_samples: 10     # give up after this many probes -> keep single home
#   tolerance: 0.05     # mm; window of `samples` must fall within this
#   retract: 10         # mm to back off (away from endstop) between probes
#   probe_speed: 0      # mm/s; 0 = use the axis homing_speed
#   retract_speed: 40   # mm/s
#   settle: 0.1         # s dwell after retract so stallguard re-arms clean
#
# Usage from homing_override, replacing the bare G28:
#   SET_TMC_CURRENT STEPPER=stepper_x CURRENT=0.55   ; your homing current
#   SET_TMC_CURRENT STEPPER=stepper_y CURRENT=0.55
#   G28 Y
#   AVERAGED_SENSORLESS_HOME AXIS=Y
#   ... (your back-off + restore run current) ...
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
        self.retract = config.getfloat('retract', 10.0, above=0.)
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
        positive_dir = hi.positive_dir
        endstops = rail.get_endstops()
        speed = gcmd.get_float('SPEED', self.probe_speed or hi.speed, above=0.)
        # Back-off direction is away from the endstop (keeps the corner clear).
        retract_delta = -retract if positive_dir else retract

        samples_list = []
        accepted = None
        for n in range(max_samples):
            self._move_axis(toolhead, axis, retract_delta, self.retract_speed)
            if self.settle:
                toolhead.dwell(self.settle)
            movepos = list(toolhead.get_position())
            # Target the endstop (== mechanical stop here), never past it.
            movepos[axis] = endstop_pos
            hmove = homing.HomingMove(self.printer, endstops)
            try:
                epos = hmove.homing_move(movepos, speed, probe_pos=True)
            except self.printer.command_error as e:
                gcmd.respond_info(
                    "AVERAGED_SENSORLESS_HOME: probe %d failed (%s)"
                    % (n + 1, str(e)))
                continue
            samples_list.append(epos[axis])
            # Converged once the last `samples` fall within tolerance.
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
            self._move_axis(toolhead, axis, retract_delta, self.retract_speed)
            return

        avg = sum(accepted) / len(accepted)
        correction = endstop_pos - avg
        pos = list(toolhead.get_position())
        pos[axis] += correction
        toolhead.set_position(pos)
        gcmd.respond_info(
            "AVERAGED_SENSORLESS_HOME %s: %d/%d samples, spread %.4f, "
            "applied correction %.4f mm"
            % (axis_name, len(accepted), len(samples_list),
               max(accepted) - min(accepted), correction))
        # Leave the axis backed off so it isn't parked against the stop.
        self._move_axis(toolhead, axis, retract_delta, self.retract_speed)

    def _move_axis(self, toolhead, axis, delta, speed):
        coord = [None, None, None, None]
        coord[axis] = toolhead.get_position()[axis] + delta
        toolhead.manual_move(coord, speed)


def load_config(config):
    return AveragedSensorlessHome(config)
