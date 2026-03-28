# AFC Tool Probe — Per-tool Z-probe support for toolchanger setups
#
# Copyright (C) 2024-2026 Armored Turtle
#
# Based on klipper-toolchanger by Viesturs Zarins (GPLv3)
# https://github.com/viesturz/klipper-toolchanger
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from . import probe


class AFCToolProbe:
    """Per-tool Z probe wrapper for Optotap/Tap toolchanger setups.

    Each tool has its own probe pin (e.g. [AFC_tool_probe T0]).
    The probe is registered with the endstop router so klipper's
    PROBE/PROBE_CALIBRATE commands use the active tool's probe.
    """
    def __init__(self, config):
        self.printer = config.get_printer()
        self.tool_number = config.getint('tool', None)
        self.name = config.get_name()

        pin = config.get('pin')
        ppins = self.printer.lookup_object('pins')
        ppins.allow_multi_use_pin(pin.replace('^', '').replace('!', ''))

        self.mcu_probe = probe.ProbeEndstopWrapper(config)
        self.probe_offsets = probe.ProbeOffsetsHelper(config)
        self.param_helper = probe.ProbeParameterHelper(config)

        if self.tool_number is not None:
            # Register with the endstop router for probe routing
            self.endstop = self.printer.load_object(config, "AFC_tool_probe_endstop")
            self.endstop.add_probe(config, self)


class AFCToolProbeEndstop:
    """Virtual Z-probe endstop that routes to the active tool's probe.

    Replaces klipper's [probe] object in toolchanger setups. Routes
    PROBE/PROBE_CALIBRATE to whichever tool is currently on the shuttle.

    Tool probes are auto-detected at connect time (single untriggered
    probe = active tool). Manual selection via SET_ACTIVE_TOOL_PROBE.
    """
    def __init__(self, config, standalone=True):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name()
        self.probes = []
        self.tool_number_to_probe = {}
        self.last_query = {}
        self.active_probe = None
        self.active_tool_number = -1
        self.logger = None

        self.mcu_probe = _EndstopRouter(self.printer)
        self.probe = _ProbeRouter(self.printer)
        self.probe_offsets = self.probe
        self.param_helper = self.probe
        self.cmd_helper = probe.ProbeCommandHelper(
            config, self, self.mcu_probe.query_endstop)
        self.homing_helper = probe.HomingViaProbeHelper(
            config, self.mcu_probe, self.probe_offsets, self.param_helper)
        self.probe_session = probe.ProbeSessionHelper(
            config, self.param_helper, self.homing_helper.start_probe_session)

        # Emulate klipper's [probe] object
        if self.printer.lookup_object('probe', default=None):
            raise self.printer.config_error(
                'Cannot have both [probe] and [AFC_tool_probe_endstop].')
        self.printer.add_object('probe', self)

        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        self.gcode = self.printer.lookup_object('gcode')
        if standalone:
            self.gcode.register_command(
                'SET_ACTIVE_TOOL_PROBE', self.cmd_SET_ACTIVE_TOOL_PROBE,
                desc="Set the tool probe that will act as the Z endstop")
            self.gcode.register_command(
                'DETECT_ACTIVE_TOOL_PROBE', self.cmd_DETECT_ACTIVE_TOOL_PROBE,
                desc="Detect which tool is active by identifying an untriggered probe")

    # --- Probe interface (delegates to active probe) ---

    def get_probe_params(self, gcmd=None):
        return self.param_helper.get_probe_params(gcmd)

    def get_offsets(self, gcmd=None):
        return self.probe_offsets.get_offsets(gcmd)

    def start_probe_session(self, gcmd):
        return self.probe_session.start_probe_session(gcmd)

    def get_status(self, eventtime):
        status = self.cmd_helper.get_status(eventtime)
        offsets = self.get_offsets()
        status['last_tools_query'] = self.last_query
        status['active_tool_number'] = self.active_tool_number
        status['active_tool_probe'] = (
            self.active_probe.name if self.active_probe else None)
        status['active_tool_probe_x_offset'] = offsets[0]
        status['active_tool_probe_y_offset'] = offsets[1]
        status['active_tool_probe_z_offset'] = offsets[2]
        return status

    # --- Lifecycle ---

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        afc = self.printer.lookup_object('AFC', None)
        self.logger = afc.logger if afc is not None else None
        self._detect_active_tool()

    # --- Probe registration and switching ---

    def add_probe(self, config, tool_probe):
        if tool_probe.tool_number is not None:
            if tool_probe.tool_number in self.tool_number_to_probe:
                raise config.error(
                    f"Duplicate tool probe nr: {tool_probe.tool_number}")
            self.tool_number_to_probe[tool_probe.tool_number] = tool_probe
        self.probes.append(tool_probe)
        self.mcu_probe.add_mcu(tool_probe.mcu_probe)

    def set_active_probe(self, tool_probe):
        if self.active_probe == tool_probe:
            return
        self.active_probe = tool_probe
        if self.active_probe:
            self.probe.set_active_probe(tool_probe)
            self.mcu_probe.set_active_mcu(tool_probe.mcu_probe)
            self.active_tool_number = self.active_probe.tool_number
        else:
            self.probe.set_active_probe(None)
            self.mcu_probe.set_active_mcu(None)
            self.active_tool_number = -1

    # --- Detection ---

    def _query_open_tools(self):
        """Query all probes and return those that are NOT triggered (tool present)."""
        print_time = self.toolhead.get_last_move_time()
        self.last_query.clear()
        candidates = []
        for tp in self.probes:
            triggered = tp.mcu_probe.query_endstop(print_time)
            if tp.tool_number is not None:
                self.last_query[tp.tool_number] = triggered
            if not triggered:
                candidates.append(tp)
        return candidates

    def _detect_active_tool(self):
        candidates = self._query_open_tools()
        if len(candidates) == 1:
            self.set_active_probe(candidates[0])

    # --- Gcode commands ---

    cmd_SET_ACTIVE_TOOL_PROBE_help = "Set the tool probe for Z endstop"
    def cmd_SET_ACTIVE_TOOL_PROBE(self, gcmd):
        probe_nr = gcmd.get_int("T")
        if probe_nr not in self.tool_number_to_probe:
            raise gcmd.error(
                "SET_ACTIVE_TOOL_PROBE no tool probe for tool %d" % probe_nr)
        self.set_active_probe(self.tool_number_to_probe[probe_nr])

    cmd_DETECT_ACTIVE_TOOL_PROBE_help = "Detect active tool probe"
    def cmd_DETECT_ACTIVE_TOOL_PROBE(self, gcmd):
        candidates = self._query_open_tools()
        msg = ""
        if len(candidates) == 1:
            self.set_active_probe(candidates[0])
            msg = "Found active tool probe: %s" % candidates[0].name
        elif len(candidates) == 0:
            self.set_active_probe(None)
            msg = "All probes triggered"
        else:
            self.set_active_probe(None)
            msg = "Multiple probes not triggered: %s" % [
                p.name for p in candidates]
        if self.logger:
            self.logger.info(msg)
        else:
            gcmd.respond_info(msg)


# --- Internal routing classes ---

class _ProbeRouter:
    """Routes probe offset/param queries to the active tool's probe."""
    def __init__(self, printer):
        self.printer = printer
        self.active_probe = None

    def set_active_probe(self, p):
        self.active_probe = p

    def get_offsets(self, *args, **kwargs):
        if not self.active_probe:
            return 0.0, 0.0, 0.0
        return self.active_probe.probe_offsets.get_offsets(*args, **kwargs)

    def create_probe_result(self, *args, **kwargs):
        if not self.active_probe:
            raise self.printer.command_error(
                "Cannot query endstop - no active tool probe.")
        return self.active_probe.probe_offsets.create_probe_result(
            *args, **kwargs)

    def get_probe_params(self, *args, **kwargs):
        if not self.active_probe:
            raise self.printer.command_error(
                "Cannot query endstop - no active tool probe.")
        return self.active_probe.param_helper.get_probe_params(
            *args, **kwargs)


class _EndstopRouter:
    """Routes endstop commands to the active tool's probe MCU."""
    def __init__(self, printer):
        self.active_mcu = None
        self.set_active_mcu(None)
        self._mcus = []
        self._steppers = []
        self.printer = printer

    def add_mcu(self, mcu_probe):
        self._mcus.append(mcu_probe)
        for s in self._steppers:
            mcu_probe.add_stepper(s)

    def set_active_mcu(self, mcu_probe):
        self.active_mcu = mcu_probe
        if self.active_mcu:
            self.get_mcu = self.active_mcu.get_mcu
            self.home_start = self.active_mcu.home_start
            self.home_wait = self.active_mcu.home_wait
            self.multi_probe_begin = self.active_mcu.multi_probe_begin
            self.multi_probe_end = self.active_mcu.multi_probe_end
            self.probe_prepare = self.active_mcu.probe_prepare
            self.probe_finish = self.active_mcu.probe_finish
        else:
            self.get_mcu = self._on_error
            self.home_start = self._on_error
            self.home_wait = self._on_error
            self.multi_probe_begin = self._on_error
            self.multi_probe_end = self._on_error
            self.probe_prepare = self._on_error
            self.probe_finish = self._on_error

    def add_stepper(self, stepper):
        self._steppers.append(stepper)
        for m in self._mcus:
            m.add_stepper(stepper)

    def get_steppers(self):
        return list(self._steppers)

    def _on_error(self, *args, **kwargs):
        raise self.printer.command_error(
            "Cannot interact with probe - no active tool probe.")

    def query_endstop(self, print_time):
        if not self.active_mcu:
            raise self.printer.command_error(
                "Cannot query endstop - no active tool probe.")
        return self.active_mcu.query_endstop(print_time)

    def get_position_endstop(self):
        if not self.active_mcu:
            return 0.0
        return self.active_mcu.get_position_endstop()


# --- Module loaders ---

def load_config(config):
    """Load [AFC_tool_probe_endstop] section."""
    return AFCToolProbeEndstop(config)

def load_config_prefix(config):
    """Load [AFC_tool_probe T0], [AFC_tool_probe T1], etc."""
    return AFCToolProbe(config)
