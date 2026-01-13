# Toolhead Tuner UI for manual extruder tuning
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging


class ToolheadTuner:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()
        self.extruder_num = config.getint('extruder', -1)
        self.extrude_len = config.getfloat('extrude_len', 0.0)
        self.retract_len = config.getfloat('retract_len', 0.0)
        self.extrude_temp = config.getfloat('extrude_temp', 240.0)
        self.gcode.register_command(
            'TOOLHEAD_TUNER', self.cmd_TOOLHEAD_TUNER,
            desc=self.cmd_TOOLHEAD_TUNER_help)
        self.gcode.register_command(
            '_TOOLHEAD_TUNER_SET', self.cmd_TOOLHEAD_TUNER_SET,
            desc=self.cmd_TOOLHEAD_TUNER_SET_help)
        self.gcode.register_command(
            '_TOOLHEAD_TUNER_SELECT', self.cmd_TOOLHEAD_TUNER_SELECT,
            desc=self.cmd_TOOLHEAD_TUNER_SELECT_help)
        self.gcode.register_command(
            '_TOOLHEAD_TUNER_MOVE', self.cmd_TOOLHEAD_TUNER_MOVE,
            desc=self.cmd_TOOLHEAD_TUNER_MOVE_help)
        self.gcode.register_command(
            '_TOOLHEAD_TUNER_SAVE', self.cmd_TOOLHEAD_TUNER_SAVE,
            desc=self.cmd_TOOLHEAD_TUNER_SAVE_help)

    def _run_script(self, script):
        try:
            self.gcode.run_script(script)
        except Exception:
            logging.exception("Toolhead Tuner: error running script")

    def _respond_command(self, message):
        self.gcode.respond_raw(f"// {message}")

    def _extruder_num_from_name(self, name):
        if name == 'extruder':
            return 0
        if name.startswith('extruder'):
            suffix = name[len('extruder'):]
            if suffix.isdigit():
                return int(suffix)
        return 0

    def _extruder_name_from_num(self, extruder_num):
        return 'extruder' if extruder_num == 0 else f'extruder{extruder_num}'

    def _get_extruder_sections(self):
        configfile = self.printer.lookup_object('configfile')
        settings = configfile.get_status(None).get('settings', {})
        return sorted(name for name in settings if name.startswith('extruder'))

    def _get_afc_extruder(self, extruder_name):
        return self.printer.lookup_object(
            f'AFC_extruder {extruder_name}', default=None)

    def _normalize_lengths(self, extrude_len, retract_len, extruder_name):
        afc_extruder = self._get_afc_extruder(extruder_name)
        extrude_default = getattr(afc_extruder, 'tool_stn', 1.0)
        retract_default = getattr(
            afc_extruder, 'tool_stn_unload', extrude_default)
        if extrude_len <= 0:
            extrude_len = float(extrude_default)
        if retract_len <= 0:
            retract_len = float(retract_default)
        return extrude_len, retract_len

    def _get_active_extruder_num(self):
        toolhead = self.printer.lookup_object('toolhead')
        active_extruder = toolhead.get_extruder().get_name()
        return self._extruder_num_from_name(active_extruder)

    def _show_prompt(self):
        if self.extruder_num < 0:
            self.extruder_num = self._get_active_extruder_num()
        selected_extruder = self._extruder_name_from_num(self.extruder_num)
        self.extrude_len, self.retract_len = self._normalize_lengths(
            self.extrude_len, self.retract_len, selected_extruder)
        self._respond_command('action:prompt_begin Toolhead Tuner')
        self._respond_command(
            'action:prompt_text Choose extruder and adjust distances (mm)')
        self._respond_command(
            'action:prompt_text Extruder buttons switch to the selected tool.')
        self._respond_command(
            'action:prompt_text Test Settings buttons heat the extruder if '
            f'needed (default: {int(self.extrude_temp)}C), then move using '
            'the values.')
        extruder_label = (
            'Extruder' if self.extruder_num == 0
            else f'Extruder {self.extruder_num}')
        self._respond_command(f'action:prompt_text {extruder_label}')
        extruder_sections = self._get_extruder_sections()
        if extruder_sections:
            self._respond_command('action:prompt_button_group_start')
            for name in extruder_sections:
                name_num = self._extruder_num_from_name(name)
                button_label = (
                    'Extruder' if name_num == 0
                    else f'Extruder {name_num}')
                style = 'primary' if name_num == self.extruder_num else 'secondary'
                self._respond_command(
                    f'action:prompt_button {button_label}|'
                    f'_TOOLHEAD_TUNER_SELECT EXTRUDER={name_num}|{style}')
            self._respond_command('action:prompt_button_group_end')
        self._respond_command(
            f'action:prompt_text Extrude (tool_stn): {self.extrude_len}')
        self._respond_command('action:prompt_button_group_start')
        self._respond_command(
            f'action:prompt_button -5|_TOOLHEAD_TUNER_SET '
            f'EXTRUDE_LEN={self.extrude_len - 5}|secondary')
        self._respond_command(
            f'action:prompt_button -1|_TOOLHEAD_TUNER_SET '
            f'EXTRUDE_LEN={self.extrude_len - 1}|secondary')
        self._respond_command(
            f'action:prompt_button +1|_TOOLHEAD_TUNER_SET '
            f'EXTRUDE_LEN={self.extrude_len + 1}|secondary')
        self._respond_command(
            f'action:prompt_button +5|_TOOLHEAD_TUNER_SET '
            f'EXTRUDE_LEN={self.extrude_len + 5}|secondary')
        self._respond_command('action:prompt_button_group_end')
        self._respond_command(
            f'action:prompt_text Retract (tool_stn_unload): {self.retract_len}')
        self._respond_command('action:prompt_button_group_start')
        self._respond_command(
            f'action:prompt_button -5|_TOOLHEAD_TUNER_SET '
            f'RETRACT_LEN={self.retract_len - 5}|secondary')
        self._respond_command(
            f'action:prompt_button -1|_TOOLHEAD_TUNER_SET '
            f'RETRACT_LEN={self.retract_len - 1}|secondary')
        self._respond_command(
            f'action:prompt_button +1|_TOOLHEAD_TUNER_SET '
            f'RETRACT_LEN={self.retract_len + 1}|secondary')
        self._respond_command(
            f'action:prompt_button +5|_TOOLHEAD_TUNER_SET '
            f'RETRACT_LEN={self.retract_len + 5}|secondary')
        self._respond_command('action:prompt_button_group_end')
        self._respond_command('action:prompt_text Test Settings:')
        self._respond_command('action:prompt_button_group_start')
        self._respond_command(
            f'action:prompt_button EXTRUDE |_TOOLHEAD_TUNER_MOVE '
            f'EXTRUDER={self.extruder_num} DIST={self.extrude_len}|primary')
        self._respond_command(
            f'action:prompt_button +1 |_TOOLHEAD_TUNER_MOVE '
            f'EXTRUDER={self.extruder_num} DIST=1|secondary')
        self._respond_command('action:prompt_button_group_end')
        self._respond_command('action:prompt_button_group_start')
        self._respond_command(
            f'action:prompt_button RETRACT |_TOOLHEAD_TUNER_MOVE '
            f'EXTRUDER={self.extruder_num} DIST=-{self.retract_len}|primary')
        self._respond_command(
            f'action:prompt_button -1 |_TOOLHEAD_TUNER_MOVE '
            f'EXTRUDER={self.extruder_num} DIST=-1|secondary')
        self._respond_command('action:prompt_button_group_end')
        self._respond_command(
            f'action:prompt_footer_button Update Toolhead Values|'
            f'_TOOLHEAD_TUNER_SAVE EXTRUDER={self.extruder_num} '
            f'EXTRUDE_LEN={self.extrude_len} RETRACT_LEN={self.retract_len}|'
            'success')
        self._respond_command(
            'action:prompt_footer_button Cancel|RESPOND TYPE=command '
            'MSG="action:prompt_end"|error')
        self._respond_command('action:prompt_show')

    cmd_TOOLHEAD_TUNER_help = "Popup UI for manual extruder tuning"
    def cmd_TOOLHEAD_TUNER(self, gcmd):
        self._show_prompt()

    cmd_TOOLHEAD_TUNER_SET_help = "Update Toolhead_Tuner values and refresh"
    def cmd_TOOLHEAD_TUNER_SET(self, gcmd):
        extruder_param = gcmd.get('EXTRUDER', None)
        extrude_param = gcmd.get('EXTRUDE_LEN', None)
        retract_param = gcmd.get('RETRACT_LEN', None)
        if extruder_param is not None:
            self.extruder_num = int(extruder_param)
        if self.extruder_num < 0:
            self.extruder_num = 0
        if extrude_param is not None:
            self.extrude_len = float(extrude_param)
        if retract_param is not None:
            self.retract_len = float(retract_param)
        if extruder_param is not None and extrude_param is None and retract_param is None:
            selected_extruder = self._extruder_name_from_num(self.extruder_num)
            self.extrude_len, self.retract_len = self._normalize_lengths(
                self.extrude_len, self.retract_len, selected_extruder)
            if self.printer.lookup_object(selected_extruder, default=None):
                self._run_script(f'ACTIVATE_EXTRUDER EXTRUDER={selected_extruder}')
        if self.extrude_len <= 0:
            self.extrude_len = 0.1
        if self.retract_len <= 0:
            self.retract_len = 0.1
        self._show_prompt()

    cmd_TOOLHEAD_TUNER_SELECT_help = "Select an extruder with AFC"
    def cmd_TOOLHEAD_TUNER_SELECT(self, gcmd):
        extruder_num = gcmd.get_int('EXTRUDER', 0)
        extruder_name = self._extruder_name_from_num(extruder_num)
        self._run_script(f'AFC_SELECT_TOOL TOOL={extruder_name}')
        self.extruder_num = extruder_num
        self.cmd_TOOLHEAD_TUNER_SET(gcmd)

    cmd_TOOLHEAD_TUNER_MOVE_help = "Move the active extruder by a requested amount"
    def cmd_TOOLHEAD_TUNER_MOVE(self, gcmd):
        extruder_num = gcmd.get_int('EXTRUDER', 0)
        extruder_name = self._extruder_name_from_num(extruder_num)
        extruder = self.printer.lookup_object(extruder_name, default=None)
        if extruder is None:
            raise gcmd.error(f"Toolhead_Tuner: extruder {extruder_num} not found")
        dist = gcmd.get_float('DIST', 0.0)
        if dist == 0:
            raise gcmd.error("Toolhead_Tuner: distance is 0")
        extrude_temp = float(self.extrude_temp)
        eventtime = self.reactor.monotonic()
        heater = None
        if hasattr(extruder, 'get_heater'):
            heater = extruder.get_heater()
        temperature = None
        min_extrude_temp = None
        can_extrude = None
        if heater is not None:
            heater_status = heater.get_status(eventtime)
            temperature = heater_status.get('temperature')
            min_extrude_temp = getattr(heater, 'min_extrude_temp', None)
            can_extrude = getattr(heater, 'can_extrude', None)
        if hasattr(extruder, 'get_status'):
            extruder_status = extruder.get_status(eventtime)
            if temperature is None:
                temperature = extruder_status.get('temperature')
            if min_extrude_temp is None:
                min_extrude_temp = extruder_status.get('min_extrude_temp')
            if can_extrude is None:
                can_extrude = extruder_status.get('can_extrude')
        if temperature is None:
            temperature = 0.0
        if min_extrude_temp is None:
            min_extrude_temp = 0.0
        if can_extrude is None:
            can_extrude = False
        script_lines = [
            f'ACTIVATE_EXTRUDER EXTRUDER={extruder_name}',
            'SAVE_GCODE_STATE NAME=TOOLHEAD_TUNER_STATE',
            'M83',
        ]
        if (not can_extrude) or (temperature < min_extrude_temp):
            script_lines.append(f'M104 T{extruder_num} S{extrude_temp}')
            script_lines.append(f'M109 T{extruder_num} S{extrude_temp}')
        script_lines.append(f'G1 E{dist} F300')
        script_lines.append('RESTORE_GCODE_STATE NAME=TOOLHEAD_TUNER_STATE')
        self._run_script('\n'.join(script_lines))

    cmd_TOOLHEAD_TUNER_SAVE_help = "Save toolhead tuning values back to AFC config"
    def cmd_TOOLHEAD_TUNER_SAVE(self, gcmd):
        extruder_num = gcmd.get_int('EXTRUDER', 0)
        extruder_name = self._extruder_name_from_num(extruder_num)
        if self.printer.lookup_object(extruder_name, default=None) is None:
            raise gcmd.error(f"Toolhead_Tuner: extruder {extruder_num} not found")
        extrude_len = gcmd.get_float('EXTRUDE_LEN', 0.0)
        retract_len = gcmd.get_float('RETRACT_LEN', 0.0)
        self._run_script(
            f'UPDATE_TOOLHEAD_SENSORS EXTRUDER={extruder_name} '
            f'TOOL_STN={extrude_len} TOOL_STN_UNLOAD={retract_len}')
        self._run_script(f'SAVE_EXTRUDER_VALUES EXTRUDER={extruder_name}')
        self._respond_command('action:prompt_end')


def load_config(config):
    return ToolheadTuner(config)