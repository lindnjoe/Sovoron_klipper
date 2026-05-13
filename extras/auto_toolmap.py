# Automatic toolmap + print-start calibration flags for Snapmaker U1
#
# Scans the gcode file footer for Orca/PrusaSlicer CONFIG_BLOCK comments
# (filament_type, filament_used, etc.) and pushes them into the U1's
# print_task_config dict.  Also sets per-print calibration flags
# (bed leveling, flow calibration, input shaper) from [auto_toolmap] config.
#
# Config example:
#   [auto_toolmap]
#   enable: True
#   auto_bed_leveling: False
#   flow_calibrate: True
#   shaper_calibrate: False

import os
import re
import logging

FOOTER_READ_SIZE = 131072  # 128 KB from end of file
MAX_EXTRUDERS = 4

class AutoToolmap:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.logger = logging.getLogger('auto_toolmap')

        self.enable = config.getboolean('enable', True)
        self.auto_bed_leveling = config.getboolean('auto_bed_leveling', False)
        self.flow_calibrate = config.getboolean('flow_calibrate', False)
        self.shaper_calibrate = config.getboolean('shaper_calibrate', False)

        self.printer.register_event_handler('klippy:connect',
                                             self._handle_connect)

        self.gcode.register_command(
            'PUSH_TOOLMAP_FROM_FILE',
            self.cmd_PUSH_TOOLMAP_FROM_FILE,
            desc="Parse gcode footer and push toolmap + calibration flags")
        self.gcode.register_command(
            'AUTO_TOOLMAP_SET',
            self.cmd_AUTO_TOOLMAP_SET,
            desc="Runtime toggle for auto_toolmap flags")

    def _handle_connect(self):
        ptc_obj = self.printer.lookup_object('print_task_config', None)
        if ptc_obj is not None:
            cfg = ptc_obj.print_task_config
            cfg['auto_bed_leveling'] = self.auto_bed_leveling
            cfg['flow_calibrate'] = self.flow_calibrate
            cfg['shaper_calibrate'] = self.shaper_calibrate

    def get_status(self, eventtime=None):
        status = {
            'enable': self.enable,
            'auto_bed_leveling': self.auto_bed_leveling,
            'flow_calibrate': self.flow_calibrate,
            'shaper_calibrate': self.shaper_calibrate,
        }
        ptc_obj = self.printer.lookup_object('print_task_config', None)
        if ptc_obj is not None:
            eu = ptc_obj.print_task_config.get('extruders_used', None)
            if eu is not None:
                status['extruders_used'] = list(eu)
        return status

    def _detect_print_file(self, gcmd=None):
        """Auto-detect current print file from virtual_sdcard.file_path."""
        try:
            vsd = self.printer.lookup_object('virtual_sdcard', None)
            if vsd is None:
                return ''
            fpath = getattr(vsd, 'file_path', None)
            if callable(fpath):
                fpath = fpath()
            if fpath and os.path.exists(fpath):
                return fpath
            return ''
        except Exception:
            return ''

    # ------------------------------------------------------------------
    #  GCode commands
    # ------------------------------------------------------------------

    def cmd_AUTO_TOOLMAP_SET(self, gcmd):
        enable = gcmd.get_int('ENABLE', None, minval=0, maxval=1)
        if enable is not None:
            self.enable = bool(enable)
        bed = gcmd.get_int('BED_LEVEL', None, minval=0, maxval=1)
        if bed is not None:
            self.auto_bed_leveling = bool(bed)
        flow = gcmd.get_int('FLOW_CALIBRATE', None, minval=0, maxval=1)
        if flow is not None:
            self.flow_calibrate = bool(flow)
        shaper = gcmd.get_int('SHAPER_CALIBRATE', None, minval=0, maxval=1)
        if shaper is not None:
            self.shaper_calibrate = bool(shaper)
        gcmd.respond_info(
            "auto_toolmap: enable=%s bed_level=%s flow_cal=%s shaper_cal=%s"
            % (self.enable, self.auto_bed_leveling,
               self.flow_calibrate, self.shaper_calibrate))

    def cmd_PUSH_TOOLMAP_FROM_FILE(self, gcmd):
        filepath = gcmd.get('FILE', '')

        # Auto-detect print file from print_stats / print_task_config / virtual_sdcard
        if not filepath:
            filepath = self._detect_print_file(gcmd=gcmd)

        ptc_obj = self.printer.lookup_object('print_task_config', None)
        if ptc_obj is None:
            gcmd.respond_info(
                "PUSH_TOOLMAP_FROM_FILE: print_task_config not available")
            return
        cfg = ptc_obj.print_task_config

        # --- calibration flags (always pushed, even when file not found) ---
        cfg['auto_bed_leveling'] = self.auto_bed_leveling
        cfg['flow_calibrate'] = self.flow_calibrate
        cfg['shaper_calibrate'] = self.shaper_calibrate

        if not filepath or not os.path.isfile(filepath):
            gcmd.respond_info(
                "PUSH_TOOLMAP_FROM_FILE: file not found: '%s'" % filepath)
            gcmd.respond_info(
                "Calibration flags still set: bed_level=%s flow_cal=%s shaper_cal=%s"
                % (self.auto_bed_leveling, self.flow_calibrate,
                   self.shaper_calibrate))
            return

        if not self.enable:
            gcmd.respond_info(
                "auto_toolmap: toolmap scan disabled, calibration flags set")
            return

        # --- scan gcode footer for slicer config block ---
        footer = self._read_footer(filepath)
        if footer is None:
            gcmd.respond_info(
                "PUSH_TOOLMAP_FROM_FILE: could not read file footer")
            return

        parsed = self._parse_config_block(footer)
        if not parsed:
            gcmd.respond_info(
                "PUSH_TOOLMAP_FROM_FILE: no CONFIG_BLOCK found in footer")
            return

        filament_keys = [k for k in parsed
                         if 'filament' in k.lower() or 'used' in k.lower()]
        gcmd.respond_info(
            "auto_toolmap: parsed keys: %s" % ', '.join(filament_keys))

        self._apply_toolmap(cfg, parsed, gcmd)

    # ------------------------------------------------------------------
    #  Footer scanning
    # ------------------------------------------------------------------

    def _read_footer(self, filepath):
        try:
            fsize = os.path.getsize(filepath)
            offset = max(0, fsize - FOOTER_READ_SIZE)
            with open(filepath, 'r', errors='replace') as f:
                f.seek(offset)
                return f.read()
        except Exception as e:
            self.logger.warning("auto_toolmap: read error: %s" % e)
            return None

    def _parse_config_block(self, text):
        """Extract key=value pairs from Orca/PrusaSlicer CONFIG_BLOCK
        plus filament usage data from loose comments above the block."""
        result = {}
        in_block = False
        for line in text.split('\n'):
            stripped = line.strip()
            if not stripped.startswith(';'):
                continue
            content = stripped.lstrip('; ').strip()
            if content == 'CONFIG_BLOCK_START':
                in_block = True
                continue
            if content == 'CONFIG_BLOCK_END':
                break
            if in_block and '=' in content:
                key, _, val = content.partition('=')
                result[key.strip()] = val.strip()

        if not result:
            result = self._parse_loose_comments(text)
        else:
            loose = self._parse_loose_comments(text)
            for key in ('filament_used', 'filament used [mm]'):
                if key in loose and key not in result:
                    result[key] = loose[key]
        return result

    def _parse_loose_comments(self, text):
        """Fallback: scan for filament-related comment lines outside a
        CONFIG_BLOCK (some slicers don't wrap them)."""
        keys_of_interest = {
            'filament_type', 'filament_used', 'filament used [mm]',
            'filament_colour', 'filament_color',
            'nozzle_diameter', 'filament_density',
        }
        result = {}
        for line in text.split('\n'):
            stripped = line.strip()
            if not stripped.startswith(';'):
                continue
            content = stripped.lstrip('; ').strip()
            if '=' in content:
                key, _, val = content.partition('=')
                key = key.strip()
                if key in keys_of_interest:
                    result[key] = val.strip()
        return result

    # ------------------------------------------------------------------
    #  Apply parsed values to print_task_config
    # ------------------------------------------------------------------

    def _apply_toolmap(self, cfg, parsed, gcmd):
        applied = []
        details = []
        used = [False] * MAX_EXTRUDERS
        have_usage_data = False

        fil_type = parsed.get('filament_type', '')
        if fil_type:
            types = [t.strip() for t in fil_type.replace(';', ',').split(',')]
            type_detail = []
            for i, ft in enumerate(types):
                if i >= MAX_EXTRUDERS:
                    break
                if ft and i < len(cfg.get('filament_type', [])):
                    cfg['filament_type'][i] = ft
                    type_detail.append('T%d=%s' % (i, ft))
            applied.append('filament_type')
            if type_detail:
                details.append('filament_type: %s' % ', '.join(type_detail))

        fil_used = parsed.get('filament_used',
                              parsed.get('filament used [mm]', ''))
        if fil_used:
            values = [v.strip() for v in fil_used.replace(';', ',').split(',')]
            used_detail = []
            for i, fv in enumerate(values):
                if i >= MAX_EXTRUDERS:
                    break
                try:
                    fu_val = float(fv)
                except ValueError:
                    continue
                have_usage_data = True
                if fu_val > 0:
                    used[i] = True
                    if i < len(cfg.get('filament_used_g', [])):
                        cfg['filament_used_g'][i] = fu_val
                    used_detail.append('T%d=%.1f' % (i, fu_val))
                else:
                    used_detail.append('T%d=0' % i)
            applied.append('filament_used')
            details.append('filament_used: %s' % ', '.join(used_detail))

        if not have_usage_data and fil_type:
            for i, ft in enumerate(types):
                if i >= MAX_EXTRUDERS:
                    break
                if ft:
                    used[i] = True
            details.append('(no filament_used data, marking all typed extruders as used)')

        fil_color = parsed.get('filament_colour',
                               parsed.get('filament_color', ''))
        if fil_color:
            colors = [c.strip() for c in fil_color.replace(';', ',').split(',')]
            color_detail = []
            for i, fc in enumerate(colors):
                if i >= MAX_EXTRUDERS:
                    break
                if fc and i < len(cfg.get('filament_color', [])):
                    cfg['filament_color'][i] = fc
                    color_detail.append('T%d=%s' % (i, fc))
            applied.append('filament_color')
            if color_detail:
                details.append('filament_color: %s' % ', '.join(color_detail))

        extruders_used = cfg.get('extruders_used', None)
        if extruders_used is not None:
            for i in range(min(MAX_EXTRUDERS, len(extruders_used))):
                extruders_used[i] = used[i]
            used_names = ','.join('T%d' % i for i in range(MAX_EXTRUDERS)
                                  if used[i])
            applied.append('extruders_used(%s)' % used_names)

        if applied:
            gcmd.respond_info(
                "auto_toolmap: applied %s from gcode footer" %
                ', '.join(applied))
            for line in details:
                gcmd.respond_info("  %s" % line)
        else:
            gcmd.respond_info(
                "auto_toolmap: CONFIG_BLOCK found but no filament data matched")


def load_config(config):
    return AutoToolmap(config)
