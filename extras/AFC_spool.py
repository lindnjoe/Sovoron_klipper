# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.


class AFCSpool:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.SPOOLMAN_REMOTE_METHOD = 'spoolman_set_active_spool'

        # Temporary status variables
        self.next_spool_id      = None
        self.next_spool_info    = None

    def handle_connect(self):
        """
        Handle the connection event.
        This function is called when the printer connects. It looks up the AFC object
        and assigns it to the instance variable `self.AFC`.

        :return: None
        """
        self.afc        = self.printer.lookup_object('AFC')
        self.error      = self.afc.error
        self.reactor    = self.afc.reactor
        self.gcode      = self.afc.gcode
        self.logger     = self.afc.logger

        self.disable_weight_check = self.afc.disable_weight_check

        # Registering stepper callback so that mux macro can be set properly with valid lane names
        self.printer.register_event_handler("afc_stepper:register_macros",self.register_lane_macros)

        self.gcode.register_command("RESET_AFC_MAPPING", self.cmd_RESET_AFC_MAPPING, desc=self.cmd_RESET_AFC_MAPPING_help)
        self.gcode.register_command("SET_TOOL_REDIRECT", self.cmd_SET_TOOL_REDIRECT, desc=self.cmd_SET_TOOL_REDIRECT_help)
        self.gcode.register_command("CLEAR_TOOL_REDIRECTS", self.cmd_CLEAR_TOOL_REDIRECTS, desc=self.cmd_CLEAR_TOOL_REDIRECTS_help)
        self.gcode.register_command("SET_NEXT_SPOOL_ID", self.cmd_SET_NEXT_SPOOL_ID, desc=self.cmd_SET_NEXT_SPOOL_ID_help)

    def register_lane_macros(self, lane_obj):
        """
        Callback function to register macros with proper lane names so that klipper errors out correctly when users supply lanes that
        are not valid

        :param lane_obj: object for lane to register
        """
        self.gcode.register_mux_command('SET_COLOR',            "LANE", lane_obj.name, self.cmd_SET_COLOR,              desc=self.cmd_SET_COLOR_help)
        self.gcode.register_mux_command('SET_WEIGHT',           "LANE", lane_obj.name, self.cmd_SET_WEIGHT,             desc=self.cmd_SET_WEIGHT_help)
        self.gcode.register_mux_command('SET_MATERIAL',         "LANE", lane_obj.name, self.cmd_SET_MATERIAL,           desc=self.cmd_SET_MATERIAL_help)
        self.gcode.register_mux_command('SET_SPOOL_ID',         "LANE", lane_obj.name, self.cmd_SET_SPOOL_ID,           desc=self.cmd_SET_SPOOL_ID_help)
        self.gcode.register_mux_command('SET_RUNOUT',           "LANE", lane_obj.name, self.cmd_SET_RUNOUT,             desc=self.cmd_SET_RUNOUT_help)
        self.gcode.register_mux_command('SET_MAP',              "LANE", lane_obj.name, self.cmd_SET_MAP,                desc=self.cmd_SET_MAP_help)
        self.gcode.register_mux_command('SET_REDIRECT',         "LANE", lane_obj.name, self.cmd_SET_REDIRECT,           desc=self.cmd_SET_REDIRECT_help)
        self.gcode.register_mux_command('AFC_SET_SPOOL_TEMP',   "LANE", lane_obj.name, self.cmd_AFC_SET_SPOOL_TEMP,     desc=self.cmd_AFC_SET_SPOOL_TEMP_help)

    cmd_AFC_SET_SPOOL_TEMP_help = "Set spool temperatures for a lane"
    def cmd_AFC_SET_SPOOL_TEMP(self, gcmd):
        """
        This function handles setting the bed and extruder temperatures for a specified lane's spool.

        Usage
        -----
        `AFC_SET_SPOOL_TEMP LANE=<lane> BED_TEMP=<temp> EXTRUDER_TEMP=<temp>`

        Example
        -----
        ```
        AFC_SET_SPOOL_TEMP LANE=lane1 BED_TEMP=60 EXTRUDER_TEMP=210
        ```
        """
        lane = gcmd.get('LANE', None)
        if lane is None:
            self.logger.info("No LANE parameter provided, please specify a valid LANE parameter.")
            return
        cur_lane = self.afc.lanes.get(lane)
        if cur_lane is None:
            self.logger.info('{} Unknown'.format(lane))
            return
        cur_lane.bed_temp = gcmd.get_int('BED_TEMP', cur_lane.bed_temp, minval=0)
        cur_lane.extruder_temp = gcmd.get_int('EXTRUDER_TEMP', cur_lane.extruder_temp, minval=0)
        cur_lane.send_lane_data()
        self.afc.save_vars()

    cmd_SET_MAP_help = "Changes T(n) mapping for a lane"
    def cmd_SET_MAP(self, gcmd):
        """
        Performs a 1:1 swap of tool mapping between the specified lane
        and the lane currently holding the target tool command.

        Usage
        -----
        `SET_MAP LANE=<lane> MAP=<cmd>`

        Example
        -----
        ```
        SET_MAP LANE=lane0 MAP=T2
        ```
        """
        lane = gcmd.get('LANE', None)
        if lane is None:
            self.logger.info("No LANE parameter provided, please specify a valid LANE parameter.")
            return

        map_cmd = gcmd.get('MAP', None)
        if map_cmd is None:
            self.logger.info("No MAP parameter provided, please specify a valid MAP parameter.")
            return

        map_cmd = map_cmd.upper()

        if map_cmd not in self.afc.tool_cmds:
            self.logger.error("Invalid map command: {}".format(map_cmd))
            return

        if lane not in self.afc.lanes:
            self.logger.info('{} Unknown'.format(lane))
            return

        cur_lane = self.afc.lanes[lane]

        if cur_lane.map == map_cmd:
            self.logger.info(f"{lane} is already mapped to {map_cmd}")
            return

        lane_switch = self.afc.tool_cmds[map_cmd]
        sw_lane = self.afc.lanes[lane_switch]
        map_switch = cur_lane.map

        self.afc.tool_cmds[map_cmd] = lane
        cur_lane.map = map_cmd
        cur_lane.send_lane_data()

        self.afc.tool_cmds[map_switch] = lane_switch
        sw_lane.map = map_switch
        sw_lane.send_lane_data()

        self.afc.save_vars()

    cmd_SET_REDIRECT_help = "Redirect a lane's tool command to a different tool"
    def cmd_SET_REDIRECT(self, gcmd):
        """
        Redirects a lane's tool command so it resolves to a different
        tool's lane at runtime. Multiple lanes can redirect to the same
        target, allowing a multi-color print to run on fewer physical
        lanes. The lane's display in Mainsail updates to show the
        redirected tool.

        Redirects are session-only, cleared by RESET_AFC_MAPPING or
        CLEAR_TOOL_REDIRECTS. Requires allow_tool_redirect: True in
        [AFC] config.

        Usage
        -----
        `SET_REDIRECT LANE=<lane> MAP=<T#>`

        Example
        -----
        ```
        SET_REDIRECT LANE=lane0 MAP=T2
        ```
        This makes lane0's tool command resolve to T2's lane.
        """
        if not self.afc.allow_tool_redirect:
            self.logger.info("Tool redirect is disabled. Set allow_tool_redirect: True in [AFC] config.")
            return

        lane = gcmd.get('LANE', None)
        if lane is None:
            self.logger.info("No LANE parameter provided, please specify a valid LANE parameter.")
            return

        map_cmd = gcmd.get('MAP', None)
        if map_cmd is None:
            self.logger.info("No MAP parameter provided, please specify a valid MAP parameter.")
            return

        map_cmd = map_cmd.upper()

        if map_cmd not in self.afc.tool_cmds:
            self.logger.error("Invalid map command: {}".format(map_cmd))
            return

        if lane not in self.afc.lanes:
            self.logger.info('{} Unknown'.format(lane))
            return

        cur_lane = self.afc.lanes[lane]

        source_cmd = None
        for tcmd, lname in self.afc.tool_cmds.items():
            if lname == lane:
                source_cmd = tcmd
                break
        if source_cmd is None:
            self.logger.error(f"Cannot find tool command for {lane}")
            return
        if source_cmd == map_cmd:
            return

        if source_cmd in self.afc.tool_redirects.values():
            self.logger.info(
                f"Cannot redirect {source_cmd} — it is already a redirect target")
            return

        self.afc.tool_redirects[source_cmd] = map_cmd
        cur_lane.map = map_cmd
        cur_lane.send_lane_data()
        target_lane = self.afc.tool_cmds.get(map_cmd, "?")
        self.logger.info(
            f"Redirected {source_cmd} ({lane}) -> {map_cmd} ({target_lane})")

    cmd_SET_TOOL_REDIRECT_help = "Redirect one or more tool commands to a target tool"
    def cmd_SET_TOOL_REDIRECT(self, gcmd):
        """
        Redirects one or more T# commands to a target T# command so that
        multiple tools from a print all resolve to the same lane. This
        allows a multi-color print to run on fewer physical lanes.

        Redirects are session-only and cleared on restart or by
        RESET_AFC_MAPPING / CLEAR_TOOL_REDIRECTS. Requires
        allow_tool_redirect: True in [AFC] config.

        Usage
        -----
        `SET_TOOL_REDIRECT SOURCE=<T#[,T#,...]> TARGET=<T#>`

        Example
        -----
        ```
        SET_TOOL_REDIRECT SOURCE=T0,T4 TARGET=T6
        ```
        This makes T0 and T4 both resolve to T6's lane.
        """
        if not self.afc.allow_tool_redirect:
            self.logger.info("Tool redirect is disabled. Set allow_tool_redirect: True in [AFC] config.")
            return

        source = gcmd.get('SOURCE', None)
        target = gcmd.get('TARGET', None)

        if source is None:
            self.logger.info("No SOURCE parameter provided")
            return
        if target is None:
            self.logger.info("No TARGET parameter provided")
            return

        target = target.upper()
        if target not in self.afc.tool_cmds:
            self.logger.error(f"TARGET {target} is not a valid tool command")
            return

        sources = [s.strip().upper() for s in source.split(',') if s.strip()]
        if not sources:
            self.logger.info("No valid SOURCE tools provided")
            return

        redirected = []
        for src in sources:
            if src == target:
                continue
            if src not in self.afc.tool_cmds:
                self.logger.warning(f"SOURCE {src} is not a registered tool, skipping")
                continue
            self.afc.tool_redirects[src] = target
            src_lane_name = self.afc.tool_cmds.get(src)
            if src_lane_name:
                src_lane = self.afc.lanes.get(src_lane_name)
                if src_lane:
                    src_lane.map = target
                    src_lane.send_lane_data()
            redirected.append(src)

        if redirected:
            target_lane = self.afc.tool_cmds.get(target, "?")
            self.logger.info(
                f"Redirected {','.join(redirected)} -> {target} ({target_lane})")

    cmd_CLEAR_TOOL_REDIRECTS_help = "Clear all tool redirects"
    def cmd_CLEAR_TOOL_REDIRECTS(self, gcmd):
        """
        Clears all tool redirects and restores lane display mappings.
        Requires allow_tool_redirect: True in [AFC] config.

        Usage
        -----
        `CLEAR_TOOL_REDIRECTS`
        """
        if not self.afc.allow_tool_redirect:
            self.logger.info("Tool redirect is disabled. Set allow_tool_redirect: True in [AFC] config.")
            return

        count = len(self.afc.tool_redirects)
        # Restore display mappings for redirected lanes
        for source_cmd in list(self.afc.tool_redirects.keys()):
            src_lane_name = self.afc.tool_cmds.get(source_cmd)
            if src_lane_name:
                src_lane = self.afc.lanes.get(src_lane_name)
                if src_lane:
                    src_lane.map = source_cmd
                    src_lane.send_lane_data()
        self.afc.tool_redirects.clear()
        self.logger.info(f"Cleared {count} tool redirect(s)")

    cmd_SET_COLOR_help = "Set filaments color for a lane"
    def cmd_SET_COLOR(self, gcmd):
        """
        This function handles changing the color of a specified lane. It retrieves the lane
        specified by the 'LANE' parameter and sets its color to the value provided by the 'COLOR' parameter.
        The 'COLOR' parameter should be a hex color code.

        Usage
        -----
        `SET_COLOR LANE=<lane> COLOR=<color>`

        Example
        -----
        ```
        SET_COLOR LANE=lane1 COLOR=FF0000
        ```
        """
        lane = gcmd.get('LANE', None)
        if lane is None:
            self.logger.info("No LANE Defined")
            return
        color = gcmd.get('COLOR', '#000000')
        if lane not in self.afc.lanes:
            self.logger.info('{} Unknown'.format(lane))
            return
        cur_lane = self.afc.lanes[lane]
        cur_lane.color = '#{}'.format(color.replace('#',''))
        cur_lane.send_lane_data()
        # Refresh LED only if filament is loaded — empty lanes keep their state color
        if cur_lane.load_state and cur_lane.unit in self.afc.units:
            unit = cur_lane.unit_obj
            self.afc.function.afc_led(unit._get_lane_color(cur_lane, cur_lane.led_ready), cur_lane.led_index)
        self.afc.save_vars()

    cmd_SET_WEIGHT_help = "Sets filaments weight for a lane"
    def cmd_SET_WEIGHT(self, gcmd):
        """
        This function handles changing the weight remaining of a spool loaded in a specified lane. It retrieves the lane
        specified by the 'LANE' parameter and sets its weight to the value provided by the 'WEIGHT' parameter.

        Usage
        -----
        `SET_WEIGHT LANE=<lane> WEIGHT=<weight>`

        Example
        -----
        ```
        SET_WEIGHT LANE=lane1 WEIGHT=850
        ```
        """
        lane = gcmd.get('LANE', None)
        if lane is None:
            self.logger.info("No LANE Defined")
            return
        weight = gcmd.get_float('WEIGHT', 1000.0)
        if lane not in self.afc.lanes:
            self.logger.info('{} Unknown'.format(lane))
            return
        cur_lane = self.afc.lanes[lane]
        cur_lane.weight = weight
        self.afc.save_vars()

    cmd_SET_MATERIAL_help = "Sets filaments material for a lane"
    def cmd_SET_MATERIAL(self, gcmd):
        """
        This function handles changing the material of a specified lane. It retrieves the lane
        specified by the 'LANE' parameter and sets its material to the value provided by the 'MATERIAL' parameter.

        Values
        ----
        MATERIAL - Material type to set to lane. eg. PLA, ASA, ABS, PETG etc.

        Optional Values
        ----
        DENSITY - Density value to assign to lane. If this is not provided then a default value will be selected based
                   of material. Current default values: PLA: 1.24, PETG:1.23, ABS:1.04, ASA:1.07
        DIAMETER - Diameter of filament, defaults to 1.75
        EMPTY_SPOOL_WEIGHT - Weight of spool once its empty. Defaults to 190.

        Usage
        -----
        `SET_MATERIAL LANE=<lane> MATERIAL=<material>`

        Example
        -----
        ```
        SET_MATERIAL LANE=lane1 MATERIAL=ABS
        ```
        """
        lane = gcmd.get('LANE', None)
        if lane is None:
            self.logger.info("No LANE Defined")
            return
        if lane not in self.afc.lanes:
            self.logger.info('{} Unknown'.format(lane))
            return
        cur_lane = self.afc.lanes[lane]
        density = gcmd.get_float('DENSITY', None)

        cur_lane.material = gcmd.get('MATERIAL')
        cur_lane.filament_diameter = gcmd.get('DIAMETER', cur_lane.filament_diameter)
        cur_lane.empty_spool_weight = gcmd.get('EMPTY_SPOOL_WEIGHT', cur_lane.empty_spool_weight)

        # Setting density if its not none, doing this after setting material as material setter
        # automatically sets density based on material name
        if density is not None:
            cur_lane.filament_density = density

        cur_lane.send_lane_data()
        self.afc.save_vars()

    def set_active_spool(self, ID):
        """
        Set the active spool in Spoolman via a remote webhook call.

        :param ID: Spool ID to set as active
        :return: None
        """
        webhooks = self.printer.lookup_object('webhooks')
        if self.afc.spoolman is not None:
            if ID and ID is not None:
                id = int(ID)
            else:
                id = None

            args = {'spool_id' : id }
            try:
                webhooks.call_remote_method(self.SPOOLMAN_REMOTE_METHOD, **args)
            except self.printer.command_error as e:
                self.logger.error("Error trying to set active spool \n{}".format(e))

    cmd_SET_SPOOL_ID_help = "Set lanes spoolman ID"
    def cmd_SET_SPOOL_ID(self, gcmd):
        """
        This function handles setting the spool ID for a specified lane. It retrieves the lane
        specified by the 'LANE' parameter and updates its spool ID, material, color, and weight
        based on the information retrieved from the Spoolman API.

        Usage
        -----
        `SET_SPOOL_ID LANE=<lane> SPOOL_ID=<spool_id>`

        Example
        -----
        ```
        SET_SPOOL_ID LANE=lane1 SPOOL_ID=12345
        ```
        """
        if self.afc.spoolman is not None:
            lane = gcmd.get('LANE', None)
            if lane is None:
                self.logger.info("No LANE Defined")
                return
            SpoolID = gcmd.get('SPOOL_ID', '')
            if lane not in self.afc.lanes:
                self.logger.info('{} Unknown'.format(lane))
                return

            cur_lane = self.afc.lanes[lane]
            # Check if spool id is already assigned to a different lane, don't assign to current lane if id
            # is already assigned
            if SpoolID != '':
                try:
                    SpoolID = int(SpoolID)
                except ValueError:
                    self.logger.error("Invalid spool ID: {}".format(SpoolID))
                    return

                if cur_lane.spool_id != SpoolID and any( SpoolID == lane.spool_id for lane in self.afc.lanes.values()):
                    self.logger.error(f"SpoolId {SpoolID} already assigned to a lane, cannot assign to {lane}.")
                    return

            self.set_spoolID(cur_lane, SpoolID)

            # If the lane is currently loaded to the toolhead, update the active spool in Spoolman
            if cur_lane.name == self.afc.current:
                self.set_active_spool(cur_lane.spool_id)

    def _get_filament_values( self, filament, field, default=None):
        '''
        Helper function for checking if field is set and returns value if it exists,
        otherwise returns None

        :param filament: Dictionary for filament values
        :param field:    Field name to check for in dictionary
        :return:         Returns value if field exists or None if field does not exist
        '''
        value = default
        if field in filament:
            value = filament[field]
        return value

    def _set_values(self, cur_lane):
        """
        Helper function for setting lane spool values

        :param cur_lane: AFCLane object whose spool values are being set
        :return: None
        """
        # Always reset debounce on spool change
        cur_lane.auto_switch_triggered = False
        # Only apply defaults (material type, 1000g weight) when no spool data has been
        # assigned to this lane. If SET_SPOOL_ID or SET_COLOR was called before loading
        # (e.g. from an NFC tag scan), the spool_id and/or color will already be set with
        # real values — don't overwrite them with defaults during the load sequence.
        if not cur_lane.remember_spool and cur_lane.spool_id is None and not cur_lane.color:
            cur_lane.material = self.afc.default_material_type
            cur_lane.weight = 1000 # Defaulting weight to 1000 upon load

        # Only apply staged next_spool during a genuine fresh-spool load,
        # not when a lane sensor re-triggers mid tool swap.
        mid_swap = self.afc.current_state in (
            "Loading", "Unloading", "Tool swap")
        if not mid_swap:
            if self.afc.spoolman is not None and self.next_spool_id is not None:
                spool_id = self.next_spool_id
                self.next_spool_id = None
                self.set_spoolID(cur_lane, spool_id)
            elif cur_lane.spool_id is None and self.next_spool_info is not None:
                info = self.next_spool_info
                self.next_spool_info = None
                self.next_spool_id = None
                if info.get("material"):
                    cur_lane.material = info["material"]
                color_hex = info.get("color_hex", "")
                if color_hex:
                    cur_lane.color = f"#{color_hex}"
                if info.get("extruder_temp"):
                    cur_lane.extruder_temp = float(info["extruder_temp"])
                if info.get("bed_temp"):
                    cur_lane.bed_temp = float(info["bed_temp"])
                if not getattr(cur_lane, "weight", 0):
                    cur_lane.weight = 1000

    def clear_values(self, cur_lane):
        """
        Helper function for clearing out lane spool values

        :param cur_lane: AFCLane object whose spool values are being cleared
        :return: None
        """
        cur_lane.spool_id = None
        cur_lane.material = ''
        cur_lane.color = ''
        cur_lane.weight = 0
        cur_lane.auto_switch_triggered = False
        cur_lane.extruder_temp = None
        cur_lane.bed_temp = None
        self.next_spool_info = None
        cur_lane.clear_lane_data()

    def set_spoolID(self, cur_lane, SpoolID, save_vars=True):
        """
        Set the spool ID for a lane and update its values from Spoolman.

        :param cur_lane: AFCLane object to assign the spool to
        :param SpoolID: Spool ID to assign to the lane
        :param save_vars: Whether to persist variables after setting (default True)
        :return: None
        """
        if self.afc.spoolman is not None:
            if SpoolID not in ('', None):
                try:
                    result = self.afc.moonraker.get_spool(SpoolID)
                    cur_lane.spool_id = SpoolID
                    cur_lane.auto_switch_triggered = False

                    cur_lane.material           = self._get_filament_values(result['filament'], 'material')
                    if not self.afc.ignore_spoolman_material_temps:
                        cur_lane.extruder_temp      = self._get_filament_values(result['filament'], 'settings_extruder_temp')
                    cur_lane.bed_temp           = self._get_filament_values(result['filament'], 'settings_bed_temp')
                    cur_lane.filament_density   = self._get_filament_values(result['filament'], 'density')
                    cur_lane.filament_diameter  = self._get_filament_values(result['filament'], 'diameter')
                    cur_lane.empty_spool_weight = self._get_filament_values(result, 'spool_weight', default=190)
                    cur_lane.weight             = self._get_filament_values(result, 'remaining_weight')
                    if hasattr(cur_lane, 'espooler'):
                        cur_lane.espooler.espooler_values.full_weight = self._get_filament_values(result, 'initial_weight', default=1000)

                    weight_check = self.disable_weight_check

                    self.afc.logger.info('Weight remaining for SpoolID {}: {}'.format(SpoolID, cur_lane.weight))

                    if not weight_check:
                        if (
                            cur_lane.weight is None or
                            cur_lane.weight <= 0
                        ):
                            self.afc.error.AFC_error("Invalid weight for spoolID: {}. Please check remaining weight before assigning.".format(SpoolID), False)
                            self.clear_values(cur_lane)
                            return

                    # Check to see if filament is defined as multi-color and take the first color for now
                    # Once support for multicolor is added this needs to be updated
                    if "multi_color_hexes" in result['filament']:
                        cur_lane.color = '#{}'.format(self._get_filament_values(result['filament'], 'multi_color_hexes').split(",")[0])
                    else:
                        cur_lane.color = '#{}'.format(self._get_filament_values(result['filament'], 'color_hex'))

                    cur_lane.send_lane_data()
                    # Refresh LED only if filament is loaded — empty lanes keep their state color
                    if cur_lane.load_state and cur_lane.unit in self.afc.units:
                        unit = cur_lane.unit_obj
                        self.afc.function.afc_led(unit._get_lane_color(cur_lane, cur_lane.led_ready), cur_lane.led_index)

                except Exception as e:
                    self.afc.error.AFC_error("Error when trying to get Spoolman data for ID:{}, Error: {}".format(SpoolID, e), False)
            elif not cur_lane.remember_spool:
                self.clear_values(cur_lane)
        elif not cur_lane.remember_spool:
            # Clears out values if users are not using spoolman and lane isn't set to remember spool, this is to cover this function being called from LANE UNLOAD and clearing out
            # Manually entered information
            self.clear_values(cur_lane)
        if save_vars: self.afc.save_vars()

    cmd_SET_RUNOUT_help = "Set runout lane"
    def cmd_SET_RUNOUT(self, gcmd):
        """
        This function handles setting the runout lane (infinite spool) for a specified lane. It retrieves the lane
        specified by the 'LANE' parameter and updates it's the lane to use if filament runs out by un-triggering prep sensor.

        Usage
        -----
        `SET_RUNOUT LANE=<lane> RUNOUT=<lane>`

        Example
        -----
        ```
        SET_RUNOUT LANE=lane1 RUNOUT=lane4
        ```
        """
        lane = gcmd.get('LANE', None)
        if lane is None:
            self.logger.info("No LANE Defined")
            return

        runout = gcmd.get('RUNOUT', 'NONE')
        is_none = runout.upper() == 'NONE'
        # Check to make sure runout does not equal lane
        if lane == runout:
            self.logger.error("Lane({}) and runout({}) cannot be the same".format(lane, runout))
            return
        # Check to make sure specified lane exists
        if lane not in self.afc.lanes:
            self.logger.error('Unknown lane: {}'.format(lane))
            return
        # Check to make sure specified runout lane exists as long as runout is not set as 'NONE'
        if not is_none and runout not in self.afc.lanes:
            self.logger.error('Unknown runout lane: {}'.format(runout))
            return

        cur_lane = self.afc.lanes[lane]
        cur_lane.runout_lane = None if is_none else runout
        self.afc.save_vars()

    cmd_RESET_AFC_MAPPING_help = "Resets all lane mapping in AFC"
    def cmd_RESET_AFC_MAPPING(self, gcmd):
        """
        Resets all tool lane mapping to the order set up in the configuration.
        Optionally resets runout lanes unless RUNOUT=no is specified.

        Useful to put in your PRINT_END macro to reset mapping

        Usage
        -----
        `RESET_AFC_MAPPING [RUNOUT=yes|no]`

        Example
        -----
        ```
        RESET_AFC_MAPPING RUNOUT=no
        ```
        """

        # Gather all tool commands from tool_cmds (never modified in redirect
        # mode) and original config (_map).  Using lane.map would collapse
        # duplicates created by redirects.
        all_cmds = set(self.afc.tool_cmds.keys())
        for lane in self.afc.lanes.values():
            if lane._map is not None:
                all_cmds.add(lane._map)
        # Build pool of auto-assignable commands (not manually configured)
        manually_assigned = {lane._map for lane in self.afc.lanes.values() if lane._map is not None}
        existing_cmds = sorted(
            all_cmds - manually_assigned,
            key=lambda x: int("".join([i for i in x if i.isdigit()])))
        for key, unit in self.afc.units.items():
            for lane in unit.lanes.values():
                if lane._map is not None:
                    map_cmd = lane._map
                else:
                    map_cmd = existing_cmds.pop(0)

                self.afc.tool_cmds[map_cmd] = lane.name
                self.afc.lanes[lane.name].map = map_cmd
                lane.send_lane_data()

        # Resetting runout lanes to None
        runout_opt = gcmd.get('RUNOUT', 'yes').lower()
        if runout_opt != 'no':
            for lane in self.afc.lanes.values():
                lane.runout_lane = None

        # Clear any session-only tool redirects and print usage
        self.afc.tool_redirects.clear()
        self.afc.print_used_tools = None

        self.afc.save_vars()
        self.logger.info("Tool mappings reset" + ("" if runout_opt == "no" else " and runout lanes reset"))

    cmd_SET_NEXT_SPOOL_ID_help = "Set the spool id to be loaded next into AFC"
    def cmd_SET_NEXT_SPOOL_ID(self, gcmd):
        """
        Sets the spool ID to be loaded next into the AFC.

        This can be used in a scanning macro to prepare the AFC for the next spool to be loaded.

        Omit the SPOOL_ID parameter to clear the next spool ID.

        Usage
        -----
        `SET_NEXT_SPOOL_ID SPOOL_ID=<spool_id>`

        Example
        -----
        ```
        SET_NEXT_SPOOL_ID SPOOL_ID=12345
        ```
        """
        SpoolID = gcmd.get('SPOOL_ID', '')
        previous_id = self.next_spool_id
        if SpoolID != '':
            try:
                self.next_spool_id = int(SpoolID)
            except ValueError:
                self.logger.error("Invalid spool ID: {}".format(SpoolID))
                self.next_spool_id = None
        else:
            self.next_spool_id = None
        if previous_id:
            self.logger.info(f"Spool ID '{previous_id}' being overwritten for next load: '{self.next_spool_id}'")
        else:
            self.logger.info(f"Spool ID set for next load: '{self.next_spool_id}'")

def load_config(config):
    return AFCSpool(config)
