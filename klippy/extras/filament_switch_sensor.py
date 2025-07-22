# Generic Filament Sensor Module
#
# Copyright (C) 2019  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging


class RunoutHelper:
    def __init__(self, config):
        self.name = config.get_name().split()[-1]
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        # Read config
        self.runout_pause = config.getboolean('pause_on_runout', True)
        if self.runout_pause:
            self.printer.load_object(config, 'pause_resume')
        self.runout_gcode = self.insert_gcode = None
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        if self.runout_pause or config.get('runout_gcode', None) is not None:
            self.runout_gcode = gcode_macro.load_template(
                config, 'runout_gcode', '')
        if config.get('insert_gcode', None) is not None:
            self.insert_gcode = gcode_macro.load_template(
                config, 'insert_gcode')
        self.pause_delay = config.getfloat('pause_delay', .5, above=.0)
        self.event_delay = config.getfloat('event_delay', 3., minval=.0)
        # Internal state
        self.min_event_systime = self.reactor.NEVER
        self.filament_present = False
        self.sensor_enabled = True
        # Register commands and event handlers
        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.gcode.register_mux_command(
            'QUERY_FILAMENT_SENSOR', 'SENSOR', self.name,
            self.cmd_QUERY_FILAMENT_SENSOR,
            desc=self.cmd_QUERY_FILAMENT_SENSOR_help)
        self.gcode.register_mux_command(
            'SET_FILAMENT_SENSOR', 'SENSOR', self.name,
            self.cmd_SET_FILAMENT_SENSOR,
            desc=self.cmd_SET_FILAMENT_SENSOR_help)

    def _handle_ready(self):
        self.min_event_systime = self.reactor.monotonic() + 2.

    def _runout_event_handler(self, eventtime):
        # Pausing from inside an event requires that the pause portion
        # of pause_resume execute immediately.
        pause_prefix = ''
        if self.runout_pause:
            pause_resume = self.printer.lookup_object('pause_resume')
            pause_resume.send_pause_command()
            pause_prefix = 'PAUSE\n'
            self.printer.get_reactor().pause(eventtime + self.pause_delay)
        self._exec_gcode(pause_prefix, self.runout_gcode)

    def _insert_event_handler(self, eventtime):
        self._exec_gcode('', self.insert_gcode)

    def _exec_gcode(self, prefix, template):
        try:
            self.gcode.run_script(prefix + template.render() + '\nM400')
        except Exception:
            logging.exception('Script running error')
        self.min_event_systime = self.reactor.monotonic() + self.event_delay

    def note_filament_present(self, eventtime, is_filament_present):
        if is_filament_present == self.filament_present:
            return
        self.filament_present = is_filament_present

        if eventtime < self.min_event_systime or not self.sensor_enabled:
            # do not process during the initialization time, duplicates,
            # during the event delay time, while an event is running, or
            # when the sensor is disabled
            return

        # Determine "printing" status
        now = self.reactor.monotonic()
        idle_timeout = self.printer.lookup_object('idle_timeout')
        is_printing = idle_timeout.get_status(now)['state'] == 'Printing'

        # Perform filament action associated with status change (if any)
        if is_filament_present:
            if not is_printing and self.insert_gcode is not None:
                # insert detected
                self.min_event_systime = self.reactor.NEVER
                logging.info(
                    'Filament Sensor %s: insert event detected, Time %.2f' %
                    (self.name, now))
                self.reactor.register_callback(self._insert_event_handler)
        elif is_printing and self.runout_gcode is not None:
            # runout detected
            self.min_event_systime = self.reactor.NEVER
            logging.info(
                'Filament Sensor %s: runout event detected, Time %.2f' %
                (self.name, now))
            self.reactor.register_callback(self._runout_event_handler)

    def get_status(self, eventtime):
        return {
            'filament_detected': bool(self.filament_present),
            'enabled': bool(self.sensor_enabled)
        }

    cmd_QUERY_FILAMENT_SENSOR_help = 'Query the status of the Filament Sensor'
    def cmd_QUERY_FILAMENT_SENSOR(self, gcmd):
        if self.filament_present:
            msg = 'Filament Sensor %s: filament detected' % (self.name)
        else:
            msg = 'Filament Sensor %s: filament not detected' % (self.name)
        gcmd.respond_info(msg)

    cmd_SET_FILAMENT_SENSOR_help = 'Sets the filament sensor on/off'
    def cmd_SET_FILAMENT_SENSOR(self, gcmd):
        self.sensor_enabled = gcmd.get_int('ENABLE', 1)


class SwitchSensor:
    def __init__(self, config):
        printer = config.get_printer()
        buttons = printer.load_object(config, 'buttons')
        switch_pin = config.get('switch_pin')
        buttons.register_debounce_button(switch_pin, self._button_handler,
                                         config)
        self.runout_helper = RunoutHelper(config)
        self.get_status = self.runout_helper.get_status

    def _button_handler(self, eventtime, state):
        self.runout_helper.note_filament_present(eventtime, state)


class VirtualSwitchSensor:
    """Emulated filament sensor triggered by a virtual pin."""
    def __init__(self, config, vpin_name):
        self.printer = config.get_printer()
        self.vpin_name = vpin_name
        self.vpin = None
        self.reactor = self.printer.get_reactor()
        self.runout_helper = RunoutHelper(config)
        # Defer binding until Klipper is ready so virtual pin sections can
        # appear anywhere in the config file
        self.printer.register_event_handler('klippy:ready', self._bind_pin)
        self.get_status = self.runout_helper.get_status

    def _bind_pin(self, eventtime):
        if self.vpin is not None:
            return
        vpin = self.printer.lookup_object('virtual_pin ' + self.vpin_name, None)
        if vpin is None:
            logging.error('Virtual pin %s not configured', self.vpin_name)
            return
        self.vpin = vpin
        self.vpin.register_watcher(self._pin_changed)
        self.runout_helper.note_filament_present(self.reactor.monotonic(),
                                                 bool(self.vpin.state))

    def _pin_changed(self, val):
        self.runout_helper.note_filament_present(self.reactor.monotonic(),
                                                 bool(val))


def load_config_prefix(config):
    printer = config.get_printer()
    switch_pin = config.get('switch_pin')
    ppins = printer.lookup_object('pins')

    # Try normal pin parsing first in case the virtual pin chip
    # has already been registered
    try:
        pin_params = ppins.parse_pin(switch_pin, can_invert=True, can_pullup=True)
        if pin_params['chip_name'] == 'virtual_pin':
            vpin_name = pin_params['pin']
            vpin = printer.lookup_object('virtual_pin ' + vpin_name, None)
            if vpin is None:
                # Delay binding until klippy:ready if pin not loaded yet
                return VirtualSwitchSensor(config, vpin_name)
            return VirtualSwitchSensor(config, vpin_name)
        return SwitchSensor(config)
    except Exception:
        # Fall back to detecting "virtual_pin:" prefix without requiring
        # the chip to be registered yet
        pass

    # Basic manual parsing for strings like "!virtual_pin:name" or
    # "^!virtual_pin:name".  Pullup and invert modifiers are ignored
    # since virtual pins do not use them.
    clean_pin = switch_pin.lstrip('!^')
    if clean_pin.startswith('virtual_pin:'):
        vpin_name = clean_pin.split('virtual_pin:', 1)[1]
        vpin_name = vpin_name.strip()
        vpin = printer.lookup_object('virtual_pin ' + vpin_name, None)
        if vpin is None:
            # Delay binding until klippy:ready if pin not loaded yet
            return VirtualSwitchSensor(config, vpin_name)
        return VirtualSwitchSensor(config, vpin_name)

    # If all checks fail, fall back to the normal hardware switch sensor
    return SwitchSensor(config)

