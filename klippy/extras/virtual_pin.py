# Combined virtual pin and filament sensor module for Klipper
#
# Provides a software-defined input pin that can be attached anywhere an
# endstop would normally be used, and optionally emulates a filament
# switch sensor using that pin.
#
# Copyright (C) 2024  The Klipper Project
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

class VirtualEndstop:
    """Simple endstop-like object representing a virtual input."""
    def __init__(self, vpin, invert):
        self._vpin = vpin
        self._invert = invert
        self._reactor = vpin.printer.get_reactor()

    def get_mcu(self):
        return None

    def add_stepper(self, stepper):
        pass

    def get_steppers(self):
        return []

    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        comp = self._reactor.completion()
        comp.complete(self.query_endstop(print_time))
        return comp

    def home_wait(self, home_end_time):
        if self.query_endstop(home_end_time):
            return home_end_time
        return 0.

    def query_endstop(self, print_time):
        return bool(self._vpin.state) ^ bool(self._invert)

class VirtualInputPin:
    """Configure and manage a virtual input pin."""
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.state = config.getboolean('initial_value', False)
        # Use a set to avoid duplicate callbacks
        self._watchers = set()

        ppins = self.printer.lookup_object('pins')
        try:
            ppins.register_chip('virtual_pin', self)
        except ppins.error:
            pass

        gcode = self.printer.lookup_object('gcode')
        cname = self.name
        gcode.register_mux_command('SET_VIRTUAL_PIN', 'PIN', cname,
                                   self.cmd_SET_VIRTUAL_PIN,
                                   desc=self.cmd_SET_VIRTUAL_PIN_help)
        gcode.register_mux_command('QUERY_VIRTUAL_PIN', 'PIN', cname,
                                   self.cmd_QUERY_VIRTUAL_PIN,
                                   desc=self.cmd_QUERY_VIRTUAL_PIN_help)

    def setup_pin(self, pin_type, pin_params):
        ppins = self.printer.lookup_object('pins')
        pin_name = pin_params['pin']
        if pin_name != self.name:
            obj = self.printer.lookup_object('virtual_pin ' + pin_name, None)
            if obj is None:
                raise ppins.error('virtual_pin %s not configured' % (pin_name,))
            return obj.setup_pin(pin_type, pin_params)
        if pin_type != 'endstop':
            raise ppins.error('virtual_pin pins only support endstop type')
        return VirtualEndstop(self, pin_params['invert'])

    def get_status(self, eventtime):
        return {'value': int(self.state)}

    def set_value(self, val):
        val = bool(val)
        if self.state == val:
            return
        self.state = val
        for cb in list(self._watchers):
            try:
                cb(val)
            except Exception:
                logging.exception('Virtual pin callback error')

    def register_watcher(self, callback):
        self._watchers.add(callback)

    cmd_SET_VIRTUAL_PIN_help = 'Set the value of a virtual input pin'
    def cmd_SET_VIRTUAL_PIN(self, gcmd):
        val = gcmd.get_int('VALUE', 1)
        self.set_value(val)

    cmd_QUERY_VIRTUAL_PIN_help = 'Report the value of a virtual pin'
    def cmd_QUERY_VIRTUAL_PIN(self, gcmd):
        gcmd.respond_info('virtual_pin %s: %d' % (self.name, self.state))

class RunoutHelper:
    def __init__(self, config):
        self.name = config.get_name().split()[-1]
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
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
        self.min_event_systime = self.reactor.NEVER
        self.filament_present = False
        self.sensor_enabled = True
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
            return
        now = self.reactor.monotonic()
        idle_timeout = self.printer.lookup_object('idle_timeout')
        is_printing = idle_timeout.get_status(now)['state'] == 'Printing'
        if is_filament_present:
            if not is_printing and self.insert_gcode is not None:
                self.min_event_systime = self.reactor.NEVER
                logging.info(
                    'Filament Sensor %s: insert event detected, Time %.2f' %
                    (self.name, now))
                self.reactor.register_callback(self._insert_event_handler)
        elif is_printing and self.runout_gcode is not None:
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

class VirtualFilamentSensor:
    """Software emulated filament switch sensor"""
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        pin_name = config.get('pin')
        self.vpin = self.printer.lookup_object('virtual_pin ' + pin_name, None)
        if self.vpin is None:
            raise config.error('Virtual pin %s not configured' % (pin_name,))
        self.reactor = self.printer.get_reactor()
        self.runout_helper = RunoutHelper(config)

        gcode = self.printer.lookup_object('gcode')
        cname = self.name
        gcode.register_mux_command('SET_VIRTUAL_FILAMENT_SENSOR', 'SENSOR', cname,
                                   self.cmd_SET_VIRTUAL_FILAMENT_SENSOR,
                                   desc=self.cmd_SET_VIRTUAL_FILAMENT_SENSOR_help)
        gcode.register_mux_command('QUERY_VIRTUAL_FILAMENT_SENSOR', 'SENSOR', cname,
                                   self.cmd_QUERY_VIRTUAL_FILAMENT_SENSOR,
                                   desc=self.cmd_QUERY_VIRTUAL_FILAMENT_SENSOR_help)
        self.vpin.register_watcher(self._pin_changed)
        self.runout_helper.note_filament_present(self.reactor.monotonic(),
                                                 bool(self.vpin.state))

    def set_value(self, val):
        self.vpin.set_value(val)
        self.runout_helper.note_filament_present(self.reactor.monotonic(),
                                                 bool(self.vpin.state))

    def _pin_changed(self, val):
        self.runout_helper.note_filament_present(self.reactor.monotonic(), bool(val))

    def get_status(self, eventtime):
        status = self.runout_helper.get_status(eventtime)
        status['value'] = int(self.vpin.state)
        return status

    cmd_SET_VIRTUAL_FILAMENT_SENSOR_help = 'Set the value of a virtual filament sensor'
    def cmd_SET_VIRTUAL_FILAMENT_SENSOR(self, gcmd):
        val = gcmd.get_int('VALUE', 1)
        self.set_value(val)

    cmd_QUERY_VIRTUAL_FILAMENT_SENSOR_help = 'Report the value of a virtual filament sensor'
    def cmd_QUERY_VIRTUAL_FILAMENT_SENSOR(self, gcmd):
        gcmd.respond_info('virtual_filament_sensor %s: %d' % (self.name, self.vpin.state))

# Configuration entry points

def load_config_prefix_virtual_pin(config):
    """Config handler for [virtual_pin] sections."""
    return VirtualInputPin(config)

def load_config_prefix_virtual_filament_sensor(config):
    """Config handler for [virtual_filament_sensor] sections."""
    return VirtualFilamentSensor(config)

def load_config_prefix_filament_switch_sensor(config):
    """Config handler for [filament_switch_sensor] sections."""
    return VirtualFilamentSensor(config)

def load_config_prefix(config):
    """Dispatch handler for backward compatibility."""
    prefix = config.get_name().split()[0]
    if prefix == 'virtual_pin':
        return load_config_prefix_virtual_pin(config)
    if prefix in ('virtual_filament_sensor', 'filament_switch_sensor'):
        return load_config_prefix_virtual_filament_sensor(config)
    raise config.error('Unknown prefix %s' % prefix)
