# Virtual input pin module for Klipper
#
# This file allows creation of software defined input pins that can be used
# wherever an endstop pin is normally required.  The pin state can be set
# and queried via gcode commands.
#
# Copyright (C) 2024  The Klipper Project
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class VirtualEndstop:
    """Simple endstop-like object representing a virtual input."""
    def __init__(self, vpin, invert):
        self._vpin = vpin
        self._invert = invert
        self._reactor = vpin.printer.get_reactor()

    # Methods used by homing and other code that expects an MCU_endstop
    def get_mcu(self):
        return None
    def add_stepper(self, stepper):
        pass
    def get_steppers(self):
        return []
    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        # Immediately complete homing based on current state
        comp = self._reactor.completion()
        comp.complete(self.query_endstop(print_time))
        return comp
    def home_wait(self, home_end_time):
        # Return the time homing completed (if triggered)
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

        ppins = self.printer.lookup_object('pins')
        try:
            ppins.register_chip('virtual_input', self)
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
            obj = self.printer.lookup_object('virtual_input ' + pin_name, None)
            if obj is None:
                raise ppins.error('virtual_input %s not configured' % (pin_name,))
            return obj.setup_pin(pin_type, pin_params)
        if pin_type != 'endstop':
            raise ppins.error('virtual_input pins only support endstop type')
        return VirtualEndstop(self, pin_params['invert'])

    def get_status(self, eventtime):
        return {'value': int(self.state)}

    def set_value(self, val):
        self.state = bool(val)

    cmd_SET_VIRTUAL_PIN_help = 'Set the value of a virtual input pin'
    def cmd_SET_VIRTUAL_PIN(self, gcmd):
        val = gcmd.get_int('VALUE', 1)
        self.set_value(val)

    cmd_QUERY_VIRTUAL_PIN_help = 'Report the value of a virtual input pin'
    def cmd_QUERY_VIRTUAL_PIN(self, gcmd):
        gcmd.respond_info('virtual_input %s: %d' % (self.name, self.state))


def load_config_prefix(config):
    return VirtualInputPin(config)
