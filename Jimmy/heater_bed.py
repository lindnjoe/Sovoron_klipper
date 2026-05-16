# Support for a heated bed
#
# Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

class PrinterHeaterBed:
    def __init__(self, config):
        self.printer = config.get_printer()
        pheaters = self.printer.load_object(config, 'heaters')
        self.heater = pheaters.setup_heater(config, 'B')
        self.get_status = self.heater.get_status
        self.stats = self.heater.stats
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("M140", self.cmd_M140)
        gcode.register_command("M190", self.cmd_M190)
        wh = self.printer.lookup_object('webhooks')
        wh.register_endpoint("control/bed_temp", self._handle_control_bed_temp)
    def _set_bed_temp(self, temp, wait=False):
        pheaters = self.printer.lookup_object('heaters')
        pheaters.set_temperature(self.heater, temp, wait)
    # webhook interface
    def _handle_control_bed_temp(self, web_request):
        """Handle bed temperature setting request"""
        try:
            temp = web_request.get_float('S', 0.)
            if temp < 0:
                temp = 0
            self._set_bed_temp(temp)
            web_request.send({'state': 'success'})
        except Exception as e:
            logging.error(f'failed to set bed temp: {str(e)}')
            web_request.send({'state': 'error', 'message': str(e)})
    def cmd_M140(self, gcmd, wait=False):
        # Set Bed Temperature
        temp = gcmd.get_float('S', 0.)
        self._set_bed_temp(temp, wait)
    def cmd_M190(self, gcmd):
        # Set Bed Temperature and Wait
        self.cmd_M140(gcmd, wait=True)

def load_config(config):
    return PrinterHeaterBed(config)
