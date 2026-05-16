# Support generic temperature sensors
#
# Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging, os, time, pathlib, queuefile

KELVIN_TO_CELSIUS = -273.15

class PrinterSensorGeneric:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.name = config.get_name().split()[-1]
        pheaters = self.printer.load_object(config, 'heaters')
        self.sensor = pheaters.setup_sensor(config)
        self.min_temp = config.getfloat('min_temp', KELVIN_TO_CELSIUS,
                                        minval=KELVIN_TO_CELSIUS)
        self.max_temp = config.getfloat('max_temp', 99999999.9,
                                        above=self.min_temp)
        self.sensor.setup_minmax(self.min_temp, self.max_temp)
        self.sensor.setup_callback(self.temperature_callback)
        pheaters.register_sensor(config, self)
        self.last_temp = 0.
        self.measured_min = 99999999.
        self.measured_max = 0.
        self.record_temp_interval = 3
        self.record_temp_file = None
        self.record_temp_timer = self.reactor.register_timer(self._record_temp_callback)

        self.gcode.register_mux_command(
            "RECORD_TEMPERATURE", "SENSOR", self.name,
            self.cmd_RECORD_TEMPERATURE)

        self.printer.register_event_handler('klippy:shutdown',
                self._handle_shutdown)

    def _handle_shutdown(self):
        if self.record_temp_timer:
            self.reactor.update_timer(self.record_temp_timer, self.reactor.NEVER)

    def temperature_callback(self, read_time, temp):
        self.last_temp = temp
        if temp:
            self.measured_min = min(self.measured_min, temp)
            self.measured_max = max(self.measured_max, temp)

    def _record_temp_callback(self, eventtime):
        if self.record_temp_file:
            self._write_to_file(self.record_temp_file, self.reactor.monotonic(), self.last_temp)

        return self.reactor.monotonic() + self.record_temp_interval

    def _write_to_file(self, filename, timestamp, temp):
        try:
            content = "%.4f,%.4f\n" % (timestamp,temp)
            queuefile.async_append_file(filename, content)
        except Exception as e:
            logging.exception(f"Failed to append to file: {filename}: {e}")

    def get_temp(self, eventtime):
        return self.last_temp, 0.
    def stats(self, eventtime):
        return False, '%s: temp=%.1f' % (self.name, self.last_temp)
    def get_status(self, eventtime):
        return {
            'temperature': round(self.last_temp, 0),
            'measured_min_temp': round(self.measured_min, 0),
            'measured_max_temp': round(self.measured_max, 0)
        }

    def cmd_RECORD_TEMPERATURE(self, gcmd):
        action = gcmd.get('ACTION', None)
        self.record_temp_interval  = gcmd.get_float('INTERVAL', 3, minval=0.5)

        try:
            record_temp_dir = pathlib.Path(f'/userdata/gcodes/{self.name}_temp')
            if not os.path.exists(record_temp_dir):
                os.makedirs(record_temp_dir)
            record_temp_file_name = f'{self.name}_{time.strftime("%m%d-%H%M")}.csv'
            self.record_temp_file = record_temp_dir.joinpath(record_temp_file_name)

            if action == "START":
                with open(self.record_temp_file, 'w+') as f:
                    f.write("time, temp\n")

                self.reactor.update_timer(self.record_temp_timer, self.reactor.NOW)
            else:
                self.reactor.update_timer(self.record_temp_timer, self.reactor.NEVER)

        except:
            self.reactor.update_timer(self.record_temp_timer, self.reactor.NEVER)

def load_config_prefix(config):
    return PrinterSensorGeneric(config)
