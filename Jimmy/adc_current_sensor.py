class ADCCurrentSensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]  # Get the last part of config name

        # Current sensing parameters
        self.sense_resistor = config.getfloat('sense_resistor', above=0.0)
        self.scale = config.getfloat('scale', 1.0, above=0.0)
        self.adc_reference = config.getfloat('adc_reference_voltage', 3.3, above=0.0)
        self.voltage_offset = config.getfloat('voltage_offset', 0.0)
        self.last_current = None

        # Setup ADC pin
        ppins = self.printer.lookup_object('pins')
        self.mcu_adc = ppins.setup_pin('adc', config.get('pin'))
        self.report_time = config.getfloat('report_time', 0.300, above=0.0)
        self.mcu_adc.setup_adc_sample(
            config.getfloat('sample_time', 0.001, above=0.0),
            config.getint('sample_count', 8, minval=1))
        self.mcu_adc.setup_adc_callback(self.report_time, self.adc_callback)

        # Register mux command
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command(
            "QUERY_ADC_CURRENT", "SENSOR", self.name,
            self.cmd_QUERY_ADC_CURRENT,
            desc=self.cmd_QUERY_ADC_CURRENT_help)

    def adc_callback(self, read_time, read_value):
        """Convert ADC voltage to current using Ohm's law"""
        voltage = read_value * self.adc_reference  # read_value is already normalized (0-1)
        self.last_current = round((voltage + self.voltage_offset) / (self.sense_resistor * self.scale), 3)

    def get_status(self, eventtime):
        """Return sensor status dictionary"""
        return {
            'current': self.last_current,
            'sense_resistor': self.sense_resistor,
            'adc_reference': self.adc_reference,
            'voltage_offset': self.voltage_offset,
            'scale': self.scale
        }

    def stats(self, eventtime):
        current_val = self.last_current if self.last_current is not None else "N/A"
        return False, '{}: current={}'.format(self.name, current_val)

    cmd_QUERY_ADC_CURRENT_help = "Query current ADC current reading"
    def cmd_QUERY_ADC_CURRENT(self, gcmd):
        gcmd.respond_info("Sensor %s current: %.3fA (Sense resistor: %.3fΩ, Reference voltage: %.1fV)" % (
            self.name, self.last_current, self.sense_resistor, self.adc_reference))

def load_config_prefix(config):
    return ADCCurrentSensor(config)
