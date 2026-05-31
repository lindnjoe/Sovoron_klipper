# Fast input shaper calibration using narrow frequency band around current settings
class AFCFastShaper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.delta_freq = config.getfloat('delta_freq', 10.0, minval=5.0)
        self.min_freq = config.getfloat('min_freq', 5.0)
        self.max_freq = config.getfloat('max_freq', 200.0)
        self.gcode.register_command(
            'FAST_SHAPER_CALIBRATE', self.cmd_FAST_SHAPER_CALIBRATE,
            desc="Run narrow-band shaper calibration around current frequencies")

    def cmd_FAST_SHAPER_CALIBRATE(self, gcmd):
        input_shaper = self.printer.lookup_object('input_shaper', None)
        if input_shaper is None:
            gcmd.respond_info(
                "Error: [input_shaper] not found. "
                "Add [input_shaper] to your config first.")
            return

        delta = gcmd.get_float('DELTA_FREQ', self.delta_freq, minval=5.0)

        for axis_idx, axis_name in enumerate(['x', 'y']):
            shaper = input_shaper.shapers[axis_idx]
            current_freq = shaper.params.shaper_freq
            shaper_type = shaper.params.shaper_type

            if current_freq == 0:
                gcmd.respond_info(
                    "Error: %s axis shaper_freq is 0. "
                    "Run full SHAPER_CALIBRATE first." % axis_name.upper())
                return

            freq_start = current_freq - abs(delta)
            if freq_start < self.min_freq:
                freq_start = self.min_freq
            freq_end = current_freq + abs(delta)
            if freq_end > self.max_freq:
                freq_end = self.max_freq

            gcmd.respond_info(
                "FAST_SHAPER_CALIBRATE: axis=%s type=%s freq_range=%.1f-%.1f"
                % (axis_name.upper(), shaper_type, freq_start, freq_end))

            command = ("SHAPER_CALIBRATE AXIS=%s SHAPER_TYPES=%s "
                       "FREQ_START=%d FREQ_END=%d"
                       % (axis_name, shaper_type, freq_start, freq_end))
            self.gcode.run_script_from_command(command)

def load_config(config):
    return AFCFastShaper(config)
