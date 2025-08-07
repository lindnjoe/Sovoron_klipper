"""Virtual endstop object backed by a virtual input pin."""


class VirtualEndstop:
    """Simple endstop object backed by a virtual pin."""

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
        return 0.0

    def query_endstop(self, print_time):
        return bool(self._vpin.state) ^ bool(self._invert)


def load_config(config):
    """Instantiate a VirtualEndstop from config."""
    vpin_name = config.get('pin')
    invert = config.getboolean('invert', False)
    printer = config.get_printer()
    vpin = printer.lookup_object('virtual_pin ' + vpin_name, None)
    if vpin is None:
        raise config.error(f'virtual pin {vpin_name} not configured')
    return VirtualEndstop(vpin, invert)
