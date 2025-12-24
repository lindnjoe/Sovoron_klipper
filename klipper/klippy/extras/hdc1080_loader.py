import logging


class HDC1080Loader:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.logger = logging.getLogger("hdc1080_loader")
        self.printer.register_event_handler("klippy:ready", self._register_sensor)

    def _register_sensor(self):
        # Register HDC1080 sensor factory with heaters when Klipper is ready
        try:
            from . import hdc1080

            pheater = self.printer.lookup_object("heaters")
            pheater.add_sensor_factory("HDC1080", hdc1080.HDC1080)
            self.logger.info("HDC1080 temperature sensor registered successfully")
        except Exception as exc:
            self.logger.error("Failed to register HDC1080 sensor: %s", exc)
            raise


def load_config(config):
    return HDC1080Loader(config)