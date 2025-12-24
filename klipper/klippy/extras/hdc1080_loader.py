import logging


class HDC1080Loader:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.config = config
        self.logger = logging.getLogger("hdc1080_loader")
        self._registered = False

        # Register immediately so the sensor type is available during config
        # validation, and register again on ready in case Klipper reloads.
        self._register_sensor(initial=True)
        self.printer.register_event_handler("klippy:ready", self._register_sensor)

    def _register_sensor(self, initial=False):
        if self._registered:
            return
        try:
            from . import hdc1080

            pheater = self.printer.lookup_object("heaters")
            pheater.add_sensor_factory("HDC1080", hdc1080.HDC1080)
            self._registered = True
            self.logger.info("HDC1080 temperature sensor registered successfully")
        except Exception as exc:
            self.logger.error("Failed to register HDC1080 sensor: %s", exc)
            if initial:
                raise self.config.error(
                    f"HDC1080 sensor registration failed: {exc}"
                )
            raise


def load_config(config):
    return HDC1080Loader(config)