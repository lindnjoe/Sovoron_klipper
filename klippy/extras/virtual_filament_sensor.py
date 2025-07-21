"""Stub module for a virtual filament sensor."""

class VirtualFilamentSensor:
    """Minimal placeholder implementation."""
    def __init__(self, config):
        self.printer = config.get_printer()


def load_config(config):
    return VirtualFilamentSensor(config)
