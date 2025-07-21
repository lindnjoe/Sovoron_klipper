"""Stub module for a virtual input pin."""

class VirtualInputPin:
    """Minimal placeholder implementation."""
    def __init__(self, config):
        self.printer = config.get_printer()
        self.pin_name = config.get('pin', 'virtual_pin')


def load_config(config):
    return VirtualInputPin(config)
