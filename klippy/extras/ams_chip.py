"""Register the 'ams' virtual chip early."""

from .ams_pin import _ensure_chip


def load_config(config):
    """Config handler for `[ams_chip]` sections."""
    printer = config.get_printer()
    chip = _ensure_chip(printer)
    return chip
