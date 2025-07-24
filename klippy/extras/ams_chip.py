"""Register the 'ams' virtual pin chip early."""

from .virtual_input_pin import _ensure_chip


def load_config(config):
    _ensure_chip(config.get_printer())
    # no printer objects to return
    return None

# support [ams_chip] and [ams_chip whatever]
def load_config_prefix(config):
    return load_config(config)
