"""Compatibility wrapper for `[ams_pins]` sections."""

__all__ = ["load_config", "load_config_prefix", "AmsPins"]

# Import lazily inside the helper functions so this module can be loaded even
# if ``ams_pin`` has not finished initializing.

def load_config(config):
    from . import ams_pin as _ams_pin
    return _ams_pin.load_config(config)


def load_config_prefix(config):
    from . import ams_pin as _ams_pin
    return _ams_pin.load_config(config)


def AmsPins(*args, **kwargs):
    from . import ams_pin as _ams_pin
    return _ams_pin.AmsPins(*args, **kwargs)


