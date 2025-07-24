"""Compatibility wrapper for `[ams_pins]` sections."""

__all__ = ["load_config", "load_config_prefix", "AmsPins"]

# Import lazily inside the helper functions so this module can be loaded even
# if ``ams_pin`` has not finished initializing.

def load_config(config):
    from . import ams_pin as _ams_pin
    # Older implementations exported `load_config` from ``ams_pin`` but that
    # attribute may not be present if a different version is loaded.  Create
    # the pins directly to avoid errors if the helper is missing.
    if hasattr(_ams_pin, "load_config"):
        return _ams_pin.load_config(config)
    return _ams_pin.AmsPins(config)


def load_config_prefix(config):
    from . import ams_pin as _ams_pin
    if hasattr(_ams_pin, "load_config"):
        return _ams_pin.load_config(config)
    return _ams_pin.AmsPins(config)


def AmsPins(*args, **kwargs):
    from . import ams_pin as _ams_pin
    return _ams_pin.AmsPins(*args, **kwargs)


