"""Wrapper for `[ams_pins]` sections."""

__all__ = ["AmsPins", "load_config", "load_config_prefix"]

def _import():
    from . import ams_pin as _ams_pin
    return _ams_pin

def load_config(config):
    mod = _import()
    return mod.AmsPins(config)

def load_config_prefix(config):
    mod = _import()
    return mod.load_config_prefix(config)

class AmsPins:
    pass  # placeholder for type checkers; real class imported dynamically
