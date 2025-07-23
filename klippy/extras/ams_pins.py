"""Compatibility wrapper for `[ams_pins]` sections."""

__all__ = ["load_config", "load_config_prefix", "AmsPins"]

# Import lazily to avoid problems if ams_pin hasn't finished loading yet
from . import ams_pin as _ams_pin

AmsPins = _ams_pin.AmsPins
load_config = _ams_pin.load_config

# The bare `[ams_pins]` section uses the same loader for both prefixed and
# unprefixed forms.  Export it under the names expected by Klipper's config
# loader.

load_config_prefix = load_config

