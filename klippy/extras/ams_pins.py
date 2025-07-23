"""Compatibility wrapper for `[ams_pins]` sections."""

from .ams_pin import AmsPins, load_config

# The bare `[ams_pins]` section uses the same loader for both prefixed and
# unprefixed forms.  Export it under the names expected by Klipper's config
# loader.

load_config_prefix = load_config

