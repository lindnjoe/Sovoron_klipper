"""Compatibility wrapper for `[ams_pins]` sections."""

__all__ = ["load_config", "load_config_prefix", "AmsPins"]

# Import lazily inside the helper functions so this module can be loaded even
# if ``ams_pin`` has not finished initializing.

