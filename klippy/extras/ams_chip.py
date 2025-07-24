"""Register the AMS virtual pin chip early.

This optional module creates the virtual `ams` chip before other modules
reference it. Include `[ams_chip]` early in your configuration if you
plan to refer to `ams:` pins in sections that Klipper parses before
`[ams_pin]` or `[ams_pins]` sections are loaded.
"""

from .ams_pin import _ensure_chip


def load_config(config):
    printer = config.get_printer()
    _ensure_chip(printer)
    # no objects need to be returned
    return None

# allow `[ams_chip something]` style
load_config_prefix = load_config
