"""Register the ams_pin chip early for other modules.

Add a simple `[ams_pins]` section anywhere in the configuration to
pre-register the `ams_pin` chip before any other module tries to parse
pins.  This avoids unknown pin errors when modules look up
`ams_pin:<name>` before the pin sections load.
"""

from .ams_pin import _ensure_chip


def load_config(config):
    """Config handler for a bare [ams_pins] section."""
    _ensure_chip(config.get_printer())
    return None
