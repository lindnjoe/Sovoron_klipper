"""Register the ams_pin chip early for other modules.

Add a simple `[ams_pins]` section anywhere in the configuration to
pre-register the `ams_pin` chip before any other module tries to parse
pins.  This avoids unknown pin errors when modules look up
`ams_pin:<name>` before the pin sections load.
"""


def _register_chip(printer):
    """Register the ams_pin chip on demand."""
    ppins = printer.lookup_object('pins')
    if 'ams_pin' in ppins.chips:
        return
    # Import lazily to avoid potential circular imports
    from .ams_pin import AmsPinChip
    chip = AmsPinChip(printer)
    ppins.register_chip('ams_pin', chip)
    try:
        setattr(printer, 'ams_pins', chip)
    except Exception:
        pass


def _handle_config(config):
    """Shared helper for configuration loading."""
    _register_chip(config.get_printer())
    return None


def load_config(config):
    """Config handler for a bare `[ams_pins]` section."""
    return _handle_config(config)


def load_config_prefix(config):
    """Config handler for `[ams_pins <name>]` sections."""
    return _handle_config(config)
