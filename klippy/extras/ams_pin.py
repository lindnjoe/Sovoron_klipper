"""Compatibility wrapper for deprecated ams_pin module.

This module preserves the old :mod:`ams_pin` name while reusing the
``virtual_input_pin`` implementation.  Config sections such as
``[ams_pin my_pin]`` will continue to work by forwarding to
``virtual_input_pin``.
"""

from .virtual_input_pin import VirtualInputPin, load_config_prefix

CHIP_NAME = 'virtual_pin'


def _norm(name: str) -> str:
    """Normalize a pin name."""
    return name.strip().lower()

__all__ = [
    'VirtualInputPin',
    'load_config_prefix',
    '_norm',
    'CHIP_NAME',
]
