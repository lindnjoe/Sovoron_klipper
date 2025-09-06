"""Klipper configuration entry point for OpenAMS integration."""

from extras.AFC import AFCOpenAMS


def load_config(config):
    """Load AFCOpenAMS so Klipper recognizes [afc_openams]."""
    return AFCOpenAMS(config)