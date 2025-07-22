from .virtual_pin import VirtualFilamentSensor, load_config_prefix_filament_switch_sensor


def load_config_prefix(config):
    return load_config_prefix_filament_switch_sensor(config)
