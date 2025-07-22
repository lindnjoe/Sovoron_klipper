# Wrapper module providing [virtual_filament_sensor] via virtual pins
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from .filament_switch_sensor import VirtualSwitchSensor


def load_config_prefix(config):
    """Config handler for [virtual_filament_sensor] sections."""
    pin = config.get('pin')
    if pin.startswith('ams_pin:'):
        vpin_name = pin.split('ams_pin:', 1)[1].strip()
    else:
        vpin_name = pin.strip()
    return VirtualSwitchSensor(config, vpin_name)
