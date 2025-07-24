# Wrapper module providing [virtual_filament_sensor] via virtual pins
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from .filament_switch_sensor import VirtualSwitchSensor
from .ams_pin import CHIP_NAME


def load_config_prefix(config):
    """Config handler for [virtual_filament_sensor] sections."""
    pin = config.get('pin')
    if pin.startswith(CHIP_NAME + ':'):
        vpin_name = pin.split(CHIP_NAME + ':', 1)[1].strip()
    else:
        vpin_name = pin.strip()
    return VirtualSwitchSensor(config, vpin_name)
