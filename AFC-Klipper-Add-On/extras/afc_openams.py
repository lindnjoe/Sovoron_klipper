# Armored Turtle Automated Filament Control
#
# Copyright (C) 2024 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from extras.AFC import AFCOpenAMS


def load_config(config):
    return AFCOpenAMS(config)
