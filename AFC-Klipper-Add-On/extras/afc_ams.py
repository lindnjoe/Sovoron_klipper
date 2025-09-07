# Armored Turtle Automated Filament Control
#
# Copyright (C) 2024 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import traceback

from configparser import Error as error

try:
    from extras.AFC_utils import ERROR_STR
except Exception:
    raise error("Error when trying to import AFC_utils.ERROR_STR\n{trace}".format(trace=traceback.format_exc()))

try:
    from extras.AFC_BoxTurtle import afcBoxTurtle
except Exception:
    raise error(ERROR_STR.format(import_lib="AFC_BoxTurtle", trace=traceback.format_exc()))


class afcAMS(afcBoxTurtle):
    """AFC unit that sources lane and hub states from OpenAMS."""

    def __init__(self, config):
        super().__init__(config)
        self.type = config.get('type', 'AMS')
        self.afc.add_openams(config)


def load_config_prefix(config):
    return afcAMS(config)


def load_config(config):
    return afcAMS(config)