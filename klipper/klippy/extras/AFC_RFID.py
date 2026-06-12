# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Common RFID infrastructure for AFC. Provides shared utilities for
# color lookup, material density, Spoolman sync, and filament defaults.
# Implementation-specific readers (U1, ACE) import from here.

from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from extras.AFC_lane import AFCLane

# Max RGB Euclidean distance for two spool colours to be the SAME filament.
# 0.0 = exact hex match (different colours -> different spools). Raise a little
# (e.g. 2-3) only if you want to tolerate minor rounding between scans.
COLOR_MATCH_TOLERANCE = 0.0

_COLOR_NAMES = {
    "000000": "Black", "000066": "Dark Blue", "000080": "Navy",
    "00008b": "Dark Blue", "000099": "Dark Blue", "0000cc": "Blue",
    "0000cd": "Medium Blue", "0000ee": "Blue", "0000ff": "Blue",
    "002080": "Dark Azure", "003300": "Dark Green", "003333": "Dark Teal",
    "003366": "Dark Blue", "0033cc": "Deep Azure", "004080": "Dark Cerulean",
    "0047ab": "Cobalt", "004d4d": "Dark Teal", "006080": "Dark Sky Blue",
    "006400": "Dark Green", "006600": "Dark Green", "0066cc": "Deep Cerulean",
    "007f80": "Dark Cyan", "008000": "Green", "008020": "Dark Emerald",
    "008040": "Dark Mint", "008060": "Dark Teal", "008080": "Teal",
    "008b8b": "Teal", "0099cc": "Deep Sky Blue", "00a86b": "Jade",
    "00bfff": "Deep Sky Blue", "00cc00": "Green", "00cc33": "Deep Emerald",
    "00cc66": "Deep Mint", "00cc99": "Deep Teal", "00cccc": "Deep Cyan",
    "00ced1": "Dark Turquoise", "00dd00": "Green", "00ee00": "Green",
    "00eeee": "Cyan", "00fa9a": "Medium Spring Green", "00ff00": "Green",
    "00ff7f": "Mint", "00ffff": "Cyan", "0a0a0a": "Black",
    "0f52ba": "Sapphire", "141414": "Black", "161669": "Dark Soft Blue",
    "162b69": "Dark Soft Azure", "164069": "Dark Soft Cerulean", "165469": "Dark Soft Sky Blue",
    "166916": "Dark Soft Green", "16692b": "Dark Soft Emerald", "166940": "Dark Soft Mint",
    "166954": "Dark Soft Teal", "166969": "Dark Soft Cyan", "171717": "Matte Black",
    "191970": "Midnight Blue", "1a1a1a": "Black", "1a1aff": "Blue",
    "1a3300": "Dark Green", "1a53ff": "Azure", "1a8cff": "Cerulean",
    "1ac6ff": "Sky Blue", "1aff1a": "Green", "1aff53": "Emerald",
    "1aff8c": "Mint", "1affc6": "Teal", "1affff": "Cyan",
    "1e90ff": "Dodger Blue", "200080": "Dark Indigo", "202020": "Black",
    "208000": "Dark Spring Green", "20b2aa": "Teal", "228b22": "Forest Green",
    "2424a8": "Deep Soft Blue", "2445a8": "Deep Soft Azure", "2466a8": "Deep Soft Cerulean",
    "2487a8": "Deep Soft Sky Blue", "24a824": "Deep Soft Green", "24a845": "Deep Soft Emerald",
    "24a866": "Deep Soft Mint", "24a887": "Deep Soft Teal", "24a8a8": "Deep Soft Cyan",
    "292956": "Dark Muted Blue", "293556": "Dark Muted Azure", "294056": "Dark Muted Cerulean",
    "294b56": "Dark Muted Sky Blue", "295629": "Dark Muted Green", "295635": "Dark Muted Emerald",
    "295640": "Dark Muted Mint", "29564b": "Dark Muted Teal", "295656": "Dark Muted Cyan",
    "2a3439": "Gunmetal", "2b1669": "Dark Soft Indigo", "2b6916": "Dark Soft Spring Green",
    "2c1a0e": "Dark Brown", "2e2e2e": "Charcoal", "2e8b57": "Sea Green",
    "2f4f4f": "Dark Slate Gray", "32cd32": "Lime", "330000": "Maroon",
    "330033": "Dark Purple", "3300cc": "Deep Indigo", "333300": "Dark Olive",
    "333333": "Dark Gray", "33cc00": "Deep Spring Green", "352956": "Dark Muted Indigo",
    "355629": "Dark Muted Spring Green", "36454f": "Charcoal", "383838": "Graphite",
    "3b1e08": "Dark Brown", "3cb371": "Sea Green", "3eb489": "Mint",
    "3f231c": "Dark Brown", "400080": "Dark Violet", "401669": "Dark Soft Violet",
    "402956": "Dark Muted Violet", "405629": "Dark Muted Chartreuse", "406916": "Dark Soft Chartreuse",
    "408000": "Dark Chartreuse", "40e0d0": "Turquoise", "4169e1": "Royal Blue",
    "42428a": "Deep Muted Blue", "4242d7": "Soft Blue", "42548a": "Deep Muted Azure",
    "42668a": "Deep Muted Cerulean", "4267d7": "Soft Azure", "42788a": "Deep Muted Sky Blue",
    "428a42": "Deep Muted Green", "428a54": "Deep Muted Emerald", "428a66": "Deep Muted Mint",
    "428a78": "Deep Muted Teal", "428a8a": "Deep Muted Cyan", "428cd7": "Soft Cerulean",
    "42b2d7": "Soft Sky Blue", "42d742": "Soft Green", "42d767": "Soft Emerald",
    "42d78c": "Soft Mint", "42d7b2": "Soft Teal", "42d7d7": "Soft Cyan",
    "4524a8": "Deep Soft Indigo", "454545": "Dark Gray", "45a824": "Deep Soft Spring Green",
    "4682b4": "Steel Blue", "483c32": "Taupe", "483d8b": "Dark Slate Blue",
    "48d1cc": "Turquoise", "4a0066": "Dark Purple", "4a0100": "Mahogany",
    "4a2812": "Dark Brown", "4b0082": "Indigo", "4b2956": "Dark Muted Purple",
    "4b3621": "Espresso", "4b5629": "Dark Muted Lime", "4d0000": "Dark Red",
    "4d4d4d": "Dark Gray", "50c878": "Emerald", "531aff": "Indigo",
    "53ff1a": "Spring Green", "541669": "Dark Soft Purple", "54428a": "Deep Muted Indigo",
    "546916": "Dark Soft Lime", "548a42": "Deep Muted Spring Green", "556b2f": "Olive",
    "562929": "Dark Muted Red", "562935": "Dark Muted Rose", "562940": "Dark Muted Pink",
    "56294b": "Dark Muted Fuchsia", "562956": "Dark Muted Magenta", "563529": "Dark Muted Scarlet",
    "564029": "Dark Muted Orange", "564b29": "Dark Muted Amber", "565629": "Dark Muted Yellow",
    "5c3a1e": "Dark Brown", "5c5c5c": "Dim Gray", "5e1414": "Wine Red",
    "5f9ea0": "Cadet Blue", "600080": "Dark Purple", "608000": "Dark Lime",
    "6464b4": "Muted Blue", "6478b4": "Muted Azure", "648cb4": "Muted Cerulean",
    "6495ed": "Cornflower Blue", "64a0b4": "Muted Sky Blue", "64b464": "Muted Green",
    "64b478": "Muted Emerald", "64b48c": "Muted Mint", "64b4a0": "Muted Teal",
    "64b4b4": "Muted Cyan", "654321": "Brown", "660000": "Dark Red",
    "660066": "Purple", "6600cc": "Deep Violet", "6624a8": "Deep Soft Violet",
    "663399": "Rebecca Purple", "66428a": "Deep Muted Violet", "666666": "Dark Gray",
    "6666ff": "Light Blue", "668a42": "Deep Muted Chartreuse", "668cff": "Light Azure",
    "66a824": "Deep Soft Chartreuse", "66b2ff": "Light Cerulean", "66cc00": "Deep Chartreuse",
    "66cdaa": "Medium Aquamarine", "66d9ff": "Light Sky Blue", "66ff66": "Light Green",
    "66ff8c": "Light Emerald", "66ffb3": "Light Mint", "66ffd9": "Light Teal",
    "66ffff": "Light Cyan", "6742d7": "Soft Indigo", "67d742": "Soft Spring Green",
    "691616": "Dark Soft Red", "69162b": "Dark Soft Rose", "691640": "Dark Soft Pink",
    "691654": "Dark Soft Fuchsia", "691669": "Dark Soft Magenta", "692b16": "Dark Soft Scarlet",
    "694016": "Dark Soft Orange", "695416": "Dark Soft Amber", "696916": "Dark Soft Yellow",
    "696969": "Dim Gray", "6a5acd": "Slate Blue", "6b3a2a": "Brown",
    "6b8e23": "Olive Drab", "6d071a": "Burgundy", "6e7479": "Nardo Gray",
    "6f2da8": "Grape", "6f4e37": "Coffee", "704214": "Sepia",
    "708090": "Slate Gray", "737373": "Gray", "778899": "Slate Gray",
    "78428a": "Deep Muted Purple", "7864b4": "Muted Indigo", "788a42": "Deep Muted Lime",
    "78b464": "Muted Spring Green", "7b3f00": "Chocolate", "7b68ee": "Slate Blue",
    "7cfc00": "Lawn Green", "7f8000": "Dark Yellow", "7fff00": "Chartreuse",
    "7fffd4": "Aquamarine", "800000": "Maroon", "800020": "Dark Rose",
    "800040": "Dark Pink", "800060": "Dark Fuchsia", "80007f": "Dark Magenta",
    "800080": "Purple", "801010": "Dark Red", "802000": "Dark Scarlet",
    "804000": "Dark Orange", "806000": "Dark Amber", "808000": "Olive",
    "808080": "Gray", "8181e4": "Light Soft Blue", "819ae4": "Light Soft Azure",
    "81b2e4": "Light Soft Cerulean", "81cbe4": "Light Soft Sky Blue", "81e481": "Light Soft Green",
    "81e49a": "Light Soft Emerald", "81e4b3": "Light Soft Mint", "81e4cb": "Light Soft Teal",
    "81e4e4": "Light Soft Cyan", "8724a8": "Deep Soft Purple", "87a824": "Deep Soft Lime",
    "87ceeb": "Sky Blue", "87cefa": "Light Sky Blue", "8a2be2": "Blue Violet",
    "8a4242": "Deep Muted Red", "8a4254": "Deep Muted Rose", "8a4266": "Deep Muted Pink",
    "8a4278": "Deep Muted Fuchsia", "8a428a": "Deep Muted Magenta", "8a5442": "Deep Muted Scarlet",
    "8a6642": "Deep Muted Orange", "8a7842": "Deep Muted Amber", "8a8a42": "Deep Muted Yellow",
    "8a9a9a": "Pewter", "8b0000": "Dark Red", "8b008b": "Dark Magenta",
    "8b4513": "Brown", "8c1aff": "Violet", "8c42d7": "Soft Violet",
    "8c64b4": "Muted Violet", "8c66ff": "Light Indigo", "8c8c8c": "Medium Gray",
    "8cb464": "Muted Chartreuse", "8cd742": "Soft Chartreuse", "8cff1a": "Chartreuse",
    "8cff66": "Light Spring Green", "8fbc8f": "Dark Sea Green", "90ee90": "Light Green",
    "9370db": "Medium Purple", "9400d3": "Dark Violet", "967bb6": "Lavender Purple",
    "9898cd": "Light Muted Blue", "98a5cd": "Light Muted Azure", "98b2cd": "Light Muted Cerulean",
    "98c0cd": "Light Muted Sky Blue", "98cd98": "Light Muted Green", "98cda5": "Light Muted Emerald",
    "98cdb2": "Light Muted Mint", "98cdc0": "Light Muted Teal", "98cdcd": "Light Muted Cyan",
    "98fb98": "Pale Green", "990000": "Dark Red", "9900cc": "Purple",
    "9932cc": "Dark Orchid", "993d00": "Dark Orange", "998200": "Dark Gold",
    "999999": "Gray", "99cc00": "Deep Lime", "9a81e4": "Light Soft Indigo",
    "9acd32": "Yellow Green", "9ae481": "Light Soft Spring Green", "9fe2bf": "Seafoam",
    "a01818": "Dark Red", "a0522d": "Sienna", "a064b4": "Muted Purple",
    "a0a0a0": "Gray", "a0b464": "Muted Lime", "a3a3a3": "Slate Gray",
    "a52a2a": "Brown", "a598cd": "Light Muted Indigo", "a5cd98": "Light Muted Spring Green",
    "a82424": "Deep Soft Red", "a82445": "Deep Soft Rose", "a82466": "Deep Soft Pink",
    "a82487": "Deep Soft Fuchsia", "a824a8": "Deep Soft Magenta", "a84524": "Deep Soft Scarlet",
    "a86624": "Deep Soft Orange", "a88724": "Deep Soft Amber", "a8a824": "Deep Soft Yellow",
    "a9a9a9": "Silver", "add8e6": "Light Blue", "adff2f": "Green Yellow",
    "af6e4d": "Caramel", "afeeee": "Pale Turquoise", "b02018": "Red",
    "b0b0b0": "Silver", "b0c4de": "Light Steel Blue", "b0e0e6": "Powder Blue",
    "b22222": "Firebrick", "b242d7": "Soft Purple", "b266ff": "Light Violet",
    "b281e4": "Light Soft Violet", "b298cd": "Light Muted Violet", "b2b2ff": "Pale Blue",
    "b2c6ff": "Pale Azure", "b2cd98": "Light Muted Chartreuse", "b2d742": "Soft Lime",
    "b2d9ff": "Pale Cerulean", "b2ecff": "Pale Sky Blue", "b2ffb2": "Pale Green",
    "b2ffc6": "Pale Emerald", "b2ffd9": "Pale Mint", "b2ffec": "Pale Teal",
    "b2ffff": "Pale Cyan", "b3e481": "Light Soft Chartreuse", "b3ff66": "Light Chartreuse",
    "b46464": "Muted Red", "b46478": "Muted Rose", "b4648c": "Muted Pink",
    "b464a0": "Muted Fuchsia", "b464b4": "Muted Magenta", "b47864": "Muted Scarlet",
    "b48c64": "Muted Orange", "b4a064": "Muted Amber", "b4b464": "Muted Yellow",
    "b5a642": "Brass", "b7410e": "Rust", "b76e79": "Rose Gold",
    "b87333": "Copper", "b8860b": "Dark Goldenrod", "ba55d3": "Medium Orchid",
    "bababa": "Gray", "bc8f8f": "Rosy Brown", "bdb76b": "Dark Khaki",
    "c03020": "Red", "c098cd": "Light Muted Purple", "c0c0c0": "Silver",
    "c0c0f2": "Pale Soft Blue", "c0ccf2": "Pale Soft Azure", "c0cd98": "Light Muted Lime",
    "c0d9f2": "Pale Soft Cerulean", "c0e5f2": "Pale Soft Sky Blue", "c0f2c0": "Pale Soft Green",
    "c0f2cc": "Pale Soft Emerald", "c0f2d9": "Pale Soft Mint", "c0f2e5": "Pale Soft Teal",
    "c0f2f2": "Pale Soft Cyan", "c61aff": "Purple", "c66b3d": "Terracotta",
    "c6b2ff": "Pale Indigo", "c6ff1a": "Lime", "c6ffb2": "Pale Spring Green",
    "c71585": "Magenta", "c8a2c8": "Lilac", "cb81e4": "Light Soft Purple",
    "cbcbe6": "Pale Muted Blue", "cbd2e6": "Pale Muted Azure", "cbd9e6": "Pale Muted Cerulean",
    "cbdfe6": "Pale Muted Sky Blue", "cbe481": "Light Soft Lime", "cbe6cb": "Pale Muted Green",
    "cbe6d2": "Pale Muted Emerald", "cbe6d9": "Pale Muted Mint", "cbe6df": "Pale Muted Teal",
    "cbe6e6": "Pale Muted Cyan", "cc0000": "Red", "cc0033": "Deep Rose",
    "cc0066": "Deep Pink", "cc0099": "Deep Fuchsia", "cc00cc": "Magenta",
    "cc3300": "Deep Scarlet", "cc3333": "Red", "cc5500": "Dark Orange",
    "cc6600": "Deep Orange", "cc9900": "Deep Amber", "ccc0f2": "Pale Soft Indigo",
    "cccc00": "Yellow", "ccccff": "Periwinkle", "ccf2c0": "Pale Soft Spring Green",
    "cd5c5c": "Indian Red", "cd7f32": "Bronze", "cd853f": "Peru",
    "cd9898": "Light Muted Red", "cd98a5": "Light Muted Rose", "cd98b2": "Light Muted Pink",
    "cd98c0": "Light Muted Fuchsia", "cd98cd": "Light Muted Magenta", "cda598": "Light Muted Scarlet",
    "cdb298": "Light Muted Orange", "cdc098": "Light Muted Amber", "cdcd98": "Light Muted Yellow",
    "d1d1d1": "Light Gray", "d2691e": "Chocolate", "d2b48c": "Tan",
    "d2cbe6": "Pale Muted Indigo", "d2e6cb": "Pale Muted Spring Green", "d3d3d3": "Light Gray",
    "d74242": "Soft Red", "d74267": "Soft Rose", "d7428c": "Soft Pink",
    "d742b2": "Soft Fuchsia", "d742d7": "Soft Magenta", "d76742": "Soft Scarlet",
    "d78c42": "Soft Orange", "d7b242": "Soft Amber", "d7d742": "Soft Yellow",
    "d8bfd8": "Thistle", "d966ff": "Light Purple", "d9b2ff": "Pale Violet",
    "d9c0f2": "Pale Soft Violet", "d9cbe6": "Pale Muted Violet", "d9e6cb": "Pale Muted Chartreuse",
    "d9f2c0": "Pale Soft Chartreuse", "d9ff66": "Light Lime", "d9ffb2": "Pale Chartreuse",
    "da70d6": "Orchid", "daa520": "Goldenrod", "db7093": "Pale Violet Red",
    "dc143c": "Crimson", "dcdcdc": "Gainsboro", "dd0000": "Red",
    "dda0dd": "Plum", "de3163": "Cerise", "deb887": "Burlywood",
    "dfcbe6": "Pale Muted Purple", "dfe6cb": "Pale Muted Lime", "e02020": "Red",
    "e0c79b": "Sand", "e0e0e0": "Light Gray", "e0ffff": "Light Cyan",
    "e1ad01": "Mustard", "e30b5d": "Raspberry", "e3dac9": "Bone",
    "e48181": "Light Soft Red", "e4819a": "Light Soft Rose", "e481b3": "Light Soft Pink",
    "e481cb": "Light Soft Fuchsia", "e481e4": "Light Soft Magenta", "e49a81": "Light Soft Scarlet",
    "e4b281": "Light Soft Orange", "e4cb81": "Light Soft Amber", "e4e481": "Light Soft Yellow",
    "e5c0f2": "Pale Soft Purple", "e5f2c0": "Pale Soft Lime", "e6cbcb": "Pale Muted Red",
    "e6cbd2": "Pale Muted Rose", "e6cbd9": "Pale Muted Pink", "e6cbdf": "Pale Muted Fuchsia",
    "e6cbe6": "Pale Muted Magenta", "e6d2cb": "Pale Muted Scarlet", "e6d9cb": "Pale Muted Orange",
    "e6dfcb": "Pale Muted Amber", "e6e6cb": "Pale Muted Yellow", "e6e6e6": "Silver",
    "e6e6fa": "Lavender", "e9967a": "Salmon", "ecb2ff": "Pale Purple",
    "ecffb2": "Pale Lime", "ee0000": "Red", "ee00ee": "Magenta",
    "ee82ee": "Violet", "eee8aa": "Pale Goldenrod", "efe6d2": "Natural",
    "f08080": "Light Coral", "f0e68c": "Khaki", "f0f0f0": "White",
    "f0f8ff": "Alice Blue", "f0ffff": "Azure", "f2c0c0": "Pale Soft Red",
    "f2c0cc": "Pale Soft Rose", "f2c0d9": "Pale Soft Pink", "f2c0e5": "Pale Soft Fuchsia",
    "f2c0f2": "Pale Soft Magenta", "f2ccc0": "Pale Soft Scarlet", "f2d9c0": "Pale Soft Orange",
    "f2e5c0": "Pale Soft Amber", "f2f2c0": "Pale Soft Yellow", "f2f2f2": "White Smoke",
    "f4a460": "Sandy Brown", "f5deb3": "Wheat", "f5f5dc": "Beige",
    "f5f5f5": "White", "f5fffa": "Mint Cream", "f7e7ce": "Champagne",
    "fa8072": "Salmon", "faebd7": "Antique White", "fafafa": "White",
    "ff0000": "Red", "ff00ff": "Magenta", "ff1493": "Hot Pink",
    "ff1a1a": "Red", "ff1a53": "Rose", "ff1a8c": "Pink",
    "ff1ac6": "Fuchsia", "ff1aff": "Magenta", "ff4500": "Orange Red",
    "ff531a": "Scarlet", "ff6347": "Tomato", "ff6600": "Orange",
    "ff6666": "Light Red", "ff668c": "Light Rose", "ff66b3": "Light Pink",
    "ff66d9": "Light Fuchsia", "ff66ff": "Light Magenta", "ff69b4": "Hot Pink",
    "ff7f50": "Coral", "ff8000": "Orange", "ff8c00": "Dark Orange",
    "ff8c1a": "Orange", "ff8c66": "Light Scarlet", "ff91a4": "Pink",
    "ffa07a": "Light Salmon", "ffa500": "Orange", "ffb266": "Light Orange",
    "ffb2b2": "Pale Red", "ffb2c6": "Pale Rose", "ffb2d9": "Pale Pink",
    "ffb2ec": "Pale Fuchsia", "ffb2ff": "Pale Magenta", "ffb6c1": "Light Pink",
    "ffbf00": "Amber", "ffc0cb": "Pink", "ffc61a": "Amber",
    "ffc6b2": "Pale Scarlet", "ffd700": "Gold", "ffd966": "Light Amber",
    "ffd9b2": "Pale Orange", "ffdab9": "Peach", "ffdead": "Navajo White",
    "ffe4b5": "Moccasin", "ffe4c4": "Bisque", "ffe4e1": "Misty Rose",
    "ffecb2": "Pale Amber", "ffee00": "Yellow", "fff0f5": "Blush",
    "fff44f": "Lemon Yellow", "fff8dc": "Cornsilk", "fffacd": "Lemon",
    "fffdd0": "Cream", "ffff00": "Yellow", "ffff1a": "Yellow",
    "ffff66": "Light Yellow", "ffffb2": "Pale Yellow", "ffffe0": "Light Yellow",
    "fffff0": "Ivory", "ffffff": "White",
}

MATERIAL_DENSITY = {
    'pla':     1.24,
    'plahf':   1.24,
    'pla+':    1.24,
    'plapro':  1.24,
    'plasilk': 1.24,
    'plamatte':1.24,
    'placf':   1.30,
    'plagf':   1.30,
    'petg':    1.27,
    'pet':     1.34,
    'petgcf':  1.32,
    'petgf':   1.32,
    'tpu':     1.21,
    'tpu95':   1.21,
    'tpu85':   1.18,
    'abs':     1.04,
    'abscf':   1.13,
    'absgf':   1.13,
    'asa':     1.07,
    'asacf':   1.14,
    'pc':      1.20,
    'pccf':    1.28,
    'pa':      1.13,
    'nylon':   1.13,
    'pacf':    1.16,
    'pagf':    1.20,
    'pa6':     1.14,
    'pa6cf':   1.20,
    'pa12':    1.01,
    'peek':    1.30,
    'pps':     1.34,
    'ppscf':   1.42,
    'hips':    1.04,
    'pva':     1.23,
    'bvoh':    1.10,
}


def color_name(hex_str: str) -> str:
    """Return nearest human-readable color name for a hex color string."""
    if not hex_str or len(hex_str) < 6:
        return ""
    h = hex_str.strip().lstrip("#").lower()[:6]
    try:
        r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
    except ValueError:
        return ""
    best_name = ""
    best_dist = float('inf')
    for ref_hex, name in _COLOR_NAMES.items():
        rr, gg, bb = int(ref_hex[0:2], 16), int(ref_hex[2:4], 16), int(ref_hex[4:6], 16)
        dist = ((r - rr) ** 2 + (g - gg) ** 2 + (b - bb) ** 2) ** 0.5
        if dist < best_dist:
            best_dist = dist
            best_name = name
    return best_name


def color_label(colors) -> str:
    """Human-readable label for one or more hex colours.

    Single colour -> 'Pale Muted Rose #e7c1d5'.
    Dual/multi    -> 'Pale Muted Rose #e7c1d5 + Bisque #ffe9c9' (dual-colour).
    Accepts a single hex string or a list of hex strings.
    """
    if isinstance(colors, str):
        colors = [colors] if colors else []
    parts = []
    for c in colors:
        c = (c or "").lstrip("#").lower()
        if not c:
            continue
        nm = color_name(c)
        parts.append(f"{nm} #{c}" if nm else f"#{c}")
    label = " + ".join(parts)
    if len(parts) > 1:
        label += " (dual-colour)" if len(parts) == 2 else " (multi-colour)"
    return label


def color_distance(hex1: str, hex2: str) -> float:
    """Euclidean RGB distance between two hex color strings."""
    r1, g1, b1 = int(hex1[0:2], 16), int(hex1[2:4], 16), int(hex1[4:6], 16)
    r2, g2, b2 = int(hex2[0:2], 16), int(hex2[2:4], 16), int(hex2[4:6], 16)
    return ((r1 - r2) ** 2 + (g1 - g2) ** 2 + (b1 - b2) ** 2) ** 0.5


def colors_match(scanned, candidate, tol=0.0) -> bool:
    """True if two colour lists are the same set within ``tol`` (order-free).

    Used for exact spool identity: a single-colour [a] only matches [a], a
    dual-colour [a,b] only matches another [a,b] (either order). tol=0 = exact.
    """
    if len(scanned) != len(candidate):
        return False
    remaining = list(candidate)
    for c in scanned:
        hit = None
        for i, d in enumerate(remaining):
            try:
                if color_distance(c, d) <= tol:
                    hit = i
                    break
            except (ValueError, IndexError):
                continue
        if hit is None:
            return False
        remaining.pop(hit)
    return True


def density_for_material(material: str) -> float:
    """Return Spoolman density (g/cm^3) for a material string.
    Strips spaces, dashes, underscores so 'PLA-CF', 'pla cf', 'pla_cf'
    all match 'placf'. Falls back to PLA (1.24) for unknown materials."""
    if not material:
        return 1.24
    key = material.strip().lower()
    for ch in (' ', '-', '_', '/'):
        key = key.replace(ch, '')
    if key in MATERIAL_DENSITY:
        return MATERIAL_DENSITY[key]
    for k in sorted(MATERIAL_DENSITY, key=len, reverse=True):
        if key.startswith(k):
            return MATERIAL_DENSITY[k]
    return 1.24


def rgb_array_to_hex(color_array) -> str:
    """Convert an [r, g, b] array to '#rrggbb' hex string."""
    if isinstance(color_array, (list, tuple)) and len(color_array) >= 3:
        r, g, b = int(color_array[0]), int(color_array[1]), int(color_array[2])
        return f"#{r:02x}{g:02x}{b:02x}"
    return "#000000"


def log_new_filament(logger, prefix, filament, brand, material, color_hex,
                     diameter, ext_temp, bed_temp, sku=""):
    """Log a detailed breakdown when a new filament is created in Spoolman."""
    fid = filament.get("id", "?")
    parts = [f"{prefix}: created filament #{fid} in Spoolman:"]
    if brand:
        parts.append(f"  vendor: {brand}")
    if material:
        parts.append(f"  material: {material}")
    if color_hex:
        cname = color_name(color_hex)
        clabel = f"{cname} #{color_hex}" if cname else f"#{color_hex}"
        parts.append(f"  color: {clabel}")
    parts.append(f"  diameter: {diameter}mm")
    if ext_temp:
        parts.append(f"  nozzle temp: {ext_temp}°C")
    if bed_temp:
        parts.append(f"  bed temp: {bed_temp}°C")
    if sku:
        parts.append(f"  SKU: {sku}")
    logger.info("\n".join(parts))


def log_new_spool(logger, prefix, spool, weight, spool_weight=None):
    """Log a detailed breakdown when a new spool is created in Spoolman."""
    sid = spool.get("id", "?")
    parts = [f"{prefix}: created spool #{sid} in Spoolman:"]
    parts.append(f"  filament weight: {weight}g")
    if spool_weight:
        parts.append(f"  spool weight (tare): {spool_weight}g")
    parts.append(f"  remaining: {weight}g")
    logger.info("\n".join(parts))


def get_auto_spoolman_create(lane, unit_default=False):
    """Check if auto Spoolman creation is enabled — unit then extruder fallback."""
    unit = getattr(lane, 'unit_obj', None)
    if unit is not None and getattr(unit, 'auto_spoolman_create', False):
        return True
    extruder = getattr(lane, 'extruder_obj', None)
    if extruder is not None and getattr(extruder, 'auto_spoolman_create', False):
        return True
    return unit_default


def apply_filament_defaults(lane: AFCLane, slot_info: dict,
                            color_converter=None, afc_defaults=None):
    """Apply RFID material/color/temps to a lane if not already set.

    :param lane: Lane to update.
    :param slot_info: Dict with material, color_hex or color, extruder_temp, bed_temp.
    :param color_converter: Optional callable to convert slot_info 'color' to hex string.
    :param afc_defaults: Optional dict with 'default_material_type', 'default_color' fallbacks.
    """
    has_material = getattr(lane, "material", None) not in (None, "")
    has_color = getattr(lane, "color", None) not in (None, "", "#000000")
    has_extruder_temp = getattr(lane, "extruder_temp", None) is not None
    has_bed_temp = getattr(lane, "bed_temp", None) is not None

    rfid_material = slot_info.get("material", "") if slot_info else ""
    rfid_extruder_temp = slot_info.get("extruder_temp") if slot_info else None
    rfid_bed_temp = slot_info.get("bed_temp") if slot_info else None

    if rfid_material and rfid_material.lower() == "unknown":
        rfid_material = ""

    color_hex = slot_info.get("color_hex", "") if slot_info else ""
    if not color_hex and color_converter is not None:
        raw_color = slot_info.get("color", [0, 0, 0]) if slot_info else [0, 0, 0]
        if raw_color != [0, 0, 0]:
            color_hex = color_converter(raw_color)

    if not has_material and rfid_material:
        lane.material = rfid_material
    if not has_color and color_hex:
        lane.color = color_hex if color_hex.startswith("#") else f"#{color_hex}"
    if not has_extruder_temp and rfid_extruder_temp is not None:
        try:
            lane.extruder_temp = float(rfid_extruder_temp)
        except (TypeError, ValueError):
            pass
    if not has_bed_temp and rfid_bed_temp is not None:
        try:
            lane.bed_temp = float(rfid_bed_temp)
        except (TypeError, ValueError):
            pass

    if afc_defaults is not None:
        if not has_material and not getattr(lane, "material", None):
            default_mat = afc_defaults.get("default_material_type")
            if default_mat:
                lane.material = default_mat
        if not has_color and not getattr(lane, "color", None):
            default_color = afc_defaults.get("default_color")
            if default_color:
                lane.color = default_color

    if not getattr(lane, "weight", 0):
        lane.weight = 1000


def sync_rfid_to_spoolman(afc, lane, slot_info: dict, logger, prefix: str,
                          allow_create: bool = False, set_next: bool = False,
                          spool_weight=None):
    """Sync RFID tag data to Spoolman: match existing or create new filament + spool.

    :param afc: AFC main object (needs .spoolman, .moonraker, .spool).
    :param lane: Lane to assign spool to.
    :param slot_info: Dict with sku, brand, material, color_hex, diameter, extruder_temp, bed_temp.
    :param logger: Logger for info/error messages.
    :param prefix: Log prefix (e.g. 'ACE RFID', 'U1 RFID').
    :param allow_create: If True, create new filaments/spools when no match found.
    :param set_next: If True, stage as next_spool_id instead of assigning to lane.
    :param spool_weight: Optional tare weight from RFID tag (ACE provides this).
    """
    if set_next:
        try:
            afc.spool.next_spool_info = dict(slot_info)
        except Exception:
            pass

    if afc.spoolman is None or afc.moonraker is None:
        return
    if not set_next and getattr(lane, "spool_id", None) not in (None, "", 0):
        return

    sku = slot_info.get("sku", "")
    brand = slot_info.get("brand", "")
    material = slot_info.get("material", "")
    color_hex = slot_info.get("color_hex", "") or None
    # All colours the tag carries (dual/multi-colour spools have >1). Used for
    # exact identity so a spool only matches a filament with the same colours.
    multi_color = slot_info.get("multi_color") or ([color_hex] if color_hex else [])
    scanned_colors = [c.lstrip("#").lower() for c in multi_color if c]
    is_multi = len(scanned_colors) > 1
    diameter = slot_info.get("diameter", 1.75)
    ext_temp = slot_info.get("extruder_temp")
    bed_temp = slot_info.get("bed_temp")
    default_filament_weight = 1000

    if not sku and not material:
        return

    moonraker = afc.moonraker

    try:
        filament = None

        if sku:
            filaments = moonraker.search_filaments(article_number=sku)
            for f in filaments:
                if f.get("article_number", "") == sku:
                    filament = f
                    break

        if filament is None and (brand or material):
            fallback_candidates = []
            if brand and material:
                fallback_candidates = moonraker.search_filaments(
                    vendor_name=brand, material=material)
            elif material:
                fallback_candidates = moonraker.search_filaments(
                    material=material)
            elif brand:
                fallback_candidates = moonraker.search_filaments(
                    vendor_name=brand)

            best = None
            best_score = -1
            best_color_dist = float('inf')
            for f in fallback_candidates:
                score = 0
                f_material = (f.get("material") or "").strip().lower()
                f_vendor = (f.get("vendor", {}).get("name") or "").strip().lower()
                f_color = (f.get("color_hex") or "").strip().lstrip("#").lower()
                f_diameter = f.get("diameter")

                if material and f_material == material.strip().lower():
                    score += 3
                if brand and f_vendor == brand.strip().lower():
                    score += 3

                # Candidate's colour set (multi-colour aware).
                f_multi = (f.get("multi_color_hexes") or "").strip()
                if f_multi:
                    f_colors = [c.strip().lstrip("#").lower()
                                for c in f_multi.split(",") if c.strip()]
                elif f_color:
                    f_colors = [f_color]
                else:
                    f_colors = []

                cdist = float('inf')
                if scanned_colors and f_colors:
                    if colors_match(scanned_colors, f_colors,
                                    COLOR_MATCH_TOLERANCE):
                        score += 5
                        cdist = 0.0
                    else:
                        # Different colour(s) = different filament. Never
                        # collapse a slightly-off or differently-coloured spool
                        # onto this one (also keeps dual-colour spools distinct).
                        continue
                elif scanned_colors and not f_colors:
                    score -= 3

                if diameter and f_diameter is not None:
                    try:
                        if abs(float(f_diameter) - float(diameter)) <= 0.05:
                            score += 1
                    except Exception:
                        pass

                is_better = (score > best_score or
                             (score == best_score and cdist < best_color_dist))
                if is_better:
                    best = f
                    best_score = score
                    best_color_dist = cdist

            if best is not None and best_score >= 4:
                if color_hex and best_color_dist >= 10 and allow_create:
                    pass
                else:
                    filament = best

        if filament is None:
            if not allow_create:
                return

            vendor_id = None
            if brand:
                vendor = moonraker.get_or_create_vendor(brand)
                if vendor:
                    vendor_id = vendor.get("id")

            filament_name = f"{material} {sku}".strip() if material else (sku or "Unknown")
            filament = moonraker.create_filament(
                name=filament_name,
                vendor_id=vendor_id,
                material=material or None,
                density=density_for_material(material),
                diameter=diameter,
                # Always store the primary colour. Dual/multi spools also store
                # the full set in multi_color_hexes (used for identity matching),
                # but keeping color_hex set means the AFC spool picker shows the
                # primary colour instead of black for multi-colour spools.
                color_hex=color_hex,
                multi_color_hexes=scanned_colors if is_multi else None,
                settings_extruder_temp=ext_temp,
                settings_bed_temp=bed_temp,
                weight=default_filament_weight,
                spool_weight=spool_weight if spool_weight and spool_weight > 0 else None,
                article_number=sku or None,
            )
            if filament is None:
                return
            log_new_filament(logger, prefix, filament,
                             brand, material, color_hex, diameter,
                             ext_temp, bed_temp, sku)

        filament_id = filament.get("id")
        if filament_id is None:
            return

        spool = None
        existing_spools = moonraker.search_spools(filament_id=filament_id)
        if existing_spools:
            best = None
            for s in existing_spools:
                remaining = s.get("remaining_weight") or 0
                if best is None or remaining > (best.get("remaining_weight") or 0):
                    best = s
            if best is not None:
                spool = best

        if spool is None:
            if not allow_create:
                return

            spool = moonraker.create_spool(
                filament_id=filament_id,
                initial_weight=default_filament_weight,
                remaining_weight=default_filament_weight,
                spool_weight=spool_weight if spool_weight and spool_weight > 0 else None,
            )
            if spool is None:
                return
            log_new_spool(logger, prefix, spool,
                          default_filament_weight,
                          spool_weight if spool_weight and spool_weight > 0 else None)

        spool_id = spool.get("id")
        fil_name = filament.get("name", "")
        fil_color = (filament.get("color_hex") or "").strip().lstrip("#")
        remaining = spool.get("remaining_weight")
        remaining_str = f", {remaining:.0f}g left" if remaining else ""
        if fil_color:
            cname = color_name(fil_color)
            color_str = f", {cname} #{fil_color}" if cname else f", #{fil_color}"
        else:
            color_str = ""
        desc = f"'{fil_name}'{color_str}{remaining_str}"

        if set_next:
            afc.spool.next_spool_id = spool_id
            logger.info(
                f"{prefix}: spool #{spool_id} ({desc}) staged as next_spool_id")
        else:
            afc.spool.set_spoolID(lane, spool_id)
            lane.send_lane_data()
            logger.info(
                f"{prefix}: spool #{spool_id} ({desc}) assigned to {lane.name}")

    except Exception as e:
        lane_name = getattr(lane, 'name', '?')
        logger.error(f"{prefix} Spoolman sync failed for {lane_name}: {e}")
