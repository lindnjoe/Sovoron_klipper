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

_COLOR_NAMES = {
    "ff0000": "Red",       "cc0000": "Red",       "990000": "Dark Red",
    "660000": "Dark Red",  "4d0000": "Dark Red",  "330000": "Maroon",
    "c03020": "Red",       "b02018": "Red",       "cc3333": "Red",
    "e02020": "Red",       "a01818": "Dark Red",  "801010": "Dark Red",
    "00ff00": "Green",     "00cc00": "Green",     "006600": "Dark Green",
    "003300": "Dark Green","1a3300": "Dark Green",
    "0000ff": "Blue",      "0000cc": "Blue",      "000099": "Dark Blue",
    "000066": "Dark Blue", "003366": "Dark Blue",
    "ffffff": "White",     "f0f0f0": "White",     "e0e0e0": "Light Gray",
    "000000": "Black",     "0a0a0a": "Black",     "141414": "Black",
    "ffff00": "Yellow",    "cccc00": "Yellow",    "ffd700": "Gold",
    "998200": "Dark Gold", "b8860b": "Dark Gold",
    "ff8000": "Orange",    "ff6600": "Orange",    "ff4500": "Orange",
    "cc5500": "Dark Orange","993d00": "Dark Orange",
    "800080": "Purple",    "660066": "Purple",    "9900cc": "Purple",
    "330033": "Dark Purple","4a0066": "Dark Purple",
    "ff00ff": "Magenta",   "cc00cc": "Magenta",
    "ffc0cb": "Pink",      "ff69b4": "Pink",      "ff1493": "Hot Pink",
    "808080": "Gray",      "999999": "Gray",      "666666": "Dark Gray",
    "4d4d4d": "Dark Gray", "333333": "Dark Gray",
    "a0a0a0": "Gray",      "c0c0c0": "Silver",    "b0b0b0": "Silver",
    "8b4513": "Brown",     "654321": "Brown",     "a0522d": "Brown",
    "3b1e08": "Dark Brown","4a2812": "Dark Brown", "3f231c": "Dark Brown",
    "2c1a0e": "Dark Brown","5c3a1e": "Dark Brown", "6b3a2a": "Brown",
    "d2691e": "Chocolate", "7b3f00": "Chocolate",
    "00ffff": "Cyan",      "008b8b": "Teal",      "008080": "Teal",
    "004d4d": "Dark Teal", "003333": "Dark Teal",
    "000080": "Navy",      "191970": "Navy",
    "f5f5dc": "Beige",     "d2b48c": "Tan",       "ffe4c4": "Bisque",
    "ff7f50": "Coral",     "fa8072": "Salmon",
    "7fff00": "Chartreuse","32cd32": "Lime",      "00ff7f": "Mint",
    "4b0082": "Indigo",    "ee82ee": "Violet",    "dda0dd": "Plum",
    "40e0d0": "Turquoise", "87ceeb": "Sky Blue",  "4169e1": "Royal Blue",
    "556b2f": "Olive",     "808000": "Olive",     "333300": "Dark Olive",
    "f0e68c": "Khaki",     "bdb76b": "Dark Khaki",
    "fffdd0": "Cream",     "fffacd": "Lemon",
    "e6e6fa": "Lavender",  "d8bfd8": "Thistle",
    "fff0f5": "Blush",     "ffe4e1": "Misty Rose",
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


def color_distance(hex1: str, hex2: str) -> float:
    """Euclidean RGB distance between two hex color strings."""
    r1, g1, b1 = int(hex1[0:2], 16), int(hex1[2:4], 16), int(hex1[4:6], 16)
    r2, g2, b2 = int(hex2[0:2], 16), int(hex2[2:4], 16), int(hex2[4:6], 16)
    return ((r1 - r2) ** 2 + (g1 - g2) ** 2 + (b1 - b2) ** 2) ** 0.5


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

                cdist = float('inf')
                if color_hex and f_color and len(f_color) >= 6:
                    cdist = color_distance(
                        color_hex.strip().lower(), f_color[:6])
                    if cdist < 10:
                        score += 5
                    else:
                        score -= 6
                elif color_hex and not f_color:
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
                color_hex=color_hex,
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
