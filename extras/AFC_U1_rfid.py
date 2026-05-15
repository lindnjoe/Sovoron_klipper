# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# U1 RFID integration: reads spool data from Snapmaker U1's filament_detect
# Klipper module and syncs to AFC lanes / Spoolman.

from __future__ import annotations
from typing import TYPE_CHECKING, Optional, Dict

if TYPE_CHECKING:
    from extras.AFC import afc
    from extras.AFC_lane import AFCLane

POLL_INTERVAL = 2.0

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


def _color_name(hex_str: str) -> str:
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


def _log_new_filament(logger, prefix, filament, brand, material, color_hex,
                      diameter, ext_temp, bed_temp, sku=""):
    """Log a detailed breakdown when a new filament is created in Spoolman."""
    fid = filament.get("id", "?")
    parts = [f"{prefix}: created filament #{fid} in Spoolman:"]
    if brand:
        parts.append(f"  vendor: {brand}")
    if material:
        parts.append(f"  material: {material}")
    if color_hex:
        cname = _color_name(color_hex)
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


def _log_new_spool(logger, prefix, spool, weight, spool_weight=None):
    """Log a detailed breakdown when a new spool is created in Spoolman."""
    sid = spool.get("id", "?")
    parts = [f"{prefix}: created spool #{sid} in Spoolman:"]
    parts.append(f"  filament weight: {weight}g")
    if spool_weight:
        parts.append(f"  spool weight (tare): {spool_weight}g")
    parts.append(f"  remaining: {weight}g")
    logger.info("\n".join(parts))


# Material density (g/cm^3) used when creating a Spoolman filament from
# an RFID tag. Spoolman computes remaining length from weight using this
# value, so hardcoding PLA here makes length tracking wrong for every
# other material. Anything not listed falls back to PLA.
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


def _density_for_material(material: str) -> float:
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
    # Longest-prefix match so 'placfblack' -> 'placf', 'absgf25' -> 'absgf'.
    for k in sorted(MATERIAL_DENSITY, key=len, reverse=True):
        if key.startswith(k):
            return MATERIAL_DENSITY[k]
    return 1.24


class AFC_U1_RFID:
    """Polls the Snapmaker U1 filament_detect Klipper object for RFID tag data
    and applies it to AFC lanes (material, color) and Spoolman."""

    def __init__(self, afc_obj: afc):
        self.afc = afc_obj
        self.printer = afc_obj.printer
        self.reactor = afc_obj.reactor
        self.logger = afc_obj.logger
        self._filament_detect = None
        self._lane_channel_map: Dict[str, int] = {}
        self._lane_objects: Dict[str, AFCLane] = {}
        self._last_uid: Dict[int, Optional[list]] = {}
        self._poll_timer = None
        self._scanner_channels: set = set()
        self._channel_to_lane: Dict[int, str] = {}

    def register_lane(self, lane: AFCLane, channel: int):
        """Register a lane to monitor a specific filament_detect channel."""
        self._lane_channel_map[lane.name] = channel
        self._lane_objects[lane.name] = lane
        self._last_uid[channel] = None
        self._channel_to_lane[channel] = lane.name

    def start(self):
        """Start polling filament_detect for RFID data."""
        if not self._lane_channel_map:
            return
        self._filament_detect = self.printer.lookup_object("filament_detect", None)
        if self._filament_detect is None:
            self.logger.info("U1 RFID: filament_detect module not found")
            return
        channels = list(self._lane_channel_map.items())
        self._scanner_channels = {ch for name, ch in channels
                                   if getattr(self._lane_objects.get(name), 'spool_scanner', False)}
        self.logger.info(
            f"U1 RFID: monitoring {len(channels)} channel(s): "
            + ", ".join(f"{name}=ch{ch}" for name, ch in channels))
        if self._scanner_channels:
            scanner_names = [name for name, ch in channels
                             if ch in self._scanner_channels]
            self.logger.info(f"U1 RFID: spool scanner active on: {', '.join(scanner_names)}")
        self._gcode = self.afc.gcode
        if hasattr(self._filament_detect, 'register_cb_2_update_filament_info'):
            try:
                self._filament_detect.register_cb_2_update_filament_info(
                    self._on_filament_info_update)
            except Exception:
                pass
        self._poll_timer = self.reactor.register_timer(
            self._poll_cb, self.reactor.monotonic() + POLL_INTERVAL)

    def _on_filament_info_update(self, *args):
        """Callback fired by filament_detect with (channel, info_dict, official)."""
        if len(args) >= 2 and isinstance(args[0], int) and isinstance(args[1], dict):
            channel = args[0]
            info = args[1]
            lane_name = self._channel_to_lane.get(channel)
            if lane_name is not None:
                try:
                    self._check_channel(lane_name, channel, info=info)
                except Exception:
                    pass
            return
        for lane_name, channel in self._lane_channel_map.items():
            try:
                self._check_channel(lane_name, channel)
            except Exception:
                pass

    def stop(self):
        """Stop polling."""
        if self._poll_timer is not None:
            self.reactor.update_timer(self._poll_timer, self.reactor.NEVER)

    def _poll_cb(self, eventtime):
        """Periodic check for new RFID data on registered channels."""
        for ch in self._scanner_channels:
            try:
                self._gcode.run_script_from_command(
                    f"FILAMENT_DT_UPDATE CHANNEL={ch}")
            except Exception:
                pass
        for lane_name, channel in self._lane_channel_map.items():
            if channel in self._scanner_channels:
                continue
            try:
                self._check_channel(lane_name, channel)
            except Exception:
                pass
        return eventtime + POLL_INTERVAL

    _LOCKED_STATES = frozenset({
        "Loaded", "Tooled", "Tool Loaded", "Tool Loading", "Tool Unloading",
        "HUB Loading",
    })

    def _check_channel(self, lane_name: str, channel: int, info: dict = None):
        """Check a single channel for new or changed RFID data."""
        if self._filament_detect is None:
            return

        if info is None:
            info = self._get_channel_info(channel)
        lane = self._lane_objects.get(lane_name)
        is_scanner = lane is not None and getattr(lane, 'spool_scanner', False)

        if info is None:
            return

        card_uid = info.get("CARD_UID")
        if not card_uid or card_uid == 0:
            if self._last_uid.get(channel) not in (None, 0):
                if not is_scanner:
                    self._last_uid[channel] = 0
                    if lane is not None and getattr(lane, "status", "") not in self._LOCKED_STATES:
                        self._clear_lane(lane, lane_name)
            return

        if card_uid == self._last_uid.get(channel):
            if is_scanner:
                if getattr(self.afc.spool, 'next_spool_id', None):
                    return
            else:
                return

        if lane is None:
            return

        if not is_scanner and getattr(lane, "status", "") in self._LOCKED_STATES:
            return

        self._last_uid[channel] = card_uid

        main_type = info.get("MAIN_TYPE", "")
        if not main_type or main_type.upper() == "NONE":
            return

        slot_info = self._map_to_slot_info(info)
        brand = slot_info.get('brand', '')
        material = slot_info.get('material', '')
        color = slot_info.get('color_hex', '')
        tag_desc = f"{brand} {material}".strip() or "Unknown"
        if color:
            cname = _color_name(color)
            tag_desc += f" ({cname} #{color})" if cname else f" (#{color})"

        if is_scanner:
            self.logger.info(f"U1 RFID: spool scanned — {tag_desc}")
            self._sync_to_spoolman(lane, slot_info, set_next=True)
            return

        self.logger.info(f"U1 RFID: tag detected on {lane_name} — {tag_desc}")
        if getattr(lane, "spool_id", None) not in (None, "", 0):
            self.afc.spool.set_spoolID(lane, "")
        self._apply_filament_defaults(lane, slot_info)
        self._sync_to_spoolman(lane, slot_info)
        lane.send_lane_data()
        self.afc.save_vars()

    def _clear_lane(self, lane, lane_name: str):
        """Clear RFID data from a lane when tag is removed.

        Also clears spool_id so the next tag inserted into this lane
        triggers a fresh Spoolman match in _sync_to_spoolman (which
        early-returns if spool_id is already set)."""
        lane.material = ""
        lane.color = ""
        if getattr(lane, "spool_id", None) not in (None, "", 0):
            try:
                self.afc.spool.set_spoolID(lane, "")
            except Exception as e:
                self.logger.warning(
                    f"U1 RFID: failed to clear spool_id on {lane_name}: {e}")
        lane.send_lane_data()
        self.afc.save_vars()

    def _get_channel_info(self, channel: int) -> Optional[dict]:
        """Read filament info for a channel from filament_detect."""
        fd = self._filament_detect
        if hasattr(fd, 'get_a_filament_info'):
            try:
                info = fd.get_a_filament_info(channel)
                if isinstance(info, dict):
                    return info
            except Exception:
                pass
        if hasattr(fd, 'get_all_filament_info'):
            try:
                all_info = fd.get_all_filament_info()
                if isinstance(all_info, (list, tuple)) and channel < len(all_info):
                    entry = all_info[channel]
                    if isinstance(entry, dict):
                        return entry
                elif isinstance(all_info, dict):
                    entry = all_info.get(channel) or all_info.get(str(channel))
                    if isinstance(entry, dict):
                        return entry
            except Exception:
                pass
        if hasattr(fd, 'get_status'):
            try:
                status = fd.get_status()
                if isinstance(status, dict):
                    info_list = status.get('info')
                    if info_list and channel < len(info_list):
                        entry = info_list[channel]
                        if isinstance(entry, dict) and entry.get("CARD_UID"):
                            return entry
            except Exception:
                pass
        return None

    def _map_to_slot_info(self, info: dict) -> dict:
        """Map filament_detect fields to AFC RFID slot_info format."""
        rgb_int = info.get("RGB_1", 0)
        if rgb_int == 16777215 or rgb_int == 0:
            color_hex = ""
        else:
            r = (rgb_int >> 16) & 0xFF
            g = (rgb_int >> 8) & 0xFF
            b = rgb_int & 0xFF
            color_hex = f"{r:02x}{g:02x}{b:02x}"
        ext_max = info.get("HOTEND_MAX_TEMP")
        ext_min = info.get("HOTEND_MIN_TEMP")
        bed_max = info.get("BED_TEMP")
        sku_raw = info.get("SKU", "")
        sku = "" if (not sku_raw or sku_raw == 0) else str(sku_raw)
        vendor = info.get("VENDOR", "")
        if vendor.upper() == "NONE":
            vendor = ""
        if ext_max and ext_min:
            ext_temp = (int(ext_max) + int(ext_min)) // 2
        elif ext_max:
            ext_temp = int(ext_max)
        else:
            ext_temp = None
        return {
            "material": info.get("MAIN_TYPE", ""),
            "color_hex": color_hex,
            "sku": sku,
            "brand": vendor,
            "sub_type": info.get("SUB_TYPE", ""),
            "diameter": 1.75,
            "extruder_temp": ext_temp,
            "bed_temp": int(bed_max) if bed_max else None,
        }

    def _apply_filament_defaults(self, lane: AFCLane, slot_info: dict):
        """Apply RFID material/color/temps to lane."""
        rfid_material = slot_info.get("material", "")
        color_hex = slot_info.get("color_hex", "")

        if rfid_material:
            lane.material = rfid_material
        if color_hex:
            lane.color = f"#{color_hex}"

        ext_temp = slot_info.get("extruder_temp")
        if ext_temp:
            lane.extruder_temp = float(ext_temp)
        bed_temp = slot_info.get("bed_temp")
        if bed_temp:
            lane.bed_temp = float(bed_temp)

        if not getattr(lane, "weight", 0):
            lane.weight = 1000

    @staticmethod
    def _color_distance(hex1: str, hex2: str) -> float:
        """Euclidean RGB distance between two hex color strings."""
        r1, g1, b1 = int(hex1[0:2], 16), int(hex1[2:4], 16), int(hex1[4:6], 16)
        r2, g2, b2 = int(hex2[0:2], 16), int(hex2[2:4], 16), int(hex2[4:6], 16)
        return ((r1 - r2) ** 2 + (g1 - g2) ** 2 + (b1 - b2) ** 2) ** 0.5

    def _get_auto_spoolman_create(self, lane: AFCLane) -> bool:
        """Check if auto Spoolman creation is enabled — unit then extruder fallback."""
        unit = getattr(lane, 'unit_obj', None)
        if unit is not None and getattr(unit, 'auto_spoolman_create', False):
            return True
        extruder = getattr(lane, 'extruder_obj', None)
        if extruder is not None and getattr(extruder, 'auto_spoolman_create', False):
            return True
        return False

    def _sync_to_spoolman(self, lane: AFCLane, slot_info: dict,
                          set_next: bool = False):
        """Sync RFID data to Spoolman: match existing or create filament+spool.

        Always searches for existing filaments/spools to match.
        Only creates new ones when auto_spoolman_create is enabled on the unit.

        When set_next is True, stages the spool as next_spool_id instead of
        assigning it to the lane directly (spool scanner mode).
        Also stages next_spool_info with raw RFID data for use without Spoolman.
        """
        if set_next:
            try:
                self.afc.spool.next_spool_info = dict(slot_info)
            except Exception:
                pass
        if self.afc.spoolman is None or self.afc.moonraker is None:
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
        allow_create = self._get_auto_spoolman_create(lane)

        if not sku and not material:
            return

        moonraker = self.afc.moonraker

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

                    if material and f_material == material.strip().lower():
                        score += 3
                    if brand and f_vendor == brand.strip().lower():
                        score += 3

                    cdist = float('inf')
                    if color_hex and f_color and len(f_color) >= 6:
                        cdist = self._color_distance(
                            color_hex.strip().lower(), f_color[:6])
                        if cdist < 10:
                            score += 5
                        else:
                            score -= 6
                    elif color_hex and not f_color:
                        score -= 3

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
                    density=_density_for_material(material),
                    diameter=diameter,
                    color_hex=color_hex,
                    settings_extruder_temp=ext_temp,
                    settings_bed_temp=bed_temp,
                    weight=default_filament_weight,
                    spool_weight=None,
                    article_number=sku or None,
                )
                if filament is None:
                    return
                _log_new_filament(self.logger, "U1 RFID", filament,
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
                    spool_weight=None,
                )
                if spool is None:
                    return
                _log_new_spool(self.logger, "U1 RFID", spool,
                               default_filament_weight)

            spool_id = spool.get("id")
            fil_name = filament.get("name", "")
            fil_color = (filament.get("color_hex") or "").strip().lstrip("#")
            remaining = spool.get("remaining_weight")
            remaining_str = f", {remaining:.0f}g left" if remaining else ""
            if fil_color:
                cname = _color_name(fil_color)
                color_str = f", {cname} #{fil_color}" if cname else f", #{fil_color}"
            else:
                color_str = ""
            desc = f"'{fil_name}'{color_str}{remaining_str}"

            if set_next:
                self.afc.spool.next_spool_id = spool_id
                self.logger.info(
                    f"U1 RFID: spool #{spool_id} ({desc}) staged as next_spool_id")
            else:
                self.afc.spool.set_spoolID(lane, spool_id)
                lane.send_lane_data()
                self.logger.info(
                    f"U1 RFID: spool #{spool_id} ({desc}) assigned to {lane.name}")

        except Exception as e:
            self.logger.error(f"U1 RFID Spoolman sync failed for {lane.name}: {e}")

    def force_read(self, lane_name: str):
        """Force an RFID re-read for a specific lane (triggers FILAMENT_DT_UPDATE)."""
        channel = self._lane_channel_map.get(lane_name)
        if channel is None:
            return
        self._last_uid[channel] = None
        self.afc.gcode.run_script_from_command(
            f"FILAMENT_DT_UPDATE CHANNEL={channel}")
        self.reactor.pause(self.reactor.monotonic() + 0.5)
        self._check_channel(lane_name, channel)
