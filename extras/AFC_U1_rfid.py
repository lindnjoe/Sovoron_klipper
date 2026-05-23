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

from extras.AFC_RFID import (
    color_name, color_distance, density_for_material,
    log_new_filament, log_new_spool,
    get_auto_spoolman_create, apply_filament_defaults,
    sync_rfid_to_spoolman,
)

POLL_INTERVAL = 2.0


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
        self._last_scan_time: Dict[int, float] = {}

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
        if self._scanner_channels:
            self._guard_ptc_rfid_clears()
        self._poll_timer = self.reactor.register_timer(
            self._poll_cb, self.reactor.monotonic() + POLL_INTERVAL)

    def _guard_ptc_rfid_clears(self):
        """Patch print_task_config's RFID callback to block clear events
        on spool_scanner channels.

        The U1 fires is_clear=True when a tool docks/undocks and the RFID
        reader loses contact.  That wipes filament_type to NONE in
        print_task_config, breaking flow calibration.  For channels where
        spool_scanner is True, AFC owns the filament identity — let the
        original callback handle reads but suppress clears.
        """
        ptc = self.printer.lookup_object('print_task_config', None)
        if ptc is None or not hasattr(ptc, '_rfid_filament_info_update_cb'):
            return
        fd = self._filament_detect
        if not hasattr(fd, '_notify_data_update_cb'):
            return
        original_cb = ptc._rfid_filament_info_update_cb
        guarded = self._scanner_channels

        def guarded_cb(channel, info, is_clear=False):
            if is_clear and channel in guarded:
                return
            original_cb(channel, info, is_clear)

        cb_list = fd._notify_data_update_cb
        for i, cb in enumerate(cb_list):
            if cb is original_cb:
                cb_list[i] = guarded_cb
                break
        self.logger.info(
            f"U1 RFID: blocking RFID clears on scanner channels {guarded}")

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
                now = self.reactor.monotonic()
                last_t = getattr(self, '_last_scan_time', {}).get(channel, 0)
                if now - last_t < 12.0:
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
            cname = color_name(color)
            tag_desc += f" ({cname} #{color})" if cname else f" (#{color})"

        if is_scanner:
            self.logger.info(f"U1 RFID: spool scanned — {tag_desc}")
            self._last_scan_time[channel] = self.reactor.monotonic()
            allow_create = get_auto_spoolman_create(lane)
            sync_rfid_to_spoolman(
                self.afc, lane, slot_info, self.logger, "U1 RFID",
                allow_create=allow_create, set_next=True)
            self._notify_scan(brand, material, color, slot_info)
            return

        self.logger.info(f"U1 RFID: tag detected on {lane_name} — {tag_desc}")
        if getattr(lane, "spool_id", None) not in (None, "", 0):
            self.afc.spool.set_spoolID(lane, "")
        apply_filament_defaults(lane, slot_info)
        allow_create = get_auto_spoolman_create(lane)
        sync_rfid_to_spoolman(
            self.afc, lane, slot_info, self.logger, "U1 RFID",
            allow_create=allow_create)
        self._notify_scan(brand, material, color, slot_info, lane_name)
        lane.send_lane_data()
        self.afc.save_vars()

    def _clear_lane(self, lane, lane_name: str):
        """Clear RFID data from a lane when tag is removed."""
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

    def _notify_scan(self, brand: str, material: str, color: str,
                     slot_info: dict, lane_name: str = ""):
        """Send a user-visible notification when RFID reads a spool."""
        try:
            cname = color_name(color) if color else ""
            ext = slot_info.get("extruder_temp")
            bed = slot_info.get("bed_temp")
            raw = self.logger.raw
            is_scanner = not lane_name

            if is_scanner:
                title = "Spool Scanned"
                header = "Spool scanned and staged for next load:"
                spool_id = getattr(self.afc.spool, 'next_spool_id', None)
            else:
                title = f"Spool Loaded — {lane_name}"
                header = f"Spool loaded on {lane_name}:"
                lane = self._lane_objects.get(lane_name)
                spool_id = getattr(lane, "spool_id", None) if lane else None

            lines = [header]
            if brand:
                lines.append(f"  Brand: {brand}")
            if material:
                lines.append(f"  Material: {material}")
            if color:
                label = f"{cname} (#{color})" if cname else f"#{color}"
                lines.append(f"  Color: {label}")
            if ext:
                lines.append(f"  Nozzle temp: {ext}°C")
            if bed:
                lines.append(f"  Bed temp: {bed}°C")
            if spool_id:
                lines.append(f"  Spoolman ID: {spool_id}")
            self.afc.gcode.respond_info("\n".join(lines))

            prompt_lines = []
            if brand:
                prompt_lines.append(f"Brand: {brand}")
            if material:
                prompt_lines.append(f"Material: {material}")
            if color:
                label = f"{cname} (#{color})" if cname else f"#{color}"
                prompt_lines.append(f"Color: {label}")
            if ext:
                prompt_lines.append(f"Nozzle: {ext}°C")
            if bed:
                prompt_lines.append(f"Bed: {bed}°C")
            if spool_id:
                id_label = "Set as Spoolman next ID" if is_scanner else "Spoolman ID"
                prompt_lines.append(f"{id_label}: {spool_id}")
            raw(f"// action:prompt_begin {title}")
            for pl in prompt_lines:
                raw(f"// action:prompt_text {pl}")
            raw("// action:prompt_footer_button OK|RESPOND TYPE=command MSG=action:prompt_end|info")
            raw("// action:prompt_show")

            self.reactor.register_callback(
                lambda e: self.logger.raw("// action:prompt_end"),
                self.reactor.monotonic() + 10.0)
        except Exception:
            pass

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
