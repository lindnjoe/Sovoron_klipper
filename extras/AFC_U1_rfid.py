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
_MAX_CONSECUTIVE_FAILURES = 5
_BACKOFF_INTERVAL = 10.0
_FORCE_READ_TIMEOUT = 1.0
_FORCE_READ_POLL_STEP = 0.05


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
        self._consecutive_failures: Dict[int, int] = {}
        self._backed_off: bool = False

    def register_lane(self, lane: AFCLane, channel: int):
        """Register a lane to monitor a specific filament_detect channel."""
        self._lane_channel_map[lane.name] = channel
        self._lane_objects[lane.name] = lane
        self._last_uid[channel] = None
        self._channel_to_lane[channel] = lane.name
        self._consecutive_failures[channel] = 0

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
            except Exception as e:
                self.logger.warning(f"U1 RFID: failed to register info callback: {e}")
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
                except Exception as e:
                    self.logger.warning(
                        f"U1 RFID: _on_filament_info_update error ch{channel}: {e}")
            return
        for lane_name, channel in self._lane_channel_map.items():
            try:
                self._check_channel(lane_name, channel)
            except Exception as e:
                self.logger.warning(
                    f"U1 RFID: _on_filament_info_update error {lane_name}: {e}")

    def stop(self):
        """Stop polling."""
        if self._poll_timer is not None:
            self.reactor.update_timer(self._poll_timer, self.reactor.NEVER)

    def _trigger_channel_update(self, channel: int) -> bool:
        """Trigger a fresh read from hardware for a scanner channel.
        Returns True on success, False on failure."""
        fd = self._filament_detect
        if hasattr(fd, 'update_filament_info'):
            try:
                fd.update_filament_info(channel)
                return True
            except Exception:
                pass
        if hasattr(fd, 'request_update'):
            try:
                fd.request_update(channel)
                return True
            except Exception:
                pass
        try:
            self._gcode.run_script_from_command(
                f"FILAMENT_DT_UPDATE CHANNEL={channel}")
            return True
        except Exception as e:
            self.logger.warning(
                f"U1 RFID: FILAMENT_DT_UPDATE failed ch{channel}: {e}")
            return False

    def _poll_cb(self, eventtime):
        """Periodic check for new RFID data on registered channels."""
        for ch in self._scanner_channels:
            if not self._trigger_channel_update(ch):
                self._consecutive_failures[ch] = \
                    self._consecutive_failures.get(ch, 0) + 1
                if self._consecutive_failures[ch] == _MAX_CONSECUTIVE_FAILURES:
                    self.logger.error(
                        f"U1 RFID: ch{ch} failed {_MAX_CONSECUTIVE_FAILURES} "
                        f"times consecutively, backing off")
                    self._backed_off = True
            else:
                self._consecutive_failures[ch] = 0

        for lane_name, channel in self._lane_channel_map.items():
            if channel in self._scanner_channels:
                continue
            try:
                self._check_channel(lane_name, channel)
            except Exception as e:
                self.logger.warning(
                    f"U1 RFID: poll error on {lane_name} ch{channel}: {e}")

        if self._backed_off:
            all_recovered = all(
                self._consecutive_failures.get(ch, 0) < _MAX_CONSECUTIVE_FAILURES
                for ch in self._scanner_channels)
            if all_recovered:
                self._backed_off = False
            return eventtime + _BACKOFF_INTERVAL
        return eventtime + POLL_INTERVAL

    _LOCKED_STATES = frozenset({
        "Loaded", "Tooled", "Tool Loaded", "Tool Loading", "Tool Unloading",
        "HUB Loading",
    })

    def _check_channel(self, lane_name: str, channel: int, info: dict = None):
        """Check a single channel for new or changed RFID data.

        For spool_scanner lanes the spool is assigned directly to the lane
        (bypassing the ``next_spool_id`` staging mechanism) so that Spoolman
        flow K sync triggers immediately on scan.

        :param lane_name: AFC lane name mapped to this RFID channel.
        :param channel: U1 filament_detect channel index.
        :param info: Pre-fetched RFID info dict, or *None* to read live.
        """
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
                if getattr(lane, 'spool_id', None) not in (None, "", 0):
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
            apply_filament_defaults(lane, slot_info)
            allow_create = get_auto_spoolman_create(lane)
            sync_rfid_to_spoolman(
                self.afc, lane, slot_info, self.logger, "U1 RFID",
                allow_create=allow_create)
            lane.send_lane_data()
            self.afc.save_vars()
            self._notify_scan(brand, material, color, slot_info,
                              lane_name=lane_name, is_scanner=True)
            return

        self.logger.info(f"U1 RFID: tag detected on {lane_name} — {tag_desc}")
        if getattr(lane, "spool_id", None) not in (None, "", 0):
            self.afc.spool.set_spoolID(lane, "")
        apply_filament_defaults(lane, slot_info)
        allow_create = get_auto_spoolman_create(lane)
        sync_rfid_to_spoolman(
            self.afc, lane, slot_info, self.logger, "U1 RFID",
            allow_create=allow_create)
        self._notify_scan(brand, material, color, slot_info,
                          lane_name=lane_name)
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
                     slot_info: dict, lane_name: str = "",
                     is_scanner: bool = False):
        """Send a user-visible notification when RFID reads a spool.

        Sends to three targets: Klipper console (respond_info), Mainsail/
        Octoprint (action:prompt), and U1 factory display (exception_manager).

        :param brand: Filament brand name.
        :param material: Filament material type (e.g. "PLA").
        :param color: Hex color string (without '#').
        :param slot_info: Full slot info dict from RFID tag.
        :param lane_name: Lane name; empty string if unknown.
        :param is_scanner: True when this is a spool_scanner read.
        """
        try:
            cname = color_name(color) if color else ""
            ext = slot_info.get("extruder_temp")
            bed = slot_info.get("bed_temp")
            raw = self.logger.raw
            lane = self._lane_objects.get(lane_name) if lane_name else None
            spool_id = getattr(lane, "spool_id", None) if lane else None

            if is_scanner:
                title = "Spool Scanned"
                header = "Spool scanned on %s:" % lane_name if lane_name else "Spool scanned:"
            else:
                title = "Spool Loaded — %s" % lane_name if lane_name else "Spool Loaded"
                header = "Spool loaded on %s:" % lane_name if lane_name else "Spool loaded:"

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
                prompt_lines.append(f"Spoolman ID: {spool_id}")
            raw(f"// action:prompt_begin {title}")
            for pl in prompt_lines:
                raw(f"// action:prompt_text {pl}")
            raw("// action:prompt_footer_button OK|RESPOND TYPE=command MSG=action:prompt_end|info")
            raw("// action:prompt_show")

            self.reactor.register_callback(
                lambda e: self.logger.raw("// action:prompt_end"),
                self.reactor.monotonic() + 10.0)

            em = self.printer.lookup_object("exception_manager", None)
            if em is not None:
                parts = []
                if brand:
                    parts.append(brand)
                if material:
                    parts.append(material)
                if cname:
                    parts.append(cname)
                msg = "%s: %s" % (title, " ".join(parts)) if parts else title
                channel = self._lane_channel_map.get(lane_name, 0)
                em.raise_exception_async(
                    id=529,
                    index=channel,
                    code=99,
                    message=msg,
                    oneshot=1,
                    level=1)
        except Exception as e:
            self.logger.warning(f"U1 RFID: notification error: {e}")

    def force_read(self, lane_name: str):
        """Force an RFID re-read for a specific lane."""
        channel = self._lane_channel_map.get(lane_name)
        if channel is None:
            return
        self._last_uid[channel] = None
        if not self._trigger_channel_update(channel):
            self.logger.warning(
                f"U1 RFID: force_read failed to trigger update for {lane_name}")
            return
        deadline = self.reactor.monotonic() + _FORCE_READ_TIMEOUT
        while self.reactor.monotonic() < deadline:
            info = self._get_channel_info(channel)
            if info is not None and info.get("CARD_UID"):
                self._check_channel(lane_name, channel, info=info)
                return
            self.reactor.pause(
                self.reactor.monotonic() + _FORCE_READ_POLL_STEP)
        self._check_channel(lane_name, channel)
