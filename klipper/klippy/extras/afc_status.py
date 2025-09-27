"""A lightweight AFC status proxy for OpenAMS deployments."""

import logging
from typing import Any, Dict, Optional


class OpenAMSAFCStatus:
    """Expose AFC-compatible status endpoints backed by the OAMS manager."""

    def __init__(self, config) -> None:
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.webhooks = None
        try:
            self.webhooks = self.printer.lookup_object("webhooks")
        except Exception as err:
            logging.warning(
                "AFC status proxy: WebHooks unavailable (%s); endpoints disabled.",
                err,
            )

        self._manager: Optional[Any] = None
        self._stream_registered = False

        if self.webhooks is not None:
            self.webhooks.register_endpoint(
                "afc/status", self._handle_status_request
            )
            self.webhooks.register_status("afc", self._webhooks_status)

        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self) -> None:
        """Register the AFC status stream once the printer is fully ready."""

        if self._stream_registered:
            return

        manager = self._get_manager()
        if manager is None:
            logging.info("AFC status proxy: OAMS manager not available at ready.")
            return

        register_stream = getattr(manager, "register_status_stream", None)
        if callable(register_stream):
            register_stream("afc/stream_status", "type", "summary")
            self._stream_registered = True
        else:
            logging.warning(
                "AFC status proxy: OAMS manager does not expose register_status_stream."
            )

    def _get_manager(self) -> Optional[Any]:
        """Locate and cache the OAMS manager object."""

        if self._manager is not None:
            return self._manager

        manager = self.printer.lookup_object("oams_manager", None)
        if manager is not None:
            self._manager = manager
        return manager

    def _collect_payload(self, eventtime: float) -> Dict[str, Any]:
        """Assemble the AFC-style status payload."""

        manager = self._get_manager()
        if manager is None:
            return {
                "lanes": {},
                "summary": {
                    "timestamp": eventtime,
                    "followers": {},
                    "spools": {},
                    "pressure": {},
                    "faults": {},
                    "groups": {},
                    "lanes": {},
                    "lanes_by_group": {},
                },
                "system": {
                    "ready": False,
                    "timestamp": eventtime,
                    "current_group": None,
                    "num_groups": 0,
                    "num_fps": 0,
                    "num_oams": 0,
                    "delegated_runouts": 0,
                },
            }

        if hasattr(manager, "get_afc_status_payload"):
            return manager.get_afc_status_payload(eventtime)

        snapshot = manager.get_status(eventtime)
        summary = snapshot.get("summary", {})
        lanes: Dict[str, Dict[str, Any]] = {}
        for group_name, group_status in snapshot.get("filament_groups", {}).items():
            fps_name = manager.group_fps_name(group_name)
            fps_status = snapshot.get("fps", {}).get(fps_name, {})
            lanes[group_name] = {
                "group_name": group_name,
                "fps": fps_name,
                "lane": manager._canonical_lane_by_group.get(group_name),
                "loaded_spool": group_status.get("loaded_spool"),
                "available_spools": group_status.get("available_spools"),
                "has_available": group_status.get("has_available"),
                "is_loaded": group_status.get("is_loaded"),
                "current_spool_idx": fps_status.get("current_spool_idx"),
                "following": fps_status.get("following"),
                "direction": fps_status.get("direction"),
                "stuck_spool_active": fps_status.get("stuck_spool_active"),
                "clog_active": fps_status.get("clog_active"),
            }

        system = {
            "ready": manager.ready,
            "timestamp": snapshot.get("timestamp", eventtime),
            "current_group": manager.current_group,
            "num_groups": len(snapshot.get("filament_groups", {})),
            "num_fps": len(snapshot.get("fps", {})),
            "num_oams": len(manager.oams),
            "delegated_runouts": sum(
                1
                for status in snapshot.get("fps", {}).values()
                if status.get("afc_delegation_active")
            ),
        }

        return {
            "lanes": lanes,
            "summary": summary,
            "system": system,
        }

    def _handle_status_request(self, web_request) -> None:
        """Serve the /printer/afc/status REST endpoint."""

        eventtime = self.reactor.monotonic()
        payload = self._collect_payload(eventtime)
        web_request.send({"status": {"AFC": payload}})

    def _webhooks_status(self, eventtime: float) -> Dict[str, Any]:
        """Expose AFC summary data via the WebHooks status subscription."""

        payload = self._collect_payload(eventtime)
        return payload.get("summary", {})


def load_config(config):
    return OpenAMSAFCStatus(config)


def load_config_prefix(config):
    return load_config(config)

