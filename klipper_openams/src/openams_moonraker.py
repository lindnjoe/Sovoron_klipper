# OpenAMS Moonraker helpers
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from __future__ import annotations

import json
from typing import Any, Dict, Optional, Literal
from urllib.parse import urljoin
from urllib.request import Request, urlopen


class OpenAMSMoonrakerClient:
    """Minimal Moonraker client for OpenAMS-owned status publishing.

    This client is intentionally scoped to OpenAMS integration so we avoid
    modifying AFC core Moonraker behavior.
    """

    def __init__(self, host: str, port: int, logger) -> None:
        base_host = host.rstrip("/")
        self.base_url = f"{base_host}:{int(port)}"
        self.database_url = urljoin(self.base_url, "server/database/item")
        self.logger = logger
        self._last_status_fingerprint: Optional[str] = None

    def _request(self, req: Request, *, timeout: float = 1.5) -> Optional[Dict[str, Any]]:
        try:
            with urlopen(req, timeout=timeout) as response:
                if not (200 <= response.status <= 299):
                    self.logger.debug(
                        f"OpenAMS Moonraker request failed: status={response.status} reason={response.reason}"
                    )
                    return None
                payload = json.load(response)
                return payload.get("result") if isinstance(payload, dict) else None
        except Exception as exc:
            self.logger.debug(f"OpenAMS Moonraker request error: {exc}")
            return None

    def is_available(self) -> bool:
        req = Request(urljoin(self.base_url, "server/info"), method="GET")
        return self._request(req, timeout=1.0) is not None

    def _status_fingerprint(self, status: Dict[str, Any]) -> str:
        return json.dumps(status, sort_keys=True, separators=(",", ":"))

    def publish_manager_status(
        self,
        status: Dict[str, Any],
        *,
        eventtime: float,
    ) -> Literal["published", "skipped", "failed"]:
        fingerprint = self._status_fingerprint(status)
        if fingerprint == self._last_status_fingerprint:
            return "skipped"

        payload = {
            "request_method": "POST",
            "namespace": "openams",
            "key": "manager_status",
            "value": {
                "eventtime": eventtime,
                "status": status,
            },
        }
        req = Request(
            self.database_url,
            data=json.dumps(payload).encode(),
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        if self._request(req) is None:
            return "failed"

        self._last_status_fingerprint = fingerprint
        return "published"
