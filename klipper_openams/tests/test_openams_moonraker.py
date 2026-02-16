from pathlib import Path
from unittest.mock import Mock, patch
import importlib.util

MODULE_PATH = Path(__file__).resolve().parents[1] / "src" / "openams_moonraker.py"
_SPEC = importlib.util.spec_from_file_location("openams_moonraker", MODULE_PATH)
_MODULE = importlib.util.module_from_spec(_SPEC)
assert _SPEC is not None and _SPEC.loader is not None
_SPEC.loader.exec_module(_MODULE)
OpenAMSMoonrakerClient = _MODULE.OpenAMSMoonrakerClient


class _FakeResponse:
    def __init__(self, payload, status=200, reason="OK"):
        self._payload = payload
        self.status = status
        self.reason = reason

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return False

    def read(self, *args, **kwargs):
        return self._payload


def test_publish_manager_status_skips_identical_payloads():
    logger = Mock()
    client = OpenAMSMoonrakerClient("http://localhost", 7125, logger)
    status = {"oams": {"oams1": {"action_status": None}}}

    with patch.object(_MODULE, "urlopen") as mock_urlopen:
        mock_urlopen.return_value = _FakeResponse(b'{"result": {"ok": true}}')

        first = client.publish_manager_status(status, eventtime=1.0)
        second = client.publish_manager_status(status, eventtime=2.0)

    assert first == "published"
    assert second == "skipped"
    assert mock_urlopen.call_count == 1


def test_publish_manager_status_republishes_when_status_changes():
    logger = Mock()
    client = OpenAMSMoonrakerClient("http://localhost", 7125, logger)

    with patch.object(_MODULE, "urlopen") as mock_urlopen:
        mock_urlopen.return_value = _FakeResponse(b'{"result": {"ok": true}}')

        first = client.publish_manager_status({"state": "a"}, eventtime=1.0)
        second = client.publish_manager_status({"state": "b"}, eventtime=2.0)

    assert first == "published"
    assert second == "published"
    assert mock_urlopen.call_count == 2


def test_publish_manager_status_returns_failed_on_request_error():
    logger = Mock()
    client = OpenAMSMoonrakerClient("http://localhost", 7125, logger)

    with patch.object(_MODULE, "urlopen", side_effect=OSError("boom")):
        result = client.publish_manager_status({"state": "a"}, eventtime=1.0)

    assert result == "failed"
