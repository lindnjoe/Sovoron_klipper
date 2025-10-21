#!/usr/bin/env python3
"""Robust QR scanner daemon for posting spool IDs to Moonraker."""

import argparse
import json
import logging
import os
import sys
import threading
import time
from typing import Dict, List, Optional
from urllib.error import URLError
from urllib.request import Request, urlopen

from evdev import InputDevice, categorize, ecodes
from evdev.events import KeyEvent

DEFAULT_DEVICE = "/dev/input/by-id/usb-MINJCODE_MINJCODE_MJ2818A_00000000011C-event-kbd"
DEFAULT_PREFIX = "web+spoolman:s-"
DEFAULT_HOST = "localhost"
DEFAULT_PORT = 7125

SHIFT_KEYS = {ecodes.KEY_LEFTSHIFT, ecodes.KEY_RIGHTSHIFT}
ENTER_KEYS = {ecodes.KEY_ENTER, ecodes.KEY_KPENTER}

KEYCODE_MAP: Dict[int, str] = {
    ecodes.KEY_1: "1",
    ecodes.KEY_2: "2",
    ecodes.KEY_3: "3",
    ecodes.KEY_4: "4",
    ecodes.KEY_5: "5",
    ecodes.KEY_6: "6",
    ecodes.KEY_7: "7",
    ecodes.KEY_8: "8",
    ecodes.KEY_9: "9",
    ecodes.KEY_0: "0",
    ecodes.KEY_MINUS: "-",
    ecodes.KEY_EQUAL: "=",
    ecodes.KEY_Q: "q",
    ecodes.KEY_W: "w",
    ecodes.KEY_E: "e",
    ecodes.KEY_R: "r",
    ecodes.KEY_T: "t",
    ecodes.KEY_Y: "y",
    ecodes.KEY_U: "u",
    ecodes.KEY_I: "i",
    ecodes.KEY_O: "o",
    ecodes.KEY_P: "p",
    ecodes.KEY_LEFTBRACE: "[",
    ecodes.KEY_RIGHTBRACE: "]",
    ecodes.KEY_A: "a",
    ecodes.KEY_S: "s",
    ecodes.KEY_D: "d",
    ecodes.KEY_F: "f",
    ecodes.KEY_G: "g",
    ecodes.KEY_H: "h",
    ecodes.KEY_J: "j",
    ecodes.KEY_K: "k",
    ecodes.KEY_L: "l",
    ecodes.KEY_SEMICOLON: ";",
    ecodes.KEY_APOSTROPHE: "'",
    ecodes.KEY_GRAVE: "`",
    ecodes.KEY_BACKSLASH: "\\",
    ecodes.KEY_Z: "z",
    ecodes.KEY_X: "x",
    ecodes.KEY_C: "c",
    ecodes.KEY_V: "v",
    ecodes.KEY_B: "b",
    ecodes.KEY_N: "n",
    ecodes.KEY_M: "m",
    ecodes.KEY_COMMA: ",",
    ecodes.KEY_DOT: ".",
    ecodes.KEY_SLASH: "/",
    ecodes.KEY_SPACE: " ",
    ecodes.KEY_KP0: "0",
    ecodes.KEY_KP1: "1",
    ecodes.KEY_KP2: "2",
    ecodes.KEY_KP3: "3",
    ecodes.KEY_KP4: "4",
    ecodes.KEY_KP5: "5",
    ecodes.KEY_KP6: "6",
    ecodes.KEY_KP7: "7",
    ecodes.KEY_KP8: "8",
    ecodes.KEY_KP9: "9",
    ecodes.KEY_KPMINUS: "-",
    ecodes.KEY_KPPLUS: "+",
    ecodes.KEY_KPDOT: ".",
    ecodes.KEY_KPSLASH: "/",
    ecodes.KEY_KPASTERISK: "*",
}

SHIFTED_MAP: Dict[str, str] = {
    "1": "!",
    "2": "@",
    "3": "#",
    "4": "$",
    "5": "%",
    "6": "^",
    "7": "&",
    "8": "*",
    "9": "(",
    "0": ")",
    "-": "_",
    "=": "+",
    "[": "{",
    "]": "}",
    "\\": "|",
    ";": ":",
    "'": '"',
    "`": "~",
    ",": "<",
    ".": ">",
    "/": "?",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="USB QR scanner daemon")
    parser.add_argument(
        "--device",
        default=os.environ.get("SCANNER_DEVICE", DEFAULT_DEVICE),
        help="Input event device to read (default: %(default)s)",
    )
    parser.add_argument(
        "--moonraker-host",
        default=os.environ.get("MOONRAKER_HOST", DEFAULT_HOST),
        help="Moonraker host (default: %(default)s)",
    )
    parser.add_argument(
        "--moonraker-port",
        type=int,
        default=int(os.environ.get("MOONRAKER_PORT", DEFAULT_PORT)),
        help="Moonraker port (default: %(default)s)",
    )
    parser.add_argument(
        "--spoolman-prefix",
        default=os.environ.get("SPOOLMAN_PREFIX", DEFAULT_PREFIX),
        help="Prefix to strip from spool QR codes (default: %(default)s)",
    )
    parser.add_argument(
        "--retry-initial",
        type=float,
        default=float(os.environ.get("RETRY_INITIAL", 2.0)),
        help="Initial reconnect delay in seconds (default: %(default)s)",
    )
    parser.add_argument(
        "--retry-max",
        type=float,
        default=float(os.environ.get("RETRY_MAX", 30.0)),
        help="Maximum reconnect delay in seconds (default: %(default)s)",
    )
    parser.add_argument(
        "--log-level",
        default=os.environ.get("LOG_LEVEL", "INFO"),
        help="Logging level (default: %(default)s)",
    )
    return parser.parse_args()


def configure_logging(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s [%(levelname)s] %(message)s",
    )


class MoonrakerError(Exception):
    """Raised when Moonraker returns an error payload."""


def post_next_spool_id(host: str, port: int, spool_id: str) -> None:
    url = f"http://{host}:{port}/printer/gcode/script"
    payload = json.dumps({"script": f"SET_NEXT_SPOOL_ID SPOOL_ID={spool_id}"}).encode()
    request = Request(
        url,
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urlopen(request, timeout=5) as response:
        body = response.read()
        if not body:
            return
        try:
            data = json.loads(body.decode())
        except json.JSONDecodeError:
            logging.debug("Moonraker returned non-JSON response: %r", body)
            return
        if isinstance(data, dict) and "error" in data:
            error = data["error"]
            if isinstance(error, dict):
                message = error.get("message") or error.get("detail") or str(error)
            else:
                message = str(error)
            raise MoonrakerError(message)


def extract_spool_id(raw: str, prefix: str) -> Optional[str]:
    data = raw.strip()
    if not data:
        return None
    if data.startswith(prefix):
        return data[len(prefix) :]
    if data.startswith("http"):
        parts = data.split("/")
        if len(parts) >= 6:
            return parts[5]
    return None


def process_scan(buffer: List[str], args: argparse.Namespace, poster: SpoolPoster) -> None:
    raw = "".join(buffer)
    spool_id = extract_spool_id(raw, args.spoolman_prefix)
    if not spool_id:
        logging.warning("Ignoring unrecognized scan: %r", raw)
        return
    normalized = normalize_spool_id(spool_id)
    if not normalized:
        logging.warning("Discarding invalid spool ID from scan: %r", spool_id)
        return
    logging.info("Queueing spool ID %s for submission", normalized)
    poster.submit(normalized)


def normalize_spool_id(spool_id: str) -> Optional[str]:
    trimmed = spool_id.strip()
    if not trimmed:
        return None
    if not trimmed.isdigit():
        return None
    return str(int(trimmed, 10))


class SpoolPoster:
    def __init__(self, host: str, port: int, initial_delay: float, max_delay: float) -> None:
        self.host = host
        self.port = port
        self.initial_delay = max(0.5, initial_delay)
        self.max_delay = max(self.initial_delay, max_delay)
        self._pending: Optional[str] = None
        self._condition = threading.Condition()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._worker, name="SpoolPoster", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        with self._condition:
            self._condition.notify_all()
        if self._thread.is_alive():
            self._thread.join(timeout=5)

    def submit(self, spool_id: str) -> None:
        with self._condition:
            if self._pending and self._pending != spool_id:
                logging.info(
                    "Replacing pending spool ID %s with new scan %s", self._pending, spool_id
                )
            self._pending = spool_id
            self._condition.notify_all()

    def _worker(self) -> None:
        delay = self.initial_delay
        while not self._stop_event.is_set():
            with self._condition:
                while self._pending is None and not self._stop_event.is_set():
                    self._condition.wait()
                if self._stop_event.is_set():
                    break
                spool_id = self._pending
            try:
                post_next_spool_id(self.host, self.port, spool_id)
            except MoonrakerError as exc:
                logging.error("Moonraker rejected spool ID %s: %s", spool_id, exc)
            except URLError as exc:
                extra = ""
                body_reader = getattr(exc, "read", None)
                if callable(body_reader):
                    body = body_reader()
                    if body:
                        extra = f" (body: {body.decode(errors='replace')})"
                logging.error("Failed to post spool ID %s: %s%s", spool_id, exc, extra)
            except Exception:  # noqa: BLE001
                logging.exception("Unexpected error posting spool ID %s", spool_id)
            else:
                logging.info("Posted spool ID %s to Moonraker", spool_id)
                delay = self.initial_delay
                with self._condition:
                    if self._pending == spool_id:
                        self._pending = None
                continue

            with self._condition:
                if self._pending != spool_id:
                    delay = self.initial_delay
                    continue
                self._condition.wait(timeout=delay)
            if self._stop_event.is_set():
                break
            delay = min(delay * 2, self.max_delay)


def translate_key(code: int, shift_active: bool) -> Optional[str]:
    base = KEYCODE_MAP.get(code)
    if base is None:
        return None
    if shift_active:
        if base.isalpha():
            return base.upper()
        return SHIFTED_MAP.get(base, base)
    return base


def read_device(device: InputDevice, args: argparse.Namespace, poster: SpoolPoster) -> None:
    logging.info("Reading from %s (%s)", device.path, device.name)
    shift_active = False
    buffer: List[str] = []

    for event in device.read_loop():
        if event.type != ecodes.EV_KEY:
            continue
        key_event = categorize(event)
        code = key_event.scancode

        if key_event.keystate == KeyEvent.key_down:
            if code in SHIFT_KEYS:
                shift_active = True
                continue
            if code in ENTER_KEYS:
                if buffer:
                    logging.info("Scanned code: %s", "".join(buffer))
                    process_scan(buffer, args, poster)
                    buffer.clear()
                else:
                    logging.debug("Enter pressed with empty buffer")
                continue
            character = translate_key(code, shift_active)
            if character is not None:
                buffer.append(character)
        elif key_event.keystate == KeyEvent.key_up and code in SHIFT_KEYS:
            shift_active = False


def acquire_device(device_path: str) -> InputDevice:
    return InputDevice(device_path)


def run(args: argparse.Namespace) -> None:
    poster = SpoolPoster(args.moonraker_host, args.moonraker_port, args.retry_initial, args.retry_max)
    poster.start()
    delay = max(0.5, args.retry_initial)
    try:
        while True:
            try:
                device = acquire_device(args.device)
            except FileNotFoundError:
                logging.warning(
                    "Device %s not found; retrying in %.1f seconds", args.device, delay
                )
                time.sleep(delay)
                delay = min(delay * 2, args.retry_max)
                continue
            except PermissionError:
                logging.error(
                    "Permission denied opening %s. Ensure correct udev rules.", args.device
                )
                time.sleep(delay)
                delay = min(delay * 2, args.retry_max)
                continue

            delay = max(0.5, args.retry_initial)
            try:
                read_device(device, args, poster)
            except OSError as exc:
                logging.warning("Device %s disconnected (%s)", args.device, exc)
            except Exception:  # noqa: BLE001
                logging.exception("Unhandled exception while reading device")
            finally:
                try:
                    device.close()
                except Exception:  # noqa: BLE001
                    pass
                time.sleep(delay)
                delay = min(delay * 2, args.retry_max)
    finally:
        poster.stop()


def main() -> None:
    args = parse_args()
    configure_logging(args.log_level)
    logging.info("Starting USB QR scanner daemon")
    run(args)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logging.info("Shutting down due to keyboard interrupt")
        sys.exit(0)
