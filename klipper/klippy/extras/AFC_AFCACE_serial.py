# Armored Turtle Automated Filament Changer
#
# AFCACE Serial Communication Layer
# Direct serial communication with Anycubic ACE PRO hardware via JSON-RPC
# over USB serial. No dependency on ACEPRO or DuckACE.
#
# Protocol: Binary frames with CRC-16/MCRF4XX checksums wrapping JSON-RPC.
# Frame format: [0xFF 0xAA] [length_le16] [json_payload] [crc_le16] [0xFE]
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import json
import logging
import struct
import traceback

from typing import TYPE_CHECKING, Any, Dict, Optional

if TYPE_CHECKING:
    pass


_logger = logging.getLogger("AFC_AFCACE_serial")

# Frame constants
FRAME_HEADER = b'\xff\xaa'
FRAME_FOOTER = b'\xfe'
MAX_PAYLOAD_SIZE = 1024
DEFAULT_BAUD = 115200
REQUEST_TIMEOUT = 5.0
MAX_CONCURRENT_REQUESTS = 4


def crc16_mcrf4xx(data: bytes) -> int:
    """Calculate CRC-16/MCRF4XX checksum over data bytes."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF


class ACESerialError(Exception):
    """Raised when ACE serial communication fails."""
    pass


class ACETimeoutError(ACESerialError):
    """Raised when an ACE command times out."""
    pass


class ACEConnection:
    """Low-level serial connection to a single ACE PRO hardware unit.

    Integrates with Klipper's reactor for non-blocking I/O. Commands are
    sent as JSON-RPC over a binary-framed USB serial protocol. Responses
    are matched by request ID and delivered via reactor completions for
    synchronous use during tool change operations.
    """

    def __init__(self, reactor, serial_port, logger=None, baud_rate=DEFAULT_BAUD):
        self._reactor = reactor
        self._serial_port = serial_port
        self._baud_rate = baud_rate
        self._logger = logger or _logger

        self._serial = None
        self._fd_handle = None
        self._connected = False

        # Request tracking
        self._next_request_id = 0
        self._pending = {}  # request_id -> reactor.completion()

        # Read buffer for frame reassembly
        self._read_buffer = b''

        # Device info cache (populated on connect)
        self.device_info = {}
        self.slot_count = 4  # ACE PRO always has 4 slots

    @property
    def connected(self):
        return self._connected

    def connect(self):
        """Open the serial port and register with Klipper's reactor for reads."""
        if self._connected:
            return

        try:
            import serial as pyserial
        except ImportError:
            raise ACESerialError(
                "pyserial not installed. Install with: pip install pyserial"
            )

        try:
            self._serial = pyserial.Serial(
                port=self._serial_port,
                baudrate=self._baud_rate,
                timeout=0,           # Non-blocking reads
                write_timeout=0.1,
            )
            # Flush any stale data
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        except Exception as e:
            raise ACESerialError(
                f"Failed to open ACE serial port {self._serial_port}: {e}"
            )

        # Register fd with reactor for incoming data
        self._fd_handle = self._reactor.register_fd(
            self._serial.fileno(), self._handle_read
        )
        self._connected = True
        self._logger.info(
            f"ACE serial connected: {self._serial_port} @ {self._baud_rate}"
        )

        # Query device info
        try:
            self.device_info = self.send_command("get_info", timeout=3.0)
            self._logger.info(f"ACE device info: {self.device_info}")
        except Exception as e:
            self._logger.warning(f"ACE get_info failed (non-fatal): {e}")

    def disconnect(self):
        """Close serial port and clean up reactor registrations."""
        self._connected = False

        if self._fd_handle is not None:
            try:
                self._reactor.unregister_fd(self._fd_handle)
            except Exception:
                pass
            self._fd_handle = None

        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

        # Fail any pending requests
        for req_id, completion in list(self._pending.items()):
            try:
                completion.complete(None)
            except Exception:
                pass
        self._pending.clear()
        self._read_buffer = b''

        self._logger.info("ACE serial disconnected")

    def send_command(self, method, params=None, timeout=REQUEST_TIMEOUT):
        """Send a JSON-RPC command and wait for the response.

        Uses Klipper's reactor completion mechanism to block the calling
        context while still allowing other reactor events to be processed.

        Returns the 'result' field from the response dict, or raises on error.
        """
        if not self._connected or self._serial is None:
            raise ACESerialError("ACE not connected")

        request_id = self._next_request_id
        self._next_request_id += 1

        # Build JSON-RPC request
        request = {"id": request_id, "method": method}
        if params is not None:
            request["params"] = params

        payload = json.dumps(request, separators=(',', ':')).encode('utf-8')
        if len(payload) > MAX_PAYLOAD_SIZE:
            raise ACESerialError(
                f"ACE payload too large ({len(payload)} > {MAX_PAYLOAD_SIZE})"
            )

        # Create completion for blocking wait
        completion = self._reactor.completion()
        self._pending[request_id] = completion

        # Build and send frame
        frame = self._build_frame(payload)
        try:
            self._serial.write(frame)
            self._serial.flush()
        except Exception as e:
            self._pending.pop(request_id, None)
            raise ACESerialError(f"ACE write failed: {e}")

        self._logger.debug(f"ACE TX: {request}")

        # Wait for response (yields reactor)
        deadline = self._reactor.monotonic() + timeout
        result = completion.wait(deadline)

        self._pending.pop(request_id, None)

        if result is None:
            raise ACETimeoutError(
                f"ACE command '{method}' (id={request_id}) timed out after {timeout}s"
            )

        # Check for error response
        if isinstance(result, dict):
            code = result.get("code", 0)
            if code != 0:
                msg = result.get("msg", "unknown error")
                raise ACESerialError(
                    f"ACE command '{method}' failed: code={code}, msg={msg}"
                )
            return result.get("result", result)

        return result

    def send_command_async(self, method, params=None):
        """Send a JSON-RPC command without waiting for a response.

        Useful for fire-and-forget commands like stop operations.
        """
        if not self._connected or self._serial is None:
            return

        request_id = self._next_request_id
        self._next_request_id += 1

        request = {"id": request_id, "method": method}
        if params is not None:
            request["params"] = params

        payload = json.dumps(request, separators=(',', ':')).encode('utf-8')
        frame = self._build_frame(payload)

        try:
            self._serial.write(frame)
            self._serial.flush()
        except Exception as e:
            self._logger.debug(f"ACE async write failed: {e}")

        self._logger.debug(f"ACE TX (async): {request}")

    def _build_frame(self, payload: bytes) -> bytes:
        """Build a binary frame wrapping the JSON payload.

        Frame: [0xFF 0xAA] [length_le16] [payload] [crc_le16] [0xFE]
        """
        length = len(payload)
        crc = crc16_mcrf4xx(payload)
        return (
            FRAME_HEADER
            + struct.pack('<H', length)
            + payload
            + struct.pack('<H', crc)
            + FRAME_FOOTER
        )

    def _handle_read(self, eventtime):
        """Reactor callback invoked when data is available on the serial fd."""
        try:
            data = self._serial.read(4096)
        except Exception as e:
            self._logger.error(f"ACE read error: {e}")
            return

        if not data:
            return

        self._read_buffer += data
        self._parse_frames()

    def _parse_frames(self):
        """Extract and process complete frames from the read buffer."""
        while True:
            # Find frame header
            header_pos = self._read_buffer.find(FRAME_HEADER)
            if header_pos < 0:
                # No header found - discard buffer
                self._read_buffer = b''
                return
            if header_pos > 0:
                # Discard bytes before header
                self._read_buffer = self._read_buffer[header_pos:]

            # Need at least header(2) + length(2) = 4 bytes to read length
            if len(self._read_buffer) < 4:
                return

            payload_length = struct.unpack_from('<H', self._read_buffer, 2)[0]

            # Total frame size: header(2) + length(2) + payload + crc(2) + footer(1)
            frame_size = 2 + 2 + payload_length + 2 + 1
            if len(self._read_buffer) < frame_size:
                return  # Incomplete frame, wait for more data

            # Extract frame components
            payload = self._read_buffer[4:4 + payload_length]
            crc_received = struct.unpack_from(
                '<H', self._read_buffer, 4 + payload_length
            )[0]
            footer = self._read_buffer[4 + payload_length + 2]

            # Consume this frame from buffer
            self._read_buffer = self._read_buffer[frame_size:]

            # Validate footer
            if footer != FRAME_FOOTER[0]:
                self._logger.warning("ACE frame: invalid footer byte")
                continue

            # Validate CRC
            crc_calculated = crc16_mcrf4xx(payload)
            if crc_received != crc_calculated:
                self._logger.warning(
                    f"ACE frame: CRC mismatch (recv=0x{crc_received:04x}, "
                    f"calc=0x{crc_calculated:04x})"
                )
                continue

            # Parse JSON payload
            try:
                response = json.loads(payload.decode('utf-8'))
            except (json.JSONDecodeError, UnicodeDecodeError) as e:
                self._logger.warning(f"ACE frame: JSON parse error: {e}")
                continue

            self._logger.debug(f"ACE RX: {response}")
            self._handle_response(response)

    def _handle_response(self, response: dict):
        """Route a parsed response to its pending request completion."""
        response_id = response.get("id")
        if response_id is None:
            self._logger.debug(f"ACE unsolicited message: {response}")
            return

        completion = self._pending.get(response_id)
        if completion is not None:
            try:
                completion.complete(response)
            except Exception:
                pass
        else:
            self._logger.debug(
                f"ACE response for unknown request id={response_id}"
            )

    # ---- High-Level Hardware Commands ----

    def get_status(self, timeout=3.0):
        """Query current device status (temperatures, sensors, slots)."""
        return self.send_command("get_status", timeout=timeout)

    def get_filament_info(self, slot_index, timeout=3.0):
        """Get RFID/NFC filament data for a specific slot (0-based)."""
        return self.send_command(
            "get_filament_info",
            params={"index": slot_index},
            timeout=timeout,
        )

    def feed_filament(self, slot_index, length_mm, speed_mm_min):
        """Feed filament forward from the specified slot."""
        return self.send_command(
            "feed_filament",
            params={
                "index": slot_index,
                "length": length_mm,
                "speed": speed_mm_min,
            },
            timeout=max(REQUEST_TIMEOUT, (length_mm / speed_mm_min) * 60 + 10),
        )

    def stop_feed_filament(self, slot_index):
        """Stop an active feed operation on the specified slot."""
        self.send_command_async(
            "stop_feed_filament", params={"index": slot_index}
        )

    def unwind_filament(self, slot_index, length_mm, speed_mm_min, mode="normal"):
        """Retract/unwind filament back into the specified slot."""
        return self.send_command(
            "unwind_filament",
            params={
                "index": slot_index,
                "length": length_mm,
                "speed": speed_mm_min,
                "mode": mode,
            },
            timeout=max(REQUEST_TIMEOUT, (length_mm / speed_mm_min) * 60 + 10),
        )

    def stop_unwind_filament(self, slot_index):
        """Stop an active unwind/retract operation."""
        self.send_command_async(
            "stop_unwind_filament", params={"index": slot_index}
        )

    def start_feed_assist(self, slot_index):
        """Enable continuous motorized feed assist for a slot."""
        return self.send_command(
            "start_feed_assist", params={"index": slot_index}
        )

    def stop_feed_assist(self, slot_index):
        """Disable feed assist for a slot."""
        self.send_command_async(
            "stop_feed_assist", params={"index": slot_index}
        )

    def update_feeding_speed(self, slot_index, speed_mm_min):
        """Update the speed of an active feed operation."""
        self.send_command_async(
            "update_feeding_speed",
            params={"index": slot_index, "speed": speed_mm_min},
        )

    def update_unwinding_speed(self, slot_index, speed_mm_min):
        """Update the speed of an active unwind operation."""
        self.send_command_async(
            "update_unwinding_speed",
            params={"index": slot_index, "speed": speed_mm_min},
        )

    def start_drying(self, temp_c, fan_speed_rpm, duration_min):
        """Start a filament drying cycle."""
        return self.send_command(
            "drying",
            params={
                "temp": temp_c,
                "fan_speed": fan_speed_rpm,
                "duration": duration_min,
            },
        )

    def stop_drying(self):
        """Stop an active drying cycle."""
        return self.send_command("drying_stop")

    def enable_rfid(self):
        """Enable RFID/NFC tag detection."""
        return self.send_command("enable_rfid")

    def disable_rfid(self):
        """Disable RFID/NFC tag detection."""
        return self.send_command("disable_rfid")
