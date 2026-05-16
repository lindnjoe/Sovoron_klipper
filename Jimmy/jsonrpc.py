import json, logging, copy
from abc import ABC, abstractmethod
from typing import Callable, Dict
import threading

JSONRPC_ERR_SERVER_ERROR            = -32000
JSONRPC_ERR_INVALID_REQUEST         = -32600
JSONRPC_ERR_METHOD_NOT_FOUND        = -32601
JSONRPC_ERR_INVALID_PARAMS          = -32602
JSONRPC_ERR_PARSE_ERROR             = -32700

JSONRPC_ERR_TRANSPORT_ERROR         = -111
JSONRPC_ERR_TIMEOUT                 = -112
JSONRPC_ERR_NOT_CONNECTED           = -113

JSONRPC_PENDING_REQUEST_SIZE        = 100

JSONRPC_VERSION = "2.0"

JSONRPC_TRANSPORT_MQTT = "mqtt"


class GlobalIdGenerator:
    def __init__(self):
        self._current_id = 0
        self._lock = threading.Lock()
        self._max_id = 0x7FFFFFFF

    def generate_id(self):
        with self._lock:
            self._current_id += 1
            if self._current_id > self._max_id:
                self._current_id = 1
            return self._current_id

class TransportInterface(ABC):
    @abstractmethod
    def connect(self) -> bool:
        pass

    @abstractmethod
    def disconnect(self) -> bool:
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        pass

    @abstractmethod
    def send(self, data: str) -> bool:
        pass

    @abstractmethod
    def set_message_handler(self, handler: Callable) -> None:
        pass

class JSONRPCClient:
    global_id_generator = GlobalIdGenerator()

    def __init__(self, reactor, transport_type, **kwargs):
        self.reactor = reactor
        self._lock = threading.Lock()

        self.pending_requests = {}
        self.request_timeout = 30

        self.sync_request_id = None
        self.sync_request_result = None

        self.transport = None
        if transport_type == JSONRPC_TRANSPORT_MQTT:
            self.transport = MQTTTransport(**kwargs)
            self.transport.set_message_handler(self._handle_message)

    def connect(self):
        if self.transport is None:
            logging.error("[jsonrpc] transport not found!")
            return False
        return self.transport.connect()

    def disconnect(self):
        with self._lock:
            self.sync_request_id = None
            self.sync_request_result = None
            self.pending_requests.clear()
        if self.transport is not None:
            return self.transport.disconnect()
        else:
            return True

    def generate_id(self):
        return JSONRPCClient.global_id_generator.generate_id()

    def send_request(self, method, params={}, callback=None):
        if not method or not isinstance(method, str):
            raise ValueError("[jsonrpc] method must be a non-empty string")

        if callback is not None and not callable(callback):
            raise ValueError("[jsonrpc] callback must be a callable function")

        if self.transport is None:
            raise Exception("[jsonrpc] transport not found!")

        with self._lock:
            try:
                if len(self.pending_requests) >= JSONRPC_PENDING_REQUEST_SIZE:
                    logging.error("[jsonrpc] Too many pending requests")
                    self.pending_requests.pop(0)
            except Exception as e:
                logging.error(f"[jsonrpc] failed to pop pending requests {str(e)}")
                self.pending_requests.clear()

        request_id = self.generate_id()
        request = {
            "jsonrpc": JSONRPC_VERSION,
            "method": method,
            "params": params,
            "id": request_id
        }

        if self.transport.send(json.dumps(request)):
            with self._lock:
                self.pending_requests[request_id] = {
                    "request": request,
                    "callback": callback
                }

        return request_id

    def send_request_with_response(self, method, params={}, timeout=None):
        timeout = max(timeout or self.request_timeout, 0.1)
        request_id = self.generate_id()
        result_err = {
            "jsonrpc": JSONRPC_VERSION,
            "id": request_id,
            "error": {
                "code": JSONRPC_ERR_INVALID_REQUEST,
                "message": "Invalid request"
            }
        }

        if not method or not isinstance(method, str):
            logging.error("[jsonrpc] method must be a non-empty string")
            result_err["error"]["code"] = JSONRPC_ERR_INVALID_REQUEST
            result_err["error"]["message"] =  "Method must be a non-empty string"
            return result_err

        if self.transport is None or self.transport.is_connected() == False:
            logging.error("[jsonrpc] transport not connected")
            result_err["error"]["code"] = JSONRPC_ERR_NOT_CONNECTED
            result_err["error"]["message"] = "Transport not connected"
            return result_err

        request = {
            "jsonrpc": JSONRPC_VERSION,
            "method": method,
            "params": params,
            "id": request_id
        }

        # wait for previous request, but not too long
        current_time = self.reactor.monotonic()
        while self.sync_request_id is not None:
            if self.reactor.monotonic() >= self.request_timeout + current_time:
                result_err["error"]["code"] = JSONRPC_ERR_TIMEOUT
                result_err["error"]["message"] = "Request timed out"
                return result_err
            self.reactor.pause(self.reactor.monotonic() + 0.05)

        try:
            with self._lock:
                self.sync_request_id = request_id
                self.sync_request_result = None
            request_time = self.reactor.monotonic()

            if self.transport.send(json.dumps(request)):
                while self.reactor.monotonic() < request_time + timeout:
                    if self.sync_request_result and self.sync_request_result["id"] == self.sync_request_id:
                        break
                    self.reactor.pause(self.reactor.monotonic() + 0.05)

                if self.sync_request_result is None:
                    result_err["error"]["code"] = JSONRPC_ERR_TIMEOUT
                    result_err["error"]["message"] = "Request timed out"
                    return result_err

                result = copy.deepcopy(self.sync_request_result)
                return result
            else:
                result_err["error"]["code"] = JSONRPC_ERR_TRANSPORT_ERROR
                result_err["error"]["message"] = "Transport error"
                return result_err
        finally:
            with self._lock:
                self.sync_request_result = None
                self.sync_request_id = None

    def _handle_message(self, message: str):
        try:
            data = json.loads(message)

            if isinstance(data, list):
                for item in data:
                    self._handle_response(item)
            else:
                self._handle_response(data)

        except json.JSONDecodeError as e:
            logging.error(f"[jsonrpc] json parsing error: {str(e)}")
        except Exception as e:
            logging.error(f"[jsonrpc] message handling error: {str(e)}")

    def _handle_response(self, response: Dict):
        request_id = response.get("id")
        pending_request = None
        with self._lock:
            try:
                if request_id in self.pending_requests:
                    pending_request = self.pending_requests.pop(request_id)
            except Exception as e:
                logging.error(f"[jsonrpc] failed to pop pending requests {str(e)}")
                self.pending_requests.clear()
                pending_request = None

        if pending_request is not None:
            try:
                if pending_request["callback"]:
                    pending_request["callback"](response)
            except Exception as e:
                logging.error(f"[jsonrpc] callback error: {e}")

        else:
            with self._lock:
                if request_id == self.sync_request_id:
                    self.sync_request_result = response

class MQTTTransport(TransportInterface):
    def __init__(self, mqtt_client, request_topic, response_topic, qos):
        self.request_topic = request_topic
        self.response_topic = response_topic
        self.mqtt_client = mqtt_client
        self.qos = max(min(qos or 0, 2), 0)
        self.message_handler = None
        self.subscribe_handler = None
        self._is_connected = False

    def connect(self):
        try:
            if self.mqtt_client is None:
                logging.error("[jsonrpc][mqtt] mqtt client is None!")
                return False

            if self.response_topic is None:
                logging.error("[jsonrpc][mqtt] response topic is None!")
                return False

            if self._is_connected == True:
                logging.info("[jsonrpc][mqtt] already connected!")
                return True

            self.subscribe_handler = self.mqtt_client.subscribe_topic(
                                self.response_topic,
                                self.message_handler,
                                self.qos)
            self._is_connected = True

        except Exception as e:
            logging.error(f"[jsonrpc][mqtt] faiil to connect: {str(e)}")
            return False
        else:
            logging.info("[jsonrpc][mqtt] connected!")
            return True

    def disconnect(self):
        try:
            if self.mqtt_client is not None:
                if self._is_connected == True:
                    self.mqtt_client.unsubscribe(self.subscribe_handler)
                    self.subscribe_handler = None
                else:
                    pass

        except Exception as e:
            logging.error(f"[jsonrpc][mqtt] disconnect error: {str(e)}")
            return False

        else:
            return True

        finally:
            self._is_connected = False

    def is_connected(self):
        return self._is_connected

    def set_message_handler(self, handler):
        self.message_handler = handler

    def send(self, data):
        try:
            if self.mqtt_client is None:
                logging.error("[jsonrpc][mqtt] mqtt client is None!")
                return False

            if self.request_topic is None:
                logging.error("[jsonrpc][mqtt] request topic is None!")
                return False

            self.mqtt_client.publish_topic(self.request_topic, data)

        except Exception as e:
            logging.error(f"[jsonrpc][mqtt] send failed: {str(e)}")
            return False
        else:
            return True

