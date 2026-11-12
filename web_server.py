import base64
import hashlib
import json
import os
import queue
import socket
import threading
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler

from websocket_protocol import WebSocketProtocol

# ---- Configuration constants ----
OUTBOUND_QUEUE_SIZE = (
    256  # Max messages buffered for WebSocket broadcast before dropping oldest
)
LOG_RATE_LIMIT_HZ = (
    10.0  # Max log messages broadcast per second to avoid flooding clients
)


# ------------------- lightweight WebSocket Server -------------------
class WebSocketHandler(WebSocketProtocol):
    def __init__(self, conn, addr, server):
        self.conn = conn
        self.addr = addr
        self.server = server
        self.handshake_done = False
        self.keep_alive = True

    def handle(self):
        try:
            buf = b""
            while self.keep_alive:
                try:
                    chunk = self.conn.recv(4096)
                except OSError:
                    break
                if not chunk:
                    self.keep_alive = False
                    break

                buf += chunk

                if not self.handshake_done:
                    if b"\r\n\r\n" in buf:
                        headers = self.parse_headers(buf)
                        if "Sec-WebSocket-Key" in headers:
                            self.send_handshake(headers["Sec-WebSocket-Key"])
                            self.handshake_done = True
                            self.server.add_client(self)
                        buf = b""
                else:
                    # Process all complete frames in buffer
                    while True:
                        payload, buf = self.decode_frame(buf)
                        if payload is None:
                            break
                        try:
                            msg = json.loads(payload)
                            if (
                                CMD_CALLBACK
                                and callable(CMD_CALLBACK)
                                and msg.get("type") in ("swarm_command", "set_leader")
                            ):
                                CMD_CALLBACK(msg)
                        except Exception:
                            pass
        except Exception:
            pass
        finally:
            with self.server._broadcast_lock:
                if self in self.server.clients:
                    self.server.clients.remove(self)
            try:
                self.conn.close()
            except Exception:
                pass

    def send_handshake(self, key):
        guid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
        accept_key = base64.b64encode(
            hashlib.sha1((key + guid).encode("utf-8")).digest()
        ).decode("utf-8")
        response = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept_key}\r\n\r\n"
        )
        self.conn.send(response.encode("utf-8"))

    def send_message(self, message):
        try:
            self.conn.sendall(self.encode_frame(message))
        except Exception:
            self.keep_alive = False


class IntegratedServer:
    def __init__(self, port_http=8000, port_ws=8001):
        self.port_http = port_http
        self.port_ws = port_ws
        self.clients = []
        self.running = True

        # Serializes state updates, recipient capture, queue insertion, and
        # client registration. Network delivery happens outside this lock.
        self._broadcast_lock = threading.RLock()

        # Single outbound queue consumed by one sender thread (drops oldest when full)
        self._out_queue = queue.Queue(maxsize=OUTBOUND_QUEUE_SIZE)

        # Retained state used to initialize clients that connect after the
        # first status/telemetry broadcast.
        self._status_snapshot = {}
        self._last_telem = {}

        # Log-rate throttle using token bucket
        self._log_tokens = LOG_RATE_LIMIT_HZ
        self._log_last_refill = time.monotonic()
        self._log_lock = threading.Lock()

    def start(self):
        http_thread = threading.Thread(target=self._run_http, daemon=True)
        http_thread.start()

        ws_thread = threading.Thread(target=self._run_ws, daemon=True)
        ws_thread.start()

        sender_thread = threading.Thread(target=self._sender_loop, daemon=True)
        sender_thread.start()

        print(f"Web Dashboard ready at http://localhost:{self.port_http}")

    def _run_http(self):
        os.chdir(os.path.dirname(os.path.abspath(__file__)))
        if os.path.exists("static"):
            os.chdir("static")

        handler = SimpleHTTPRequestHandler
        handler.log_message = lambda *args: None

        with HTTPServer(("", self.port_http), handler) as httpd:
            httpd.serve_forever()

    def _run_ws(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", self.port_ws))
        sock.listen(10)

        while self.running:
            try:
                conn, addr = sock.accept()
                handler = WebSocketHandler(conn, addr, self)
                t = threading.Thread(target=handler.handle, daemon=True)
                t.start()
            except Exception:
                pass

    def _sender_loop(self):
        """Dedicated thread that drains the outbound queue and sends to all clients."""
        while self.running:
            try:
                msg, recipients = self._out_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            for client in recipients:
                client.send_message(msg)

    def add_client(self, client):
        """Register a handshaken client and enqueue its retained state replay."""
        with self._broadcast_lock:
            self.clients.append(client)
            if self._status_snapshot:
                self._enqueue(
                    {"type": "status", "data": dict(self._status_snapshot)},
                    recipients=(client,),
                )
            if self._last_telem:
                self._enqueue(
                    {
                        "type": "telemetry",
                        "data": {
                            sysid: dict(vals)
                            for sysid, vals in self._last_telem.items()
                        },
                    },
                    recipients=(client,),
                )

    def _enqueue(self, data_dict, recipients=None):
        msg = json.dumps(data_dict, separators=(",", ":"))
        with self._broadcast_lock:
            # Capture recipients at enqueue time. A client connecting later
            # receives its own retained snapshot, never older queued updates.
            target_clients = tuple(self.clients) if recipients is None else recipients
            queued_message = (msg, target_clients)
            try:
                self._out_queue.put_nowait(queued_message)
            except queue.Full:
                # Queue is full: drop the oldest item to make room for the newest.
                # This is safe — the sender loop uses queue.Empty as a normal
                # "nothing to send yet" signal; an extra Empty here is benign.
                try:
                    self._out_queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self._out_queue.put_nowait(queued_message)
                except queue.Full:
                    pass

    # Kept for backwards compatibility; routes through queue
    def broadcast(self, data_dict):
        self._enqueue(data_dict)

    def broadcast_telemetry_dedup(self, data):
        """Broadcast only drone entries whose data has changed since last send."""
        changed = {}
        with self._broadcast_lock:
            for sysid, vals in data.items():
                prev = self._last_telem.get(sysid)
                if prev != vals:
                    changed[sysid] = vals
                    self._last_telem[sysid] = dict(vals)
            if changed:
                self._enqueue({"type": "telemetry", "data": changed})

    def broadcast_status(self, status_dict):
        """Merge and broadcast status fields so reconnects get current state."""
        with self._broadcast_lock:
            self._status_snapshot.update(status_dict)
            self._enqueue({"type": "status", "data": dict(status_dict)})

    def broadcast_log_throttled(self, text):
        """Enqueue a log message, capped at LOG_RATE_LIMIT_HZ messages per second."""
        now = time.monotonic()
        with self._log_lock:
            elapsed = now - self._log_last_refill
            self._log_tokens = min(
                LOG_RATE_LIMIT_HZ, self._log_tokens + elapsed * LOG_RATE_LIMIT_HZ
            )
            self._log_last_refill = now
            if self._log_tokens >= 1.0:
                self._log_tokens -= 1.0
                self._enqueue({"type": "log", "content": text})


# Singleton instance
server = None
CMD_CALLBACK = None


def start_server():
    global server
    if server is None:
        server = IntegratedServer()
        server.start()
    return server


def set_command_callback(func):
    global CMD_CALLBACK
    CMD_CALLBACK = func


def broadcast_log(text):
    if server:
        server.broadcast_log_throttled(text)


def broadcast_telemetry(data):
    if server:
        server.broadcast_telemetry_dedup(data)


def broadcast_status(status_dict):
    """Broadcast a status update (connection mode, leader, etc.) to all clients."""
    if server:
        server.broadcast_status(status_dict)
