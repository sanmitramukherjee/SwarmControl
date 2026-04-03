import threading
import socket
import sys
import os
import base64
import hashlib
import struct
import json
import time
import queue
from http.server import SimpleHTTPRequestHandler, HTTPServer

# ------------------- lightweight WebSocket Server -------------------
class WebSocketHandler:
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
                            self.server.clients.append(self)
                        buf = b""
                else:
                    # Process all complete frames in buffer
                    while True:
                        payload, buf = self.decode_frame(buf)
                        if payload is None:
                            break
                        try:
                            msg = json.loads(payload)
                            if CMD_CALLBACK and msg.get("type") in (
                                "swarm_command", "set_leader"
                            ):
                                CMD_CALLBACK(msg)
                        except Exception:
                            pass
        except Exception:
            pass
        finally:
            if self in self.server.clients:
                self.server.clients.remove(self)
            try:
                self.conn.close()
            except Exception:
                pass

    def parse_headers(self, data):
        headers = {}
        try:
            lines = data.decode('utf-8').split('\r\n')
        except Exception:
            return headers
        for line in lines[1:]:
            if ': ' in line:
                key, _, val = line.partition(': ')
                headers[key] = val
        return headers

    def send_handshake(self, key):
        guid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
        accept_key = base64.b64encode(
            hashlib.sha1((key + guid).encode('utf-8')).digest()
        ).decode('utf-8')
        response = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept_key}\r\n\r\n"
        )
        self.conn.send(response.encode('utf-8'))

    def send_message(self, message):
        try:
            self.conn.sendall(self.encode_frame(message))
        except Exception:
            self.keep_alive = False

    def decode_frame(self, data):
        """Return (payload_str, remaining_bytes) or (None, data) if frame incomplete."""
        try:
            if len(data) < 2:
                return None, data

            second_byte = data[1]
            masked = bool(second_byte & 0x80)
            length = second_byte & 0x7F
            idx = 2

            if length == 126:
                if len(data) < 4:
                    return None, data
                length = struct.unpack("!H", data[2:4])[0]
                idx = 4
            elif length == 127:
                if len(data) < 10:
                    return None, data
                length = struct.unpack("!Q", data[2:10])[0]
                idx = 10

            if masked:
                if len(data) < idx + 4 + length:
                    return None, data
                masks = data[idx: idx + 4]
                idx += 4
                decoded = bytearray(
                    data[idx + i] ^ masks[i % 4] for i in range(length)
                )
            else:
                if len(data) < idx + length:
                    return None, data
                decoded = data[idx: idx + length]

            remaining = data[idx + length:]
            return decoded.decode('utf-8'), remaining
        except Exception:
            return None, b""

    def encode_frame(self, message):
        data = message.encode('utf-8')
        length = len(data)
        frame = bytearray([0x81])

        if length <= 125:
            frame.append(length)
        elif length <= 65535:
            frame.append(126)
            frame.extend(struct.pack("!H", length))
        else:
            frame.append(127)
            frame.extend(struct.pack("!Q", length))

        frame.extend(data)
        return bytes(frame)


class IntegratedServer:
    def __init__(self, port_http=8000, port_ws=8001):
        self.port_http = port_http
        self.port_ws = port_ws
        self.clients = []
        self.running = True

        # Single outbound queue consumed by one sender thread
        self._out_queue = queue.Queue(maxsize=256)

        # Telemetry deduplication: last broadcast snapshot per drone
        self._last_telem = {}
        self._telem_lock = threading.Lock()

        # Log-rate throttle: max 10 msgs/s
        self._log_tokens = 10.0
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
                msg = self._out_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            clients_snapshot = self.clients[:]
            for client in clients_snapshot:
                client.send_message(msg)

    def _enqueue(self, data_dict):
        msg = json.dumps(data_dict, separators=(',', ':'))
        try:
            self._out_queue.put_nowait(msg)
        except queue.Full:
            # Drop oldest, enqueue new
            try:
                self._out_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._out_queue.put_nowait(msg)
            except queue.Full:
                pass

    # Kept for backwards compatibility; routes through queue
    def broadcast(self, data_dict):
        self._enqueue(data_dict)

    def broadcast_telemetry_dedup(self, data):
        """Broadcast only drone entries whose data has changed since last send."""
        changed = {}
        with self._telem_lock:
            for sysid, vals in data.items():
                prev = self._last_telem.get(sysid)
                if prev != vals:
                    changed[sysid] = vals
                    self._last_telem[sysid] = dict(vals)
        if changed:
            self._enqueue({"type": "telemetry", "data": changed})

    def broadcast_log_throttled(self, text):
        """Enqueue a log message, capped at 10 messages per second."""
        now = time.monotonic()
        with self._log_lock:
            elapsed = now - self._log_last_refill
            self._log_tokens = min(10.0, self._log_tokens + elapsed * 10.0)
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
        server._enqueue({"type": "status", "data": status_dict})
