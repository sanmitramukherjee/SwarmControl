import threading
import socket
import sys
import os
import base64
import hashlib
import struct
import json
import time
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
            while self.keep_alive:
                data = self.conn.recv(1024)
                if not data: 
                    self.keep_alive = False
                    break
                
                if not self.handshake_done:
                    headers = self.parse_headers(data)
                    if "Sec-WebSocket-Key" in headers:
                        self.send_handshake(headers["Sec-WebSocket-Key"])
                        self.handshake_done = True
                        self.server.clients.append(self)
                else:
                    # Handle incoming frames
                    payload = self.decode_frame(data)
                    if payload:
                        try:
                            msg = json.loads(payload)
                            if msg.get("type") == "swarm_command" and CMD_CALLBACK:
                                CMD_CALLBACK(msg)
                        except:
                            pass
        except:
            pass
        finally:
            if self in self.server.clients:
                self.server.clients.remove(self)
            self.conn.close()

    def parse_headers(self, data):
        headers = {}
        lines = data.decode('utf-8').split('\r\n')
        for line in lines[1:]:
            parts = line.split(': ')
            if len(parts) == 2:
                headers[parts[0]] = parts[1]
        return headers

    def send_handshake(self, key):
        guid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
        accept_key = base64.b64encode(hashlib.sha1((key + guid).encode('utf-8')).digest()).decode('utf-8')
        response = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept_key}\r\n\r\n"
        )
        self.conn.send(response.encode('utf-8'))

    def send_message(self, message):
        try:
            self.conn.send(self.encode_frame(message))
        except:
            self.keep_alive = False

    def decode_frame(self, data):
        # Very basic server-side decoding (handling single-frame masked text)
        try:
            # We assume data is a complete frame for simplicity in this synchronous server
            # Byte 0: FIN + Opcode
            # Byte 1: Mask + Len
            
            if len(data) < 6: return None
            
            second_byte = data[1]
            length = second_byte & 127
            index_first_mask = 2
            
            if length == 126:
                index_first_mask = 4
            elif length == 127:
                index_first_mask = 10
                
            masks = data[index_first_mask : index_first_mask+4]
            index_first_data = index_first_mask + 4
            decoded = bytearray()
            
            # Decrypt
            for i in range(len(data) - index_first_data):
                j = i % 4
                decoded.append(data[index_first_data + i] ^ masks[j])
                
            return decoded.decode('utf-8')
        except:
            return None

    def encode_frame(self, message):
        # Text frame (opcode 0x1)
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
        return frame

class IntegratedServer:
    def __init__(self, port_http=8000, port_ws=8001):
        self.port_http = port_http
        self.port_ws = port_ws
        self.clients = []
        self.running = True
        
    def start(self):
        # 1. Start HTTP Server
        http_thread = threading.Thread(target=self._run_http)
        http_thread.daemon = True
        http_thread.start()
        
        # 2. Start WebSocket Server
        ws_thread = threading.Thread(target=self._run_ws)
        ws_thread.daemon = True
        ws_thread.start()
        
        print(f"Web Dashboard ready at http://localhost:{self.port_http}")

    def _run_http(self):
        # Serve files from 'static' directory
        os.chdir(os.path.dirname(os.path.abspath(__file__)))
        if os.path.exists("static"):
            os.chdir("static")
            
        handler = SimpleHTTPRequestHandler
        # Suppress log messages
        handler.log_message = lambda *args: None
        
        with HTTPServer(("", self.port_http), handler) as httpd:
            httpd.serve_forever()

    def _run_ws(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", self.port_ws))
        sock.listen(5)
        
        while self.running:
            conn, addr = sock.accept()
            handler = WebSocketHandler(conn, addr, self)
            t = threading.Thread(target=handler.handle)
            t.daemon = True
            t.start()

    def broadcast(self, data_dict):
        msg = json.dumps(data_dict)
        # Use a copy of the list to avoid concurrent modification issues
        for client in self.clients[:]:
            client.send_message(msg)

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
        server.broadcast({"type": "log", "content": text})

def broadcast_telemetry(data):
    if server:
        server.broadcast({"type": "telemetry", "data": data})
