"""
SwarmControl — Multi-Drone Ground Control Station
Usage:
  SITL:        python DroneContUI.py --sitl --drones 5762 5772 5782
  Real:        python DroneContUI.py --real --drones COM3:1 COM3:2
  + GUI:       python DroneContUI.py --sitl --drones 5762 5772 --control
  No browser:  python DroneContUI.py --sitl --drones 5762 --no-browser
"""

import argparse
import threading
import math
import time
import sys

from pymavlink import mavutil
from commands import arm_drone, takeoff, land, rtl, fly_to_gps, disarm_drone

# ======================== Configuration Constants ========================
DATA_STREAM_RATE_HZ      = 4     # MAVLink data stream request rate (Hz)
HEARTBEAT_TIMEOUT_S      = 10.0  # Seconds without heartbeat before watchdog reconnects
HEARTBEAT_WARN_S         = 3.0   # Seconds without heartbeat before GUI status turns red
HEARTBEAT_WAIT_REAL_S    = 2.0   # Seconds to wait for heartbeats after serial connect
METERS_PER_DEG_LAT       = 110574.0   # Approximate metres per degree of latitude
METERS_PER_DEG_LON_EQ    = 111320.0   # Approximate metres per degree of longitude at equator

# ======================== CLI Argument Parsing ========================
def parse_args():
    parser = argparse.ArgumentParser(
        description="SwarmControl – Multi-Drone Ground Control Station",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  SITL:      python DroneContUI.py --sitl --drones 5762 5772 5782\n"
            "  Real:      python DroneContUI.py --real --drones COM3:1 COM3:2\n"
            "  + GUI:     python DroneContUI.py --sitl --drones 5762 --control\n"
        ),
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sitl", action="store_true",
                      help="Connect to SITL drones via TCP localhost")
    mode.add_argument("--real", action="store_true",
                      help="Connect to real drones via serial/mesh (COM:SYSID pairs)")

    parser.add_argument(
        "--drones", nargs="+", required=True,
        metavar="PORT_OR_COM:SYSID",
        help=(
            "SITL: TCP port numbers (e.g. 5762 5772). "
            "Real: COM_PORT:SYSID pairs (e.g. COM3:1 COM3:2)"
        ),
    )
    parser.add_argument("--control", action="store_true",
                        help="Launch Tkinter GUI control station alongside the web dashboard")
    parser.add_argument("--baud", type=int, default=57600,
                        help="Baud rate for real drone serial connections (default: 57600)")
    parser.add_argument("--no-browser", action="store_true",
                        help="Do not auto-open the web dashboard in the default browser")
    return parser.parse_args()


# ======================== Formation Engine ========================
def _meters_to_deg(lat_deg):
    """Return (m_per_deg_lat, m_per_deg_lon) at the given latitude."""
    lat_rad = math.radians(lat_deg)
    return METERS_PER_DEG_LAT, METERS_PER_DEG_LON_EQ * math.cos(lat_rad)


def calculate_formation_offsets(leader_lat, leader_lon, leader_heading_deg,
                                follower_count, pattern="V", spacing_m=10.0):
    """
    Compute GPS positions for *follower_count* drones in the requested formation
    relative to the leader.

    Supported patterns:
        "V"    – classic V-shape, followers trailing and fanning outward
        "line" – single-file trail behind leader

    Returns:
        List of (lat, lon) tuples, one per follower.
    """
    m_per_lat, m_per_lon = _meters_to_deg(leader_lat)
    heading_rad = math.radians(leader_heading_deg)

    # Forward unit vector (NED: X=East, Y=North)
    fwd_x = math.sin(heading_rad)
    fwd_y = math.cos(heading_rad)
    # Right unit vector (90° CW from forward)
    right_x = math.cos(heading_rad)
    right_y = -math.sin(heading_rad)

    positions = []

    if pattern == "V":
        for i in range(follower_count):
            row = (i // 2) + 1
            side = 1 if i % 2 == 0 else -1  # even=right, odd=left

            behind_m = row * spacing_m
            side_m = row * spacing_m * side

            # World offsets (East=x, North=y)
            offset_x = -behind_m * fwd_x + side_m * right_x
            offset_y = -behind_m * fwd_y + side_m * right_y

            positions.append((
                leader_lat + offset_y / m_per_lat,
                leader_lon + offset_x / m_per_lon,
            ))

    elif pattern == "line":
        for i in range(follower_count):
            behind_m = (i + 1) * spacing_m
            offset_x = -behind_m * fwd_x
            offset_y = -behind_m * fwd_y
            positions.append((
                leader_lat + offset_y / m_per_lat,
                leader_lon + offset_x / m_per_lon,
            ))

    else:
        # Unknown pattern — stack all followers at leader position
        positions = [(leader_lat, leader_lon)] * follower_count

    return positions


# ======================== Connection Layer ========================
class VirtualConnection:
    """Mimics a mavlink_connection for a specific sysid, routing through SharedConnection."""

    def __init__(self, shared_conn, sysid):
        self.shared_conn = shared_conn
        self.target_system = sysid
        self.target_component = 1
        self.mav = mavutil.mavlink.MAVLink(self, srcSystem=255, srcComponent=0)

    def recv_match(self, condition=None, type=None, blocking=False, timeout=None):
        return self.shared_conn.get_message(self.target_system, type, blocking, timeout)

    def write(self, buf):
        self.shared_conn.write(buf)

    def mav_send(self, msg):
        self.shared_conn.write(msg.pack(self.mav))


class SharedConnection:
    """Manages one physical serial/TCP connection and dispatches MAVLink msgs by sysid."""

    def __init__(self, connection_string, baud=57600):
        self.connection_string = connection_string
        self.baud = baud
        self.master = mavutil.mavlink_connection(connection_string, baud=baud)
        self.buffers = {}  # {sysid: [messages]}
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

    def _reader_loop(self):
        while not self.stop_event.is_set():
            try:
                msg = self.master.recv_match(blocking=True, timeout=1.0)
                if msg:
                    sysid = msg.get_srcSystem()
                    with self.lock:
                        if sysid not in self.buffers:
                            self.buffers[sysid] = []
                        self.buffers[sysid].append(msg)
                        if len(self.buffers[sysid]) > 100:
                            self.buffers[sysid].pop(0)
            except Exception:
                pass

    def get_message(self, sysid, type_filter=None, blocking=False, timeout=None):
        start_time = time.time()
        while True:
            with self.lock:
                if sysid in self.buffers and self.buffers[sysid]:
                    if type_filter:
                        for i, m in enumerate(self.buffers[sysid]):
                            if m.get_type() == type_filter:
                                return self.buffers[sysid].pop(i)
                    else:
                        return self.buffers[sysid].pop(0)
            if not blocking:
                return None
            if timeout and (time.time() - start_time > timeout):
                return None
            time.sleep(0.01)

    def write(self, buf):
        self.master.write(buf)

    def close(self):
        self.stop_event.set()
        try:
            self.master.close()
        except Exception:
            pass


def connect_drones(mode, drone_specs, baud=57600):
    """
    Create MAVLink connections for all specified drones.

    Args:
        mode:        "sitl" or "real"
        drone_specs: SITL – list of port numbers/strings (e.g. ["5762", "5772"])
                     Real – list of "COM:SYSID" strings (e.g. ["COM3:1", "COM3:2"])
        baud:        Baud rate for serial connections.

    Returns:
        (drone_list, shared_connections)
          drone_list         – list of dicts {sysid, connection, port}
          shared_connections – list of SharedConnection objects to close at exit
    """
    drone_list = []
    shared_connections = []

    if mode == "sitl":
        for spec in drone_specs:
            port = int(spec)
            address = f"tcp:127.0.0.1:{port}"
            print(f"Connecting to SITL drone at {address}...")
            try:
                conn = mavutil.mavlink_connection(address)
                msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
                if msg:
                    sysid = conn.target_system
                    print(f"  ✓ Drone sysid={sysid} on port {port}")
                    drone_list.append({"sysid": sysid, "connection": conn, "port": port})
                else:
                    print(f"  ✗ No HEARTBEAT on port {port}")
                    conn.close()
            except Exception as e:
                print(f"  ✗ Error connecting to {address}: {e}")

    elif mode == "real":
        # Group by COM port so one SharedConnection handles all sysids on the same bus
        port_map = {}
        for spec in drone_specs:
            if ":" not in spec:
                print(f"  Invalid real drone spec '{spec}' (expected COM:SYSID). Skipping.")
                continue
            com, _, sysid_str = spec.rpartition(":")
            try:
                sysid = int(sysid_str)
            except ValueError:
                print(f"  Invalid sysid '{sysid_str}' in '{spec}'. Skipping.")
                continue
            port_map.setdefault(com, []).append(sysid)

        for com_port, sysids in port_map.items():
            print(f"Connecting to real drones on {com_port} @ {baud}bps, sysids={sysids}...")
            try:
                shared_conn = SharedConnection(com_port, baud=baud)
                shared_connections.append(shared_conn)
                time.sleep(HEARTBEAT_WAIT_REAL_S)  # Allow heartbeats to arrive
                for sysid in sysids:
                    v_conn = VirtualConnection(shared_conn, sysid)
                    drone_list.append({"sysid": sysid, "connection": v_conn, "port": com_port})
                    print(f"  ✓ Registered sysid={sysid} on {com_port}")
            except Exception as e:
                print(f"  ✗ Error on {com_port}: {e}")

    return drone_list, shared_connections


def start_watchdog(drone_list, mode, baud):
    """
    Background thread: monitors heartbeat age per drone and attempts reconnection.
    Broadcasts live status to the web dashboard.
    """
    def _loop():
        try:
            from web_server import broadcast_log, broadcast_status
        except ImportError:
            return

        while True:
            time.sleep(5)
            for entry in drone_list:
                handler = entry.get("handler")
                if handler is None:
                    continue
                sysid = handler.sysid
                age = time.time() - handler.last_heartbeat
                if age > HEARTBEAT_TIMEOUT_S:
                    broadcast_log(
                        f"[Watchdog] Drone {sysid} lost "
                        f"(no heartbeat for {age:.0f}s). Reconnecting..."
                    )
                    if mode == "sitl":
                        port = entry.get("port")
                        address = f"tcp:127.0.0.1:{port}"
                        try:
                            new_conn = mavutil.mavlink_connection(address)
                            hb = new_conn.recv_match(
                                type="HEARTBEAT", blocking=True, timeout=5
                            )
                            if hb:
                                entry["connection"] = new_conn
                                handler.connection = new_conn
                                handler.request_data_stream()
                                broadcast_log(f"[Watchdog] Drone {sysid} reconnected.")
                            else:
                                new_conn.close()
                        except Exception as e:
                            broadcast_log(
                                f"[Watchdog] Reconnect failed for drone {sysid}: {e}"
                            )
                    broadcast_status({
                        "drone_status": {"sysid": sysid, "connected": age <= HEARTBEAT_TIMEOUT_S}
                    })

    threading.Thread(target=_loop, daemon=True).start()


# ======================== Log Tee ========================
class LoggerTee:
    def __init__(self, stream, callback):
        self.stream = stream
        self.callback = callback

    def write(self, message):
        self.stream.write(message)
        if message.strip():
            self.callback(message.strip())

    def flush(self):
        self.stream.flush()


# ======================== Base Drone Handler (headless) ========================
class BaseDroneHandler:
    """MAVLink telemetry handler – no GUI dependency."""

    def __init__(self, sysid, connection):
        self.sysid = sysid
        self.connection = connection

        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0

        self.lat = 0.0
        self.lon = 0.0
        self.current_alt = 0.0   # relative to home (AGL)
        self.alt_msl = 0.0       # MSL altitude
        self.home_alt = None     # MSL alt at first valid fix

        self.last_heartbeat = 0.0
        self.armed = False

        if self.connection:
            self.request_data_stream()
            threading.Thread(target=self.telemetry_loop, daemon=True).start()

    def request_data_stream(self):
        try:
            self.connection.mav.request_data_stream_send(
                self.sysid,
                self.connection.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                DATA_STREAM_RATE_HZ,
                1,
            )
        except Exception:
            pass

    def telemetry_loop(self):
        while True:
            try:
                msg = self.connection.recv_match(blocking=True, timeout=1)
                if msg:
                    self.process_message(msg)
            except Exception as e:
                print(f"[Drone {self.sysid}] Telemetry error: {e}")
                time.sleep(1)

    def process_message(self, msg):
        t = msg.get_type()
        if t == "GLOBAL_POSITION_INT" and msg.get_srcSystem() == self.sysid:
            self._on_position(msg)
        elif t == "HEARTBEAT" and msg.get_srcSystem() == self.sysid:
            self._on_heartbeat(msg)
        elif t == "ATTITUDE" and msg.get_srcSystem() == self.sysid:
            self._on_attitude(msg)

    def _on_position(self, msg):
        self.lat = msg.lat / 1e7
        self.lon = msg.lon / 1e7
        self.alt_msl = msg.alt / 1000.0
        self.current_alt = msg.relative_alt / 1000.0

        # Capture home altitude on first valid fix
        if self.home_alt is None and abs(self.lat) > 0.001:
            self.home_alt = self.alt_msl - self.current_alt
            print(f"[Drone {self.sysid}] Home alt: {self.home_alt:.2f}m MSL")

        h_speed = math.sqrt(msg.vx ** 2 + msg.vy ** 2) / 100.0
        v_speed = -(msg.vz / 100.0)

        try:
            from web_server import broadcast_telemetry
            broadcast_telemetry({
                self.sysid: {
                    "lat": self.lat,
                    "lon": self.lon,
                    "alt": self.current_alt,
                    "alt_msl": self.alt_msl,
                    "home_alt": self.home_alt if self.home_alt is not None else 0.0,
                    "pitch": self.pitch,
                    "roll": self.roll,
                    "yaw": self.yaw,
                    "h_speed": h_speed,
                    "v_speed": v_speed,
                    "armed": self.armed,
                }
            })
        except ImportError:
            pass

    def _on_heartbeat(self, msg):
        self.last_heartbeat = time.time()
        self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def _on_attitude(self, msg):
        self.roll = math.degrees(msg.roll)
        self.pitch = math.degrees(msg.pitch)
        self.yaw = math.degrees(msg.yaw)

    # ---- Commands ----
    def arm_drone(self):
        if self.connection:
            arm_drone(self.connection, self.sysid)

    def disarm_drone(self):
        if self.connection:
            disarm_drone(self.connection, self.sysid)

    def takeoff(self, altitude):
        if self.connection:
            takeoff(self.connection, self.sysid, altitude)

    def fly_to_gps(self, lat, lon, alt, use_msl=False):
        if self.connection:
            fly_to_gps(self.connection, self.sysid, lat, lon, alt, use_msl=use_msl)

    def rtl(self):
        if self.connection:
            rtl(self.connection, self.sysid)

    def land(self):
        if self.connection:
            land(self.connection, self.sysid)


# ======================== Tkinter GUI Drone Handler ========================
# Imported lazily so headless mode never touches tkinter.

def _build_gui_handler_class():
    """Returns the DroneHandler class (Tkinter). Called only when --control is used."""
    import tkinter as tk
    from tkinter import font as tkfont

    COLORS = {
        'bg_dark': '#1a1a1a',
        'bg_panel': '#2a2a2a',
        'bg_header': '#333333',
        'border': '#4a4a4a',
        'text_primary': '#ffffff',
        'text_secondary': '#b0b0b0',
        'accent_blue': '#2196f3',
        'accent_cyan': '#00d4ff',
        'status_good': '#4caf50',
        'status_warning': '#ffc107',
        'status_critical': '#f44336',
        'btn_arm': '#616161',
        'btn_takeoff': '#4caf50',
        'btn_flyto': '#ffc107',
        'btn_rtl': '#ff9800',
        'btn_land': '#2196f3',
    }

    class DroneHandler(BaseDroneHandler):
        def __init__(self, sysid, connection, container):
            # GUI widgets (set before super().__init__ starts threads)
            self.gps_label = None
            self.speed_label = None
            self.status_label = None
            self.status_light = None
            self._colors = COLORS

            title = f"DRONE {sysid}" if sysid != "N/A" else "DRONE (DISCONNECTED)"
            self.frame = tk.LabelFrame(
                container, text=title, padx=4, pady=4,
                font=("Consolas", 9, "bold"),
                bg=COLORS['bg_panel'], fg=COLORS['text_primary'],
                bd=1, relief="flat",
                highlightbackground=COLORS['border'], highlightthickness=1,
            )
            self.frame.pack(side="left", padx=3, pady=3, fill="both", expand=True)

            header_frame = tk.Frame(self.frame, bg=COLORS['bg_panel'])
            header_frame.pack(fill="x", pady=(0, 3))

            self.status_light = tk.Canvas(
                header_frame, width=20, height=20,
                bg=COLORS['bg_panel'], highlightthickness=0,
            )
            self.status_light.pack(side="right", padx=3)

            display_frame = tk.Frame(self.frame, bg=COLORS['bg_panel'])
            display_frame.pack(fill="x", pady=3)

            self.canvas = tk.Canvas(
                display_frame, width=100, height=100,
                bg='#0a0a0a', highlightthickness=1,
                highlightbackground=COLORS['border'],
            )
            self.canvas.pack(side="left", padx=(0, 6))

            dashboard_frame = tk.Frame(display_frame, bg=COLORS['bg_panel'])
            dashboard_frame.pack(side="left", fill="both", expand=True)

            tf = ("Consolas", 8)
            self.gps_label = tk.Label(
                dashboard_frame, text="Lat: --, Lon: --, Alt: --",
                font=tf, bg=COLORS['bg_panel'], fg=COLORS['accent_cyan'], anchor="w",
            )
            self.gps_label.pack(fill="x", pady=1)

            self.speed_label = tk.Label(
                dashboard_frame, text="H.Spd: --, V.Spd: -- m/s",
                font=tf, bg=COLORS['bg_panel'], fg=COLORS['accent_cyan'], anchor="w",
            )
            self.speed_label.pack(fill="x", pady=1)

            self.status_label = tk.Label(
                dashboard_frame, text="Status: --",
                font=tf, bg=COLORS['bg_panel'], fg=COLORS['text_secondary'], anchor="w",
            )
            self.status_label.pack(fill="x", pady=1)

            separator = tk.Frame(self.frame, height=1, bg=COLORS['border'])
            separator.pack(fill="x", pady=3)

            inputs = tk.Frame(self.frame, bg=COLORS['bg_panel'])
            inputs.pack(fill="x", pady=2)

            lf = ("Consolas", 8, "bold")
            ef = ("Consolas", 8)
            for row_idx, label in enumerate(["LAT:", "LON:", "ALT:"]):
                tk.Label(inputs, text=label, font=lf,
                         bg=COLORS['bg_panel'], fg=COLORS['text_secondary']
                         ).grid(row=row_idx, column=0, sticky="e", padx=(0, 3), pady=1)
            self.lat_entry = tk.Entry(
                inputs, width=10, font=ef,
                bg=COLORS['bg_header'], fg=COLORS['text_primary'],
                insertbackground=COLORS['text_primary'], relief="flat", bd=1,
            )
            self.lat_entry.grid(row=0, column=1, padx=2, pady=1, sticky="ew")
            self.lon_entry = tk.Entry(
                inputs, width=10, font=ef,
                bg=COLORS['bg_header'], fg=COLORS['text_primary'],
                insertbackground=COLORS['text_primary'], relief="flat", bd=1,
            )
            self.lon_entry.grid(row=1, column=1, padx=2, pady=1, sticky="ew")
            self.alt_entry = tk.Entry(
                inputs, width=10, font=ef,
                bg=COLORS['bg_header'], fg=COLORS['text_primary'],
                insertbackground=COLORS['text_primary'], relief="flat", bd=1,
            )
            self.alt_entry.grid(row=2, column=1, padx=2, pady=1, sticky="ew")
            inputs.grid_columnconfigure(1, weight=1)

            btns = tk.Frame(self.frame, bg=COLORS['bg_panel'])
            btns.pack(fill="x", pady=(3, 0))

            bf = ("Consolas", 8, "bold")

            def _threaded(func):
                return lambda: threading.Thread(target=func, daemon=True).start() if connection else None

            def _btn(parent, text, command, bg_color, row, col, colspan=1):
                b = tk.Button(
                    parent, text=text, command=command,
                    bg=bg_color, fg='white', font=bf,
                    relief="flat", bd=0, padx=4, pady=3,
                    cursor="hand2" if connection else "arrow",
                )
                b.grid(row=row, column=col, columnspan=colspan,
                       padx=1, pady=1, sticky="ew")
                if connection:
                    b.bind("<Enter>", lambda e: b.config(bg=self._lighten(bg_color)))
                    b.bind("<Leave>", lambda e: b.config(bg=bg_color))
                else:
                    b.config(bg=COLORS['bg_header'], cursor="arrow")
                return b

            _btn(btns, "ARM", _threaded(self.arm_drone), COLORS['btn_arm'], 0, 0)
            _btn(btns, "TAKEOFF",
                 _threaded(lambda: self.takeoff(float(self.alt_entry.get() or 3))),
                 COLORS['btn_takeoff'], 0, 1)
            _btn(btns, "FLY TO",
                 _threaded(lambda: self.fly_to_gps(
                     float(self.lat_entry.get()),
                     float(self.lon_entry.get()),
                     float(self.alt_entry.get() or 3),
                 )),
                 COLORS['btn_flyto'], 1, 0)
            _btn(btns, "RTL", _threaded(self.rtl), COLORS['btn_rtl'], 1, 1)
            _btn(btns, "LAND", _threaded(self.land), COLORS['btn_land'], 2, 0, 2)
            btns.grid_columnconfigure(0, weight=1)
            btns.grid_columnconfigure(1, weight=1)

            # Call parent __init__ AFTER widgets exist so _on_position can update them
            super().__init__(sysid, connection)
            self.set_status_light("red")
            self.update_pfd()
            if connection:
                threading.Thread(target=self._check_connection_status, daemon=True).start()
            else:
                self.status_label.config(text="Status: Disconnected")

        # ---- GUI helpers ----
        def _lighten(self, color):
            cmap = {
                'red': COLORS['status_critical'],
                'green': COLORS['status_good'],
                'yellow': COLORS['status_warning'],
            }
            color = cmap.get(color, color)
            h = color.lstrip('#')
            r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
            return (f'#{min(255, int(r*1.2)):02x}'
                    f'{min(255, int(g*1.2)):02x}'
                    f'{min(255, int(b*1.2)):02x}')

        def set_status_light(self, color):
            cmap = {
                'red': COLORS['status_critical'],
                'green': COLORS['status_good'],
                'yellow': COLORS['status_warning'],
            }
            color = cmap.get(color, color)
            self.status_light.delete("all")
            self.status_light.create_oval(2, 2, 18, 18,
                                          fill=color, outline=self._lighten(color), width=2)
            self.status_light.create_oval(5, 5, 15, 15,
                                          fill=self._lighten(color), outline="")

        def _check_connection_status(self):
            while True:
                age = time.time() - self.last_heartbeat
                color = COLORS['status_critical'] if age > HEARTBEAT_WARN_S else COLORS['status_good']
                self.set_status_light(color)
                time.sleep(1)

        # ---- Override telemetry callbacks to also update GUI ----
        def _on_position(self, msg):
            super()._on_position(msg)
            h_speed = math.sqrt(msg.vx ** 2 + msg.vy ** 2) / 100.0
            v_speed = -(msg.vz / 100.0)
            try:
                self.gps_label.config(
                    text=f"Lat: {self.lat:.6f}, Lon: {self.lon:.6f}, Alt: {self.current_alt:.1f}m"
                )
                self.speed_label.config(
                    text=f"H.Spd: {h_speed:.1f}, V.Spd: {v_speed:.1f} m/s"
                )
            except Exception:
                pass

        def _on_heartbeat(self, msg):
            super()._on_heartbeat(msg)
            status = "Armed" if self.armed else "Disarmed"
            try:
                self.status_label.config(text=f"Status: {status}")
            except Exception:
                pass

        def update_pfd(self):
            try:
                self.canvas.delete("all")
                w, h = 100, 100
                cx, cy = w // 2, h // 2
                pitch_shift = self.pitch * 1.0
                roll_angle = math.radians(self.roll)

                self.canvas.create_rectangle(0, 0, w, h // 2, fill="#1a4d7a", outline="")
                self.canvas.create_rectangle(0, h // 2, w, h, fill="#3d2817", outline="")

                line_len = 150
                x1 = cx - line_len * math.cos(roll_angle)
                y1 = cy - line_len * math.sin(roll_angle) + pitch_shift
                x2 = cx + line_len * math.cos(roll_angle)
                y2 = cy + line_len * math.sin(roll_angle) + pitch_shift
                self.canvas.create_line(x1, y1, x2, y2, fill=COLORS['text_primary'], width=2)

                self.canvas.create_line(cx - 15, cy, cx + 15, cy, fill=COLORS['accent_cyan'], width=2)
                self.canvas.create_line(cx, cy - 7, cx, cy + 7, fill=COLORS['accent_cyan'], width=2)
                self.canvas.create_oval(cx - 2, cy - 2, cx + 2, cy + 2,
                                        fill=COLORS['accent_cyan'], outline="")

                self.canvas.create_rectangle(cx - 20, 6, cx + 20, 18,
                                             fill=COLORS['bg_header'], outline=COLORS['border'])
                self.canvas.create_text(cx, 12, text=f"{self.yaw:.0f}°",
                                        fill=COLORS['text_primary'],
                                        font=("Consolas", 8, "bold"))
                self.canvas.create_text(8, h - 8, text=f"P:{self.pitch:.0f}°",
                                        fill=COLORS['text_secondary'],
                                        font=("Consolas", 7), anchor="w")
                self.canvas.create_text(w - 8, h - 8, text=f"R:{self.roll:.0f}°",
                                        fill=COLORS['text_secondary'],
                                        font=("Consolas", 7), anchor="e")
                self.canvas.after(100, self.update_pfd)
            except Exception:
                pass

    return DroneHandler, COLORS


# ======================== Swarm Logic ========================
# Module-level state for swarm
_leader_sysid = -1  # -1 = auto (lowest sysid)
_leader_lock = threading.Lock()


def set_leader(sysid):
    global _leader_sysid
    with _leader_lock:
        _leader_sysid = int(sysid)
    print(f"[Swarm] Leader set to drone {sysid}")
    try:
        from web_server import broadcast_status
        broadcast_status({"leader": sysid})
    except ImportError:
        pass


def get_leader(active_handlers):
    """Return the leader handler from active_handlers."""
    with _leader_lock:
        lsid = _leader_sysid
    if lsid != -1:
        for h in active_handlers:
            if h.sysid == lsid:
                return h
    # Fallback: lowest sysid
    return active_handlers[0] if active_handlers else None


def perform_swarm_command(cmd_data, drone_handlers):
    cmd = cmd_data.get("cmd")
    print(f"[Swarm] Command: {cmd}")

    # Handle set_leader as a direct message (not a drone command)
    if cmd_data.get("type") == "set_leader":
        set_leader(cmd_data.get("sysid", -1))
        return

    formation = cmd_data.get("formation", False)
    pattern = cmd_data.get("pattern", "V")
    spacing_m = float(cmd_data.get("spacing_m", 10.0))

    active_handlers = [h for h in drone_handlers if h.sysid != "N/A" and h.connection]
    if not active_handlers:
        return

    active_handlers.sort(key=lambda h: h.sysid)

    if cmd == "arm":
        for h in active_handlers:
            threading.Thread(target=h.arm_drone, daemon=True).start()

    elif cmd == "disarm":
        for h in active_handlers:
            threading.Thread(target=h.disarm_drone, daemon=True).start()

    elif cmd == "takeoff":
        alt = float(cmd_data.get("alt", 5))
        for h in active_handlers:
            threading.Thread(target=h.takeoff, args=(alt,), daemon=True).start()

    elif cmd == "rtl":
        for h in active_handlers:
            threading.Thread(target=h.rtl, daemon=True).start()

    elif cmd == "land":
        for h in active_handlers:
            threading.Thread(target=h.land, daemon=True).start()

    elif cmd == "fly_to":
        try:
            lat = float(cmd_data.get("lat"))
            lon = float(cmd_data.get("lon"))
            alt = float(cmd_data.get("alt"))

            leader = get_leader(active_handlers)
            if leader is None:
                return

            if formation and len(active_handlers) > 1:
                # Leader flies to target
                threading.Thread(
                    target=leader.fly_to_gps, args=(lat, lon, alt), daemon=True
                ).start()

                followers = [h for h in active_handlers if h is not leader]

                # Use leader's current heading (yaw) for formation orientation
                heading = leader.yaw

                offsets = calculate_formation_offsets(
                    lat, lon, heading,
                    follower_count=len(followers),
                    pattern=pattern,
                    spacing_m=spacing_m,
                )
                for h, (f_lat, f_lon) in zip(followers, offsets):
                    threading.Thread(
                        target=h.fly_to_gps, args=(f_lat, f_lon, alt), daemon=True
                    ).start()
            else:
                for h in active_handlers:
                    threading.Thread(
                        target=h.fly_to_gps, args=(lat, lon, alt), daemon=True
                    ).start()

        except Exception as e:
            print(f"[Swarm] fly_to error: {e}")


# ======================== Main Entry Point ========================
def main():
    args = parse_args()
    mode = "sitl" if args.sitl else "real"

    # Start web server first
    try:
        import web_server
        web_server.start_server()
        sys.stdout = LoggerTee(sys.stdout, web_server.broadcast_log)
    except Exception as e:
        print(f"Failed to start web server: {e}")

    # Connect drones
    drone_list, shared_connections = connect_drones(mode, args.drones, baud=args.baud)

    if not drone_list:
        print("No drones found. Exiting.")
        sys.exit(1)

    # Broadcast initial status
    try:
        from web_server import broadcast_status
        broadcast_status({
            "mode": mode.upper(),
            "drone_count": len(drone_list),
            "leader": -1,
        })
    except ImportError:
        pass

    # Build handlers
    drone_handlers = []

    if args.control:
        # GUI mode
        DroneHandler, COLORS = _build_gui_handler_class()
        import tkinter as tk

        root = tk.Tk()
        connected_count = len(drone_list)
        root.title(f"Multi-Drone Ground Control Station – {mode.upper()} MODE")
        root.state("zoomed")
        root.configure(bg=COLORS['bg_dark'])

        # Title bar
        title_bar = tk.Frame(root, bg=COLORS['bg_header'], height=50)
        title_bar.pack(fill="x")

        tk.Label(
            title_bar,
            text="MULTI-DRONE GROUND CONTROL STATION",
            font=("Consolas", 16, "bold"),
            bg=COLORS['bg_header'], fg=COLORS['accent_cyan'],
        ).pack(side="left", padx=20, pady=10)

        btn_frame = tk.Frame(title_bar, bg=COLORS['bg_header'])
        btn_frame.pack(side="right", padx=10)

        def _launch_map():
            import webbrowser
            webbrowser.open("http://localhost:8000")

        tk.Button(
            btn_frame, text="LAUNCH 3D MAP", command=_launch_map,
            bg=COLORS['accent_blue'], fg="white",
            font=("Consolas", 10, "bold"), relief="flat", padx=10,
        ).pack(side="left", padx=10)

        tk.Label(
            title_bar,
            text=f"● {connected_count} DRONES CONNECTED ({mode.upper()})",
            font=("Consolas", 12),
            bg=COLORS['bg_header'],
            fg=COLORS['status_good'] if connected_count > 0 else COLORS['status_critical'],
        ).pack(side="right", padx=20, pady=10)

        content_frame = tk.Frame(root, bg=COLORS['bg_dark'])
        content_frame.pack(fill="both", expand=True, padx=5, pady=5)

        top_frame = tk.Frame(content_frame, bg=COLORS['bg_dark'])
        top_frame.pack(fill="both", expand=True)
        bottom_frame = tk.Frame(content_frame, bg=COLORS['bg_dark'])
        bottom_frame.pack(fill="both", expand=True)

        for i, data in enumerate(drone_list):
            sysid = data["sysid"]
            conn = data["connection"]
            container = top_frame if i < 4 else bottom_frame
            handler = DroneHandler(sysid, conn, container)
            drone_handlers.append(handler)
            data["handler"] = handler

        # Fill remaining slots with empty panels (up to 8)
        for i in range(len(drone_list), 8):
            container = top_frame if i < 4 else bottom_frame
            h = DroneHandler("N/A", None, container)
            drone_handlers.append(h)

    else:
        # Headless mode – BaseDroneHandler only
        for data in drone_list:
            handler = BaseDroneHandler(data["sysid"], data["connection"])
            drone_handlers.append(handler)
            data["handler"] = handler

    # Register swarm command callback
    try:
        from web_server import set_command_callback
        set_command_callback(lambda msg: perform_swarm_command(msg, drone_handlers))
    except ImportError:
        pass

    # Start connection watchdog
    start_watchdog(drone_list, mode, args.baud)

    # Optionally open browser
    if not args.no_browser:
        try:
            import webbrowser
            webbrowser.open("http://localhost:8000")
        except Exception:
            pass

    if args.control:
        try:
            root.mainloop()
        except KeyboardInterrupt:
            pass
    else:
        print("[SwarmControl] Running in headless mode. Web dashboard: http://localhost:8000")
        print("Press Ctrl+C to quit.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass

    # Cleanup
    print("Shutting down...")
    for data in drone_list:
        conn = data.get("connection")
        if conn and not isinstance(conn, VirtualConnection):
            try:
                conn.close()
            except Exception:
                pass
    for sc in shared_connections:
        sc.close()


if __name__ == "__main__":
    main()
