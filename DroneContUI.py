import threading
import tkinter as tk
from tkinter import font as tkfont
from pymavlink import mavutil
import math
import time
from commands import arm_drone, takeoff, land, rtl, fly_to_gps, disarm_drone

# ------------------- Color Scheme -------------------
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
    'btn_land': '#2196f3'
}

# ------------------- Drone Handler -------------------
class DroneHandler:
    def __init__(self, sysid, connection, container):
        self.sysid = sysid
        self.connection = connection
        self.gps_label = None
        self.speed_label = None
        self.status_label = None
        self.status_light = None
        
        # Attitude data
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        
        # Last heartbeat time for status monitoring
        self.last_heartbeat = 0
        
        # Current altitude for control loops
        self.current_alt = 0.0

        # Title for the frame
        title = f"DRONE {self.sysid}" if self.sysid != "N/A" else "DRONE (DISCONNECTED)"
        
        # Create Frame with dark theme
        self.frame = tk.LabelFrame(
            container, 
            text=title, 
            padx=4, 
            pady=4, 
            font=("Consolas", 9, "bold"),
            bg=COLORS['bg_panel'],
            fg=COLORS['text_primary'],
            bd=1,
            relief="flat",
            highlightbackground=COLORS['border'],
            highlightthickness=1
        )
        self.frame.pack(side="left", padx=3, pady=3, fill="both", expand=True)

        # Header with Status Light
        header_frame = tk.Frame(self.frame, bg=COLORS['bg_panel'])
        header_frame.pack(fill="x", pady=(0, 3))
        
        # Status Light (compact)
        self.status_light = tk.Canvas(
            header_frame, 
            width=20, 
            height=20, 
            bg=COLORS['bg_panel'],
            highlightthickness=0
        )
        self.status_light.pack(side="right", padx=3)
        self.set_status_light("red")

        # Container for PFD and Dashboard side by side
        display_frame = tk.Frame(self.frame, bg=COLORS['bg_panel'])
        display_frame.pack(fill="x", pady=3)

        # Primary Flight Display (compact)
        self.canvas = tk.Canvas(
            display_frame, 
            width=100, 
            height=100, 
            bg='#0a0a0a',
            highlightthickness=1,
            highlightbackground=COLORS['border']
        )
        self.canvas.pack(side="left", padx=(0, 6))

        # Dashboard (GPS, Speed, Status) with better styling
        dashboard_frame = tk.Frame(display_frame, bg=COLORS['bg_panel'])
        dashboard_frame.pack(side="left", fill="both", expand=True)
        
        # Telemetry labels with monospace font
        telemetry_font = ("Consolas", 8)
        
        self.gps_label = tk.Label(
            dashboard_frame, 
            text="Lat: --, Lon: --, Alt: --", 
            font=telemetry_font,
            bg=COLORS['bg_panel'],
            fg=COLORS['accent_cyan'],
            anchor="w"
        )
        self.gps_label.pack(fill="x", pady=1)
        
        self.speed_label = tk.Label(
            dashboard_frame, 
            text="H.Spd: --, V.Spd: -- m/s", 
            font=telemetry_font,
            bg=COLORS['bg_panel'],
            fg=COLORS['accent_cyan'],
            anchor="w"
        )
        self.speed_label.pack(fill="x", pady=1)
        
        self.status_label = tk.Label(
            dashboard_frame, 
            text="Status: --", 
            font=telemetry_font,
            bg=COLORS['bg_panel'],
            fg=COLORS['text_secondary'],
            anchor="w"
        )
        self.status_label.pack(fill="x", pady=1)

        # Separator
        separator = tk.Frame(self.frame, height=1, bg=COLORS['border'])
        separator.pack(fill="x", pady=3)

        # Inputs with better styling
        inputs = tk.Frame(self.frame, bg=COLORS['bg_panel'])
        inputs.pack(fill="x", pady=2)
        
        input_font = ("Consolas", 8)
        label_font = ("Consolas", 8, "bold")
        
        # Lat
        tk.Label(inputs, text="LAT:", font=label_font, bg=COLORS['bg_panel'], fg=COLORS['text_secondary']).grid(row=0, column=0, sticky="e", padx=(0, 3), pady=1)
        self.lat_entry = tk.Entry(inputs, width=10, font=input_font, bg=COLORS['bg_header'], fg=COLORS['text_primary'], insertbackground=COLORS['text_primary'], relief="flat", bd=1)
        self.lat_entry.grid(row=0, column=1, padx=2, pady=1, sticky="ew")
        
        # Lon
        tk.Label(inputs, text="LON:", font=label_font, bg=COLORS['bg_panel'], fg=COLORS['text_secondary']).grid(row=1, column=0, sticky="e", padx=(0, 3), pady=1)
        self.lon_entry = tk.Entry(inputs, width=10, font=input_font, bg=COLORS['bg_header'], fg=COLORS['text_primary'], insertbackground=COLORS['text_primary'], relief="flat", bd=1)
        self.lon_entry.grid(row=1, column=1, padx=2, pady=1, sticky="ew")
        
        # Alt
        tk.Label(inputs, text="ALT:", font=label_font, bg=COLORS['bg_panel'], fg=COLORS['text_secondary']).grid(row=2, column=0, sticky="e", padx=(0, 3), pady=1)
        self.alt_entry = tk.Entry(inputs, width=10, font=input_font, bg=COLORS['bg_header'], fg=COLORS['text_primary'], insertbackground=COLORS['text_primary'], relief="flat", bd=1)
        self.alt_entry.grid(row=2, column=1, padx=2, pady=1, sticky="ew")
        
        inputs.grid_columnconfigure(1, weight=1)

        # Buttons with professional styling
        btns = tk.Frame(self.frame, bg=COLORS['bg_panel'])
        btns.pack(fill="x", pady=(3, 0))
        
        btn_font = ("Consolas", 8, "bold")
        
        # Helper to create styled buttons
        def create_cmd(func):
            return lambda: threading.Thread(target=func, daemon=True).start() if self.connection else None

        def create_button(parent, text, command, bg_color, row, col, colspan=1):
            btn = tk.Button(
                parent,
                text=text,
                command=command,
                bg=bg_color,
                fg='white',
                font=btn_font,
                relief="flat",
                bd=0,
                padx=4,
                pady=3,
                cursor="hand2" if self.connection else "arrow"
            )
            btn.grid(row=row, column=col, columnspan=colspan, padx=1, pady=1, sticky="ew")
            
            # Hover effects
            if self.connection:
                btn.bind("<Enter>", lambda e: btn.config(bg=self._lighten_color(bg_color)))
                btn.bind("<Leave>", lambda e: btn.config(bg=bg_color))
            else:
                btn.config(bg=COLORS['bg_header'], cursor="arrow")
            
            return btn

        create_button(btns, "ARM", create_cmd(self.arm_drone), COLORS['btn_arm'], 0, 0)
        create_button(btns, "TAKEOFF", create_cmd(lambda: self.takeoff(float(self.alt_entry.get() or 3))), COLORS['btn_takeoff'], 0, 1)
        create_button(btns, "FLY TO", create_cmd(lambda: self.fly_to_gps(float(self.lat_entry.get()), float(self.lon_entry.get()), float(self.alt_entry.get() or 3))), COLORS['btn_flyto'], 1, 0)
        create_button(btns, "RTL", create_cmd(self.rtl), COLORS['btn_rtl'], 1, 1)
        create_button(btns, "LAND", create_cmd(self.land), COLORS['btn_land'], 2, 0, 2)
        
        btns.grid_columnconfigure(0, weight=1)
        btns.grid_columnconfigure(1, weight=1)

        self.update_pfd()
        
        # Start telemetry loop for this drone if connected
        if self.connection:
            # Request data stream to ensure we get GLOBAL_POSITION_INT
            self.request_data_stream()
            
            threading.Thread(target=self.telemetry_loop, daemon=True).start()
            # Start status checker
            threading.Thread(target=self.check_connection_status, daemon=True).start()
        else:
            # If not connected, ensure status is red
            self.set_status_light("red")
            self.status_label.config(text="Status: Disconnected")

    def _lighten_color(self, color):
        """Lighten a hex color for hover effect"""
        # Handle color names
        color_map = {
            'red': COLORS['status_critical'],
            'green': COLORS['status_good'],
            'yellow': COLORS['status_warning']
        }
        if color in color_map:
            color = color_map[color]
            
        hex_color = color.lstrip('#')
        r, g, b = tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
        r = min(255, int(r * 1.2))
        g = min(255, int(g * 1.2))
        b = min(255, int(b * 1.2))
        return f'#{r:02x}{g:02x}{b:02x}'

    def request_data_stream(self):
        # Request all data streams at 2Hz
        self.connection.mav.request_data_stream_send(
            self.sysid,
            self.connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            2, # Rate in Hz
            1  # Start
        )

    def set_status_light(self, color):
        # Map color names to hex
        color_map = {
            'red': COLORS['status_critical'],
            'green': COLORS['status_good'],
            'yellow': COLORS['status_warning']
        }
        if color in color_map:
            color = color_map[color]
            
        self.status_light.delete("all")
        # Draw outer ring
        self.status_light.create_oval(2, 2, 18, 18, fill=color, outline=self._lighten_color(color), width=2)
        # Draw inner glow
        self.status_light.create_oval(5, 5, 15, 15, fill=self._lighten_color(color), outline="")

    def check_connection_status(self):
        while True:
            if time.time() - self.last_heartbeat > 3:
                self.set_status_light(COLORS['status_critical'])
            else:
                self.set_status_light(COLORS['status_good'])
            time.sleep(1)

    def arm_drone(self):
        if self.connection: arm_drone(self.connection, self.sysid)

    def disarm_drone(self):
        if self.connection: disarm_drone(self.connection, self.sysid)

    def takeoff(self, altitude):
        if self.connection: takeoff(self.connection, self.sysid, altitude)

    def fly_to_gps(self, lat, lon, alt):
        if self.connection: fly_to_gps(self.connection, self.sysid, lat, lon, alt)

    def rtl(self):
        if self.connection: rtl(self.connection, self.sysid)

    def land(self):
        if self.connection: land(self.connection, self.sysid)

    # ---------------- Telemetry Updates ----------------
    def telemetry_loop(self):
        while True:
            try:
                msg = self.connection.recv_match(blocking=True, timeout=1)
                if msg:
                    self.update_dashboard(msg)
            except Exception as e:
                print(f"Error in telemetry loop for drone {self.sysid}: {e}")
                time.sleep(1)

    def update_dashboard(self, msg):
        if msg.get_type() == "GLOBAL_POSITION_INT" and msg.get_srcSystem() == self.sysid:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            self.current_alt = msg.relative_alt / 1000.0
            
            # Calculate speeds
            h_speed = math.sqrt(msg.vx**2 + msg.vy**2) / 100.0
            v_speed = -(msg.vz / 100.0)
            
            self.gps_label.config(text=f"Lat: {lat:.6f}, Lon: {lon:.6f}, Alt: {self.current_alt:.1f}m")
            self.speed_label.config(text=f"H.Spd: {h_speed:.1f}, V.Spd: {v_speed:.1f} m/s")

            # Update Shared Telemetry State for Web Dashboard
            try:
                from web_server import broadcast_telemetry
                state = {
                    self.sysid: {
                        "lat": lat,
                        "lon": lon,
                        "alt": self.current_alt,
                        "pitch": self.pitch,
                        "roll": self.roll,
                        "yaw": self.yaw,
                        "h_speed": h_speed,
                        "v_speed": v_speed
                    }
                }
                broadcast_telemetry(state)
            except ImportError:
                pass

        if msg.get_type() == "HEARTBEAT" and msg.get_srcSystem() == self.sysid:
            self.last_heartbeat = time.time()
            conn_status = "Armed" if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else "Disarmed"
            self.status_label.config(text=f"Status: {conn_status}")

        if msg.get_type() == "ATTITUDE" and msg.get_srcSystem() == self.sysid:
            self.roll = math.degrees(msg.roll)
            self.pitch = math.degrees(msg.pitch)
            self.yaw = math.degrees(msg.yaw)

    # ---------------- Primary Flight Display ----------------
    def update_pfd(self):
        try:
            self.canvas.delete("all")

            w, h = 100, 100
            cx, cy = w//2, h//2

            # Horizon shift
            pitch_shift = self.pitch * 1.0
            roll_angle = math.radians(self.roll)

            # Sky (gradient blue) and ground (gradient brown)
            self.canvas.create_rectangle(0, 0, w, h//2, fill="#1a4d7a", outline="")
            self.canvas.create_rectangle(0, h//2, w, h, fill="#3d2817", outline="")

            # Horizon line
            line_len = 150
            x1 = cx - line_len*math.cos(roll_angle)
            y1 = cy - line_len*math.sin(roll_angle) + pitch_shift
            x2 = cx + line_len*math.cos(roll_angle)
            y2 = cy + line_len*math.sin(roll_angle) + pitch_shift
            self.canvas.create_line(x1, y1, x2, y2, fill=COLORS['text_primary'], width=2)

            # Aircraft symbol
            self.canvas.create_line(cx-15, cy, cx+15, cy, fill=COLORS['accent_cyan'], width=2)
            self.canvas.create_line(cx, cy-7, cx, cy+7, fill=COLORS['accent_cyan'], width=2)
            self.canvas.create_oval(cx-2, cy-2, cx+2, cy+2, fill=COLORS['accent_cyan'], outline="")

            # Heading text with background
            heading_text = f"{self.yaw:.0f}°"
            self.canvas.create_rectangle(cx-20, 6, cx+20, 18, fill=COLORS['bg_header'], outline=COLORS['border'])
            self.canvas.create_text(cx, 12, text=heading_text, fill=COLORS['text_primary'], font=("Consolas", 8, "bold"))

            # Pitch/Roll indicators
            self.canvas.create_text(8, h-8, text=f"P:{self.pitch:.0f}°", fill=COLORS['text_secondary'], font=("Consolas", 7), anchor="w")
            self.canvas.create_text(w-8, h-8, text=f"R:{self.roll:.0f}°", fill=COLORS['text_secondary'], font=("Consolas", 7), anchor="e")

            self.canvas.after(100, self.update_pfd)
        except Exception:
            pass

    # ------------------- Connection Logic -------------------

class VirtualConnection:
    """
    Mimics a mavlink_connection for a specific system ID, 
    routing traffic through a SharedConnection.
    """
    def __init__(self, shared_conn, sysid):
        self.shared_conn = shared_conn
        self.target_system = sysid
        self.target_component = 1
        self.mav = mavutil.mavlink.MAVLink(self, srcSystem=255, srcComponent=0)
        
    def recv_match(self, condition=None, type=None, blocking=False, timeout=None):
        # We delegate receiving to the shared connection's buffer for this sysid
        return self.shared_conn.get_message(self.target_system, type, blocking, timeout)
    
    def write(self, buf):
        # Forward raw writes based on the MAVLink protocol
        self.shared_conn.write(buf)

    def mav_send(self, msg):
        self.shared_conn.write(msg.pack(self.mav))

class SharedConnection:
    """
    Manages a single physical connection (e.g., COM port) and dispatches 
    messages to VirtualConnections based on their source system ID.
    """
    def __init__(self, connection_string, baud=57600):
        self.master = mavutil.mavlink_connection(connection_string, baud=baud)
        self.buffers = {} # {sysid: [messages]}
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        
        # Start a background thread to read from master
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
                        # Append to buffer (limit size to prevent leaks)
                        self.buffers[sysid].append(msg)
                        if len(self.buffers[sysid]) > 100:
                            self.buffers[sysid].pop(0)
            except Exception as e:
                # print(f"SharedConnection Read Error: {e}")
                pass

    def get_message(self, sysid, type_filter=None, blocking=False, timeout=None):
        start_time = time.time()
        while True:
            found_msg = None
            with self.lock:
                if sysid in self.buffers and self.buffers[sysid]:
                    # Check filter
                    if type_filter:
                        # Iterate to find matching type
                        for i, m in enumerate(self.buffers[sysid]):
                            if m.get_type() == type_filter:
                                found_msg = self.buffers[sysid].pop(i)
                                break
                    else:
                        found_msg = self.buffers[sysid].pop(0)
            
            if found_msg:
                return found_msg
            
            if not blocking:
                return None
            
            if timeout and (time.time() - start_time > timeout):
                return None
            
            time.sleep(0.01)

    def write(self, buf):
        self.master.write(buf)

    def close(self):
        self.stop_event.set()
        self.master.close()

# ------------------- Log Tee -------------------
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

# ------------------- Connection Scanning -------------------
def scan_for_drones_sim(max_drones=8, start_port=5762):
    import socket
    drone_slots = [None] * max_drones
    
    print(f"Scanning for up to {max_drones} simulated drones starting at port {start_port}...")
    
    for i in range(max_drones):
        port = start_port + (i * 10)
        address = f'tcp:127.0.0.1:{port}'
        print(f"Checking {address}...", end=" ", flush=True)
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(0.5)
        result = sock.connect_ex(('127.0.0.1', port))
        sock.close()

        if result != 0:
            print("x Port closed")
            continue

        try:
            conn = mavutil.mavlink_connection(address)
            msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            
            if msg:
                sysid = conn.target_system
                print(f"✅ Found Drone (System ID: {sysid})")
                drone_slots[i] = {'sysid': sysid, 'connection': conn}
            else:
                print(f"x No heartbeat")
                conn.close()
                
        except Exception as e:
            print(f"x Error: {e}")
            
    return drone_slots, [] # Return empty list for shared connections to close

def scan_for_drones_real(com_port, baud_rate):
    print(f"Connecting to Mesh Swarm on {com_port} @ {baud_rate}...")
    try:
        shared_conn = SharedConnection(com_port, baud=baud_rate)
        
        print("Listening for heartbeats (5 seconds)...")
        # Listen for a few seconds to identify active drones
        active_sysids = set()
        start = time.time()
        while time.time() - start < 5:
            # We peek into the buffers of the shared conn without Popping to just see who is there
            # Or simplified: The SharedConnection assumes we want to consume.
            # Let's just wait. The reader loop will populate buffers.
            with shared_conn.lock:
                for sysid in shared_conn.buffers.keys():
                    active_sysids.add(sysid)
            time.sleep(0.1)
            print(f"\rFound: {list(active_sysids)}", end="")
        
        print("\nDiscovery complete.")
        
        drone_slots = [None] * 8
        # Fill slots with found system IDs. 
        # We can map them 1-to-1 or just fill sequentially.
        # Let's try to map sysid 1 -> slot 0, sysid 2 -> slot 1, etc if possible.
        
        for sysid in active_sysids:
            idx = sysid - 1
            if 0 <= idx < 8:
                v_conn = VirtualConnection(shared_conn, sysid)
                drone_slots[idx] = {'sysid': sysid, 'connection': v_conn}
            else:
                print(f"Warning: Drone {sysid} out of slot range (1-8). Ignoring.")
                
        return drone_slots, [shared_conn]

    except Exception as e:
        print(f"Connection Failed: {e}")
        return [None]*8, []

# ------------------- Mode Selection Dialog -------------------
class ConnectionDialog:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Connection Settings")
        self.root.geometry("400x300")
        self.root.configure(bg=COLORS['bg_panel'])
        
        self.result = None # Will hold (mode, com_port, baud)
        
        self._setup_ui()
        
    def _setup_ui(self):
        style_font = ("Consolas", 10)
        
        tk.Label(self.root, text="Select Connection Mode", font=("Consolas", 12, "bold"), 
                 bg=COLORS['bg_panel'], fg=COLORS['text_primary']).pack(pady=20)
        
        # Mode variable
        self.mode_var = tk.StringVar(value="SIM")
        
        # Frames
        frame_sim = tk.Frame(self.root, bg=COLORS['bg_panel'])
        frame_real = tk.Frame(self.root, bg=COLORS['bg_panel'])
        
        def update_visibility():
            if self.mode_var.get() == "REAL":
                entry_state = "normal"
                bg_color = COLORS['bg_header']
            else:
                entry_state = "disabled"
                bg_color = COLORS['bg_dark']
                
            self.com_entry.config(state=entry_state, bg=bg_color)
            self.baud_entry.config(state=entry_state, bg=bg_color)

        tk.Radiobutton(self.root, text="Simulated Swarm (TCP Localhost)", variable=self.mode_var, 
                       value="SIM", command=update_visibility,
                       bg=COLORS['bg_panel'], fg=COLORS['text_primary'], selectcolor=COLORS['bg_dark'],
                       activebackground=COLORS['bg_panel'], activeforeground=COLORS['text_primary'], font=style_font).pack(anchor="w", padx=30)
        
        tk.Radiobutton(self.root, text="Real Swarm (Mesh/Serial)", variable=self.mode_var, 
                       value="REAL", command=update_visibility,
                       bg=COLORS['bg_panel'], fg=COLORS['text_primary'], selectcolor=COLORS['bg_dark'],
                       activebackground=COLORS['bg_panel'], activeforeground=COLORS['text_primary'], font=style_font).pack(anchor="w", padx=30)
        
        # Options Frame
        opts = tk.Frame(self.root, bg=COLORS['bg_panel'], padx=20, pady=10)
        opts.pack(fill="x", padx=30)
        
        tk.Label(opts, text="COM Port:", bg=COLORS['bg_panel'], fg=COLORS['text_secondary'], font=style_font).grid(row=0, column=0, sticky="w")
        self.com_entry = tk.Entry(opts, font=style_font, bg=COLORS['bg_dark'], fg=COLORS['text_primary'], disabledbackground=COLORS['bg_dark'])
        self.com_entry.insert(0, "COM14")
        self.com_entry.grid(row=0, column=1, padx=10, sticky="ew")
        
        tk.Label(opts, text="Baud Rate:", bg=COLORS['bg_panel'], fg=COLORS['text_secondary'], font=style_font).grid(row=1, column=0, sticky="w", pady=5)
        self.baud_entry = tk.Entry(opts, font=style_font, bg=COLORS['bg_dark'], fg=COLORS['text_primary'], disabledbackground=COLORS['bg_dark'])
        self.baud_entry.insert(0, "57600")
        self.baud_entry.grid(row=1, column=1, padx=10, sticky="ew", pady=5)
        
        update_visibility()
        
        # Connect Button
        tk.Button(self.root, text="CONNECT", command=self.on_connect,
                 bg=COLORS['accent_blue'], fg='white', font=("Consolas", 11, "bold"),
                 relief="flat", padx=20, pady=5).pack(pady=20)
                 
    def on_connect(self):
        mode = self.mode_var.get()
        com = self.com_entry.get()
        baud = int(self.baud_entry.get())
        self.result = (mode, com, baud)
        self.root.destroy()
        
    def run(self):
        self.root.mainloop()
        return self.result

# ------------------- Tkinter UI -------------------
def main():
    try:
        import web_server
        import webbrowser
        import sys
        
        # Start web server
        web_server.start_server()
        
        # Redirect stdout
        sys.stdout = LoggerTee(sys.stdout, web_server.broadcast_log)
    except Exception as e:
        print(f"Failed to start web server: {e}")

    dialog = ConnectionDialog()
    config = dialog.run()
    
    if not config:
        print("Cancelled")
        return

    mode, com, baud = config
    
    shared_connections = []
    
    if mode == "SIM":
        drones_data, _ = scan_for_drones_sim()
    else:
        drones_data, shared_connections = scan_for_drones_real(com, baud)
    
    root = tk.Tk()
    connected_count = sum(1 for d in drones_data if d is not None)
    root.title(f"Multi-Drone Ground Control Station - {mode} MODE")
    root.state('zoomed')
    root.configure(bg=COLORS['bg_dark'])

    # Title bar
    title_bar = tk.Frame(root, bg=COLORS['bg_header'], height=50)
    title_bar.pack(fill="x", padx=0, pady=0)
    
    title_label = tk.Label(
        title_bar,
        text="MULTI-DRONE GROUND CONTROL STATION",
        font=("Consolas", 16, "bold"),
        bg=COLORS['bg_header'],
        fg=COLORS['accent_cyan']
    )
    title_label.pack(side="left", padx=20, pady=10)
    
    # Buttons Frame in Title Bar
    btn_frame = tk.Frame(title_bar, bg=COLORS['bg_header'])
    btn_frame.pack(side="right", padx=10)
    
    # Launch Map Button
    def launch_map():
        import webbrowser
        webbrowser.open("http://localhost:8000")
        
    tk.Button(
        btn_frame,
        text="LAUNCH 3D MAP",
        command=launch_map,
        bg=COLORS['accent_blue'],
        fg='white',
        font=("Consolas", 10, "bold"),
        relief="flat",
        padx=10
    ).pack(side="left", padx=10)
    
    status_label = tk.Label(
        title_bar,
        text=f"● {connected_count} DRONES CONNECTED ({mode})",
        font=("Consolas", 12),
        bg=COLORS['bg_header'],
        fg=COLORS['status_good'] if connected_count > 0 else COLORS['status_critical']
    )
    status_label.pack(side="right", padx=20, pady=10)

    # Main content area with scrollable frame if needed
    content_frame = tk.Frame(root, bg=COLORS['bg_dark'])
    content_frame.pack(fill="both", expand=True, padx=5, pady=5)

    # Create 4x2 grid layout
    top_frame = tk.Frame(content_frame, bg=COLORS['bg_dark'])
    top_frame.pack(fill="both", expand=True)
    bottom_frame = tk.Frame(content_frame, bg=COLORS['bg_dark'])
    bottom_frame.pack(fill="both", expand=True)

    drone_handlers = []
    
    for i, data in enumerate(drones_data):
        if data:
            sysid = data['sysid']
            conn = data['connection']
        else:
            sysid = "N/A"
            conn = None
        
        container = top_frame if i < 4 else bottom_frame
        handler = DroneHandler(sysid, conn, container)
        drone_handlers.append(handler)
    
    # --- Swarm Logic ---
    def perform_swarm_command(cmd_data):
        cmd = cmd_data.get("cmd")
        print(f"Received Swarm Command: {cmd}")
        formation = cmd_data.get("formation", False)
        
        # Filter active drones
        active_handlers = [h for h in drone_handlers if h.sysid != "N/A"]
        if not active_handlers: return
        
        # Sort by SysID for consistent formation
        active_handlers.sort(key=lambda h: h.sysid)
        
        # Helper for common commands
        if cmd == "arm":
            for h in active_handlers: h.arm_drone()
        elif cmd == "disarm":
            for h in active_handlers: h.disarm_drone()
        elif cmd == "takeoff":
            alt = float(cmd_data.get("alt", 5))
            for h in active_handlers: h.takeoff(alt)
        elif cmd == "rtl":
            for h in active_handlers: h.rtl()
        elif cmd == "land":
            for h in active_handlers: h.land()
        elif cmd == "fly_to":
            try:
                lat = float(cmd_data.get("lat"))
                lon = float(cmd_data.get("lon"))
                alt = float(cmd_data.get("alt"))
                
                if formation and len(active_handlers) > 1:
                    # Leader (lowest ID) goes to target
                    leader = active_handlers[0]
                    leader.fly_to_gps(lat, lon, alt)
                    
                    # Followers
                    for i, h in enumerate(active_handlers[1:]):
                        # idx 0 is first follower
                        idx = i 
                        row = (idx // 2) + 1
                        
                        # Even index in followers array (Drone 2, 4...) -> Right (+), Odd -> Left (-)
                        # Note: idx=0 (Drone 2) -> Side Right. idx=1 (Drone 3) -> Side Left.
                        side = 1 if idx % 2 == 0 else -1
                        
                        # Offsets (approx in degrees)
                        # 1 deg lat ~ 111km -> 1m ~ 0.000009 deg
                        # 10m spacing
                        d_lat = -0.00010 * row # Behind
                        d_lon = 0.00015 * row * side # Side
                        
                        target_lat = lat + d_lat
                        target_lon = lon + d_lon
                        
                        h.fly_to_gps(target_lat, target_lon, alt)
                else:
                    # All to same point
                    for h in active_handlers:
                        h.fly_to_gps(lat, lon, alt)
            except Exception as e:
                print(f"Error in fly_to: {e}")

    # Register callback
    try:
        import web_server
        web_server.set_command_callback(perform_swarm_command)
    except:
        pass

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        print("Closing connections...")
        # Close virtual connections? They don't restrict closes but let's be safe
        for data in drones_data:
            if data and data['connection']:
                # If it's a TCP socket it needs closing
                # If it's a VirtualConnection, we just let it be, SharedConnection cleanup handles it
                if hasattr(data['connection'], 'close') and not isinstance(data['connection'], VirtualConnection):
                    data['connection'].close()
        
        # Close shared connections
        for sc in shared_connections:
            sc.close()

if __name__ == "__main__":
    main()
