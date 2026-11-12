import math
import threading
import time

from .config import HEARTBEAT_WARN_S
from .telemetry import BaseDroneHandler

# ======================== Tkinter GUI Drone Handler ========================
# Imported lazily so headless mode never touches tkinter.


def _build_gui_handler_class():
    """Returns the DroneHandler class (Tkinter). Called only when --control is used."""
    import tkinter as tk

    COLORS = {
        "bg_dark": "#1a1a1a",
        "bg_panel": "#2a2a2a",
        "bg_header": "#333333",
        "border": "#4a4a4a",
        "text_primary": "#ffffff",
        "text_secondary": "#b0b0b0",
        "accent_blue": "#2196f3",
        "accent_cyan": "#00d4ff",
        "status_good": "#4caf50",
        "status_warning": "#ffc107",
        "status_critical": "#f44336",
        "btn_arm": "#616161",
        "btn_takeoff": "#4caf50",
        "btn_flyto": "#ffc107",
        "btn_rtl": "#ff9800",
        "btn_land": "#2196f3",
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
                container,
                text=title,
                padx=4,
                pady=4,
                font=("Consolas", 9, "bold"),
                bg=COLORS["bg_panel"],
                fg=COLORS["text_primary"],
                bd=1,
                relief="flat",
                highlightbackground=COLORS["border"],
                highlightthickness=1,
            )
            self.frame.pack(side="left", padx=3, pady=3, fill="both", expand=True)

            header_frame = tk.Frame(self.frame, bg=COLORS["bg_panel"])
            header_frame.pack(fill="x", pady=(0, 3))

            self.status_light = tk.Canvas(
                header_frame,
                width=20,
                height=20,
                bg=COLORS["bg_panel"],
                highlightthickness=0,
            )
            self.status_light.pack(side="right", padx=3)

            display_frame = tk.Frame(self.frame, bg=COLORS["bg_panel"])
            display_frame.pack(fill="x", pady=3)

            self.canvas = tk.Canvas(
                display_frame,
                width=100,
                height=100,
                bg="#0a0a0a",
                highlightthickness=1,
                highlightbackground=COLORS["border"],
            )
            self.canvas.pack(side="left", padx=(0, 6))

            dashboard_frame = tk.Frame(display_frame, bg=COLORS["bg_panel"])
            dashboard_frame.pack(side="left", fill="both", expand=True)

            tf = ("Consolas", 8)
            self.gps_label = tk.Label(
                dashboard_frame,
                text="Lat: --, Lon: --, Alt: --",
                font=tf,
                bg=COLORS["bg_panel"],
                fg=COLORS["accent_cyan"],
                anchor="w",
            )
            self.gps_label.pack(fill="x", pady=1)

            self.speed_label = tk.Label(
                dashboard_frame,
                text="H.Spd: --, V.Spd: -- m/s",
                font=tf,
                bg=COLORS["bg_panel"],
                fg=COLORS["accent_cyan"],
                anchor="w",
            )
            self.speed_label.pack(fill="x", pady=1)

            self.status_label = tk.Label(
                dashboard_frame,
                text="Status: --",
                font=tf,
                bg=COLORS["bg_panel"],
                fg=COLORS["text_secondary"],
                anchor="w",
            )
            self.status_label.pack(fill="x", pady=1)

            separator = tk.Frame(self.frame, height=1, bg=COLORS["border"])
            separator.pack(fill="x", pady=3)

            inputs = tk.Frame(self.frame, bg=COLORS["bg_panel"])
            inputs.pack(fill="x", pady=2)

            lf = ("Consolas", 8, "bold")
            ef = ("Consolas", 8)
            for row_idx, label in enumerate(["LAT:", "LON:", "ALT:"]):
                tk.Label(
                    inputs,
                    text=label,
                    font=lf,
                    bg=COLORS["bg_panel"],
                    fg=COLORS["text_secondary"],
                ).grid(row=row_idx, column=0, sticky="e", padx=(0, 3), pady=1)
            self.lat_entry = tk.Entry(
                inputs,
                width=10,
                font=ef,
                bg=COLORS["bg_header"],
                fg=COLORS["text_primary"],
                insertbackground=COLORS["text_primary"],
                relief="flat",
                bd=1,
            )
            self.lat_entry.grid(row=0, column=1, padx=2, pady=1, sticky="ew")
            self.lon_entry = tk.Entry(
                inputs,
                width=10,
                font=ef,
                bg=COLORS["bg_header"],
                fg=COLORS["text_primary"],
                insertbackground=COLORS["text_primary"],
                relief="flat",
                bd=1,
            )
            self.lon_entry.grid(row=1, column=1, padx=2, pady=1, sticky="ew")
            self.alt_entry = tk.Entry(
                inputs,
                width=10,
                font=ef,
                bg=COLORS["bg_header"],
                fg=COLORS["text_primary"],
                insertbackground=COLORS["text_primary"],
                relief="flat",
                bd=1,
            )
            self.alt_entry.grid(row=2, column=1, padx=2, pady=1, sticky="ew")
            inputs.grid_columnconfigure(1, weight=1)

            btns = tk.Frame(self.frame, bg=COLORS["bg_panel"])
            btns.pack(fill="x", pady=(3, 0))

            bf = ("Consolas", 8, "bold")

            def _threaded(func):
                return lambda: (
                    threading.Thread(target=func, daemon=True).start()
                    if connection
                    else None
                )

            def _btn(parent, text, command, bg_color, row, col, colspan=1):
                b = tk.Button(
                    parent,
                    text=text,
                    command=command,
                    bg=bg_color,
                    fg="white",
                    font=bf,
                    relief="flat",
                    bd=0,
                    padx=4,
                    pady=3,
                    cursor="hand2" if connection else "arrow",
                )
                b.grid(
                    row=row, column=col, columnspan=colspan, padx=1, pady=1, sticky="ew"
                )
                if connection:
                    b.bind("<Enter>", lambda e: b.config(bg=self._lighten(bg_color)))
                    b.bind("<Leave>", lambda e: b.config(bg=bg_color))
                else:
                    b.config(bg=COLORS["bg_header"], cursor="arrow")
                return b

            _btn(btns, "ARM", _threaded(self.arm_drone), COLORS["btn_arm"], 0, 0)
            _btn(
                btns,
                "TAKEOFF",
                _threaded(lambda: self.takeoff(float(self.alt_entry.get() or 3))),
                COLORS["btn_takeoff"],
                0,
                1,
            )
            _btn(
                btns,
                "FLY TO",
                _threaded(
                    lambda: self.fly_to_gps(
                        float(self.lat_entry.get()),
                        float(self.lon_entry.get()),
                        float(self.alt_entry.get() or 3),
                    )
                ),
                COLORS["btn_flyto"],
                1,
                0,
            )
            _btn(btns, "RTL", _threaded(self.rtl), COLORS["btn_rtl"], 1, 1)
            _btn(btns, "LAND", _threaded(self.land), COLORS["btn_land"], 2, 0, 2)
            btns.grid_columnconfigure(0, weight=1)
            btns.grid_columnconfigure(1, weight=1)

            # Call parent __init__ AFTER widgets exist so _on_position can update them
            super().__init__(sysid, connection)
            self.set_status_light("red")
            self.update_pfd()
            if connection:
                threading.Thread(
                    target=self._check_connection_status, daemon=True
                ).start()
            else:
                self.status_label.config(text="Status: Disconnected")

        # ---- GUI helpers ----
        def _lighten(self, color):
            cmap = {
                "red": COLORS["status_critical"],
                "green": COLORS["status_good"],
                "yellow": COLORS["status_warning"],
            }
            color = cmap.get(color, color)
            h = color.lstrip("#")
            r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
            return (
                f"#{min(255, int(r * 1.2)):02x}"
                f"{min(255, int(g * 1.2)):02x}"
                f"{min(255, int(b * 1.2)):02x}"
            )

        def set_status_light(self, color):
            cmap = {
                "red": COLORS["status_critical"],
                "green": COLORS["status_good"],
                "yellow": COLORS["status_warning"],
            }
            color = cmap.get(color, color)
            self.status_light.delete("all")
            self.status_light.create_oval(
                2, 2, 18, 18, fill=color, outline=self._lighten(color), width=2
            )
            self.status_light.create_oval(
                5, 5, 15, 15, fill=self._lighten(color), outline=""
            )

        def _check_connection_status(self):
            while True:
                age = time.time() - self.last_heartbeat
                color = (
                    COLORS["status_critical"]
                    if age > HEARTBEAT_WARN_S
                    else COLORS["status_good"]
                )
                self.set_status_light(color)
                time.sleep(1)

        # ---- Override telemetry callbacks to also update GUI ----
        def _on_position(self, msg):
            super()._on_position(msg)
            h_speed = math.sqrt(msg.vx**2 + msg.vy**2) / 100.0
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

                self.canvas.create_rectangle(
                    0, 0, w, h // 2, fill="#1a4d7a", outline=""
                )
                self.canvas.create_rectangle(
                    0, h // 2, w, h, fill="#3d2817", outline=""
                )

                line_len = 150
                x1 = cx - line_len * math.cos(roll_angle)
                y1 = cy - line_len * math.sin(roll_angle) + pitch_shift
                x2 = cx + line_len * math.cos(roll_angle)
                y2 = cy + line_len * math.sin(roll_angle) + pitch_shift
                self.canvas.create_line(
                    x1, y1, x2, y2, fill=COLORS["text_primary"], width=2
                )

                self.canvas.create_line(
                    cx - 15, cy, cx + 15, cy, fill=COLORS["accent_cyan"], width=2
                )
                self.canvas.create_line(
                    cx, cy - 7, cx, cy + 7, fill=COLORS["accent_cyan"], width=2
                )
                self.canvas.create_oval(
                    cx - 2,
                    cy - 2,
                    cx + 2,
                    cy + 2,
                    fill=COLORS["accent_cyan"],
                    outline="",
                )

                self.canvas.create_rectangle(
                    cx - 20,
                    6,
                    cx + 20,
                    18,
                    fill=COLORS["bg_header"],
                    outline=COLORS["border"],
                )
                self.canvas.create_text(
                    cx,
                    12,
                    text=f"{self.yaw:.0f}°",
                    fill=COLORS["text_primary"],
                    font=("Consolas", 8, "bold"),
                )
                self.canvas.create_text(
                    8,
                    h - 8,
                    text=f"P:{self.pitch:.0f}°",
                    fill=COLORS["text_secondary"],
                    font=("Consolas", 7),
                    anchor="w",
                )
                self.canvas.create_text(
                    w - 8,
                    h - 8,
                    text=f"R:{self.roll:.0f}°",
                    fill=COLORS["text_secondary"],
                    font=("Consolas", 7),
                    anchor="e",
                )
                self.canvas.after(100, self.update_pfd)
            except Exception:
                pass

    return DroneHandler, COLORS
