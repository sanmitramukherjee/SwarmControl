import math
import threading
import time

from pymavlink import mavutil

from commands import arm_drone, disarm_drone, fly_to_gps, land, rtl, takeoff

from .config import DATA_STREAM_RATE_HZ


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
        self.current_alt = 0.0  # relative to home (AGL)
        self.alt_msl = 0.0  # MSL altitude
        self.home_alt = None  # MSL alt at first valid fix

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

        h_speed = math.sqrt(msg.vx**2 + msg.vy**2) / 100.0
        v_speed = -(msg.vz / 100.0)

        try:
            from web_server import broadcast_telemetry

            broadcast_telemetry(
                {
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
                }
            )
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

    def fly_to_gps(self, lat, lon, alt, set_guided=True):
        if self.connection:
            fly_to_gps(self.connection, self.sysid, lat, lon, alt, set_guided=set_guided)

    def rtl(self):
        if self.connection:
            rtl(self.connection, self.sysid)

    def land(self):
        if self.connection:
            land(self.connection, self.sysid)
