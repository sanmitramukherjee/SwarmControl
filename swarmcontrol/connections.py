import threading
import time

from pymavlink import mavutil

from .config import HEARTBEAT_TIMEOUT_S, HEARTBEAT_WAIT_REAL_S


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
        self.buffer_lock = threading.Lock()
        self.write_lock = threading.Lock()

        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

    def _reader_loop(self):
        while not self.stop_event.is_set():
            try:
                msg = self.master.recv_match(blocking=True, timeout=1.0)
                if msg:
                    sysid = msg.get_srcSystem()
                    with self.buffer_lock:
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
            with self.buffer_lock:
                if self.buffers.get(sysid):
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
        with self.write_lock:
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
                    drone_list.append(
                        {"sysid": sysid, "connection": conn, "port": port}
                    )
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
                print(
                    f"  Invalid real drone spec '{spec}' (expected COM:SYSID). Skipping."
                )
                continue
            com, _, sysid_str = spec.rpartition(":")
            try:
                sysid = int(sysid_str)
            except ValueError:
                print(f"  Invalid sysid '{sysid_str}' in '{spec}'. Skipping.")
                continue
            port_map.setdefault(com, []).append(sysid)

        for com_port, sysids in port_map.items():
            print(
                f"Connecting to real drones on {com_port} @ {baud}bps, sysids={sysids}..."
            )
            try:
                shared_conn = SharedConnection(com_port, baud=baud)
                shared_connections.append(shared_conn)
                time.sleep(HEARTBEAT_WAIT_REAL_S)  # Allow heartbeats to arrive
                for sysid in sysids:
                    v_conn = VirtualConnection(shared_conn, sysid)
                    drone_list.append(
                        {"sysid": sysid, "connection": v_conn, "port": com_port}
                    )
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
                    broadcast_status(
                        {
                            "drone_status": {
                                "sysid": sysid,
                                "connected": age <= HEARTBEAT_TIMEOUT_S,
                            }
                        }
                    )

    threading.Thread(target=_loop, daemon=True).start()
