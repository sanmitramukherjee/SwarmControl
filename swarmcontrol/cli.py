import argparse
import sys
import threading
import time

from .config import WEB_DASH_URL
from .connections import VirtualConnection, connect_drones, start_watchdog
from .gui import _build_gui_handler_class
from .swarm import perform_swarm_command
from .telemetry import BaseDroneHandler


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


def _open_dashboard():
    """Open the web dashboard with a cache-busting query string."""
    import webbrowser
    url = f"{WEB_DASH_URL}?v={int(time.time())}"
    webbrowser.open(url)


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
            "leader": 1,
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
            _open_dashboard()

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
            _open_dashboard()
        except Exception:
            pass

    if args.control:
        try:
            root.mainloop()
        except KeyboardInterrupt:
            pass
    else:
        print(f"[SwarmControl] Running in headless mode. Web dashboard: {WEB_DASH_URL}")
        print("Press Ctrl+C to quit.")
        _shutdown = threading.Event()
        try:
            _shutdown.wait()  # blocks efficiently until interrupted
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
