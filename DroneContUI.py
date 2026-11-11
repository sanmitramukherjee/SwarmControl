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

from swarmcontrol.config import (
    DATA_STREAM_RATE_HZ,
    HEARTBEAT_TIMEOUT_S,
    HEARTBEAT_WARN_S,
    HEARTBEAT_WAIT_REAL_S,
    METERS_PER_DEG_LAT,
    METERS_PER_DEG_LON_EQ,
    WEB_DASH_URL,
    COLLISION_WARN_M,
    COLLISION_ALT_STEP_M,
    TAKEOFF_STAGGER_S,
    INACTIVITY_RTL_TIMEOUT_S,
    SERIAL_DISPATCH_SPACING_S,
)
from swarmcontrol.connections import (
    VirtualConnection,
    SharedConnection,
    connect_drones,
    start_watchdog,
)
from swarmcontrol.formation import (
    _meters_to_deg,
    _haversine_m,
    check_formation_separation,
    calculate_formation_offsets,
)
from swarmcontrol.gui import _build_gui_handler_class
from swarmcontrol.telemetry import BaseDroneHandler


from swarmcontrol import swarm as _swarm_state
from swarmcontrol.cli import LoggerTee, _open_dashboard, main, parse_args
from swarmcontrol.swarm import (
    _autonomous_mission_worker,
    _inactivity_watchdog_loop,
    _start_inactivity_watchdog,
    get_leader,
    notify_user_activity,
    perform_swarm_command,
    set_leader,
)


def __getattr__(name):
    return getattr(_swarm_state, name)


if __name__ == "__main__":
    main()
