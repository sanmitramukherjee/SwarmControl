# ======================== Configuration Constants ========================
DATA_STREAM_RATE_HZ      = 4     # MAVLink data stream request rate (Hz)
HEARTBEAT_TIMEOUT_S      = 10.0  # Seconds without heartbeat before watchdog reconnects
HEARTBEAT_WARN_S         = 3.0   # Seconds without heartbeat before GUI status turns red
HEARTBEAT_WAIT_REAL_S    = 2.0   # Seconds to wait for heartbeats after serial connect
METERS_PER_DEG_LAT       = 110574.0   # Approximate metres per degree of latitude
METERS_PER_DEG_LON_EQ    = 111320.0   # Approximate metres per degree of longitude at equator
WEB_DASH_URL             = "http://localhost:8000/index.html"

# ---- Collision Avoidance ----
# Minimum 3-D separation (metres) below which a collision warning is issued and
# vertical stagger is applied automatically.
COLLISION_WARN_M     = 1.5
# Vertical altitude increment (metres) added per drone index when flying to the
# same lat/lon without a formation pattern, preventing mid-air convergence.
COLLISION_ALT_STEP_M = 2.5
# Seconds between each consecutive drone takeoff when staggering launch.
TAKEOFF_STAGGER_S    = 1.5
# Inactivity timeout (seconds) before hovering drones RTL
INACTIVITY_RTL_TIMEOUT_S = 30.0
# Minimum interval (seconds) between sequential command dispatches on a shared serial bus
SERIAL_DISPATCH_SPACING_S = 0.25
