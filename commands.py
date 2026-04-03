import time
from pymavlink import mavutil

# ---------------- MAVLink Commands ----------------
def arm_drone(connection, systemID):
    print(f"[Drone {systemID}] Arming...")
    connection.mav.command_long_send(systemID, 1,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)

def disarm_drone(connection, systemID):
    print(f"[Drone {systemID}] Disarming...")
    connection.mav.command_long_send(systemID, 1,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)

def takeoff(connection, systemID, altitude):
    print(f"[Drone {systemID}] Switching to GUIDED mode...")
    connection.mav.set_mode_send(
        systemID,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4  # 4 = GUIDED for ArduCopter
    )
    arm_drone(connection, systemID)
    time.sleep(1)

    print(f"[Drone {systemID}] Takeoff to {altitude}m")
    connection.mav.command_long_send(systemID, 1,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)

def fly_to_gps(connection, systemID, lat, lon, alt, use_msl=False):
    """Fly to GPS coordinates.

    Args:
        use_msl: When True, altitude is treated as MSL (MAV_FRAME_GLOBAL_INT).
                 When False (default), altitude is relative to home
                 (MAV_FRAME_GLOBAL_RELATIVE_ALT_INT).
    """
    frame = (
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT
        if use_msl
        else mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    )
    print(f"[Drone {systemID}] Switching to GUIDED for Mission...")
    connection.mav.set_mode_send(
        systemID,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4  # 4 = GUIDED for ArduCopter
    )
    frame_label = "MSL" if use_msl else "REL"
    print(f"[Drone {systemID}] Flying to {lat}, {lon}, {alt}m ({frame_label})...")
    connection.mav.set_position_target_global_int_send(
        0, systemID, 1,
        frame,
        0b110111111000,
        int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0, 0, 0, 0, 0, 0
    )

def rtl(connection, systemID):
    print(f"[Drone {systemID}] RTL...")
    connection.mav.command_long_send(systemID, 1,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)

def land(connection, systemID):
    print(f"[Drone {systemID}] Landing...")
    connection.mav.command_long_send(systemID, 1,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)