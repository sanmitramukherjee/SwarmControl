import threading
import time

from pymavlink import mavutil

from .config import (
    COLLISION_ALT_STEP_M,
    COLLISION_WARN_M,
    INACTIVITY_RTL_TIMEOUT_S,
    SERIAL_DISPATCH_SPACING_S,
    TAKEOFF_STAGGER_S,
)
from .formation import (
    _haversine_m,
    calculate_formation_offsets,
    check_formation_separation,
)

# ======================== Swarm Logic ========================
# Module-level state for swarm
_leader_sysid = 1  # Hardcoded default leader is Drone 1
_leader_lock = threading.Lock()

# Mission state & Inactivity watchdog
_mission_in_progress = threading.Event()
_last_activity_time = time.time()
_watchdog_thread_started = False
_active_handlers_ref = []


def notify_user_activity():
    """Reset the 30-second inactivity RTL timer."""
    global _last_activity_time
    _last_activity_time = time.time()


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
    """Return the leader handler (default SysID 1) from active_handlers."""
    with _leader_lock:
        lsid = _leader_sysid
    for h in active_handlers:
        if h.sysid == lsid:
            return h
    # If sysid 1 not present, fallback to first active drone
    return active_handlers[0] if active_handlers else None


def _inactivity_watchdog_loop():
    """Background watchdog: if drones are airborne/hovering and inactive for 30s, RTL.
    DOES NOT fire when a drone is actively climbing or navigating to a target.
    """
    global _last_activity_time
    while True:
        time.sleep(1.0)
        # Check if mission is currently in progress (flying/climbing)
        if _mission_in_progress.is_set():
            _last_activity_time = time.time()
            continue

        active = [h for h in _active_handlers_ref if h.sysid != "N/A" and h.connection]
        # Only trigger if at least one drone is armed and airborne (alt > 1.0m)
        airborne_drones = [h for h in active if h.armed and h.current_alt > 1.0]
        if not airborne_drones:
            _last_activity_time = time.time()
            continue

        idle_seconds = time.time() - _last_activity_time
        if idle_seconds >= INACTIVITY_RTL_TIMEOUT_S:
            print(
                f"[Watchdog] Inactivity timeout ({idle_seconds:.1f}s >= {INACTIVITY_RTL_TIMEOUT_S}s). "
                f"Hovering drones initiating automatic RTL."
            )
            try:
                from web_server import broadcast_log

                broadcast_log(
                    f"[AUTO-RTL] Inactivity timeout ({INACTIVITY_RTL_TIMEOUT_S:.0f}s with no commands). "
                    f"Swarm returning to launch."
                )
            except Exception:
                pass

            for h in airborne_drones:
                threading.Thread(target=h.rtl, daemon=True).start()

            # Reset timer so it doesn't repeatedly spam RTL
            _last_activity_time = time.time()


def _start_inactivity_watchdog():
    global _watchdog_thread_started
    if not _watchdog_thread_started:
        _watchdog_thread_started = True
        threading.Thread(target=_inactivity_watchdog_loop, daemon=True).start()


def _autonomous_mission_worker(cmd_data, active_handlers):
    """
    Autonomous one-click mission:
    1. If not armed/airborne, sets GUIDED, arms, and commands takeoff to target altitude.
    2. Waits for all drones to reach hover altitude (alt >= target_alt - 1.0m).
    3. Calculates formation offsets relative to Drone 1 (Leader).
    4. Commands drones to their respective target positions.
    5. Monitors navigation progress until drones arrive near targets.
    6. Returns swarm to hovering state; arms the 30-second inactivity watchdog.
    """
    global _last_activity_time
    _mission_in_progress.set()
    try:
        lat = float(cmd_data.get("lat"))
        lon = float(cmd_data.get("lon"))
        target_alt = float(cmd_data.get("alt", 5.0))
        formation = cmd_data.get("formation", False)
        pattern = cmd_data.get("pattern", "V")
        spacing_m = float(cmd_data.get("spacing_m", 10.0))

        leader = get_leader(active_handlers)
        if leader is None:
            return

        print(
            f"[Mission] Starting autonomous mission to ({lat:.6f}, {lon:.6f}, {target_alt:.1f}m)..."
        )
        try:
            from web_server import broadcast_log

            broadcast_log(
                f"[Mission] Initiating autonomous swarm mission to {lat:.6f}, {lon:.6f} at {target_alt}m."
            )
        except Exception:
            pass

        # Step 1: Check which drones need takeoff
        takeoff_needed = [
            h for h in active_handlers if not h.armed or h.current_alt < 1.0
        ]
        if takeoff_needed:
            print(
                f"[Mission] {len(takeoff_needed)} drone(s) require launch. Executing staggered takeoff..."
            )
            for i, h in enumerate(takeoff_needed):
                if i > 0:
                    time.sleep(TAKEOFF_STAGGER_S)
                threading.Thread(
                    target=h.takeoff, args=(target_alt,), daemon=True
                ).start()

            # Step 2: Wait until all drones reach target altitude (within 1.0m tolerance)
            climb_start = time.time()
            climb_timeout_s = 60.0
            while time.time() - climb_start < climb_timeout_s:
                all_at_alt = True
                for h in active_handlers:
                    if h.current_alt < (target_alt - 1.0):
                        all_at_alt = False
                        break
                if all_at_alt:
                    print("[Mission] All drones reached target altitude.")
                    break
                time.sleep(0.5)

        # Step 3: Compute targets & formations
        followers = [h for h in active_handlers if h is not leader]
        if formation and followers:
            heading = leader.yaw
            offsets = calculate_formation_offsets(
                lat,
                lon,
                heading,
                follower_count=len(followers),
                pattern=pattern,
                spacing_m=spacing_m,
            )
            targets = [(lat, lon, target_alt)] + [
                (f_lat, f_lon, target_alt) for f_lat, f_lon in offsets
            ]
            conflicts = check_formation_separation(targets, min_sep_m=COLLISION_WARN_M)
            if conflicts:
                print(
                    f"[Swarm] Collision warning – {len(conflicts)} pair(s) within {COLLISION_WARN_M}m. Staggering altitudes."
                )
                targets = [
                    (t_lat, t_lon, t_alt + i * COLLISION_ALT_STEP_M)
                    for i, (t_lat, t_lon, t_alt) in enumerate(targets)
                ]
            drone_target_map = {leader: targets[0]}
            for h, tgt in zip(followers, targets[1:]):
                drone_target_map[h] = tgt
        else:
            drone_target_map = {}
            for i, h in enumerate(active_handlers):
                stagger_alt = target_alt + (
                    i * COLLISION_ALT_STEP_M if len(active_handlers) > 1 else 0
                )
                drone_target_map[h] = (lat, lon, stagger_alt)

        # Step 4: Dispatch fly_to commands with staggered bus spacing
        for i, (h, (t_lat, t_lon, t_alt)) in enumerate(drone_target_map.items()):
            if i > 0:
                time.sleep(SERIAL_DISPATCH_SPACING_S)
            threading.Thread(
                target=h.fly_to_gps, args=(t_lat, t_lon, t_alt), daemon=True
            ).start()

        # Step 5: Monitor horizontal progress until arrived (within 2.5m)
        nav_start = time.time()
        nav_timeout_s = 180.0
        while time.time() - nav_start < nav_timeout_s:
            all_arrived = True
            for h, (t_lat, t_lon, _) in drone_target_map.items():
                if abs(h.lat) > 0.001 and abs(h.lon) > 0.001:
                    dist = _haversine_m(h.lat, h.lon, t_lat, t_lon)
                    if dist > 2.5:
                        all_arrived = False
                        break
            if all_arrived:
                print(
                    "[Mission] All drones arrived at waypoint formation. Now hovering."
                )
                try:
                    from web_server import broadcast_log

                    broadcast_log(
                        "[Mission] Swarm arrived at target formation. Hovering for new commands (Auto-RTL in 30s if idle)."
                    )
                except Exception:
                    pass
                break
            time.sleep(1.0)

    except Exception as e:
        print(f"[Mission] Error in autonomous mission worker: {e}")
    finally:
        # Step 6: Mark mission complete; start the 30-second idle countdown from now
        _mission_in_progress.clear()
        _last_activity_time = time.time()


def perform_swarm_command(cmd_data, drone_handlers):
    global _active_handlers_ref
    notify_user_activity()
    _start_inactivity_watchdog()

    cmd = cmd_data.get("cmd")
    print(f"[Swarm] Command: {cmd}")

    if cmd_data.get("type") == "set_leader":
        set_leader(cmd_data.get("sysid", 1))
        return

    active_handlers = [h for h in drone_handlers if h.sysid != "N/A" and h.connection]
    if not active_handlers:
        return

    active_handlers.sort(key=lambda h: h.sysid)
    _active_handlers_ref = active_handlers

    if cmd == "arm":

        def _staggered_arm():
            for i, h in enumerate(active_handlers):
                if i > 0:
                    time.sleep(SERIAL_DISPATCH_SPACING_S)
                threading.Thread(target=h.arm_drone, daemon=True).start()

        threading.Thread(target=_staggered_arm, daemon=True).start()

    elif cmd == "disarm":
        _mission_in_progress.clear()

        def _staggered_disarm():
            for i, h in enumerate(active_handlers):
                if i > 0:
                    time.sleep(SERIAL_DISPATCH_SPACING_S)
                threading.Thread(target=h.disarm_drone, daemon=True).start()

        threading.Thread(target=_staggered_disarm, daemon=True).start()

    elif cmd == "takeoff":
        alt = float(cmd_data.get("alt", 5))

        def _staggered_takeoff():
            for i, h in enumerate(active_handlers):
                if i > 0:
                    time.sleep(TAKEOFF_STAGGER_S)
                threading.Thread(target=h.takeoff, args=(alt,), daemon=True).start()

        threading.Thread(target=_staggered_takeoff, daemon=True).start()

    elif cmd == "rtl":
        _mission_in_progress.clear()
        print(
            f"[Swarm] Executing RTL with vertical stagger ({COLLISION_ALT_STEP_M}m step)..."
        )

        def _staggered_rtl():
            for i, h in enumerate(active_handlers):
                if i > 0:
                    time.sleep(SERIAL_DISPATCH_SPACING_S)
                # Stagger RTL return altitude to prevent mid-air collision during transit/descent
                stagger_rtl_alt = 10.0 + (i * COLLISION_ALT_STEP_M)
                try:
                    # Set RTL_ALT parameter per drone (in cm: 10m = 1000cm, 12.5m = 1250cm, etc.)
                    h.connection.mav.param_set_send(
                        h.sysid,
                        h.connection.target_component,
                        b"RTL_ALT\x00\x00\x00\x00\x00\x00\x00\x00",
                        float(stagger_rtl_alt * 100),
                        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                    )
                except Exception:
                    pass
                threading.Thread(target=h.rtl, daemon=True).start()

        threading.Thread(target=_staggered_rtl, daemon=True).start()

    elif cmd == "land":
        _mission_in_progress.clear()

        def _staggered_land():
            for i, h in enumerate(active_handlers):
                if i > 0:
                    time.sleep(SERIAL_DISPATCH_SPACING_S)
                threading.Thread(target=h.land, daemon=True).start()

        threading.Thread(target=_staggered_land, daemon=True).start()

    elif cmd == "fly_to":
        # One-click autonomous mission: auto arm/takeoff if on ground, fly in formation, hover & watchdog
        threading.Thread(
            target=_autonomous_mission_worker,
            args=(cmd_data, active_handlers),
            daemon=True,
        ).start()
