"""Swarm command sequencing and conservative waypoint transition checks."""

import math
import threading
import time

from .config import (
    COLLISION_WARN_M,
    INACTIVITY_RTL_TIMEOUT_S,
    LAND_TIMEOUT_S,
    MIN_FORMATION_SPACING_M,
    NAV_TIMEOUT_S,
    SERIAL_DISPATCH_SPACING_S,
    TAKEOFF_STAGGER_S,
    TAKEOFF_TIMEOUT_S,
)
from .formation import _haversine_m, calculate_formation_offsets, segment_clear_of_point

_leader_sysid = 1
_leader_lock = threading.Lock()
_worker_lock = threading.Lock()
_dispatch_lock = threading.Lock()
_current_cancel = None
_current_worker = None
_current_kind = None
_mission_in_progress = threading.Event()
_last_activity_time = time.time()
_watchdog_thread_started = False
_active_handlers_ref = []


def _log(message):
    print(message)
    try:
        from web_server import broadcast_log
        broadcast_log(message)
    except Exception:
        pass


def _manual_status(active):
    try:
        from web_server import broadcast_status
        broadcast_status({"manual_mode": active})
    except Exception:
        pass


def notify_user_activity():
    global _last_activity_time
    _last_activity_time = time.time()


def set_leader(sysid):
    global _leader_sysid
    sysid = int(sysid)
    with _leader_lock:
        old_sysid = _leader_sysid
        changed = _leader_sysid != sysid
        _leader_sysid = sysid
    if changed:
        with _worker_lock:
            if _current_cancel is not None:
                _current_cancel.set()
                _mission_in_progress.clear()
                to_hold = ([h for h in _active_handlers_ref if h.sysid != old_sysid]
                           if _current_kind == "manual_mode" else _active_handlers_ref)
                _hold(to_hold, threading.Event())
    _log(f"[Swarm] Leader set to drone {sysid}")
    try:
        from web_server import broadcast_status
        broadcast_status({"leader": sysid})
    except (ImportError, RuntimeError):
        pass


def get_leader(active_handlers):
    with _leader_lock:
        chosen = _leader_sysid
    return next((h for h in active_handlers if h.sysid == chosen),
                active_handlers[0] if active_handlers else None)


def _ordered(active_handlers):
    leader = get_leader(active_handlers)
    return [leader] + sorted((h for h in active_handlers if h is not leader),
                             key=lambda h: h.sysid) if leader else []


def _number(value, name, minimum=None):
    result = float(value)
    if not math.isfinite(result) or (minimum is not None and result < minimum):
        raise ValueError(f"{name} must be finite and at least {minimum}")
    return result


def _position(h):
    lat, lon, alt = float(h.lat), float(h.lon), float(h.current_alt)
    if not all(map(math.isfinite, (lat, lon, alt))) or not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise ValueError(f"Drone {h.sysid} has invalid position telemetry")
    if lat == 0.0 and lon == 0.0:
        raise ValueError(f"Drone {h.sysid} has no valid GPS position")
    return lat, lon, alt


def _landed(h):
    return not h.armed and math.isfinite(h.current_alt) and h.current_alt <= 0.5


def _wait_until(predicate, timeout, cancel):
    deadline = time.monotonic() + timeout
    while not cancel.is_set():
        if predicate():
            return True
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        cancel.wait(min(0.2, remaining))
    return False


def _call(cancel, h, method, *args, **kwargs):
    # A new land/RTL/disarm acquires this lock after cancellation, preventing a
    # cancelled worker from issuing a later navigation command.
    with _dispatch_lock:
        if cancel.is_set():
            return False
        getattr(h, method)(*args, **kwargs)
        return True


def _hold(handlers, cancel):
    """Replace outstanding GUIDED waypoints with current-position hover targets."""
    for h in handlers:
        if cancel.is_set():
            return
        if not h.armed or h.current_alt <= 0.5:
            continue
        try:
            position = _position(h)
            _call(cancel, h, "fly_to_gps", *position, set_guided=False)
        except ValueError as exc:
            _log(f"[Swarm] Could not issue hold to drone {h.sysid}: {exc}")


def _start_worker(kind, work):
    global _current_cancel, _current_worker, _current_kind
    with _worker_lock:
        replacing_manual = _current_kind == "manual_mode" and kind != "manual_mode"
        if _current_cancel is not None:
            _current_cancel.set()
        if replacing_manual:
            _manual_status(False)
        cancel = threading.Event()
        _current_cancel = cancel
        _current_kind = kind
        _mission_in_progress.set()

        def run():
            global _current_cancel, _current_worker, _current_kind
            try:
                work(cancel)
            except (ValueError, TypeError) as exc:
                _log(f"[Swarm] {kind} aborted: {exc}")
            except Exception as exc:
                _log(f"[Swarm] {kind} failed: {exc}")
            finally:
                with _worker_lock:
                    if _current_cancel is cancel:
                        if kind == "manual_mode":
                            _manual_status(False)
                        _current_cancel = None
                        _current_worker = None
                        _current_kind = None
                        _mission_in_progress.clear()
                        notify_user_activity()

        _current_worker = threading.Thread(target=run, daemon=True)
        _current_worker.start()


def _cancel_worker():
    with _worker_lock:
        if _current_cancel is not None:
            _current_cancel.set()
        _mission_in_progress.clear()


def stop_manual_mode():
    with _worker_lock:
        if _current_kind != "manual_mode" or _current_cancel is None:
            return False
        _current_cancel.set()
        _mission_in_progress.clear()
        _hold(_ordered(_active_handlers_ref)[1:], threading.Event())
        return True


def _takeoff_ordered(handlers, altitude, cancel):
    # The selected leader reaches an airborne hover before any follower launch.
    leader = handlers[0]
    for i, h in enumerate(handlers):
        if cancel.is_set():
            return False
        if h.armed and h.current_alt >= max(1.0, altitude - 1.0):
            continue
        if i and cancel.wait(TAKEOFF_STAGGER_S):
            return False
        if not _call(cancel, h, "takeoff", altitude):
            return False
        if not _wait_until(lambda: h.armed and h.current_alt >= max(1.0, altitude - 1.0),
                           TAKEOFF_TIMEOUT_S, cancel):
            if cancel.is_set():
                return False
            _log(f"[Swarm] Takeoff aborted: drone {h.sysid} did not reach altitude in time; no further drones launched.")
            return False
        if i == 0:
            _log(f"[Swarm] Leader {leader.sysid} airborne; launching followers in order.")
    return True


def _land_ordered(handlers, method, cancel):
    # Leader must be observed near home and disarmed before follower dispatch.
    for i, h in enumerate(handlers):
        if cancel.is_set():
            return
        if not _landed(h):
            if i and cancel.wait(SERIAL_DISPATCH_SPACING_S):
                return
            if not _call(cancel, h, method):
                return
            if i == 0:
                # The old mission may have left follower waypoints in the
                # autopilot. Hold them before waiting for leader touchdown.
                _hold(handlers[1:], cancel)
            if not _wait_until(lambda: _landed(h), LAND_TIMEOUT_S, cancel):
                if cancel.is_set():
                    return
                _log(f"[Swarm] {method.upper()} aborted: drone {h.sysid} was not observed grounded and disarmed; later drones remain on hold.")
                return
        _log(f"[Swarm] Drone {h.sysid} grounded and disarmed.")


def _target_map(cmd_data, handlers, leader, lat, lon, alt):
    followers = [h for h in handlers if h is not leader]
    spacing = _number(cmd_data.get("spacing_m", 10.0), "spacing_m", MIN_FORMATION_SPACING_M)
    result = {leader: (lat, lon, alt)}
    if cmd_data.get("formation", False):
        heading = _number(leader.yaw, "leader yaw")
        offsets = calculate_formation_offsets(lat, lon, heading, len(followers),
                                              pattern=cmd_data.get("pattern", "V"), spacing_m=spacing)
        if len(offsets) != len(followers):
            raise ValueError("unknown formation pattern")
        result.update((h, (p[0], p[1], alt)) for h, p in zip(followers, offsets))
    else:
        # Preserve existing horizontal offsets from the leader when formation is off.
        # A common waypoint would cause convergence even with altitude staggering.
        old_leader = _position(leader)
        for h in followers:
            old = _position(h)
            result[h] = (lat + old[0] - old_leader[0], lon + old[1] - old_leader[1], alt)
    for h, p in result.items():
        if not (-90 <= p[0] <= 90 and -180 <= p[1] <= 180):
            raise ValueError(f"target for drone {h.sysid} is outside valid coordinates")
    return result


def _safe_sequential_paths(handlers, targets):
    """Check each straight horizontal leg against every other drone's hold point.

    Completed drones hold at target; later drones hold at their current position.
    Real flight dynamics, GPS error, and a moving RC leader are not modeled.
    """
    holds = {h: _position(h) for h in handlers}
    for moving in handlers:
        start = holds[moving]
        end = targets[moving]
        # Horizontal separation is used because relative AGL may have different
        # home elevations and cannot safely be compared as a common vertical frame.
        flat_start = (start[0], start[1], 0.0)
        flat_end = (end[0], end[1], 0.0)
        for other in handlers:
            if other is moving:
                continue
            hold = holds[other]
            if not segment_clear_of_point(flat_start, flat_end,
                                          (hold[0], hold[1], 0.0), COLLISION_WARN_M + 0.5):
                raise ValueError(f"unsafe path for drone {moving.sysid} near drone {other.sysid}; navigation held")
        holds[moving] = end


def _autonomous_mission_worker(cmd_data, handlers, cancel):
    lat = _number(cmd_data["lat"], "latitude")
    lon = _number(cmd_data["lon"], "longitude")
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise ValueError("target coordinates out of range")
    alt = _number(cmd_data.get("alt", 5.0), "altitude", 1.0)
    leader = handlers[0]
    targets = _target_map(cmd_data, handlers, leader, lat, lon, alt)
    try:
        _safe_sequential_paths(handlers, targets)
    except ValueError:
        _hold(handlers, cancel)
        raise
    if not _takeoff_ordered(handlers, alt, cancel):
        return
    # Positions may drift during climb; recheck immediately before navigation.
    try:
        _safe_sequential_paths(handlers, targets)
    except ValueError:
        _hold(handlers, cancel)
        raise
    for h in handlers:
        if cancel.is_set():
            return
        target = targets[h]
        try:
            # A completed craft may be up to 0.5m from its planned hold point.
            # Recheck against live positions before each new leg.
            live_targets = {other: _position(other) for other in handlers}
            live_targets[h] = target
            _safe_sequential_paths(handlers, live_targets)
        except ValueError:
            _hold(handlers, cancel)
            raise
        if not _call(cancel, h, "fly_to_gps", *target):
            return
        if not _wait_until(lambda: _haversine_m(h.lat, h.lon, target[0], target[1]) <= 0.5
                           and abs(h.current_alt - target[2]) <= 1.0,
                           NAV_TIMEOUT_S, cancel):
            if cancel.is_set():
                return
            _hold(handlers, cancel)
            _log(f"[Mission] Navigation aborted: drone {h.sysid} did not arrive in time; remaining drones hold.")
            return
    _log("[Mission] Swarm arrived at planned target positions and is hovering.")


def _manual_follow_loop(cmd_data, handlers, cancel):
    leader = handlers[0]
    followers = handlers[1:]
    if not followers:
        raise ValueError("manual mode requires a follower")
    spacing = _number(cmd_data.get("spacing_m", 10.0), "spacing_m", MIN_FORMATION_SPACING_M)
    _number(cmd_data.get("alt", 10.0), "altitude", 1.0)
    formation = bool(cmd_data.get("formation", True))
    pattern = cmd_data.get("pattern", "V")
    if formation and pattern not in ("V", "line"):
        raise ValueError("unknown formation pattern")
    retained_offsets = None
    with _worker_lock:
        if cancel.is_set() or _current_cancel is not cancel:
            return
        _manual_status(True)
    _log("[Manual Mode] Waiting for leader to become armed and airborne.")
    was_flying = False
    launched = False
    while not cancel.is_set():
        notify_user_activity()
        flying = leader.armed and leader.current_alt >= 1.0
        if not flying:
            if was_flying and _landed(leader):
                _log("[Manual Mode] Leader grounded and disarmed; landing followers.")
                _land_ordered(followers, "land", cancel)
                return
            cancel.wait(0.2)
            continue
        was_flying = True
        _position(leader)
        target_alt = _number(leader.current_alt, "leader altitude", 1.0)
        if not launched:
            # A vertical launch is unsafe when the current horizontal holds
            # already overlap; do not start follower takeoff in that case.
            current_holds = {h: _position(h) for h in handlers}
            _safe_sequential_paths(handlers, current_holds)
            for follower in followers:
                if cancel.is_set():
                    return
                if follower.armed and follower.current_alt >= 1.0:
                    continue
                if not _call(cancel, follower, "takeoff", target_alt):
                    return
                if not _wait_until(lambda f=follower: f.armed and f.current_alt >= max(1.0, target_alt - 1.0),
                                   TAKEOFF_TIMEOUT_S, cancel):
                    if cancel.is_set():
                        return
                    _log(f"[Manual Mode] Follower {follower.sysid} did not reach hover altitude; follow aborted.")
                    return
            launched = True
        if formation:
            heading = _number(leader.yaw, "leader yaw")
            offsets = calculate_formation_offsets(leader.lat, leader.lon, heading,
                                                  len(followers), pattern, spacing)
            targets = {h: (p[0], p[1], target_alt) for h, p in zip(followers, offsets)}
        else:
            if retained_offsets is None:
                for follower in followers:
                    _position(follower)
                retained_offsets = {h: (h.lat - leader.lat, h.lon - leader.lon)
                                    for h in followers}
            targets = {h: (leader.lat + retained_offsets[h][0],
                           leader.lon + retained_offsets[h][1], target_alt)
                       for h in followers}
        if any(not (-90 <= target[0] <= 90 and -180 <= target[1] <= 180)
               for target in targets.values()):
            raise ValueError("manual follower target is outside valid coordinates")
        # Dispatch one follower at a time. The next leg is checked only after
        # the previous follower has reached a 0.5m hold circle.
        for follower in followers:
            if cancel.is_set() or not leader.armed or leader.current_alt < 1.0:
                break
            try:
                live_targets = {h: _position(h) for h in handlers}
                live_targets[follower] = targets[follower]
                _safe_sequential_paths(handlers, live_targets)
            except ValueError as exc:
                _hold(followers, cancel)
                _log(f"[Manual Mode] Holding followers: {exc}")
                break
            _call(cancel, follower, "fly_to_gps", *targets[follower], set_guided=False)
            def reached_or_leader_landed():
                if not leader.armed or leader.current_alt < 1.0:
                    return True
                # RC leader can move during the leg; reject an observed new
                # crossing instead of waiting blindly on the old safe plan.
                live = {h: _position(h) for h in handlers}
                live[follower] = targets[follower]
                _safe_sequential_paths(handlers, live)
                return _haversine_m(follower.lat, follower.lon,
                                    targets[follower][0], targets[follower][1]) <= 0.5

            try:
                arrived = _wait_until(reached_or_leader_landed, NAV_TIMEOUT_S, cancel)
            except ValueError as exc:
                _hold(followers, cancel)
                _log(f"[Manual Mode] Holding followers after leader movement: {exc}")
                return
            if not arrived:
                if cancel.is_set():
                    return
                _hold(followers, cancel)
                _log(f"[Manual Mode] Follower {follower.sysid} did not reach target; follow aborted.")
                return
            if cancel.wait(SERIAL_DISPATCH_SPACING_S):
                return
        cancel.wait(0.5)


def _inactivity_watchdog_loop():
    while True:
        time.sleep(1.0)
        if _mission_in_progress.is_set():
            notify_user_activity()
            continue
        active = [h for h in _active_handlers_ref if h.connection and h.sysid != "N/A"]
        if not any(h.armed and h.current_alt > 1.0 for h in active):
            notify_user_activity()
        elif time.time() - _last_activity_time >= INACTIVITY_RTL_TIMEOUT_S:
            _log("[AUTO-RTL] Inactivity timeout; returning swarm to launch.")
            perform_swarm_command({"cmd": "rtl"}, active)


def _start_inactivity_watchdog():
    global _watchdog_thread_started
    if not _watchdog_thread_started:
        _watchdog_thread_started = True
        threading.Thread(target=_inactivity_watchdog_loop, daemon=True).start()


def perform_swarm_command(cmd_data, drone_handlers):
    global _active_handlers_ref
    notify_user_activity()
    if cmd_data.get("type") == "set_leader":
        set_leader(cmd_data.get("sysid", 1))
        return
    active = sorted((h for h in drone_handlers if h.sysid != "N/A" and h.connection),
                    key=lambda h: h.sysid)
    handlers = _ordered(active)
    if not handlers:
        return
    _active_handlers_ref = active
    _start_inactivity_watchdog()
    cmd = cmd_data.get("cmd")
    if cmd == "manual_mode":
        if stop_manual_mode():
            return
        _start_worker(cmd, lambda cancel: _manual_follow_loop(cmd_data, handlers, cancel))
    elif cmd == "fly_to":
        _start_worker(cmd, lambda cancel: _autonomous_mission_worker(cmd_data, handlers, cancel))
    elif cmd == "takeoff":
        try:
            altitude = _number(cmd_data.get("alt", 5), "altitude", 1.0)
        except (TypeError, ValueError) as exc:
            _log(f"[Swarm] Takeoff aborted: {exc}")
            return
        _start_worker(cmd, lambda cancel: _takeoff_ordered(handlers, altitude, cancel))
    elif cmd in ("land", "rtl"):
        _start_worker(cmd, lambda cancel: _land_ordered(handlers, cmd, cancel))
    elif cmd in ("arm", "disarm"):
        _cancel_worker()
        # Emergency disarm has no stagger or telemetry wait. This dispatch is
        # serialized with old navigation, with the selected leader first.
        for h in handlers:
            with _dispatch_lock:
                getattr(h, "arm_drone" if cmd == "arm" else "disarm_drone")()
