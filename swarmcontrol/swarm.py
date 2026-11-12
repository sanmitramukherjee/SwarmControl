"""Swarm command sequencing and conservative waypoint transition checks."""

import math
import threading
import time

from .config import (
    COLLISION_WARN_M,
    INACTIVITY_RTL_TIMEOUT_S,
    LAND_TIMEOUT_S,
    METERS_PER_DEG_LAT,
    METERS_PER_DEG_LON_EQ,
    MIN_FORMATION_SPACING_M,
    NAV_SPEED_ASSUMED_MPS,
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
                to_hold = (
                    [h for h in _active_handlers_ref if h.sysid != old_sysid]
                    if _current_kind == "manual_mode"
                    else _active_handlers_ref
                )
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
    return next(
        (h for h in active_handlers if h.sysid == chosen),
        active_handlers[0] if active_handlers else None,
    )


def _ordered(active_handlers):
    leader = get_leader(active_handlers)
    return (
        [leader]
        + sorted((h for h in active_handlers if h is not leader), key=lambda h: h.sysid)
        if leader
        else []
    )


def _number(value, name, minimum=None):
    result = float(value)
    if not math.isfinite(result) or (minimum is not None and result < minimum):
        raise ValueError(f"{name} must be finite and at least {minimum}")
    return result


def _position(h):
    lat, lon, alt = float(h.lat), float(h.lon), float(h.current_alt)
    if not all(map(math.isfinite, (lat, lon, alt))) or not (
        -90 <= lat <= 90 and -180 <= lon <= 180
    ):
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
        if not _wait_until(
            lambda: h.armed and h.current_alt >= max(1.0, altitude - 1.0),
            TAKEOFF_TIMEOUT_S,
            cancel,
        ):
            if cancel.is_set():
                return False
            _log(
                f"[Swarm] Takeoff aborted: drone {h.sysid} did not reach altitude in time; no further drones launched."
            )
            return False
        if i == 0:
            _log(
                f"[Swarm] Leader {leader.sysid} airborne; launching followers in order."
            )
    return True


_rtl_spacing_m = 10.0


def _rtl_centered_targets(handlers, leader, spacing, yaw):
    origin = _position(leader)
    # The RTL staging line is perpendicular to the fleet's initial launch
    # line, avoiding order swaps as a stacked SITL/field line spreads out.
    east_axis, north_axis = math.cos(yaw), -math.sin(yaw)
    metres_lon = METERS_PER_DEG_LON_EQ * math.cos(math.radians(origin[0]))
    follower_order = sorted(
        (h for h in handlers if h is not leader),
        key=lambda h: (
            _local_xy(_position(h), origin)[0] * math.sin(yaw)
            + _local_xy(_position(h), origin)[1] * math.cos(yaw)
        ),
    )
    targets = {leader: origin}
    side_indices = {1.0: 0, -1.0: 0}
    for follower in follower_order:
        # Keep each vehicle on the side of the leader it already occupies.
        follower_position = _local_xy(_position(follower), origin)
        side = (
            1.0
            if follower_position[0] * east_axis + follower_position[1] * north_axis >= 0
            else -1.0
        )
        side_indices[side] += 1
        distance = spacing * side_indices[side] * side
        targets[follower] = (
            origin[0] - north_axis * distance / METERS_PER_DEG_LAT,
            origin[1] - east_axis * distance / metres_lon,
            origin[2],
        )
    return targets


def _rtl_paths_are_safe(handlers, targets):
    starts = {h: _position(h) for h in handlers}
    absolute_starts = {}
    absolute_targets = {}
    for handler in handlers:
        home_alt = getattr(handler, "home_alt", None)
        if home_alt is None or not math.isfinite(float(home_alt)):
            # Without an absolute home reference, use same-frame AGL. It is
            # conservative for separate SITL home elevations.
            home_alt = 0.0
        absolute_starts[handler] = (
            starts[handler][0],
            starts[handler][1],
            float(home_alt) + starts[handler][2],
        )
        absolute_targets[handler] = (
            targets[handler][0],
            targets[handler][1],
            float(home_alt) + targets[handler][2],
        )

    # Compare horizontal and altitude routes in one shared world frame.
    origin = absolute_starts[handlers[0]]

    def local_route_points(handler, position):
        east, north = _local_xy(position, origin)
        return (east, north, position[2])

    routes = {}
    for handler in handlers:
        start = local_route_points(handler, absolute_starts[handler])
        end = local_route_points(handler, absolute_targets[handler])
        distance = math.sqrt(sum((end[i] - start[i]) ** 2 for i in range(3)))
        routes[handler] = (start, end, distance / NAV_SPEED_ASSUMED_MPS)

    def at(route, seconds):
        start, end, duration = route
        fraction = min(1.0, seconds / duration) if duration else 1.0
        return tuple(start[i] + (end[i] - start[i]) * fraction for i in range(3))

    nearest = math.inf
    for index, first in enumerate(handlers):
        for second in handlers[index + 1 :]:
            first_route, second_route = routes[first], routes[second]
            duration = max(first_route[2], second_route[2])
            boundaries = sorted({0.0, first_route[2], second_route[2], duration})
            for begin, end in zip(boundaries, boundaries[1:]):
                first_begin, first_end = at(first_route, begin), at(first_route, end)
                second_begin, second_end = (
                    at(second_route, begin),
                    at(second_route, end),
                )
                relative_begin = tuple(
                    first_begin[i] - second_begin[i] for i in range(3)
                )
                relative_end = tuple(first_end[i] - second_end[i] for i in range(3))
                nearest = min(
                    nearest,
                    _point_segment_distance(
                        (0.0, 0.0, 0.0), relative_begin, relative_end
                    ),
                )
    if nearest < COLLISION_WARN_M + 0.5:
        return False
    for index, first in enumerate(handlers):
        for second in handlers[index + 1 :]:
            first_target = absolute_targets[first]
            second_target = absolute_targets[second]
            east, north = _local_xy(first_target, second_target)
            horizontal = math.hypot(east, north)
            vertical = abs(first_target[2] - second_target[2])
            if math.hypot(horizontal, vertical) < COLLISION_WARN_M + 0.5:
                return False
    return True


def _land_ordered(handlers, method, cancel):
    if method == "rtl":
        leader, followers = handlers[0], handlers[1:]
        # ArduPilot RTL has no formation awareness: drones launched in a line
        # can converge on home together. Re-form them with the leader at the
        # center of the existing line before issuing leader-first RTL.
        if followers and all(h.current_alt > 1.0 for h in handlers):
            yaw = math.radians(_number(leader.yaw, "leader yaw"))
            spacing = max(MIN_FORMATION_SPACING_M, _rtl_spacing_m)
            targets = _rtl_centered_targets(handlers, leader, spacing, yaw)
            can_reform = _rtl_paths_are_safe(handlers, targets)
            if can_reform:
                _log("[RTL] Re-forming the line with leader at its center before RTL.")
                if not _dispatch_group_waypoints(
                    handlers, targets, cancel, "RTL form-up"
                ):
                    return
                # ArduPilot RTL targets each vehicle's own home coordinate,
                # so vehicles whose launch points were staggered still
                # converge. Group-move them to their individual home points
                # in safely separated altitude lanes first; once grounded,
                # the leader-first RTL sequence skips landed vehicles.
                origin = _position(leader)
                home_targets = {
                    h: (
                        origin[0],
                        origin[1],
                        max(1.0, origin[2] - spacing * index)
                        if getattr(h, "home_alt", None) is None
                        else origin[2],
                    )
                    for index, h in enumerate(handlers)
                }
                if _rtl_paths_are_safe(handlers, home_targets):
                    _log("[RTL] Returning the formed fleet to common home together.")
                    if not _dispatch_group_waypoints(
                        handlers, home_targets, cancel, "common-home return"
                    ):
                        return
            else:
                _log(
                    "[RTL] Keeping current positions; centered form-up path is unsafe."
                )
        if not _landed(leader) and not _call(cancel, leader, "rtl"):
            return
        if followers and cancel.wait(SERIAL_DISPATCH_SPACING_S):
            return
        for follower in followers:
            if cancel.is_set():
                return
            if not _landed(follower) and not _call(cancel, follower, "rtl"):
                return
        if _wait_until(
            lambda: all(_landed(h) for h in handlers), LAND_TIMEOUT_S, cancel
        ):
            for h in handlers:
                _log(f"[Swarm] Drone {h.sysid} returned and grounded.")
        elif not cancel.is_set():
            pending = ", ".join(str(h.sysid) for h in handlers if not _landed(h))
            _log(f"[Swarm] RTL timed out; drones still airborne: {pending}")
        return

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
                _log(
                    f"[Swarm] {method.upper()} aborted: drone {h.sysid} was not observed grounded and disarmed; later drones remain on hold."
                )
                return
        _log(f"[Swarm] Drone {h.sysid} grounded and disarmed.")


def _target_map(cmd_data, handlers, leader, lat, lon, alt, follower_order=None):
    followers = follower_order or [h for h in handlers if h is not leader]
    spacing = _number(
        cmd_data.get("spacing_m", 10.0), "spacing_m", MIN_FORMATION_SPACING_M
    )
    result = {leader: (lat, lon, alt)}
    if cmd_data.get("formation", False):
        heading = _number(leader.yaw, "leader yaw")
        offsets = calculate_formation_offsets(
            lat,
            lon,
            heading,
            len(followers),
            pattern=cmd_data.get("pattern", "V"),
            spacing_m=spacing,
        )
        if len(offsets) != len(followers):
            raise ValueError("unknown formation pattern")
        result.update((h, (p[0], p[1], alt)) for h, p in zip(followers, offsets))
    else:
        # Preserve existing horizontal offsets from the leader when formation is off.
        # A common waypoint would cause convergence even with altitude staggering.
        old_leader = _position(leader)
        for h in followers:
            old = _position(h)
            result[h] = (
                lat + old[0] - old_leader[0],
                lon + old[1] - old_leader[1],
                alt,
            )
    for h, p in result.items():
        if not (-90 <= p[0] <= 90 and -180 <= p[1] <= 180):
            raise ValueError(f"target for drone {h.sysid} is outside valid coordinates")
    return result


def _formation_follower_order(handlers, leader, formation, pattern):
    followers = [h for h in handlers if h is not leader]
    if not formation or not followers:
        return followers
    heading = math.radians(_number(leader.yaw, "leader yaw"))
    forward_east, forward_north = math.sin(heading), math.cos(heading)
    origin = _position(leader)
    metres_lon = METERS_PER_DEG_LON_EQ * math.cos(math.radians(origin[0]))
    projection = lambda h: (
        (h.lon - origin[1]) * metres_lon * forward_east
        + (h.lat - origin[0]) * METERS_PER_DEG_LAT * forward_north
    )
    # Assign the closest follower to the nearest trailing slot while preserving
    # the fleet's physical order. This prevents routes swapping across each other.
    return sorted(followers, key=lambda h: (projection(h), -int(h.sysid)), reverse=True)


def _local_xy(position, origin):
    metres_lon = METERS_PER_DEG_LON_EQ * math.cos(math.radians(origin[0]))
    return (
        (position[1] - origin[1]) * metres_lon,
        (position[0] - origin[0]) * METERS_PER_DEG_LAT,
    )


def _point_segment_distance(point, start, end):
    delta = [end[i] - start[i] for i in range(len(start))]
    length2 = sum(value * value for value in delta)
    if not length2:
        return math.sqrt(sum((point[i] - start[i]) ** 2 for i in range(len(start))))
    fraction = max(
        0.0,
        min(
            1.0,
            sum((point[i] - start[i]) * delta[i] for i in range(len(start))) / length2,
        ),
    )
    return math.sqrt(
        sum(
            (point[i] - (start[i] + fraction * delta[i])) ** 2
            for i in range(len(start))
        )
    )


def _group_route_separation(starts, targets):
    handlers = list(targets)
    if len(handlers) < 2:
        return math.inf
    origin = starts[handlers[0]]

    def local_xyz(handler, position):
        east, north = _local_xy(position, origin)
        home_alt = getattr(handler, "home_alt", None)
        if home_alt is not None and math.isfinite(float(home_alt)):
            altitude = float(home_alt) + position[2]
        else:
            altitude = position[2]
        return east, north, altitude

    origin_alt = local_xyz(handlers[0], origin)[2]
    routes = {}
    for h in handlers:
        start = local_xyz(h, starts[h])
        end = local_xyz(h, targets[h])
        start = (start[0], start[1], start[2] - origin_alt)
        end = (end[0], end[1], end[2] - origin_alt)
        distance = math.sqrt(sum((end[i] - start[i]) ** 2 for i in range(3)))
        routes[h] = (start, end, distance / NAV_SPEED_ASSUMED_MPS)

    def at(route, seconds):
        start, end, duration = route
        fraction = min(1.0, seconds / duration) if duration else 1.0
        return tuple(start[i] + (end[i] - start[i]) * fraction for i in range(3))

    nearest = math.inf
    for i, first in enumerate(handlers):
        for second in handlers[i + 1 :]:
            first_route, second_route = routes[first], routes[second]
            duration = max(first_route[2], second_route[2])
            boundaries = sorted({0.0, first_route[2], second_route[2], duration})
            for begin, end in zip(boundaries, boundaries[1:]):
                first_begin, first_end = at(first_route, begin), at(first_route, end)
                second_begin, second_end = (
                    at(second_route, begin),
                    at(second_route, end),
                )
                relative_begin = (
                    first_begin[0] - second_begin[0],
                    first_begin[1] - second_begin[1],
                    first_begin[2] - second_begin[2],
                )
                relative_end = (
                    first_end[0] - second_end[0],
                    first_end[1] - second_end[1],
                    first_end[2] - second_end[2],
                )
                nearest = min(
                    nearest,
                    _point_segment_distance(
                        (0.0, 0.0, 0.0), relative_begin, relative_end
                    ),
                )
    return nearest


def _coordinated_stage_targets(cmd_data, handlers, leader, final_targets, alt):
    starts = {h: _position(h) for h in handlers}
    formation = bool(cmd_data.get("formation", False))
    pattern = cmd_data.get("pattern", "V")
    follower_order = _formation_follower_order(handlers, leader, formation, pattern)
    heading = math.radians(_number(leader.yaw, "leader yaw"))
    right_east, right_north = math.cos(heading), -math.sin(heading)
    spacing = _number(
        cmd_data.get("spacing_m", 10.0), "spacing_m", MIN_FORMATION_SPACING_M
    )
    stage_distance = max(20.0, spacing * (len(follower_order) + 2))
    metres_lon = METERS_PER_DEG_LON_EQ * math.cos(math.radians(starts[leader][0]))
    minimum = COLLISION_WARN_M + 0.5
    leader_stage_alt = min(alt + 3.0, 10000.0)
    flight_starts = {
        h: (starts[h][0], starts[h][1], leader_stage_alt if h is leader else alt)
        for h in handlers
    }
    choices = []
    for sign in (-1, 1):
        east = sign * right_east * stage_distance
        north = sign * right_north * stage_distance
        stage_lat = starts[leader][0] + north / METERS_PER_DEG_LAT
        stage_lon = starts[leader][1] + east / metres_lon
        stage = _target_map(
            cmd_data,
            handlers,
            leader,
            stage_lat,
            stage_lon,
            alt,
            follower_order=follower_order,
        )
        stage[leader] = (stage_lat, stage_lon, leader_stage_alt)
        final_starts = dict(stage)
        final_starts[leader] = (stage[leader][0], stage[leader][1], alt)
        clearance = min(
            _group_route_separation(flight_starts, stage),
            _group_route_separation(final_starts, final_targets),
        )
        choices.append((clearance, stage))
    clearance, stage = max(choices, key=lambda choice: choice[0])
    if clearance < minimum:
        raise ValueError(
            f"no collision-clear coordinated formation route is available "
            f"(closest planned separation {clearance:.1f}m; need {minimum:.1f}m)"
        )
    return stage, follower_order


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
            if not segment_clear_of_point(
                flat_start, flat_end, (hold[0], hold[1], 0.0), COLLISION_WARN_M + 0.5
            ):
                raise ValueError(
                    f"unsafe path for drone {moving.sysid} near drone {other.sysid}; navigation held"
                )
        holds[moving] = end


def _wait_group_arrival(handlers, targets, cancel, description):
    def all_arrived():
        return all(
            _haversine_m(h.lat, h.lon, targets[h][0], targets[h][1]) <= 0.5
            and abs(h.current_alt - targets[h][2]) <= 1.0
            for h in handlers
        )

    if _wait_until(all_arrived, NAV_TIMEOUT_S, cancel):
        return True
    if cancel.is_set():
        return False
    _hold(handlers, cancel)
    pending = ", ".join(
        str(h.sysid)
        for h in handlers
        if _haversine_m(h.lat, h.lon, targets[h][0], targets[h][1]) > 0.5
        or abs(h.current_alt - targets[h][2]) > 1.0
    )
    _log(
        f"[Mission] {description} aborted: arrival timed out for drones {pending}; fleet held."
    )
    return False


def _dispatch_group_waypoints(handlers, targets, cancel, description):
    starts = {h: _position(h) for h in handlers}
    if _group_route_separation(starts, targets) < COLLISION_WARN_M + 0.5:
        if description != "target flight":
            _hold(handlers, cancel)
            raise ValueError(f"unsafe coordinated {description} path; fleet held")
        leader = handlers[0]
        leader_start = starts[leader]
        leader_target = targets[leader]
        # Route estimates use a nominal speed and can be overly conservative
        # for long legs. Preserve the current formation by translating every
        # aircraft by the leader's requested displacement. This keeps the
        # leader on the exact requested coordinate and avoids follower path
        # crossings caused by reassigning formation slots mid-flight.
        targets = {
            h: (
                leader_target[0] + starts[h][0] - leader_start[0],
                leader_target[1] + starts[h][1] - leader_start[1],
                leader_target[2] + starts[h][2] - leader_start[2],
            )
            for h in handlers
        }
        targets[leader] = leader_target
        _log(
            f"[Mission] Reusing current formation for {description}; leader waypoint unchanged."
        )
    for h in handlers:
        if not _call(cancel, h, "fly_to_gps", *targets[h], set_guided=False):
            return False
    return _wait_group_arrival(handlers, targets, cancel, description)


def _takeoff_group(handlers, altitude, cancel):
    for h in handlers:
        if not _call(cancel, h, "prepare_takeoff"):
            return False
    if not _wait_until(
        lambda: all(h.armed for h in handlers), TAKEOFF_TIMEOUT_S, cancel
    ):
        if not cancel.is_set():
            pending = ", ".join(str(h.sysid) for h in handlers if not h.armed)
            _log(
                f"[Mission] Arming aborted: drones not armed: {pending}; no takeoff commands sent."
            )
        return False
    _log("[Mission] Fleet armed; leader taking off first.")
    leader = handlers[0]
    leader_altitude = min(altitude + 3.0, 10000.0)
    if not _call(cancel, leader, "start_takeoff", leader_altitude):
        return False
    if cancel.wait(1.0):
        return False
    if not leader.armed:
        _log(
            f"[Mission] Leader {leader.sysid} did not arm after takeoff command; follower takeoff aborted."
        )
        return False
    _log(
        f"[Mission] Leader {leader.sysid} takeoff started; launching all followers together after 1s."
    )
    for follower in handlers[1:]:
        if not _call(cancel, follower, "start_takeoff", altitude):
            return False
    if not _wait_until(
        lambda: (
            leader.armed
            and leader.current_alt >= max(1.0, leader_altitude - 1.0)
            and all(
                h.armed and h.current_alt >= max(1.0, altitude - 1.0)
                for h in handlers[1:]
            )
        ),
        TAKEOFF_TIMEOUT_S,
        cancel,
    ):
        if not cancel.is_set():
            pending = ", ".join(
                str(h.sysid)
                for h in handlers
                if not h.armed
                or h.current_alt
                < max(1.0, (leader_altitude if h is leader else altitude) - 1.0)
            )
            _hold(handlers, cancel)
            _log(
                f"[Mission] Follower launch aborted: drones not airborne: {pending}; fleet held."
            )
        return False
    return True


def _coordinated_ground_mission(cmd_data, handlers, leader, lat, lon, altitude, cancel):
    formation = bool(cmd_data.get("formation", False))
    pattern = cmd_data.get("pattern", "V")
    follower_order = _formation_follower_order(handlers, leader, formation, pattern)
    final_targets = _target_map(
        cmd_data, handlers, leader, lat, lon, altitude, follower_order=follower_order
    )
    stage_targets, _ = _coordinated_stage_targets(
        cmd_data, handlers, leader, final_targets, altitude
    )
    if not _takeoff_group(handlers, altitude, cancel):
        return
    _log("[Mission] Fleet airborne; moving together to the formation point.")
    if not _dispatch_group_waypoints(handlers, stage_targets, cancel, "formation-up"):
        return
    leader_level = {
        leader: (stage_targets[leader][0], stage_targets[leader][1], altitude)
    }
    _log("[Mission] Leader lowering to formation altitude.")
    if not _dispatch_group_waypoints(
        [leader], leader_level, cancel, "leader altitude join"
    ):
        return
    _log("[Mission] Formation established; flying to the requested location together.")
    if not _dispatch_group_waypoints(handlers, final_targets, cancel, "target flight"):
        return
    _log("[Mission] Swarm arrived at planned target positions and is hovering.")


def _autonomous_mission_worker(cmd_data, handlers, cancel):
    lat = _number(cmd_data["lat"], "latitude")
    lon = _number(cmd_data["lon"], "longitude")
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise ValueError("target coordinates out of range")
    alt = _number(cmd_data.get("alt", 5.0), "altitude", 1.0)
    leader = handlers[0]
    all_landed = all(_landed(h) for h in handlers)
    all_airborne = all(h.armed and h.current_alt >= 1.0 for h in handlers)
    if len(handlers) > 1 and all_landed:
        _coordinated_ground_mission(cmd_data, handlers, leader, lat, lon, alt, cancel)
        return
    if not all_airborne and not (len(handlers) == 1 and all_landed):
        raise ValueError("fly-to requires the whole fleet grounded or already airborne")
    follower_order = _formation_follower_order(
        handlers,
        leader,
        bool(cmd_data.get("formation", False)),
        cmd_data.get("pattern", "V"),
    )
    targets = _target_map(
        cmd_data, handlers, leader, lat, lon, alt, follower_order=follower_order
    )
    if len(handlers) == 1 and all_landed:
        if not _takeoff_ordered(handlers, alt, cancel):
            return
        if not _call(cancel, leader, "fly_to_gps", *targets[leader]):
            return
        _wait_group_arrival(handlers, targets, cancel, "target flight")
        return
    if not _dispatch_group_waypoints(handlers, targets, cancel, "target flight"):
        return
    _log("[Mission] Swarm arrived at planned target positions and is hovering.")


def _manual_follow_loop(cmd_data, handlers, cancel):
    leader = handlers[0]
    followers = handlers[1:]
    if not followers:
        raise ValueError("manual mode requires a follower")
    spacing = _number(
        cmd_data.get("spacing_m", 10.0), "spacing_m", MIN_FORMATION_SPACING_M
    )
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
                if not _wait_until(
                    lambda f=follower: (
                        f.armed and f.current_alt >= max(1.0, target_alt - 1.0)
                    ),
                    TAKEOFF_TIMEOUT_S,
                    cancel,
                ):
                    if cancel.is_set():
                        return
                    _log(
                        f"[Manual Mode] Follower {follower.sysid} did not reach hover altitude; follow aborted."
                    )
                    return
            launched = True
        if formation:
            heading = _number(leader.yaw, "leader yaw")
            offsets = calculate_formation_offsets(
                leader.lat, leader.lon, heading, len(followers), pattern, spacing
            )
            targets = {h: (p[0], p[1], target_alt) for h, p in zip(followers, offsets)}
        else:
            if retained_offsets is None:
                for follower in followers:
                    _position(follower)
                retained_offsets = {
                    h: (h.lat - leader.lat, h.lon - leader.lon) for h in followers
                }
            targets = {
                h: (
                    leader.lat + retained_offsets[h][0],
                    leader.lon + retained_offsets[h][1],
                    target_alt,
                )
                for h in followers
            }
        if any(
            not (-90 <= target[0] <= 90 and -180 <= target[1] <= 180)
            for target in targets.values()
        ):
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
                return (
                    _haversine_m(
                        follower.lat,
                        follower.lon,
                        targets[follower][0],
                        targets[follower][1],
                    )
                    <= 0.5
                )

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
                _log(
                    f"[Manual Mode] Follower {follower.sysid} did not reach target; follow aborted."
                )
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
    active = sorted(
        (h for h in drone_handlers if h.sysid != "N/A" and h.connection),
        key=lambda h: h.sysid,
    )
    handlers = _ordered(active)
    if not handlers:
        return
    _active_handlers_ref = active
    _start_inactivity_watchdog()
    cmd = cmd_data.get("cmd")
    if cmd == "manual_mode":
        if stop_manual_mode():
            return
        _start_worker(
            cmd, lambda cancel: _manual_follow_loop(cmd_data, handlers, cancel)
        )
    elif cmd == "fly_to":
        _start_worker(
            cmd, lambda cancel: _autonomous_mission_worker(cmd_data, handlers, cancel)
        )
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
