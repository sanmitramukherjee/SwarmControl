import math

from .config import COLLISION_WARN_M, METERS_PER_DEG_LAT, METERS_PER_DEG_LON_EQ


# ======================== Formation Engine ========================
def _meters_to_deg(lat_deg):
    """Return (m_per_deg_lat, m_per_deg_lon) at the given latitude."""
    lat_rad = math.radians(lat_deg)
    return METERS_PER_DEG_LAT, METERS_PER_DEG_LON_EQ * math.cos(lat_rad)


def _haversine_m(lat1, lon1, lat2, lon2):
    """Horizontal distance in metres between two lat/lon points."""
    EARTH_RADIUS_M = 6_371_000
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(math.radians(lat1))
        * math.cos(math.radians(lat2))
        * math.sin(dlon / 2) ** 2
    )
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def check_formation_separation(positions_with_alt, min_sep_m=COLLISION_WARN_M):
    """
    Check pairwise 3-D separation between target positions.

    Args:
        positions_with_alt: list of (lat, lon, alt) tuples.
        min_sep_m:          Minimum acceptable 3-D separation in metres.

    Returns:
        List of (i, j, dist_3d_m) for every pair that is closer than min_sep_m.
    """
    warnings = []
    for i in range(len(positions_with_alt)):
        for j in range(i + 1, len(positions_with_alt)):
            lat1, lon1, alt1 = positions_with_alt[i]
            lat2, lon2, alt2 = positions_with_alt[j]
            h = _haversine_m(lat1, lon1, lat2, lon2)
            v = abs(alt1 - alt2)
            dist_3d = math.sqrt(h * h + v * v)
            if dist_3d < min_sep_m:
                warnings.append((i, j, dist_3d))
    return warnings


def calculate_formation_offsets(
    leader_lat,
    leader_lon,
    leader_heading_deg,
    follower_count,
    pattern="V",
    spacing_m=10.0,
):
    """
    Compute GPS positions for *follower_count* drones in the requested formation
    relative to the leader.

    Supported patterns:
        "V"    – classic V-shape, followers trailing and fanning outward
        "line" – single-file trail behind leader

    Returns:
        List of (lat, lon) tuples, one per follower.
    """
    m_per_lat, m_per_lon = _meters_to_deg(leader_lat)
    heading_rad = math.radians(leader_heading_deg)

    # Forward unit vector (NED: X=East, Y=North)
    fwd_x = math.sin(heading_rad)
    fwd_y = math.cos(heading_rad)
    # Right unit vector (90° CW from forward)
    right_x = math.cos(heading_rad)
    right_y = -math.sin(heading_rad)

    positions = []

    if pattern == "V":
        for i in range(follower_count):
            row = (i // 2) + 1
            side = 1 if i % 2 == 0 else -1  # even=right, odd=left

            behind_m = row * spacing_m
            side_m = row * spacing_m * side

            # World offsets (East=x, North=y)
            offset_x = -behind_m * fwd_x + side_m * right_x
            offset_y = -behind_m * fwd_y + side_m * right_y

            positions.append(
                (
                    leader_lat + offset_y / m_per_lat,
                    leader_lon + offset_x / m_per_lon,
                )
            )

    elif pattern == "line":
        for i in range(follower_count):
            behind_m = (i + 1) * spacing_m
            offset_x = -behind_m * fwd_x
            offset_y = -behind_m * fwd_y
            positions.append(
                (
                    leader_lat + offset_y / m_per_lat,
                    leader_lon + offset_x / m_per_lon,
                )
            )

    else:
        # Unknown pattern — return empty list so the caller skips unknown followers
        # rather than stacking all drones at the same position (collision hazard).
        print(
            f"[Formation] Unknown pattern '{pattern}' – returning no follower positions."
        )
        positions = []

    return positions
