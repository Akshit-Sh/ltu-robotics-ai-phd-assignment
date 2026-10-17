"""Pure-Python sensor heartbeat checks for robot health monitoring."""


def stale_sensor_names(
    now,
    started_at,
    last_odom_time,
    last_scan_time,
    startup_grace_sec=10.0,
    sensor_timeout_sec=2.0,
):
    """Return stale sensor names, respecting startup grace."""
    if now - started_at < startup_grace_sec:
        return ()

    stale = []
    for name, timestamp in (
        ("odometry", last_odom_time),
        ("lidar", last_scan_time),
    ):
        if timestamp <= 0.0 or now - timestamp > sensor_timeout_sec:
            stale.append(name)

    return tuple(stale)
