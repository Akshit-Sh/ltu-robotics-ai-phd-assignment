"""Tests for sensor heartbeat and startup-grace handling."""

from multi_robot_sim.health_monitor import stale_sensor_names


def test_startup_grace_ignores_missing_sensor_messages():
    assert stale_sensor_names(5.0, 0.0, 0.0, 0.0) == ()


def test_fresh_sensor_messages_are_healthy():
    assert stale_sensor_names(
        20.0, 0.0, 19.0, 19.5
    ) == ()


def test_stale_odometry_is_detected():
    assert stale_sensor_names(
        20.0, 0.0, 17.0, 19.5
    ) == ("odometry",)


def test_stale_lidar_is_detected():
    assert stale_sensor_names(
        20.0, 0.0, 19.5, 17.0
    ) == ("lidar",)


def test_missing_sensor_streams_are_detected_after_grace():
    assert stale_sensor_names(
        20.0, 0.0, 0.0, 19.5
    ) == ("odometry",)
