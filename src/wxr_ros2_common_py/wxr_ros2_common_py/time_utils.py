"""Time conversion helpers that rely on the ROS 2 clock API."""

from __future__ import annotations

from typing import Tuple

from rclpy.node import Node


def now_sec_nsec(node: Node) -> Tuple[int, int]:
    """Return the current ROS time as integer seconds and nanoseconds.

    Args:
        node: ROS 2 node whose clock should be queried.

    Returns:
        A tuple ``(sec, nsec)`` representing the current ROS time.
    """

    now = node.get_clock().now()
    return now.seconds_nanoseconds()


def now_sec_of_day(node: Node) -> Tuple[int, int]:
    """Return seconds since midnight (UTC) with nanosecond precision.

    Args:
        node: ROS 2 node whose clock should be queried.

    Returns:
        A tuple ``(sec_of_day, nsec)`` relative to midnight.
    """

    sec, nsec = now_sec_nsec(node)
    sec_of_day = sec % 86400
    return sec_of_day, nsec


def stamp_to_float(sec: int, nsec: int) -> float:
    """Convert integer ROS time stamp components into seconds as ``float``.

    Args:
        sec: Seconds component of the timestamp.
        nsec: Nanoseconds component of the timestamp.

    Returns:
        Floating-point timestamp expressed in seconds.
    """

    return float(sec) + float(nsec) / 1e9


