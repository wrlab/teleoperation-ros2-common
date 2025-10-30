"""Helpers for tracking transient publisher/subscriber activity.

The :class:`ActivityTracker` collects topic names for subscriptions and
publications that occur between timer callbacks.  When the timer fires, the
tracked topics are emitted through the provided ROS 2 node logger.  This is
useful for debugging nodes that create subscriptions or publishers in response
to callbacks rather than at startup.
"""

from __future__ import annotations

from typing import MutableSet

from rclpy.node import Node


class ActivityTracker:
    """Track subscription and publication activity within a ROS 2 node.

    Args:
        node: ROS 2 node used to create the timer and emit log messages.
        interval_s: Frequency in seconds at which accumulated activity is
            reported.

    Example:
        >>> tracker = ActivityTracker(node)
        >>> tracker.mark_sub('/image_raw')
        >>> tracker.mark_pub('/processed_image')
        ... # timer callback logs "[Activity] sub: /image_raw" etc.
    """

    def __init__(self, node: Node, interval_s: float = 1.0) -> None:
        self._node: Node = node
        self._sub: MutableSet[str] = set()
        self._pub: MutableSet[str] = set()
        self._timer = node.create_timer(interval_s, self._tick)

    def mark_sub(self, topic: str) -> None:
        """Record that a subscription was created for ``topic``.

        Args:
            topic: Name of the subscription topic.

        Returns:
            ``None``. The topic is buffered until the next timer tick.
        """

        self._sub.add(topic)

    def mark_pub(self, topic: str) -> None:
        """Record that a publisher was created for ``topic``.

        Args:
            topic: Name of the publication topic.

        Returns:
            ``None``. The topic is buffered until the next timer tick.
        """

        self._pub.add(topic)

    def _tick(self) -> None:
        """Flush accumulated activity to the ROS 2 log output."""

        if self._sub:
            topics = ', '.join(sorted(self._sub))
            self._node.get_logger().info(f"[Activity] sub: {topics}")
        if self._pub:
            topics = ', '.join(sorted(self._pub))
            self._node.get_logger().info(f"[Activity] pub: {topics}")
        self._sub.clear()
        self._pub.clear()


