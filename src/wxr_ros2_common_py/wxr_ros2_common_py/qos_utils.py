"""Formatting helpers for ROS 2 QoS profiles."""

from __future__ import annotations

from typing import Any


def qos_to_multiline(qos_profile: Any) -> str:
    """Render a ROS 2 QoS profile as a newline-prefixed string block.

    Args:
        qos_profile: Object with QoS attributes (history, depth, reliability,
            etc.).  Typically a :class:`rclpy.qos.QoSProfile` instance.

    Returns:
        A string containing each QoS attribute on a separate line that is easy
        to embed within log messages.

    Example:
        >>> from rclpy.qos import QoSProfile
        >>> qos_to_multiline(QoSProfile(depth=10))
        '\\n    history: RMW_QOS_POLICY_HISTORY_KEEP_LAST\\n    depth: 10'
    """

    try:

        def _name(value: Any) -> str:
            return getattr(value, "name", str(value))

        lines = []
        if hasattr(qos_profile, "history"):
            lines.append(f"\n    history: {_name(qos_profile.history)}")
        if hasattr(qos_profile, "depth"):
            lines.append(f"\n    depth: {getattr(qos_profile, 'depth', '')}")
        if hasattr(qos_profile, "reliability"):
            lines.append(f"\n    reliability: {_name(qos_profile.reliability)}")
        if hasattr(qos_profile, "durability"):
            lines.append(f"\n    durability: {_name(qos_profile.durability)}")
        if hasattr(qos_profile, "liveliness"):
            lines.append(f"\n    liveliness: {_name(qos_profile.liveliness)}")
        return "".join(lines) if lines else "\n    (no qos fields)"
    except Exception:
        return "\n    (error printing qos)"


