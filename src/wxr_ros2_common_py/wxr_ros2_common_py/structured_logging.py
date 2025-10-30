"""Structured logging helpers shared across teleoperation nodes."""

from __future__ import annotations

from typing import Any, Iterable, Mapping, Optional, TextIO

from rclpy.node import Node

from .qos_utils import qos_to_multiline


def _format_params(params: Mapping[str, Any]) -> str:
    """Render parameter mappings in ``key=value`` form for log messages."""

    if not params:
        return "(none)"
    return ", ".join(f"{k}={v}" for k, v in params.items())


def log_startup(
    node: Node, params: Mapping[str, Any], endpoints: Iterable[Mapping[str, Any]]
) -> None:
    """Emit startup details including parameters and declared endpoints.

    Args:
        node: ROS 2 node used to write to the log.
        params: Mapping of configuration parameter names to values.
        endpoints: Iterable describing subscriptions, publications and
            services. Each item should contain ``dir`` (``sub``, ``pub``,
            ``server`` or ``client``), endpoint identifiers (``topic`` or
            ``service``), ``type`` and optional ``qos`` information.

    Example:
        >>> log_startup(
        ...     node,
        ...     params={'enable_pipeline': True},
        ...     endpoints=[{'dir': 'pub', 'topic': '/cmd_vel', 'type': 'Twist'}],
        ... )

    Returns:
        ``None``. Results are written to the ROS 2 log.
    """

    node.get_logger().info(f"[Param]: {_format_params(params)}")
    for ep in endpoints:
        direction = ep.get("dir", "").lower()

        if direction == "sub":
            tag = "[Sub]"
            topic = ep.get("topic", "")
        elif direction == "pub":
            tag = "[Pub]"
            topic = ep.get("topic", "")
        elif direction == "server":
            tag = "[Service-Server]"
            topic = ep.get("service", "")
        elif direction == "client":
            tag = "[Service-Client]"
            topic = ep.get("service", "")
        else:
            tag = "[Endpoint]"
            topic = ep.get("topic", ep.get("service", ""))

        mtype = ep.get("type", "")
        qos = ep.get("qos", None)
        node.get_logger().info(
            f"{tag}: {topic} [{mtype}] qos={qos_to_multiline(qos)}"
        )


def log_latency(
    node: Node,
    label: str,
    start_time_sec: float,
    end_time_sec: float,
    file_handle: Optional[TextIO] = None,
) -> None:
    """Log the latency between ``start_time_sec`` and ``end_time_sec``.

    Args:
        node: ROS 2 node used to emit the latency message.
        label: Description included in the log output.
        start_time_sec: Start timestamp in seconds.
        end_time_sec: End timestamp in seconds.
        file_handle: Optional handle that receives the same log message.

    Example:
        >>> start = node.get_clock().now().nanoseconds / 1e9
        >>> end = node.get_clock().now().nanoseconds / 1e9
        >>> log_latency(node, 'pipeline', start, end)

    Returns:
        ``None``. The log message is emitted and optionally persisted.
    """

    latency = end_time_sec - start_time_sec
    msg = f"[Latency] {label}: {latency:.6f} sec"
    node.get_logger().info(msg)
    if file_handle is not None:
        file_handle.write(msg + "\n")
        file_handle.flush()


