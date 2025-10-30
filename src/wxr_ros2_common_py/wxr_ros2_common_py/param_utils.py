"""Utilities for declaring and retrieving typed ROS 2 parameters."""

from __future__ import annotations

from typing import Iterable, TypeVar

from rclpy.node import Node


def get_bool_param(node: Node, name: str, default: bool = False) -> bool:
    """Fetch a boolean parameter, declaring it with ``default`` if necessary.

    Args:
        node: ROS 2 node providing the parameter API.
        name: Fully-qualified parameter name.
        default: Value used when the parameter is undeclared.

    Returns:
        The boolean parameter value retrieved from ``node``.

    Example:
        >>> enabled = get_bool_param(node, 'enable_feature', default=True)
    """

    node.declare_parameter(name, default)
    return bool(node.get_parameter(name).value)


_T = TypeVar("_T")


def get_enum_param(
    node: Node, name: str, allowed: Iterable[_T], default: _T
) -> _T:
    """Return a parameter constrained to a known set of values.

    Args:
        node: ROS 2 node providing the parameter API.
        name: Fully-qualified parameter name.
        allowed: Iterable of allowed parameter values.
        default: Value to fall back to if the retrieved value is not allowed.

    Returns:
        A value from ``allowed`` that represents the current parameter value.

    Example:
        >>> mode = get_enum_param(node, 'mode', {'auto', 'manual'}, 'auto')
    """

    node.declare_parameter(name, default)
    value = node.get_parameter(name).value
    if value not in allowed:
        node.get_logger().warn(
            f"Invalid {name}='{value}', falling back to '{default}'"
        )
        return default
    return value


