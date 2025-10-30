"""Angle conversion helpers used by teleoperation utilities."""

from __future__ import annotations

import math
from typing import Iterable, List


def rad_to_deg(x: float) -> float:
    """Convert radians to degrees.

    Args:
        x: Angle expressed in radians.

    Returns:
        The input angle converted to degrees.
    """

    return math.degrees(x)


def deg_to_rad(x: float) -> float:
    """Convert degrees to radians.

    Args:
        x: Angle expressed in degrees.

    Returns:
        The input angle converted to radians.
    """

    return math.radians(x)


def list_rad_to_deg(values: Iterable[float]) -> List[float]:
    """Convert an iterable of angles in radians to degrees.

    Args:
        values: Iterable of angles expressed in radians.

    Returns:
        A list containing each input angle converted to degrees.
    """

    return [math.degrees(v) for v in values]


def list_deg_to_rad(values: Iterable[float]) -> List[float]:
    """Convert an iterable of angles in degrees to radians.

    Args:
        values: Iterable of angles expressed in degrees.

    Returns:
        A list containing each input angle converted to radians.
    """

    return [math.radians(v) for v in values]


