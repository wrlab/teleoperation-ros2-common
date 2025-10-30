# Python usage

This package provides a set of light-weight helpers that simplify common ROS 2
teleoperation tasks.  The snippets below demonstrate typical usage patterns for
each module.

## Activity tracking

```python
from wxr_ros2_common_py import ActivityTracker

tracker = ActivityTracker(node, interval_s=0.5)
tracker.mark_sub('/camera/image_raw')
tracker.mark_pub('/camera/image_overlay')
```

The tracker reports active topics every 0.5 seconds using the node logger.

## Structured logging

```python
from wxr_ros2_common_py import log_startup, log_latency

log_startup(
    node,
    params={'auto_start': True},
    endpoints=[
        {'dir': 'pub', 'topic': '/cmd_vel', 'type': 'geometry_msgs/msg/Twist'},
        {'dir': 'sub', 'topic': '/joy', 'type': 'sensor_msgs/msg/Joy'},
    ],
)

with processing_context() as start:
    # ...
    end = node.get_clock().now().nanoseconds / 1e9
    log_latency(node, 'pipeline', start, end)
```

## Parameter utilities

```python
from wxr_ros2_common_py import get_bool_param, get_enum_param

enabled = get_bool_param(node, 'enable_streaming', default=True)
mode = get_enum_param(node, 'control_mode', {'auto', 'manual'}, 'auto')
```

## QoS formatting

```python
from rclpy.qos import QoSProfile
from wxr_ros2_common_py import qos_to_multiline

profile = QoSProfile(depth=10)
node.get_logger().info(f'QoS:{qos_to_multiline(profile)}')
```

## Time helpers

```python
from wxr_ros2_common_py import now_sec_nsec, now_sec_of_day, stamp_to_float

sec, nsec = now_sec_nsec(node)
sec_of_day, _ = now_sec_of_day(node)
timestamp = stamp_to_float(sec, nsec)
```

## Unit conversion helpers

```python
from wxr_ros2_common_py import list_deg_to_rad

angles_rad = list_deg_to_rad([0.0, 90.0, 180.0])
```

