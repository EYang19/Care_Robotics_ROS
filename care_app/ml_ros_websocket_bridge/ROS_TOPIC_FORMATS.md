# ROS Topic Formats — Care Robot Bridge Reference

## `/odom` — `nav_msgs/Odometry`

Published by Gazebo's differential drive plugin. Represents the robot's estimated position and velocity based on wheel encoder data.

**Useful fields:**

| Field | Type | Description |
|---|---|---|
| `header.stamp.sec` | int | Seconds component of timestamp |
| `header.stamp.nanosec` | int | Nanoseconds remainder of timestamp |
| `pose.pose.position.x` | float64 | Position in metres, forward axis |
| `pose.pose.position.y` | float64 | Position in metres, left axis |
| `pose.pose.orientation.x/y/z/w` | float64 | Orientation as quaternion — convert z/w to yaw |
| `twist.twist.linear.x` | float64 | Forward velocity in m/s |
| `twist.twist.angular.z` | float64 | Rotational velocity in rad/s |

**Ignore:** `position.z`, `twist.linear.y/z`, `twist.angular.x/y`, both `covariance` arrays — not meaningful for a ground robot.

**Full structure:**
```
header:
  stamp: {sec: int, nanosec: int}
  frame_id: "odom"
child_frame_id: "base_footprint"
pose:
  pose:
    position:    {x: float, y: float, z: ~0}
    orientation: {x: float, y: float, z: float, w: float}
  covariance: [float64 x 36]
twist:
  twist:
    linear:  {x: float, y: 0.0, z: 0.0}
    angular: {x: 0.0,   y: 0.0, z: float}
  covariance: [float64 x 36]
```

**Converting quaternion to yaw (Python):**
```python
import math

def quat_to_yaw(orientation):
    q = orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)  # radians
```

---

## `/cmd_vel` — `geometry_msgs/Twist`

Published by Nav2's velocity smoother. Commands sent to Gazebo to drive the robot. No header or timestamp.

**Useful fields:**

| Field | Type | Description | Limits |
|---|---|---|---|
| `linear.x` | float64 | Forward/backward velocity in m/s | ±0.26 m/s |
| `angular.z` | float64 | Rotational velocity in rad/s | ±1.0 rad/s |

**Ignore:** `linear.y`, `linear.z`, `angular.x`, `angular.y` — always 0.0 for a differential drive robot.

**Full structure:**
```
linear:  {x: float, y: 0.0, z: 0.0}
angular: {x: 0.0,   y: 0.0, z: float}
```

---

## Notes for the Bridge

- Both topics run at up to **20 Hz** (set by `controller_frequency` and `smoothing_frequency` in `care_bot_nav2_params.yaml`)
- `/odom` is published even when the robot is stationary
- `/cmd_vel` is only published when Nav2 is actively navigating — it goes silent when the robot reaches its goal
- Timestamps use ROS sim time (`use_sim_time: true`) — wall clock time will differ from `header.stamp`
