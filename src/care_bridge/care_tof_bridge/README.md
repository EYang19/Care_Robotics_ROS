# STM32 ToF Serial Bridge

This package reads 4x4 ToF distance arrays from the STM32 over serial and
publishes ROS 2 `sensor_msgs/msg/PointCloud2` topics for Nav2.

## Important Serial Note

The STM32 is also used for drive commands. Two separate ROS processes should not
open the same serial device at the same time. For first testing, run either the
drive bridge or this ToF bridge. If the same STM32 must handle both drive and
ToF continuously, the next step is to combine both protocols into one serial
bridge node.

## STM32 Output Format

The STM32 should send one ASCII line per ToF frame:

```text
TOF <sensor_name> <d00> <d01> <d02> <d03> <d10> ... <d33>\n
```

Supported sensor names:

```text
front_left
front_center
front_right
```

Short aliases are also accepted:

```text
fl -> front_left
fc -> front_center
fr -> front_right
```

Distance values are expected in millimeters by default.

Example:

```text
TOF front_center 420 430 440 450 410 415 420 425 390 400 405 410 380 385 390 395\n
```

The 16 values are row-major order:

```text
d00 d01 d02 d03
d10 d11 d12 d13
d20 d21 d22 d23
d30 d31 d32 d33
```

## ROS Output

The bridge publishes:

```text
/tof_cloud/front_left
/tof_cloud/front_center
/tof_cloud/front_right
```

Message type:

```text
sensor_msgs/msg/PointCloud2
```

Frame IDs:

```text
front_left_tof
front_center_tof
front_right_tof
```

These match the existing URDF frames and the topics already configured in
`care_navigation/config/nav2_params.yaml`.

## Projection

The sensor field of view is configured as:

```text
fov_degrees: 65.0
```

The bridge treats the 4x4 sensor as a square 65 degree FOV. Each cell is
projected into a 3D point:

```text
x = range * cos(pitch) * cos(yaw)
y = range * cos(pitch) * sin(yaw)
z = range * sin(pitch)
```

Invalid or out-of-range values are published as NaN points. Default valid range:

```text
min_range_m: 0.05
max_range_m: 2.0
```

## Run

Build:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select care_tof_bridge
source install/setup.bash
```

Run on the Raspberry Pi connected to the STM32:

```bash
ros2 launch care_tof_bridge tof_serial_bridge.launch.py serial_port:=/dev/ttyACM0
```

Check output:

```bash
ros2 topic echo /tof_cloud/front_center --once
```

## Nav2

Nav2 already has obstacle sources configured for these point cloud topics:

```text
/tof_cloud/front_left
/tof_cloud/front_center
/tof_cloud/front_right
```

To use ToF data in Nav2, make sure these are running:

```text
robot_state_publisher  # publishes TF for front_*_tof frames
tof_serial_bridge      # publishes PointCloud2 topics
nav2                   # consumes the PointCloud2 topics in obstacle_layer
```
