# STM32 Drive Command Protocol

This package sends drive-wheel commands from ROS 2 to the STM32 over a serial
port. The STM32 does not receive ROS messages directly. It receives one plain
text string per command.

## ROS Input

The bridge node subscribes to:

```text
/cmd_vel
```

Message type:

```text
geometry_msgs/msg/Twist
```

Fields used:

```text
linear.x   forward velocity in meters/second
angular.z  yaw velocity in radians/second
```

All other `Twist` fields are ignored.

## Serial Output To STM32

Default output is an ASCII string:

```text
V <left_rad_s> <right_rad_s>\n
```

Example:

```text
V 1.000 1.000\n
```

Meaning:

```text
left_rad_s   left drive wheel angular velocity in radians/second
right_rad_s  right drive wheel angular velocity in radians/second
\n           newline terminator
```

The STM32 should read until newline, parse the leading `V`, then parse the two
floating-point values as left and right wheel velocity setpoints.

## Conversion From /cmd_vel

The ROS bridge converts robot velocity into differential-drive wheel velocity:

```text
left_rad_s  = (linear.x - angular.z * wheel_separation / 2) / wheel_radius
right_rad_s = (linear.x + angular.z * wheel_separation / 2) / wheel_radius
```

Current robot values:

```text
wheel_separation = 0.458 m
wheel_radius     = 0.060 m
wheel_diameter   = 0.120 m
```

## Examples

Forward command:

```text
/cmd_vel:
linear.x = 0.060
angular.z = 0.000
```

Serial output:

```text
V 1.000 1.000\n
```

Turn left in place:

```text
/cmd_vel:
linear.x = 0.000
angular.z = 0.500
```

Calculated wheel speeds:

```text
left_rad_s  = -1.908
right_rad_s =  1.908
```

Serial output:

```text
V -1.908 1.908\n
```

Stop command:

```text
V 0.000 0.000\n
```

## Timeout Behavior

If no `/cmd_vel` message is received for `0.5` seconds, the bridge sends:

```text
V 0.000 0.000\n
```

The STM32 should also implement its own serial timeout. If it stops receiving
valid `V` lines, it should stop both drive wheels.

## STM32 Parser Requirements

The STM32 firmware should:

```text
1. Read serial bytes until '\n'.
2. Confirm the line starts with 'V '.
3. Parse two signed numbers:
   - left wheel rad/s
   - right wheel rad/s
4. Use those values as motor velocity setpoints.
5. Stop the motors if commands stop arriving.
```

Recommended initial parser behavior:

```text
Valid line:   V 1.000 -1.000\n
Invalid line: anything that does not start with V or does not contain two numbers
```

## Run On Raspberry Pi

The bridge must run on the Raspberry Pi that is physically connected to the
STM32 serial port.

Check the STM32 serial device:

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

Run the bridge:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch care_hw_bridge cmd_vel_serial_bridge.launch.py serial_port:=/dev/ttyACM0
```

Bench test without opening the serial port:

```bash
ros2 run care_hw_bridge cmd_vel_serial_bridge --ros-args \
  -p open_serial:=false \
  -p log_packets:=true
```

Send a test ROS command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.06}, angular: {z: 0.0}}" -r 10
```

## Config

Default parameters are in:

```text
config/cmd_vel_serial_bridge.yaml
```

Relevant defaults:

```text
serial_port: /dev/ttyACM0
baudrate: 115200
protocol: ascii
send_rate_hz: 20.0
command_timeout_s: 0.5
```
