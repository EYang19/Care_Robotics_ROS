# CareRobotics Project Structure

## src/

### 1. `care_interfaces/` (Core Infrastructure)
- `msg/`, `srv/`, `action/`: Team-wide shared custom communication interfaces. Isolated into a standalone package to avoid circular dependencies.

### 2. `care_bridge/` (Bridge Layer)
- `care_micro_ros/`: Responsible for starting `micro_ros_agent` and handling USB-CDC serial communication.
- `care_hw_bridge/`: Preliminary conversion of lower-level hardware data into standard ROS 2 topics.

### 3. `care_perception/` (Perception Layer)
- `care_sensor_fusion/`: EKF fusion (processing BNO055 quaternions + Odometry).
- `care_tof_process/`: Conversion of ToF 8x8 array data into `PointCloud2`.
- `care_vision/`: OV9281 camera driver and AprilTag visual localization/re-localization.

### 4. `care_localization/` (Mapping & Localization Layer)
- `care_slam/`: Configuration and launch for `slam_toolbox`.
- `care_amcl/`: `nav2_amcl` configuration and compensation nodes for visual tag-based drift correction in long corridors.

### 5. `care_navigation/` (Navigation Layer)
- `care_nav2_bringup/`: Core Nav2 configuration (includes parameter files for Hybrid Costmap settings).
- `care_docking/`: `opennav_docking` logic using visual feedback and blind-push docking.

### 6. `care_app/` (App & Behavior Layer)
- `care_behavior/`: High-level business logic managed via `BehaviorTree.CPP`.

### 7. `care_bringup/` (Top-level Launch Package)
- `launch/`: Global one-click launch files or layer-specific launch sequences.
- `config/`: Global configurations (e.g., `CycloneDDS.xml`).
- `rviz/`: RViz2 configuration files for remote monitoring (e.g., via Windows App).
