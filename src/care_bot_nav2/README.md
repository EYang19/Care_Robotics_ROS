# CareBot Nav2

This package is specifically for running CareBot with Nav2 in simulation.

## Prerequisites

Make sure you have the following packages installed:
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-teleop-twist-keyboard
```

## Build the workspace

```bash
cd ~/care_robotics_ws
colcon build --symlink-install
source install/setup.bash
```

## Package Layout

- `launch/` contains the two supported workflows: Nav2 testing and teleop mapping.
- `config/nav2_params.yaml` configures Nav2 planners, controllers, costmaps, robot limits, and sensor topics.
- `config/slam_mapping_params.yaml` configures `slam_toolbox`.
- `worlds/`, `maps/`, and `rviz/` contain the simulation world, saved maps, and RViz view used by the launch files.

## Usage

### Nav Test

Launch everything in a single command:
```bash
ros2 launch care_bot_nav2 care_bot_nav_test.launch.py
```

This will:
1. Start Gazebo with the local hallway world
2. Spawn the care_bot robot
3. Launch Nav2 navigation stack
4. Open RViz with navigation visualization

**Important**: In RViz, you need to set the initial pose:
1. Click "2D Pose Estimate" button in RViz
2. Click on the map where your robot is located
3. Drag to set the orientation

Then you can set navigation goals:
1. Click "Nav2 Goal" button in RViz
2. Click on the map where you want the robot to go
3. Drag to set the goal orientation

### Teleop Mapping

Build a map by driving the robot manually:
```bash
ros2 launch care_bot_nav2 care_bot_teleop_mapping.launch.py
```

This starts Gazebo, spawns CareBot, runs `slam_toolbox`, opens RViz, starts keyboard teleop in one terminal, and opens a second terminal for saving the map.

Drive the robot around the room until:
- the map covers the whole area you care about,
- walls are not doubled or skewed,
- you have returned near previously visited areas so SLAM can close loops,
- the map in RViz looks stable.

Then press Enter in the `CareBot Save Map` terminal. By default this saves:
```text
~/care_robotics_ws/src/care_bot_nav2/maps/school_room.yaml
~/care_robotics_ws/src/care_bot_nav2/maps/school_room.pgm
```

You can choose a different save path:
```bash
ros2 launch care_bot_nav2 care_bot_teleop_mapping.launch.py map_save_path:=~/care_robotics_ws/src/care_bot_nav2/maps/my_school_map
```

If you do not want the helper terminals:
```bash
ros2 launch care_bot_nav2 care_bot_teleop_mapping.launch.py start_teleop:=false start_map_saver:=false
```

The old launch name still works as a wrapper:
```bash
ros2 launch care_bot_nav2 care_bot_hallway_teleop_mapping.launch.py
```

## Troubleshooting

### Robot doesn't move
- Check that the differential drive plugin is working: `ros2 topic echo /odom`
- Check cmd_vel is being published: `ros2 topic echo /cmd_vel`
- Verify the robot description is correct: `ros2 topic echo /robot_description`

### Localization issues
- Make sure you set the initial pose in RViz
- Check that scan data is being published: `ros2 topic echo /scan`
- Verify transforms: `ros2 run tf2_tools view_frames`

### Navigation fails
- Check costmaps in RViz - they should show obstacles
- Verify parameters in `config/nav2_params.yaml` match your robot dimensions
- Check that goals are reachable (not inside obstacles)

## Configuration Files

- `config/nav2_params.yaml` - Nav2 parameters tuned for CareBot
- `config/slam_mapping_params.yaml` - SLAM Toolbox mapping parameters
- Robot dimensions used:
  - Footprint: 1.14 m x 0.66 m
  - Local/global footprint padding: 0.05 m
  - Max linear velocity: 0.30 m/s
  - Max angular velocity: 0.85 rad/s

## Adjusting Parameters

To tune navigation performance, edit `config/nav2_params.yaml`:
- Increase/decrease `robot_radius` for obstacle avoidance sensitivity
- Adjust `max_vel_x` and `max_vel_theta` for speed limits
- Modify `inflation_radius` for safety margins around obstacles
- Change `xy_goal_tolerance` for goal precision
