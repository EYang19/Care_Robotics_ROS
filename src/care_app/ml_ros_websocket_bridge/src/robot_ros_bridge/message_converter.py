#!/usr/bin/env python3
"""
Message Converter - Stateless converters between ROS messages and JSON
"""

import json
import math
from datetime import datetime
from typing import Dict, Any, List, Optional
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry


def quaternion_to_euler(quat: Quaternion) -> Dict[str, float]:
    """
    Convert quaternion to Euler angles (roll, pitch, yaw)

    Args:
        quat: geometry_msgs.msg.Quaternion

    Returns:
        Dict with keys: roll, pitch, yaw (in radians)
    """
    x, y, z, w = quat.x, quat.y, quat.z, quat.w

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    sinp = 1.0 if sinp > 1.0 else sinp
    sinp = -1.0 if sinp < -1.0 else sinp
    pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return {"roll": roll, "pitch": pitch, "yaw": yaw}


def odom_to_json(msg: Odometry) -> Dict[str, Any]:
    """
    Convert Odometry message to JSON format

    Args:
        msg: nav_msgs.msg.Odometry

    Returns:
        Dict with position, orientation, velocity
    """
    pose = msg.pose.pose
    twist = msg.twist.twist

    return {
        "position": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
        },
        "orientation_quat": {
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
            "w": float(pose.orientation.w),
        },
        "orientation_euler": quaternion_to_euler(pose.orientation),
        "velocity": {
            "linear_x": float(twist.linear.x),
            "angular_z": float(twist.angular.z),
        },
        "frame_id": msg.header.frame_id,
        "timestamp": datetime.fromtimestamp(
            msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        ).isoformat(),
    }


def robot_state_to_json(
    position: Dict[str, float],
    velocity: Dict[str, float],
    battery_level: float,
    current_capacity: int,
    max_capacity: int,
    active_task_id: Optional[str] = None,
) -> Dict[str, Any]:
    """
    Convert robot state to JSON format

    Args:
        position: Dict with x, y, z
        velocity: Dict with linear_x, linear_y, angular_z
        battery_level: 0.0-1.0
        current_capacity: Current load capacity
        max_capacity: Maximum capacity
        active_task_id: Current task ID (optional)

    Returns:
        Dict with robot state
    """
    return {
        "message_type": "robot_state",
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "position": position,
        "velocity": velocity,
        "battery_level": float(battery_level),
        "current_capacity": int(current_capacity),
        "max_capacity": int(max_capacity),
        "active_task_id": active_task_id,
    }


def task_status_to_json(
    task_id: str,
    status: str,
    progress_percent: Optional[float] = None,
    distance_remaining_m: Optional[float] = None,
    time_elapsed_s: Optional[float] = None,
    final_position: Optional[Dict[str, float]] = None,
    total_time_s: Optional[float] = None,
    error: Optional[str] = None,
    position_at_failure: Optional[Dict[str, float]] = None,
) -> Dict[str, Any]:
    """
    Convert task status to JSON format

    Args:
        task_id: Unique task ID
        status: 'in_progress', 'completed', 'failed', 'canceled'
        progress_percent: Progress 0-100 (for in_progress)
        distance_remaining_m: Meters remaining (for in_progress)
        time_elapsed_s: Seconds elapsed (for in_progress)
        final_position: Position on completion (for completed)
        total_time_s: Total time (for completed)
        error: Error message (for failed)
        position_at_failure: Position on failure (for failed)

    Returns:
        Dict with task status
    """
    result = {
        "message_type": "task_status",
        "task_id": task_id,
        "status": status,
        "timestamp": datetime.utcnow().isoformat() + "Z",
    }

    if status == "in_progress":
        result["progress_percent"] = float(progress_percent or 0)
        result["distance_remaining_m"] = float(distance_remaining_m or 0)
        result["time_elapsed_s"] = float(time_elapsed_s or 0)
    elif status == "completed":
        result["final_position"] = final_position
        result["total_time_s"] = float(total_time_s or 0)
    elif status == "failed":
        result["error"] = error or "Unknown error"
        result["position_at_failure"] = position_at_failure

    return result


def consumption_rates_to_json(
    location_consumption: List[Dict[str, Any]]
) -> Dict[str, Any]:
    """
    Convert consumption rates to JSON format for broadcast

    Args:
        location_consumption: List of consumption rate dicts

    Returns:
        Dict with consumption rates message
    """
    return {
        "message_type": "consumption_rates",
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "location_consumption": location_consumption,
    }


def location_inventory_to_json(
    location_updates: List[Dict[str, Any]]
) -> Dict[str, Any]:
    """
    Convert inventory data to JSON format for broadcast

    Args:
        location_updates: List of location inventory dicts

    Returns:
        Dict with location inventory message
    """
    return {
        "message_type": "location_inventory",
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "location_updates": location_updates,
    }


def system_state_to_json(
    robot_state: Dict[str, Any],
    location_inventories: List[Dict[str, Any]],
) -> Dict[str, Any]:
    """
    Convert full system state to JSON format

    Args:
        robot_state: Robot state dict
        location_inventories: List of location inventory dicts

    Returns:
        Dict with complete system state
    """
    return {
        "message_type": "system_state",
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "robot_state": robot_state,
        "location_inventories": location_inventories,
    }


def task_json_to_waypoints(task_json: Dict[str, Any]) -> List[Dict[str, float]]:
    """
    Extract and validate waypoints from task JSON

    Args:
        task_json: Task command JSON

    Returns:
        List of waypoint dicts with x, y, z, frame_id

    Raises:
        ValueError: If waypoints are invalid
    """
    if "waypoints" not in task_json:
        raise ValueError("Task must contain 'waypoints' field")

    waypoints = task_json["waypoints"]
    if not isinstance(waypoints, list) or len(waypoints) == 0:
        raise ValueError("Waypoints must be a non-empty list")

    validated = []
    for i, wp in enumerate(waypoints):
        if not isinstance(wp, dict):
            raise ValueError(f"Waypoint {i} is not a dict")
        if "x" not in wp or "y" not in wp:
            raise ValueError(f"Waypoint {i} missing x or y coordinate")

        validated.append({
            "x": float(wp["x"]),
            "y": float(wp["y"]),
            "z": float(wp.get("z", 0.0)),
            "frame_id": str(wp.get("frame_id", "map")),
        })

    return validated


def nav2_feedback_to_json(
    current_pose: Dict[str, float],
    distance_remaining: float,
    estimated_time_remaining: float,
    navigation_time: float,
    number_of_recoveries: int,
) -> Dict[str, Any]:
    """
    Convert Nav2 feedback to JSON format

    Args:
        current_pose: Current robot pose dict
        distance_remaining: Distance to goal in meters
        estimated_time_remaining: Time estimate in seconds
        navigation_time: Total navigation time in seconds
        number_of_recoveries: Number of recoveries attempted

    Returns:
        Dict with Nav2 feedback
    """
    return {
        "current_pose": current_pose,
        "distance_remaining": float(distance_remaining),
        "estimated_time_remaining": float(estimated_time_remaining),
        "navigation_time": float(navigation_time),
        "number_of_recoveries": int(number_of_recoveries),
    }
