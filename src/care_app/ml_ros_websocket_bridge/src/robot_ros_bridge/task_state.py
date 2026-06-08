#!/usr/bin/env python3
"""
Task & Robot State Management - Thread-safe state holder
"""

import threading
from typing import Dict, Any, Optional, List
from datetime import datetime


class TaskState:
    """Thread-safe storage for current task and robot state"""

    def __init__(self):
        self._lock = threading.Lock()

        # Task state
        self._current_task_id: Optional[str] = None
        self._current_waypoints: List[Dict[str, float]] = []
        self._task_metadata: Dict[str, Any] = {}

        # Robot position/velocity (from odometry)
        self._position: Dict[str, float] = {"x": 0.0, "y": 0.0, "z": 0.0}
        self._orientation: Dict[str, float] = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "w": 1.0,
        }
        self._velocity: Dict[str, float] = {
            "linear_x": 0.0,
            "angular_z": 0.0,
        }

        # Robot state
        self._battery_level: float = 1.0
        self._current_capacity: int = 0
        self._max_capacity: int = 12
        self._frame_id: str = "map"
        self._last_update: Optional[datetime] = None

    def set_current_task(
        self,
        task_id: str,
        waypoints: List[Dict[str, float]],
        metadata: Optional[Dict[str, Any]] = None,
    ) -> None:
        """
        Set the current task

        Args:
            task_id: Unique task ID
            waypoints: List of waypoint dicts
            metadata: Optional task metadata
        """
        with self._lock:
            self._current_task_id = task_id
            self._current_waypoints = waypoints.copy()
            self._task_metadata = metadata.copy() if metadata else {}

    def clear_current_task(self) -> None:
        """Clear the current task"""
        with self._lock:
            self._current_task_id = None
            self._current_waypoints = []
            self._task_metadata = {}

    def update_from_odom(
        self,
        position: Dict[str, float],
        orientation: Dict[str, float],
        velocity: Dict[str, float],
        frame_id: str = "map",
    ) -> None:
        """
        Update robot state from odometry message

        Args:
            position: Dict with x, y, z
            orientation: Dict with x, y, z, w (quaternion)
            velocity: Dict with linear_x, linear_y, linear_z, angular_x, angular_y, angular_z
            frame_id: Frame ID (default: "map")
        """
        with self._lock:
            self._position = position.copy()
            self._orientation = orientation.copy()
            self._velocity = velocity.copy()
            self._frame_id = frame_id
            self._last_update = datetime.utcnow()

    def update_battery(self, level: float) -> None:
        """
        Update battery level

        Args:
            level: Battery level 0.0-1.0
        """
        with self._lock:
            self._battery_level = max(0.0, min(1.0, float(level)))

    def update_capacity(self, current: int, max_val: int) -> None:
        """
        Update robot capacity

        Args:
            current: Current load
            max_val: Maximum capacity
        """
        with self._lock:
            self._current_capacity = int(current)
            self._max_capacity = int(max_val)

    def get_robot_state(self) -> Dict[str, Any]:
        """
        Get current robot state

        Returns:
            Dict with position, orientation, velocity, battery, capacity
        """
        with self._lock:
            return {
                "position": self._position.copy(),
                "orientation_quat": self._orientation.copy(),
                "velocity": self._velocity.copy(),
                "battery_level": self._battery_level,
                "current_capacity": self._current_capacity,
                "max_capacity": self._max_capacity,
                "frame_id": self._frame_id,
                "active_task_id": self._current_task_id,
                "timestamp": (
                    self._last_update.isoformat()
                    if self._last_update
                    else datetime.utcnow().isoformat()
                ),
            }

    def get_current_task(self) -> Dict[str, Any]:
        """
        Get current task

        Returns:
            Dict with task_id, waypoints, metadata
        """
        with self._lock:
            return {
                "task_id": self._current_task_id,
                "waypoints": self._current_waypoints.copy(),
                "metadata": self._task_metadata.copy(),
            }

    def get_position(self) -> Dict[str, float]:
        """Get current position"""
        with self._lock:
            return self._position.copy()

    def get_orientation(self) -> Dict[str, float]:
        """Get current orientation (quaternion)"""
        with self._lock:
            return self._orientation.copy()

    def get_battery_level(self) -> float:
        """Get current battery level"""
        with self._lock:
            return self._battery_level

    def has_active_task(self) -> bool:
        """Check if there's an active task"""
        with self._lock:
            return self._current_task_id is not None

    def get_active_task_id(self) -> Optional[str]:
        """Get active task ID"""
        with self._lock:
            return self._current_task_id
