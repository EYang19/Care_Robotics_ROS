#!/usr/bin/env python3
"""
Nav2 Controller - Handles navigate_through_poses action client
"""

from typing import Dict, Any, Optional, List, Callable
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from action_msgs.msg import GoalStatus
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math


class Nav2Controller:
    """Wrapper around Nav2's navigate_through_poses action client"""

    def __init__(
        self,
        node: Node,
        global_frame: str = "map",
        action_name: str = "navigate_through_poses",
    ):
        """
        Initialize Nav2 controller

        Args:
            node: ROS2 node instance
            global_frame: Global frame ID
            action_name: Nav2 action name
        """
        self.node = node
        self.global_frame = global_frame
        self.action_name = action_name

        # Action client
        self.action_client = ActionClient(
            node,
            NavigateThroughPoses,
            action_name,
        )

        # Current goal handle
        self.current_goal_handle = None
        self.current_task_id: Optional[str] = None

        # Callbacks
        self._feedback_callback: Optional[Callable] = None
        self._result_callback: Optional[Callable] = None

        self.node.get_logger().info(f"Nav2Controller initialized with action: {action_name}")

    def set_callbacks(
        self,
        feedback_callback: Optional[Callable] = None,
        result_callback: Optional[Callable] = None,
    ) -> None:
        """
        Set feedback and result callbacks

        Args:
            feedback_callback: Callable(task_id, feedback_data)
            result_callback: Callable(task_id, status, result_data)
        """
        self._feedback_callback = feedback_callback
        self._result_callback = result_callback

    def waypoints_to_poses(
        self,
        waypoints: List[Dict[str, float]],
    ) -> List[PoseStamped]:
        """
        Convert waypoint list to PoseStamped messages

        Args:
            waypoints: List of dicts with x, y, z, and optional frame_id

        Returns:
            List of PoseStamped messages
        """
        poses = []

        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = wp.get("frame_id", self.global_frame)
            pose.header.stamp = self.node.get_clock().now().to_msg()

            # Position
            pose.pose.position.x = float(wp["x"])
            pose.pose.position.y = float(wp["y"])
            pose.pose.position.z = float(wp.get("z", 0.0))

            # Orientation (default: identity quaternion)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            poses.append(pose)

        return poses

    async def send_goal(
        self,
        task_id: str,
        waypoints: List[Dict[str, float]],
        behavior_tree: str = "",
    ) -> Dict[str, Any]:
        """
        Send navigation goal to Nav2

        Args:
            task_id: Unique task identifier
            waypoints: List of waypoint dicts
            behavior_tree: Optional behavior tree override

        Returns:
            Dict with status and details

        Raises:
            Exception: If action server unavailable or goal rejected
        """
        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("navigate_through_poses action server not available")
            raise Exception("Nav2 action server not available")

        # Convert waypoints to poses
        try:
            poses = self.waypoints_to_poses(waypoints)
            if not poses:
                raise ValueError("No valid waypoints provided")
        except Exception as e:
            self.node.get_logger().error(f"Failed to convert waypoints: {e}")
            raise

        # Create goal message
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        if behavior_tree:
            goal_msg.behavior_tree = behavior_tree

        # Send goal
        self.node.get_logger().info(
            f"Sending goal for task {task_id} with {len(poses)} waypoints"
        )

        try:
            send_goal_future = self.action_client.send_goal_async(
                goal_msg,
                feedback_callback=self._make_feedback_callback(task_id),
            )
            goal_handle = await send_goal_future
        except Exception as e:
            self.node.get_logger().error(f"Failed to send goal: {e}")
            raise

        if not goal_handle.accepted:
            self.node.get_logger().error(f"Goal rejected for task {task_id}")
            raise Exception(f"Goal rejected by Nav2")

        self.current_goal_handle = goal_handle
        self.current_task_id = task_id

        self.node.get_logger().info(f"Goal accepted for task {task_id}")

        # Set up result callback
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._make_result_callback(task_id))

        return {
            "status": "accepted",
            "task_id": task_id,
            "goal_id": goal_handle.goal_id.uuid.hex(),
            "num_waypoints": len(poses),
        }

    def _make_feedback_callback(self, task_id: str):
        """Create feedback callback for specific task"""

        def feedback_callback(feedback_msg):
            if self._feedback_callback:
                fb = feedback_msg.feedback
                feedback_data = {
                    "current_pose": {
                        "x": fb.current_pose.pose.position.x,
                        "y": fb.current_pose.pose.position.y,
                        "z": fb.current_pose.pose.position.z,
                    },
                    "distance_remaining": float(fb.distance_remaining),
                    "estimated_time_remaining": fb.estimated_time_remaining.sec + fb.estimated_time_remaining.nanosec / 1e9,
                    "number_of_recoveries": int(fb.number_of_recoveries),
                    "navigation_time": fb.navigation_time.sec + fb.navigation_time.nanosec / 1e9,
                }
                self._feedback_callback(task_id, feedback_data)

        return feedback_callback

    def _make_result_callback(self, task_id: str):
        """Create result callback for specific task"""

        def result_callback(future):
            result = future.result()
            status = result.status

            result_data = {
                "status_code": status,
            }

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.node.get_logger().info(f"Task {task_id} succeeded")
                result_data["status"] = "succeeded"
            elif status == GoalStatus.STATUS_ABORTED:
                self.node.get_logger().warn(f"Task {task_id} aborted")
                result_data["status"] = "aborted"
            elif status == GoalStatus.STATUS_CANCELED:
                self.node.get_logger().info(f"Task {task_id} canceled")
                result_data["status"] = "canceled"
            else:
                self.node.get_logger().error(f"Task {task_id} failed with status {status}")
                result_data["status"] = "failed"

            if self._result_callback:
                self._result_callback(task_id, result_data)

        return result_callback

    async def cancel_goal(self) -> Dict[str, Any]:
        """
        Cancel current navigation goal

        Returns:
            Dict with cancellation status

        Raises:
            Exception: If no active goal
        """
        if self.current_goal_handle is None:
            self.node.get_logger().warn("No active goal to cancel")
            raise Exception("No active goal")

        task_id = self.current_task_id
        self.node.get_logger().info(f"Canceling goal for task {task_id}")

        try:
            cancel_response = await self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
            self.current_task_id = None

            return {
                "status": "canceled",
                "task_id": task_id,
            }
        except Exception as e:
            self.node.get_logger().error(f"Failed to cancel goal: {e}")
            raise

    def get_current_task_id(self) -> Optional[str]:
        """Get current task ID"""
        return self.current_task_id

    def has_active_goal(self) -> bool:
        """Check if there's an active goal"""
        return self.current_goal_handle is not None
