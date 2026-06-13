#!/usr/bin/env python3
"""
Docking Controller - Handles Nav2 / OpenNav docking action client.
"""

from typing import Any, Callable, Dict, Optional

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from rclpy.node import Node

try:
    from nav2_msgs.action import DockRobot
except ImportError:
    try:
        from opennav_docking_msgs.action import DockRobot
    except ImportError:
        DockRobot = None


class DockingController:
    """Wrapper around the DockRobot action client."""

    def __init__(
        self,
        node: Node,
        action_name: str = "/dock_robot",
        action_server_timeout: float = 5.0,
    ):
        self.node = node
        self.action_name = action_name
        self.action_server_timeout = action_server_timeout
        self.action_client = None
        self.current_goal_handle = None
        self.current_task_id: Optional[str] = None
        self.current_dock_id: Optional[str] = None
        self._feedback_callback: Optional[Callable] = None
        self._result_callback: Optional[Callable] = None

        if DockRobot is None:
            self.node.get_logger().warn(
                "DockRobot action type is unavailable. Install nav2 docking or "
                "opennav_docking_msgs to enable station docking."
            )
            return

        self.action_client = ActionClient(node, DockRobot, action_name)
        self.node.get_logger().info(f"DockingController initialized with action: {action_name}")

    def set_callbacks(
        self,
        feedback_callback: Optional[Callable] = None,
        result_callback: Optional[Callable] = None,
    ) -> None:
        self._feedback_callback = feedback_callback
        self._result_callback = result_callback

    @staticmethod
    def _set_goal_duration(goal_msg, field_name: str, seconds: float) -> None:
        if not hasattr(goal_msg, field_name):
            return

        current_value = getattr(goal_msg, field_name)
        if hasattr(current_value, "sec") and hasattr(current_value, "nanosec"):
            duration = Duration()
            duration.sec = int(seconds)
            duration.nanosec = int((seconds - int(seconds)) * 1e9)
            setattr(goal_msg, field_name, duration)
        else:
            setattr(goal_msg, field_name, float(seconds))

    async def send_dock_goal(
        self,
        task_id: str,
        dock_id: str,
        navigate_to_staging_pose: bool = True,
        max_staging_time: float = 60.0,
    ) -> Dict[str, Any]:
        if DockRobot is None or self.action_client is None:
            raise Exception(
                "DockRobot action type is not available. Install/configure Nav2 docking first."
            )

        if not self.action_client.wait_for_server(timeout_sec=self.action_server_timeout):
            raise Exception(f"Docking action server not available: {self.action_name}")

        goal_msg = DockRobot.Goal()

        if hasattr(goal_msg, "use_dock_id"):
            goal_msg.use_dock_id = True
        if hasattr(goal_msg, "dock_id"):
            goal_msg.dock_id = dock_id
        if hasattr(goal_msg, "navigate_to_staging_pose"):
            goal_msg.navigate_to_staging_pose = bool(navigate_to_staging_pose)
        self._set_goal_duration(goal_msg, "max_staging_time", max_staging_time)

        self.node.get_logger().info(
            f"Sending docking goal for task {task_id} to dock {dock_id}"
        )

        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._make_feedback_callback(task_id, dock_id),
        )
        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            raise Exception(f"Docking goal rejected for dock {dock_id}")

        self.current_goal_handle = goal_handle
        self.current_task_id = task_id
        self.current_dock_id = dock_id

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._make_result_callback(task_id, dock_id))

        return {
            "status": "accepted",
            "task_id": task_id,
            "dock_id": dock_id,
            "goal_id": goal_handle.goal_id.uuid.hex(),
        }

    def _make_feedback_callback(self, task_id: str, dock_id: str):
        def feedback_callback(feedback_msg):
            if self._feedback_callback is None:
                return

            feedback = feedback_msg.feedback
            feedback_data = {"dock_id": dock_id}

            for field_name in (
                "state",
                "distance_remaining",
                "num_retries",
                "navigation_time",
            ):
                if hasattr(feedback, field_name):
                    feedback_data[field_name] = getattr(feedback, field_name)

            self._feedback_callback(task_id, feedback_data)

        return feedback_callback

    def _make_result_callback(self, task_id: str, dock_id: str):
        def result_callback(future):
            result = future.result()
            status = result.status
            result_data = {
                "dock_id": dock_id,
                "status_code": status,
            }

            if status == GoalStatus.STATUS_SUCCEEDED:
                result_data["status"] = "succeeded"
            elif status == GoalStatus.STATUS_CANCELED:
                result_data["status"] = "canceled"
            elif status == GoalStatus.STATUS_ABORTED:
                result_data["status"] = "aborted"
            else:
                result_data["status"] = "failed"

            if self._result_callback:
                self._result_callback(task_id, result_data)

            self.current_goal_handle = None
            self.current_task_id = None
            self.current_dock_id = None

        return result_callback

    async def cancel_goal(self) -> Dict[str, Any]:
        if self.current_goal_handle is None:
            raise Exception("No active docking goal")

        task_id = self.current_task_id
        dock_id = self.current_dock_id
        await self.current_goal_handle.cancel_goal_async()
        self.current_goal_handle = None
        self.current_task_id = None
        self.current_dock_id = None

        return {
            "status": "canceled",
            "task_id": task_id,
            "dock_id": dock_id,
        }

    def has_active_goal(self) -> bool:
        return self.current_goal_handle is not None
