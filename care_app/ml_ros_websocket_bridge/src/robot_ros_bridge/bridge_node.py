#!/usr/bin/env python3
"""
Bridge Node - Main ROS2 orchestrator
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String
import asyncio
import queue
import json
from typing import Dict, Any, Optional, Callable

from .task_state import TaskState
from .system_state import SystemState
from .nav2_controller import Nav2Controller
from .message_converter import (
    odom_to_json,
    task_status_to_json,
    robot_state_to_json,
    system_state_to_json,
    location_inventory_to_json,
)


class BridgeNode(Node):
    """Main ROS2 node for robot bridge"""

    def __init__(self):
        super().__init__("robot_ros_bridge")

        # Declare all parameters from config
        self.declare_parameter("websocket_host", "0.0.0.0")
        self.declare_parameter("websocket_port", 8765)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("sku_inventory_topic", "/sensor/sku_inventory")
        self.declare_parameter("battery_topic", "/sensor/battery")
        self.declare_parameter("estop_topic", "/emergency_stop")
        self.declare_parameter("nav2_action", "navigate_through_poses")
        self.declare_parameter("nav2_timeout", 300.0)
        self.declare_parameter("robot_state_publish_rate", 10.0)
        self.declare_parameter("system_state_publish_rate", 1.0)
        self.declare_parameter("feedback_poll_rate", 10.0)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("buffer_inventory_updates", True)
        self.declare_parameter("log_level", "INFO")
        self.declare_parameter("sensor_data_timeout", 30.0)
        self.declare_parameter("nav2_action_server_timeout", 5.0)

        # Get all parameters
        self.config = {
            "websocket_host": self.get_parameter("websocket_host").value,
            "websocket_port": self.get_parameter("websocket_port").value,
            "odom_topic": self.get_parameter("odom_topic").value,
            "sku_inventory_topic": self.get_parameter("sku_inventory_topic").value,
            "battery_topic": self.get_parameter("battery_topic").value,
            "estop_topic": self.get_parameter("estop_topic").value,
            "nav2_action": self.get_parameter("nav2_action").value,
            "nav2_timeout": self.get_parameter("nav2_timeout").value,
            "robot_state_publish_rate": self.get_parameter("robot_state_publish_rate").value,
            "system_state_publish_rate": self.get_parameter("system_state_publish_rate").value,
            "feedback_poll_rate": self.get_parameter("feedback_poll_rate").value,
            "global_frame": self.get_parameter("global_frame").value,
            "base_frame": self.get_parameter("base_frame").value,
            "buffer_inventory_updates": self.get_parameter("buffer_inventory_updates").value,
            "log_level": self.get_parameter("log_level").value,
            "sensor_data_timeout": self.get_parameter("sensor_data_timeout").value,
            "nav2_action_server_timeout": self.get_parameter("nav2_action_server_timeout").value,
        }

        # State management
        self.task_state = TaskState()
        self.system_state = SystemState()
        self.system_state.set_buffering(
            self.config["buffer_inventory_updates"],
        )

        # Message queue for thread-safe communication
        self.outgoing_queue: queue.Queue = queue.Queue(maxsize=1000)

        # Nav2 controller
        self.nav2_controller = Nav2Controller(
            self,
            global_frame=self.config["global_frame"],
            action_name=self.config["nav2_action"],
        )
        self.nav2_controller.set_callbacks(
            feedback_callback=self._on_nav2_feedback,
            result_callback=self._on_nav2_result,
        )

        # WebSocket command handler callback
        self._websocket_command_handler: Optional[Callable] = None

        # Create subscriptions with QoS
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1)

        self.odom_sub = self.create_subscription(
            Odometry,
            self.config["odom_topic"],
            self._on_odometry,
            qos,
        )

        self.inventory_sub = self.create_subscription(
            String,
            self.config["sku_inventory_topic"],
            self._on_sku_inventory,
            qos,
        )

        self.battery_sub = self.create_subscription(
            Float32,
            self.config["battery_topic"],
            self._on_battery,
            qos,
        )

        # Emergency stop uses RELIABLE QoS — safety signals must not be dropped
        self.estop_sub = self.create_subscription(
            Bool,
            self.config["estop_topic"],
            self._on_emergency_stop,
            reliable_qos,
        )

        # Create timers for periodic publishing
        robot_publish_period = 1.0 / self.config["robot_state_publish_rate"]
        system_publish_period = 1.0 / self.config["system_state_publish_rate"]

        self.robot_state_timer = self.create_timer(
            robot_publish_period,
            self._publish_robot_state,
        )

        self.system_state_timer = self.create_timer(
            system_publish_period,
            self._publish_system_state,
        )

        self.get_logger().info(
            f"BridgeNode initialized with config: {json.dumps(self.config, indent=2)}"
        )

    def set_websocket_command_handler(self, handler: Callable) -> None:
        """
        Set the WebSocket command handler

        Args:
            handler: Callable(message_dict)
        """
        self._websocket_command_handler = handler

    def _on_odometry(self, msg: Odometry) -> None:
        """Callback for odometry updates"""
        odom_dict = odom_to_json(msg)

        # Update task state
        self.task_state.update_from_odom(
            position=odom_dict["position"],
            orientation=odom_dict["orientation_quat"],
            velocity=odom_dict["velocity"],
            frame_id=msg.header.frame_id,
        )

        # Queue robot state message
        robot_state = self.task_state.get_robot_state()
        self.outgoing_queue.put(("robot_state", robot_state))

    def _on_sku_inventory(self, msg: String) -> None:
        """Callback for SKU inventory"""
        try:
            data = json.loads(msg.data)

            if isinstance(data, dict):
                # Single location
                location_id = data.get("location_id")
                if location_id:
                    self.system_state.update_location_inventory(
                        location_id=location_id,
                        location_name=data.get("location_name", location_id),
                        sku_inventory=data.get("sku_inventory", {}),
                        category_inventory=data.get("category_inventory", {}),
                    )
            elif isinstance(data, list):
                # Multiple locations
                for item in data:
                    location_id = item.get("location_id")
                    if location_id:
                        self.system_state.update_location_inventory(
                            location_id=location_id,
                            location_name=item.get("location_name", location_id),
                            sku_inventory=item.get("sku_inventory", {}),
                            category_inventory=item.get("category_inventory", {}),
                        )

            if not self.config["buffer_inventory_updates"]:
                # Immediate publish
                self.outgoing_queue.put(
                    ("location_inventory", location_inventory_to_json(self.system_state.get_all_location_inventories()))
                )
        except Exception as e:
            self.get_logger().error(f"Error processing inventory: {e}")

    def _on_battery(self, msg: Float32) -> None:
        """Callback for battery level"""
        self.task_state.update_battery(msg.data)

    def _on_emergency_stop(self, msg: Bool) -> None:
        """Callback for hardware emergency stop signal"""
        if msg.data:
            self.get_logger().error("Emergency stop active — cancelling navigation")

            current_task = self.task_state.get_current_task()
            task_id = current_task.get("task_id")

            asyncio.run_coroutine_threadsafe(
                self.nav2_controller.cancel_goal(),
                asyncio.get_event_loop(),
            )
            self.task_state.clear_current_task()

            robot_state = self.task_state.get_robot_state()
            self.outgoing_queue.put((
                "emergency_stop",
                {
                    "active": True,
                    "reason": "hardware_triggered",
                    "task_id": task_id,
                    "position": robot_state.get("position"),
                },
            ))
        else:
            self.get_logger().info("Emergency stop cleared")
            self.outgoing_queue.put((
                "emergency_stop",
                {
                    "active": False,
                },
            ))

    def _publish_robot_state(self) -> None:
        """Timer callback to publish robot state"""
        robot_state = self.task_state.get_robot_state()
        self.outgoing_queue.put(("robot_state", robot_state))

    def _publish_system_state(self) -> None:
        """Timer callback to publish full system state"""
        robot_state = self.task_state.get_robot_state()
        inventories = self.system_state.get_all_location_inventories()

        system_state = system_state_to_json(robot_state, inventories)
        self.outgoing_queue.put(("system_state", system_state))

    def _on_nav2_feedback(self, task_id: str, feedback_data: Dict[str, Any]) -> None:
        """Callback for Nav2 feedback"""
        # Calculate progress
        distance_remaining = feedback_data.get("distance_remaining", 0.0)
        progress_percent = max(0, 100 - (distance_remaining * 10))  # Rough estimate

        status = task_status_to_json(
            task_id=task_id,
            status="in_progress",
            progress_percent=progress_percent,
            distance_remaining_m=distance_remaining,
            time_elapsed_s=feedback_data.get("navigation_time", 0.0),
        )
        self.outgoing_queue.put(("task_status", status))

    def _on_nav2_result(self, task_id: str, result_data: Dict[str, Any]) -> None:
        """Callback for Nav2 result"""
        status_str = result_data.get("status", "failed")

        if status_str == "succeeded":
            robot_state = self.task_state.get_robot_state()
            status = task_status_to_json(
                task_id=task_id,
                status="completed",
                final_position=robot_state["position"],
                total_time_s=0.0,
            )
        else:
            robot_state = self.task_state.get_robot_state()
            status = task_status_to_json(
                task_id=task_id,
                status="failed" if status_str != "canceled" else "canceled",
                error=f"Navigation {status_str}",
                position_at_failure=robot_state["position"],
            )

        self.outgoing_queue.put(("task_status", status))
        self.task_state.clear_current_task()

    async def handle_task_command(self, task_json: Dict[str, Any]) -> Dict[str, Any]:
        """
        Handle task command from WebSocket

        Args:
            task_json: Task command dict

        Returns:
            Response dict
        """
        try:
            task_id = task_json.get("task_id")
            if not task_id:
                return {"status": "error", "message": "Missing task_id"}

            waypoints = task_json.get("waypoints", [])
            if not waypoints:
                return {"status": "error", "message": "Missing waypoints"}

            metadata = task_json.get("task_metadata", {})

            # Update task state
            self.task_state.set_current_task(task_id, waypoints, metadata)

            # Send to Nav2
            result = await self.nav2_controller.send_goal(task_id, waypoints)

            # Queue task status update
            status = task_status_to_json(
                task_id=task_id,
                status="in_progress",
                progress_percent=0.0,
                distance_remaining_m=0.0,
                time_elapsed_s=0.0,
            )
            self.outgoing_queue.put(("task_status", status))

            return {"status": "accepted", "task_id": task_id}
        except Exception as e:
            self.get_logger().error(f"Error handling task command: {e}")
            return {"status": "error", "message": str(e)}

    async def handle_emergency_stop(self) -> Dict[str, Any]:
        """Handle emergency stop command received from CareRobotics via WebSocket"""
        try:
            self.get_logger().error("Emergency stop received from CareRobotics — cancelling navigation")

            current_task = self.task_state.get_current_task()
            task_id = current_task.get("task_id")

            await self.nav2_controller.cancel_goal()
            self.task_state.clear_current_task()

            robot_state = self.task_state.get_robot_state()
            self.outgoing_queue.put((
                "emergency_stop",
                {
                    "active": True,
                    "reason": "operator_triggered",
                    "task_id": task_id,
                    "position": robot_state.get("position"),
                },
            ))

            return {"status": "ok", "message": "Emergency stop executed"}
        except Exception as e:
            self.get_logger().error(f"Error handling emergency stop: {e}")
            return {"status": "error", "message": str(e)}

    async def handle_task_cancel(self, task_id: str) -> Dict[str, Any]:
        """
        Handle task cancellation

        Args:
            task_id: Task ID to cancel

        Returns:
            Response dict
        """
        try:
            current_task = self.task_state.get_current_task()
            if current_task["task_id"] != task_id:
                return {"status": "error", "message": f"Task {task_id} is not active"}

            await self.nav2_controller.cancel_goal()
            self.task_state.clear_current_task()

            # Queue task status
            status = task_status_to_json(
                task_id=task_id,
                status="canceled",
            )
            self.outgoing_queue.put(("task_status", status))

            return {"status": "canceled", "task_id": task_id}
        except Exception as e:
            self.get_logger().error(f"Error canceling task: {e}")
            return {"status": "error", "message": str(e)}

    def get_outgoing_queue(self) -> queue.Queue:
        """Get the outgoing message queue"""
        return self.outgoing_queue
