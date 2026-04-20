#!/usr/bin/env python3
"""
WebSocket Handler - FastAPI server and WebSocket endpoint
"""

import asyncio
import json
import queue
from typing import Dict, Any, Optional, Callable, Set
from datetime import datetime

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel

import logging

logger = logging.getLogger(__name__)


class TaskCommand(BaseModel):
    """Task command model"""

    message_type: str
    task_id: str
    waypoints: list
    task_metadata: Optional[Dict[str, Any]] = None


class TaskCancel(BaseModel):
    """Task cancel model"""

    message_type: str
    task_id: str


# Global references
bridge_node = None
app = FastAPI(title="Robot ROS Bridge", version="0.1.0")

# Enable CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class WebSocketHandler:
    """Manages WebSocket connections and message broadcasting"""

    def __init__(self):
        self.active_connections: Set[WebSocket] = set()
        self._lock = asyncio.Lock()
        self.bridge_node = None

    def set_bridge_node(self, node):
        """Set reference to bridge node"""
        self.bridge_node = node

    async def broadcast(self, message: Dict[str, Any]) -> None:
        """
        Broadcast message to all connected clients

        Args:
            message: Dict to send as JSON
        """
        if not self.active_connections:
            return

        message_json = json.dumps(message)
        disconnected = set()

        for websocket in self.active_connections:
            try:
                await websocket.send_text(message_json)
            except Exception as e:
                logger.error(f"Error broadcasting to client: {e}")
                disconnected.add(websocket)

        # Clean up disconnected clients
        async with self._lock:
            for ws in disconnected:
                self.active_connections.discard(ws)

    async def add_connection(self, websocket: WebSocket) -> None:
        """Add a new WebSocket connection"""
        async with self._lock:
            self.active_connections.add(websocket)

    async def remove_connection(self, websocket: WebSocket) -> None:
        """Remove a WebSocket connection"""
        async with self._lock:
            self.active_connections.discard(websocket)

    def get_connection_count(self) -> int:
        """Get number of connected clients"""
        return len(self.active_connections)


# Global WebSocket handler
ws_handler = WebSocketHandler()


async def drain_outgoing_queue(node) -> None:
    """
    Drain outgoing message queue and broadcast to WebSocket clients

    Args:
        node: Bridge node instance
    """
    outgoing_queue = node.get_outgoing_queue()

    while True:
        try:
            # Non-blocking check with small timeout
            try:
                msg_type, msg_data = outgoing_queue.get(timeout=0.01)
                msg_data["message_type"] = msg_type

                # Broadcast to all connected clients
                await ws_handler.broadcast(msg_data)
            except queue.Empty:
                await asyncio.sleep(0.01)
                continue
        except Exception as e:
            logger.error(f"Error draining queue: {e}")
            await asyncio.sleep(0.1)


async def handle_incoming_message(data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Process incoming WebSocket message

    Args:
        data: Message dict

    Returns:
        Response dict
    """
    if not bridge_node:
        return {"status": "error", "message": "Bridge not initialized"}

    message_type = data.get("message_type")

    try:
        if message_type == "task_command":
            return await bridge_node.handle_task_command(data)
        elif message_type == "task_cancel":
            return await bridge_node.handle_task_cancel(data.get("task_id"))
        elif message_type == "emergency_stop":
            return await bridge_node.handle_emergency_stop()
        else:
            return {"status": "error", "message": f"Unknown message type: {message_type}"}
    except Exception as e:
        logger.error(f"Error handling message: {e}")
        return {"status": "error", "message": str(e)}


# REST Endpoints


@app.get("/")
async def root():
    """Root endpoint"""
    return {
        "name": "Robot ROS Bridge API",
        "version": "0.1.0",
        "endpoints": {
            "health": "/health",
            "status": "/status",
            "websocket": "/ws",
        },
    }


@app.get("/health")
async def health():
    """Health check"""
    return {
        "status": "ok",
        "timestamp": datetime.utcnow().isoformat(),
        "connected_clients": ws_handler.get_connection_count(),
    }


@app.get("/status")
async def status():
    """Get system status"""
    if not bridge_node:
        return {"status": "error", "message": "Bridge not initialized"}

    robot_state = bridge_node.task_state.get_robot_state()
    current_task = bridge_node.task_state.get_current_task()

    return {
        "status": "ok",
        "robot_state": robot_state,
        "current_task": current_task,
        "connected_clients": ws_handler.get_connection_count(),
        "timestamp": datetime.utcnow().isoformat(),
    }


# WebSocket Endpoint


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for real-time communication"""
    await websocket.accept()
    await ws_handler.add_connection(websocket)

    logger.info(f"WebSocket client connected. Total: {ws_handler.get_connection_count()}")

    try:
        # Send initial heartbeat
        await websocket.send_text(
            json.dumps({
                "message_type": "connected",
                "timestamp": datetime.utcnow().isoformat(),
            })
        )

        # Listen for incoming messages
        while True:
            try:
                data = await asyncio.wait_for(websocket.receive_text(), timeout=30.0)
                msg_dict = json.loads(data)

                # Handle incoming message
                response = await handle_incoming_message(msg_dict)
                await websocket.send_text(json.dumps(response))
            except asyncio.TimeoutError:
                # Send heartbeat on timeout
                await websocket.send_text(
                    json.dumps({
                        "message_type": "heartbeat",
                        "timestamp": datetime.utcnow().isoformat(),
                    })
                )
    except WebSocketDisconnect:
        logger.info("WebSocket client disconnected")
        await ws_handler.remove_connection(websocket)
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        await ws_handler.remove_connection(websocket)


def create_app():
    """Create and configure FastAPI app"""
    return app


def set_bridge_node(node):
    """Set bridge node reference"""
    global bridge_node
    bridge_node = node
    ws_handler.set_bridge_node(node)
