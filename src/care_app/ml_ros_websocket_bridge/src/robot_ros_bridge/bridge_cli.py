#!/usr/bin/env python3
"""Command line client for the Care robot WebSocket bridge."""

import argparse
import asyncio
import json
import sys
from datetime import datetime

try:
    import websockets
except ImportError:
    websockets = None


def build_command(args):
    task_id = args.task_id or f"{args.command}_{datetime.utcnow().strftime('%Y%m%d_%H%M%S')}"

    if args.command == "home":
        command = {
            "message_type": "go_home",
            "task_id": task_id,
        }
    elif args.command == "delivery":
        command = {
            "message_type": "go_delivery_station_1",
            "task_id": task_id,
        }
    elif args.command == "dock":
        command = {
            "message_type": "dock_command",
            "task_id": task_id,
            "dock_id": args.dock_id,
        }
    elif args.command == "estop":
        return {"message_type": "emergency_stop"}
    else:
        raise ValueError(f"Unsupported command: {args.command}")

    if args.tag_id is not None:
        command["tag_id"] = args.tag_id
    if args.no_staging:
        command["navigate_to_staging_pose"] = False

    return command


async def send_command(uri: str, command: dict, listen: bool) -> int:
    if websockets is None:
        print("websockets is not installed. Install python3-websockets or pip install websockets.", file=sys.stderr)
        return 1

    async with websockets.connect(uri) as ws:
        initial = await ws.recv()
        print(initial)

        await ws.send(json.dumps(command))
        response = await ws.recv()
        print(response)

        if not listen:
            return 0

        while True:
            print(await ws.recv())


def main() -> int:
    parser = argparse.ArgumentParser(description="Send commands to ml_ros_websocket_bridge")
    parser.add_argument(
        "--uri",
        default="ws://localhost:8765/ws",
        help="Bridge WebSocket URI",
    )
    parser.add_argument(
        "--task-id",
        default=None,
        help="Optional task ID to send with the command",
    )
    parser.add_argument(
        "--tag-id",
        type=int,
        default=None,
        help="Override target AprilTag ID for this docking command",
    )
    parser.add_argument(
        "--no-staging",
        action="store_true",
        help="Ask docking server to skip navigation to staging pose",
    )
    parser.add_argument(
        "--listen",
        action="store_true",
        help="Keep printing bridge status messages after the command is accepted",
    )

    subcommands = parser.add_subparsers(dest="command", required=True)
    subcommands.add_parser("home", help="Dock at home_station")
    subcommands.add_parser("delivery", help="Dock at delivery_station_1")
    subcommands.add_parser("estop", help="Emergency stop")
    dock_parser = subcommands.add_parser("dock", help="Dock at a specific dock ID")
    dock_parser.add_argument("dock_id", help="Dock ID from docks.yaml")

    args = parser.parse_args()
    command = build_command(args)
    return asyncio.run(send_command(args.uri, command, args.listen))


if __name__ == "__main__":
    raise SystemExit(main())
