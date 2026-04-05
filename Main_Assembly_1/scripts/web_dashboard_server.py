#!/usr/bin/env python3
"""
Lightweight ROS-backed web dashboard server for the cinematic ball pickup UI.

Serves the static dashboard assets and exposes a small JSON API:
  GET  /api/state
  POST /api/command   {"action": "pick_place", "source": "F4", "destination": "L0"}
  POST /api/command   {"action": "cancel"}
"""

import json
import mimetypes
import os
import threading
import time
from functools import partial
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Int32, String


class DashboardBridge(Node):
    def __init__(self):
        super().__init__("web_dashboard_server")

        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8765)

        self._host = str(self.get_parameter("host").value)
        self._port = int(self.get_parameter("port").value)

        self._lock = threading.Lock()
        self._state = {
            "planner_status": "IDLE — waiting for command",
            "planner_state": "IDLE",
            "arduino_status": "",
            "ball_status": "",
            "suction_on": False,
            "ball_detected": False,
            "pressure_raw": None,
            "ik_target": {"x": 0.0, "y": 0.25},
            "route": {"source": "F4", "destination": "L0"},
            "updated_at": time.time(),
        }

        self._cmd_pub = self.create_publisher(String, "/pick_place_cmd", 10)
        self._cancel_pub = self.create_publisher(Empty, "/cancel_plan", 10)

        self.create_subscription(String, "/planner_status", self._cb_planner_status, 10)
        self.create_subscription(String, "/pick_place_cmd", self._cb_pick_place_cmd, 10)
        self.create_subscription(String, "/arduino_status", self._cb_arduino_status, 10)
        self.create_subscription(String, "/ball_status", self._cb_ball_status, 10)
        self.create_subscription(Point, "/ik_target", self._cb_ik_target, 10)
        self.create_subscription(Bool, "/suction_cmd", self._cb_suction_cmd, 10)
        self.create_subscription(Bool, "/ball_detected", self._cb_ball_detected, 10)
        self.create_subscription(Int32, "/pressure_raw", self._cb_pressure_raw, 10)

        self._web_root = self._resolve_web_root()

        handler = partial(DashboardRequestHandler, bridge=self)
        self._server = ThreadingHTTPServer((self._host, self._port), handler)
        self._server.daemon_threads = True
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()

        self.get_logger().info(
            f"Web dashboard available at http://{self._host}:{self._port} "
            f"(assets: {self._web_root})"
        )

    def _resolve_web_root(self):
        candidates = []

        try:
            share_dir = get_package_share_directory("Main_Assembly_1")
            candidates.append(os.path.join(share_dir, "web", "ball-pickup-ui"))
        except Exception:
            pass

        script_dir = os.path.dirname(os.path.realpath(__file__))
        candidates.append(os.path.join(script_dir, "..", "web", "ball-pickup-ui"))

        for candidate in candidates:
            index_file = os.path.join(candidate, "index.html")
            if os.path.isfile(index_file):
                return os.path.abspath(candidate)

        raise FileNotFoundError(
            "Unable to locate dashboard web assets. Checked: " + ", ".join(candidates)
        )

    def _update_state(self, **kwargs):
        with self._lock:
            self._state.update(kwargs)
            self._state["updated_at"] = time.time()

    def snapshot(self):
        with self._lock:
            return json.loads(json.dumps(self._state))

    def web_root(self):
        return self._web_root

    def handle_command(self, payload):
        action = payload.get("action")
        if action == "pick_place":
            source = str(payload.get("source", "")).upper().strip()
            destination = str(payload.get("destination", "")).upper().strip()
            if not source or not destination:
                return {"ok": False, "error": "source and destination are required"}, HTTPStatus.BAD_REQUEST
            msg = String()
            msg.data = f"{source} {destination}"
            self._cmd_pub.publish(msg)
            self._update_state(route={"source": source, "destination": destination})
            return {"ok": True, "route": {"source": source, "destination": destination}}, HTTPStatus.OK

        if action == "cancel":
            self._cancel_pub.publish(Empty())
            return {"ok": True}, HTTPStatus.OK

        return {"ok": False, "error": f"unsupported action: {action}"}, HTTPStatus.BAD_REQUEST

    def _cb_planner_status(self, msg):
        text = msg.data
        planner_state = text.split()[0] if text else "IDLE"
        self._update_state(planner_status=text, planner_state=planner_state)

    def _cb_pick_place_cmd(self, msg):
        parts = msg.data.strip().upper().split()
        if len(parts) == 2:
            self._update_state(route={"source": parts[0], "destination": parts[1]})

    def _cb_arduino_status(self, msg):
        self._update_state(arduino_status=msg.data)

    def _cb_ball_status(self, msg):
        self._update_state(ball_status=msg.data)

    def _cb_ik_target(self, msg):
        self._update_state(ik_target={"x": float(msg.x), "y": float(msg.y)})

    def _cb_suction_cmd(self, msg):
        self._update_state(suction_on=bool(msg.data))

    def _cb_ball_detected(self, msg):
        self._update_state(ball_detected=bool(msg.data))

    def _cb_pressure_raw(self, msg):
        self._update_state(pressure_raw=int(msg.data))

    def destroy_node(self):
        try:
            self._server.shutdown()
            self._server.server_close()
        finally:
            super().destroy_node()


class DashboardRequestHandler(BaseHTTPRequestHandler):
    def __init__(self, *args, bridge=None, **kwargs):
        self._bridge = bridge
        super().__init__(*args, **kwargs)

    def log_message(self, _format, *args):
        if self._bridge is not None:
            self._bridge.get_logger().debug("HTTP: " + (_format % args))

    def do_GET(self):
        if self.path == "/api/state":
            self._write_json(self._bridge.snapshot(), HTTPStatus.OK)
            return

        self._serve_static()

    def do_POST(self):
        if self.path != "/api/command":
            self._write_json({"ok": False, "error": "not found"}, HTTPStatus.NOT_FOUND)
            return

        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length else b"{}"
        try:
            payload = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError:
            self._write_json({"ok": False, "error": "invalid json"}, HTTPStatus.BAD_REQUEST)
            return

        body, status = self._bridge.handle_command(payload)
        self._write_json(body, status)

    def _serve_static(self):
        relative = self.path.lstrip("/") or "index.html"
        if relative.endswith("/"):
            relative += "index.html"

        root = self._bridge.web_root()
        full = os.path.abspath(os.path.join(root, relative))
        root_abs = os.path.abspath(root)
        if not full.startswith(root_abs):
            self._write_json({"ok": False, "error": "forbidden"}, HTTPStatus.FORBIDDEN)
            return

        if not os.path.isfile(full):
            fallback = os.path.join(root, "index.html")
            if os.path.isfile(fallback):
                full = fallback
            else:
                self._write_json({"ok": False, "error": "not found"}, HTTPStatus.NOT_FOUND)
                return

        ctype, _ = mimetypes.guess_type(full)
        ctype = ctype or "application/octet-stream"
        with open(full, "rb") as handle:
            data = handle.read()

        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _write_json(self, payload, status):
        data = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)


def main():
    rclpy.init()
    node = DashboardBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()