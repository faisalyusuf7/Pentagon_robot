#!/usr/bin/env python3
"""
Standalone 5-bar parallel linkage IK node.

No MoveIt, no ros2_control.  Publishes ALL joint angles directly
to /joint_states so robot_state_publisher can visualise the closed
linkage in RViz.

Subscribe to /ik_target (geometry_msgs/Point) — x,y in the IK
plane (origin centred between the two motor shafts, y = up).

Joints published:
    motor_joint_left          (actuated)
    motor_joint_right         (actuated)
    joint_link_left           (passive – left coupler)
    right_joint_link          (passive – right coupler)
    gear_servo_suction_joint  (suction servo, default 0)
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose


# ------------------------------------------------------------------ #
#  Rotation helpers
# ------------------------------------------------------------------ #
def _Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def _Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def _Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def _rpy(r, p, y):
    return _Rz(y) @ _Ry(p) @ _Rx(r)

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class FiveBarIKNode(Node):
    """Standalone 5-bar IK → joint_states publisher."""

    # ---- URDF constants (from SolidWorks export) ----
    # Motor joint frame orientations (rpy from URDF)
    _R_ML = _rpy(-math.pi / 2, 0, 0.00292493868428481)
    _R_MR = _rpy(-math.pi / 2, 0, 0.0108765596422485)

    # tray_to_linkage_joint: tray_base_link → base_link offset (URDF fixed joint)
    _BASE_LINK_OFFSET = np.array([-0.006403, -0.103113, -0.00474])

    # Motor joint positions in base_link frame (xyz from URDF)
    # Converted to tray_base_link (world) frame by adding _BASE_LINK_OFFSET
    _POS_ML = np.array([0.695406926143987, -1.58664693818658, 0.0500000000000009]) + _BASE_LINK_OFFSET
    _POS_MR = np.array([0.885406926135972, -1.58664693819113, 0.0699999999999998]) + _BASE_LINK_OFFSET

    # Passive joint origin offsets in motor-link frame (xyz from URDF)
    _PASS_L_XYZ = np.array([0.000019, -0.014000, 0.200001])
    _PASS_R_XYZ = np.array([0.0, -0.00309999999999989, 0.199999999999012])

    # Passive (coupler) joint initial orientations (rpy from URDF)
    _R_PL_INIT = _rpy(0, 1.07736121547448, math.pi)
    _R_PR_INIT = _rpy(0, 1.09202325866201, 0)

    # Suction joint offset in right_link frame (fixed joint from URDF)
    _SUCTION_XYZ = np.array([-0.20999988943129,
                              -0.0469999999999999,
                              -0.000148706565137574])

    # suction_link -> channel_suction_link (fixed joint from URDF: suction_channel)
    _CHANNEL_XYZ = np.array([-0.000149999999999983, -0.0098999999999998, 0.05])
    _R_CHANNEL = _rpy(-math.pi / 2.0, 0.0, math.pi)

    # Nozzle tip (very bottom center) in channel_suction_link frame.
    # Derived from `meshes/channel_suction_link.STL` bounds: long axis is +Y,
    # bottom-most end is at y = -0.00102810119, with x,z ~ 0.
    _TIP_CH = np.array([0.0, -0.00102810119, 0.0])

    # IK origin = midpoint of motor shafts (pre-computed)
    _IK_ORIGIN = (_POS_ML + _POS_MR) / 2.0

    ALL_JOINTS = [
        "motor_joint_left",
        "motor_joint_right",
        "joint_link_left",
        "right_joint_link",
        "gear_servo_suction_joint",
    ]

    # ---- Ball / tray geometry ----
    BALL_RADIUS = 0.02           # 35 mm diameter ball
    HOLE_DEPTH_FRONT = 0.015       # 15 mm (front tray holes)
    HOLE_DEPTH_LEFT  = 0.003       # ~3 mm (left tray holes, much shallower)

    # Hole positions: (x, y, z_top_surface) in world (tray_base_link) frame
    # z_top_surface = Z of the tray top where the hole rim is
    FRONT_TRAY_HOLES = {
        "F0": (0.724254, -1.440010, 0.020256),
        "F1": (0.724254, -1.395010, 0.020256),
        "F2": (0.724254, -1.350010, 0.020256),
        "F3": (0.769254, -1.440010, 0.020256),
        "F4": (0.769254, -1.395010, 0.020256),
        "F5": (0.769254, -1.350010, 0.020256),
        "F6": (0.814254, -1.440010, 0.020256),
        "F7": (0.814254, -1.395010, 0.020256),
        "F8": (0.824004, -1.349760, 0.020256),
    }
    LEFT_TRAY_HOLES = {
        "L0": (0.594254, -1.600010, 0.000256),
        "L1": (0.549254, -1.600010, 0.000256),
        "L2": (0.504254, -1.600010, 0.000256),
        "L3": (0.594254, -1.645010, 0.000256),
        "L4": (0.549254, -1.645010, 0.000256),
        "L5": (0.504254, -1.645010, 0.000256),
        "L6": (0.594254, -1.690010, 0.000256),
        "L7": (0.549254, -1.690010, 0.000256),
        "L8": (0.504254, -1.690010, 0.000256),
    }

    def __init__(self):
        super().__init__("fivebar_ik_node")

        # --- parameters ---
        self.declare_parameter("L1", 0.200)          # crank length (m)
        self.declare_parameter("L2", 0.200)          # coupler length (m)
        self.declare_parameter("d",  0.190)          # base separation (m)
        self.declare_parameter("publish_rate", 50.0) # Hz

        self.L1 = float(self.get_parameter("L1").value)
        self.L2 = float(self.get_parameter("L2").value)
        self.d  = float(self.get_parameter("d").value)
        rate    = float(self.get_parameter("publish_rate").value)

        # current joint positions (start at home = all zeros)
        self._positions = [0.0] * len(self.ALL_JOINTS)
        self._target_received = False   # don't drive motors until first /ik_target

        # ---- Ball state ----
        # Each entry: hole_name → {"tray": "front"|"left", "held": False}
        # Only balls that exist are tracked here.
        self._balls = {
            "F4": {"tray": "front", "held": False},
        }
        self._held_ball = None  # name of ball currently gripped by suction
        self._suction_on = False
        self._current_ik_target = (0.0, 0.25)  # track last IK target for EE pos

        # FK-tracked nozzle tip position (world). Updated on every IK target.
        self._suction_world_pos = None

        # IK origin (midpoint of motor shafts)
        self._ik_origin = (self._POS_ML + self._POS_MR) / 2.0

        # Compute initial suction position
        suc = self._compute_suction_world(0.0, 0.25)
        if suc is not None:
            self._suction_world_pos = suc.copy()

        # publisher & subscriber
        self._js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._marker_pub = self.create_publisher(MarkerArray, "/ball_markers", 10)
        # Debug markers (EE + targets) to make frame/mapping issues obvious in RViz.
        self._debug_pub = self.create_publisher(MarkerArray, "/debug_markers", 10)
        self._status_pub = self.create_publisher(String, "/ball_status", 10)
        self.create_subscription(Point, "/ik_target", self._cb_target, 10)
        self.create_subscription(Bool, "/suction_cmd", self._cb_suction, 10)

        # periodic publisher so RViz always has fresh TF
        self.create_timer(1.0 / rate, self._publish_js)
        # ball markers at 5 Hz
        self.create_timer(0.2, self._publish_ball_markers)

        # hole markers at 1 Hz (debug: verify mapping)
        self.create_timer(1.0, self._publish_hole_markers)

        # EE / target debug markers at 20 Hz
        self.create_timer(0.05, self._publish_debug_markers)

        self.get_logger().info(
            f"5-bar IK node ready  L1={self.L1}  L2={self.L2}  "
            f"d={self.d}  rate={rate} Hz"
        )

    # ================================================================ #
    #  IK  (law-of-cosines, same as Arduino solver)
    # ================================================================ #
    def _solve_motor_ik(self, px, py):
        """Return (theta_left, theta_right) in IK-plane radians, or None."""
        L1, L2, d = self.L1, self.L2, self.d
        b1x, b1y = -d / 2.0, 0.0
        b2x, b2y =  d / 2.0, 0.0

        r1 = math.hypot(px - b1x, py - b1y)
        r2 = math.hypot(px - b2x, py - b2y)

        rmin, rmax = abs(L1 - L2), L1 + L2
        tol = 1e-6
        if r1 < rmin - tol or r1 > rmax + tol:
            return None
        if r2 < rmin - tol or r2 > rmax + tol:
            return None

        # left motor – elbow up
        phi1 = math.atan2(py - b1y, px - b1x)
        ca1  = _clamp((L1*L1 + r1*r1 - L2*L2) / (2.0*L1*r1), -1, 1)
        th1  = phi1 + math.acos(ca1)

        # right motor – elbow down
        phi2 = math.atan2(py - b2y, px - b2x)
        ca2  = _clamp((L1*L1 + r2*r2 - L2*L2) / (2.0*L1*r2), -1, 1)
        th2  = phi2 - math.acos(ca2)

        return th1, th2

    #  IK → URDF angle mapping
    
    @staticmethod
    def _motor_ik_to_urdf(th_ik):
        """
        IK plane : θ=0 → crank along +X,  θ=π/2 → crank along +Y
        URDF     : θ=0 → crank along +Y,  θ=π/2 → crank along +X
        (joint frames rotated -90° about X, rotation axis = local Y)

        Mapping : θ_urdf = π/2 − θ_ik
        """
        return math.pi / 2.0 - th_ik

    def _passive_urdf_angle(self, th_motor_urdf, R_motor, pos_motor,
                            pass_xyz, R_pass_init, target_world):
        """
        Compute the URDF passive-joint angle that makes the coupler
        link point from the crank tip toward the target.

        The coupler link extends along **-X** in its local frame
        (confirmed from URDF mesh/inertial data).

        Transform chain (world ← base ← motor ← coupler):
            R_motor · Ry(th_motor_urdf) · R_pass_init · Ry(θ_passive)
        Coupler tip = passive_origin + above_rotation · (-L2, 0, 0).

        We solve:
            passive_origin + A · Ry(p) · (-L2, 0, 0) = target
        where A = R_motor · Ry(th_motor) · R_pass_init

        Ry(p) · (-L2, 0, 0) = (-L2·cos(p), 0, L2·sin(p))
        So v = A⁻¹ · delta  →  p = atan2(v[2], -v[0])
        """
        # passive joint origin in world
        pass_origin = pos_motor + R_motor @ _Ry(th_motor_urdf) @ pass_xyz

        # delta from passive joint origin to target
        delta = target_world - pass_origin

        # A = R_motor · Ry(th_motor) · R_pass_init
        A = R_motor @ _Ry(th_motor_urdf) @ R_pass_init

        # solve:  Ry(p) · (-L2,0,0) = A⁻¹ · delta
        # (-L2·cos(p), 0, L2·sin(p)) = v
        v = np.linalg.inv(A) @ delta
        return math.atan2(v[2], -v[0])

    def _compute_target_world(self, px, py):
        """IK-plane point → world coordinates (mid-Z of motors)."""
        mid = (self._POS_ML + self._POS_MR) / 2.0
        return np.array([mid[0] + px, mid[1] + py, mid[2]])

    # ================================================================ #
    #  Suction tip FK — actual suction_link world position
    # ================================================================ #
    def _compute_suction_world(self, px, py):
        """
                Given an IK-plane target (px, py), compute the world XYZ
                of the *nozzle tip* (bottom center of channel_suction_link)
                by tracing the FK chain:
                    base → motor_right → right_link → suction_link → channel_suction_link

        Returns np.array([x, y, z]) or None if IK fails.
        """
        sol = self._solve_motor_ik(px, py)
        if sol is None:
            return None

        _, th2_ik = sol
        th2_urdf = self._motor_ik_to_urdf(th2_ik)

        # Right motor frame → crank tip (passive joint origin)
        R_mr = self._R_MR @ _Ry(th2_urdf)
        crank_tip = self._POS_MR + R_mr @ self._PASS_R_XYZ

        # Passive angle on right coupler
        target_w = self._compute_target_world(px, py)
        A = R_mr @ self._R_PR_INIT
        delta = target_w - crank_tip
        v = np.linalg.inv(A) @ delta
        pass_angle = math.atan2(v[2], -v[0])

        # Right-link (coupler) orientation in world
        R_right = A @ _Ry(pass_angle)

        # suction_link orientation & origin in world
        # (rotation from right_link -> suction_link is fixed in URDF)
        R_suction = R_right @ _rpy(-math.pi / 2.0, math.pi / 2.0, 0.0)
        p_suction = crank_tip + R_right @ self._SUCTION_XYZ

        # channel_suction_link origin and orientation in world
        p_channel = p_suction + R_suction @ self._CHANNEL_XYZ
        R_channel = R_suction @ self._R_CHANNEL

        # nozzle tip in world
        tip_world = p_channel + R_channel @ self._TIP_CH
        return tip_world

    def _solve_ik_for_suction(self, target_wx, target_wy, max_iter=5):
        """
        Iteratively find the IK target (px, py) that places the
        suction_link origin at (target_wx, target_wy) in world XY.

        Returns corrected (px, py) or None if unreachable.
        """
        mid = self._IK_ORIGIN
        px = target_wx - mid[0]
        py = target_wy - mid[1]

        for _ in range(max_iter):
            suc = self._compute_suction_world(px, py)
            if suc is None:
                return None
            err_x = target_wx - suc[0]
            err_y = target_wy - suc[1]
            if math.hypot(err_x, err_y) < 0.0001:  # 0.1 mm
                break
            px += err_x
            py += err_y

        return (px, py)

    #  Callbacks
    def _cb_target(self, msg: Point):
        self._target_received = True
        sol = self._solve_motor_ik(msg.x, msg.y)
        if sol is None:
            self.get_logger().warn(
                f"Target ({msg.x:.3f}, {msg.y:.3f}) unreachable"
            )
            return

        th1_ik, th2_ik = sol
        th1_urdf = self._motor_ik_to_urdf(th1_ik)
        th2_urdf = self._motor_ik_to_urdf(th2_ik)

        target_w = self._compute_target_world(msg.x, msg.y)

        pass_L = self._passive_urdf_angle(
            th1_urdf, self._R_ML, self._POS_ML,
            self._PASS_L_XYZ, self._R_PL_INIT, target_w,
        )
        pass_R = self._passive_urdf_angle(
            th2_urdf, self._R_MR, self._POS_MR,
            self._PASS_R_XYZ, self._R_PR_INIT, target_w,
        )

        self._positions = [th1_urdf, th2_urdf, pass_L, pass_R, 0.0]

        # Track current IK target for ball-follow
        self._current_ik_target = (msg.x, msg.y)

        # Compute actual suction tip position via FK
        suc = self._compute_suction_world(msg.x, msg.y)
        if suc is not None:
            self._suction_world_pos = suc.copy()

        self.get_logger().info(
            f"Target ({msg.x:.3f}, {msg.y:.3f}) → "
            f"motors [{math.degrees(th1_urdf):.1f}°, "
            f"{math.degrees(th2_urdf):.1f}°]  "
            f"passives [{math.degrees(pass_L):.1f}°, "
            f"{math.degrees(pass_R):.1f}°]"
        )

    # ------------------------------------------------------------ #
    #  Suction pick / place
    # ------------------------------------------------------------ #
    def _cb_suction(self, msg: Bool):
        """Handle suction on/off commands from the planner."""
        if msg.data:
            self._suction_pick()
        else:
            self._suction_release()

    def _suction_pick(self):
        """Suction ON — grab the nearest ball at current suction tip position."""
        if self._held_ball is not None:
            self.get_logger().warn("Already holding a ball!")
            return

        if self._suction_world_pos is None:
            self.get_logger().warn("Suction position unknown!")
            return

        sx, sy = self._suction_world_pos[0], self._suction_world_pos[1]
        best_name = None
        best_dist = float("inf")

        for name, info in self._balls.items():
            if info["held"]:
                continue
            pos = self._ball_world_pos(name)
            if pos is None:
                continue
            dist = math.hypot(sx - pos[0], sy - pos[1])
            if dist < best_dist:
                best_dist = dist
                best_name = name

        PICK_THRESHOLD = 0.05   # 50 mm
        if best_name is None or best_dist > PICK_THRESHOLD:
            self.get_logger().warn(
                f"No ball within {PICK_THRESHOLD*1000:.0f} mm of EE "
                f"(nearest: {best_name} @ {best_dist*1000:.1f} mm)")
            self._publish_ball_status(f"PICK FAILED — no ball nearby")
            return

        self._balls[best_name]["held"] = True
        self._held_ball = best_name
        self._suction_on = True
        self.get_logger().info(
            f"PICKED ball '{best_name}' (dist {best_dist*1000:.1f} mm)")
        self._publish_ball_status(f"PICKED {best_name}")

    def _suction_release(self):
        """Suction OFF — drop the held ball into the nearest hole."""
        if self._held_ball is None:
            self.get_logger().warn("No ball held to release!")
            return

        if self._suction_world_pos is None:
            self.get_logger().warn("Suction position unknown!")
            return

        sx, sy = self._suction_world_pos[0], self._suction_world_pos[1]
        ball_name = self._held_ball

        # Find the nearest empty hole to snap the ball into
        all_holes = {**self.FRONT_TRAY_HOLES, **self.LEFT_TRAY_HOLES}
        occupied = {n for n, i in self._balls.items() if not i["held"]}

        best_hole = None
        best_dist = float("inf")
        for hole_name, (hx, hy, _) in all_holes.items():
            if hole_name in occupied and hole_name != ball_name:
                continue  # skip occupied holes
            dist = math.hypot(sx - hx, sy - hy)
            if dist < best_dist:
                best_dist = dist
                best_hole = hole_name

        PLACE_THRESHOLD = 0.05
        if best_hole is None or best_dist > PLACE_THRESHOLD:
            self.get_logger().warn(
                f"No hole within {PLACE_THRESHOLD*1000:.0f} mm — "
                f"ball dropped at current position")
            # Just unhold without snapping
            self._balls[ball_name]["held"] = False
            self._held_ball = None
            self._suction_on = False
            self._publish_ball_status(f"DROPPED {ball_name} (no hole)")
            return

        # Snap ball to the destination hole
        old_tray = self._balls[ball_name]["tray"]
        new_tray = "front" if best_hole.startswith("F") else "left"

        # Remove ball from old position, add to new
        del self._balls[ball_name]
        self._balls[best_hole] = {"tray": new_tray, "held": False}

        self._held_ball = None
        self._suction_on = False
        self.get_logger().info(
            f"PLACED ball '{ball_name}' → hole '{best_hole}' "
            f"(dist {best_dist*1000:.1f} mm)")
        self._publish_ball_status(f"PLACED {ball_name} → {best_hole}")

    def _publish_ball_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _ee_world_pos(self):
        """Return the current suction-tip world position (x, y, z) via FK."""
        if self._suction_world_pos is not None:
            return self._suction_world_pos.copy()
        # fallback to IK meeting point
        return self._compute_target_world(*self._current_ik_target)

    def _ball_world_pos(self, ball_name):
        """Return (x, y, z) world position of the ball centre.

        If held: follows suction tip (XY + Z from FK).
        If in hole: z_centre = z_top_surface − hole_depth + ball_radius
        """
        info = self._balls[ball_name]
        if info["held"]:
            if self._suction_world_pos is not None:
                s = self._suction_world_pos
                return (s[0], s[1], s[2])
            ee = self._compute_target_world(*self._current_ik_target)
            return (ee[0], ee[1], ee[2])
        holes = self.FRONT_TRAY_HOLES if info["tray"] == "front" \
                else self.LEFT_TRAY_HOLES
        hx, hy, z_top = holes[ball_name]
        depth = self.HOLE_DEPTH_FRONT if info["tray"] == "front" \
                else self.HOLE_DEPTH_LEFT
        z_centre = z_top - depth + self.BALL_RADIUS
        return (hx, hy, z_centre)

    def _publish_ball_markers(self):
        """Publish sphere markers for every tracked ball."""
        ma = MarkerArray()
        for idx, (name, info) in enumerate(self._balls.items()):
            pos = self._ball_world_pos(name)
            if pos is None:
                continue

            m = Marker()
            m.header.frame_id = "tray_base_link"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "balls"
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = pos[0]
            m.pose.position.y = pos[1]
            m.pose.position.z = pos[2]
            m.pose.orientation.w = 1.0
            d = self.BALL_RADIUS * 2.0
            m.scale.x = d
            m.scale.y = d
            m.scale.z = d
            # orange when in hole, green when held
            if info["held"]:
                m.color.r = 0.2
                m.color.g = 1.0
                m.color.b = 0.2
            else:
                m.color.r = 1.0
                m.color.g = 0.4
                m.color.b = 0.0
            m.color.a = 1.0
            m.lifetime.sec = 0
            ma.markers.append(m)
        self._marker_pub.publish(ma)

    def _publish_hole_markers(self):
        """Publish markers for tray hole centers (debug)."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        def add_hole(name, x, y, z, idx, r, g, b):
            m = Marker()
            m.header.frame_id = "tray_base_link"
            m.header.stamp = now
            m.ns = "holes"
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = float(z)
            m.pose.orientation.w = 1.0
            m.scale.x = 0.01
            m.scale.y = 0.01
            m.scale.z = 0.01
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 1.0
            ma.markers.append(m)

            t = Marker()
            t.header.frame_id = "tray_base_link"
            t.header.stamp = now
            t.ns = "hole_labels"
            t.id = 1000 + idx
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(x)
            t.pose.position.y = float(y)
            t.pose.position.z = float(z) + 0.02
            t.pose.orientation.w = 1.0
            t.scale.z = 0.02
            t.color.r = 1.0
            t.color.g = 1.0
            t.color.b = 1.0
            t.color.a = 1.0
            t.text = name
            ma.markers.append(t)

        idx = 0
        for name, (x, y, z) in self.FRONT_TRAY_HOLES.items():
            if name == "F4":
                add_hole(name, x, y, z, idx, 0.0, 1.0, 1.0)   # cyan
            else:
                add_hole(name, x, y, z, idx, 0.3, 0.3, 0.3)
            idx += 1

        for name, (x, y, z) in self.LEFT_TRAY_HOLES.items():
            if name == "L0":
                add_hole(name, x, y, z, idx, 0.0, 1.0, 0.0)   # green
            else:
                add_hole(name, x, y, z, idx, 1.0, 1.0, 0.0)   # yellow
            idx += 1

        self._marker_pub.publish(ma)

    def _publish_debug_markers(self):
        """Publish debug markers for nozzle tip + IK meeting point."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 1) IK meeting-point in world (what /ik_target directly represents)
        ik_w = self._compute_target_world(*self._current_ik_target)
        m_ik = Marker()
        m_ik.header.frame_id = "tray_base_link"
        m_ik.header.stamp = now
        m_ik.ns = "ee_debug"
        m_ik.id = 1
        m_ik.type = Marker.SPHERE
        m_ik.action = Marker.ADD
        m_ik.pose.position.x = float(ik_w[0])
        m_ik.pose.position.y = float(ik_w[1])
        m_ik.pose.position.z = float(ik_w[2])
        m_ik.pose.orientation.w = 1.0
        m_ik.scale.x = 0.012
        m_ik.scale.y = 0.012
        m_ik.scale.z = 0.012
        m_ik.color.r = 1.0
        m_ik.color.g = 1.0
        m_ik.color.b = 0.0
        m_ik.color.a = 1.0
        ma.markers.append(m_ik)

        t_ik = Marker()
        t_ik.header.frame_id = "tray_base_link"
        t_ik.header.stamp = now
        t_ik.ns = "ee_debug_text"
        t_ik.id = 101
        t_ik.type = Marker.TEXT_VIEW_FACING
        t_ik.action = Marker.ADD
        t_ik.pose.position.x = float(ik_w[0])
        t_ik.pose.position.y = float(ik_w[1])
        t_ik.pose.position.z = float(ik_w[2]) + 0.03
        t_ik.pose.orientation.w = 1.0
        t_ik.scale.z = 0.02
        t_ik.color.r = 1.0
        t_ik.color.g = 1.0
        t_ik.color.b = 0.0
        t_ik.color.a = 1.0
        t_ik.text = "IK target"
        ma.markers.append(t_ik)

        # 2) Nozzle tip (FK)
        if self._suction_world_pos is not None:
            tip = self._suction_world_pos
            m_tip = Marker()
            m_tip.header.frame_id = "tray_base_link"
            m_tip.header.stamp = now
            m_tip.ns = "ee_debug"
            m_tip.id = 2
            m_tip.type = Marker.SPHERE
            m_tip.action = Marker.ADD
            m_tip.pose.position.x = float(tip[0])
            m_tip.pose.position.y = float(tip[1])
            m_tip.pose.position.z = float(tip[2])
            m_tip.pose.orientation.w = 1.0
            m_tip.scale.x = 0.014
            m_tip.scale.y = 0.014
            m_tip.scale.z = 0.014
            m_tip.color.r = 0.0
            m_tip.color.g = 1.0
            m_tip.color.b = 1.0
            m_tip.color.a = 1.0
            ma.markers.append(m_tip)

            t_tip = Marker()
            t_tip.header.frame_id = "tray_base_link"
            t_tip.header.stamp = now
            t_tip.ns = "ee_debug_text"
            t_tip.id = 102
            t_tip.type = Marker.TEXT_VIEW_FACING
            t_tip.action = Marker.ADD
            t_tip.pose.position.x = float(tip[0])
            t_tip.pose.position.y = float(tip[1])
            t_tip.pose.position.z = float(tip[2]) + 0.03
            t_tip.pose.orientation.w = 1.0
            t_tip.scale.z = 0.02
            t_tip.color.r = 0.0
            t_tip.color.g = 1.0
            t_tip.color.b = 1.0
            t_tip.color.a = 1.0
            t_tip.text = "Nozzle tip"
            ma.markers.append(t_tip)

        self._debug_pub.publish(ma)

    def _publish_js(self):
        if not self._target_received:
            return  # don't publish until first /ik_target → prevents startup arm motion
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.ALL_JOINTS)
        js.position = list(self._positions)
        self._js_pub.publish(js)


def main():
    rclpy.init()
    node = FiveBarIKNode()
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
