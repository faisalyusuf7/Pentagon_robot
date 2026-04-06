#!/usr/bin/env python3
"""
Pick-and-place state-machine planner for the 5-bar parallel linkage.

PATCHED:
- Uses minimum-jerk easing (smoother than smooth-step)
- Continuous trajectory sampling (no waypoint popping / less jitter)
- Reachability check (same law-of-cosines check as your IK) + auto-reduce approach offset
- Starts every segment from current commanded EE position (self._current_ik)
- Cancel is safe: suction OFF + return HOME
- Publishes a denser /planner_path (optional but helpful)
"""

import math
import time
from enum import Enum, auto

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String, Bool, Empty, Int32
from nav_msgs.msg import Path


# ================================================================== #
#  Hole positions  (world frame)
# ================================================================== #
# Front tray 3×3 grid — calibrated from calibrate_holes.py
#        col0 (IK x=-0.035)  col1 (IK x=0.010)   col2 (IK x=0.055)
# row0  F0                    F3                    F6
# row1  F1                    F4                    F7
# row2  F2                    F5                    F8
FRONT_TRAY_HOLES = {
    "F0": (0.749004, -1.429760, 0.020256),
    "F1": (0.749004, -1.389760, 0.020256),
    "F2": (0.749004, -1.344760, 0.020256),
    "F3": (0.794004, -1.429990, 0.020256),
    "F4": (0.794004, -1.389760, 0.020256),
    "F5": (0.794004, -1.344760, 0.020256),
    "F6": (0.839004, -1.434760, 0.020256),
    "F7": (0.839004, -1.389760, 0.020256),
    "F8": (0.839004, -1.344760, 0.020256),
}

LEFT_TRAY_HOLES = {
    "L0": (0.599254, -1.650010, 0.000256),
    "L1": (0.554254, -1.650010, 0.000256),
    "L2": (0.509254, -1.650010, 0.000256),
    "L3": (0.599254, -1.695010, 0.000256),
    "L4": (0.554254, -1.695010, 0.000256),
    "L5": (0.509254, -1.695010, 0.000256),
    "L6": (0.599254, -1.740010, 0.000256),
    "L7": (0.554254, -1.740010, 0.000256),
    "L8": (0.509254, -1.740010, 0.000256),
}

ALL_HOLES = {**FRONT_TRAY_HOLES, **LEFT_TRAY_HOLES}

# IK origin = midpoint of the two motor shafts (world frame)
# Motor positions in URDF are relative to base_link; must add
# tray_to_linkage_joint offset to get tray_base_link (world) frame.
_BASE_LINK_OFFSET = np.array([-0.006403, -0.103113, -0.00474])
_POS_ML = np.array([0.695406926143987, -1.58664693818658, 0.05]) + _BASE_LINK_OFFSET
_POS_MR = np.array([0.885406926135972, -1.58664693819113, 0.07]) + _BASE_LINK_OFFSET
IK_ORIGIN = (_POS_ML + _POS_MR) / 2.0


# ================================================================== #
#  Rotation helpers (duplicated from IK node for suction FK)
# ================================================================== #
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


# ---- URDF constants for right-side FK chain → suction tip ----
_R_MR  = _rpy(-math.pi / 2, 0, 0.0108765596422485)
_R_PR_INIT = _rpy(0, 1.09202325866201, 0)
_PASS_R_XYZ = np.array([0.0, -0.00309999999999989, 0.199999999999012])
_SUCTION_XYZ = np.array([-0.20999988943129, -0.0469999999999999, -0.000148706565137574])

# suction_link -> channel_suction_link (fixed joint suction_channel)
_CHANNEL_XYZ = np.array([-0.000149999999999983, -0.0098999999999998, 0.05])
_R_CHANNEL = _rpy(-math.pi / 2.0, 0.0, math.pi)

# Nozzle tip in channel_suction_link frame (from mesh bounds)
_TIP_CH = np.array([0.0, -0.00102810119, 0.0])
_L1_DEFAULT = 0.200
_L2_DEFAULT = 0.200
_D_DEFAULT  = 0.190


def _solve_motor_ik(px, py, L1=_L1_DEFAULT, L2=_L2_DEFAULT, d=_D_DEFAULT):
    """Return (theta_left, theta_right) in IK-plane radians, or None."""
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
    phi1 = math.atan2(py - b1y, px - b1x)
    ca1  = _clamp((L1*L1 + r1*r1 - L2*L2) / (2.0*L1*r1), -1, 1)
    th1  = phi1 + math.acos(ca1)
    phi2 = math.atan2(py - b2y, px - b2x)
    ca2  = _clamp((L1*L1 + r2*r2 - L2*L2) / (2.0*L1*r2), -1, 1)
    th2  = phi2 - math.acos(ca2)
    return th1, th2


def _compute_suction_world(px, py):
    """
    Given IK-plane target (px, py), compute nozzle tip world XYZ
    via FK chain: motor_right → right_link → suction_link → channel_suction_link.
    """
    sol = _solve_motor_ik(px, py)
    if sol is None:
        return None
    _, th2_ik = sol
    th2_urdf = math.pi / 2.0 - th2_ik
    R_mr = _R_MR @ _Ry(th2_urdf)
    crank_tip = _POS_MR + R_mr @ _PASS_R_XYZ
    target_w = np.array([IK_ORIGIN[0] + px, IK_ORIGIN[1] + py, IK_ORIGIN[2]])
    A = R_mr @ _R_PR_INIT
    delta = target_w - crank_tip
    v = np.linalg.inv(A) @ delta
    pass_angle = math.atan2(v[2], -v[0])
    R_right = A @ _Ry(pass_angle)

    # suction_link orientation & origin in world (fixed rot from right_link)
    R_suction = R_right @ _rpy(-math.pi / 2.0, math.pi / 2.0, 0.0)
    p_suction = crank_tip + R_right @ _SUCTION_XYZ

    # channel_suction_link origin + orientation in world
    p_channel = p_suction + R_suction @ _CHANNEL_XYZ
    R_channel = R_suction @ _R_CHANNEL

    # nozzle tip
    return p_channel + R_channel @ _TIP_CH


def solve_ik_for_suction(target_wx, target_wy, max_iter=5):
    """
    Iteratively find the IK target (px, py) that places the
    suction_link at (target_wx, target_wy) in world XY.
    Returns corrected (px, py) or None if unreachable.
    """
    px = target_wx - IK_ORIGIN[0]
    py = target_wy - IK_ORIGIN[1]
    for _ in range(max_iter):
        suc = _compute_suction_world(px, py)
        if suc is None:
            return None
        err_x = target_wx - suc[0]
        err_y = target_wy - suc[1]
        if math.hypot(err_x, err_y) < 0.0001:
            break
        px += err_x
        py += err_y
    return (px, py)


def world_to_ik(wx, wy):
    """Convert world (x, y) → IK plane (x, y)."""
    # Suction FK correction disabled — stale old-URDF constants cause drift
    # result = solve_ik_for_suction(wx, wy)
    # if result is not None:
    #     return result
    return wx - IK_ORIGIN[0], wy - IK_ORIGIN[1]


def ik_to_world(ix, iy):
    """Convert IK plane (x, y) → world (x, y)."""
    return ix + IK_ORIGIN[0], iy + IK_ORIGIN[1]


# ================================================================== #
#  Trajectory helpers
# ================================================================== #
def lerp(a, b, t):
    return a + (b - a) * t


def min_jerk(t):
    """Minimum-jerk easing (C2 continuous)."""
    t = max(0.0, min(1.0, t))
    return 10*t**3 - 15*t**4 + 6*t**5


# ================================================================== #
#  State machine
# ================================================================== #
class State(Enum):
    IDLE            = auto()
    MOVE_ABOVE_SRC  = auto()
    DESCEND_SRC     = auto()
    SUCTION_ON      = auto()
    ASCEND_SRC      = auto()
    RETRACT_SRC     = auto()   # pull back towards centre before transfer
    TRANSFER        = auto()   # swing to approach direction of dst
    MOVE_ABOVE_DST  = auto()
    DESCEND_DST     = auto()
    SUCTION_OFF     = auto()
    ASCEND_DST      = auto()
    RETRACT_DST     = auto()   # pull back to TRANSFER_RADIUS before homing
    MOVE_HOME       = auto()   # arc sweep at TRANSFER_RADIUS to HOME angle


class PickAndPlacePlanner(Node):
    # ---- Defaults ----
    APPROACH_OFFSET_Y = 0.06       # IK radial offset for approach (meters)
    MOVE_SPEED        = 0.15       # m/s lateral
    DESCEND_SPEED     = 0.05       # m/s radial in/out
    ARM_SETTLE_TIME   = 1.5        # seconds for physical steppers to reach target
    SERVO_SETTLE_TIME = 2.5        # seconds for servo to reach down/up position
    DWELL_TIME        = 9.5        # seconds — must cover ARM_SETTLE + suction HW sequence
                                   # HW pick:  servo_down 2.5 + grip 3.0 + servo_up 2.5 = 8.0
                                   # HW place: servo_down 2.5 + release 2.0 + 0.1 + servo_up 2.5 = 7.1
    USE_BALL_DETECT    = True       # gate SUCTION_ON→ASCEND on pressure sensor
    BALL_DETECT_TIMEOUT = 30.0     # seconds — safety-only; proceeds on sensor confirmation
    PICK_RAW_FALLBACK_MIN = 10000000 # raw_u24 threshold for confirming pick (raised to avoid noisy false positives)
    PICK_RAW_FALLBACK_GRACE = 0.4  # ignore pressure spikes immediately after suction starts
    HOVER_DELAY        = 3.0        # seconds after suction before starting hover pattern
    HOVER_RADIUS       = 0.005      # 5 mm circular wobble around hole centre
    HOVER_PERIOD       = 1.5        # seconds per full circle
    HOME_POS          = (-0.02115, 0.35464)  # Just above F3 (radius 0.156m, angle 97.8°)
    TICK_RATE         = 30.0       # Hz
    TRANSFER_RADIUS   = 0.2      # safe radial dist for lateral moves (m)
    TRANSFER_ARC_SPEED = 0.26     # m/s along the arc
    MIN_SAFE_RADIUS   = 0.15     # m: avoid planning straight segments that go inside this radius (singularity zone)

    # Reachability fallback
    MIN_APPROACH      = 0.01       # m
    APPROACH_REDUCE   = 0.75       # multiply if unreachable
    MAX_APPROACH_TRIES = 8

    def __init__(self):
        super().__init__("pick_place_planner")

        # --- kinematics params (must match IK node) ---
        self.declare_parameter("L1", 0.200)
        self.declare_parameter("L2", 0.200)
        self.declare_parameter("d",  0.190)

        self.L1 = float(self.get_parameter("L1").value)
        self.L2 = float(self.get_parameter("L2").value)
        self.d  = float(self.get_parameter("d").value)

        # ---- planner params ----
        self.declare_parameter("approach_offset", self.APPROACH_OFFSET_Y)
        self.declare_parameter("move_speed", self.MOVE_SPEED)
        self.declare_parameter("descend_speed", self.DESCEND_SPEED)
        self.declare_parameter("dwell_time", self.DWELL_TIME)
        self.declare_parameter("arm_settle_time", self.ARM_SETTLE_TIME)
        self.declare_parameter("tick_rate", self.TICK_RATE)
        self.declare_parameter("use_ball_detect", self.USE_BALL_DETECT)
        self.declare_parameter("ball_detect_timeout", self.BALL_DETECT_TIMEOUT)
        self.declare_parameter("pick_raw_fallback_min", self.PICK_RAW_FALLBACK_MIN)
        self.declare_parameter("pick_raw_fallback_grace", self.PICK_RAW_FALLBACK_GRACE)

        self._approach_base = float(self.get_parameter("approach_offset").value)
        self._approach      = self._approach_base
        self._move_speed    = float(self.get_parameter("move_speed").value)
        self._desc_speed    = float(self.get_parameter("descend_speed").value)
        self._dwell         = float(self.get_parameter("dwell_time").value)
        self._arm_settle    = float(self.get_parameter("arm_settle_time").value)
        self._tick_rate     = float(self.get_parameter("tick_rate").value)
        self._use_ball_detect = bool(self.get_parameter("use_ball_detect").value)
        self._ball_detect_timeout = float(self.get_parameter("ball_detect_timeout").value)
        self._pick_raw_fallback_min = int(self.get_parameter("pick_raw_fallback_min").value)
        self._pick_raw_fallback_grace = float(self.get_parameter("pick_raw_fallback_grace").value)

        # ---- state ----
        self._state = State.IDLE
        self._src_name = ""
        self._dst_name = ""
        self._src_ik = (0.0, 0.0)
        self._dst_ik = (0.0, 0.0)

        # Set HOME_POS to a central, neutral, safe position (0, 0.18) in the IK plane
        self.HOME_POS = (0.0, 0.27)

        # where EE is now (commanded)
        self._current_ik = self.HOME_POS

        # cancel / suction tracking
        self._cancelled = False
        self._suction_on = False

        # ball-detect state (from /ball_detected topic)
        self._ball_detected = False
        self._pick_seen_false = False
        self._pick_confirmed = False
        self._pick_detection_armed = False
        self._pick_arm_time = 0.0
        self._latest_pressure_raw = None

        # dwell two-phase tracking (settle then suction)
        self._dwell_settled = False
        self._settle_end = 0.0

        # hover retry state
        self._hover_active = False
        self._hover_start_time = 0.0
        self._hover_centre_ik = (0.0, 0.0)

        # segment (continuous trajectory)
        self._seg_active = False
        self._seg_start = (0.0, 0.0)
        self._seg_end = (0.0, 0.0)
        self._seg_start_time = 0.0
        self._seg_duration = 0.0

        # arc transfer state (used during TRANSFER)
        self._arc_active = False
        self._arc_radius = 0.0
        self._arc_start_angle = 0.0
        self._arc_end_angle = 0.0
        self._arc_start_time = 0.0
        self._arc_duration = 0.0

        # dwell timing
        self._dwell_end = 0.0

        # ---- publishers ----
        self._ik_pub     = self.create_publisher(Point,  "/ik_target",      10)
        self._suc_pub    = self.create_publisher(Bool,   "/suction_cmd",    10)
        self._suc_manual = self.create_publisher(String, "/suction_manual", 10)
        self._status_pub = self.create_publisher(String, "/planner_status", 10)
        self._path_pub   = self.create_publisher(Path,   "/planner_path",   10)

        # ---- subscribers ----
        self.create_subscription(String, "/pick_place_cmd", self._cb_command, 10)
        self.create_subscription(Empty,  "/cancel_plan",    self._cb_cancel, 10)
        self.create_subscription(Bool,   "/ball_detected",  self._cb_ball_detected, 10)
        self.create_subscription(Int32,  "/pressure_raw",   self._cb_pressure_raw, 10)

        # ---- timer ----
        self._timer = self.create_timer(1.0 / self._tick_rate, self._tick)

        self._publish_status("IDLE — waiting for command")
        self.get_logger().info(
            "Pick-and-place planner ready.\n"
            "Command:  /pick_place_cmd  e.g. 'F4 L0'\n"
            "Cancel :  /cancel_plan"
        )

        # NOTE: startup publish removed — motors stay at calibrated 0° (arms parallel) on launch

    # ============================================================== #
    #  Reachability check (matches your IK solver reach check)
    # ============================================================== #
    def _reachable(self, ix, iy):
        L1, L2, d = self.L1, self.L2, self.d
        b1x, b1y = -d / 2.0, 0.0
        b2x, b2y =  d / 2.0, 0.0

        r1 = math.hypot(ix - b1x, iy - b1y)
        r2 = math.hypot(ix - b2x, iy - b2y)

        rmin, rmax = abs(L1 - L2), L1 + L2
        tol = 1e-6
        if r1 < rmin - tol or r1 > rmax + tol:
            return False
        if r2 < rmin - tol or r2 > rmax + tol:
            return False
        return True

    def _compute_above(self, hole_ik):
        """
        Compute an 'above' (approach) point by moving radially outward
        from the IK origin through the hole, by `approach_offset` metres.

        This ensures the arm approaches each hole along a straight line
        from the robot centre, regardless of which tray the hole is in.

        Returns (above_point, used_offset) or (None, None) if impossible.
        """
        x, y = hole_ik
        r = math.hypot(x, y)
        if r < 1e-6:
            # hole is at origin — fall back to +Y
            ux, uy = 0.0, 1.0
        else:
            ux, uy = x / r, y / r

        approach = self._approach_base
        for _ in range(self.MAX_APPROACH_TRIES):
            ax = x + ux * approach
            ay = y + uy * approach
            if self._reachable(ax, ay):
                return (ax, ay), approach
            approach *= self.APPROACH_REDUCE
            if approach < self.MIN_APPROACH:
                break

        return None, None

    @staticmethod
    def _radial_point(hole_ik, radius):
        """Return the point at `radius` from IK origin in the direction of `hole_ik`."""
        x, y = hole_ik
        r = math.hypot(x, y)
        if r < 1e-6:
            return (0.0, radius)
        return (x / r * radius, y / r * radius)

    # ============================================================== #
    #  Arc transfer (constant-radius sweep, avoids singularity)
    # ============================================================== #
    def _start_arc_transfer(self, target_ik=None):
        """
        Begin a circular-arc transfer from the current position to a
        target direction, sweeping at constant TRANSFER_RADIUS around
        the IK origin.

        If target_ik is None (default), sweeps towards self._dst_ik.
        Otherwise sweeps towards target_ik (used by MOVE_HOME to
        sweep to HOME direction).

        Always sweeps through the +Y half-plane (angles stay positive)
        to avoid crossing the motor axis (Y=0 line).
        """
        R = self.TRANSFER_RADIUS

        # Start angle from current position
        sx, sy = self._current_ik
        a_start = math.atan2(sy, sx)

        # End angle from target direction
        if target_ik is None:
            target_ik = self._dst_ik
        dx, dy = target_ik
        a_end = math.atan2(dy, dx)

        # Ensure we sweep through the UPPER half-plane (+Y) to stay
        # away from the motor axis.  Both front-tray (~90°) and
        # left-tray (~155-170°) are in the upper half, so the shorter
        # arc through +Y is always the safe one.
        # Normalise both angles to [0, 2π)
        a_start = a_start % (2.0 * math.pi)
        a_end   = a_end   % (2.0 * math.pi)

        # Choose the sweep direction that keeps us in +Y
        # (both trays are between ~80° and ~180°, so the short arc is fine)
        sweep = a_end - a_start
        # take the shorter arc (< π in magnitude)
        if sweep > math.pi:
            sweep -= 2.0 * math.pi
        elif sweep < -math.pi:
            sweep += 2.0 * math.pi

        arc_length = abs(sweep) * R
        duration = max(0.15, arc_length / max(1e-6, self.TRANSFER_ARC_SPEED))

        self._arc_active = True
        self._seg_active = False   # not using linear segment
        self._arc_radius = R
        self._arc_start_angle = a_start
        self._arc_end_angle = a_start + sweep
        self._arc_start_time = time.monotonic()
        self._arc_duration = duration

        self.get_logger().info(
            f"  arc R={R:.3f}  {math.degrees(a_start):.1f}° → "
            f"{math.degrees(a_start + sweep):.1f}°  "
            f"sweep={math.degrees(sweep):.1f}°  len={arc_length:.3f} m  "
            f"dur={duration:.2f} s"
        )

    def _arc_sample(self, t):
        """Return (x, y) on the transfer arc at parameter t ∈ [0, 1]."""
        s = min_jerk(t)
        angle = lerp(self._arc_start_angle, self._arc_end_angle, s)
        return (self._arc_radius * math.cos(angle),
                self._arc_radius * math.sin(angle))

    # ============================================================== #
    #  Command callbacks
    # ============================================================== #
    def _cb_command(self, msg: String):
        if self._state != State.IDLE:
            self._publish_status(f"BUSY — in {self._state.name}, ignoring command")
            self.get_logger().warn("Planner busy, ignoring command")
            return

        parts = msg.data.strip().upper().split()
        if len(parts) != 2:
            self._publish_status("ERROR — expected 'SRC DST' e.g. 'F4 L0'")
            self.get_logger().error(f"Bad command: '{msg.data}'")
            return

        src, dst = parts
        if src not in ALL_HOLES:
            self._publish_status(f"ERROR — unknown source hole '{src}'")
            return
        if dst not in ALL_HOLES:
            self._publish_status(f"ERROR — unknown destination hole '{dst}'")
            return

        wx_s, wy_s, _ = ALL_HOLES[src]
        wx_d, wy_d, _ = ALL_HOLES[dst]
        src_ik = world_to_ik(wx_s, wy_s)
        dst_ik = world_to_ik(wx_d, wy_d)

        # --- TEST OVERRIDE: hardcode F3 IK target ---
        _F3_IK = (0.010000, 0.260000)
        if src == "F3":
            src_ik = _F3_IK
        if dst == "F3":
            dst_ik = _F3_IK

        # must be reachable
        if not self._reachable(src_ik[0], src_ik[1]):
            self._publish_status(f"ERROR — source '{src}' unreachable")
            return
        if not self._reachable(dst_ik[0], dst_ik[1]):
            self._publish_status(f"ERROR — dest '{dst}' unreachable")
            return

        self._src_name = src
        self._dst_name = dst
        self._src_ik = src_ik
        self._dst_ik = dst_ik
        self._cancelled = False

        # Reset approach (will be auto-reduced if needed per-hole)
        self._approach = self._approach_base

        self.get_logger().info(
            f"Planning: {src} ({src_ik[0]:+.4f},{src_ik[1]:+.4f}) → "
            f"{dst} ({dst_ik[0]:+.4f},{dst_ik[1]:+.4f})"
        )

        # Publish planned path for RViz
        self._publish_path_dense()

        # Start
        self._enter_state(State.MOVE_ABOVE_SRC)

    def _cb_cancel(self, _msg):
        if self._state != State.IDLE:
            self._cancelled = True
            self._publish_status("CANCELLING — suction OFF, returning HOME")
            self.get_logger().warn("Cancel requested")

    def _cb_ball_detected(self, msg: Bool):
        previous = self._ball_detected
        self._ball_detected = msg.data

        if self._state == State.SUCTION_ON and self._pick_detection_armed:
            if not msg.data:
                self._pick_seen_false = True
            elif self._pick_seen_false and not previous:
                self._pick_confirmed = True

    def _cb_pressure_raw(self, msg: Int32):
        self._latest_pressure_raw = int(msg.data)

        if self._state != State.SUCTION_ON or not self._pick_detection_armed:
            return

        if self._pick_confirmed:
            return

        now = time.monotonic()
        if now - self._pick_arm_time < self._pick_raw_fallback_grace:
            return

        if self._latest_pressure_raw >= self._pick_raw_fallback_min:
            self._pick_confirmed = True
            self.get_logger().info(
                f"  Strong pressure raw sample confirms pick (raw_u24={self._latest_pressure_raw})"
            )

    # ============================================================== #
    #  State transitions
    # ============================================================== #
    def _enter_state(self, new_state: State):
        self._state = new_state
        self.get_logger().info(f"→ {new_state.name}")
        self._publish_status(f"{new_state.name}  [{self._src_name} → {self._dst_name}]")

        if new_state == State.IDLE:
            self._seg_active = False
            return

        if new_state == State.MOVE_ABOVE_SRC:
            above_src, used = self._compute_above(self._src_ik)
            if above_src is None:
                self._publish_status("ERROR — above source unreachable (approach too high)")
                self.get_logger().error("Above source unreachable even after reducing approach.")
                self._enter_state(State.IDLE)
                return
            self._approach = used
            self._start_segment(self._current_ik, above_src, speed=self._move_speed)

        elif new_state == State.DESCEND_SRC:
            self.get_logger().info(
                f"  DESCEND target: IK ({self._src_ik[0]:+.4f},{self._src_ik[1]:+.4f})  "
                f"from ({self._current_ik[0]:+.4f},{self._current_ik[1]:+.4f})"
            )
            self._start_segment(self._current_ik, self._src_ik, speed=self._desc_speed)

        elif new_state == State.SUCTION_ON:
            self._pick_confirmed = False
            self._pick_detection_armed = False
            self._pick_seen_false = False
            self._pick_arm_time = 0.0
            self._hover_active = False

            # Phase 1: wait for arm to physically reach the target position
            # Servo stays UP until the arm has settled
            self._dwell_settled = False
            self._servo_sent = False
            self._settle_end = time.monotonic() + self._arm_settle
            # dwell_end set after settle (sensor-driven with safety timeout)
            self._dwell_end = self._settle_end + self.SERVO_SETTLE_TIME + self._ball_detect_timeout

        elif new_state == State.ASCEND_SRC:
            above_src, _ = self._compute_above(self._src_ik)
            if above_src is None:
                above_src = self._radial_point(self._src_ik,
                                               math.hypot(*self._src_ik) + 0.02)
            self._start_segment(self._current_ik, above_src, speed=self._desc_speed)

        elif new_state == State.RETRACT_SRC:
            # Pull back to a safe transfer radius in the source direction
            retract_pt = self._radial_point(self._src_ik, self.TRANSFER_RADIUS)
            self._start_segment(self._current_ik, retract_pt, speed=self._move_speed)

        elif new_state == State.TRANSFER:
            # Arc transfer: sweep at constant TRANSFER_RADIUS from source
            # angle to destination angle (avoids cutting through centre).
            self._start_arc_transfer()

        elif new_state == State.MOVE_ABOVE_DST:
            above_dst, used = self._compute_above(self._dst_ik)
            if above_dst is None:
                self._publish_status("ERROR — above dest unreachable (approach too high)")
                self.get_logger().error("Above dest unreachable even after reducing approach.")
                self._enter_state(State.IDLE)
                return
            # Note: keep the smaller of both approaches (safe)
            self._approach = min(self._approach, used)
            self._start_segment(self._current_ik, above_dst, speed=self._move_speed)

        elif new_state == State.DESCEND_DST:
            self.get_logger().info(
                f"  DESCEND target: IK ({self._dst_ik[0]:+.4f},{self._dst_ik[1]:+.4f})  "
                f"from ({self._current_ik[0]:+.4f},{self._current_ik[1]:+.4f})"
            )
            self._start_segment(self._current_ik, self._dst_ik, speed=self._desc_speed)

        elif new_state == State.SUCTION_OFF:
            # Phase 1: let physical arm settle before triggering release
            self._dwell_settled = False
            self._settle_end = time.monotonic() + self._arm_settle
            self._dwell_end = self._settle_end + (self._dwell - self._arm_settle)

        elif new_state == State.ASCEND_DST:
            above_dst, _ = self._compute_above(self._dst_ik)
            if above_dst is None:
                above_dst = self._radial_point(self._dst_ik,
                                               math.hypot(*self._dst_ik) + 0.02)
            self._start_segment(self._current_ik, above_dst, speed=self._desc_speed)

        elif new_state == State.RETRACT_DST:
            # Pull back to TRANSFER_RADIUS in the destination direction
            # before sweeping home via arc.
            retract_pt = self._radial_point(self._dst_ik, self.TRANSFER_RADIUS)
            self._start_segment(self._current_ik, retract_pt, speed=self._move_speed)

        elif new_state == State.MOVE_HOME:
            # Arc sweep at TRANSFER_RADIUS from current angle to HOME
            # angle (90°).  HOME_POS is ON the TRANSFER_RADIUS circle,
            # so the arc ends exactly at HOME — no extra radial move.
            self._start_arc_transfer(target_ik=self.HOME_POS)

    def _next_state(self):
        order = [
            State.MOVE_ABOVE_SRC,
            State.DESCEND_SRC,
            State.SUCTION_ON,
            State.ASCEND_SRC,
            State.RETRACT_SRC,
            State.TRANSFER,
            State.MOVE_ABOVE_DST,
            State.DESCEND_DST,
            State.SUCTION_OFF,
            State.ASCEND_DST,
            State.RETRACT_DST,
            State.MOVE_HOME,
            State.IDLE,
        ]
        try:
            idx = order.index(self._state)
            return order[idx + 1]
        except (ValueError, IndexError):
            return State.IDLE

    # ============================================================== #
    #  Segment execution (continuous sampling)
    # ============================================================== #
    def _start_segment(self, start, end, speed):
        if not self._reachable(end[0], end[1]):
            self.get_logger().error(
                f"Segment end unreachable: ({end[0]:+.4f},{end[1]:+.4f})"
            )
            self._enter_state(State.IDLE)
            return

        # Safety: avoid planning non-radial straight-line segments that move
        # the end-effector inside the MIN_SAFE_RADIUS (singularity zone).
        # If this segment would do that, redirect the segment to the
        # TRANSFER_RADIUS radial point in the target direction instead.
        sx, sy = start
        ex, ey = end
        sr = math.hypot(sx, sy)
        er = math.hypot(ex, ey)
        sa = math.atan2(sy, sx)
        ea = math.atan2(ey, ex)
        # consider it non-radial if angles differ significantly
        if abs((sa - ea)) > 1e-2 and er < self.MIN_SAFE_RADIUS and sr > er:
            # redirect to safe radial point on TRANSFER_RADIUS
            safe_pt = self._radial_point((ex, ey), self.TRANSFER_RADIUS)
            self.get_logger().warn(
                f"Segment to inner radius ({er:.3f}m) blocked; redirecting to safe radius {self.TRANSFER_RADIUS:.3f}m"
            )
            end = safe_pt

        dist = math.hypot(end[0] - start[0], end[1] - start[1])
        duration = max(0.10, dist / max(1e-6, speed))

        self._seg_active = True
        self._arc_active = False    # deactivate arc if any
        self._seg_start = (float(start[0]), float(start[1]))
        self._seg_end   = (float(end[0]), float(end[1]))
        self._seg_start_time = time.monotonic()
        self._seg_duration = float(duration)

        self.get_logger().info(
            f"  seg ({self._seg_start[0]:+.4f},{self._seg_start[1]:+.4f}) → "
            f"({self._seg_end[0]:+.4f},{self._seg_end[1]:+.4f})  "
            f"dist={dist:.4f} m  dur={duration:.2f} s  speed={speed:.3f}"
        )

    def _tick(self):
        if self._state == State.IDLE:
            return

        # --- cancel handling (safe) ---
        if self._cancelled:
            # suction OFF immediately
            if self._suction_on:
                self._suction(False)
            # go home
            self._cancelled = False
            self._seg_active = False
            self._arc_active = False
            self._enter_state(State.MOVE_HOME)
            return

        # --- dwell states (SUCTION_ON: three-phase; SUCTION_OFF: two-phase) ---
        if self._state in (State.SUCTION_ON, State.SUCTION_OFF):
            now = time.monotonic()

            if self._state == State.SUCTION_ON:
                # Phase 1: wait for arm to settle (servo still UP)
                if not self._dwell_settled and not self._servo_sent and now >= self._settle_end:
                    # Arm has settled — now lower the servo
                    self._servo_sent = True
                    cmd = String()
                    cmd.data = "servo_down"
                    self._suc_manual.publish(cmd)
                    self.get_logger().info("  Arm settled — servo down command sent")
                    self._servo_end = now + self.SERVO_SETTLE_TIME
                    return

                # Phase 2: wait for servo to reach down position
                if self._servo_sent and not self._dwell_settled and now >= self._servo_end:
                    # Servo has settled — turn on suction valve
                    self._dwell_settled = True
                    self._suction(True)
                    self.get_logger().info("  Servo settled — suction command sent")
                    self._pick_detection_armed = True
                    self._pick_confirmed = False
                    self._pick_seen_false = not self._ball_detected
                    self._pick_arm_time = now
                    self._wait_log_time = now + 5.0  # first reminder after 5 s
                    if self._ball_detected:
                        self.get_logger().warn(
                            "  Pressure sensor already reports PICKED after suction start; "
                            "waiting for a fresh release->pick transition"
                        )
                    self.get_logger().info(
                        "  Waiting for pressure confirmation (ball_detected or strong raw sample) "
                        "(no timeout — use /cancel_plan to abort)"
                    )
                    return

                # Phase 3: wait for pressure sensor pick confirmation
                if self._dwell_settled and self._pick_confirmed:
                    self._pick_detection_armed = False
                    # If hovering, snap back to hole centre before ascending
                    if self._hover_active:
                        self._hover_active = False
                        self._current_ik = self._hover_centre_ik
                        self._publish_ik(self._current_ik)
                    self.get_logger().info("  Pick confirmed by pressure sensor — raising servo")
                    cmd = String()
                    cmd.data = "servo_up"
                    self._suc_manual.publish(cmd)
                    self._enter_state(self._next_state())
                elif self._dwell_settled and not self._pick_confirmed:
                    elapsed_since_suction = now - self._pick_arm_time

                    # If the sensor already reported PICKED when suction
                    # started (e.g. servo pressure alone triggered it) and
                    # it is STILL picked after a 1-second grace, accept it.
                    if (not self._pick_seen_false
                            and self._ball_detected
                            and elapsed_since_suction >= 1.0):
                        self._pick_confirmed = True
                        self.get_logger().info(
                            "  Sensor continuously PICKED since suction start — accepting"
                        )
                        return

                    # After HOVER_DELAY seconds, start a circular wobble to
                    # catch the ball if the cup is slightly offset
                    if elapsed_since_suction >= self.HOVER_DELAY and not self._hover_active:
                        self._hover_active = True
                        self._hover_start_time = now
                        self._hover_centre_ik = self._src_ik
                        self.get_logger().info(
                            f"  No pick after {self.HOVER_DELAY:.1f}s — "
                            f"starting hover pattern (r={self.HOVER_RADIUS*1000:.1f}mm, "
                            f"T={self.HOVER_PERIOD:.1f}s)"
                        )

                    if self._hover_active:
                        t = now - self._hover_start_time
                        angle = 2.0 * math.pi * t / self.HOVER_PERIOD
                        cx, cy = self._hover_centre_ik
                        hx = cx + self.HOVER_RADIUS * math.cos(angle)
                        hy = cy + self.HOVER_RADIUS * math.sin(angle)
                        self._current_ik = (hx, hy)
                        self._publish_ik(self._current_ik)

                    if now >= getattr(self, '_wait_log_time', now + 1):
                        extra = " (hovering)" if self._hover_active else ""
                        self.get_logger().info(f"  Still waiting for ball pick-up …{extra}")
                        self._wait_log_time = now + 10.0
                return

            # SUCTION_OFF handling (unchanged two-phase)
            if not self._dwell_settled and now >= self._settle_end:
                # Phase 2: arm has settled — now trigger suction hardware
                self._dwell_settled = True
                self._suction(False)
                self.get_logger().info("  Arm settled — suction command sent")

            # SUCTION_OFF: still uses dwell timer (place doesn't need sensor)
            if now >= self._dwell_end:
                self._enter_state(self._next_state())
            return

        # --- arc execution (TRANSFER state) ---
        if self._arc_active:
            elapsed = time.monotonic() - self._arc_start_time
            t = 1.0 if self._arc_duration <= 1e-6 else (elapsed / self._arc_duration)
            if t >= 1.0:
                x, y = self._arc_sample(1.0)
                self._arc_active = False
            else:
                x, y = self._arc_sample(t)

            self._current_ik = (x, y)
            self._publish_ik(self._current_ik)

            if not self._arc_active:
                self._enter_state(self._next_state())
            return

        # --- segment execution ---
        if not self._seg_active:
            self._enter_state(self._next_state())
            return

        elapsed = time.monotonic() - self._seg_start_time
        t = 1.0 if self._seg_duration <= 1e-6 else (elapsed / self._seg_duration)
        if t >= 1.0:
            # finish exactly at end
            x, y = self._seg_end
            self._seg_active = False
        else:
            s = min_jerk(t)
            x = lerp(self._seg_start[0], self._seg_end[0], s)
            y = lerp(self._seg_start[1], self._seg_end[1], s)

        self._current_ik = (x, y)
        self._publish_ik(self._current_ik)

        if not self._seg_active:
            self._enter_state(self._next_state())

    # ============================================================== #
    #  Publishing helpers
    # ============================================================== #
    def _publish_ik(self, ik_xy):
        msg = Point()
        msg.x = float(ik_xy[0])
        msg.y = float(ik_xy[1])
        self._ik_pub.publish(msg)

    def _suction(self, on: bool):
        msg = Bool()
        msg.data = bool(on)
        self._suc_pub.publish(msg)
        self._suction_on = bool(on)
        self.get_logger().info(f"  Suction {'ON' if on else 'OFF'}")

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _publish_path_dense(self):
        """
        Publish a denser nav_msgs/Path matching the intended motion:
        home → above_src → src → above_src → retract_src →
        ARC(retract_src → retract_dst) → above_dst → dst →
        above_dst → home
        """
        above_src, _ = self._compute_above(self._src_ik)
        above_dst, _ = self._compute_above(self._dst_ik)
        if above_src is None or above_dst is None:
            self._publish_path_waypoints()
            return

        R = self.TRANSFER_RADIUS
        retract_src = self._radial_point(self._src_ik, R)
        retract_dst = self._radial_point(self._dst_ik, R)

        # Linear segments (before and after the arc)
        linear_pairs = [
            (self._current_ik, above_src),
            (above_src, self._src_ik),
            (self._src_ik, above_src),
            (above_src, retract_src),
        ]

        samples = []
        for a, b in linear_pairs:
            for i in range(25):
                t = i / 24.0
                s = min_jerk(t)
                samples.append((lerp(a[0], b[0], s), lerp(a[1], b[1], s)))

        # Arc segment: retract_src → retract_dst
        a_start = math.atan2(retract_src[1], retract_src[0])
        a_end   = math.atan2(retract_dst[1], retract_dst[0])
        sweep = a_end - a_start
        if sweep > math.pi:
            sweep -= 2.0 * math.pi
        elif sweep < -math.pi:
            sweep += 2.0 * math.pi
        for i in range(40):
            t = i / 39.0
            s = min_jerk(t)
            angle = a_start + sweep * s
            samples.append((R * math.cos(angle), R * math.sin(angle)))

        # Linear segments after arc: retract_dst → above_dst → dst → above_dst → retract_dst (for home arc)
        linear_pairs_2 = [
            (retract_dst, above_dst),
            (above_dst, self._dst_ik),
            (self._dst_ik, above_dst),
            (above_dst, retract_dst),  # RETRACT_DST
        ]
        for a, b in linear_pairs_2:
            for i in range(25):
                t = i / 24.0
                s = min_jerk(t)
                samples.append((lerp(a[0], b[0], s), lerp(a[1], b[1], s)))

        # Arc from retract_dst to HOME (at TRANSFER_RADIUS)
        home_angle = math.atan2(self.HOME_POS[1], self.HOME_POS[0])
        a_start = math.atan2(retract_dst[1], retract_dst[0])
        a_end = home_angle
        sweep = a_end - a_start
        if sweep > math.pi:
            sweep -= 2.0 * math.pi
        elif sweep < -math.pi:
            sweep += 2.0 * math.pi
        for i in range(40):
            t = i / 39.0
            s = min_jerk(t)
            angle = a_start + sweep * s
            samples.append((R * math.cos(angle), R * math.sin(angle)))

        path = Path()
        path.header.frame_id = "tray_base_link"
        path.header.stamp = self.get_clock().now().to_msg()

        for ix, iy in samples:
            ps = PoseStamped()
            ps.header = path.header
            wx, wy = ik_to_world(ix, iy)
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.position.z = float(IK_ORIGIN[2])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        self._path_pub.publish(path)

    def _publish_path_waypoints(self):
        """Fallback: waypoint-only path."""
        path = Path()
        path.header.frame_id = "tray_base_link"
        path.header.stamp = self.get_clock().now().to_msg()

        retract_src = self._radial_point(self._src_ik, self.TRANSFER_RADIUS)
        retract_dst = self._radial_point(self._dst_ik, self.TRANSFER_RADIUS)

        waypoints_ik = [
            self._current_ik,
            self._src_ik,
            retract_src,
            retract_dst,
            self._dst_ik,
            self.HOME_POS,
        ]

        for ix, iy in waypoints_ik:
            ps = PoseStamped()
            ps.header = path.header
            wx, wy = ik_to_world(ix, iy)
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.position.z = float(IK_ORIGIN[2])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        self._path_pub.publish(path)


# ================================================================== #
def main():
    rclpy.init()
    node = PickAndPlacePlanner()
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
