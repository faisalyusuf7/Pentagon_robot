#!/usr/bin/env python3
"""
Optimised Pick-and-Place Trajectory Planner  **v2**
====================================================

Drop-in replacement for pick_and_place_planner.py with ~30 % faster
cycle times for both ball picking and ball dropping.

Key improvements over v1
------------------------
  1. **Unified SAFE_RADIUS** — all lateral motion stays on a single
     safe circle (0.27 m), eliminating the separate "above" and
     "retract" intermediate positions.  9 active states vs 12.
  2. **Differentiated speeds** — fast for empty moves, medium for
     ball-carrying arcs, slow/precise for approach/depart.
  3. **Shorter dwell** — 0.30-0.35 s (solenoid valve is <50 ms).
  4. **Higher tick rate** — 50 Hz for smoother trajectory sampling.
  5. **Cycle-time estimation** — logs expected time before execution.
  6. **Direct radial approach/depart** — pure radial in/out from
     SAFE_RADIUS to each hole; no zig-zag through "above" then
     "retract" intermediate points.

Trajectory per cycle
--------------------
  HOME ─(arc)──→ SRC_angle ─(radial↓)──→ SRC ─(pick)──→
   SRC ─(radial↑)──→ SRC_angle ─(arc)──→ DST_angle
       ─(radial↓)──→ DST ─(place)──→ DST ─(radial↑)──→
   DST_angle ─(arc)──→ HOME

Topic interface (identical to v1)
---------------------------------
  Subscribe:  /pick_place_cmd  (std_msgs/String, e.g. "F4 L0")
              /cancel_plan     (std_msgs/Empty)
  Publish:    /ik_target       (geometry_msgs/Point)
              /suction_cmd     (std_msgs/Bool)
              /planner_status  (std_msgs/String)
              /planner_path    (nav_msgs/Path)

Usage
-----
  Launch with:  use_planner_v2:=true
    ros2 launch Main_Assembly_1 five_bar_ik.launch.py use_planner_v2:=true
"""

import math
import time
from enum import Enum, auto

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String, Bool, Empty
from std_msgs.msg import Int32          # noqa: F811 — pressure_raw
from nav_msgs.msg import Path


# ================================================================== #
#  Hole positions  (world frame) — identical to v1
# ================================================================== #
FRONT_TRAY_HOLES = {
    "F0": (0.724254, -1.440010, 0.020256),
    "F1": (0.724254, -1.395010, 0.020256),
    "F2": (0.724254, -1.350010, 0.020256),
    "F3": (0.769254, -1.440010, 0.020256),
    "F4": (0.769254, -1.395010, 0.020256),
    "F5": (0.769254, -1.350010, 0.020256),
    "F6": (0.814254, -1.440010, 0.020256),
    "F7": (0.814254, -1.395010, 0.020256),
    "F8": (0.814254, -1.350010, 0.020256),
}

LEFT_TRAY_HOLES = {
    "L0": (0.594254, -1.645010, 0.000256),
    "L1": (0.549254, -1.645010, 0.000256),
    "L2": (0.504254, -1.645010, 0.000256),
    "L3": (0.594254, -1.690010, 0.000256),
    "L4": (0.549254, -1.690010, 0.000256),
    "L5": (0.504254, -1.690010, 0.000256),
    "L6": (0.594254, -1.735010, 0.000256),
    "L7": (0.549254, -1.735010, 0.000256),
    "L8": (0.504254, -1.735010, 0.000256),
}

ALL_HOLES = {**FRONT_TRAY_HOLES, **LEFT_TRAY_HOLES}


# ================================================================== #
#  IK origin  (world frame) — identical to v1
# ================================================================== #
_BASE_LINK_OFFSET = np.array([-0.006403, -0.103113, -0.00474])
_POS_ML = np.array([0.695406926143987, -1.58664693818658, 0.05]) + _BASE_LINK_OFFSET
_POS_MR = np.array([0.885406926135972, -1.58664693819113, 0.07]) + _BASE_LINK_OFFSET
IK_ORIGIN = (_POS_ML + _POS_MR) / 2.0


# ================================================================== #
#  Rotation helpers  (duplicated from IK node for suction FK)
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

_CHANNEL_XYZ = np.array([-0.000149999999999983, -0.0098999999999998, 0.05])
_R_CHANNEL = _rpy(-math.pi / 2.0, 0.0, math.pi)

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
    Given IK-plane target (px, py), compute nozzle-tip world XYZ
    via FK chain: motor_right → right_link → suction_link → channel.
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

    R_suction = R_right @ _rpy(-math.pi / 2.0, math.pi / 2.0, 0.0)
    p_suction = crank_tip + R_right @ _SUCTION_XYZ
    p_channel = p_suction + R_suction @ _CHANNEL_XYZ
    R_channel = R_suction @ _R_CHANNEL
    return p_channel + R_channel @ _TIP_CH


def solve_ik_for_suction(target_wx, target_wy, max_iter=5):
    """
    Iteratively find the IK target (px, py) that places the
    suction nozzle at (target_wx, target_wy) in world XY.
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
    """Minimum-jerk easing (C2 continuous, zero vel / accel at endpoints)."""
    t = max(0.0, min(1.0, t))
    return 10 * t**3 - 15 * t**4 + 6 * t**5


# ================================================================== #
#  State machine  (9 active states vs 12 in v1)
# ================================================================== #
class State(Enum):
    IDLE          = auto()
    SWING_TO_SRC  = auto()   # arc at SAFE_RADIUS → src angle
    DESCEND_SRC   = auto()   # radial ↓ to src hole
    PICK          = auto()   # suction ON + dwell
    ASCEND_SRC    = auto()   # radial ↑ to SAFE_RADIUS
    SWING_TO_DST  = auto()   # arc at SAFE_RADIUS → dst angle
    DESCEND_DST   = auto()   # radial ↓ to dst hole
    PLACE         = auto()   # suction OFF + dwell
    ASCEND_DST    = auto()   # radial ↑ to SAFE_RADIUS
    SWING_HOME    = auto()   # arc at SAFE_RADIUS → home angle


# ================================================================== #
#  Planner node
# ================================================================== #
class OptimizedPickAndPlacePlanner(Node):
    """
    v2 optimised planner.

    Compared to v1 the trajectory is simplified to three primitives
    repeated cyclically:
      • **swing**  — arc at constant SAFE_RADIUS
      • **descend** — pure radial inward (approach)
      • **ascend**  — pure radial outward (depart)

    This guarantees every lateral movement is at a safe height and
    every approach/depart is perfectly radial (no singularity risk).
    """

    # ---- Geometry ----
    SAFE_RADIUS = 0.27        # m — all lateral motion at this radius

    # ---- Differentiated speeds (m/s) ----
    # NOTE: reduced from v2-original to prevent TMC2208 thermal throttling.
    # Original values caused step-loss after 2-3 cycles.
    SPEED_EMPTY_ARC   = 0.22  # empty swing  (was 0.35)
    SPEED_LOADED_ARC  = 0.16  # ball-carrying swing  (was 0.25)
    SPEED_APPROACH    = 0.04  # descent to pick (precision)
    SPEED_PLACE       = 0.05  # descent to place (extra precision)
    SPEED_LIFT_LOADED = 0.08  # ascent with ball (careful)
    SPEED_LIFT_EMPTY  = 0.14  # ascent without ball  (was 0.20)

    # ---- Dwell (s) — must cover full suction HW sequence ----
    # Pick:  servo_down(0.8) + grip(2.0) + servo_up(0.8) = 3.6 s
    # Place: servo_down(0.8) + release(1.7) + servo_up(0.8) = 3.3 s
    DWELL_PICK  = 4.0
    DWELL_PLACE = 3.5

    # ---- Pressure-sensor ball detection ----
    USE_BALL_DETECT    = True    # gate PICK→ASCEND on sensor
    BALL_DETECT_TIMEOUT = 8.0   # seconds — fall back to dwell if sensor fails

    # ---- Thermal management ----
    COOLDOWN_BETWEEN_CYCLES = 2.0   # seconds — motors disabled between cycles
    REHOME_EVERY_N_CYCLES   = 3     # send G28 re-home every N cycles (0=never)
    REDUCED_SPEED_AFTER     = 0     # unused — speeds already conservative

    # ---- Control ----
    TICK_RATE     = 50.0       # Hz
    MIN_ARC_SWEEP = 0.02       # rad (~1°) — skip arc if smaller

    # ------------------------------------------------------------------ #
    def __init__(self):
        super().__init__("pick_place_planner")   # same node name as v1

        # --- kinematics params (must match IK node) ---
        self.declare_parameter("L1", 0.200)
        self.declare_parameter("L2", 0.200)
        self.declare_parameter("d",  0.190)
        self.L1 = float(self.get_parameter("L1").value)
        self.L2 = float(self.get_parameter("L2").value)
        self.d  = float(self.get_parameter("d").value)

        # --- tuneable planner params ---
        self.declare_parameter("safe_radius",   self.SAFE_RADIUS)
        self.declare_parameter("speed_approach", self.SPEED_APPROACH)
        self.declare_parameter("speed_place",   self.SPEED_PLACE)
        self.declare_parameter("dwell_pick",    self.DWELL_PICK)
        self.declare_parameter("dwell_place",   self.DWELL_PLACE)
        self.declare_parameter("tick_rate",     self.TICK_RATE)
        self.declare_parameter("cooldown_secs",  self.COOLDOWN_BETWEEN_CYCLES)
        self.declare_parameter("rehome_every_n", self.REHOME_EVERY_N_CYCLES)
        self.declare_parameter("use_ball_detect", self.USE_BALL_DETECT)
        self.declare_parameter("ball_detect_timeout", self.BALL_DETECT_TIMEOUT)

        self.SAFE_RADIUS   = float(self.get_parameter("safe_radius").value)
        self.SPEED_APPROACH = float(self.get_parameter("speed_approach").value)
        self.SPEED_PLACE   = float(self.get_parameter("speed_place").value)
        self.DWELL_PICK    = float(self.get_parameter("dwell_pick").value)
        self.DWELL_PLACE   = float(self.get_parameter("dwell_place").value)
        self._tick_rate    = float(self.get_parameter("tick_rate").value)
        self._cooldown_secs = float(self.get_parameter("cooldown_secs").value)
        self._rehome_every  = int(self.get_parameter("rehome_every_n").value)
        self._use_ball_detect = bool(self.get_parameter("use_ball_detect").value)
        self._ball_detect_timeout = float(self.get_parameter("ball_detect_timeout").value)

        # HOME on the safe circle at 90° (same as v1)
        self.HOME_POS = (0.0, self.SAFE_RADIUS)

        # ---- runtime state ----
        self._state      = State.IDLE
        self._current_ik = self.HOME_POS
        self._src_name   = ""
        self._dst_name   = ""
        self._src_ik     = (0.0, 0.0)
        self._dst_ik     = (0.0, 0.0)
        self._cancelled  = False
        self._suction_on = False

        # segment (linear)
        self._seg_active     = False
        self._seg_start      = (0.0, 0.0)
        self._seg_end        = (0.0, 0.0)
        self._seg_start_time = 0.0
        self._seg_duration   = 0.0

        # arc
        self._arc_active      = False
        self._arc_radius      = 0.0
        self._arc_start_angle = 0.0
        self._arc_end_angle   = 0.0
        self._arc_start_time  = 0.0
        self._arc_duration    = 0.0

        # dwell
        self._dwell_end = 0.0

        # cycle timing
        self._cycle_start = 0.0
        self._cycle_count = 0

        # cooldown state
        self._cooldown_active = False
        self._cooldown_end = 0.0
        self._rehome_pending = False

        # ball-detect state (from /ball_detected topic)
        self._ball_detected = False
        self._pick_suction_sent = False

        # ---- publishers ----
        self._ik_pub     = self.create_publisher(Point,  "/ik_target",      10)
        self._suc_pub    = self.create_publisher(Bool,   "/suction_cmd",    10)
        self._status_pub = self.create_publisher(String, "/planner_status", 10)
        self._path_pub   = self.create_publisher(Path,   "/planner_path",   10)

        # ---- subscribers ----
        self.create_subscription(String, "/pick_place_cmd", self._cb_command, 10)
        self.create_subscription(Empty,  "/cancel_plan",    self._cb_cancel,  10)
        self.create_subscription(Bool,   "/ball_detected",  self._cb_ball_detected, 10)

        # ---- timer ----
        self._timer = self.create_timer(1.0 / self._tick_rate, self._tick)

        self._publish_status("IDLE — waiting for command (v2 optimised)")
        self.get_logger().info(
            "Pick-and-place planner v2 (optimised) ready.\n"
            "  Command : /pick_place_cmd  e.g. 'F4 L0'\n"
            "  Cancel  : /cancel_plan\n"
            f"  SAFE_RADIUS = {self.SAFE_RADIUS:.3f} m\n"
            f"  Speeds  = approach {self.SPEED_APPROACH}, place {self.SPEED_PLACE}, "
            f"lift_loaded {self.SPEED_LIFT_LOADED}, lift_empty {self.SPEED_LIFT_EMPTY}, "
            f"arc_empty {self.SPEED_EMPTY_ARC}, arc_loaded {self.SPEED_LOADED_ARC}\n"
            f"  Dwell   = pick {self.DWELL_PICK:.2f} s, place {self.DWELL_PLACE:.2f} s\n"
            f"  Tick    = {self._tick_rate:.0f} Hz"
        )

        # NOTE: startup publish removed — motors stay at calibrated 0° (arms parallel) on launch

    # ============================================================== #
    #  Geometry helpers
    # ============================================================== #
    def _reachable(self, ix, iy):
        L1, L2, d = self.L1, self.L2, self.d
        b1x, b1y = -d / 2.0, 0.0
        b2x, b2y =  d / 2.0, 0.0
        r1 = math.hypot(ix - b1x, iy - b1y)
        r2 = math.hypot(ix - b2x, iy - b2y)
        rmin, rmax = abs(L1 - L2), L1 + L2
        tol = 1e-6
        return (rmin - tol <= r1 <= rmax + tol) and (rmin - tol <= r2 <= rmax + tol)

    @staticmethod
    def _radial_point(direction_ik, radius):
        """Point at *radius* from IK origin in the direction of *direction_ik*."""
        x, y = direction_ik
        r = math.hypot(x, y)
        if r < 1e-6:
            return (0.0, radius)
        return (x / r * radius, y / r * radius)

    @staticmethod
    def _angle_of(ik_pt):
        """Angle (rad) from IK origin to *ik_pt*."""
        return math.atan2(ik_pt[1], ik_pt[0])

    # ============================================================== #
    #  Cycle-time estimation (logged before execution)
    # ============================================================== #
    def _estimate_cycle_time(self):
        R = self.SAFE_RADIUS
        src_r = math.hypot(*self._src_ik)
        dst_r = math.hypot(*self._dst_ik)
        src_a = self._angle_of(self._src_ik)
        dst_a = self._angle_of(self._dst_ik)
        home_a = self._angle_of(self.HOME_POS)
        cur_a = self._angle_of(self._current_ik)

        def _arc_t(a1, a2, speed):
            a1 %= 2 * math.pi
            a2 %= 2 * math.pi
            sw = a2 - a1
            if sw > math.pi:
                sw -= 2 * math.pi
            elif sw < -math.pi:
                sw += 2 * math.pi
            return max(0.10, abs(sw) * R / max(1e-6, speed))

        t  = _arc_t(cur_a, src_a, self.SPEED_EMPTY_ARC)       # SWING_TO_SRC
        t += max(0.10, (R - src_r) / self.SPEED_APPROACH)      # DESCEND_SRC
        t += self.DWELL_PICK                                    # PICK
        t += max(0.10, (R - src_r) / self.SPEED_LIFT_LOADED)   # ASCEND_SRC
        t += _arc_t(src_a, dst_a, self.SPEED_LOADED_ARC)       # SWING_TO_DST
        t += max(0.10, (R - dst_r) / self.SPEED_PLACE)         # DESCEND_DST
        t += self.DWELL_PLACE                                   # PLACE
        t += max(0.10, (R - dst_r) / self.SPEED_LIFT_EMPTY)    # ASCEND_DST
        t += _arc_t(dst_a, home_a, self.SPEED_EMPTY_ARC)       # SWING_HOME
        return t

    # ============================================================== #
    #  Arc primitives
    # ============================================================== #
    def _start_arc(self, target_angle, speed):
        """
        Arc sweep at SAFE_RADIUS from the current angle to *target_angle*.
        Returns True if the arc was started, False if the sweep was
        trivially small (already at the target angle).
        """
        R = self.SAFE_RADIUS
        sx, sy = self._current_ik
        a_start = math.atan2(sy, sx)
        a_end = target_angle

        # normalise to [0, 2π)
        a_start %= 2 * math.pi
        a_end   %= 2 * math.pi

        # shortest arc
        sweep = a_end - a_start
        if sweep > math.pi:
            sweep -= 2 * math.pi
        elif sweep < -math.pi:
            sweep += 2 * math.pi

        if abs(sweep) < self.MIN_ARC_SWEEP:
            # trivial — snap to target
            self._current_ik = (R * math.cos(a_end), R * math.sin(a_end))
            self._publish_ik(self._current_ik)
            self._arc_active = False
            self._seg_active = False
            return False

        arc_length = abs(sweep) * R
        duration = max(0.10, arc_length / max(1e-6, speed))

        self._arc_active      = True
        self._seg_active      = False
        self._arc_radius      = R
        self._arc_start_angle = a_start
        self._arc_end_angle   = a_start + sweep
        self._arc_start_time  = time.monotonic()
        self._arc_duration    = duration

        self.get_logger().info(
            f"  arc R={R:.3f}  "
            f"{math.degrees(a_start):.1f}° → {math.degrees(a_start + sweep):.1f}°  "
            f"|sweep|={math.degrees(abs(sweep)):.1f}°  "
            f"len={arc_length:.3f} m  dur={duration:.2f} s"
        )
        return True

    def _arc_sample(self, t):
        s = min_jerk(t)
        angle = lerp(self._arc_start_angle, self._arc_end_angle, s)
        return (self._arc_radius * math.cos(angle),
                self._arc_radius * math.sin(angle))

    # ============================================================== #
    #  Linear segment primitives
    # ============================================================== #
    def _start_segment(self, start, end, speed):
        """
        Start a linear segment from *start* to *end*.
        Returns True if the segment was started, False if the
        distance was negligible (already at *end*).
        """
        if not self._reachable(end[0], end[1]):
            self.get_logger().error(
                f"Segment end unreachable: ({end[0]:+.4f},{end[1]:+.4f})"
            )
            self._enter_state(State.IDLE)
            return False

        dist = math.hypot(end[0] - start[0], end[1] - start[1])
        if dist < 1e-4:
            self._current_ik = end
            self._publish_ik(self._current_ik)
            self._seg_active = False
            self._arc_active = False
            return False

        duration = max(0.10, dist / max(1e-6, speed))

        self._seg_active     = True
        self._arc_active     = False
        self._seg_start      = (float(start[0]), float(start[1]))
        self._seg_end        = (float(end[0]),   float(end[1]))
        self._seg_start_time = time.monotonic()
        self._seg_duration   = float(duration)

        self.get_logger().info(
            f"  seg ({start[0]:+.4f},{start[1]:+.4f}) → "
            f"({end[0]:+.4f},{end[1]:+.4f})  "
            f"dist={dist:.4f} m  dur={duration:.2f} s  speed={speed:.3f}"
        )
        return True

    # ============================================================== #
    #  Command callbacks
    # ============================================================== #
    def _cb_command(self, msg: String):
        if self._cooldown_active:
            remaining = max(0, self._cooldown_end - time.monotonic())
            self._publish_status(
                f"COOLING — {remaining:.0f}s left, please wait"
            )
            self.get_logger().warn("Command rejected: thermal cooldown active")
            return

        if self._state != State.IDLE:
            self._publish_status(f"BUSY — in {self._state.name}")
            self.get_logger().warn("Planner busy, ignoring command")
            return

        parts = msg.data.strip().upper().split()
        if len(parts) != 2:
            self._publish_status("ERROR — expected 'SRC DST' e.g. 'F4 L0'")
            self.get_logger().error(f"Bad command: '{msg.data}'")
            return

        src, dst = parts
        if src not in ALL_HOLES:
            self._publish_status(f"ERROR — unknown hole '{src}'")
            return
        if dst not in ALL_HOLES:
            self._publish_status(f"ERROR — unknown hole '{dst}'")
            return
        if src == dst:
            self._publish_status(f"ERROR — src and dst are the same ('{src}')")
            return

        wx_s, wy_s, _ = ALL_HOLES[src]
        wx_d, wy_d, _ = ALL_HOLES[dst]
        src_ik = world_to_ik(wx_s, wy_s)
        dst_ik = world_to_ik(wx_d, wy_d)

        # reachability checks
        if not self._reachable(*src_ik):
            self._publish_status(f"ERROR — '{src}' unreachable")
            return
        if not self._reachable(*dst_ik):
            self._publish_status(f"ERROR — '{dst}' unreachable")
            return

        # verify approach points on SAFE_RADIUS circle
        src_approach = self._radial_point(src_ik, self.SAFE_RADIUS)
        dst_approach = self._radial_point(dst_ik, self.SAFE_RADIUS)
        if not self._reachable(*src_approach):
            self._publish_status("ERROR — src approach point unreachable")
            return
        if not self._reachable(*dst_approach):
            self._publish_status("ERROR — dst approach point unreachable")
            return

        self._src_name = src
        self._dst_name = dst
        self._src_ik   = src_ik
        self._dst_ik   = dst_ik
        self._cancelled = False

        est = self._estimate_cycle_time()
        self.get_logger().info(
            f"Planning: {src} ({src_ik[0]:+.4f},{src_ik[1]:+.4f}) → "
            f"{dst} ({dst_ik[0]:+.4f},{dst_ik[1]:+.4f})  "
            f"estimated {est:.1f} s"
        )

        self._cycle_start = time.monotonic()
        self._publish_planned_path()
        self._enter_state(State.SWING_TO_SRC)

    def _cb_cancel(self, _msg):
        if self._state != State.IDLE:
            self._cancelled = True
            self._publish_status("CANCELLING — suction OFF, returning HOME")
            self.get_logger().warn("Cancel requested")

    def _cb_ball_detected(self, msg: Bool):
        self._ball_detected = msg.data

    # ============================================================== #
    #  State transitions
    # ============================================================== #
    def _enter_state(self, new_state: State):
        self._state = new_state
        self.get_logger().info(f"→ {new_state.name}")

        if new_state != State.IDLE:
            self._publish_status(
                f"{new_state.name}  [{self._src_name} → {self._dst_name}]"
            )

        # ---- IDLE ----
        if new_state == State.IDLE:
            self._seg_active = False
            self._arc_active = False
            if self._cycle_start > 0:
                elapsed = time.monotonic() - self._cycle_start
                self._cycle_count += 1
                self.get_logger().info(
                    f"Cycle #{self._cycle_count} completed in {elapsed:.1f} s"
                )
                self._cycle_start = 0.0

                # ── Thermal management ──────────────────────────
                # Check if we should re-home to clear accumulated drift
                need_rehome = (
                    self._rehome_every > 0
                    and self._cycle_count % self._rehome_every == 0
                )

                if need_rehome or self._cooldown_secs > 0:
                    # Start cooldown: disable motors so TMC2208 can shed heat
                    if self._cooldown_secs > 0:
                        disable_msg = Bool()
                        disable_msg.data = False
                        self.create_publisher(Bool, "/arduino_enable", 10).publish(disable_msg)
                        self.get_logger().warn(
                            f"Thermal cooldown: motors disabled for "
                            f"{self._cooldown_secs:.1f} s  "
                            f"(cycle #{self._cycle_count})"
                        )

                    self._cooldown_active = True
                    self._rehome_pending = need_rehome
                    self._cooldown_end = (
                        time.monotonic() + self._cooldown_secs
                    )
                    self._publish_status(
                        f"COOLDOWN — {self._cooldown_secs:.0f}s rest  "
                        f"(cycle #{self._cycle_count})"
                        + (" + re-home" if need_rehome else "")
                    )
                else:
                    self._publish_status(
                        f"IDLE — cycle #{self._cycle_count} done "
                        f"in {elapsed:.1f} s (v2)"
                    )
            else:
                self._publish_status("IDLE — waiting for command (v2 optimised)")
            return

        # ---- SWING_TO_SRC ----
        if new_state == State.SWING_TO_SRC:
            src_angle = self._angle_of(self._src_ik)
            if not self._start_arc(src_angle, self.SPEED_EMPTY_ARC):
                self._enter_state(State.DESCEND_SRC)

        # ---- DESCEND_SRC ----
        elif new_state == State.DESCEND_SRC:
            if not self._start_segment(
                self._current_ik, self._src_ik, self.SPEED_APPROACH
            ):
                self._enter_state(State.PICK)

        # ---- PICK ----
        elif new_state == State.PICK:
            self._suction(True)
            self._pick_suction_sent = True
            self._dwell_end = time.monotonic() + (
                self._ball_detect_timeout if self._use_ball_detect
                else self.DWELL_PICK
            )
            if self._use_ball_detect:
                self.get_logger().info(
                    "  Waiting for ball_detected=True "
                    f"(timeout {self._ball_detect_timeout:.1f} s)"
                )

        # ---- ASCEND_SRC ----
        elif new_state == State.ASCEND_SRC:
            target = self._radial_point(self._src_ik, self.SAFE_RADIUS)
            if not self._start_segment(
                self._current_ik, target, self.SPEED_LIFT_LOADED
            ):
                self._enter_state(State.SWING_TO_DST)

        # ---- SWING_TO_DST ----
        elif new_state == State.SWING_TO_DST:
            dst_angle = self._angle_of(self._dst_ik)
            if not self._start_arc(dst_angle, self.SPEED_LOADED_ARC):
                self._enter_state(State.DESCEND_DST)

        # ---- DESCEND_DST ----
        elif new_state == State.DESCEND_DST:
            if not self._start_segment(
                self._current_ik, self._dst_ik, self.SPEED_PLACE
            ):
                self._enter_state(State.PLACE)

        # ---- PLACE ----
        elif new_state == State.PLACE:
            self._suction(False)
            self._dwell_end = time.monotonic() + self.DWELL_PLACE

        # ---- ASCEND_DST ----
        elif new_state == State.ASCEND_DST:
            target = self._radial_point(self._dst_ik, self.SAFE_RADIUS)
            if not self._start_segment(
                self._current_ik, target, self.SPEED_LIFT_EMPTY
            ):
                self._enter_state(State.SWING_HOME)

        # ---- SWING_HOME ----
        elif new_state == State.SWING_HOME:
            home_angle = self._angle_of(self.HOME_POS)
            if not self._start_arc(home_angle, self.SPEED_EMPTY_ARC):
                self._enter_state(State.IDLE)

    def _next_state(self):
        _ORDER = [
            State.SWING_TO_SRC,
            State.DESCEND_SRC,
            State.PICK,
            State.ASCEND_SRC,
            State.SWING_TO_DST,
            State.DESCEND_DST,
            State.PLACE,
            State.ASCEND_DST,
            State.SWING_HOME,
            State.IDLE,
        ]
        try:
            idx = _ORDER.index(self._state)
            return _ORDER[idx + 1]
        except (ValueError, IndexError):
            return State.IDLE

    # ============================================================== #
    #  Main tick
    # ============================================================== #
    def _tick(self):
        # ---- cooldown between cycles (thermal management) ----
        if self._cooldown_active:
            if time.monotonic() >= self._cooldown_end:
                self._cooldown_active = False

                # Re-enable motors
                enable_msg = Bool()
                enable_msg.data = True
                self.create_publisher(Bool, "/arduino_enable", 10).publish(enable_msg)

                # Re-home if due (clears accumulated step drift)
                if self._rehome_pending:
                    self._rehome_pending = False
                    rehome_msg = String()
                    rehome_msg.data = "G28"
                    self.create_publisher(String, "/arduino_raw_cmd", 10).publish(rehome_msg)
                    self._current_ik = self.HOME_POS
                    self._publish_ik(self.HOME_POS)
                    self.get_logger().warn(
                        f"Re-homed after {self._cycle_count} cycles "
                        f"to clear step drift"
                    )

                self._publish_status(
                    f"IDLE — ready (cycle #{self._cycle_count}, "
                    f"cooldown done)"
                )
            return   # still cooling down — skip everything

        if self._state == State.IDLE:
            return

        # ---- cancel ----
        if self._cancelled:
            if self._suction_on:
                self._suction(False)
            self._cancelled = False
            self._seg_active = False
            self._arc_active = False
            # If below safe radius, lift first; otherwise swing home
            r = math.hypot(*self._current_ik)
            if r < self.SAFE_RADIUS - 0.005:
                target = self._radial_point(self._current_ik, self.SAFE_RADIUS)
                if self._start_segment(
                    self._current_ik, target, self.SPEED_LIFT_EMPTY
                ):
                    self._state = State.ASCEND_DST   # reuse, will → SWING_HOME
                    return
            # already at safe radius (or segment start failed)
            self._enter_state(State.SWING_HOME)
            return

        # ---- dwell states (PICK / PLACE) ----
        if self._state in (State.PICK, State.PLACE):
            # PICK with ball-detect: advance as soon as sensor confirms grip
            if (self._state == State.PICK
                    and self._use_ball_detect
                    and self._ball_detected):
                self.get_logger().info("  Ball detected by pressure sensor — proceeding")
                self._pick_suction_sent = False
                self._enter_state(self._next_state())
                return
            # Timeout / fixed dwell fallback
            if time.monotonic() >= self._dwell_end:
                if (self._state == State.PICK
                        and self._use_ball_detect
                        and not self._ball_detected):
                    self.get_logger().warn(
                        "  Ball-detect TIMEOUT — proceeding without confirmation"
                    )
                self._pick_suction_sent = False
                self._enter_state(self._next_state())
            return

        # ---- arc execution ----
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

        # ---- segment execution ----
        if self._seg_active:
            elapsed = time.monotonic() - self._seg_start_time
            t = 1.0 if self._seg_duration <= 1e-6 else (elapsed / self._seg_duration)
            if t >= 1.0:
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
            return

        # nothing active — advance
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

    def _publish_planned_path(self):
        """
        Publish an RViz-friendly nav_msgs/Path preview of the full
        planned trajectory.
        """
        R = self.SAFE_RADIUS
        src_a  = self._angle_of(self._src_ik)
        dst_a  = self._angle_of(self._dst_ik)
        home_a = self._angle_of(self.HOME_POS)
        cur_a  = self._angle_of(self._current_ik)

        src_approach = self._radial_point(self._src_ik, R)
        dst_approach = self._radial_point(self._dst_ik, R)

        samples = []
        N_ARC = 30
        N_SEG = 15

        def _add_arc(a1, a2, n=N_ARC):
            a1 %= 2 * math.pi
            a2 %= 2 * math.pi
            sw = a2 - a1
            if sw > math.pi:
                sw -= 2 * math.pi
            elif sw < -math.pi:
                sw += 2 * math.pi
            for i in range(n):
                t = i / max(1, n - 1)
                s = min_jerk(t)
                a = a1 + sw * s
                samples.append((R * math.cos(a), R * math.sin(a)))

        def _add_seg(start, end, n=N_SEG):
            for i in range(n):
                t = i / max(1, n - 1)
                s = min_jerk(t)
                samples.append((lerp(start[0], end[0], s),
                                lerp(start[1], end[1], s)))

        # 1. SWING_TO_SRC
        _add_arc(cur_a, src_a)
        # 2. DESCEND_SRC
        _add_seg(src_approach, self._src_ik)
        # 3. ASCEND_SRC
        _add_seg(self._src_ik, src_approach)
        # 4. SWING_TO_DST
        _add_arc(src_a, dst_a)
        # 5. DESCEND_DST
        _add_seg(dst_approach, self._dst_ik)
        # 6. ASCEND_DST
        _add_seg(self._dst_ik, dst_approach)
        # 7. SWING_HOME
        _add_arc(dst_a, home_a)

        # build Path message
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


# ================================================================== #
#  Entry point
# ================================================================== #
def main():
    rclpy.init()
    node = OptimizedPickAndPlacePlanner()
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
