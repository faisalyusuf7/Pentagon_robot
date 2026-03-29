#!/usr/bin/env python3
"""
5-Bar Linkage 2D Visualizer GUI
=================================
Interactive matplotlib GUI to verify IK angles visually before
sending to real hardware.

Features:
  • Full 2D drawing of the 5-bar linkage (motors, cranks, couplers, EE)
  • All hole positions drawn on both trays (Front + Left)
  • Click a hole button or click on the canvas to move the arm
  • Smooth animation between positions
  • Real-time display: IK angles, stepper angles, URDF angles, step counts
  • Optional ROS 2 publishing (auto-detected, works standalone too)

Usage:
    python3 linkage_gui.py              # standalone (no ROS)
    ros2 run Main_Assembly_1 linkage_gui.py   # with ROS publishing

Controls:
    • Left-click on canvas:  move end-effector there
    • Hole buttons:          jump to that hole position
    • Home button:           return to (0, 250mm)
    • Animation slider:      adjust move speed
"""

import math
import sys
import time
import threading
import numpy as np

import tkinter as tk
from tkinter import ttk

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Circle, FancyArrowPatch
import matplotlib.patches as mpatches

# ================================================================== #
#  Robot Geometry (must match ROS IK + Arduino)
# ================================================================== #
L1_MM = 200.0   # crank length (mm)
L2_MM = 200.0   # coupler length (mm)
D_MM  = 190.0   # base separation (mm)

STEPS_PER_REV    = 200.0
MICROSTEPS       = 16.0
STEPS_PER_DEGREE = (STEPS_PER_REV * MICROSTEPS) / 360.0   # 8.8889

# Motor positions in IK frame
B1X, B1Y = -D_MM / 2.0, 0.0   # left motor
B2X, B2Y =  D_MM / 2.0, 0.0   # right motor

# World-frame constants (for hole coordinate conversion)
_BASE_LINK_OFFSET = np.array([-0.006403, -0.103113, -0.00474])
_POS_ML = np.array([0.695406926143987, -1.58664693818658, 0.05]) + _BASE_LINK_OFFSET
_POS_MR = np.array([0.885406926135972, -1.58664693819113, 0.07]) + _BASE_LINK_OFFSET
IK_ORIGIN = (_POS_ML + _POS_MR) / 2.0

# Hole positions in world frame (x, y)
FRONT_HOLES = {
    "F0": (0.724254, -1.440010), "F1": (0.724254, -1.395010), "F2": (0.724254, -1.350010),
    "F3": (0.769254, -1.440010), "F4": (0.769254, -1.395010), "F5": (0.769254, -1.350010),
    "F6": (0.814254, -1.440010), "F7": (0.814254, -1.395010), "F8": (0.814254, -1.350010),
}
LEFT_HOLES = {
    "L0": (0.594254, -1.600010), "L1": (0.549254, -1.600010), "L2": (0.504254, -1.600010),
    "L3": (0.594254, -1.645010), "L4": (0.549254, -1.645010), "L5": (0.504254, -1.645010),
    "L6": (0.594254, -1.690010), "L7": (0.549254, -1.690010), "L8": (0.504254, -1.690010),
}
ALL_HOLES = {**FRONT_HOLES, **LEFT_HOLES}


# ================================================================== #
#  IK / FK
# ================================================================== #
def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def ik_solve(px_mm, py_mm):
    """IK in mm. Returns (theta_left_deg, theta_right_deg) or None."""
    r1 = math.hypot(px_mm - B1X, py_mm - B1Y)
    r2 = math.hypot(px_mm - B2X, py_mm - B2Y)

    rmin, rmax = abs(L1_MM - L2_MM), L1_MM + L2_MM
    if r1 < rmin - 0.1 or r1 > rmax + 0.1:
        return None
    if r2 < rmin - 0.1 or r2 > rmax + 0.1:
        return None

    phi1 = math.atan2(py_mm - B1Y, px_mm - B1X)
    ca1  = _clamp((L1_MM**2 + r1**2 - L2_MM**2) / (2 * L1_MM * r1), -1, 1)
    th1  = phi1 + math.acos(ca1)

    phi2 = math.atan2(py_mm - B2Y, px_mm - B2X)
    ca2  = _clamp((L1_MM**2 + r2**2 - L2_MM**2) / (2 * L1_MM * r2), -1, 1)
    th2  = phi2 - math.acos(ca2)

    return math.degrees(th1), math.degrees(th2)


def fk_full(th1_deg, th2_deg):
    """
    Full FK: given IK-frame motor angles (degrees),
    return dict with all joint positions (mm) for drawing.
    """
    th1 = math.radians(th1_deg)
    th2 = math.radians(th2_deg)

    # Crank tips (elbow joints)
    e1x = B1X + L1_MM * math.cos(th1)
    e1y = B1Y + L1_MM * math.sin(th1)
    e2x = B2X + L1_MM * math.cos(th2)
    e2y = B2Y + L1_MM * math.sin(th2)

    # End-effector: circle-circle intersection of couplers
    dx = e2x - e1x
    dy = e2y - e1y
    dist = math.hypot(dx, dy)

    if dist < 1e-9 or dist > 2 * L2_MM:
        return None

    a = dist / 2.0
    h_sq = L2_MM * L2_MM - a * a
    if h_sq < 0:
        return None
    h = math.sqrt(h_sq)

    mx = (e1x + e2x) / 2.0
    my = (e1y + e2y) / 2.0

    # Pick elbow-up solution (larger Y)
    ee_x = mx - h * dy / dist
    ee_y = my + h * dx / dist

    return {
        "motor_L": (B1X, B1Y),
        "motor_R": (B2X, B2Y),
        "elbow_L": (e1x, e1y),
        "elbow_R": (e2x, e2y),
        "ee": (ee_x, ee_y),
    }


def world_to_ik_mm(wx, wy):
    """World (m) → IK plane (mm). Clamp Y >= 0.1mm."""
    ix = (wx - IK_ORIGIN[0]) * 1000.0
    iy = (wy - IK_ORIGIN[1]) * 1000.0
    iy = max(0.1, iy)
    return ix, iy


def ik_to_urdf(ik_deg):
    return 90.0 - ik_deg


# ================================================================== #
#  Try to import ROS 2 (optional)
# ================================================================== #
_ros_available = False
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Point
    _ros_available = True
except ImportError:
    pass


# ================================================================== #
#  Main GUI Class
# ================================================================== #
class LinkageGUI:
    """Interactive 5-bar linkage visualizer."""

    # colours
    COL_BASE    = "#333333"
    COL_CRANK_L = "#2196F3"   # blue
    COL_CRANK_R = "#F44336"   # red
    COL_COUPL_L = "#64B5F6"   # light blue
    COL_COUPL_R = "#EF9A9A"   # light red
    COL_EE      = "#4CAF50"   # green
    COL_FRONT   = "#FF9800"   # orange
    COL_LEFT    = "#9C27B0"   # purple
    COL_WORKSPACE = "#E0E0E0"
    COL_BG      = "#FAFAFA"

    ANIM_FPS    = 60
    ANIM_DUR    = 0.4   # seconds for animation

    def __init__(self, ros_node=None):
        self._ros_node = ros_node

        # Current and target position (IK mm)
        self._current_pos = np.array([0.0, 250.0])
        self._target_pos  = np.array([0.0, 250.0])

        # Animation state
        self._anim_start_pos = np.array([0.0, 250.0])
        self._anim_start_time = None
        self._anim_duration = self.ANIM_DUR
        self._animating = False

        # Pre-compute hole IK positions
        self._hole_ik = {}
        for name, (wx, wy) in ALL_HOLES.items():
            self._hole_ik[name] = world_to_ik_mm(wx, wy)

        # Workspace boundary (precomputed)
        self._ws_boundary = self._compute_workspace_boundary()

        self._build_gui()
        self._update_display()

    # ============================================================== #
    #  Workspace boundary (for reference ring)
    # ============================================================== #
    def _compute_workspace_boundary(self):
        """Compute approximate reachable workspace boundary."""
        outer_pts = []
        inner_pts = []
        for angle in np.linspace(0, 2*math.pi, 360):
            # Outer boundary: both arms fully extended
            for r in np.linspace(50, L1_MM + L2_MM - 1, 200):
                px = r * math.cos(angle)
                py = r * math.sin(angle)
                if py < 0:
                    continue
                sol = ik_solve(px, py)
                if sol is not None:
                    outer_pts.append((px, py))
        return outer_pts

    # ============================================================== #
    #  GUI Construction
    # ============================================================== #
    def _build_gui(self):
        self._root = tk.Tk()
        self._root.title("5-Bar Linkage Visualizer — Angle Verification")
        self._root.geometry("1200x820")
        self._root.configure(bg="#F5F5F5")
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

        # ---- Main layout: left = canvas, right = controls ---- #
        main_frame = ttk.Frame(self._root)
        main_frame.pack(fill="both", expand=True, padx=5, pady=5)

        # === LEFT: matplotlib canvas === #
        canvas_frame = ttk.Frame(main_frame)
        canvas_frame.pack(side="left", fill="both", expand=True)

        self._fig, self._ax = plt.subplots(1, 1, figsize=(7.5, 7.5), dpi=100)
        self._fig.patch.set_facecolor(self.COL_BG)
        self._canvas = FigureCanvasTkAgg(self._fig, master=canvas_frame)
        self._canvas.get_tk_widget().pack(fill="both", expand=True)
        self._canvas.mpl_connect("button_press_event", self._on_canvas_click)

        # === RIGHT: control panel === #
        right_frame = ttk.Frame(main_frame, width=380)
        right_frame.pack(side="right", fill="y", padx=(5, 0))
        right_frame.pack_propagate(False)

        # ---- Angle display ---- #
        angle_frame = ttk.LabelFrame(right_frame, text="  Angles & Steps  ",
                                     padding=10)
        angle_frame.pack(fill="x", pady=(0, 5))

        self._info_labels = {}
        info_items = [
            ("ik_pos",      "IK Position:"),
            ("sep1",        ""),
            ("stepper_L",   "Stepper L:"),
            ("stepper_R",   "Stepper R:"),
            ("sep2",        ""),
            ("urdf_L",      "URDF L:"),
            ("urdf_R",      "URDF R:"),
            ("sep3",        ""),
            ("steps_L",     "Steps L:"),
            ("steps_R",     "Steps R:"),
            ("sep4",        ""),
            ("elbow_L",     "Elbow L angle:"),
            ("elbow_R",     "Elbow R angle:"),
        ]
        for key, label_text in info_items:
            if key.startswith("sep"):
                ttk.Separator(angle_frame, orient="horizontal").pack(
                    fill="x", pady=3)
                continue
            row = ttk.Frame(angle_frame)
            row.pack(fill="x", pady=1)
            ttk.Label(row, text=label_text, width=16, anchor="w",
                      font=("Consolas", 10)).pack(side="left")
            lbl = ttk.Label(row, text="—", font=("Consolas", 10, "bold"),
                            foreground="#1565C0")
            lbl.pack(side="left", fill="x", expand=True)
            self._info_labels[key] = lbl

        # ---- Hole buttons ---- #
        hole_frame = ttk.LabelFrame(right_frame, text="  Hole Positions  ",
                                    padding=8)
        hole_frame.pack(fill="x", pady=(0, 5))

        # Home button
        ttk.Button(hole_frame, text="⌂ HOME (0, 250)",
                   command=self._go_home).pack(fill="x", pady=(0, 8))

        # Front tray 3x3
        ttk.Label(hole_frame, text="Front Tray",
                  font=("", 9, "bold"), foreground=self.COL_FRONT).pack(anchor="w")
        front_grid = ttk.Frame(hole_frame)
        front_grid.pack(fill="x", pady=(2, 6))
        for i in range(9):
            r, c = divmod(i, 3)
            name = f"F{i}"
            btn = tk.Button(front_grid, text=name, width=5,
                            bg="#FFF3E0", activebackground="#FFE0B2",
                            font=("Consolas", 9),
                            command=lambda n=name: self._go_hole(n))
            btn.grid(row=r, column=c, padx=2, pady=2, sticky="ew")
        for c in range(3):
            front_grid.columnconfigure(c, weight=1)

        # Left tray 3x3
        ttk.Label(hole_frame, text="Left Tray",
                  font=("", 9, "bold"), foreground=self.COL_LEFT).pack(anchor="w")
        left_grid = ttk.Frame(hole_frame)
        left_grid.pack(fill="x", pady=(2, 6))
        for i in range(9):
            r, c = divmod(i, 3)
            name = f"L{i}"
            btn = tk.Button(left_grid, text=name, width=5,
                            bg="#F3E5F5", activebackground="#E1BEE7",
                            font=("Consolas", 9),
                            command=lambda n=name: self._go_hole(n))
            btn.grid(row=r, column=c, padx=2, pady=2, sticky="ew")
        for c in range(3):
            left_grid.columnconfigure(c, weight=1)

        # ---- Animation speed ---- #
        speed_frame = ttk.LabelFrame(right_frame, text="  Animation  ", padding=8)
        speed_frame.pack(fill="x", pady=(0, 5))

        self._speed_var = tk.DoubleVar(value=0.4)
        ttk.Label(speed_frame, text="Duration (s):", font=("", 9)).pack(anchor="w")
        speed_scale = ttk.Scale(speed_frame, from_=0.05, to=2.0,
                                variable=self._speed_var, orient="horizontal")
        speed_scale.pack(fill="x")
        self._speed_label = ttk.Label(speed_frame, text="0.40 s")
        self._speed_label.pack(anchor="w")

        def _update_speed(_=None):
            v = self._speed_var.get()
            self._anim_duration = v
            self._speed_label.config(text=f"{v:.2f} s")
        speed_scale.config(command=_update_speed)

        # ---- Status bar ---- #
        status_frame = ttk.Frame(right_frame)
        status_frame.pack(fill="x", side="bottom", pady=5)
        ros_text = "ROS 2 Connected ✓" if self._ros_node else "Standalone (no ROS)"
        ros_color = "#4CAF50" if self._ros_node else "#9E9E9E"
        self._status_lbl = ttk.Label(status_frame, text=ros_text,
                                     foreground=ros_color, font=("", 9))
        self._status_lbl.pack(anchor="w")

        self._current_hole_lbl = ttk.Label(status_frame, text="Position: Home",
                                           font=("", 10, "bold"),
                                           foreground="#333")
        self._current_hole_lbl.pack(anchor="w", pady=(3, 0))

        # ---- Animation timer ---- #
        self._running = True
        self._anim_after_id = None
        self._tick()

    # ============================================================== #
    #  Drawing
    # ============================================================== #
    def _draw_linkage(self, px, py):
        """Full redraw of the linkage at position (px, py) in IK mm."""
        ax = self._ax
        ax.clear()

        sol = ik_solve(px, py)
        if sol is None:
            ax.set_title("UNREACHABLE", color="red", fontsize=14)
            self._canvas.draw_idle()
            return

        th_L, th_R = sol
        fk = fk_full(th_L, th_R)
        if fk is None:
            ax.set_title("FK ERROR", color="red", fontsize=14)
            self._canvas.draw_idle()
            return

        # --- Workspace shading (light background) --- #
        ws_x = [p[0] for p in self._ws_boundary]
        ws_y = [p[1] for p in self._ws_boundary]
        if ws_x:
            ax.scatter(ws_x, ws_y, s=0.3, c=self.COL_WORKSPACE, zorder=0)

        # --- Base bar --- #
        ax.plot([B1X, B2X], [B1Y, B2Y], color=self.COL_BASE,
                linewidth=6, solid_capstyle="round", zorder=2)

        # --- Ground (hatch) --- #
        ax.axhline(y=0, color="#999", linewidth=0.5, linestyle="--", zorder=1)

        # --- Left crank --- #
        e1 = fk["elbow_L"]
        ax.plot([B1X, e1[0]], [B1Y, e1[1]], color=self.COL_CRANK_L,
                linewidth=4, solid_capstyle="round", zorder=3)

        # --- Right crank --- #
        e2 = fk["elbow_R"]
        ax.plot([B2X, e2[0]], [B2Y, e2[1]], color=self.COL_CRANK_R,
                linewidth=4, solid_capstyle="round", zorder=3)

        # --- Left coupler --- #
        ee = fk["ee"]
        ax.plot([e1[0], ee[0]], [e1[1], ee[1]], color=self.COL_COUPL_L,
                linewidth=3, solid_capstyle="round", zorder=3)

        # --- Right coupler --- #
        ax.plot([e2[0], ee[0]], [e2[1], ee[1]], color=self.COL_COUPL_R,
                linewidth=3, solid_capstyle="round", zorder=3)

        # --- Joint circles --- #
        for jx, jy, color, size in [
            (B1X, B1Y, self.COL_BASE, 8),
            (B2X, B2Y, self.COL_BASE, 8),
            (e1[0], e1[1], self.COL_CRANK_L, 6),
            (e2[0], e2[1], self.COL_CRANK_R, 6),
            (ee[0], ee[1], self.COL_EE, 10),
        ]:
            ax.plot(jx, jy, 'o', color=color, markersize=size, zorder=5)

        # --- EE crosshair --- #
        ch = 15
        ax.plot([ee[0]-ch, ee[0]+ch], [ee[1], ee[1]],
                color=self.COL_EE, linewidth=0.8, zorder=4)
        ax.plot([ee[0], ee[0]], [ee[1]-ch, ee[1]+ch],
                color=self.COL_EE, linewidth=0.8, zorder=4)

        # --- Hole positions --- #
        for name, (hx, hy) in self._hole_ik.items():
            if name.startswith("F"):
                color = self.COL_FRONT
            else:
                color = self.COL_LEFT
            ax.plot(hx, hy, 's', color=color, markersize=7, zorder=4,
                    markeredgecolor="white", markeredgewidth=0.5)
            ax.annotate(name, (hx, hy), fontsize=6, ha="center",
                        va="bottom", xytext=(0, 5),
                        textcoords="offset points", color=color,
                        fontweight="bold")

        # --- Angle arcs (visual) --- #
        arc_radius = 40
        # Left motor angle arc
        angles_l = np.linspace(0, math.radians(th_L), 30)
        arc_lx = B1X + arc_radius * np.cos(angles_l)
        arc_ly = B1Y + arc_radius * np.sin(angles_l)
        ax.plot(arc_lx, arc_ly, color=self.COL_CRANK_L, linewidth=1.5,
                alpha=0.6, zorder=4)
        ax.annotate(f"{th_L:.1f}°", (B1X, B1Y),
                    xytext=(25, 15), textcoords="offset points",
                    fontsize=8, color=self.COL_CRANK_L, fontweight="bold")

        # Right motor angle arc
        angles_r = np.linspace(0, math.radians(th_R), 30)
        arc_rx = B2X + arc_radius * np.cos(angles_r)
        arc_ry = B2Y + arc_radius * np.sin(angles_r)
        ax.plot(arc_rx, arc_ry, color=self.COL_CRANK_R, linewidth=1.5,
                alpha=0.6, zorder=4)
        ax.annotate(f"{th_R:.1f}°", (B2X, B2Y),
                    xytext=(-55, 15), textcoords="offset points",
                    fontsize=8, color=self.COL_CRANK_R, fontweight="bold")

        # --- Labels --- #
        ax.set_xlabel("X (mm)", fontsize=10)
        ax.set_ylabel("Y (mm)", fontsize=10)
        ax.set_title(f"5-Bar Linkage   EE = ({px:.1f}, {py:.1f}) mm",
                     fontsize=11, fontweight="bold")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3, linewidth=0.5)
        ax.set_xlim(-420, 420)
        ax.set_ylim(-50, 420)

        # Legend
        legend_elements = [
            mpatches.Patch(color=self.COL_CRANK_L, label="Left crank"),
            mpatches.Patch(color=self.COL_CRANK_R, label="Right crank"),
            mpatches.Patch(color=self.COL_COUPL_L, label="Left coupler"),
            mpatches.Patch(color=self.COL_COUPL_R, label="Right coupler"),
            mpatches.Patch(color=self.COL_EE, label="End-effector"),
            mpatches.Patch(color=self.COL_FRONT, label="Front holes"),
            mpatches.Patch(color=self.COL_LEFT, label="Left holes"),
        ]
        ax.legend(handles=legend_elements, loc="upper right", fontsize=7,
                  framealpha=0.8)

        self._canvas.draw_idle()

    # ============================================================== #
    #  Info panel update
    # ============================================================== #
    def _update_info(self, px, py):
        """Update the right-side angle/step info panel."""
        sol = ik_solve(px, py)
        if sol is None:
            for key in self._info_labels:
                self._info_labels[key].config(text="—", foreground="red")
            return

        th_L, th_R = sol

        # IK position
        self._info_labels["ik_pos"].config(
            text=f"({px:.1f}, {py:.1f}) mm")

        # Stepper angles (= IK angles)
        self._info_labels["stepper_L"].config(text=f"{th_L:.2f}°")
        self._info_labels["stepper_R"].config(text=f"{th_R:.2f}°")

        # URDF angles
        u_L = ik_to_urdf(th_L)
        u_R = ik_to_urdf(th_R)
        self._info_labels["urdf_L"].config(text=f"{u_L:.2f}°")
        self._info_labels["urdf_R"].config(text=f"{u_R:.2f}°")

        # Step counts
        s_L = round(th_L * STEPS_PER_DEGREE)
        s_R = round(th_R * STEPS_PER_DEGREE)
        self._info_labels["steps_L"].config(text=f"{s_L}")
        self._info_labels["steps_R"].config(text=f"{s_R}")

        # Elbow angles (how close to singularity)
        fk = fk_full(th_L, th_R)
        if fk:
            e1 = fk["elbow_L"]
            ee = fk["ee"]
            e2 = fk["elbow_R"]

            # Left elbow angle
            v1 = np.array([B1X - e1[0], B1Y - e1[1]])
            v2 = np.array([ee[0] - e1[0], ee[1] - e1[1]])
            cos_a = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-9)
            elbow_L_deg = math.degrees(math.acos(_clamp(cos_a, -1, 1)))

            # Right elbow angle
            v1 = np.array([B2X - e2[0], B2Y - e2[1]])
            v2 = np.array([ee[0] - e2[0], ee[1] - e2[1]])
            cos_a = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-9)
            elbow_R_deg = math.degrees(math.acos(_clamp(cos_a, -1, 1)))

            # Colour by singularity proximity
            def _elbow_color(deg):
                if deg > 150 or deg < 30:
                    return "#F44336"   # danger
                elif deg > 130 or deg < 50:
                    return "#FF9800"   # warning
                else:
                    return "#4CAF50"   # safe

            self._info_labels["elbow_L"].config(
                text=f"{elbow_L_deg:.1f}°",
                foreground=_elbow_color(elbow_L_deg))
            self._info_labels["elbow_R"].config(
                text=f"{elbow_R_deg:.1f}°",
                foreground=_elbow_color(elbow_R_deg))

        # Reset colours for normal fields
        for key in ("ik_pos", "stepper_L", "stepper_R", "urdf_L", "urdf_R",
                    "steps_L", "steps_R"):
            self._info_labels[key].config(foreground="#1565C0")

    # ============================================================== #
    #  Animation
    # ============================================================== #
    def _start_animation(self, target_x, target_y):
        """Begin smooth animation to target position."""
        self._anim_start_pos = self._current_pos.copy()
        self._target_pos = np.array([target_x, target_y])
        self._anim_start_time = time.monotonic()
        self._animating = True

    def _smooth_step(self, t):
        """Smooth-step easing (ease in-out)."""
        t = _clamp(t, 0.0, 1.0)
        return t * t * (3 - 2 * t)

    def _tick(self):
        """Animation tick — called at ANIM_FPS."""
        if not self._running:
            return

        if self._animating:
            elapsed = time.monotonic() - self._anim_start_time
            t = elapsed / max(self._anim_duration, 0.01)

            if t >= 1.0:
                # Animation complete
                self._current_pos = self._target_pos.copy()
                self._animating = False
            else:
                # Interpolate
                s = self._smooth_step(t)
                self._current_pos = (1 - s) * self._anim_start_pos + s * self._target_pos

            self._update_display()

            # Publish to ROS if connected
            if self._ros_node and not self._animating:
                self._publish_ros(self._current_pos[0], self._current_pos[1])

        # Schedule next tick
        interval = int(1000 / self.ANIM_FPS)
        self._anim_after_id = self._root.after(interval, self._tick)

    def _update_display(self):
        """Redraw linkage + update info at current position."""
        px, py = self._current_pos
        self._draw_linkage(px, py)
        self._update_info(px, py)

    # ============================================================== #
    #  User interactions
    # ============================================================== #
    def _on_canvas_click(self, event):
        """Handle click on matplotlib canvas — move arm to clicked point."""
        if event.inaxes != self._ax:
            return
        px, py = event.xdata, event.ydata
        if py < 0:
            py = 0.1

        # Check reachability
        if ik_solve(px, py) is None:
            self._current_hole_lbl.config(text="Position: UNREACHABLE!",
                                          foreground="red")
            return

        self._current_hole_lbl.config(text=f"Position: ({px:.1f}, {py:.1f})",
                                      foreground="#333")
        self._start_animation(px, py)

        # Publish to ROS immediately for the target
        if self._ros_node:
            # Convert to IK metres for /ik_target
            self._publish_ros(px, py)

    def _go_hole(self, name):
        """Move arm to a named hole position."""
        hx, hy = self._hole_ik[name]
        self._current_hole_lbl.config(text=f"Position: {name}",
                                      foreground="#333")
        self._start_animation(hx, hy)

        if self._ros_node:
            self._publish_ros(hx, hy)

    def _go_home(self):
        """Return to home position."""
        self._current_hole_lbl.config(text="Position: Home", foreground="#333")
        self._start_animation(0.0, 250.0)

        if self._ros_node:
            self._publish_ros(0.0, 250.0)

    # ============================================================== #
    #  ROS 2 publishing
    # ============================================================== #
    def _publish_ros(self, px_mm, py_mm):
        """Publish IK target to ROS in metres."""
        if self._ros_node is None:
            return
        msg = Point()
        msg.x = px_mm / 1000.0   # mm → m
        msg.y = py_mm / 1000.0
        msg.z = 0.0
        self._ros_node.publisher.publish(msg)

    # ============================================================== #
    #  Main loop
    # ============================================================== #
    def _on_close(self):
        self._running = False
        if self._anim_after_id:
            self._root.after_cancel(self._anim_after_id)
        plt.close(self._fig)
        self._root.destroy()

    def run(self):
        """Run the GUI main loop."""
        if self._ros_node:
            # Spin ROS in background thread
            spin_thread = threading.Thread(
                target=rclpy.spin, args=(self._ros_node,), daemon=True)
            spin_thread.start()

        self._root.mainloop()


# ================================================================== #
#  ROS 2 node wrapper (minimal)
# ================================================================== #
class LinkageGUINode(Node):
    def __init__(self):
        super().__init__("linkage_gui_node")
        self.publisher = self.create_publisher(Point, "/ik_target", 10)
        self.get_logger().info("Linkage GUI ROS node ready — publishing to /ik_target")


# ================================================================== #
#  Entry point
# ================================================================== #
def main():
    ros_node = None

    if _ros_available:
        try:
            rclpy.init()
            ros_node = LinkageGUINode()
        except Exception as e:
            print(f"[WARN] ROS 2 init failed ({e}), running standalone")
            ros_node = None

    gui = LinkageGUI(ros_node=ros_node)

    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        if ros_node:
            ros_node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
