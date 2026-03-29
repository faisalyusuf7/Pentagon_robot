#!/usr/bin/env python3
"""
Slider GUI for the 5-bar parallel linkage robot.

Two modes (tabs):
  1. IK Mode   – sliders for target X, Y  → publishes to /ik_target
  2. Joint Mode – sliders for left/right motor angles → publishes to /ik_target
                  (reverse-computes the end-effector from motor angles)

Also has a suction servo slider.

Launch alongside five_bar_ik_node.py.
"""

import math
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


# ------------------------------------------------------------------ #
#  FK helper (motor IK angles → end-effector position)
# ------------------------------------------------------------------ #
def forward_kinematics(th1_deg, th2_deg, L1=0.2, L2=0.2, d=0.19):
    """
    Given left & right motor angles (in IK-plane degrees),
    compute the end-effector (x, y) where the two coupler links meet.

    Returns (x, y) or None if the configuration is invalid.
    """
    th1 = math.radians(th1_deg)
    th2 = math.radians(th2_deg)

    b1x, b1y = -d / 2.0, 0.0
    b2x, b2y = d / 2.0, 0.0

    # Crank tips
    e1x = b1x + L1 * math.cos(th1)
    e1y = b1y + L1 * math.sin(th1)
    e2x = b2x + L1 * math.cos(th2)
    e2y = b2y + L1 * math.sin(th2)

    # Distance between crank tips
    dx = e2x - e1x
    dy = e2y - e1y
    dist = math.hypot(dx, dy)

    if dist < 1e-9 or dist > 2 * L2:
        return None

    # Circle-circle intersection (both radius L2 centred at crank tips)
    a = dist / 2.0
    if L2 * L2 - a * a < 0:
        return None
    h = math.sqrt(max(0, L2 * L2 - a * a))

    mx = (e1x + e2x) / 2.0
    my = (e1y + e2y) / 2.0

    # Two solutions; pick the one with larger y (elbow-up)
    px = mx - h * dy / dist
    py = my + h * dx / dist

    return (px, py)


class SliderGUI(Node):
    def __init__(self):
        super().__init__("fivebar_slider_gui")
        self._pub = self.create_publisher(Point, "/ik_target", 10)

        # --- build the GUI in main thread ---
        self._root = tk.Tk()
        self._root.title("5-Bar Linkage Control")
        self._root.geometry("420x380")
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

        style = ttk.Style()
        style.configure("TScale", sliderlength=20)

        notebook = ttk.Notebook(self._root)
        notebook.pack(fill="both", expand=True, padx=5, pady=5)

        # ==================== IK TAB ====================
        ik_frame = ttk.Frame(notebook)
        notebook.add(ik_frame, text="  IK Target  ")

        ttk.Label(ik_frame, text="Target X (m)", font=("", 11)).pack(
            anchor="w", padx=10, pady=(15, 0))
        self._ik_x = tk.DoubleVar(value=0.0)
        self._sx = ttk.Scale(ik_frame, from_=-0.20, to=0.20,
                             variable=self._ik_x, orient="horizontal",
                             command=self._on_ik_change)
        self._sx.pack(fill="x", padx=10)
        self._lbl_x = ttk.Label(ik_frame, text="X = 0.000 m")
        self._lbl_x.pack(anchor="w", padx=14)

        ttk.Label(ik_frame, text="Target Y (m)", font=("", 11)).pack(
            anchor="w", padx=10, pady=(15, 0))
        self._ik_y = tk.DoubleVar(value=0.25)
        self._sy = ttk.Scale(ik_frame, from_=0.05, to=0.40,
                             variable=self._ik_y, orient="horizontal",
                             command=self._on_ik_change)
        self._sy.pack(fill="x", padx=10)
        self._lbl_y = ttk.Label(ik_frame, text="Y = 0.250 m")
        self._lbl_y.pack(anchor="w", padx=14)

        self._ik_info = ttk.Label(ik_frame, text="", foreground="gray")
        self._ik_info.pack(anchor="w", padx=14, pady=(20, 0))

        # ==================== JOINT TAB ====================
        jt_frame = ttk.Frame(notebook)
        notebook.add(jt_frame, text="  Motor Angles  ")

        ttk.Label(jt_frame, text="Left Motor (°)", font=("", 11)).pack(
            anchor="w", padx=10, pady=(15, 0))
        self._jt_left = tk.DoubleVar(value=90.0)
        self._sl = ttk.Scale(jt_frame, from_=0, to=180,
                             variable=self._jt_left, orient="horizontal",
                             command=self._on_joint_change)
        self._sl.pack(fill="x", padx=10)
        self._lbl_l = ttk.Label(jt_frame, text="Left = 90.0°")
        self._lbl_l.pack(anchor="w", padx=14)

        ttk.Label(jt_frame, text="Right Motor (°)", font=("", 11)).pack(
            anchor="w", padx=10, pady=(15, 0))
        self._jt_right = tk.DoubleVar(value=90.0)
        self._sr = ttk.Scale(jt_frame, from_=0, to=180,
                             variable=self._jt_right, orient="horizontal",
                             command=self._on_joint_change)
        self._sr.pack(fill="x", padx=10)
        self._lbl_r = ttk.Label(jt_frame, text="Right = 90.0°")
        self._lbl_r.pack(anchor="w", padx=14)

        self._jt_info = ttk.Label(jt_frame, text="", foreground="gray")
        self._jt_info.pack(anchor="w", padx=14, pady=(20, 0))

        # ==================== HOME BUTTON ====================
        btn_frame = ttk.Frame(self._root)
        btn_frame.pack(fill="x", padx=10, pady=5)
        ttk.Button(btn_frame, text="⌂ Home (0, 0.25)",
                   command=self._go_home).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="↻ Zero Motors (90°, 90°)",
                   command=self._go_zero).pack(side="left", padx=5)

        self._running = True

    # ---------- callbacks ----------
    def _publish(self, x, y):
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        self._pub.publish(msg)

    def _on_ik_change(self, _=None):
        x = self._ik_x.get()
        y = self._ik_y.get()
        self._lbl_x.config(text=f"X = {x:.3f} m")
        self._lbl_y.config(text=f"Y = {y:.3f} m")
        self._ik_info.config(text=f"Publishing ({x:.3f}, {y:.3f})")
        self._publish(x, y)

    def _on_joint_change(self, _=None):
        l_deg = self._jt_left.get()
        r_deg = self._jt_right.get()
        self._lbl_l.config(text=f"Left = {l_deg:.1f}°")
        self._lbl_r.config(text=f"Right = {r_deg:.1f}°")

        result = forward_kinematics(l_deg, r_deg)
        if result is None:
            self._jt_info.config(text="Invalid configuration",
                                 foreground="red")
            return

        px, py = result
        self._jt_info.config(
            text=f"EE → ({px:.3f}, {py:.3f})  •  publishing",
            foreground="gray")
        self._publish(px, py)

    def _go_home(self):
        self._ik_x.set(0.0)
        self._ik_y.set(0.25)
        self._on_ik_change()

    def _go_zero(self):
        self._jt_left.set(90.0)
        self._jt_right.set(90.0)
        self._on_joint_change()

    def _on_close(self):
        self._running = False
        self._root.destroy()

    def spin_gui(self):
        """Run tkinter mainloop + ROS spinning together."""
        # spin ROS in a background thread
        spin_thread = threading.Thread(target=rclpy.spin, args=(self,),
                                       daemon=True)
        spin_thread.start()

        # tkinter in the main thread
        while self._running:
            try:
                self._root.update_idletasks()
                self._root.update()
            except tk.TclError:
                break

        self.destroy_node()


def main():
    rclpy.init()
    gui = SliderGUI()
    try:
        gui.spin_gui()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
