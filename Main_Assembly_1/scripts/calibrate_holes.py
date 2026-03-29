#!/usr/bin/env python3
"""
Hole Calibration GUI — WASD joystick + sliders + valve/servo control.

Run ALONGSIDE the main launch file (five_bar_ik.launch.py must be running).
Publishes /ik_target to move the robot, /suction_manual for valve/servo.

Usage:
    python3 calibrate_holes.py

Controls:
  WASD      — nudge end effector (hold Shift for fine 0.5 mm steps)
  1/2       — servo down / servo up
  3/4       — valve open / valve close
  R         — record current position for selected hole
  Space     — print all recorded holes to terminal
"""

import tkinter as tk
from tkinter import ttk
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String


FRONT_HOLES = [f"F{i}" for i in range(9)]
LEFT_HOLES = [f"L{i}" for i in range(9)]
ALL_HOLES = FRONT_HOLES + LEFT_HOLES

STEP_COARSE = 0.005   # 5 mm per key press
STEP_FINE   = 0.0005  # 0.5 mm per key press (Shift held)


class CalibrateNode(Node):
    def __init__(self):
        super().__init__("calibrate_holes")
        self._ik_pub = self.create_publisher(Point, "/ik_target", 10)
        self._manual_pub = self.create_publisher(String, "/suction_manual", 10)

    def publish_target(self, x, y):
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        self._ik_pub.publish(msg)

    def send_manual(self, cmd):
        msg = String()
        msg.data = cmd
        self._manual_pub.publish(msg)


def main():
    rclpy.init()
    node = CalibrateNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    recorded = {}

    root = tk.Tk()
    root.title("Hole Calibration — WASD + Valve/Servo")
    root.geometry("580x780")

    x_var = tk.DoubleVar(value=0.0)
    y_var = tk.DoubleVar(value=0.25)

    # ==================== COORDINATE DISPLAY ====================
    coord_frame = ttk.Frame(root)
    coord_frame.pack(fill="x", padx=10, pady=(10, 0))

    coord_lbl = tk.Label(coord_frame, text="IK: (0.0000, 0.2500)",
                         font=("Courier", 18, "bold"), fg="blue")
    coord_lbl.pack()

    def sync_display():
        x, y = x_var.get(), y_var.get()
        coord_lbl.config(text=f"IK: ({x:+.4f}, {y:+.4f})")
        x_lbl.config(text=f"X = {x:+.5f} m")
        y_lbl.config(text=f"Y = {y:+.5f} m")
        x_entry.delete(0, tk.END)
        x_entry.insert(0, f"{x:.5f}")
        y_entry.delete(0, tk.END)
        y_entry.insert(0, f"{y:.5f}")

    def publish_current():
        node.publish_target(x_var.get(), y_var.get())
        sync_display()

    # ==================== WASD JOYSTICK ====================
    wasd_frame = ttk.LabelFrame(root, text="WASD Joystick  (Shift = fine)")
    wasd_frame.pack(fill="x", padx=10, pady=5)

    step_var = tk.StringVar(value=f"Step: {STEP_COARSE*1000:.1f} mm")
    step_lbl = ttk.Label(wasd_frame, textvariable=step_var,
                         font=("Courier", 10))
    step_lbl.pack(pady=(5, 0))

    btn_grid = ttk.Frame(wasd_frame)
    btn_grid.pack(pady=5)

    def nudge(dx, dy, fine=False):
        step = STEP_FINE if fine else STEP_COARSE
        x_var.set(round(x_var.get() + dx * step, 6))
        y_var.set(round(y_var.get() + dy * step, 6))
        publish_current()

    w_btn = tk.Button(btn_grid, text="W\n(+Y)", width=8, height=2,
                      command=lambda: nudge(0, 1), bg="#d0ffd0")
    w_btn.grid(row=0, column=1, padx=2, pady=2)
    a_btn = tk.Button(btn_grid, text="A\n(-X)", width=8, height=2,
                      command=lambda: nudge(-1, 0), bg="#d0d0ff")
    a_btn.grid(row=1, column=0, padx=2, pady=2)
    s_btn = tk.Button(btn_grid, text="S\n(-Y)", width=8, height=2,
                      command=lambda: nudge(0, -1), bg="#ffd0d0")
    s_btn.grid(row=1, column=1, padx=2, pady=2)
    d_btn = tk.Button(btn_grid, text="D\n(+X)", width=8, height=2,
                      command=lambda: nudge(1, 0), bg="#d0d0ff")
    d_btn.grid(row=1, column=2, padx=2, pady=2)

    # Keyboard bindings
    def on_key(event):
        fine = bool(event.state & 0x1)  # Shift held
        k = event.keysym.lower()
        if k == 'w':
            nudge(0, 1, fine)
        elif k == 's':
            nudge(0, -1, fine)
        elif k == 'a':
            nudge(-1, 0, fine)
        elif k == 'd':
            nudge(1, 0, fine)
        elif k == '1':
            node.send_manual("servo_down")
            status_lbl.config(text="Servo DOWN", foreground="orange")
        elif k == '2':
            node.send_manual("servo_up")
            status_lbl.config(text="Servo UP", foreground="green")
        elif k == '3':
            node.send_manual("valve_open")
            status_lbl.config(text="Valve OPEN", foreground="red")
        elif k == '4':
            node.send_manual("valve_close")
            status_lbl.config(text="Valve CLOSED", foreground="green")
        elif k == 'r':
            record_hole()
        elif k == 'space':
            print_all()
        if fine:
            step_var.set(f"Step: {STEP_FINE*1000:.1f} mm (fine)")
        else:
            step_var.set(f"Step: {STEP_COARSE*1000:.1f} mm")

    root.bind("<KeyPress>", on_key)

    # ==================== X/Y SLIDERS ====================
    slider_frame = ttk.LabelFrame(root, text="X / Y Sliders + Entry")
    slider_frame.pack(fill="x", padx=10, pady=5)

    ttk.Label(slider_frame, text="X (m)").grid(row=0, column=0, sticky="w", padx=10)
    x_slider = ttk.Scale(slider_frame, from_=-0.20, to=0.20,
                         variable=x_var, orient="horizontal",
                         command=lambda _: publish_current())
    x_slider.grid(row=0, column=1, sticky="ew", padx=5)
    x_entry = ttk.Entry(slider_frame, width=10)
    x_entry.grid(row=0, column=2, padx=5)
    x_entry.insert(0, "0.00000")
    x_lbl = ttk.Label(slider_frame, text="X = +0.00000 m", font=("Courier", 10))
    x_lbl.grid(row=1, column=0, columnspan=3, sticky="w", padx=14)

    ttk.Label(slider_frame, text="Y (m)").grid(row=2, column=0, sticky="w", padx=10)
    y_slider = ttk.Scale(slider_frame, from_=0.00, to=0.40,
                         variable=y_var, orient="horizontal",
                         command=lambda _: publish_current())
    y_slider.grid(row=2, column=1, sticky="ew", padx=5)
    y_entry = ttk.Entry(slider_frame, width=10)
    y_entry.grid(row=2, column=2, padx=5)
    y_entry.insert(0, "0.25000")
    y_lbl = ttk.Label(slider_frame, text="Y = +0.25000 m", font=("Courier", 10))
    y_lbl.grid(row=3, column=0, columnspan=3, sticky="w", padx=14)

    slider_frame.columnconfigure(1, weight=1)

    def update_from_entry(_=None):
        try:
            x_var.set(float(x_entry.get()))
            y_var.set(float(y_entry.get()))
        except ValueError:
            return
        publish_current()

    x_entry.bind("<Return>", update_from_entry)
    y_entry.bind("<Return>", update_from_entry)

    # ==================== VALVE / SERVO ====================
    hw_frame = ttk.LabelFrame(root, text="Valve & Servo  (keys: 1=down  2=up  3=open  4=close)")
    hw_frame.pack(fill="x", padx=10, pady=5)

    hw_btns = ttk.Frame(hw_frame)
    hw_btns.pack(pady=5)

    tk.Button(hw_btns, text="Servo DOWN (1)", width=14, bg="#FFD580",
              command=lambda: (node.send_manual("servo_down"),
                               status_lbl.config(text="Servo DOWN", foreground="orange"))
              ).grid(row=0, column=0, padx=4, pady=2)
    tk.Button(hw_btns, text="Servo UP (2)", width=14, bg="#A0E0A0",
              command=lambda: (node.send_manual("servo_up"),
                               status_lbl.config(text="Servo UP", foreground="green"))
              ).grid(row=0, column=1, padx=4, pady=2)
    tk.Button(hw_btns, text="Valve OPEN (3)", width=14, bg="#FFA0A0",
              command=lambda: (node.send_manual("valve_open"),
                               status_lbl.config(text="Valve OPEN", foreground="red"))
              ).grid(row=0, column=2, padx=4, pady=2)
    tk.Button(hw_btns, text="Valve CLOSE (4)", width=14, bg="#A0E0A0",
              command=lambda: (node.send_manual("valve_close"),
                               status_lbl.config(text="Valve CLOSED", foreground="green"))
              ).grid(row=0, column=3, padx=4, pady=2)

    status_lbl = ttk.Label(hw_frame, text="Ready", font=("", 10, "bold"),
                           foreground="gray")
    status_lbl.pack(pady=(0, 5))

    # ==================== RECORD SECTION ====================
    rec_frame = ttk.LabelFrame(root, text="Record Hole Position  (key: R)")
    rec_frame.pack(fill="x", padx=10, pady=5)

    ttk.Label(rec_frame, text="Hole:").grid(row=0, column=0, padx=10, pady=5)
    hole_var = tk.StringVar(value="F0")
    hole_combo = ttk.Combobox(rec_frame, textvariable=hole_var,
                              values=ALL_HOLES, width=8, state="readonly")
    hole_combo.grid(row=0, column=1, padx=5, pady=5)

    def record_hole():
        name = hole_var.get()
        x, y = x_var.get(), y_var.get()
        recorded[name] = (x, y)
        refresh_list()
        status_lbl.config(text=f"Recorded {name}: ({x:+.4f}, {y:+.4f})",
                          foreground="purple")
        # auto-advance to next hole
        idx = ALL_HOLES.index(name)
        if idx + 1 < len(ALL_HOLES):
            hole_var.set(ALL_HOLES[idx + 1])

    ttk.Button(rec_frame, text="Record (R)",
               command=record_hole).grid(row=0, column=2, padx=10, pady=5)

    def goto_hole():
        name = hole_var.get()
        if name in recorded:
            x, y = recorded[name]
            x_var.set(x)
            y_var.set(y)
            publish_current()

    ttk.Button(rec_frame, text="Go To",
               command=goto_hole).grid(row=0, column=3, padx=5, pady=5)

    # ==================== RECORDED LIST ====================
    list_frame = ttk.LabelFrame(root, text="Recorded Positions (IK coordinates)")
    list_frame.pack(fill="both", expand=True, padx=10, pady=5)

    list_text = tk.Text(list_frame, height=10, font=("Courier", 10),
                        state="disabled", wrap="none")
    list_text.pack(fill="both", expand=True, padx=5, pady=5)

    def refresh_list():
        list_text.config(state="normal")
        list_text.delete("1.0", tk.END)
        for name in ALL_HOLES:
            if name in recorded:
                x, y = recorded[name]
                list_text.insert(tk.END, f"  {name}: ({x:+.5f}, {y:+.5f})\n")
        list_text.config(state="disabled")

    # ==================== PRINT / EXPORT ====================
    btn_frame = ttk.Frame(root)
    btn_frame.pack(fill="x", padx=10, pady=5)

    def print_all():
        print("\n" + "=" * 60)
        print("  RECORDED HOLE IK COORDINATES")
        print("  Paste these into five_bar_ik_node.py")
        print("=" * 60)
        if any(n in recorded for n in FRONT_HOLES):
            print("\n  # Front tray holes (IK plane x, y)")
            for name in FRONT_HOLES:
                if name in recorded:
                    x, y = recorded[name]
                    print(f'  "{name}": ({x:.6f}, {y:.6f}),')
        if any(n in recorded for n in LEFT_HOLES):
            print("\n  # Left tray holes (IK plane x, y)")
            for name in LEFT_HOLES:
                if name in recorded:
                    x, y = recorded[name]
                    print(f'  "{name}": ({x:.6f}, {y:.6f}),')
        print("=" * 60 + "\n")

    ttk.Button(btn_frame, text="Print All (Space)",
               command=print_all).pack(side="left", padx=5)

    def clear_all():
        recorded.clear()
        refresh_list()

    ttk.Button(btn_frame, text="Clear All",
               command=clear_all).pack(side="left", padx=5)

    # ==================== MAINLOOP ====================
    def on_close():
        print_all()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
