#!/usr/bin/env python3
"""
Terminal-only hole offset finder.

Run ALONGSIDE the main launch (five_bar_ik.launch.py must be running).
Moves the arm to each hole's nominal IK position, lets you nudge to
the real centre, and prints the offset in mm.

Usage:
    python3 find_offset.py

Commands (type and press Enter):
    w / s       — nudge +Y / -Y  (5 mm)
    a / d       — nudge -X / +X  (5 mm)
    W / S       — nudge +Y / -Y  (0.5 mm, fine)
    A / D       — nudge -X / +X  (0.5 mm, fine)
    g <hole>    — go to hole (e.g.  g F4)
    r           — record current position as "actual" for selected hole
    home        — go to home position
    offset      — print offset summary (nominal vs recorded) in mm
    holes       — list all holes with nominal IK coords
    pos         — print current IK position
    q           — quit
"""

import threading
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String

# ------------------------------------------------------------------ #
#  IK origin — must match pick_and_place_planner.py
# ------------------------------------------------------------------ #
_BASE_LINK_OFFSET = np.array([-0.006403, -0.103113, -0.00474])
_POS_ML = np.array([0.695406926143987, -1.58664693818658, 0.05]) + _BASE_LINK_OFFSET
_POS_MR = np.array([0.885406926135972, -1.58664693819113, 0.07]) + _BASE_LINK_OFFSET
IK_ORIGIN = (_POS_ML + _POS_MR) / 2.0

# ------------------------------------------------------------------ #
#  Nominal hole positions — world frame (metres), from planner
# ------------------------------------------------------------------ #
FRONT_TRAY_HOLES = {
    "F0": (0.749004, -1.429760, 0.020256),
    "F1": (0.749004, -1.389760, 0.020256),
    "F2": (0.749004, -1.344760, 0.020256),
    "F3": (0.794004, -1.429760, 0.020256),
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

HOME_IK = (0.0, 0.27)

STEP_COARSE = 0.005   # 5 mm
STEP_FINE   = 0.0005  # 0.5 mm


def world_to_ik(wx, wy):
    return wx - IK_ORIGIN[0], wy - IK_ORIGIN[1]


def ik_to_world(ix, iy):
    return ix + IK_ORIGIN[0], iy + IK_ORIGIN[1]


class OffsetNode(Node):
    def __init__(self):
        super().__init__("find_offset")
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
    node = OffsetNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # State
    cur_ix, cur_iy = HOME_IK
    current_hole = None
    recorded = {}  # hole_name -> (ik_x, ik_y)

    def publish():
        node.publish_target(cur_ix, cur_iy)

    def show_pos():
        wx, wy = ik_to_world(cur_ix, cur_iy)
        label = f"  [{current_hole}]" if current_hole else ""
        print(f"  IK: ({cur_ix:+.4f}, {cur_iy:+.4f})   "
              f"World: ({wx*1000:+.3f}, {wy*1000:+.3f}) mm{label}")

    publish()

    print("\n" + "=" * 60)
    print("  HOLE OFFSET FINDER")
    print("  Type 'help' for commands.  Launch must be running.")
    print("=" * 60)
    show_pos()

    while True:
        try:
            raw = input("\n> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not raw:
            continue

        parts = raw.split()
        cmd = parts[0]

        # ---- nudge ----
        if cmd in ("w", "W", "s", "S", "a", "A", "d", "D"):
            step = STEP_FINE if cmd.isupper() else STEP_COARSE
            if cmd.lower() == "w":
                cur_iy += step
            elif cmd.lower() == "s":
                cur_iy -= step
            elif cmd.lower() == "a":
                cur_ix -= step
            elif cmd.lower() == "d":
                cur_ix += step
            cur_ix = round(cur_ix, 6)
            cur_iy = round(cur_iy, 6)
            publish()
            show_pos()

        # ---- go to hole ----
        elif cmd == "g" and len(parts) >= 2:
            name = parts[1].upper()
            if name not in ALL_HOLES:
                print(f"  Unknown hole '{name}'. Use F0-F8 or L0-L8.")
                continue
            wx, wy, _ = ALL_HOLES[name]
            cur_ix, cur_iy = world_to_ik(wx, wy)
            current_hole = name
            publish()
            nom_wx, nom_wy = wx * 1000, wy * 1000
            print(f"  → Moved to {name} nominal: "
                  f"World ({nom_wx:+.3f}, {nom_wy:+.3f}) mm")
            show_pos()

        # ---- record ----
        elif cmd == "r":
            if current_hole is None:
                print("  No hole selected. Use 'g F4' first.")
                continue
            recorded[current_hole] = (cur_ix, cur_iy)
            wx, wy = ik_to_world(cur_ix, cur_iy)
            nom_wx, nom_wy, _ = ALL_HOLES[current_hole]
            dx = (wx - nom_wx) * 1000
            dy = (wy - nom_wy) * 1000
            print(f"  ✓ Recorded {current_hole}: "
                  f"IK ({cur_ix:+.5f}, {cur_iy:+.5f})  "
                  f"World ({wx*1000:+.3f}, {wy*1000:+.3f}) mm")
            print(f"    Offset from nominal: ΔX = {dx:+.3f} mm, ΔY = {dy:+.3f} mm")

        # ---- home ----
        elif cmd == "home":
            cur_ix, cur_iy = HOME_IK
            current_hole = None
            publish()
            print("  → Home")
            show_pos()

        # ---- print offset summary ----
        elif cmd == "offset":
            if not recorded:
                print("  No holes recorded yet. Go to a hole and press 'r'.")
                continue
            print("\n  " + "=" * 58)
            print("  OFFSET SUMMARY  (actual − nominal, in mm)")
            print("  " + "=" * 58)
            print(f"  {'Hole':>4}  {'Nom X':>10}  {'Nom Y':>10}  "
                  f"{'Act X':>10}  {'Act Y':>10}  {'ΔX':>8}  {'ΔY':>8}")
            print("  " + "-" * 58)

            dx_sum, dy_sum, n = 0.0, 0.0, 0
            for name in list(FRONT_TRAY_HOLES) + list(LEFT_TRAY_HOLES):
                if name not in recorded:
                    continue
                nom_wx, nom_wy, _ = ALL_HOLES[name]
                act_ix, act_iy = recorded[name]
                act_wx, act_wy = ik_to_world(act_ix, act_iy)
                dx = (act_wx - nom_wx) * 1000
                dy = (act_wy - nom_wy) * 1000
                dx_sum += dx
                dy_sum += dy
                n += 1
                print(f"  {name:>4}  {nom_wx*1000:>+10.3f}  {nom_wy*1000:>+10.3f}  "
                      f"{act_wx*1000:>+10.3f}  {act_wy*1000:>+10.3f}  "
                      f"{dx:>+8.3f}  {dy:>+8.3f}")

            if n > 0:
                avg_dx = dx_sum / n
                avg_dy = dy_sum / n
                print("  " + "-" * 58)
                print(f"  {'AVG':>4}  {'':>10}  {'':>10}  "
                      f"{'':>10}  {'':>10}  "
                      f"{avg_dx:>+8.3f}  {avg_dy:>+8.3f}")
                print(f"\n  ★ Average global offset:  "
                      f"ΔX = {avg_dx:+.3f} mm,  ΔY = {avg_dy:+.3f} mm")
                print(f"    Tell copilot: \"apply offset {avg_dx:+.3f} mm X, "
                      f"{avg_dy:+.3f} mm Y to all holes\"")
            print()

        # ---- list holes ----
        elif cmd == "holes":
            print(f"\n  {'Hole':>4}  {'World X (mm)':>14}  {'World Y (mm)':>14}  "
                  f"{'IK X':>10}  {'IK Y':>10}")
            print("  " + "-" * 56)
            for name in list(FRONT_TRAY_HOLES) + list(LEFT_TRAY_HOLES):
                wx, wy, _ = ALL_HOLES[name]
                ix, iy = world_to_ik(wx, wy)
                rec = " ✓" if name in recorded else ""
                print(f"  {name:>4}  {wx*1000:>+14.3f}  {wy*1000:>+14.3f}  "
                      f"{ix:>+10.5f}  {iy:>+10.5f}{rec}")

        # ---- pos ----
        elif cmd == "pos":
            show_pos()

        # ---- servo / valve ----
        elif cmd == "servo_down" or cmd == "1":
            node.send_manual("servo_down")
            print("  Servo DOWN")
        elif cmd == "servo_up" or cmd == "2":
            node.send_manual("servo_up")
            print("  Servo UP")
        elif cmd == "valve_open" or cmd == "3":
            node.send_manual("valve_open")
            print("  Valve OPEN")
        elif cmd == "valve_close" or cmd == "4":
            node.send_manual("valve_close")
            print("  Valve CLOSED")

        # ---- help ----
        elif cmd == "help":
            print("""
  Commands:
    w/s/a/d      nudge ±Y / ±X  (5 mm)
    W/S/A/D      nudge ±Y / ±X  (0.5 mm fine)
    g <hole>     go to hole nominal (e.g. g F4)
    r            record current position for selected hole
    offset       print offset summary (nominal vs actual) in mm
    holes        list all nominal hole positions
    pos          show current position
    home         go to home
    1/2          servo down / up
    3/4          valve open / close
    q            quit
""")

        elif cmd == "q":
            break

        else:
            print(f"  Unknown command '{cmd}'. Type 'help'.")

    print("\nFinal offset summary:")
    if recorded:
        dx_sum, dy_sum, n = 0.0, 0.0, 0
        for name, (act_ix, act_iy) in recorded.items():
            nom_wx, nom_wy, _ = ALL_HOLES[name]
            act_wx, act_wy = ik_to_world(act_ix, act_iy)
            dx = (act_wx - nom_wx) * 1000
            dy = (act_wy - nom_wy) * 1000
            dx_sum += dx
            dy_sum += dy
            n += 1
            print(f"  {name}: ΔX={dx:+.3f} mm, ΔY={dy:+.3f} mm")
        if n > 0:
            print(f"\n  ★ AVG offset: ΔX={dx_sum/n:+.3f} mm, ΔY={dy_sum/n:+.3f} mm")
    else:
        print("  (no holes recorded)")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
