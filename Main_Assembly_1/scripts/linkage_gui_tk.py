#!/usr/bin/env python3
"""
5-Bar Linkage 2D Visualizer — Pure Tkinter (no matplotlib)
===========================================================
Shows the full 5-bar parallel linkage in IK plane (mm).
Click holes to move, drag on canvas, or type angles manually.
Displays stepper angles, URDF angles, step counts, elbow angles.

Optional: connect to Arduino serial to send angles live.

Usage:
    python3 linkage_gui_tk.py                       # GUI only
    python3 linkage_gui_tk.py --port /dev/ttyUSB0   # GUI + Arduino
"""

import math
import sys
import time
import tkinter as tk

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PointStamped
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

# ─────────────────────────────────────────────────────────────────────
#  Robot geometry  (must match Arduino + ROS + test_stepper_angles.py)
# ─────────────────────────────────────────────────────────────────────
L1 = 200.0          # crank length (mm)
L2 = 200.0          # coupler length (mm)
d  = 190.0          # motor separation (mm)

STEPS_PER_REV    = 200.0
MICROSTEPS       = 16.0
STEPS_PER_DEGREE = (STEPS_PER_REV * MICROSTEPS) / 360.0   # 8.8889

# Motor pivot positions in IK plane (mm)
ML = (-d / 2.0, 0.0)   # left motor
MR = ( d / 2.0, 0.0)   # right motor

# ─────────────────────────────────────────────────────────────────────
#  IK origin in world frame (metres) – for world↔IK conversion
# ─────────────────────────────────────────────────────────────────────
_BASE_LINK_OX = -0.006403
_BASE_LINK_OY = -0.103113
_ML_X = 0.695406926143987 + _BASE_LINK_OX
_ML_Y = -1.58664693818658 + _BASE_LINK_OY
_MR_X = 0.885406926135972 + _BASE_LINK_OX
_MR_Y = -1.58664693819113 + _BASE_LINK_OY
IK_ORIGIN_X = (_ML_X + _MR_X) / 2.0
IK_ORIGIN_Y = (_ML_Y + _MR_Y) / 2.0

# ─────────────────────────────────────────────────────────────────────
#  Hole positions (world metres)
# ─────────────────────────────────────────────────────────────────────
HOLES_WORLD = {
    "F0": (0.724254, -1.440010), "F1": (0.724254, -1.395010), "F2": (0.724254, -1.350010),
    "F3": (0.769254, -1.440010), "F4": (0.769254, -1.395010), "F5": (0.769254, -1.350010),
    "F6": (0.814254, -1.440010), "F7": (0.814254, -1.395010), "F8": (0.814254, -1.350010),
    "L0": (0.594254, -1.600010), "L1": (0.549254, -1.600010), "L2": (0.504254, -1.600010),
    "L3": (0.594254, -1.645010), "L4": (0.549254, -1.645010), "L5": (0.504254, -1.645010),
    "L6": (0.594254, -1.690010), "L7": (0.549254, -1.690010), "L8": (0.504254, -1.690010),
}


def world_to_ik_mm(wx, wy):
    ix = (wx - IK_ORIGIN_X) * 1000.0
    iy = (wy - IK_ORIGIN_Y) * 1000.0
    iy = max(0.1, iy)
    return ix, iy


# Pre-compute hole IK positions
HOLES_IK = {}
for _n, (_wx, _wy) in HOLES_WORLD.items():
    HOLES_IK[_n] = world_to_ik_mm(_wx, _wy)


# ─────────────────────────────────────────────────────────────────────
#  IK solver  (law of cosines — same as everywhere)
# ─────────────────────────────────────────────────────────────────────
def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def ik_solve(px, py):
    """Returns (theta_L_deg, theta_R_deg) or None."""
    b1x, b2x = ML[0], MR[0]
    r1 = math.hypot(px - b1x, py)
    r2 = math.hypot(px - b2x, py)
    rmin, rmax = abs(L1 - L2), L1 + L2
    if r1 < rmin - 0.1 or r1 > rmax + 0.1:
        return None
    if r2 < rmin - 0.1 or r2 > rmax + 0.1:
        return None
    phi1 = math.atan2(py, px - b1x)
    ca1  = _clamp((L1**2 + r1**2 - L2**2) / (2 * L1 * r1), -1, 1)
    th1  = phi1 + math.acos(ca1)
    phi2 = math.atan2(py, px - b2x)
    ca2  = _clamp((L1**2 + r2**2 - L2**2) / (2 * L1 * r2), -1, 1)
    th2  = phi2 - math.acos(ca2)
    return math.degrees(th1), math.degrees(th2)


def fk_elbow(mx, my, theta_deg):
    """Elbow position (end of crank L1)."""
    rad = math.radians(theta_deg)
    return mx + L1 * math.cos(rad), my + L1 * math.sin(rad)


def fk_full(px, py):
    """Full FK → dict with motor pivots, elbows, EE, elbow angles, or None."""
    sol = ik_solve(px, py)
    if sol is None:
        return None
    th_L, th_R = sol
    eL = fk_elbow(ML[0], ML[1], th_L)
    eR = fk_elbow(MR[0], MR[1], th_R)

    def _elbow_angle(motor, elbow, ee):
        v1 = (motor[0] - elbow[0], motor[1] - elbow[1])
        v2 = (ee[0] - elbow[0], ee[1] - elbow[1])
        dot = v1[0]*v2[0] + v1[1]*v2[1]
        m1 = math.hypot(*v1)
        m2 = math.hypot(*v2)
        if m1 < 1e-9 or m2 < 1e-9:
            return 0.0
        return math.degrees(math.acos(_clamp(dot / (m1 * m2), -1, 1)))

    return {
        "th_L": th_L, "th_R": th_R,
        "eL": eL, "eR": eR, "ee": (px, py),
        "elbow_L": _elbow_angle(ML, eL, (px, py)),
        "elbow_R": _elbow_angle(MR, eR, (px, py)),
    }


# ─────────────────────────────────────────────────────────────────────
#  Canvas coordinate transform   IK(mm) ↔ pixel
#  IK: +X right, +Y up.   Canvas: +X right, +Y down.
# ─────────────────────────────────────────────────────────────────────
class View:
    def __init__(self, cw, ch, scale=1.35):
        self.cw = cw
        self.ch = ch
        self.scale = scale
        # Origin (motors baseline) sits at 75% down the canvas
        self.origin_py = int(ch * 0.75)

    def to_px(self, ik_x, ik_y):
        px = self.cw / 2 + ik_x * self.scale
        py = self.origin_py - ik_y * self.scale
        return px, py

    def to_ik(self, px, py):
        ik_x = (px - self.cw / 2) / self.scale
        ik_y = (self.origin_py - py) / self.scale
        return ik_x, ik_y

    def resize(self, cw, ch):
        self.cw = cw
        self.ch = ch
        self.origin_py = int(ch * 0.75)


# ─────────────────────────────────────────────────────────────────────
#  Optional Arduino serial
# ─────────────────────────────────────────────────────────────────────
class ArduinoLink:
    def __init__(self, port, baud=115200):
        import serial
        self.ser = serial.Serial(port, baud, timeout=1.0)
        time.sleep(2.5)
        self.ser.reset_input_buffer()
        for _ in range(10):
            line = self.ser.readline().decode(errors="replace").strip()
            if "READY" in line:
                break

    def send(self, th_L, th_R):
        self.ser.write(f"A{th_L:.2f} B{th_R:.2f}\n".encode())
        self.ser.flush()
        resp = []
        time.sleep(0.05)
        while self.ser.in_waiting:
            resp.append(self.ser.readline().decode(errors="replace").strip())
        return resp

    def home(self):
        self.ser.write(b"G28\n")
        self.ser.flush()

    def close(self):
        try:
            self.ser.write(b"M18\n")
            self.ser.flush()
            self.ser.close()
        except Exception:
            pass


# ╔═══════════════════════════════════════════════════════════════════╗
#  Main GUI class
# ╚═══════════════════════════════════════════════════════════════════╝
class LinkageApp:
    ros_node = None
    ros_pub = None
    CW = 900
    CH = 750
    SCALE = 1.35

    # ── Catppuccin Mocha palette ──
    BG       = "#1e1e2e"
    GRID     = "#313244"
    AXIS     = "#585b70"
    MOTOR    = "#f38ba8"
    CRANK_L  = "#89b4fa"
    CRANK_R  = "#a6e3a1"
    COUPLER  = "#fab387"
    EE_COL   = "#f9e2af"
    HOLE_F   = "#74c7ec"
    HOLE_L   = "#cba6f7"
    HL       = "#f38ba8"
    TEXT     = "#cdd6f4"
    WARN     = "#f38ba8"
    OK       = "#a6e3a1"
    DIM      = "#45475a"
    TRAIL    = "#585b70"

    def __init__(self, root, arduino=None):
        self.root = root
        self.ard = arduino
        root.title("5-Bar Linkage  ·  Stepper Angle Tester")
        root.configure(bg=self.BG)

        self.view = View(self.CW, self.CH, self.SCALE)

        # State
        self.ee_x, self.ee_y = 0.0, 250.0
        self.sel_hole = "Home"
        self.trail = []

        # Animation
        self._anim = False
        self._anim_t0 = 0.0
        self._anim_dur = 0.6
        self._anim_src = (0.0, 250.0)
        self._anim_dst = (0.0, 250.0)

        # Sequence
        self._seq = []

        # ROS2 node and publisher
        if ROS_AVAILABLE:
            rclpy.init(args=None)
            self.ros_node = rclpy.create_node('linkage_gui_publisher')
            self.ros_pub = self.ros_node.create_publisher(PointStamped, 'linkage_gui/ee_point', 10)
        else:
            self.ros_node = None
            self.ros_pub = None

        self._build()
        self._redraw()

    # ────────────────── build UI ──────────────────
    def _build(self):
        top = tk.Frame(self.root, bg=self.BG)
        top.pack(fill=tk.BOTH, expand=True)

        # Canvas
        self.cv = tk.Canvas(top, width=self.CW, height=self.CH,
                            bg=self.BG, highlightthickness=0)
        self.cv.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4, pady=4)

        # Right panel
        pnl = tk.Frame(top, bg=self.BG, width=310)
        pnl.pack(side=tk.RIGHT, fill=tk.Y, padx=4, pady=4)
        pnl.pack_propagate(False)

        # Title
        tk.Label(pnl, text="5-Bar Linkage", font=("Consolas", 15, "bold"),
                 bg=self.BG, fg=self.EE_COL).pack(pady=(4, 0))
        tk.Label(pnl, text="Stepper Angle Tester", font=("Consolas", 9),
                 bg=self.BG, fg=self.TEXT).pack(pady=(0, 8))

        # ── Angle readouts ──
        af = tk.LabelFrame(pnl, text=" Angles ", font=("Consolas", 9, "bold"),
                           bg=self.BG, fg=self.TEXT, bd=1)
        af.pack(fill=tk.X, padx=4, pady=2)

        self.v_hole = self._row(af, "Target:")
        self.v_ee   = self._row(af, "EE (mm):")
        self.v_sL   = self._row(af, "Stepper L°:")
        self.v_sR   = self._row(af, "Stepper R°:")
        self.v_uL   = self._row(af, "URDF L°:")
        self.v_uR   = self._row(af, "URDF R°:")
        self.v_stL  = self._row(af, "Steps L:")
        self.v_stR  = self._row(af, "Steps R:")
        self.v_eaL  = self._row(af, "Elbow L°:")
        self.v_eaR  = self._row(af, "Elbow R°:")
        self.v_stat = self._row(af, "Status:")

        # ── Front tray ──
        ff = tk.LabelFrame(pnl, text=" Front Tray ", font=("Consolas", 9, "bold"),
                           bg=self.BG, fg=self.HOLE_F, bd=1)
        ff.pack(fill=tk.X, padx=4, pady=2)
        self._hole_grid(ff, "F", self.HOLE_F)

        # ── Left tray ──
        lf = tk.LabelFrame(pnl, text=" Left Tray ", font=("Consolas", 9, "bold"),
                           bg=self.BG, fg=self.HOLE_L, bd=1)
        lf.pack(fill=tk.X, padx=4, pady=2)
        self._hole_grid(lf, "L", self.HOLE_L)

        # ── Commands ──
        cf = tk.Frame(pnl, bg=self.BG)
        cf.pack(fill=tk.X, padx=4, pady=4)
        tk.Button(cf, text="⌂ HOME", font=("Consolas", 10, "bold"),
                  bg=self.DIM, fg=self.EE_COL, activebackground="#585b70", bd=0,
                  command=lambda: self._goto(0, 250, "Home")).pack(
                      side=tk.LEFT, padx=2, expand=True, fill=tk.X)
        tk.Button(cf, text="↻ Trail", font=("Consolas", 9),
                  bg=self.DIM, fg=self.TEXT, activebackground="#585b70", bd=0,
                  command=self._clear_trail).pack(
                      side=tk.LEFT, padx=2, expand=True, fill=tk.X)

        # ── Manual angles ──
        mf = tk.LabelFrame(pnl, text=" Manual Stepper Angles ",
                           font=("Consolas", 9, "bold"), bg=self.BG, fg=self.TEXT, bd=1)
        mf.pack(fill=tk.X, padx=4, pady=2)
        r1 = tk.Frame(mf, bg=self.BG)
        r1.pack(fill=tk.X, padx=2, pady=2)
        tk.Label(r1, text="L°:", font=("Consolas", 9), bg=self.BG,
                 fg=self.CRANK_L).pack(side=tk.LEFT)
        self.e_aL = tk.Entry(r1, width=8, font=("Consolas", 10),
                             bg="#313244", fg=self.TEXT, insertbackground=self.TEXT, bd=0)
        self.e_aL.pack(side=tk.LEFT, padx=2)
        self.e_aL.insert(0, "117.23")
        tk.Label(r1, text="R°:", font=("Consolas", 9), bg=self.BG,
                 fg=self.CRANK_R).pack(side=tk.LEFT, padx=(8, 0))
        self.e_aR = tk.Entry(r1, width=8, font=("Consolas", 10),
                             bg="#313244", fg=self.TEXT, insertbackground=self.TEXT, bd=0)
        self.e_aR.pack(side=tk.LEFT, padx=2)
        self.e_aR.insert(0, "62.77")
        tk.Button(mf, text="Go to Angles", font=("Consolas", 9, "bold"),
                  bg=self.DIM, fg=self.EE_COL, bd=0,
                  command=self._go_angles).pack(fill=tk.X, padx=2, pady=3)

        # ── Sequences ──
        sf = tk.LabelFrame(pnl, text=" Sequences ", font=("Consolas", 9, "bold"),
                           bg=self.BG, fg=self.TEXT, bd=1)
        sf.pack(fill=tk.X, padx=4, pady=2)
        sr = tk.Frame(sf, bg=self.BG)
        sr.pack(fill=tk.X, padx=2, pady=2)
        tk.Button(sr, text="▶ Front", font=("Consolas", 9),
                  bg=self.DIM, fg=self.HOLE_F, bd=0,
                  command=lambda: self._run_seq(
                      ["Home"] + [f"F{i}" for i in range(9)] + ["Home"]
                  )).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)
        tk.Button(sr, text="▶ Left", font=("Consolas", 9),
                  bg=self.DIM, fg=self.HOLE_L, bd=0,
                  command=lambda: self._run_seq(
                      ["Home"] + [f"L{i}" for i in range(9)] + ["Home"]
                  )).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)
        tk.Button(sr, text="▶ ALL", font=("Consolas", 9, "bold"),
                  bg=self.DIM, fg=self.EE_COL, bd=0,
                  command=lambda: self._run_seq(
                      ["Home"] + [f"F{i}" for i in range(9)]
                      + ["Home"] + [f"L{i}" for i in range(9)] + ["Home"]
                  )).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)

        # Speed
        sp = tk.Frame(sf, bg=self.BG)
        sp.pack(fill=tk.X, padx=2, pady=2)
        tk.Label(sp, text="Speed (s):", font=("Consolas", 8),
                 bg=self.BG, fg=self.TEXT).pack(side=tk.LEFT)
        self.spd = tk.DoubleVar(value=0.6)
        tk.Scale(sp, from_=0.1, to=3.0, resolution=0.1, orient=tk.HORIZONTAL,
                 variable=self.spd, length=180,
                 bg=self.BG, fg=self.TEXT, troughcolor="#313244",
                 highlightthickness=0, bd=0, font=("Consolas", 8)).pack(
                     side=tk.LEFT, fill=tk.X, expand=True)

        # ── Arduino status ──
        if self.ard:
            tk.Label(pnl, text="🔌 Arduino CONNECTED", font=("Consolas", 9, "bold"),
                     bg=self.BG, fg=self.OK).pack(pady=4)
        else:
            tk.Label(pnl, text="Arduino: not connected", font=("Consolas", 9),
                     bg=self.BG, fg="#585b70").pack(pady=4)

        self.send_ard = tk.BooleanVar(value=(self.ard is not None))
        tk.Checkbutton(pnl, text="Send angles to Arduino", variable=self.send_ard,
                       font=("Consolas", 9), bg=self.BG, fg=self.TEXT,
                       selectcolor="#313244", activebackground=self.BG).pack(pady=2)

        # ── Bindings ──
        self.cv.bind("<Button-1>", self._click)
        self.cv.bind("<B1-Motion>", self._drag)
        self.cv.bind("<Configure>", self._resize)

    # helpers
    def _row(self, parent, label):
        f = tk.Frame(parent, bg=self.BG)
        f.pack(fill=tk.X, padx=3, pady=1)
        tk.Label(f, text=label, width=12, anchor="w", font=("Consolas", 9),
                 bg=self.BG, fg="#a6adc8").pack(side=tk.LEFT)
        v = tk.Label(f, text="--", anchor="w", font=("Consolas", 10, "bold"),
                     bg=self.BG, fg=self.TEXT)
        v.pack(side=tk.LEFT, fill=tk.X)
        return v

    def _hole_grid(self, parent, prefix, color):
        for ri in range(3):
            rf = tk.Frame(parent, bg=self.BG)
            rf.pack(fill=tk.X, padx=2, pady=1)
            for ci in range(3):
                n = f"{prefix}{ri*3+ci}"
                tk.Button(rf, text=n, width=5, font=("Consolas", 9),
                          bg="#313244", fg=color, activebackground="#45475a", bd=0,
                          command=lambda h=n: self._go_hole(h)).pack(
                              side=tk.LEFT, padx=2, pady=1, expand=True, fill=tk.X)

    # ────────────────── actions ──────────────────
    def _go_hole(self, name):
        if name == "Home":
            self._goto(0, 250, "Home")
        elif name in HOLES_IK:
            ix, iy = HOLES_IK[name]
            self._goto(ix, iy, name)

    def _goto(self, tx, ty, label=None):
        self._anim_src = (self.ee_x, self.ee_y)
        self._anim_dst = (tx, ty)
        self._anim_t0 = time.time()
        self._anim_dur = self.spd.get()
        self.sel_hole = label or f"({tx:.0f},{ty:.0f})"
        if not self._anim:
            self._anim = True
            self._tick_anim()
        self._publish_ee_point(tx, ty)

    def _go_angles(self):
        try:
            aL = float(self.e_aL.get())
            aR = float(self.e_aR.get())
        except ValueError:
            self.v_stat.config(text="Bad angle!", fg=self.WARN)
            return
        eL = fk_elbow(ML[0], ML[1], aL)
        eR = fk_elbow(MR[0], MR[1], aR)
        ee = self._circ_int(eL, eR, L2, L2)
        if ee is None:
            self.v_stat.config(text="No FK solution!", fg=self.WARN)
            return
        self._goto(ee[0], ee[1], f"Manual({aL:.1f}°,{aR:.1f}°)")

    @staticmethod
    def _circ_int(c1, c2, r1, r2):
        dx, dy = c2[0]-c1[0], c2[1]-c1[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-9 or dist > r1+r2+0.1:
            return None
        a = (r1**2 - r2**2 + dist**2) / (2*dist)
        h2 = r1**2 - a**2
        if h2 < 0:
            return None
        h = math.sqrt(h2)
        mx = c1[0] + a*dx/dist
        my = c1[1] + a*dy/dist
        x1 = mx + h*dy/dist;  y1 = my - h*dx/dist
        x2 = mx - h*dy/dist;  y2 = my + h*dx/dist
        return (x1, y1) if y1 >= y2 else (x2, y2)

    def _clear_trail(self):
        self.trail.clear()
        self._redraw()

    # ────────────────── sequence ──────────────────
    def _run_seq(self, holes):
        if self._seq:
            return
        self._seq = list(holes)
        self._seq_step()

    def _seq_step(self):
        if not self._seq:
            return
        h = self._seq.pop(0)
        self._go_hole(h)
        delay = int(self.spd.get() * 1000) + 300
        self.root.after(delay, self._seq_step)

    # ────────────────── animation ──────────────────
    def _tick_anim(self):
        if not self._anim:
            return
        t = (time.time() - self._anim_t0) / self._anim_dur
        if t >= 1.0:
            t = 1.0
            self._anim = False
        # min-jerk ease
        s = 10*t**3 - 15*t**4 + 6*t**5
        self.ee_x = self._anim_src[0] + s*(self._anim_dst[0] - self._anim_src[0])
        self.ee_y = self._anim_src[1] + s*(self._anim_dst[1] - self._anim_src[1])
        self.trail.append((self.ee_x, self.ee_y))
        if len(self.trail) > 300:
            self.trail.pop(0)
        self._redraw()
        if not self._anim:
            self._send_ard()
        if self._anim:
            self.root.after(16, self._tick_anim)

    # ────────────────── arduino ──────────────────
    def _send_ard(self):
        if not self.ard or not self.send_ard.get():
            return
        sol = ik_solve(self.ee_x, self.ee_y)
        if sol is None:
            return
        resp = self.ard.send(sol[0], sol[1])
        if resp:
            self.v_stat.config(text=resp[-1][:30], fg=self.OK)

    # ────────────────── canvas events ──────────────────
    def _click(self, ev):
        for n, (hx, hy) in HOLES_IK.items():
            px, py = self.view.to_px(hx, hy)
            if math.hypot(ev.x-px, ev.y-py) < 14:
                self._go_hole(n)
                return
        ix, iy = self.view.to_ik(ev.x, ev.y)
        if iy > 0 and ik_solve(ix, iy):
            self._goto(ix, iy)

    def _drag(self, ev):
        ix, iy = self.view.to_ik(ev.x, ev.y)
        if iy < 1 or ik_solve(ix, iy) is None:
            return
        self.ee_x, self.ee_y = ix, iy
        self.sel_hole = f"({ix:.0f},{iy:.0f})"
        self.trail.append((ix, iy))
        if len(self.trail) > 300:
            self.trail.pop(0)
        self._redraw()
        self._publish_ee_point(ix, iy)
    def _publish_ee_point(self, ik_x, ik_y):
        # Publish EE position to ROS2 topic as PointStamped (in world coordinates)
        if self.ros_pub is None:
            return
        # Convert IK mm to world meters
        wx = IK_ORIGIN_X + ik_x / 1000.0
        wy = IK_ORIGIN_Y + ik_y / 1000.0
        msg = PointStamped()
        msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # or 'world', as appropriate for RViz
        msg.point.x = wx
        msg.point.y = wy
        msg.point.z = 0.0
        self.ros_pub.publish(msg)

    def _resize(self, ev):
        self.view.resize(ev.width, ev.height)
        self._redraw()

    # ╔═════════════════════════════════════════════════════════════╗
    #  DRAWING
    # ╚═════════════════════════════════════════════════════════════╝
    def _redraw(self):
        c = self.cv
        v = self.view
        c.delete("all")

        self._draw_grid(c, v)
        self._draw_workspace_arcs(c, v)
        self._draw_trail(c, v)
        self._draw_holes(c, v)

        fk = fk_full(self.ee_x, self.ee_y)
        if fk is None:
            c.create_text(v.cw//2, v.ch//2, text="UNREACHABLE",
                          font=("Consolas", 22, "bold"), fill=self.WARN)
            self._update_labels(None)
            return

        self._draw_linkage(c, v, fk)
        self._update_labels(fk)

    # ── grid ──
    def _draw_grid(self, c, v):
        for g in range(-400, 401, 50):
            px, _ = v.to_px(g, 0)
            c.create_line(px, 0, px, v.ch, fill=self.GRID, width=1)
            _, py = v.to_px(0, g)
            c.create_line(0, py, v.cw, py, fill=self.GRID, width=1)
        # axes
        xl, ay = v.to_px(-400, 0); xr, _ = v.to_px(400, 0)
        c.create_line(xl, ay, xr, ay, fill=self.AXIS, width=1, dash=(4, 4))
        ax, yt = v.to_px(0, 450); _, yb = v.to_px(0, -50)
        c.create_line(ax, yt, ax, yb, fill=self.AXIS, width=1, dash=(4, 4))
        ox, oy = v.to_px(0, 0)
        c.create_text(ox+8, oy+8, text="Origin", font=("Consolas", 7),
                      fill=self.AXIS, anchor="nw")
        # Scale bar
        sb1x, sb1y = v.to_px(-380, -30)
        sb2x, sb2y = v.to_px(-280, -30)
        c.create_line(sb1x, sb1y, sb2x, sb2y, fill=self.TEXT, width=2)
        c.create_text((sb1x+sb2x)/2, sb1y-8, text="100 mm",
                      font=("Consolas", 7), fill=self.TEXT)

    # ── workspace arcs ──
    def _draw_workspace_arcs(self, c, v):
        for r in (100, 200, 300, 400):
            pts = []
            for i in range(61):
                a = math.pi * i / 60
                x, y = r*math.cos(a), r*math.sin(a)
                if ik_solve(x, y) is not None:
                    px, py = v.to_px(x, y)
                    pts.extend([px, py])
                else:
                    if len(pts) >= 4:
                        c.create_line(*pts, fill=self.DIM, width=1, dash=(2,4))
                    pts = []
            if len(pts) >= 4:
                c.create_line(*pts, fill=self.DIM, width=1, dash=(2,4))

    # ── trail ──
    def _draw_trail(self, c, v):
        if len(self.trail) < 2:
            return
        pts = []
        for tx, ty in self.trail:
            px, py = v.to_px(tx, ty)
            pts.extend([px, py])
        if len(pts) >= 4:
            c.create_line(*pts, fill=self.TRAIL, width=1, smooth=True)

    # ── holes ──
    def _draw_holes(self, c, v):
        R = 6
        for name, (hx, hy) in HOLES_IK.items():
            px, py = v.to_px(hx, hy)
            col = self.HOLE_F if name[0] == "F" else self.HOLE_L
            if name == self.sel_hole:
                c.create_oval(px-R-4, py-R-4, px+R+4, py+R+4,
                              outline=self.HL, width=2)
            c.create_oval(px-R, py-R, px+R, py+R, fill=col, outline="")
            c.create_text(px, py-R-6, text=name, font=("Consolas", 7, "bold"),
                          fill=col, anchor="s")

    # ── linkage ──
    def _draw_linkage(self, c, v, fk):
        # Motor pivots
        for lbl, pos, col in [("ML", ML, self.CRANK_L), ("MR", MR, self.CRANK_R)]:
            px, py = v.to_px(*pos)
            sz = 8
            c.create_rectangle(px-sz, py-sz, px+sz, py+sz, fill=self.MOTOR, outline="")
            c.create_text(px, py+sz+5, text=lbl, font=("Consolas", 7),
                          fill=self.MOTOR, anchor="n")

        # Motor baseline
        ml = v.to_px(*ML); mr = v.to_px(*MR)
        c.create_line(ml[0], ml[1], mr[0], mr[1], fill=self.MOTOR, width=2, dash=(3,3))

        eL, eR, ee = fk["eL"], fk["eR"], fk["ee"]
        el = v.to_px(*eL); er = v.to_px(*eR); ep = v.to_px(*ee)

        # Crank arms (L1)  —  thick lines
        c.create_line(ml[0], ml[1], el[0], el[1], fill=self.CRANK_L, width=5,
                      capstyle=tk.ROUND)
        c.create_line(mr[0], mr[1], er[0], er[1], fill=self.CRANK_R, width=5,
                      capstyle=tk.ROUND)

        # Coupler arms (L2)
        c.create_line(el[0], el[1], ep[0], ep[1], fill=self.COUPLER, width=3,
                      capstyle=tk.ROUND)
        c.create_line(er[0], er[1], ep[0], ep[1], fill=self.COUPLER, width=3,
                      capstyle=tk.ROUND)

        # Joints (circles)
        for jp, jc in [(el, self.CRANK_L), (er, self.CRANK_R)]:
            c.create_oval(jp[0]-5, jp[1]-5, jp[0]+5, jp[1]+5, fill=jc, outline="")

        # End-effector
        c.create_oval(ep[0]-8, ep[1]-8, ep[0]+8, ep[1]+8,
                      fill=self.EE_COL, outline="white", width=2)

        # Angle arcs at motors
        self._arc(c, v, ML, fk["th_L"], self.CRANK_L)
        self._arc(c, v, MR, fk["th_R"], self.CRANK_R)

        # Distance to nearest hole
        best_d, best_n = 1e9, ""
        for n, (hx, hy) in HOLES_IK.items():
            dd = math.hypot(ee[0]-hx, ee[1]-hy)
            if dd < best_d:
                best_d, best_n = dd, n
        if best_d < 60:
            hp = v.to_px(*HOLES_IK[best_n])
            c.create_line(ep[0], ep[1], hp[0], hp[1], fill=self.AXIS, width=1, dash=(2,3))
            c.create_text((ep[0]+hp[0])/2, (ep[1]+hp[1])/2-8,
                          text=f"{best_d:.1f}mm", font=("Consolas", 7), fill=self.TEXT)

    # ── angle arc at motor ──
    def _arc(self, c, v, motor, angle_deg, color):
        mx, my = v.to_px(*motor)
        r = 28
        # tkinter arc: angles measured counterclockwise from 3 o'clock,
        # but since canvas Y is flipped, we negate to get IK convention
        c.create_arc(mx-r, my-r, mx+r, my+r,
                     start=0, extent=-angle_deg,   # negative because Y flipped
                     outline=color, width=2, style=tk.ARC)
        mid = math.radians(angle_deg / 2)
        tx = mx + (r+14)*math.cos(mid)
        ty = my - (r+14)*math.sin(mid)
        c.create_text(tx, ty, text=f"{angle_deg:.1f}°",
                      font=("Consolas", 7, "bold"), fill=color)

    # ── update labels ──
    def _update_labels(self, fk):
        if fk is None:
            for w in (self.v_ee, self.v_sL, self.v_sR, self.v_uL, self.v_uR,
                      self.v_stL, self.v_stR, self.v_eaL, self.v_eaR):
                w.config(text="--", fg=self.WARN)
            self.v_stat.config(text="UNREACHABLE", fg=self.WARN)
            return

        tL, tR = fk["th_L"], fk["th_R"]
        uL, uR = 90.0 - tL, 90.0 - tR
        sL, sR = round(tL * STEPS_PER_DEGREE), round(tR * STEPS_PER_DEGREE)

        self.v_hole.config(text=self.sel_hole, fg=self.EE_COL)
        self.v_ee.config(text=f"({self.ee_x:.1f}, {self.ee_y:.1f})", fg=self.TEXT)
        self.v_sL.config(text=f"{tL:.2f}°", fg=self.CRANK_L)
        self.v_sR.config(text=f"{tR:.2f}°", fg=self.CRANK_R)
        self.v_uL.config(text=f"{uL:.2f}°", fg=self.CRANK_L)
        self.v_uR.config(text=f"{uR:.2f}°", fg=self.CRANK_R)
        self.v_stL.config(text=str(sL), fg=self.TEXT)
        self.v_stR.config(text=str(sR), fg=self.TEXT)

        ea_L, ea_R = fk["elbow_L"], fk["elbow_R"]
        self.v_eaL.config(text=f"{ea_L:.1f}°", fg=self.WARN if ea_L > 75 else self.OK)
        self.v_eaR.config(text=f"{ea_R:.1f}°", fg=self.WARN if ea_R > 75 else self.OK)

        if ea_L > 80 or ea_R > 80:
            self.v_stat.config(text="⚠ NEAR SINGULARITY", fg=self.WARN)
        else:
            self.v_stat.config(text="✓ OK", fg=self.OK)

        self.e_aL.delete(0, tk.END);  self.e_aL.insert(0, f"{tL:.2f}")
        self.e_aR.delete(0, tk.END);  self.e_aR.insert(0, f"{tR:.2f}")


# ─────────────────────────────────────────────────────────────────────
#  main
# ─────────────────────────────────────────────────────────────────────
def main():
    ard = None
    port = None
    for i, a in enumerate(sys.argv[1:]):
        if a == "--port" and i+1 < len(sys.argv)-1:
            port = sys.argv[i+2]

    if port:
        try:
            ard = ArduinoLink(port)
            print(f"Arduino connected on {port}")
        except Exception as e:
            print(f"Cannot connect to {port}: {e}\nRunning GUI-only.")

    root = tk.Tk()
    app = LinkageApp(root, arduino=ard)

    def _quit():
        if ard:
            ard.close()
        if ROS_AVAILABLE and app.ros_node:
            app.ros_node.destroy_node()
            rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", _quit)
    root.mainloop()


if __name__ == "__main__":
    main()
