#!/usr/bin/env python3
"""
Sci-Fi Pick & Place Command GUI — ROS 2
========================================
A futuristic HUD-style control panel for the 5-bar parallel linkage robot.

Publishes:
    /pick_place_cmd   (std_msgs/String)   — e.g. "F4 L0"
    /cancel_plan      (std_msgs/Empty)

Subscribes:
    /planner_status   (std_msgs/String)
    /ik_target        (geometry_msgs/Point)
    /suction_cmd      (std_msgs/Bool)

Usage:
    ros2 run Main_Assembly_1 pick_place_gui.py
"""

import math
import time
import random
import threading
import tkinter as tk
from tkinter import font as tkfont
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Empty
from geometry_msgs.msg import Point

# ──────────────────────────────────────────────────────────────────────
#  Colour palette — sci-fi neon on dark
# ──────────────────────────────────────────────────────────────────────
BG_DARK      = "#060612"
BG_PANEL     = "#0c0c20"
BG_CARD      = "#10102a"
BG_INPUT     = "#181838"

CYAN         = "#00e5ff"
CYAN_DIM     = "#005f6b"
CYAN_GLOW    = "#00cfff"
GREEN        = "#00ff88"
GREEN_DIM    = "#006b3a"
RED          = "#ff2255"
RED_DIM      = "#6b001a"
ORANGE       = "#ff8800"
YELLOW       = "#ffe600"
MAGENTA      = "#ff00ff"
WHITE        = "#e0e0f0"
GREY         = "#505078"
DARK_GREY    = "#282848"

# ──────────────────────────────────────────────────────────────────────
#  IK origin (world frame) — must match planner
# ──────────────────────────────────────────────────────────────────────
_BLO = (-0.006403, -0.103113)
_ML  = (0.695406926143987 + _BLO[0], -1.58664693818658 + _BLO[1])
_MR  = (0.885406926135972 + _BLO[0], -1.58664693819113 + _BLO[1])
IK_OX = (_ML[0] + _MR[0]) / 2.0
IK_OY = (_ML[1] + _MR[1]) / 2.0

# ──────────────────────────────────────────────────────────────────────
#  Hole positions (world metres)
# ──────────────────────────────────────────────────────────────────────
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

# ──────────────────────────────────────────────────────────────────────
#  Batch patterns (predefined multi-move sequences)
# ──────────────────────────────────────────────────────────────────────
BATCH_PATTERNS = {
    "Front → Left (all)": [(f"F{i}", f"L{i}") for i in range(9)],
    "Left → Front (all)": [(f"L{i}", f"F{i}") for i in range(9)],
    "Front row 0 → Left":  [(f"F{i}", f"L{i}") for i in range(3)],
    "Front row 1 → Left":  [(f"F{i}", f"L{i}") for i in range(3, 6)],
    "Front row 2 → Left":  [(f"F{i}", f"L{i}") for i in range(6, 9)],
}


class PickPlaceGUI(Node):
    """ROS 2 node with a sci-fi Tkinter GUI for pick & place commands."""

    def __init__(self):
        super().__init__("pick_place_gui")

        # Publishers
        self._cmd_pub    = self.create_publisher(String, "/pick_place_cmd", 10)
        self._cancel_pub = self.create_publisher(Empty,  "/cancel_plan",   10)

        # Subscribers
        self.create_subscription(String, "/planner_status", self._cb_status,  10)
        self.create_subscription(Point,  "/ik_target",      self._cb_ik,      10)
        self.create_subscription(Bool,   "/suction_cmd",    self._cb_suction, 10)

        # State
        self._source      = None
        self._destination = None
        self._planner_status = "IDLE"
        self._ik_pos      = (0.0, 0.0)
        self._suction_on  = False
        self._log_lines   = deque(maxlen=120)
        self._pending_logs = deque(maxlen=120)  # thread-safe queue for ROS→GUI
        self._batch_queue = deque()
        self._batch_running = False
        self._batch_trigger = False
        self._cycle_count = 0

        # Particle FX state
        self._particles = []

        self._running = True

        # Build GUI
        self._build_gui()

    # ============================================================== #
    #  ROS callbacks (run in background thread — do NOT touch Tkinter)
    # ============================================================== #
    def _cb_status(self, msg):
        self._planner_status = msg.data
        self._pending_logs.append(f"[STATUS] {msg.data}")
        # Detect cycle completion for batch queue
        if "IDLE" in msg.data.upper() and self._batch_running:
            self._batch_trigger = True

    def _cb_ik(self, msg):
        self._ik_pos = (msg.x, msg.y)

    def _cb_suction(self, msg):
        self._suction_on = msg.data

    # ============================================================== #
    #  GUI construction
    # ============================================================== #
    def _build_gui(self):
        self._root = tk.Tk()
        self._root.title("NEXUS  //  Pick & Place Command Interface")
        self._root.configure(bg=BG_DARK)
        self._root.geometry("1280x820")
        self._root.minsize(1100, 720)
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

        # Fonts
        self._fn_title  = tkfont.Font(family="Consolas", size=18, weight="bold")
        self._fn_head   = tkfont.Font(family="Consolas", size=12, weight="bold")
        self._fn_body   = tkfont.Font(family="Consolas", size=10)
        self._fn_small  = tkfont.Font(family="Consolas", size=9)
        self._fn_btn    = tkfont.Font(family="Consolas", size=13, weight="bold")
        self._fn_big    = tkfont.Font(family="Consolas", size=15, weight="bold")
        self._fn_log    = tkfont.Font(family="Consolas", size=8)
        self._fn_tiny   = tkfont.Font(family="Consolas", size=7)

        # ── Title bar ──
        title_bar = tk.Frame(self._root, bg=BG_DARK, height=52)
        title_bar.pack(fill="x", padx=0, pady=0)
        title_bar.pack_propagate(False)
        self._title_canvas = tk.Canvas(title_bar, bg=BG_DARK, highlightthickness=0, height=52)
        self._title_canvas.pack(fill="x")
        self._draw_title_bar()

        # ── Main content area ──
        main = tk.Frame(self._root, bg=BG_DARK)
        main.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        # Left column: tray panels + command bar
        left_col = tk.Frame(main, bg=BG_DARK)
        left_col.pack(side="left", fill="both", expand=True, padx=(0, 4))

        # ── Selection display ──
        sel_outer, sel_frame = self._make_card(left_col, "COMMAND  BUILDER", height=100)
        sel_outer.pack(fill="x", pady=(0, 6))
        sel_inner = tk.Frame(sel_frame, bg=BG_CARD)
        sel_inner.pack(fill="x", padx=12, pady=6)

        self._src_label = tk.Label(sel_inner, text="SOURCE", font=self._fn_small,
                                   fg=GREY, bg=BG_CARD, anchor="w")
        self._src_label.grid(row=0, column=0, padx=(0, 4))
        self._src_val = tk.Label(sel_inner, text="---", font=self._fn_big,
                                 fg=CYAN, bg=BG_CARD, width=5, anchor="center")
        self._src_val.grid(row=0, column=1, padx=(0, 20))

        tk.Label(sel_inner, text="►", font=self._fn_big, fg=GREY, bg=BG_CARD).grid(row=0, column=2, padx=8)

        self._dst_label = tk.Label(sel_inner, text="DEST", font=self._fn_small,
                                   fg=GREY, bg=BG_CARD, anchor="w")
        self._dst_label.grid(row=0, column=3, padx=(0, 4))
        self._dst_val = tk.Label(sel_inner, text="---", font=self._fn_big,
                                 fg=GREEN, bg=BG_CARD, width=5, anchor="center")
        self._dst_val.grid(row=0, column=4, padx=(0, 20))

        # Execute + Cancel + Clear buttons
        btn_frame = tk.Frame(sel_inner, bg=BG_CARD)
        btn_frame.grid(row=0, column=5, padx=(10, 0))

        self._exec_btn = self._neon_button(btn_frame, "EXECUTE", GREEN, self._on_execute, width=10)
        self._exec_btn.pack(side="left", padx=3)
        self._cancel_btn = self._neon_button(btn_frame, "ABORT", RED, self._on_cancel, width=8)
        self._cancel_btn.pack(side="left", padx=3)
        self._clear_btn = self._neon_button(btn_frame, "CLR", GREY, self._on_clear, width=5)
        self._clear_btn.pack(side="left", padx=3)

        # ── Tray panels side by side ──
        tray_row = tk.Frame(left_col, bg=BG_DARK)
        tray_row.pack(fill="both", expand=True, pady=(0, 6))

        # Front tray
        front_outer, front_content = self._make_card(tray_row, "FRONT  TRAY  //  F0–F8")
        front_outer.pack(side="left", fill="both", expand=True, padx=(0, 3))
        self._front_btns = self._make_hole_grid(front_content, "F", 3, 3)

        # Left tray
        left_outer, left_content = self._make_card(tray_row, "LEFT  TRAY  //  L0–L8")
        left_outer.pack(side="left", fill="both", expand=True, padx=(3, 0))
        self._left_btns = self._make_hole_grid(left_content, "L", 3, 3)

        # ── Batch panel ──
        batch_outer, batch_card = self._make_card(left_col, "BATCH  SEQUENCES", height=90)
        batch_outer.pack(fill="x", pady=(0, 6))
        batch_inner = tk.Frame(batch_card, bg=BG_CARD)
        batch_inner.pack(fill="x", padx=12, pady=6)
        for i, (name, _) in enumerate(BATCH_PATTERNS.items()):
            b = self._neon_button(batch_inner, name, CYAN_DIM,
                                  lambda n=name: self._on_batch(n), width=20)
            b.pack(side="left", padx=3, pady=2)

        # Right column: radar + status + log
        right_col = tk.Frame(main, bg=BG_DARK, width=380)
        right_col.pack(side="right", fill="both", padx=(4, 0))
        right_col.pack_propagate(False)

        # ── Radar / workspace map ──
        radar_outer, radar_card = self._make_card(right_col, "WORKSPACE  RADAR", height=340)
        radar_outer.pack(fill="x", pady=(0, 6))
        self._radar = tk.Canvas(radar_card, bg=BG_DARK, highlightthickness=0,
                                width=350, height=290)
        self._radar.pack(padx=10, pady=(4, 10))

        # ── Status panel ──
        status_outer, status_card = self._make_card(right_col, "SYSTEM  STATUS")
        status_outer.pack(fill="x", pady=(0, 6))
        stat_inner = tk.Frame(status_card, bg=BG_CARD)
        stat_inner.pack(fill="x", padx=12, pady=8)

        self._status_indicators = {}
        for label_text, key in [("PLANNER", "planner"), ("SUCTION", "suction"),
                                ("IK  X", "ik_x"), ("IK  Y", "ik_y"),
                                ("CYCLES", "cycles"), ("BATCH", "batch")]:
            row = tk.Frame(stat_inner, bg=BG_CARD)
            row.pack(fill="x", pady=1)
            tk.Label(row, text=label_text, font=self._fn_small, fg=GREY,
                     bg=BG_CARD, width=10, anchor="w").pack(side="left")
            val = tk.Label(row, text="---", font=self._fn_body, fg=CYAN,
                           bg=BG_CARD, anchor="w")
            val.pack(side="left", padx=(8, 0))
            self._status_indicators[key] = val

        # ── Log console ──
        log_outer, log_card = self._make_card(right_col, "EVENT  LOG")
        log_outer.pack(fill="both", expand=True, pady=(0, 0))
        self._log_text = tk.Text(log_card, bg="#05050f", fg=CYAN_DIM,
                                 font=self._fn_log, insertbackground=CYAN,
                                 selectbackground=CYAN_DIM,
                                 highlightthickness=0, relief="flat",
                                 padx=8, pady=4, wrap="word", state="disabled")
        self._log_text.pack(fill="both", expand=True, padx=6, pady=(2, 8))
        # Tag colours for log
        self._log_text.tag_configure("status", foreground=CYAN)
        self._log_text.tag_configure("cmd", foreground=GREEN)
        self._log_text.tag_configure("error", foreground=RED)
        self._log_text.tag_configure("info", foreground=GREY)

        # ── Bottom ticker / scan‑line ──
        self._scan_canvas = tk.Canvas(self._root, bg=BG_DARK, highlightthickness=0, height=3)
        self._scan_canvas.pack(fill="x")
        self._scan_x = 0

        self._log("[INIT] NEXUS Pick & Place GUI online")
        self._log(f"[INIT] Topics: /pick_place_cmd, /cancel_plan")

    # ============================================================== #
    #  Widget factory helpers
    # ============================================================== #
    def _make_card(self, parent, title, height=None):
        """Create a sci-fi bordered card panel. Returns (outer_frame, content_frame)."""
        outer = tk.Frame(parent, bg=CYAN_DIM, bd=0)
        if height:
            outer.configure(height=height)

        # Top accent line
        accent = tk.Canvas(outer, bg=CYAN_DIM, highlightthickness=0, height=1)
        accent.pack(fill="x")

        # Header
        hdr = tk.Frame(outer, bg=BG_PANEL)
        hdr.pack(fill="x")
        tk.Label(hdr, text=f"  ◆  {title}", font=self._fn_small,
                 fg=CYAN, bg=BG_PANEL, anchor="w").pack(fill="x", padx=4, pady=2)

        # Content area
        content = tk.Frame(outer, bg=BG_CARD)
        content.pack(fill="both", expand=True)

        return outer, content

    def _neon_button(self, parent, text, color, command, width=12):
        """Create a sci-fi neon-bordered button."""
        btn = tk.Button(
            parent, text=text, font=self._fn_body, fg=color, bg=BG_INPUT,
            activeforeground=BG_DARK, activebackground=color,
            highlightbackground=color, highlightcolor=color,
            highlightthickness=1, bd=0, relief="flat",
            cursor="hand2", command=command, width=width,
            padx=6, pady=4,
        )
        btn.bind("<Enter>", lambda e, b=btn, c=color: b.configure(bg=c, fg=BG_DARK))
        btn.bind("<Leave>", lambda e, b=btn, c=color: b.configure(bg=BG_INPUT, fg=c))
        return btn

    def _make_hole_grid(self, parent, prefix, rows, cols):
        """Create a 3×3 hole selection grid with sci-fi buttons."""
        frame = tk.Frame(parent, bg=BG_CARD)
        frame.pack(padx=14, pady=10)
        buttons = {}
        for r in range(rows):
            for c in range(cols):
                idx = r * cols + c
                name = f"{prefix}{idx}"
                btn = tk.Button(
                    frame, text=name, font=self._fn_btn,
                    fg=CYAN, bg=BG_INPUT, width=5, height=2,
                    activeforeground=BG_DARK, activebackground=CYAN,
                    highlightbackground=DARK_GREY, highlightcolor=CYAN,
                    highlightthickness=1, bd=0, relief="flat", cursor="hand2",
                    command=lambda n=name: self._on_hole_click(n),
                )
                btn.grid(row=r, column=c, padx=4, pady=4)
                # Hover FX
                btn.bind("<Enter>", lambda e, b=btn: b.configure(
                    bg=CYAN_DIM, highlightbackground=CYAN))
                btn.bind("<Leave>", lambda e, b=btn, n=name: self._reset_btn_style(b, n))
                buttons[name] = btn
        return buttons

    def _reset_btn_style(self, btn, name):
        """Reset button to default or selected style."""
        if name == self._source:
            btn.configure(bg=CYAN, fg=BG_DARK, highlightbackground=CYAN)
        elif name == self._destination:
            btn.configure(bg=GREEN, fg=BG_DARK, highlightbackground=GREEN)
        else:
            btn.configure(bg=BG_INPUT, fg=CYAN, highlightbackground=DARK_GREY)

    # ============================================================== #
    #  Title bar drawing
    # ============================================================== #
    def _draw_title_bar(self):
        c = self._title_canvas
        c.delete("all")
        w = max(c.winfo_width(), 1200)

        # Background gradient line
        c.create_rectangle(0, 48, w, 52, fill=CYAN_DIM, outline="")
        c.create_rectangle(0, 49, w // 3, 52, fill=CYAN, outline="")

        # Title text
        c.create_text(20, 24, text="◈  NEXUS", font=self._fn_title,
                      fill=CYAN, anchor="w")
        c.create_text(190, 24, text="//  PICK  &  PLACE  COMMAND  INTERFACE",
                      font=self._fn_head, fill=GREY, anchor="w")

        # Right-side decorative hex
        for i in range(5):
            x = w - 30 - i * 24
            c.create_text(x, 24, text="⬡", font=self._fn_body,
                          fill=CYAN_DIM if i > 0 else CYAN)

    # ============================================================== #
    #  Hole button actions
    # ============================================================== #
    def _on_hole_click(self, name):
        if self._source is None:
            self._source = name
            self._src_val.configure(text=name)
            self._highlight_btn(name, CYAN)
            self._log(f"[SELECT] Source = {name}")
            self._spawn_particles(name, CYAN)
        elif self._destination is None and name != self._source:
            self._destination = name
            self._dst_val.configure(text=name)
            self._highlight_btn(name, GREEN)
            self._log(f"[SELECT] Destination = {name}")
            self._spawn_particles(name, GREEN)
        else:
            # Re-select: clear and start over
            self._clear_selection()
            self._source = name
            self._src_val.configure(text=name)
            self._highlight_btn(name, CYAN)
            self._log(f"[SELECT] Source = {name} (reset)")

    def _highlight_btn(self, name, color):
        btns = {**self._front_btns, **self._left_btns}
        if name in btns:
            btns[name].configure(bg=color, fg=BG_DARK, highlightbackground=color)

    def _clear_selection(self):
        self._source = None
        self._destination = None
        self._src_val.configure(text="---")
        self._dst_val.configure(text="---")
        # Reset all button styles
        for name, btn in {**self._front_btns, **self._left_btns}.items():
            btn.configure(bg=BG_INPUT, fg=CYAN, highlightbackground=DARK_GREY)

    # ============================================================== #
    #  Command actions
    # ============================================================== #
    def _on_execute(self):
        if self._source and self._destination:
            cmd = f"{self._source} {self._destination}"
            msg = String()
            msg.data = cmd
            self._cmd_pub.publish(msg)
            self._cycle_count += 1
            self._log(f"[CMD] Executing: {cmd}")
            self._clear_selection()
        else:
            self._log("[ERROR] Select SOURCE and DEST first")

    def _on_cancel(self):
        msg = Empty()
        self._cancel_pub.publish(msg)
        self._batch_queue.clear()
        self._batch_running = False
        self._log("[CMD] ABORT sent → /cancel_plan")

    def _on_clear(self):
        self._clear_selection()
        self._log("[INFO] Selection cleared")

    def _on_batch(self, pattern_name):
        moves = BATCH_PATTERNS.get(pattern_name, [])
        if not moves:
            return
        self._batch_queue = deque(moves)
        self._batch_running = True
        self._log(f"[BATCH] Queued {len(moves)} moves: {pattern_name}")
        self._batch_next()

    def _batch_next(self):
        if not self._batch_queue:
            self._batch_running = False
            self._log("[BATCH] Sequence complete")
            return
        src, dst = self._batch_queue.popleft()
        cmd = f"{src} {dst}"
        msg = String()
        msg.data = cmd
        self._cmd_pub.publish(msg)
        self._cycle_count += 1
        remaining = len(self._batch_queue)
        self._log(f"[BATCH] Executing: {cmd}  ({remaining} remaining)")

    # ============================================================== #
    #  Particle effects
    # ============================================================== #
    def _spawn_particles(self, hole_name, color):
        """Spawn a burst of particles near a hole button (pure visual flair)."""
        btns = {**self._front_btns, **self._left_btns}
        if hole_name not in btns:
            return
        btn = btns[hole_name]
        try:
            x = btn.winfo_rootx() - self._root.winfo_rootx() + btn.winfo_width() // 2
            y = btn.winfo_rooty() - self._root.winfo_rooty() + btn.winfo_height() // 2
        except Exception:
            return
        t = time.monotonic()
        for _ in range(8):
            angle = random.uniform(0, 2 * math.pi)
            speed = random.uniform(40, 100)
            self._particles.append({
                "x": float(x), "y": float(y),
                "vx": math.cos(angle) * speed,
                "vy": math.sin(angle) * speed,
                "color": color,
                "born": t,
                "life": random.uniform(0.4, 0.9),
            })

    # ============================================================== #
    #  Radar drawing
    # ============================================================== #
    def _draw_radar(self):
        c = self._radar
        c.delete("all")
        W = c.winfo_width() or 350
        H = c.winfo_height() or 290

        # Map world coords to canvas
        # All holes span roughly x:[0.50, 0.82], y:[-1.70, -1.35]
        x_min, x_max = 0.48, 0.84
        y_min, y_max = -1.72, -1.32

        def w2c(wx, wy):
            cx = 20 + (wx - x_min) / (x_max - x_min) * (W - 40)
            cy = 20 + (wy - y_min) / (y_max - y_min) * (H - 40)
            return cx, cy

        # Grid lines
        for i in range(11):
            x = 20 + i * (W - 40) / 10
            c.create_line(x, 20, x, H - 20, fill=DARK_GREY, dash=(2, 4))
        for i in range(9):
            y = 20 + i * (H - 40) / 8
            c.create_line(20, y, W - 20, y, fill=DARK_GREY, dash=(2, 4))

        # IK origin cross
        ox, oy = w2c(IK_OX, IK_OY)
        c.create_line(ox - 8, oy, ox + 8, oy, fill=MAGENTA, width=1)
        c.create_line(ox, oy - 8, ox, oy + 8, fill=MAGENTA, width=1)
        c.create_text(ox + 12, oy - 8, text="IK", font=self._fn_tiny, fill=MAGENTA, anchor="w")

        # Draw holes
        for name, (wx, wy) in ALL_HOLES.items():
            cx, cy = w2c(wx, wy)
            r = 7
            fill_col = BG_DARK
            outline_col = CYAN_DIM
            if name == self._source:
                fill_col = CYAN
                outline_col = CYAN
            elif name == self._destination:
                fill_col = GREEN
                outline_col = GREEN

            c.create_oval(cx - r, cy - r, cx + r, cy + r,
                          fill=fill_col, outline=outline_col, width=1)
            c.create_text(cx, cy + r + 8, text=name, font=self._fn_tiny,
                          fill=outline_col)

        # Draw IK target position (end-effector)
        ix, iy = self._ik_pos
        ee_wx = ix + IK_OX
        ee_wy = iy + IK_OY
        ex, ey = w2c(ee_wx, ee_wy)

        # Pulsating ring
        pulse = (math.sin(time.monotonic() * 4) + 1) / 2 * 4 + 6
        c.create_oval(ex - pulse, ey - pulse, ex + pulse, ey + pulse,
                      outline=RED if self._suction_on else ORANGE, width=2)
        c.create_oval(ex - 3, ey - 3, ex + 3, ey + 3,
                      fill=RED if self._suction_on else ORANGE, outline="")

        # Line from IK origin to EE
        c.create_line(ox, oy, ex, ey, fill=ORANGE, dash=(3, 3), width=1)

        # Tray labels
        front_cx, front_cy = w2c(0.769, -1.330)
        c.create_text(front_cx, front_cy, text="FRONT", font=self._fn_tiny, fill=CYAN_DIM)
        left_cx, left_cy = w2c(0.549, -1.580)
        c.create_text(left_cx, left_cy, text="LEFT", font=self._fn_tiny, fill=CYAN_DIM)

        # Decorative corners
        for (cx, cy) in [(20, 20), (W - 20, 20), (20, H - 20), (W - 20, H - 20)]:
            c.create_line(cx - 6, cy, cx + 6, cy, fill=CYAN_DIM, width=1)
            c.create_line(cx, cy - 6, cx, cy + 6, fill=CYAN_DIM, width=1)

    # ============================================================== #
    #  Scan line animation
    # ============================================================== #
    def _draw_scan_line(self):
        c = self._scan_canvas
        c.delete("all")
        w = max(c.winfo_width(), 1200)
        self._scan_x = (self._scan_x + 4) % w
        x = self._scan_x
        # Gradient glow
        for i in range(40):
            alpha_approx = max(0, 255 - i * 6)
            # Approximate alpha with decreasing brightness hex
            bright = max(0, int(0xe5 * (1 - i / 40)))
            col = f"#{0:02x}{bright:02x}{0xff:02x}" if bright > 10 else BG_DARK
            c.create_line(x - i, 0, x - i, 3, fill=col)
        c.create_line(x, 0, x, 3, fill=CYAN, width=1)

    # ============================================================== #
    #  Status panel update
    # ============================================================== #
    def _update_status_panel(self):
        si = self._status_indicators

        # Planner status — colour based on state
        status_text = self._planner_status[:50]
        if "IDLE" in self._planner_status.upper():
            si["planner"].configure(text=status_text, fg=GREEN)
        elif "ERROR" in self._planner_status.upper() or "CANCEL" in self._planner_status.upper():
            si["planner"].configure(text=status_text, fg=RED)
        else:
            si["planner"].configure(text=status_text, fg=YELLOW)

        # Suction
        if self._suction_on:
            si["suction"].configure(text="● ENGAGED", fg=RED)
        else:
            si["suction"].configure(text="○ RELEASED", fg=GREEN)

        # IK position
        ix, iy = self._ik_pos
        si["ik_x"].configure(text=f"{ix * 1000:.1f} mm")
        si["ik_y"].configure(text=f"{iy * 1000:.1f} mm")

        # Cycles
        si["cycles"].configure(text=str(self._cycle_count))

        # Batch
        if self._batch_running:
            si["batch"].configure(text=f"RUNNING ({len(self._batch_queue)} left)", fg=ORANGE)
        else:
            si["batch"].configure(text="IDLE", fg=GREY)

    # ============================================================== #
    #  Log
    # ============================================================== #
    def _log(self, text):
        stamp = time.strftime("%H:%M:%S")
        line = f"[{stamp}] {text}"
        self._log_lines.append(line)

        self._log_text.configure(state="normal")
        # Determine tag
        tag = "info"
        if "[STATUS]" in text:
            tag = "status"
        elif "[CMD]" in text or "[BATCH]" in text:
            tag = "cmd"
        elif "[ERROR]" in text:
            tag = "error"
        self._log_text.insert("end", line + "\n", tag)
        self._log_text.see("end")
        self._log_text.configure(state="disabled")

    # ============================================================== #
    #  Shutdown
    # ============================================================== #
    def _on_close(self):
        self._running = False
        self._root.destroy()

    @staticmethod
    def _ros_spin(node):
        """Spin ROS in background, suppressing shutdown exceptions."""
        try:
            rclpy.spin(node)
        except Exception:
            pass

    def spin_gui(self):
        """Run Tkinter in the main thread, ROS spinning in a background thread."""
        spin_thread = threading.Thread(target=self._ros_spin, args=(self,), daemon=True)
        spin_thread.start()

        while self._running:
            try:
                # Drain pending log messages from ROS thread
                while self._pending_logs:
                    self._log(self._pending_logs.popleft())

                # Batch trigger from ROS thread
                if self._batch_trigger:
                    self._batch_trigger = False
                    self._batch_next()

                self._draw_radar()
                self._draw_scan_line()
                self._update_status_panel()
                self._draw_title_bar()
                self._root.update_idletasks()
                self._root.update()
                time.sleep(0.033)  # ~30 Hz
            except tk.TclError:
                break
            except Exception as e:
                self.get_logger().error(f"GUI error: {e}")
                break

        self.destroy_node()


# ================================================================== #
#  Entry point
# ================================================================== #
def main():
    rclpy.init()
    gui = PickPlaceGUI()
    try:
        gui.spin_gui()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
