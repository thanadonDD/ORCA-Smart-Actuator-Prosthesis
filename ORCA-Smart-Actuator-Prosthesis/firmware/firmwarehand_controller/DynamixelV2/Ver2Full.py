#!/usr/bin/env python3
"""
Ver2Full.py — Robot Hand Controller + Webcam Hand Tracking
U2D2 | Baudrate: 115200 | Protocol 2.0

Mode 1: GUI Control — กดปุ่มท่า / Auto Demo
Mode 2: Webcam Control — ตรวจจับมือจริงด้วย MediaPipe → ควบคุม servo real-time
"""

from dynamixel_sdk import *
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import math
import sys
import numpy as np

try:
    import cv2
    import mediapipe as mp
    from PIL import Image, ImageTk
    WEBCAM_AVAILABLE = True
except ImportError:
    WEBCAM_AVAILABLE = False
    print("[WARN] mediapipe/opencv/Pillow not installed — Webcam mode disabled")

try:
    import speech_recognition as _sr_lib
    import sounddevice as _sd_audio
    import wave as _wave_lib
    import io as _io_lib
    VOICE_AVAILABLE = True
except ImportError:
    VOICE_AVAILABLE = False
    print("[WARN] speech_recognition/sounddevice not installed — Voice mode disabled")

# ============================================================================
# --- CONFIGURATION ---
# ============================================================================
DEVICENAME       = 'COM4'
BAUDRATE         = 115200
PROTOCOL_VERSION = 2.0

ADDR_TORQUE_ENABLE      = 64
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY   = 112
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132

TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0

MARGIN = 0.20
PROFILE_VELOCITY = 100
PROFILE_ACCELERATION = 40
MONITOR_INTERVAL = 400

# Webcam settings
WEBCAM_FPS = 30
SMOOTHING_ALPHA = 0.35      # EMA α: 0.2=smooth, 0.5=responsive
ANGLE_STRAIGHT = 10.0       # มุมนิ้วตรง (degrees) (ใกล้ 0)
ANGLE_BENT = 85.0           # มุมนิ้วงอสุด (degrees) (ยิ่งงอ มุมยิ่งมาก)

# ============================================================================
# --- SERVO DATA ---
# ============================================================================
SERVO_DATA = [
    ( 5, 247.0,  306.0,  "ชี้ Base",      "INDEX"),
    ( 7, 339.0,  262.0,  "ชี้ Mid",       "INDEX"),
    ( 8, 322.0,  207.0,  "ชี้ Tip",       "INDEX"),
    (16, 315.0,  240.29, "กลาง Mid",      "MIDDLE"),
    ( 1, 170.0,   56.0,  "กลาง Tip",      "MIDDLE"),
    (15,  43.0,    0.0,  "นาง Base",      "RING"),
    (14, 249.0,  153.0,  "นาง Mid",       "RING"),
    (13, 155.0,   41.0,  "นาง Tip",       "RING"),
    ( 4, 356.0,  260.0,  "ก้อย Base",     "PINKY"),
    ( 2, 353.0,  263.0,  "ก้อย Mid",      "PINKY"),
    ( 3, 265.0,  172.0,  "ก้อย Tip",      "PINKY"),
    (11, 152.0,   70.0,  "โป้ง Yaw",      "THUMB"),
    (12, 263.0,  200.0,  "โป้ง Pitch",    "THUMB"),
    ( 9, 105.0,   17.0,  "โป้ง Mid",      "THUMB"),
    (10, 176.0,   96.0,  "โป้ง Tip",      "THUMB"),
    (17, 305.0,  217.0,  "ข้อมือ",        "WRIST"),
]

FINGER_GROUPS = ["THUMB", "INDEX", "MIDDLE", "RING", "PINKY"]
FINGER_NAMES_TH = {
    "INDEX": "นิ้วชี้", "MIDDLE": "นิ้วกลาง", "RING": "นิ้วนาง",
    "PINKY": "นิ้วก้อย", "THUMB": "นิ้วโป้ง", "WRIST": "ข้อมือ"
}
FINGER_COLORS = {
    "THUMB": "#BA68C8", "INDEX": "#4FC3F7", "MIDDLE": "#81C784",
    "RING": "#FFB74D", "PINKY": "#F06292", "WRIST": "#90A4AE"
}

# ============================================================================
# --- POSES ---
# ============================================================================
POSES = {
    "OPEN":      {"emoji": "🖐️", "name_th": "แบมือ",
                  "data": {"INDEX": 0, "MIDDLE": 0, "RING": 0, "PINKY": 0, "THUMB": 0, "WRIST": 0}},
    "FIST":      {"emoji": "✊", "name_th": "กำมือ",
                  "data": {"INDEX": 1, "MIDDLE": 1, "RING": 1, "PINKY": 1, "THUMB": 1, "WRIST": 0}},
    "PEACE":     {"emoji": "✌️", "name_th": "Peace",
                  "data": {"INDEX": 0, "MIDDLE": 0, "RING": 1, "PINKY": 1, "THUMB": 1, "WRIST": 0}},
    "THUMBS_UP": {"emoji": "👍", "name_th": "ยกนิ้วโป้ง",
                  "data": {"INDEX": 1, "MIDDLE": 1, "RING": 1, "PINKY": 1, "THUMB": 0, "WRIST": 0}},
    "POINT":     {"emoji": "☝️", "name_th": "ชี้",
                  "data": {"INDEX": 0, "MIDDLE": 1, "RING": 1, "PINKY": 1, "THUMB": 1, "WRIST": 0}},
    "ROCK":      {"emoji": "🤟", "name_th": "Rock",
                  "data": {"INDEX": 0, "MIDDLE": 1, "RING": 1, "PINKY": 0, "THUMB": 0, "WRIST": 0}},
    "THREE":     {"emoji": "3️⃣", "name_th": "สาม",
                  "data": {"INDEX": 0, "MIDDLE": 0, "RING": 0, "PINKY": 1, "THUMB": 1, "WRIST": 0}},
    "FOUR":      {"emoji": "4️⃣", "name_th": "สี่",
                  "data": {"INDEX": 0, "MIDDLE": 0, "RING": 0, "PINKY": 0, "THUMB": 1, "WRIST": 0}},
    "PINCH":     {"emoji": "🤏", "name_th": "หยิบ",
                  "data": {"INDEX": 0.6, "MIDDLE": 0, "RING": 0, "PINKY": 0, "THUMB": 0.6, "WRIST": 0}},
    "CALL":      {"emoji": "🤙", "name_th": "โทรหา",
                  "data": {"INDEX": 1, "MIDDLE": 1, "RING": 1, "PINKY": 0, "THUMB": 0, "WRIST": 0}},
    "GUN":       {"emoji": "👈", "name_th": "ปืน",
                  "data": {"INDEX": 0, "MIDDLE": 1, "RING": 1, "PINKY": 1, "THUMB": 0, "WRIST": 0}},
    "GRIP":      {"emoji": "🫴", "name_th": "จับ",
                  "data": {"INDEX": 0.5, "MIDDLE": 0.5, "RING": 0.5, "PINKY": 0.5, "THUMB": 0.5, "WRIST": 0}},
}

# คำสั่งเสียง → pose key
VOICE_COMMANDS = {
    # ภาษาไทย
    "กำมือ":     "FIST",
    "แบมือ":     "OPEN",
    "สันติภาพ": "PEACE",
    "ยกนิ้วโป้ง": "THUMBS_UP",
    "ชี้":       "POINT",
    "โบกมือ":   "WAVE_ANIM",
    "หยิบ":      "PINCH",
    "โทรหา":   "CALL",
    "ปืน":       "GUN",
    "สาม":       "THREE",
    "สี่":        "FOUR",
    "จับ":        "GRIP",
    "ร็อค์":       "ROCK",
    # English fallback
    "fist":      "FIST",
    "open":      "OPEN",
    "peace":     "PEACE",
    "point":     "POINT",
    "wave":      "WAVE_ANIM",
    "pinch":     "PINCH",
    "call":      "CALL",
    "gun":       "GUN",
    "three":     "THREE",
    "four":      "FOUR",
    "grip":      "GRIP",
    "rock":      "ROCK",
    "thumbs up": "THUMBS_UP",
    "thumb up":  "THUMBS_UP",
}

DEMO_SEQUENCE = [
    ("OPEN", 2.0), ("FIST", 2.0), ("OPEN", 1.5), ("PEACE", 2.0),
    ("THUMBS_UP", 2.0), ("POINT", 2.0), ("ROCK", 2.0), ("GUN", 2.0),
    ("CALL", 2.0), ("THREE", 1.5), ("FOUR", 1.5), ("OPEN", 1.5),
    ("PINCH", 2.0), ("GRIP", 2.0), ("FIST", 2.0), ("OPEN", 2.5),
]

WAVE_SEQUENCE = [
    {"PINKY": 1, "RING": 0, "MIDDLE": 0, "INDEX": 0, "THUMB": 0, "WRIST": 0},
    {"PINKY": 1, "RING": 1, "MIDDLE": 0, "INDEX": 0, "THUMB": 0, "WRIST": 0},
    {"PINKY": 0, "RING": 1, "MIDDLE": 1, "INDEX": 0, "THUMB": 0, "WRIST": 0},
    {"PINKY": 0, "RING": 0, "MIDDLE": 1, "INDEX": 1, "THUMB": 0, "WRIST": 0},
    {"PINKY": 0, "RING": 0, "MIDDLE": 0, "INDEX": 1, "THUMB": 1, "WRIST": 0},
    {"PINKY": 0, "RING": 0, "MIDDLE": 0, "INDEX": 0, "THUMB": 1, "WRIST": 0},
    {"PINKY": 0, "RING": 0, "MIDDLE": 0, "INDEX": 0, "THUMB": 0, "WRIST": 0},
]


# ============================================================================
# --- SERVO LOGIC ---
# ============================================================================

def deg_to_raw(degrees):
    return int(degrees * 4096.0 / 360.0)

def raw_to_deg(raw):
    return raw * 360.0 / 4096.0

def calc_target_deg(open_deg, close_deg, ratio, margin=MARGIN):
    safe_close = calc_safe_close(open_deg, close_deg, margin)
    return open_deg + (safe_close - open_deg) * ratio

def calc_safe_close(open_deg, close_deg, margin=MARGIN):
    travel = abs(open_deg - close_deg)
    margin_deg = travel * margin
    if close_deg < open_deg:
        return close_deg + margin_deg
    else:
        return close_deg - margin_deg

def get_servos_by_group(group):
    return [(sid, op, cl, name) for sid, op, cl, name, grp in SERVO_DATA if grp == group]


# ============================================================================
# --- HAND TRACKER (MediaPipe) ---
# ============================================================================

class HandTracker:
    """ตรวจจับมือจาก webcam แล้วคำนวณ curl ratio ของแต่ละนิ้ว"""

    def __init__(self):
        try:
            from mediapipe.tasks import python
            from mediapipe.tasks.python import vision
            
            base_options = python.BaseOptions(model_asset_path='hand_landmarker.task')
            options = vision.HandLandmarkerOptions(
                base_options=base_options,
                running_mode=vision.RunningMode.IMAGE,
                num_hands=1,
                min_hand_detection_confidence=0.7,
                min_hand_presence_confidence=0.6,
                min_tracking_confidence=0.6)
            
            self.landmarker = vision.HandLandmarker.create_from_options(options)
        except Exception as e:
            print(f"[ERROR] Failed to load HandLandmarker: {e}")
            self.landmarker = None

        # Smoothed ratios (EMA)
        self.smoothed = {"INDEX": 0, "MIDDLE": 0, "RING": 0, "PINKY": 0, "THUMB": 0, "THUMB_PITCH": 0}

    def process_frame(self, frame, mirror=True):
        """ประมวลผล frame → return (annotated_frame, finger_ratios)"""
        if mirror:
            frame = cv2.flip(frame, 1)

        if not self.landmarker:
            return frame, None

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        results = self.landmarker.detect(mp_image)

        ratios = None
        if results.hand_landmarks:
            lm = results.hand_landmarks[0]

            # วาด landmarks เอง
            self._draw_landmarks(frame, lm)

            # คำนวณ curl ratios
            ratios = {
                "INDEX":  self._finger_curl(lm, [5, 6, 7, 8]),
                "MIDDLE": self._finger_curl(lm, [9, 10, 11, 12]),
                "RING":   self._finger_curl(lm, [13, 14, 15, 16]),
                "PINKY":  self._finger_curl(lm, [17, 18, 19, 20]),
                "THUMB":  self._thumb_curl(lm),
                "THUMB_PITCH": self._thumb_pitch(lm),
            }

            # Apply smoothing (EMA)
            alpha = SMOOTHING_ALPHA
            for key in ratios:
                self.smoothed[key] = alpha * ratios[key] + (1 - alpha) * self.smoothed[key]

            ratios = dict(self.smoothed)

            # แสดง ratio บน frame
            y = 30
            for finger, ratio in ratios.items():
                color = (0, 255, 128) if ratio < 0.3 else (0, 200, 255) if ratio < 0.7 else (0, 100, 255)
                bar_w = int(ratio * 100)
                cv2.rectangle(frame, (10, y - 12), (10 + bar_w, y + 2), color, -1)
                cv2.rectangle(frame, (10, y - 12), (110, y + 2), (100, 100, 100), 1)
                cv2.putText(frame, f"{finger[:3]} {ratio:.0%}", (120, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
                y += 22

        return frame, ratios

    def _draw_landmarks(self, frame, landmarks):
        h, w, _ = frame.shape
        connections = [
            (0, 1), (1, 2), (2, 3), (3, 4),
            (0, 5), (5, 6), (6, 7), (7, 8),
            (5, 9), (9, 10), (10, 11), (11, 12),
            (9, 13), (13, 14), (14, 15), (15, 16),
            (13, 17), (17, 18), (18, 19), (19, 20),
            (0, 17)
        ]
        points = []
        for lm in landmarks:
            cx, cy = int(lm.x * w), int(lm.y * h)
            points.append((cx, cy))
            cv2.circle(frame, (cx, cy), 4, (0, 255, 128), -1)
            
        for connection in connections:
            p1 = points[connection[0]]
            p2 = points[connection[1]]
            cv2.line(frame, p1, p2, (0, 200, 255), 2)

    def _finger_curl(self, landmarks, ids):
        """คำนวณ curl ratio ของนิ้ว (ไม่รวมนิ้วโป้ง)
        ids = [MCP, PIP, DIP, TIP]
        ใช้ 2 วิธีรวมกัน:
          1) มุมที่ข้อ PIP (MCP→PIP vs PIP→DIP)
          2) ระยะ TIP↔MCP เทียบกับความยาวนิ้วตรง (จับงอที่ MCP ได้ด้วย)
        ใช้ค่า max ของทั้งสองวิธี
        """
        mcp = landmarks[ids[0]]
        pip = landmarks[ids[1]]
        dip = landmarks[ids[2]]
        tip = landmarks[ids[3]]

        # --- วิธี 1: มุมที่ข้อ PIP ---
        v1 = np.array([pip.x - mcp.x, pip.y - mcp.y, pip.z - mcp.z])
        v2 = np.array([dip.x - pip.x, dip.y - pip.y, dip.z - pip.z])

        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-8)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.degrees(np.arccos(cos_angle))

        ratio_angle = (angle - ANGLE_STRAIGHT) / (ANGLE_BENT - ANGLE_STRAIGHT)
        ratio_angle = float(np.clip(ratio_angle, 0.0, 1.0))

        # --- วิธี 2: ระยะ TIP↔MCP เทียบความยาวนิ้วตรง ---
        # ความยาวนิ้วตรง = MCP→PIP + PIP→DIP + DIP→TIP
        seg1 = np.linalg.norm(v1)
        v3 = np.array([dip.x - pip.x, dip.y - pip.y, dip.z - pip.z])
        v4 = np.array([tip.x - dip.x, tip.y - dip.y, tip.z - dip.z])
        seg2 = np.linalg.norm(v3)
        seg3 = np.linalg.norm(v4)
        full_len = seg1 + seg2 + seg3 + 1e-8

        # ระยะตรงจาก MCP→TIP
        dist_tip = math.sqrt(
            (tip.x - mcp.x) ** 2 +
            (tip.y - mcp.y) ** 2 +
            (tip.z - mcp.z) ** 2
        )

        # dist_ratio: 1.0 = ตรงสุด, 0.0 = งอสุด (TIP แตะ MCP)
        dist_ratio = dist_tip / full_len
        # Map: ตรง dist_ratio~0.95 → curl 0.0, งอสุด dist_ratio~0.4 → curl 1.0
        ratio_dist = (0.95 - dist_ratio) / (0.95 - 0.4)
        ratio_dist = float(np.clip(ratio_dist, 0.0, 1.0))

        # ใช้ค่า max ของทั้ง 2 วิธี (จับงอได้ทุกแบบ)
        return max(ratio_angle, ratio_dist)

    def _thumb_curl(self, landmarks):
        """คำนวณ curl ของนิ้วโป้ง (Abduction-based: scale-invariant distance)
        วัดระยะ thumb_tip → index_mcp เทียบกับขนาดฝ่ามือ
        (angle_mcp ไม่ใช้ เพราะนิ้วโป้งหุบ/กางด้วยแกน abduction ไม่ใช่ flexion)

        Calibrated values (วัดจากมือจริง):
          กางสุด  ratio_dist ~ 0.918  → curl = 0.0
          หุบสุด  ratio_dist ~ 0.346  → curl = 1.0
        """
        thumb_tip  = landmarks[4]
        index_mcp  = landmarks[5]
        wrist      = landmarks[0]
        middle_mcp = landmarks[9]

        # ขนาดฝ่ามือ (อ้างอิง)
        palm_size = math.sqrt(
            (wrist.x - middle_mcp.x) ** 2 +
            (wrist.y - middle_mcp.y) ** 2 +
            (wrist.z - middle_mcp.z) ** 2
        )
        if palm_size < 1e-6:
            return 0.0

        # ระยะ thumb_tip → index_mcp / palm_size
        dist_thumb = math.sqrt(
            (thumb_tip.x - index_mcp.x) ** 2 +
            (thumb_tip.y - index_mcp.y) ** 2 +
            (thumb_tip.z - index_mcp.z) ** 2
        )
        ratio_dist = dist_thumb / palm_size

        # Calibrated: กางสุด=0.918, หุบสุด=0.346 (เพิ่ม buffer เล็กน้อย)
        straight_ratio = 0.93   # กางสุด (0 = กาง)
        bent_ratio     = 0.33   # หุบสุด (1 = หุบ)

        ratio = (straight_ratio - ratio_dist) / (straight_ratio - bent_ratio)
        return float(np.clip(ratio, 0.0, 1.0))


    def _thumb_pitch(self, landmarks):
        """คำนวณ pitch (ยก-ก้ม) ของนิ้วโป้ง
        วัดจากตำแหน่ง Y ของ thumb tip เทียบกับ thumb CMC (landmark 1)
        scale-invariant โดยหารด้วยขนาดฝ่ามือ
        """
        thumb_tip = landmarks[4]
        thumb_cmc = landmarks[1]
        wrist = landmarks[0]
        middle_mcp = landmarks[9]

        # ขนาดฝ่ามือ (อ้างอิง)
        palm_size = math.sqrt(
            (wrist.x - middle_mcp.x) ** 2 +
            (wrist.y - middle_mcp.y) ** 2 +
            (wrist.z - middle_mcp.z) ** 2
        )

        if palm_size == 0:
            return 0.0

        # วัด delta Y (thumb tip vs thumb CMC)
        # Y ในภาพ: ยิ่งสูง y ยิ่งน้อย → thumb tip อยู่สูงกว่า CMC = ยกขึ้น
        delta_y = (thumb_cmc.y - thumb_tip.y) / palm_size

        # Map: ยกสุด delta_y ~ 0.8+, ก้มสุด delta_y ~ -0.2
        up_ratio = 0.7    # ยกสุด
        down_ratio = -0.1  # ก้มสุด

        # Map: up=open(0.0), down=close(1.0)
        ratio = (up_ratio - delta_y) / (up_ratio - down_ratio)
        return float(np.clip(ratio, 0.0, 1.0))

    def release(self):
        if self.landmarker:
            self.landmarker.close()


# ============================================================================
# --- GUI APPLICATION ---
# ============================================================================

class RobotHandApp:
    def __init__(self, root):
        self.root = root
        self.root.title("🤖 Robot Hand — GUI + Webcam Controller")
        self.root.geometry("1100x820")
        self.root.configure(bg="#0a0a1a")
        self.root.resizable(True, True)
        self.root.minsize(1000, 750)

        self.port = None
        self.pkt = None
        self.sync_write = None
        self.connected = False
        self.busy = False
        self.demo_running = False
        self.stop_demo = False
        self.monitoring = False
        self.servo_widgets = {}
        self.current_positions = {}
        self.group_margin_vars = {}  # per-group margin IntVar

        # Voice state
        self.voice_active = False
        self.voice_thread = None
        self.voice_text_var = tk.StringVar(value="—")
        self.voice_cmd_var  = tk.StringVar(value="—")
        self.voice_status_var = tk.StringVar(value="⏹️ หยุด")

        # Webcam state
        self.webcam_active = False
        self.webcam_cap = None
        self.hand_tracker = None
        self.webcam_label = None
        self.cam_index = 0
        self.current_mode = "GUI"  # "GUI" or "WEBCAM"

        self._build_ui()

    # =========================================================================
    # UI
    # =========================================================================
    def _build_ui(self):
        BG = "#0a0a1a"
        CARD = "#111827"
        TEXT = "#e2e8f0"
        MUTED = "#64748b"

        # --- HEADER ---
        header = tk.Frame(self.root, bg="#1e3a5f", height=50)
        header.pack(fill="x")
        header.pack_propagate(False)

        tk.Label(header, text="🤖 Robot Hand Controller",
                 font=("Segoe UI", 15, "bold"), fg="white", bg="#1e3a5f"
        ).pack(side="left", padx=15)

        self.status_dot = tk.Label(header, text="● Disconnected",
                                    font=("Segoe UI", 10), fg="#ef4444", bg="#1e3a5f")
        self.status_dot.pack(side="right", padx=15)

        # Mode indicator
        self.mode_label = tk.Label(header, text="Mode: GUI",
                                    font=("Segoe UI", 10, "bold"), fg="#fbbf24", bg="#1e3a5f")
        self.mode_label.pack(side="right", padx=15)

        # --- CONNECTION + MODE BAR ---
        conn = tk.Frame(self.root, bg=CARD, pady=6)
        conn.pack(fill="x", padx=10, pady=(8, 4))

        tk.Label(conn, text="Port:", font=("Segoe UI", 9), fg=TEXT, bg=CARD
        ).pack(side="left", padx=(10, 3))
        self.port_var = tk.StringVar(value=DEVICENAME)
        tk.Entry(conn, textvariable=self.port_var, font=("Consolas", 10), width=7,
                 bg="#0f172a", fg="white", insertbackground="white", relief="flat", bd=2
        ).pack(side="left", padx=3)

        tk.Label(conn, text="Baud:", font=("Segoe UI", 9), fg=TEXT, bg=CARD
        ).pack(side="left", padx=(10, 3))
        self.baud_var = tk.StringVar(value=str(BAUDRATE))
        tk.Entry(conn, textvariable=self.baud_var, font=("Consolas", 10), width=7,
                 bg="#0f172a", fg="white", insertbackground="white", relief="flat", bd=2
        ).pack(side="left", padx=3)

        self.connect_btn = tk.Button(conn, text="🔌 Connect", font=("Segoe UI", 9, "bold"),
                                      fg="white", bg="#059669", activebackground="#10b981",
                                      relief="flat", padx=12, command=self.toggle_connection,
                                      cursor="hand2")
        self.connect_btn.pack(side="left", padx=10)

        # Mode switch buttons
        tk.Label(conn, text="│", font=("Segoe UI", 12), fg=MUTED, bg=CARD).pack(side="left", padx=5)

        self.gui_mode_btn = tk.Button(conn, text="🎮 GUI Mode", font=("Segoe UI", 9, "bold"),
                                       fg="white", bg="#2563eb", relief="flat", padx=10,
                                       cursor="hand2", command=lambda: self.switch_mode("GUI"))
        self.gui_mode_btn.pack(side="left", padx=3)

        webcam_state = "normal" if WEBCAM_AVAILABLE else "disabled"
        self.webcam_mode_btn = tk.Button(conn, text="📷 Webcam Mode", font=("Segoe UI", 9, "bold"),
                                          fg="white", bg="#475569", relief="flat", padx=10,
                                          cursor="hand2", state=webcam_state,
                                          command=lambda: self.switch_mode("WEBCAM"))
        self.webcam_mode_btn.pack(side="left", padx=3)

        voice_state = "normal" if VOICE_AVAILABLE else "disabled"
        self.voice_mode_btn = tk.Button(conn, text="🎤 Voice", font=("Segoe UI", 9, "bold"),
                                         fg="white", bg="#475569", relief="flat", padx=10,
                                         cursor="hand2", state=voice_state,
                                         command=lambda: self.switch_mode("VOICE"))
        self.voice_mode_btn.pack(side="left", padx=3)

        # Margin
        self.margin_var = tk.IntVar(value=int(MARGIN * 100))
        tk.Label(conn, text="Margin:", font=("Segoe UI", 8), fg=MUTED, bg=CARD
        ).pack(side="right", padx=(0, 3))
        tk.Scale(conn, from_=0, to=40, orient="horizontal", variable=self.margin_var,
                 length=70, font=("Consolas", 7), fg=MUTED, bg=CARD,
                 highlightthickness=0, troughcolor="#1e293b", showvalue=True
        ).pack(side="right", padx=(0, 5))

        # --- MAIN CONTENT ---
        self.main_frame = tk.Frame(self.root, bg=BG)
        self.main_frame.pack(fill="both", expand=True, padx=10, pady=4)
        self.main_frame.columnconfigure(0, weight=3)
        self.main_frame.columnconfigure(1, weight=2)

        # LEFT COLUMN
        self.left_col = tk.Frame(self.main_frame, bg=BG)
        self.left_col.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        self._build_gui_panel(self.left_col)

        # RIGHT COLUMN (will switch between servo status and webcam)
        self.right_col = tk.Frame(self.main_frame, bg=BG)
        self.right_col.grid(row=0, column=1, sticky="nsew", padx=(5, 0))

        # Build all panels (show/hide)
        self._build_servo_panel(self.right_col)
        self._build_webcam_panel(self.right_col)
        self._build_voice_panel(self.right_col)

        # Show servo panel by default
        self.servo_frame.pack(fill="both", expand=True)
        self.webcam_frame.pack_forget()
        self.voice_frame.pack_forget()

        # --- LOG ---
        log_frame = tk.Frame(self.root, bg="#111827", height=30)
        log_frame.pack(fill="x", padx=10, pady=(0, 8))
        self.log_var = tk.StringVar(value="พร้อมใช้งาน — กด Connect เพื่อเริ่ม")
        tk.Label(log_frame, textvariable=self.log_var,
                 font=("Segoe UI", 9), fg="#94a3b8", bg="#111827", anchor="w"
        ).pack(fill="x", padx=10, pady=4)

    def _build_gui_panel(self, parent):
        """สร้าง panel ฝั่งซ้าย (ปุ่มท่า, animation, demo)"""
        BG = "#0a0a1a"
        CARD = "#111827"
        TEXT = "#e2e8f0"
        MUTED = "#64748b"

        # Main actions
        act = tk.Frame(parent, bg=BG)
        act.pack(fill="x", pady=(0, 5))
        self.open_btn = tk.Button(act, text="🖐️ แบมือ", font=("Segoe UI", 13, "bold"),
                                   fg="white", bg="#2563eb", activebackground="#3b82f6",
                                   relief="flat", height=2, cursor="hand2",
                                   command=lambda: self.run_async(self.do_open_hand), state="disabled")
        self.open_btn.pack(side="left", expand=True, fill="x", padx=(0, 3))
        self.close_btn = tk.Button(act, text="✊ กำมือ", font=("Segoe UI", 13, "bold"),
                                    fg="white", bg="#dc2626", activebackground="#ef4444",
                                    relief="flat", height=2, cursor="hand2",
                                    command=lambda: self.run_async(self.do_close_hand), state="disabled")
        self.close_btn.pack(side="left", expand=True, fill="x", padx=(3, 0))

        # Poses
        pose_lf = tk.LabelFrame(parent, text="  ท่าโชว์  ", font=("Segoe UI", 9, "bold"),
                                 fg=TEXT, bg=CARD, bd=1, relief="groove")
        pose_lf.pack(fill="x", pady=(0, 4))
        pose_inner = tk.Frame(pose_lf, bg=CARD)
        pose_inner.pack(pady=6, padx=6)
        self.pose_btns = {}
        for i, key in enumerate(POSES.keys()):
            p = POSES[key]
            r, c = divmod(i, 4)
            btn = tk.Button(pose_inner, text=f"{p['emoji']}\n{p['name_th']}",
                            font=("Segoe UI", 9), fg="white", bg="#334155",
                            activebackground="#475569", relief="flat", width=9, height=2,
                            cursor="hand2", state="disabled",
                            command=lambda k=key: self.run_async(lambda: self.do_pose(k)))
            btn.grid(row=r, column=c, padx=2, pady=2, sticky="nsew")
            self.pose_btns[key] = btn
            pose_inner.columnconfigure(c, weight=1)

        # Animations
        anim_lf = tk.LabelFrame(parent, text="  Animation  ", font=("Segoe UI", 9, "bold"),
                                 fg=TEXT, bg=CARD, bd=1, relief="groove")
        anim_lf.pack(fill="x", pady=(0, 4))
        anim_inner = tk.Frame(anim_lf, bg=CARD)
        anim_inner.pack(pady=6, padx=6, fill="x")
        self.wave_btn = tk.Button(anim_inner, text="👋 Wave", font=("Segoe UI", 9),
                                   fg="white", bg="#7c3aed", relief="flat", height=2,
                                   cursor="hand2", state="disabled",
                                   command=lambda: self.run_async(self.do_wave))
        self.wave_btn.pack(side="left", padx=2, expand=True, fill="x")
        self.count_btn = tk.Button(anim_inner, text="🔢 Count", font=("Segoe UI", 9),
                                    fg="white", bg="#0891b2", relief="flat", height=2,
                                    cursor="hand2", state="disabled",
                                    command=lambda: self.run_async(self.do_count))
        self.count_btn.pack(side="left", padx=2, expand=True, fill="x")
        self.flex_btn = tk.Button(anim_inner, text="💪 Flex", font=("Segoe UI", 9),
                                   fg="white", bg="#ea580c", relief="flat", height=2,
                                   cursor="hand2", state="disabled",
                                   command=lambda: self.run_async(self.do_flex))
        self.flex_btn.pack(side="left", padx=2, expand=True, fill="x")

        # Per-finger margin
        margin_lf = tk.LabelFrame(parent, text="  📐 Margin ต่อนิ้ว  ", font=("Segoe UI", 9, "bold"),
                                   fg=TEXT, bg=CARD, bd=1, relief="groove")
        margin_lf.pack(fill="x", pady=(0, 4))
        margin_inner = tk.Frame(margin_lf, bg=CARD)
        margin_inner.pack(pady=4, padx=6, fill="x")

        default_margins = {"THUMB": 20, "INDEX": 20, "MIDDLE": 20, "RING": 20, "PINKY": 20}
        for grp in ["THUMB", "INDEX", "MIDDLE", "RING", "PINKY"]:
            color = FINGER_COLORS[grp]
            row = tk.Frame(margin_inner, bg=CARD)
            row.pack(fill="x", pady=1)
            tk.Label(row, text=FINGER_NAMES_TH[grp], font=("Segoe UI", 8),
                     fg=color, bg=CARD, width=9, anchor="w").pack(side="left")
            var = tk.IntVar(value=default_margins[grp])
            self.group_margin_vars[grp] = var
            sc = tk.Scale(row, from_=0, to=50, orient="horizontal", variable=var,
                          length=110, font=("Consolas", 7), fg=color, bg=CARD,
                          highlightthickness=0, troughcolor="#1e293b", showvalue=True)
            sc.pack(side="left", padx=2)
            val_lbl = tk.Label(row, textvariable=var, font=("Consolas", 8),
                               fg=MUTED, bg=CARD, width=3)
            val_lbl.pack(side="left")

        # Reset button
        def _reset_margins():
            for v in self.group_margin_vars.values():
                v.set(self.margin_var.get())
        tk.Button(margin_inner, text="↺ Reset ทุกนิ้ว (= Global)", font=("Segoe UI", 8),
                  fg="#94a3b8", bg="#1e293b", relief="flat", cursor="hand2",
                  command=_reset_margins).pack(fill="x", pady=(4, 0))

        # Auto Demo
        demo_lf = tk.LabelFrame(parent, text="  Auto Demo  ", font=("Segoe UI", 9, "bold"),
                                 fg=TEXT, bg=CARD, bd=1, relief="groove")
        demo_lf.pack(fill="x", pady=(0, 4))
        demo_inner = tk.Frame(demo_lf, bg=CARD)
        demo_inner.pack(pady=6, padx=6, fill="x")
        self.demo_btn = tk.Button(demo_inner, text="▶️ Auto Demo", font=("Segoe UI", 11, "bold"),
                                   fg="white", bg="#16a34a", relief="flat", height=2,
                                   cursor="hand2", state="disabled", command=self.toggle_demo)
        self.demo_btn.pack(fill="x")
        self.demo_status = tk.Label(demo_inner, text="Demo: หยุด",
                                     font=("Segoe UI", 8), fg="#64748b", bg=CARD)
        self.demo_status.pack(pady=(2, 0))

    def _build_servo_panel(self, parent):
        """สร้าง panel สถานะ servo (ฝั่งขวา)"""
        CARD = "#111827"
        TEXT = "#e2e8f0"

        self.servo_frame = tk.LabelFrame(parent, text="  📊 สถานะ Real-time  ",
                                          font=("Segoe UI", 10, "bold"),
                                          fg=TEXT, bg=CARD, bd=1, relief="groove")

        th = tk.Frame(self.servo_frame, bg="#0f172a")
        th.pack(fill="x", padx=4, pady=(4, 0))
        for hdr, w in [("ID", 3), ("ชื่อ", 9), ("เป้า", 6), ("จริง", 6), ("Bar", 10)]:
            tk.Label(th, text=hdr, font=("Segoe UI", 8, "bold"),
                     fg="#94a3b8", bg="#0f172a", width=w, anchor="center").pack(side="left", padx=1)

        canvas = tk.Canvas(self.servo_frame, bg=CARD, highlightthickness=0)
        scroll = ttk.Scrollbar(self.servo_frame, orient="vertical", command=canvas.yview)
        inner = tk.Frame(canvas, bg=CARD)
        inner.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=scroll.set)
        canvas.pack(side="left", fill="both", expand=True, padx=4)
        scroll.pack(side="right", fill="y", padx=(0, 4), pady=4)

        for idx, (sid, op, cl, name, grp) in enumerate(SERVO_DATA):
            row_bg = "#0f172a" if idx % 2 == 0 else "#1e293b"
            color = FINGER_COLORS.get(grp, "#94a3b8")
            row = tk.Frame(inner, bg=row_bg)
            row.pack(fill="x", pady=1)

            tk.Label(row, text=str(sid), font=("Consolas", 9, "bold"),
                     fg=color, bg=row_bg, width=3, anchor="center").pack(side="left", padx=1)
            tk.Label(row, text=name, font=("Segoe UI", 8),
                     fg="#cbd5e1", bg=row_bg, width=9, anchor="w").pack(side="left", padx=1)
            t_lbl = tk.Label(row, text="---", font=("Consolas", 9),
                              fg="#64748b", bg=row_bg, width=6, anchor="center")
            t_lbl.pack(side="left", padx=1)
            a_lbl = tk.Label(row, text="---", font=("Consolas", 9),
                              fg="#64748b", bg=row_bg, width=6, anchor="center")
            a_lbl.pack(side="left", padx=1)
            bar_c = tk.Canvas(row, width=80, height=12, bg="#1e293b", highlightthickness=0)
            bar_c.pack(side="left", padx=4)
            bar_f = bar_c.create_rectangle(0, 0, 0, 12, fill=color, outline="")

            self.servo_widgets[sid] = {
                "target": t_lbl, "actual": a_lbl,
                "bar_canvas": bar_c, "bar_fill": bar_f,
                "open_deg": op, "close_deg": cl, "color": color
            }

    def _build_webcam_panel(self, parent):
        """สร้าง panel webcam (ฝั่งขวา — ซ่อนไว้ก่อน)"""
        CARD = "#111827"
        TEXT = "#e2e8f0"
        MUTED = "#64748b"

        self.webcam_frame = tk.LabelFrame(parent, text="  📷 Webcam Hand Tracking  ",
                                           font=("Segoe UI", 10, "bold"),
                                           fg=TEXT, bg=CARD, bd=1, relief="groove")

        # Video preview
        self.webcam_label = tk.Label(self.webcam_frame, bg="#0f172a",
                                      text="📷 กด Start เพื่อเปิดกล้อง",
                                      font=("Segoe UI", 11), fg=MUTED)
        self.webcam_label.pack(padx=6, pady=6, fill="both", expand=True)

        # Controls
        ctrl = tk.Frame(self.webcam_frame, bg=CARD)
        ctrl.pack(fill="x", padx=6, pady=(0, 6))

        tk.Label(ctrl, text="Cam:", font=("Segoe UI", 9), fg=TEXT, bg=CARD
        ).pack(side="left", padx=(0, 3))
        self.cam_var = tk.IntVar(value=0)
        tk.Spinbox(ctrl, from_=0, to=5, textvariable=self.cam_var, width=3,
                   font=("Consolas", 10), bg="#0f172a", fg="white",
                   buttonbackground="#334155").pack(side="left", padx=3)

        self.webcam_start_btn = tk.Button(ctrl, text="▶️ Start", font=("Segoe UI", 9, "bold"),
                                           fg="white", bg="#16a34a", relief="flat", padx=12,
                                           cursor="hand2", command=self.toggle_webcam)
        self.webcam_start_btn.pack(side="left", padx=10)

        # Sensitivity
        tk.Label(ctrl, text="Smooth:", font=("Segoe UI", 8), fg=MUTED, bg=CARD
        ).pack(side="right", padx=(0, 3))
        self.smooth_var = tk.IntVar(value=int(SMOOTHING_ALPHA * 100))
        tk.Scale(ctrl, from_=10, to=80, orient="horizontal", variable=self.smooth_var,
                 length=70, font=("Consolas", 7), fg=MUTED, bg=CARD,
                 highlightthickness=0, troughcolor="#1e293b", showvalue=True
        ).pack(side="right", padx=(0, 3))

        # Finger ratio display
        ratio_frame = tk.Frame(self.webcam_frame, bg=CARD)
        ratio_frame.pack(fill="x", padx=6, pady=(0, 6))

        self.ratio_bars = {}
        for finger in FINGER_GROUPS:
            color = FINGER_COLORS[finger]
            f = tk.Frame(ratio_frame, bg=CARD)
            f.pack(fill="x", pady=1)
            tk.Label(f, text=FINGER_NAMES_TH[finger], font=("Segoe UI", 8),
                     fg=color, bg=CARD, width=8, anchor="w").pack(side="left")
            bar_c = tk.Canvas(f, width=120, height=14, bg="#1e293b", highlightthickness=0)
            bar_c.pack(side="left", padx=4)
            bar_fill = bar_c.create_rectangle(0, 0, 0, 14, fill=color, outline="")
            val_lbl = tk.Label(f, text="0%", font=("Consolas", 8),
                                fg=MUTED, bg=CARD, width=4)
            val_lbl.pack(side="left")
            self.ratio_bars[finger] = {"canvas": bar_c, "fill": bar_fill, "label": val_lbl}

    def _build_voice_panel(self, parent):
        """สร้าง panel Voice Control"""
        CARD = "#111827"
        TEXT = "#e2e8f0"
        MUTED = "#64748b"

        self.voice_frame = tk.LabelFrame(parent, text="  🎤 Voice Control  ",
                                          font=("Segoe UI", 10, "bold"),
                                          fg=TEXT, bg=CARD, bd=1, relief="groove")

        # --- Mic icon + status ---
        top = tk.Frame(self.voice_frame, bg=CARD)
        top.pack(fill="x", padx=10, pady=(12, 6))
        self.voice_mic_label = tk.Label(top, text="🎤",
                                         font=("Segoe UI", 48), fg=MUTED, bg=CARD)
        self.voice_mic_label.pack()
        self.voice_status_lbl = tk.Label(top, textvariable=self.voice_status_var,
                                          font=("Segoe UI", 11, "bold"), fg=MUTED, bg=CARD)
        self.voice_status_lbl.pack(pady=(4, 0))

        # --- Recognized text ---
        rec_lf = tk.LabelFrame(self.voice_frame, text="  🔍 ได้ยิน  ",
                                font=("Segoe UI", 8, "bold"), fg=MUTED, bg=CARD, bd=1)
        rec_lf.pack(fill="x", padx=10, pady=(0, 6))
        tk.Label(rec_lf, textvariable=self.voice_text_var,
                 font=("Segoe UI", 14, "bold"), fg="#fbbf24", bg=CARD,
                 wraplength=220).pack(padx=8, pady=6)

        # --- Command result ---
        cmd_lf = tk.LabelFrame(self.voice_frame, text="  ✅ คำสั่งที่เรียก  ",
                                font=("Segoe UI", 8, "bold"), fg=MUTED, bg=CARD, bd=1)
        cmd_lf.pack(fill="x", padx=10, pady=(0, 6))
        tk.Label(cmd_lf, textvariable=self.voice_cmd_var,
                 font=("Segoe UI", 12), fg="#22c55e", bg=CARD).pack(padx=8, pady=6)

        # --- Controls ---
        ctrl = tk.Frame(self.voice_frame, bg=CARD)
        ctrl.pack(fill="x", padx=10, pady=(0, 6))

        self.voice_start_btn = tk.Button(
            ctrl, text="▶️ Start Listening", font=("Segoe UI", 10, "bold"),
            fg="white", bg="#16a34a", relief="flat", padx=14, pady=6,
            cursor="hand2", command=self.toggle_voice)
        self.voice_start_btn.pack(side="left", padx=(0, 8))

        tk.Label(ctrl, text="Lang:", font=("Segoe UI", 8), fg=MUTED, bg=CARD).pack(side="left")
        self.voice_lang_var = tk.StringVar(value="th-TH")
        lang_cb = ttk.Combobox(ctrl, textvariable=self.voice_lang_var,
                                values=["th-TH", "en-US"], width=7, state="readonly")
        lang_cb.pack(side="left", padx=4)

        # --- Available commands ---
        cmds_lf = tk.LabelFrame(self.voice_frame, text="  📋 คำสั่งที่ใช้ได้  ",
                                 font=("Segoe UI", 8, "bold"), fg=MUTED, bg=CARD, bd=1)
        cmds_lf.pack(fill="x", padx=10, pady=(0, 10))
        inner = tk.Frame(cmds_lf, bg=CARD)
        inner.pack(padx=6, pady=4, fill="x")

        cmd_list = [
            ("กำมือ", "✊"), ("แบมือ", "🖐️"),
            ("สันติภาพ", "✌️"), ("ยกนิ้วโป้ง", "👍"),
            ("ชี้", "☝️"),      ("โบกมือ", "👋"),
            ("หยิบ", "🤏"),     ("โทรหา", "🦙"),
            ("ปืน", "👈"),      ("สาม", "3️⃣"),
            ("สี่", "4️⃣"),       ("จับ", "🫴"),
        ]
        for i, (cmd, emoji) in enumerate(cmd_list):
            r, c = divmod(i, 2)
            cell = tk.Frame(inner, bg="#0f172a", padx=6, pady=3)
            cell.grid(row=r, column=c, padx=2, pady=1, sticky="ew")
            tk.Label(cell, text=f"{emoji} {cmd}",
                     font=("Segoe UI", 9), fg="#94a3b8", bg="#0f172a").pack(anchor="w")
        inner.columnconfigure(0, weight=1)
        inner.columnconfigure(1, weight=1)

    # =========================================================================
    # MODE SWITCHING
    # =========================================================================
    def switch_mode(self, mode):
        if mode == self.current_mode:
            return

        # Hide all right panels first
        self.servo_frame.pack_forget()
        self.webcam_frame.pack_forget()
        self.voice_frame.pack_forget()
        # Reset all mode buttons
        self.gui_mode_btn.config(bg="#475569")
        self.webcam_mode_btn.config(bg="#475569")
        if VOICE_AVAILABLE:
            self.voice_mode_btn.config(bg="#475569")

        if mode == "WEBCAM":
            if not WEBCAM_AVAILABLE:
                messagebox.showerror("Error", "ต้องติดตั้ง mediapipe, opencv-python, Pillow ก่อน!")
                self.servo_frame.pack(fill="both", expand=True)
                self.gui_mode_btn.config(bg="#2563eb")
                return
            self.stop_demo = True
            if self.voice_active:
                self.stop_voice()
            self.webcam_frame.pack(fill="both", expand=True)
            self.webcam_mode_btn.config(bg="#7c3aed")
            self.mode_label.config(text="Mode: 📷 Webcam")
            self.log("📷 สลับเป็น Webcam Mode")

        elif mode == "VOICE":
            if not VOICE_AVAILABLE:
                messagebox.showerror("Error", "ต้องติดตั้ง speech_recognition และ sounddevice ก่อน!")
                self.servo_frame.pack(fill="both", expand=True)
                self.gui_mode_btn.config(bg="#2563eb")
                return
            self.stop_demo = True
            if self.webcam_active:
                self.stop_webcam()
            self.voice_frame.pack(fill="both", expand=True)
            self.voice_mode_btn.config(bg="#7c3aed")
            self.mode_label.config(text="Mode: 🎤 Voice")
            self.log("🎤 สลับเป็น Voice Mode")

        elif mode == "GUI":
            if self.webcam_active:
                self.stop_webcam()
            if self.voice_active:
                self.stop_voice()
            self.servo_frame.pack(fill="both", expand=True)
            self.gui_mode_btn.config(bg="#2563eb")
            self.mode_label.config(text="Mode: 🎮 GUI")
            self.log("🎮 สลับเป็น GUI Mode")

        self.current_mode = mode

    # =========================================================================
    # WEBCAM
    # =========================================================================
    def toggle_webcam(self):
        if self.webcam_active:
            self.stop_webcam()
        else:
            self.start_webcam()

    def start_webcam(self):
        self.cam_index = self.cam_var.get()
        self.webcam_cap = cv2.VideoCapture(self.cam_index)
        if not self.webcam_cap.isOpened():
            messagebox.showerror("Error", f"เปิดกล้อง {self.cam_index} ไม่ได้!")
            return

        self.webcam_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.webcam_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.hand_tracker = HandTracker()
        self.webcam_active = True
        self.webcam_start_btn.config(text="⏹️ Stop", bg="#dc2626")
        self.log("📷 Webcam เริ่มทำงาน")

        # Start webcam loop
        self._webcam_loop()

    def stop_webcam(self):
        self.webcam_active = False
        if self.webcam_cap:
            self.webcam_cap.release()
            self.webcam_cap = None
        if self.hand_tracker:
            self.hand_tracker.release()
            self.hand_tracker = None
        self.webcam_start_btn.config(text="▶️ Start", bg="#16a34a")
        self.webcam_label.config(image="", text="📷 กด Start เพื่อเปิดกล้อง")
        self.log("📷 Webcam หยุดทำงาน")

    def _webcam_loop(self):
        if not self.webcam_active or not self.webcam_cap:
            return

        ret, frame = self.webcam_cap.read()
        if not ret:
            self.root.after(30, self._webcam_loop)
            return

        # Update smoothing alpha from slider
        global SMOOTHING_ALPHA
        SMOOTHING_ALPHA = self.smooth_var.get() / 100.0

        # Process frame
        annotated, ratios = self.hand_tracker.process_frame(frame)

        # Display in tkinter
        display = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(display)
        # Resize to fit panel
        img = img.resize((360, 270), Image.LANCZOS)
        photo = ImageTk.PhotoImage(image=img)
        self.webcam_label.config(image=photo, text="")
        self.webcam_label.image = photo

        # Send to servos if connected and hand detected
        if ratios and self.connected:
            self._send_webcam_ratios(ratios)
            # Update ratio bars
            for finger, ratio in ratios.items():
                if finger in self.ratio_bars:
                    bar = self.ratio_bars[finger]
                    fill_w = int(120 * ratio)
                    bar["canvas"].coords(bar["fill"], 0, 0, fill_w, 14)
                    bar["label"].config(text=f"{ratio:.0%}")

        # Schedule next frame
        self.root.after(int(1000 / WEBCAM_FPS), self._webcam_loop)

    def _send_webcam_ratios(self, ratios):
        """ส่ง finger ratios ไป servo ด้วย SyncWrite"""
        if not self.connected or not self.sync_write:
            return

        positions = {}
        for sid, op, cl, name, grp in SERVO_DATA:
            if grp == "WRIST":
                continue  # ไม่ควบคุมข้อมือด้วย webcam
            # ID 12 (โป้ง Pitch) ใช้ THUMB_PITCH ratio แยกจาก thumb curl
            if sid == 12:
                ratio = ratios.get("THUMB_PITCH", 0)
            else:
                ratio = ratios.get(grp, 0)
            # ใช้ per-group margin ถ้ามี ไม่งั้นใช้ global
            if grp in self.group_margin_vars:
                margin = self.group_margin_vars[grp].get() / 100.0
            else:
                margin = self.margin_var.get() / 100.0
            positions[sid] = calc_target_deg(op, cl, ratio, margin)

        self.sync_write_all(positions)

    # =========================================================================
    # CONNECTION
    # =========================================================================
    def toggle_connection(self):
        if not self.connected:
            self.do_connect()
        else:
            self.do_disconnect()

    def do_connect(self):
        dev = self.port_var.get().strip()
        baud = int(self.baud_var.get().strip())
        self.port = PortHandler(dev)
        self.pkt = PacketHandler(PROTOCOL_VERSION)

        if not self.port.openPort():
            messagebox.showerror("Error", f"เปิด {dev} ไม่ได้!")
            return
        if not self.port.setBaudRate(baud):
            messagebox.showerror("Error", f"ตั้ง baudrate ไม่ได้!")
            self.port.closePort()
            return

        ok = 0
        for sid, _, _, _, _ in SERVO_DATA:
            res, err = self.pkt.write1ByteTxRx(self.port, sid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if res == COMM_SUCCESS and err == 0:
                ok += 1
            time.sleep(0.03)

        for sid, _, _, _, _ in SERVO_DATA:
            self.pkt.write4ByteTxRx(self.port, sid, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY)
            self.pkt.write4ByteTxRx(self.port, sid, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION)
            time.sleep(0.02)

        self.sync_write = GroupSyncWrite(self.port, self.pkt, ADDR_GOAL_POSITION, 4)

        self.connected = True
        self.status_dot.config(text=f"● Connected ({dev})", fg="#22c55e")
        self.connect_btn.config(text="🔌 Disconnect", bg="#dc2626")
        self.set_buttons_state("normal")
        self.log(f"✅ เชื่อมต่อ {dev} @ {baud} — Torque ON {ok}/{len(SERVO_DATA)}")

        self.monitoring = True
        self.monitor_loop()

    def do_disconnect(self):
        self.monitoring = False
        self.stop_demo = True
        if self.webcam_active:
            self.stop_webcam()
        if self.port:
            for sid, _, _, _, _ in SERVO_DATA:
                self.pkt.write1ByteTxRx(self.port, sid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
                time.sleep(0.02)
            self.port.closePort()

        self.connected = False
        self.status_dot.config(text="● Disconnected", fg="#ef4444")
        self.connect_btn.config(text="🔌 Connect", bg="#059669")
        self.set_buttons_state("disabled")
        self.log("ปิดการเชื่อมต่อ")

    # =========================================================================
    # MONITORING
    # =========================================================================
    def monitor_loop(self):
        if not self.monitoring or not self.connected:
            return
        if not self.busy and self.current_mode == "GUI":
            threading.Thread(target=self._read_positions_bg, daemon=True).start()
        self.root.after(MONITOR_INTERVAL, self.monitor_loop)

    def _read_positions_bg(self):
        if not self.connected or not self.port:
            return
        for sid, _, _, _, _ in SERVO_DATA:
            try:
                raw_pos, res, err = self.pkt.read4ByteTxRx(self.port, sid, ADDR_PRESENT_POSITION)
                if res == COMM_SUCCESS and err == 0:
                    deg = raw_to_deg(raw_pos)
                    self.current_positions[sid] = deg
                    self.root.after(0, lambda s=sid, d=deg: self._update_actual(s, d))
            except:
                pass
            time.sleep(0.02)

    def _update_actual(self, sid, deg):
        if sid not in self.servo_widgets:
            return
        w = self.servo_widgets[sid]
        w["actual"].config(text=f"{deg:.1f}°", fg="#22c55e")
        op, cl = w["open_deg"], w["close_deg"]
        travel = abs(cl - op)
        if travel > 0:
            ratio = (deg - op) / (cl - op) if cl > op else (op - deg) / (op - cl)
            ratio = max(0, min(1, ratio))
        else:
            ratio = 0
        w["bar_canvas"].coords(w["bar_fill"], 0, 0, int(80 * ratio), 12)

    # =========================================================================
    # SERVO COMMANDS
    # =========================================================================
    def send_pos_deg(self, servo_id, degrees):
        if not self.connected:
            return
        raw = deg_to_raw(degrees)
        res, err = self.pkt.write4ByteTxRx(self.port, servo_id, ADDR_GOAL_POSITION, raw)
        self.current_positions[servo_id] = degrees
        if servo_id in self.servo_widgets:
            c = "#22c55e" if (res == COMM_SUCCESS and err == 0) else "#ef4444"
            self.root.after(0, lambda s=servo_id, d=degrees, cl=c:
                self.servo_widgets[s]["target"].config(text=f"{d:.1f}°", fg=cl))
        time.sleep(0.02)

    def sync_write_all(self, positions_dict):
        if not self.connected or not self.sync_write:
            return
        self.sync_write.clearParam()
        for sid, deg in positions_dict.items():
            raw = deg_to_raw(deg)
            param = [DXL_LOBYTE(DXL_LOWORD(raw)), DXL_HIBYTE(DXL_LOWORD(raw)),
                     DXL_LOBYTE(DXL_HIWORD(raw)), DXL_HIBYTE(DXL_HIWORD(raw))]
            self.sync_write.addParam(sid, param)
            self.current_positions[sid] = deg
        self.sync_write.txPacket()
        for sid, deg in positions_dict.items():
            if sid in self.servo_widgets:
                self.root.after(0, lambda s=sid, d=deg:
                    self.servo_widgets[s]["target"].config(text=f"{d:.1f}°", fg="white"))

    def smooth_move_to_pose(self, pose_data, duration=1.0):
        targets = {}
        for sid, op, cl, name, grp in SERVO_DATA:
            ratio = pose_data.get(grp, 0)
            # ใช้ per-group margin ถ้ามี ไม่งั้นใช้ global
            if grp in self.group_margin_vars:
                margin = self.group_margin_vars[grp].get() / 100.0
            else:
                margin = self.margin_var.get() / 100.0
            targets[sid] = calc_target_deg(op, cl, ratio, margin)
        self.sync_write_all(targets)
        time.sleep(duration)

    def safe_close_pose(self, pose_data):
        thumb_ratio = pose_data.get("THUMB", 0)
        other_closing = any(pose_data.get(g, 0) > 0.3 for g in ["INDEX", "MIDDLE", "RING", "PINKY"])
        if thumb_ratio > 0.3 and other_closing:
            pose_no_thumb = dict(pose_data)
            pose_no_thumb["THUMB"] = 0
            self.log("  ↪ นิ้วโป้ง หลบ...")
            self.smooth_move_to_pose(pose_no_thumb)
            time.sleep(0.3)
            self.log("  ↪ นิ้วโป้ง ทับ...")
            self.smooth_move_to_pose(pose_data)
        else:
            self.smooth_move_to_pose(pose_data)

    # =========================================================================
    # POSE ACTIONS
    # =========================================================================
    def do_open_hand(self):
        self.log("🖐️ แบมือ...")
        self.smooth_move_to_pose(POSES["OPEN"]["data"])
        self.log("✅ แบมือเรียบร้อย")

    def do_close_hand(self):
        self.log("✊ กำมือ...")
        self.safe_close_pose(POSES["FIST"]["data"])
        self.log("✅ กำมือเรียบร้อย")

    def do_pose(self, pose_key):
        p = POSES[pose_key]
        self.log(f"{p['emoji']} ท่า {p['name_th']}...")
        self.safe_close_pose(p["data"])
        self.log(f"✅ ท่า {p['name_th']} เรียบร้อย")

    def do_wave(self):
        self.log("👋 โบกมือ...")
        for cycle in range(3):
            if self.stop_demo: break
            for frame in WAVE_SEQUENCE:
                if self.stop_demo: break
                self.smooth_move_to_pose(frame, duration=0.25)
        self.smooth_move_to_pose(POSES["OPEN"]["data"])
        self.log("✅ โบกมือเสร็จ")

    def do_count(self):
        self.log("🔢 นับ 1-5...")
        counts = [
            {"INDEX": 0, "MIDDLE": 1, "RING": 1, "PINKY": 1, "THUMB": 1, "WRIST": 0},
            {"INDEX": 0, "MIDDLE": 0, "RING": 1, "PINKY": 1, "THUMB": 1, "WRIST": 0},
            {"INDEX": 0, "MIDDLE": 0, "RING": 0, "PINKY": 1, "THUMB": 1, "WRIST": 0},
            {"INDEX": 0, "MIDDLE": 0, "RING": 0, "PINKY": 0, "THUMB": 1, "WRIST": 0},
            {"INDEX": 0, "MIDDLE": 0, "RING": 0, "PINKY": 0, "THUMB": 0, "WRIST": 0},
        ]
        self.safe_close_pose(POSES["FIST"]["data"])
        time.sleep(0.5)
        for i, pose in enumerate(counts):
            if self.stop_demo: break
            self.log(f"  {i+1}️⃣")
            self.smooth_move_to_pose(pose)
            time.sleep(1.0)
        self.log("✅ นับเสร็จ!")

    def do_flex(self):
        self.log("💪 Flex...")
        for i in range(5):
            if self.stop_demo: break
            self.safe_close_pose(POSES["FIST"]["data"])
            time.sleep(0.3)
            self.smooth_move_to_pose(POSES["OPEN"]["data"])
            time.sleep(0.3)
        self.log("✅ Flex เสร็จ!")

    # =========================================================================
    # AUTO DEMO
    # =========================================================================
    def toggle_demo(self):
        if self.demo_running:
            self.stop_demo = True
            self.demo_btn.config(text="▶️ Auto Demo", bg="#16a34a")
            self.demo_status.config(text="Demo: หยุด...", fg="#eab308")
        else:
            self.stop_demo = False
            self.demo_running = True
            self.demo_btn.config(text="⏹️ หยุด", bg="#dc2626")
            self.demo_status.config(text="Demo: กำลังเล่น", fg="#22c55e")
            self.run_async(self._demo_loop, allow_during_busy=True)

    def _demo_loop(self):
        cycle = 0
        while not self.stop_demo:
            cycle += 1
            self.root.after(0, lambda c=cycle: self.demo_status.config(
                text=f"Demo: รอบ {c}", fg="#22c55e"))
            for key, hold in DEMO_SEQUENCE:
                if self.stop_demo: break
                p = POSES[key]
                self.log(f"🎭 {p['emoji']} {p['name_th']}")
                self.safe_close_pose(p["data"])
                t_end = time.time() + hold
                while time.time() < t_end and not self.stop_demo:
                    time.sleep(0.1)
            if not self.stop_demo:
                self.do_wave()
                time.sleep(1)

        self.demo_running = False
        self.root.after(0, lambda: self.demo_btn.config(text="▶️ Auto Demo", bg="#16a34a"))
        self.root.after(0, lambda: self.demo_status.config(text="Demo: หยุดแล้ว", fg="#64748b"))
        self.log("⏹️ Demo หยุด")

    # =========================================================================
    # UI HELPERS
    # =========================================================================
    def set_buttons_state(self, state):
        for btn in [self.open_btn, self.close_btn, self.wave_btn,
                    self.count_btn, self.flex_btn, self.demo_btn]:
            btn.config(state=state)
        for btn in self.pose_btns.values():
            btn.config(state=state)

    def log(self, msg):
        self.root.after(0, lambda: self.log_var.set(msg))

    def run_async(self, func, allow_during_busy=False):
        if self.busy and not allow_during_busy:
            self.log("⏳ กำลังทำงาน...")
            return
        if not self.connected:
            self.log("❌ ยังไม่ได้เชื่อมต่อ!")
            return

        def wrapper():
            self.busy = True
            self.root.after(0, lambda: self.set_buttons_state("disabled"))
            if self.demo_running:
                self.root.after(0, lambda: self.demo_btn.config(state="normal"))
            try:
                func()
            except Exception as e:
                self.log(f"❌ Error: {e}")
            finally:
                self.busy = False
                if self.connected:
                    self.root.after(0, lambda: self.set_buttons_state("normal"))

        threading.Thread(target=wrapper, daemon=True).start()

    # =========================================================================
    # VOICE
    # =========================================================================
    def toggle_voice(self):
        if self.voice_active:
            self.stop_voice()
        else:
            self.start_voice()

    def start_voice(self):
        self.voice_active = True
        self.voice_start_btn.config(text="⏹️ Stop", bg="#dc2626")
        self.voice_text_var.set("—")
        self.voice_cmd_var.set("—")
        self._update_voice_status("🔧 กำลังเตรียม...", "#fbbf24")
        self.voice_thread = threading.Thread(target=self._voice_loop, daemon=True)
        self.voice_thread.start()
        self.log("🎤 Voice: เริ่มฟัง...")

    def stop_voice(self):
        self.voice_active = False
        self.root.after(0, lambda: self.voice_start_btn.config(text="▶️ Start Listening", bg="#16a34a"))
        self._update_voice_status("⏹️ หยุด", "#64748b")
        self.root.after(0, lambda: self.voice_mic_label.config(fg="#64748b"))
        self.log("🎤 Voice: หยุด")

    def _update_voice_status(self, text, color="#64748b"):
        self.voice_status_var.set(text)
        self.root.after(0, lambda t=text, c=color: (
            self.voice_status_var.set(t),
            self.voice_status_lbl.config(fg=c)
        ))

    def _voice_loop(self):
        """Background: VAD-based continuous speech recognition via sounddevice"""
        import sounddevice as _sd
        import wave as _wv
        import io as _io

        SAMPLE_RATE    = 16000
        CHUNK_SAMPLES  = int(SAMPLE_RATE * 0.1)   # 100 ms per chunk
        SILENCE_THR    = 400     # RMS — ปรับตามห้อง
        MIN_SPEECH_CHK = 4       # ต้องมีเสียงอย่างน้อย 400ms
        MAX_SPEECH_CHK = 60      # สูงสุด 6 วินาที
        SILENCE_END    = 12      # 1.2 วินาทีเงียบ = จบ

        buf, sil_cnt, speaking = [], 0, False

        def callback(indata, frames, ti, status):
            nonlocal buf, sil_cnt, speaking
            chunk = indata[:, 0].copy()
            rms = float(np.sqrt(np.mean(chunk.astype(np.float32) ** 2)))

            if rms > SILENCE_THR:
                if not speaking:
                    speaking = True
                    buf, sil_cnt = [], 0
                    self.root.after(0, lambda: self._update_voice_status("🗣️ กำลังฟัง...", "#22c55e"))
                    self.root.after(0, lambda: self.voice_mic_label.config(fg="#22c55e"))
                buf.append(chunk)
                sil_cnt = 0
            elif speaking:
                buf.append(chunk)
                sil_cnt += 1
                if sil_cnt >= SILENCE_END or len(buf) >= MAX_SPEECH_CHK:
                    if len(buf) >= MIN_SPEECH_CHK:
                        audio_np = np.concatenate(buf).astype(np.int16)
                        threading.Thread(
                            target=self._recognize_audio,
                            args=(audio_np, SAMPLE_RATE),
                            daemon=True
                        ).start()
                    buf, sil_cnt, speaking = [], 0, False
                    self.root.after(0, lambda: self._update_voice_status("👂 กำลังฟัง...", "#3b82f6"))
                    self.root.after(0, lambda: self.voice_mic_label.config(fg="#3b82f6"))

        try:
            self.root.after(0, lambda: self._update_voice_status("👂 กำลังฟัง...", "#3b82f6"))
            self.root.after(0, lambda: self.voice_mic_label.config(fg="#3b82f6"))
            with _sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype='int16',
                                 blocksize=CHUNK_SAMPLES, callback=callback):
                while self.voice_active:
                    time.sleep(0.05)
        except Exception as e:
            self.root.after(0, lambda err=str(e): self._update_voice_status(f"❌ {err[:40]}", "#ef4444"))
        finally:
            self.voice_active = False
            self.root.after(0, lambda: self._update_voice_status("⏹️ หยุด", "#64748b"))
            self.root.after(0, lambda: self.voice_mic_label.config(fg="#64748b"))
            self.root.after(0, lambda: self.voice_start_btn.config(text="▶️ Start Listening", bg="#16a34a"))

    def _recognize_audio(self, audio_np, samplerate):
        """STT via Google + match command"""
        import speech_recognition as _sr
        import wave as _wv
        import io as _io

        self.root.after(0, lambda: self._update_voice_status("🔄 กำลังแปลง...", "#fbbf24"))
        try:
            buf = _io.BytesIO()
            with _wv.open(buf, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(samplerate)
                wf.writeframes(audio_np.tobytes())
            buf.seek(0)

            r = _sr.Recognizer()
            with _sr.AudioFile(buf) as source:
                audio_data = r.record(source)

            lang = self.voice_lang_var.get()
            text = r.recognize_google(audio_data, language=lang).lower().strip()
            self.root.after(0, lambda t=text: self.voice_text_var.set(f'"{t}"'))

            matched = self._match_voice_command(text)
            if matched:
                self.root.after(0, lambda m=matched: self._execute_voice_command(m))
            else:
                self.root.after(0, lambda: self.voice_cmd_var.set("❓ ไม่รู้จักคำสั่ง"))

        except Exception as e:
            err = str(e)
            if "UnknownValue" in type(e).__name__ or "recognize" in err.lower():
                self.root.after(0, lambda: self.voice_cmd_var.set("❓ ไม่ได้ยินชัด"))
            elif "Connection" in err or "request" in err.lower():
                self.root.after(0, lambda: self.voice_cmd_var.set("⚠️ ไม่มีอินเทอร์เน็ต"))
            else:
                self.root.after(0, lambda e=err: self.voice_cmd_var.set(f"❌ {e[:35]}"))
        finally:
            if self.voice_active:
                self.root.after(0, lambda: self._update_voice_status("👂 กำลังฟัง...", "#3b82f6"))

    def _match_voice_command(self, text):
        """Match recognized text → command key"""
        for keyword, cmd_key in VOICE_COMMANDS.items():
            if keyword in text:
                return cmd_key
        return None

    def _execute_voice_command(self, cmd_key):
        """Execute the matched voice command"""
        if not self.connected:
            self.root.after(0, lambda: self.voice_cmd_var.set("❌ ยังไม่ได้เชื่อมต่อ!"))
            return
        if cmd_key == "WAVE_ANIM":
            self.root.after(0, lambda: self.voice_cmd_var.set("✅ 👋 โบกมือ"))
            self.run_async(self.do_wave)
        elif cmd_key == "OPEN":
            p = POSES["OPEN"]
            self.root.after(0, lambda: self.voice_cmd_var.set(f"✅ {p['emoji']} {p['name_th']}"))
            self.run_async(self.do_open_hand)
        elif cmd_key in POSES:
            p = POSES[cmd_key]
            self.root.after(0, lambda: self.voice_cmd_var.set(f"✅ {p['emoji']} {p['name_th']}"))
            self.run_async(lambda k=cmd_key: self.do_pose(k))

    def on_close(self):
        self.stop_demo = True
        self.monitoring = False
        if self.voice_active:
            self.stop_voice()
        if self.webcam_active:
            self.stop_webcam()
        if self.connected:
            self.do_disconnect()
        self.root.destroy()


# ============================================================================
# --- MAIN ---
# ============================================================================
if __name__ == "__main__":
    root = tk.Tk()
    app = RobotHandApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()