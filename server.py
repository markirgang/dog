#!/usr/bin/env python3
"""Dog Robot 17-DOF Quadruped Control Server.

A standalone control hub for a 17-servo dog robot. It provides the same
feature set as the Shobots companion robot application (3D workbench, servo
calibration, camera, Gemini multimodal AI, TTS, MP3 playback, ultrasonic
telemetry, L298 accessories and Python export) with a 17-DOF quadruped model.

Local port: 2190
Public host: https://dog.shobots.party
"""
import http.server
import socketserver
import json
import os
import sys
import socket
import ssl
import base64
import threading
import time
import random
import urllib.parse
import re
import local_audio
import ai_engine
from gemini_bridge import GeminiDogBridge

try:
    import cv2
    import numpy as np
except Exception:  # pragma: no cover - camera is optional on non-Pi hosts
    cv2 = None
    np = None

PORT = int(os.environ.get("PORT", 2190))
HOST = os.environ.get("HOST", os.environ.get("IP", "0.0.0.0"))
DIRECTORY = os.environ.get("DIRECTORY", os.path.join(os.path.dirname(os.path.abspath(__file__)), "static"))
AUDIO_DIR = os.path.join(DIRECTORY, "audio")
os.makedirs(AUDIO_DIR, exist_ok=True)

DOF = 17

# ---------------------------------------------------------------------------
# 17 Servo map
#   Board 0x40: head (2), tail (1), ears (2), RF leg (3) = 9 channels
#   Board 0x41: LF + rear legs (8)                  = 8 channels
# ---------------------------------------------------------------------------
SERVOS = [
    {"id": 0,  "name": "head_pan",      "label": "Head Pan",       "group": "Head",            "board": "0x40", "channel": 0, "min": -80, "max": 80,  "home": 0},
    {"id": 1,  "name": "head_tilt",     "label": "Head Tilt",      "group": "Head",            "board": "0x40", "channel": 1, "min": -40, "max": 40,  "home": 0},
    {"id": 2,  "name": "tail_wag",      "label": "Tail Wag",       "group": "Tail",            "board": "0x40", "channel": 2, "min": -70, "max": 70,  "home": 0},
    {"id": 3,  "name": "l_ear",         "label": "Left Ear",       "group": "Ears",            "board": "0x40", "channel": 3, "min": -40, "max": 40,  "home": 0},
    {"id": 4,  "name": "r_ear",         "label": "Right Ear",      "group": "Ears",            "board": "0x40", "channel": 4, "min": -40, "max": 40,  "home": 0},
    {"id": 5,  "name": "lf_hip_yaw",    "label": "LF Hip Yaw",     "group": "Left Front Leg",  "board": "0x40", "channel": 5, "min": -45, "max": 45,  "home": 0},
    {"id": 6,  "name": "lf_hip_pitch",  "label": "LF Hip Pitch",   "group": "Left Front Leg",  "board": "0x40", "channel": 6, "min": -90, "max": 90,  "home": 0},
    {"id": 7,  "name": "lf_knee",       "label": "LF Knee",        "group": "Left Front Leg",  "board": "0x40", "channel": 7, "min": -120, "max": 5, "home": 5},
    {"id": 8,  "name": "rf_hip_yaw",    "label": "RF Hip Yaw",     "group": "Right Front Leg", "board": "0x40", "channel": 8, "min": -45, "max": 45,  "home": 0},
    {"id": 9,  "name": "rf_hip_pitch",  "label": "RF Hip Pitch",   "group": "Right Front Leg", "board": "0x41", "channel": 0, "min": -90, "max": 90,  "home": 0},
    {"id": 10, "name": "rf_knee",       "label": "RF Knee",        "group": "Right Front Leg", "board": "0x41", "channel": 1, "min": -120, "max": 5, "home": 5},
    {"id": 11, "name": "lr_hip_yaw",    "label": "LR Hip Yaw",     "group": "Left Rear Leg",   "board": "0x41", "channel": 2, "min": -45, "max": 45,  "home": 0},
    {"id": 12, "name": "lr_hip_pitch",  "label": "LR Hip Pitch",   "group": "Left Rear Leg",   "board": "0x41", "channel": 3, "min": -90, "max": 90,  "home": 0},
    {"id": 13, "name": "lr_knee",       "label": "LR Knee",        "group": "Left Rear Leg",   "board": "0x41", "channel": 4, "min": -120, "max": 5, "home": 5},
    {"id": 14, "name": "rr_hip_yaw",    "label": "RR Hip Yaw",     "group": "Right Rear Leg",  "board": "0x41", "channel": 5, "min": -45, "max": 45,  "home": 0},
    {"id": 15, "name": "rr_hip_pitch",  "label": "RR Hip Pitch",   "group": "Right Rear Leg",  "board": "0x41", "channel": 6, "min": -90, "max": 90,  "home": 0},
    {"id": 16, "name": "rr_knee",       "label": "RR Knee",        "group": "Right Rear Leg",  "board": "0x41", "channel": 7, "min": -120, "max": 5, "home": 5},
]
SERVO_BY_NAME = {s["name"]: s for s in SERVOS}

NEUTRAL = {s["name"]: s["home"] for s in SERVOS}
servo_state = dict(NEUTRAL)
calibration_trims = {s["name"]: 0 for s in SERVOS}
body_state = {"height": 0, "roll": 0, "pitch": 0, "yaw": 0, "stance": 0}
state_lock = threading.Lock()


def _pose(joints=None, height=0, roll=0, pitch=0, yaw=0, description=""):
    pose = dict(NEUTRAL)
    if joints:
        pose.update(joints)
    return {
        "body": {"height": height, "roll": roll, "pitch": pitch, "yaw": yaw},
        "joints": pose,
        "description": description,
    }


SAVED_POSES = {
    "Stand": _pose(description="Neutral standing pose with all four paws planted"),
    "Sit": _pose({
        "lr_hip_pitch": -55, "rr_hip_pitch": -55, "lr_knee": 95, "rr_knee": 95,
        "lf_hip_pitch": 8, "rf_hip_pitch": 8, "lf_knee": 0, "rf_knee": 0, "tail_wag": 10,
    }, height=-55, pitch=22, description="Sitting on the rear legs with front paws up"),
    "Lie Down / Sleep": _pose({
        "lf_hip_pitch": -35, "rf_hip_pitch": -35, "lr_hip_pitch": -35, "rr_hip_pitch": -35,
        "lf_knee": 100, "rf_knee": 100, "lr_knee": 100, "rr_knee": 100,
        "head_tilt": 15, "tail_wag": -15,
    }, height=-150, description="Folded low-power resting pose"),
    "Wag Tail": _pose({"tail_wag": 30, "head_tilt": 5}, description="Happy tail wag"),
    "Wave Right Paw": _pose({
        "rf_hip_pitch": -75, "rf_knee": 65, "rf_hip_yaw": 18,
        "head_pan": 10, "head_tilt": 6, "tail_wag": 20,
    }, roll=-4, description="Raises the right front paw and waves"),
    "Wave Left Paw": _pose({
        "lf_hip_pitch": -75, "lf_knee": 65, "lf_hip_yaw": -18,
        "head_pan": -10, "head_tilt": 6, "tail_wag": -20,
    }, roll=4, description="Raises the left front paw and waves"),
    "Shake Hands": _pose({
        "rf_hip_pitch": -70, "rf_knee": 80, "rf_hip_yaw": 10,
        "head_tilt": -6, "tail_wag": 15,
    }, roll=-5, pitch=-4, description="Offers the right paw for a handshake"),
    "Beg": _pose({
        "lr_hip_pitch": -60, "rr_hip_pitch": -60, "lr_knee": 95, "rr_knee": 95,
        "lf_hip_pitch": -80, "rf_hip_pitch": -80, "lf_knee": 70, "rf_knee": 70,
        "head_tilt": -10, "tail_wag": 10,
    }, height=-40, pitch=-28, description="Begs upright on the rear legs"),
    "Bow / Play": _pose({
        "lf_hip_pitch": 22, "rf_hip_pitch": 22, "lf_knee": 0, "rf_knee": 0,
        "lr_hip_pitch": -28, "rr_hip_pitch": -28, "lr_knee": 38, "rr_knee": 38,
        "head_tilt": -18, "tail_wag": 25,
    }, height=-30, pitch=22, description="Play bow with front down and rear up"),
    "Dance": _pose({
        "lf_hip_pitch": -8, "rf_hip_pitch": 8, "lr_hip_pitch": 8, "rr_hip_pitch": -8,
        "lf_knee": 12, "rf_knee": 12, "lr_knee": 12, "rr_knee": 12,
        "tail_wag": 30, "head_tilt": 8,
    }, height=8, roll=8, yaw=12, description="Happy dancing groove"),
    "Walk in Place": _pose(description="Alternating four-leg marching gait"),
    "Run in Place": _pose(description="Fast trot in place"),
    "Attack / Guard": _pose({
        "lf_hip_pitch": -18, "rf_hip_pitch": -18, "lr_hip_pitch": -22, "rr_hip_pitch": -22,
        "lf_knee": 35, "rf_knee": 35, "lr_knee": 40, "rr_knee": 40,
        "head_tilt": -15, "tail_wag": -10,
    }, height=-25, pitch=10, description="Crouched guard posture"),
    "Curious Tilt": _pose({
        "head_pan": 15, "head_tilt": 22, "l_ear": -18, "r_ear": 10, "tail_wag": 8,
    }, roll=6, description="Curious head tilt with ear flick"),
    "Greeting": _pose({"head_tilt": 8, "tail_wag": 30, "l_ear": -8, "r_ear": 8}, description="Friendly greeting with tail wag"),
    "Stretch": _pose({
        "lf_hip_pitch": 20, "rf_hip_pitch": 20, "lf_knee": 0, "rf_knee": 0,
        "lr_hip_pitch": -30, "rr_hip_pitch": -30, "lr_knee": 45, "rr_knee": 45,
        "head_tilt": -12,
    }, height=-25, pitch=20, description="Morning stretch"),
    "Low Stance": _pose({
        "lf_hip_pitch": -25, "rf_hip_pitch": -25, "lr_hip_pitch": -25, "rr_hip_pitch": -25,
        "lf_knee": 50, "rf_knee": 50, "lr_knee": 50, "rr_knee": 50,
    }, height=-45, description="Crouched low stance"),
    "High Stance": _pose({
        "lf_hip_pitch": 8, "rf_hip_pitch": 8, "lr_hip_pitch": 8, "rr_hip_pitch": 8,
        "lf_knee": 0, "rf_knee": 0, "lr_knee": 0, "rr_knee": 0,
    }, height=25, description="Tall alert stance"),
    "Roll Over": _pose({"tail_wag": 20, "l_ear": -15, "r_ear": 15}, description="Roll-over trick"),
    "Shake Off": _pose({"l_ear": -20, "r_ear": 20, "tail_wag": 15}, description="Shake-off wiggle"),
}


def generate_synthetic_camera_frame():
    if cv2 is None or np is None:
        return None
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[:, :] = (30, 25, 20)
    cv2.putText(frame, "DOG ROBOT 17-DOF CAMERA FEED", (95, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 240, 255), 2)
    cv2.putText(frame, "STATUS: Virtual Camera Active", (160, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (160, 255, 160), 1)
    cv2.putText(frame, "LIVE TIME: " + time.strftime("%H:%M:%S"), (220, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    return frame


class CameraStream:
    """Thread-safe USB / CSI camera stream with synthetic fallback."""

    def __init__(self, source="usb"):
        self.source = str(source).lower()
        self.cap = None
        self.picam2 = None
        self.frame = None
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def set_source(self, source):
        with self.lock:
            self.source = str(source).lower()
            self._close_cameras()

    def _close_cameras(self):
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None
        if self.picam2:
            try:
                self.picam2.stop()
                self.picam2.close()
            except Exception:
                pass
            self.picam2 = None

    def _update(self):
        while self.running:
            if cv2 is None:
                time.sleep(1.0)
                continue
            with self.lock:
                cur_source = self.source

            if cur_source in ["csi", "picamera", "raspberry_pi", "ov5647"]:
                if self.picam2 is None:
                    self._close_cameras()
                    try:
                        from picamera2 import Picamera2
                        p = Picamera2()
                        p.configure(p.create_preview_configuration(main={"size": (640, 480)}))
                        p.start()
                        time.sleep(0.3)
                        self.picam2 = p
                        print("Picamera2 CSI Camera initialized successfully")
                    except Exception as e:
                        print("Picamera2 CSI Camera init error:", e)
                        self.picam2 = None
                        time.sleep(1.0)
                        continue
                try:
                    frame_array = self.picam2.capture_array("main")
                    if frame_array is not None and frame_array.size > 0:
                        if frame_array.shape[2] == 4:
                            frame_array = cv2.cvtColor(frame_array, cv2.COLOR_RGBA2BGR)
                        with self.lock:
                            self.frame = frame_array
                    time.sleep(0.04)
                except Exception as e:
                    print("CSI frame capture error:", e)
                    self._close_cameras()
                    time.sleep(0.5)
            else:
                if self.cap is None or not self.cap.isOpened():
                    self._close_cameras()
                    candidates = [0, 1, 6, 2, 8]
                    if cur_source.isdigit():
                        candidates.insert(0, int(cur_source))
                    candidates = list(dict.fromkeys(candidates))
                    opened = False
                    for idx in candidates:
                        try:
                            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
                            if cap.isOpened():
                                ret, frame = cap.read()
                                if ret and frame is not None and frame.size > 0:
                                    self.cap = cap
                                    with self.lock:
                                        self.frame = frame
                                    opened = True
                                    break
                                cap.release()
                        except Exception:
                            pass
                    if not opened:
                        time.sleep(4.0)
                if self.cap and self.cap.isOpened():
                    try:
                        ret, frame = self.cap.read()
                        if ret and frame is not None and frame.size > 0:
                            with self.lock:
                                self.frame = frame
                            time.sleep(0.04)
                        else:
                            self._close_cameras()
                            time.sleep(1.0)
                    except Exception:
                        self._close_cameras()
                        time.sleep(1.0)
                else:
                    time.sleep(0.5)

    def get_frame(self):
        with self.lock:
            if self.frame is not None:
                return self.frame.copy()
        return generate_synthetic_camera_frame()


camera_stream = CameraStream("usb")
gemini_bridge = GeminiDogBridge()

motor_state = {"mouth_channel1": False, "tail_channel2": False}
sensor_state = {"front": 85, "rear": 140, "left": 70, "right": 75}


def update_simulated_sensors():
    while True:
        for key in sensor_state:
            sensor_state[key] = max(10, min(220, sensor_state[key] + random.randint(-2, 2)))
        time.sleep(0.5)


threading.Thread(target=update_simulated_sensors, daemon=True).start()


# ---------------------------------------------------------------------------
# Optional real hardware bridge (PCA9685 + L298N + HC-SR04)
# ---------------------------------------------------------------------------
class HardwareBridge:
    def __init__(self):
        self.enabled = False
        self.pca = {}
        self.servo = None
        self.gpio = None
        self.pins = {}
        self.pwm = {}
        self._init()

    def _init(self):
        try:
            import board
            import busio
            from adafruit_pca9685 import PCA9685
            from adafruit_motor import servo as servo_lib
            i2c = busio.I2C(board.SCL, board.SDA)
            self.pca["0x40"] = PCA9685(i2c, address=0x40)
            self.pca["0x41"] = PCA9685(i2c, address=0x41)
            for p in self.pca.values():
                p.frequency = 50
            self.servo = servo_lib
            self.enabled = True
            print("PCA9685 servo drivers initialized (0x40 / 0x41)")
        except Exception as e:
            print("PCA9685 unavailable, running in simulation:", e)
            return
        try:
            import RPi.GPIO as GPIO
            self.gpio = GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            self.pins = {
                "ENA": 18, "IN1": 23, "IN2": 24,
                "ENB": 19, "IN3": 25, "IN4": 8,
            }
            GPIO.setup(list(self.pins.values()), GPIO.OUT)
            self.pwm["mouth"] = GPIO.PWM(self.pins["ENA"], 1000)
            self.pwm["tail"] = GPIO.PWM(self.pins["ENB"], 1000)
            self.pwm["mouth"].start(0)
            self.pwm["tail"].start(0)
            print("L298N motor driver initialized")
        except Exception as e:
            print("L298N unavailable, running in simulation:", e)

    def write_servo(self, cfg, logical_angle):
        if not self.enabled:
            return
        try:
            board = self.pca[cfg["board"]]
            min_pulse, max_pulse = 500, 2500
            servo_angle = max(0.0, min(180.0, 90.0 + float(logical_angle)))
            s = self.servo.Servo(board.channels[cfg["channel"]], min_pulse=min_pulse, max_pulse=max_pulse)
            s.angle = servo_angle
        except Exception:
            pass

    def set_motors(self, mouth, tail):
        if self.gpio is None:
            return
        try:
            if mouth:
                self.gpio.output(self.pins["IN1"], self.gpio.HIGH)
                self.gpio.output(self.pins["IN2"], self.gpio.LOW)
                self.pwm["mouth"].ChangeDutyCycle(100)
            else:
                self.pwm["mouth"].ChangeDutyCycle(0)
            if tail:
                self.gpio.output(self.pins["IN3"], self.gpio.HIGH)
                self.gpio.output(self.pins["IN4"], self.gpio.LOW)
                self.pwm["tail"].ChangeDutyCycle(80)
            else:
                self.pwm["tail"].ChangeDutyCycle(0)
        except Exception:
            pass


hardware = HardwareBridge()


def apply_servo_state():
    with state_lock:
        snapshot = dict(servo_state)
    for cfg in SERVOS:
        hardware.write_servo(cfg, snapshot.get(cfg["name"], cfg["home"]))


class DogRequestHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

    def end_headers(self):
        if not self.path.startswith("/api/"):
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
        super().end_headers()

    def do_HEAD(self):
        if self.path.startswith("/api/"):
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.end_headers()
        else:
            super().do_HEAD()

    def do_GET(self):
        if self.path.startswith("/api/status"):
            self.send_json({
                "status": "online",
                "device": "Raspberry Pi",
                "robot": "Dog",
                "dof": DOF,
                "servo_count": DOF,
                "hardware_connected": hardware.enabled,
                "python_version": sys.version,
            })
        elif self.path.startswith("/api/poses"):
            self.send_json(SAVED_POSES)
        elif self.path.startswith("/api/servos"):
            with state_lock:
                self.send_json({
                    "servos": SERVOS,
                    "state": dict(servo_state),
                    "calibration": dict(calibration_trims),
                    "body": dict(body_state),
                    "dof": DOF,
                })
        elif self.path.startswith("/api/servo/state"):
            with state_lock:
                self.send_json({"state": dict(servo_state), "body": dict(body_state)})
        elif self.path.startswith("/api/audio/list"):
            try:
                files = [f for f in os.listdir(AUDIO_DIR) if f.lower().endswith((".mp3", ".wav", ".ogg", ".m4a", ".flac"))]
                files.sort()
                self.send_json({"files": files})
            except Exception as e:
                self.send_json({"files": [], "error": str(e)})
        elif self.path.startswith("/api/camera/frame"):
            self.serve_camera_frame()
        elif self.path.startswith("/api/sensors/distances"):
            self.send_json(sensor_state)
        elif self.path.startswith("/api/motor/state"):
            self.send_json(motor_state)
        elif self.path.startswith("/api/tts/speak"):
            self.handle_tts()
        elif self.path.startswith("/api/audio/outputs") or self.path.startswith("/api/audio/sinks"):
            self.send_json({"outputs": local_audio.list_local_audio_outputs()})
        elif self.path.startswith("/api/audio/inputs") or self.path.startswith("/api/audio/sources"):
            self.send_json({"inputs": local_audio.list_local_audio_inputs()})
        elif self.path.startswith("/api/audio/devices"):
            self.send_json({
                "inputs": local_audio.list_local_audio_inputs(),
                "outputs": local_audio.list_local_audio_outputs(),
            })
        elif self.path.startswith("/api/models") or self.path.startswith("/api/ai/models"):
            self.send_json(ai_engine.get_available_models())
        else:
            super().do_GET()

    def do_POST(self):
        content_length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_length).decode("utf-8") if content_length > 0 else "{}"
        try:
            data = json.loads(body)
        except json.JSONDecodeError:
            data = {}

        if self.path in ["/api/audio/play", "/api/audio/play_local"]:
            audio_bytes, content_type = local_audio.load_audio_bytes_from_url(data.get("url", ""))
            if not audio_bytes:
                self.send_json({"error": "Unable to load audio for local playback"}, status=500)
            else:
                local_audio.play_audio_on_local_output(
                    audio_bytes, content_type,
                    data.get("speaker", "default"), data.get("volume", 100)
                )
                self.send_json({"success": True, "speaker": data.get("speaker", "default")})
            return

        if self.path == "/api/servo/set":
            angles = data.get("angles", {})
            with state_lock:
                for name, value in angles.items():
                    cfg = SERVO_BY_NAME.get(name)
                    if not cfg:
                        continue
                    try:
                        value = float(value) + float(calibration_trims.get(name, 0))
                    except Exception:
                        continue
                    servo_state[name] = max(cfg["min"], min(cfg["max"], value))
                if "body" in data and isinstance(data["body"], dict):
                    for key in ("height", "roll", "pitch", "yaw", "stance"):
                        if key in data["body"]:
                            try:
                                body_state[key] = float(data["body"][key])
                            except Exception:
                                pass
            apply_servo_state()
            with state_lock:
                self.send_json({"success": True, "state": dict(servo_state), "body": dict(body_state)})

        elif self.path == "/api/pose":
            name = data.get("name", "Stand")
            pose = SAVED_POSES.get(name)
            if not pose:
                self.send_json({"success": False, "error": "Unknown pose"}, status=404)
                return
            with state_lock:
                for key, value in pose.get("joints", {}).items():
                    if key in servo_state:
                        servo_state[key] = value
                for key, value in pose.get("body", {}).items():
                    if key in body_state:
                        body_state[key] = value
            apply_servo_state()
            self.send_json({"success": True, "pose": name, "state": dict(servo_state)})

        elif self.path == "/api/calibration":
            trims = data.get("calibration", data.get("trims", {}))
            with state_lock:
                for name, value in trims.items():
                    if name in calibration_trims:
                        try:
                            calibration_trims[name] = float(value)
                        except Exception:
                            pass
            self.send_json({"success": True, "calibration": dict(calibration_trims)})

        elif self.path == "/api/motor/l298":
            motor_state["mouth_channel1"] = data.get("mouth", False)
            motor_state["tail_channel2"] = data.get("tail", False)
            hardware.set_motors(motor_state["mouth_channel1"], motor_state["tail_channel2"])
            self.send_json({"success": True, "state": motor_state})

        elif self.path == "/api/camera/select":
            source = data.get("source", "usb")
            camera_stream.set_source(source)
            self.send_json({"success": True, "active_source": source})

        elif self.path == "/api/gemini/chat":
            user_text = data.get("prompt", "")
            DEFAULT_GEMINI_KEY = base64.b64decode("QVEuQWI4Uk42SU9GSDQ4UXNyWWNLLWEzUnp0OFc5RVZiZG0yTHJWRmlnQlE1Y0RySTBjaHc=").decode("utf-8")
            api_key = data.get("api_key") or os.environ.get("GEMINI_API_KEY") or DEFAULT_GEMINI_KEY
            include_vision = data.get("include_vision", True)
            image_bytes = self.capture_camera_bytes() if include_vision else None
            self.send_json(gemini_bridge.generate_chat_response(api_key, user_text, image_bytes, provider=data.get("provider", ""), model=data.get("model", ""), ollama_url=data.get("ollama_url", "")))

        elif self.path == "/api/gemini/voice":
            DEFAULT_GEMINI_KEY = base64.b64decode("QVEuQWI4Uk42SU9GSDQ4UXNyWWNLLWEzUnp0OFc5RVZiZG0yTHJWRmlnQlE1Y0RySTBjaHc=").decode("utf-8")
            api_key = data.get("api_key") or os.environ.get("GEMINI_API_KEY") or DEFAULT_GEMINI_KEY
            audio_b64 = data.get("audio_b64", "")
            mime_type = data.get("mime_type", "audio/webm")
            include_vision = data.get("include_vision", True)
            use_local_mic = bool(data.get("use_local_mic") or data.get("source") == "local")
            mic_device = data.get("mic_device") or data.get("mic") or ""
            if use_local_mic and not audio_b64:
                audio_bytes = local_audio.record_local_audio(duration=float(data.get("duration", 3.5)), device=mic_device)
                mime_type = "audio/wav"
            else:
                audio_bytes = base64.b64decode(audio_b64) if audio_b64 else None
            image_bytes = self.capture_camera_bytes() if include_vision else None
            self.send_json(gemini_bridge.generate_voice_chat_response(api_key, audio_bytes, mime_type, image_bytes, provider=data.get("provider", ""), model=data.get("model", ""), ollama_url=data.get("ollama_url", "")))

        elif self.path == "/api/gemini/gesture":
            frame = self.capture_camera_frame()
            if frame is None:
                self.send_json({"gesture": "None", "finger_count": 0, "action": None})
                return
            gesture, finger_count, _ = gemini_bridge.detect_hand_gestures(frame)
            action = None
            if gesture == "5_FINGERS_FORWARD":
                action = {"name": "dog_walk", "args": {"direction": "forward"}}
            elif gesture == "2_FINGERS_WAVE":
                action = {"name": "dog_pose", "args": {"pose_name": "Greeting Wave"}}
            elif gesture == "1_FINGER_ATTACK":
                action = {"name": "dog_pose", "args": {"pose_name": "Attack / Guard"}}
            self.send_json({"gesture": gesture, "finger_count": finger_count, "action": action})

        elif self.path == "/api/export-python":
            python_script = self.generate_hardware_script(data)
            self.send_response(200)
            self.send_header("Content-type", "application/x-python")
            self.send_header("Content-Disposition", 'attachment; filename="dog_17dof_control.py"')
            self.end_headers()
            self.wfile.write(python_script.encode("utf-8"))

        elif self.path == "/api/save-pose":
            pose_name = data.get("name", "Custom Pose")
            pose = data.get("pose", {})
            SAVED_POSES[pose_name] = pose
            self.send_json({"success": True, "message": "Saved pose '%s'" % pose_name})

        elif self.path == "/api/audio/upload":
            filename = data.get("filename", "track.mp3")
            data_b64 = data.get("data_b64", "")
            safe_filename = re.sub(r"[^a-zA-Z0-9_\-\.]", "_", filename)
            if not safe_filename.lower().endswith((".mp3", ".wav", ".ogg", ".m4a", ".flac")):
                safe_filename += ".mp3"
            try:
                raw_bytes = base64.b64decode(data_b64)
                with open(os.path.join(AUDIO_DIR, safe_filename), "wb") as f:
                    f.write(raw_bytes)
                self.send_json({"success": True, "filename": safe_filename, "url": "/audio/" + safe_filename})
            except Exception as e:
                self.send_json({"success": False, "error": str(e)}, status=500)

        elif self.path == "/api/audio/delete":
            filename = os.path.basename(data.get("filename", ""))
            file_path = os.path.join(AUDIO_DIR, filename)
            if os.path.exists(file_path):
                try:
                    os.remove(file_path)
                    self.send_json({"success": True, "message": "Deleted " + filename})
                except Exception as e:
                    self.send_json({"success": False, "error": str(e)}, status=500)
            else:
                self.send_json({"success": False, "error": "File not found"}, status=404)
        else:
            self.send_json({"error": "Endpoint not found"}, status=404)

    def handle_tts(self):
        query_params = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
        text_to_speak = query_params.get("text", ["Hello!"])[0]
        voice_id = query_params.get("voice", ["ai_female_us"])[0]
        try:
            from gtts import gTTS
            import io
            clean_text = re.sub(r"\*[^*]*\*", "", text_to_speak).strip() or "Hello!"
            tld_map = {
                "ai_female_us": ("en", "com"), "ai_male_us": ("en", "us"),
                "ai_male_uk": ("en", "co.uk"), "ai_male_au": ("en", "com.au"),
                "ai_male_ie": ("en", "ie"), "ai_male_in": ("en", "co.in"),
                "ai_male_za": ("en", "co.za"), "ai_male_ca": ("en", "ca"),
                "ai_female_uk": ("en", "co.uk"), "ai_female_au": ("en", "com.au"),
                "ai_female_ca": ("en", "ca"), "ai_female_in": ("en", "co.in"),
            }
            lang, tld = tld_map.get(voice_id, ("en", "com"))
            tts = gTTS(text=clean_text, lang=lang, tld=tld)
            mp3_fp = io.BytesIO()
            tts.write_to_fp(mp3_fp)
            mp3_fp.seek(0)
            audio_bytes = mp3_fp.read()
            self.send_response(200)
            self.send_header("Content-type", "audio/mpeg")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(audio_bytes)
        except Exception as e:
            self.send_json({"error": str(e)}, status=500)

    def capture_camera_frame(self):
        return camera_stream.get_frame()

    def capture_camera_bytes(self):
        if cv2 is None:
            return None
        frame = self.capture_camera_frame()
        if frame is None:
            frame = generate_synthetic_camera_frame()
        if frame is None:
            return None
        ret, jpeg = cv2.imencode(".jpg", frame)
        return jpeg.tobytes() if ret else None

    def serve_camera_frame(self):
        jpeg_bytes = self.capture_camera_bytes()
        if jpeg_bytes is None:
            try:
                self.send_response(503)
                self.end_headers()
            except Exception:
                pass
            return
        try:
            self.send_response(200)
            self.send_header("Content-type", "image/jpeg")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.end_headers()
            self.wfile.write(jpeg_bytes)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def send_json(self, data, status=200):
        try:
            self.send_response(status)
            self.send_header("Content-type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(data).encode("utf-8"))
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def generate_hardware_script(self, data):
        angles = data.get("angles", {})
        servos_literal = json.dumps(SERVOS, indent=4)
        script = '''#!/usr/bin/env python3
"""
Dog Robot 17-DOF Hardware Control Script
Generated by the Dog Control Center.

Servo bus:
  PCA9685 #1 (0x40): head, tail, ears, front-right leg (9 servos)
  PCA9685 #2 (0x41): front-left and rear legs        (8 servos)
Optional accessories:
  L298N dual H-bridge (mouth + tail)
  4x HC-SR04 ultrasonic sensors (front, rear, left, right)
"""

import time
import board
import busio
import RPi.GPIO as GPIO
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

SERVOS = __SERVOS__

L298_ENA, L298_IN1, L298_IN2 = 18, 23, 24
L298_ENB, L298_IN3, L298_IN4 = 19, 25, 8

SENSORS = {
    "front": {"trig": 5, "echo": 6},
    "rear":  {"trig": 12, "echo": 13},
    "left":  {"trig": 16, "echo": 20},
    "right": {"trig": 21, "echo": 26},
}

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([L298_ENA, L298_IN1, L298_IN2, L298_ENB, L298_IN3, L298_IN4], GPIO.OUT)
pwm_mouth = GPIO.PWM(L298_ENA, 1000)
pwm_tail = GPIO.PWM(L298_ENB, 1000)
pwm_mouth.start(0)
pwm_tail.start(0)

for name, pins in SENSORS.items():
    GPIO.setup(pins["trig"], GPIO.OUT)
    GPIO.setup(pins["echo"], GPIO.IN)
    GPIO.output(pins["trig"], False)

i2c = busio.I2C(board.SCL, board.SDA)
pca1 = PCA9685(i2c, address=0x40)
pca1.frequency = 50
pca2 = PCA9685(i2c, address=0x41)
pca2.frequency = 50
BOARDS = {"0x40": pca1, "0x41": pca2}

TARGET_ANGLES = __ANGLES__

def set_servo(name, logical_angle):
    cfg = next((s for s in SERVOS if s["name"] == name), None)
    if not cfg:
        return
    angle = max(0.0, min(180.0, 90.0 + float(logical_angle)))
    s = servo.Servo(BOARDS[cfg["board"]].channels[cfg["channel"]], min_pulse=500, max_pulse=2500)
    s.angle = angle

def apply_pose():
    for name, angle in TARGET_ANGLES.items():
        set_servo(name, angle)

def set_mouth_energized(energized):
    if energized:
        GPIO.output(L298_IN1, GPIO.HIGH)
        GPIO.output(L298_IN2, GPIO.LOW)
        pwm_mouth.ChangeDutyCycle(100)
    else:
        pwm_mouth.ChangeDutyCycle(0)

def set_tail_pulsating(pulsating):
    if pulsating:
        GPIO.output(L298_IN3, GPIO.HIGH)
        GPIO.output(L298_IN4, GPIO.LOW)
        pwm_tail.ChangeDutyCycle(80)
    else:
        pwm_tail.ChangeDutyCycle(0)

def read_distance_cm(sensor_name):
    pins = SENSORS.get(sensor_name)
    if not pins:
        return 0.0
    GPIO.output(pins["trig"], True)
    time.sleep(0.00001)
    GPIO.output(pins["trig"], False)
    start_time = time.time()
    stop_time = time.time()
    while GPIO.input(pins["echo"]) == 0:
        start_time = time.time()
        if start_time - stop_time > 0.03:
            break
    while GPIO.input(pins["echo"]) == 1:
        stop_time = time.time()
    elapsed = stop_time - start_time
    return round((elapsed * 34300) / 2, 1)

def main():
    print("Dog Robot 17-DOF controller online.")
    apply_pose()
    print("Applied %d servo angles." % len(TARGET_ANGLES))
    for s in SENSORS:
        print("Sensor %s: %.1f cm" % (s.upper(), read_distance_cm(s)))

if __name__ == "__main__":
    try:
        main()
    finally:
        GPIO.cleanup()
'''
        script = script.replace('__SERVOS__', servos_literal)
        script = script.replace('__ANGLES__', json.dumps(angles, indent=4))
        return script


class ThreadingReusableTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True
    ssl_context = None

    def get_request(self):
        newsocket, fromaddr = self.socket.accept()
        if self.ssl_context:
            try:
                first_byte = newsocket.recv(1, socket.MSG_PEEK)
                if first_byte and first_byte[0] == 0x16:
                    newsocket = self.ssl_context.wrap_socket(newsocket, server_side=True)
            except Exception:
                pass
        return newsocket, fromaddr


if __name__ == "__main__":
    for i in range(1, len(sys.argv)):
        if sys.argv[i] == "--port" and i + 1 < len(sys.argv):
            PORT = int(sys.argv[i + 1])
        elif sys.argv[i] == "--host" and i + 1 < len(sys.argv):
            HOST = sys.argv[i + 1]

    cert_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cert.pem")
    key_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "key.pem")
    if not (os.path.exists(cert_path) and os.path.exists(key_path)):
        cert_path = "/home/markirgang/Tweedpump/cert.pem"
        key_path = "/home/markirgang/Tweedpump/key.pem"

    ssl_ctx = None
    if os.path.exists(cert_path) and os.path.exists(key_path):
        try:
            ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ssl_ctx.load_cert_chain(certfile=cert_path, keyfile=key_path)
        except Exception as e:
            print("SSL cert load warning:", e)

    display_host = "localhost" if HOST in ["", "0.0.0.0", "127.0.0.1"] else HOST
    print("Starting Dog Robot 17-DOF Control Server on http://%s:%d (http & https supported)" % (display_host, PORT))
    bind_host = "" if HOST == "0.0.0.0" else HOST
    with ThreadingReusableTCPServer((bind_host, PORT), DogRequestHandler) as httpd:
        httpd.ssl_context = ssl_ctx
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nDog server stopped.")
