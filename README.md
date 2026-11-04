# Dog Robot 17-DOF Control Center

A standalone Raspberry Pi web application for a 17-servo quadruped dog robot.
It brings the full Shobots companion-robot feature set to a four-legged robot:
a Three.js 3D workbench, live servo calibration, Gemini multimodal AI,
camera/gesture control, speech synthesis, MP3 playback with dance sync,
ultrasonic telemetry, L298 accessory control and one-click Python export.

- **Local URL:** http://localhost:2190
- **Public URL:** https://dog.shobots.party
- **Shobots menu:** https://shobots.party
- **Project path:** `/home/markirgang/dog`

## 17 Degrees of Freedom

| # | Joint | Board | Channel | Group |
|---|-------|-------|---------|-------|
| 0 | Head Pan | 0x40 | 0 | Head |
| 1 | Head Tilt | 0x40 | 1 | Head |
| 2 | Tail Wag | 0x40 | 2 | Tail |
| 3 | L Ear | 0x40 | 3 | Ears |
| 4 | R Ear | 0x40 | 4 | Ears |
| 5 | LF Hip Yaw | 0x40 | 5 | Left Front Leg |
| 6 | LF Hip Pitch | 0x40 | 6 | Left Front Leg |
| 7 | LF Knee | 0x40 | 7 | Left Front Leg |
| 8 | RF Hip Yaw | 0x40 | 8 | Right Front Leg |
| 9 | RF Hip Pitch | 0x41 | 0 | Right Front Leg |
| 10 | RF Knee | 0x41 | 1 | Right Front Leg |
| 11 | LR Hip Yaw | 0x41 | 2 | Left Rear Leg |
| 12 | LR Hip Pitch | 0x41 | 3 | Left Rear Leg |
| 13 | LR Knee | 0x41 | 4 | Left Rear Leg |
| 14 | RR Hip Yaw | 0x41 | 5 | Right Rear Leg |
| 15 | RR Hip Pitch | 0x41 | 6 | Right Rear Leg |
| 16 | RR Knee | 0x41 | 7 | Right Rear Leg |

**2 head + 1 tail + 2 ears + 12 leg joints = 17 servos.**

## Features

- **3D control workbench** — procedural 17-joint quadruped, orbit camera, body
  height/pitch/roll/yaw, walk, trot, march and run gaits.
- **Servo calibration** — per-joint trim sliders, live 3D preview and test gait
  buttons. Trims are applied to live motion and exported code.
- **Gemini AI** — text chat, camera scene description, voice input and hand
  gesture actions (`dog_walk`, `dog_pose`, `dog_stop`).
- **Camera** — USB webcam or Raspberry Pi CSI camera via Picamera2, with a
  synthetic fallback frame when no camera is attached.
- **Audio** — MP3 import/playback, browser or local speaker output, gTTS speech
  synthesis and music-synced dance poses.
- **Sensors & accessories** — 4x HC-SR04 telemetry, obstacle avoidance and an
  L298N dual H-bridge for a mouth and pulsating-tail motor.
- **Python export** — downloads a ready-to-run `dog_17dof_control.py` script
  for both PCA9685 boards, the L298N and the ultrasonic sensors.

## Poses

Stand, Sit, Lie Down / Sleep, Wag Tail, Wave Right Paw, Wave Left Paw, Shake
Hands, Beg, Bow / Play, Dance, Walk in Place, Run in Place, Attack / Guard,
Curious Tilt, Greeting, Stretch, Low Stance, High Stance, Roll Over, Shake Off.

## Quick Start

```bash
cd /home/markirgang/dog
python3 server.py --port 2190 --host 0.0.0.0
```

Then open http://localhost:2190. The server also accepts HTTP and HTTPS on the
same port when a certificate is available.

## Hardware

- Raspberry Pi 4/5
- 2x PCA9685 16-channel PWM drivers (I2C addresses `0x40` and `0x41`)
- 17x servos
- 5V 10A UBEC for servo power (never power the servos from the Pi 5V rail)
- L298N dual H-bridge (optional mouth / tail motors)
- 4x HC-SR04 ultrasonic sensors (front, rear, left, right)
- USB webcam or Raspberry Pi CSI camera

Wiring details, GPIO pinouts and power/safety notes are available in the
**Wiring Schematic** tab of the web application.

## API Overview

| Method | Endpoint | Purpose |
|--------|----------|---------|
| GET | `/api/status` | Server, DOF and hardware status |
| GET | `/api/servos` | 17-servo map, state, calibration and body state |
| GET | `/api/poses` | Saved pose library |
| GET | `/api/sensors/distances` | Ultrasonic distances |
| GET | `/api/camera/frame` | JPEG camera frame |
| GET | `/api/audio/list` | MP3 files in `static/audio/` |
| GET | `/api/tts/speak?text=...` | Speech synthesis MP3 |
| POST | `/api/servo/set` | Set joint angles and body attitude |
| POST | `/api/pose` | Apply a named pose |
| POST | `/api/calibration` | Save servo trim offsets |
| POST | `/api/motor/l298` | Toggle mouth / tail motors |
| POST | `/api/gemini/chat` | Gemini text + vision chat |
| POST | `/api/gemini/voice` | Gemini voice chat |
| POST | `/api/gemini/gesture` | Hand gesture detection |
| POST | `/api/export-python` | Download hardware control script |
| POST | `/api/audio/upload` | Upload an MP3 file |

## Shobots Integration

The dog is registered in the Shobots master menu (`Shobots/config.json` and
`Shobots/server.py`) and in the Cloudflare tunnel ingress as
`dog.shobots.party -> http://localhost:2190`.

Start/stop it from the Shobots dashboard, or directly:

```bash
curl -X POST http://127.0.0.1:7090/api/start  -d '{"robot":"dog"}' -H 'Content-Type: application/json'
curl -X POST http://127.0.0.1:7090/api/stop   -d '{"robot":"dog"}' -H 'Content-Type: application/json'
curl -X POST http://127.0.0.1:7090/api/restart -d '{"robot":"dog"}' -H 'Content-Type: application/json'
```

## Notes

- The server runs in simulation mode when PCA9685 / RPi.GPIO libraries are not
  available. Set up the Adafruit CircuitPython PCA9685 and `RPi.GPIO` packages
  on the robot for real servo output.
- Servo calibration trims are stored in memory for the current run. Export the
  Python script after calibrating to persist the final angles in code.
- Keep all grounds (Pi, PCA9685 boards, UBEC, L298N) tied to a common star
  point and add a large capacitor across the servo power rail.
