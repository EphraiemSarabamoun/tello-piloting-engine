"""Tello face-follow autonomy v2. YuNet DNN detection + EMA-smoothed PID-style control.

Run on loki (or any host with WiFi joined to the Tello AP):

    uv run python follow.py [duration_sec]

Default duration 60s. Press Ctrl-C to land safely at any time.

Changes from v1:
- YuNet ONNX detector (more robust than Haar to angles/lighting/motion blur)
- EMA-smoothed control errors (less jerky → less battery drain)
- Lower KP gains and lower MAX_RC
- Refuses autonomous launch below MIN_BATTERY_TAKEOFF
- Lower-rate save of annotated frames
"""

import os
import signal
import sys
import time

import cv2
from djitellopy import Tello

MODEL_PATH = os.path.join(os.path.dirname(__file__), "models", "face_detection_yunet_2023mar.onnx")

# Tunables
LOOP_HZ = 20
TARGET_FACE_FRAC = 0.18      # face width as fraction of frame width — distance proxy
DEAD_ZONE_X_FRAC = 0.07
DEAD_ZONE_Y_FRAC = 0.10
DEAD_ZONE_SIZE = 0.04
MAX_RC = 22                  # cap RC magnitude (-100..100); lower = gentler
NO_FACE_TIMEOUT = 2.5
SCAN_TIMEOUT = 5.0
MIN_BATTERY_TAKEOFF = 40
LOW_BATTERY_LAND = 20
SAVE_FRAME_EVERY = 0.5
EMA_ALPHA = 0.4              # 0=full smoothing, 1=no smoothing

# Halved from v1
KP_YAW = 45
KP_UD = 40
KP_FB = 130


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class FaceFollower:
    def __init__(self):
        self.tello = Tello()
        if not os.path.exists(MODEL_PATH):
            raise FileNotFoundError(f"YuNet model missing at {MODEL_PATH}")
        self.det = cv2.FaceDetectorYN.create(MODEL_PATH, "", (320, 240), 0.6, 0.3, 5000)
        self.last_face_t = 0.0
        self.airborne = False
        self.last_save_t = 0.0
        self.ex_s = 0.0
        self.ey_s = 0.0
        self.esz_s = 0.0

    def safe_land(self):
        try:
            self.tello.send_rc_control(0, 0, 0, 0)
            time.sleep(0.3)
            self.tello.land()
            print("[safe_land] landed")
        except Exception as e:
            print(f"[safe_land] land failed ({e}); firing emergency")
            try: self.tello.emergency()
            except Exception as e2: print(f"[safe_land] emergency failed: {e2}")
        self.airborne = False

    def install_signal_handler(self):
        def handler(signum, _frame):
            print(f"\n[signal {signum}] landing")
            self.safe_land()
            sys.exit(0)
        signal.signal(signal.SIGINT, handler)
        signal.signal(signal.SIGTERM, handler)

    def detect(self, frame):
        h, w = frame.shape[:2]
        self.det.setInputSize((w, h))
        _, faces = self.det.detect(frame)
        if faces is None or len(faces) == 0:
            return None
        best = max(faces, key=lambda f: f[2] * f[3])
        x, y, fw, fh = best[:4].astype(int)
        return x, y, fw, fh

    def step(self, frame, now):
        h, w = frame.shape[:2]
        face = self.detect(frame)

        if face is not None:
            self.last_face_t = now
            x, y, fw, fh = face
            face_xc = (x + fw / 2) / w
            face_yc = (y + fh / 2) / h
            face_wf = fw / w

            ex = face_xc - 0.5
            ey = face_yc - 0.5
            esz = TARGET_FACE_FRAC - face_wf
            self.ex_s = EMA_ALPHA * ex + (1 - EMA_ALPHA) * self.ex_s
            self.ey_s = EMA_ALPHA * ey + (1 - EMA_ALPHA) * self.ey_s
            self.esz_s = EMA_ALPHA * esz + (1 - EMA_ALPHA) * self.esz_s

            yaw = int(clamp(self.ex_s * KP_YAW, -MAX_RC, MAX_RC)) if abs(self.ex_s) > DEAD_ZONE_X_FRAC else 0
            ud  = int(clamp(-self.ey_s * KP_UD, -MAX_RC, MAX_RC)) if abs(self.ey_s) > DEAD_ZONE_Y_FRAC else 0
            fb  = int(clamp(self.esz_s * KP_FB, -MAX_RC, MAX_RC)) if abs(self.esz_s) > DEAD_ZONE_SIZE else 0

            self.tello.send_rc_control(0, fb, ud, yaw)
            print(f"face xc={face_xc:.2f} yc={face_yc:.2f} w={face_wf:.2f} → fb={fb:+d} ud={ud:+d} yaw={yaw:+d}")

            if now - self.last_save_t > SAVE_FRAME_EVERY:
                cv2.rectangle(frame, (x, y), (x + fw, y + fh), (0, 255, 0), 2)
                cv2.circle(frame, (w // 2, h // 2), 8, (0, 0, 255), -1)
                cv2.imwrite("/tmp/follow_view.jpg", frame)
                self.last_save_t = now
        else:
            self.ex_s *= 0.7; self.ey_s *= 0.7; self.esz_s *= 0.7
            since = now - self.last_face_t if self.last_face_t else 999
            if since > SCAN_TIMEOUT:
                self.tello.send_rc_control(0, 0, 0, 18)
                print(f"no face {since:.1f}s — scanning (yaw +18)")
            elif since > NO_FACE_TIMEOUT:
                self.tello.send_rc_control(0, 0, 0, 0)
                print(f"no face {since:.1f}s — hovering")
            if now - self.last_save_t > SAVE_FRAME_EVERY:
                cv2.imwrite("/tmp/follow_view.jpg", frame)
                self.last_save_t = now

    def run(self, duration_sec=60.0):
        self.tello.connect()
        bat = self.tello.get_battery()
        print(f"[boot] battery={bat}%")
        if bat < MIN_BATTERY_TAKEOFF:
            print(f"battery {bat}% < {MIN_BATTERY_TAKEOFF}% required for autonomous run, abort")
            return

        try: self.tello.set_speed(20)
        except Exception as e: print(f"[boot] set_speed failed: {e}")

        self.install_signal_handler()
        self.tello.streamon()
        time.sleep(2.0)
        fr = self.tello.get_frame_read()

        self.tello.takeoff()
        self.airborne = True
        time.sleep(2.0)

        period = 1.0 / LOOP_HZ
        start = time.time()
        last_bat_check = start
        try:
            while True:
                t0 = time.time()
                if t0 - start > duration_sec:
                    print(f"[time] duration {duration_sec}s elapsed")
                    break
                if t0 - last_bat_check > 5.0:
                    bat = self.tello.get_battery()
                    last_bat_check = t0
                    if bat < LOW_BATTERY_LAND:
                        print(f"[battery] {bat}% < {LOW_BATTERY_LAND}% — landing")
                        break

                frame = fr.frame
                if frame is None or frame.size == 0:
                    time.sleep(period)
                    continue

                try:
                    self.step(frame, t0)
                except Exception as e:
                    print(f"[step] {e}")

                dt = time.time() - t0
                if dt < period:
                    time.sleep(period - dt)
        finally:
            self.safe_land()
            try: self.tello.streamoff()
            except Exception: pass
            print(f"[exit] battery={self.tello.get_battery()}%")


def validate_on_image(path):
    """Run the detector on a saved frame for sanity-check before flying."""
    import numpy as np
    img = cv2.imread(path)
    if img is None:
        print(f"could not read {path}")
        return
    det = cv2.FaceDetectorYN.create(MODEL_PATH, "", (img.shape[1], img.shape[0]), 0.6, 0.3, 5000)
    _, faces = det.detect(img)
    if faces is None or len(faces) == 0:
        print(f"{path}: NO face detected")
        return
    for f in faces:
        x, y, w, h = f[:4].astype(int)
        score = f[-1]
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        print(f"{path}: face at ({x},{y}) size {w}x{h} score {score:.2f}")
    out = path.rsplit(".", 1)[0] + "_detected.jpg"
    cv2.imwrite(out, img)
    print(f"  → annotated {out}")


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--validate":
        for p in sys.argv[2:]:
            validate_on_image(p)
        return
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    FaceFollower().run(duration)


if __name__ == "__main__":
    main()
