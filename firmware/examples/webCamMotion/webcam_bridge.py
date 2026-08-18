#!/usr/bin/env python3
"""Webcam head-pose and audio LED bridge for the BEN ESP32-C3 follower."""

import argparse
import math
import sys
import threading
import time

import cv2
import mediapipe as mp
import numpy as np
import serial
import soundcard as sc
from pynput import keyboard
from serial.tools import list_ports


ESPRESSIF_USB_VID = 0x303A
PACKET_SYNC = b"\xA5\x5A"
FLAG_ATTACH = 0x01

SERVO1_MIN = 90
SERVO1_MAX = 135
SERVO2_MIN = 50
SERVO2_MAX = 90
SERVO1_CENTER = (SERVO1_MIN + SERVO1_MAX) / 2.0
SERVO2_CENTER = (SERVO2_MIN + SERVO2_MAX) / 2.0

# MediaPipe Face Mesh indices: nose, chin, eye corners, mouth corners.
POSE_INDICES = (1, 152, 33, 263, 61, 291)
MODEL_POINTS = np.array(
    [
        (0.0, 0.0, 0.0),
        (0.0, -63.6, -12.5),
        (-43.3, 32.7, -26.0),
        (43.3, 32.7, -26.0),
        (-28.9, -28.9, -24.1),
        (28.9, -28.9, -24.1),
    ],
    dtype=np.float64,
)


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def find_esp32_port():
    ports = [
        port.device
        for port in list_ports.comports()
        if port.vid == ESPRESSIF_USB_VID
    ]
    if not ports:
        return None
    if len(ports) > 1:
        print(f"Multiple Espressif ports found; using {ports[0]}.", file=sys.stderr)
    return ports[0]


class WebcamBridge:
    def __init__(self, args, port):
        self.args = args
        self.serial = serial.Serial(port, args.baudrate, timeout=0.1)
        self.serial.reset_input_buffer()
        self.serial_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.space_keyboard = keyboard.Controller()

        self.eye_brightness = 0.0
        self.filtered_pitch = 0.0
        self.filtered_roll = 0.0
        self.neutral_pitch = 0.0
        self.neutral_roll = 0.0
        self.calibration_pitch = []
        self.calibration_roll = []
        self.calibration_remaining = args.calibration_frames
        self.last_face_time = 0.0
        self.last_servo1 = SERVO1_CENTER
        self.last_servo2 = SERVO2_CENTER

    def send_packet(self, servo1, servo2, heart, eye, attach):
        data = bytes(
            (
                int(clamp(round(servo1), SERVO1_MIN, SERVO1_MAX)),
                int(clamp(round(servo2), SERVO2_MIN, SERVO2_MAX)),
                int(clamp(round(heart), 0, 255)),
                int(clamp(round(eye), 0, 255)),
                int(clamp(round(eye), 0, 255)),
                FLAG_ATTACH if attach else 0,
            )
        )
        packet = PACKET_SYNC + data + bytes((sum(data) & 0xFF,))
        with self.serial_lock:
            self.serial.write(packet)

    def read_switch_events(self):
        while not self.stop_event.is_set():
            try:
                line = self.serial.readline().decode("utf-8", errors="ignore").strip()
            except serial.SerialException as error:
                if not self.stop_event.is_set():
                    print(f"Serial connection failed: {error}", file=sys.stderr)
                    self.stop_event.set()
                return
            if line == "EVENT:SPACE":
                self.space_keyboard.press(keyboard.Key.space)
                self.space_keyboard.release(keyboard.Key.space)
                print("Switch pressed -> Space")

    def capture_audio(self):
        if self.args.audio_source == "off":
            return
        try:
            if self.args.audio_source == "microphone":
                source = sc.default_microphone()
            else:
                speaker = sc.default_speaker()
                source = sc.get_microphone(
                    id=str(speaker.name), include_loopback=True
                )
            if source is None:
                raise RuntimeError(f"no {self.args.audio_source} source found")

            sample_rate = 48000
            frames = 2400
            smoothed = 0.0
            print(f"Audio source: {source.name}")
            with source.recorder(
                samplerate=sample_rate, channels=1, blocksize=frames
            ) as recorder:
                while not self.stop_event.is_set():
                    samples = np.asarray(
                        recorder.record(numframes=frames)[:, 0], dtype=np.float64
                    )
                    rms = float(np.sqrt(np.mean(samples * samples)))
                    magnitude = clamp(rms * self.args.audio_gain, 0.0, 1.0)
                    target = math.sqrt(magnitude) * self.args.eye_max
                    alpha = 0.30 if target > smoothed else 0.10
                    smoothed += (target - smoothed) * alpha
                    with self.state_lock:
                        self.eye_brightness = smoothed
        except Exception as error:
            print(f"Audio capture disabled: {error}", file=sys.stderr)
            with self.state_lock:
                self.eye_brightness = 0.0

    def reset_calibration(self):
        self.calibration_pitch.clear()
        self.calibration_roll.clear()
        self.calibration_remaining = self.args.calibration_frames
        print("Look naturally at the camera; recalibrating neutral pose.")

    def estimate_pose(self, landmarks, width, height):
        image_points = np.array(
            [
                (landmarks[index].x * width, landmarks[index].y * height)
                for index in POSE_INDICES
            ],
            dtype=np.float64,
        )
        focal_length = float(width)
        camera_matrix = np.array(
            [
                (focal_length, 0.0, width / 2.0),
                (0.0, focal_length, height / 2.0),
                (0.0, 0.0, 1.0),
            ],
            dtype=np.float64,
        )
        success, rotation_vector, _ = cv2.solvePnP(
            MODEL_POINTS,
            image_points,
            camera_matrix,
            np.zeros((4, 1), dtype=np.float64),
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success:
            return None
        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
        pitch, _, roll = cv2.RQDecomp3x3(rotation_matrix)[0]
        return float(pitch), float(roll)

    def pose_to_servos(self, pitch, roll):
        pitch_delta = pitch - self.neutral_pitch
        roll_delta = roll - self.neutral_roll
        if self.args.invert_pitch:
            pitch_delta = -pitch_delta
        if self.args.invert_roll:
            roll_delta = -roll_delta

        pitch_normalized = clamp(
            pitch_delta / self.args.pitch_limit_degrees, -1.0, 1.0
        )
        roll_normalized = clamp(
            roll_delta / self.args.roll_limit_degrees, -1.0, 1.0
        )

        # Pitch drives the servos oppositely; roll drives them together.
        servo1 = SERVO1_CENTER + pitch_normalized * 15.0 + roll_normalized * 7.5
        servo2 = SERVO2_CENTER - pitch_normalized * 13.0 + roll_normalized * 7.0
        return (
            clamp(servo1, SERVO1_MIN, SERVO1_MAX),
            clamp(servo2, SERVO2_MIN, SERVO2_MAX),
            pitch_delta,
            roll_delta,
        )

    def run(self):
        reader = threading.Thread(target=self.read_switch_events, daemon=True)
        audio = threading.Thread(target=self.capture_audio, daemon=True)
        reader.start()
        audio.start()

        camera = cv2.VideoCapture(self.args.camera)
        if not camera.isOpened():
            self.stop_event.set()
            raise RuntimeError(f"could not open camera {self.args.camera}")

        face_mesh = mp.solutions.face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        print(f"Connected to {self.serial.port}; Q exits, C recalibrates.")
        print("Hold a neutral pose while the first frames are calibrated.")

        try:
            while not self.stop_event.is_set():
                ok, frame = camera.read()
                if not ok:
                    print("Camera frame capture failed.", file=sys.stderr)
                    break
                frame = cv2.flip(frame, 1)
                height, width = frame.shape[:2]
                result = face_mesh.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                face_found = bool(result.multi_face_landmarks)
                now = time.monotonic()
                servo1 = self.last_servo1
                servo2 = self.last_servo2
                pitch_delta = 0.0
                roll_delta = 0.0

                if face_found:
                    pose = self.estimate_pose(
                        result.multi_face_landmarks[0].landmark, width, height
                    )
                    if pose is not None:
                        pitch, roll = pose
                        if self.calibration_remaining > 0:
                            self.calibration_pitch.append(pitch)
                            self.calibration_roll.append(roll)
                            self.calibration_remaining -= 1
                            if self.calibration_remaining == 0:
                                self.neutral_pitch = float(
                                    np.median(self.calibration_pitch)
                                )
                                self.neutral_roll = float(
                                    np.median(self.calibration_roll)
                                )
                                self.filtered_pitch = self.neutral_pitch
                                self.filtered_roll = self.neutral_roll
                                print("Neutral pose calibrated.")
                        else:
                            alpha = self.args.pose_smoothing
                            self.filtered_pitch += (
                                pitch - self.filtered_pitch
                            ) * alpha
                            self.filtered_roll += (
                                roll - self.filtered_roll
                            ) * alpha
                            servo1, servo2, pitch_delta, roll_delta = (
                                self.pose_to_servos(
                                    self.filtered_pitch, self.filtered_roll
                                )
                            )
                            self.last_servo1 = servo1
                            self.last_servo2 = servo2
                            self.last_face_time = now

                attach = (
                    self.calibration_remaining == 0
                    and now - self.last_face_time <= self.args.face_hold_seconds
                )
                with self.state_lock:
                    eye = self.eye_brightness
                self.send_packet(
                    servo1, servo2, self.args.heart_brightness, eye, attach
                )

                if self.args.preview:
                    status = (
                        f"calibrating {self.calibration_remaining}"
                        if self.calibration_remaining > 0
                        else f"pitch {pitch_delta:+.1f} roll {roll_delta:+.1f}"
                    )
                    cv2.putText(
                        frame, status, (20, 35), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 255, 0) if face_found else (0, 0, 255), 2
                    )
                    cv2.putText(
                        frame, f"servo {servo1:.0f}, {servo2:.0f} eye {eye:.0f}",
                        (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (255, 255, 255), 2
                    )
                    cv2.imshow("BEN webcam motion", frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break
                    if key == ord("c"):
                        self.reset_calibration()
        finally:
            self.stop_event.set()
            try:
                self.send_packet(
                    SERVO1_CENTER, SERVO2_CENTER, 0, 0, False
                )
            except serial.SerialException:
                pass
            face_mesh.close()
            camera.release()
            cv2.destroyAllWindows()
            reader.join(timeout=1.0)
            audio.join(timeout=1.0)
            self.serial.close()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Drive BEN from webcam head pose and laptop audio."
    )
    parser.add_argument("--port", help="COM5 or /dev/ttyACM0; auto-detected")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument(
        "--audio-source", choices=("output", "microphone", "off"),
        default="output"
    )
    parser.add_argument("--audio-gain", type=float, default=12.0)
    parser.add_argument("--eye-max", type=float, default=15.0)
    parser.add_argument("--heart-brightness", type=float, default=5.0)
    parser.add_argument("--pitch-limit-degrees", type=float, default=20.0)
    parser.add_argument("--roll-limit-degrees", type=float, default=20.0)
    parser.add_argument("--pose-smoothing", type=float, default=0.18)
    parser.add_argument("--calibration-frames", type=int, default=30)
    parser.add_argument("--face-hold-seconds", type=float, default=0.5)
    parser.add_argument("--invert-pitch", action="store_true")
    parser.add_argument("--invert-roll", action="store_true")
    parser.add_argument(
        "--no-preview", dest="preview", action="store_false"
    )
    parser.set_defaults(preview=True)
    return parser.parse_args()


def main():
    args = parse_args()
    port = args.port or find_esp32_port()
    if port is None:
        print("No Espressif serial port found; use --port.", file=sys.stderr)
        return 1
    try:
        WebcamBridge(args, port).run()
    except (serial.SerialException, RuntimeError) as error:
        print(error, file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
