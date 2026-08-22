## BEN - Basic Edge Node
## minimal desktop robot platform for everyone.
## 
## 
## 
## Copyright (c) 2026 Minjae Kim
##
## See LICENSE for the full MIT License text.



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
NOSE_INDEX = 1
LEFT_EYE_INDEX = 33
RIGHT_EYE_INDEX = 263


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
                    target = self.args.eye_min + math.sqrt(magnitude) * (
                        self.args.eye_max - self.args.eye_min
                    )
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
        left_eye = np.array(
            (
                landmarks[LEFT_EYE_INDEX].x * width,
                landmarks[LEFT_EYE_INDEX].y * height,
            ),
            dtype=np.float64,
        )
        right_eye = np.array(
            (
                landmarks[RIGHT_EYE_INDEX].x * width,
                landmarks[RIGHT_EYE_INDEX].y * height,
            ),
            dtype=np.float64,
        )
        nose = np.array(
            (
                landmarks[NOSE_INDEX].x * width,
                landmarks[NOSE_INDEX].y * height,
            ),
            dtype=np.float64,
        )
        eye_vector = right_eye - left_eye
        eye_distance = float(np.linalg.norm(eye_vector))
        if eye_distance < 1.0:
            return None

        # Roll is the inclination of the line joining the two outer eye points.
        roll = math.degrees(math.atan2(eye_vector[1], eye_vector[0]))

        # Pitch is the nose's signed perpendicular distance from the eye line,
        # normalized by eye spacing and expressed as a geometric angle.
        nose_vector = nose - left_eye
        perpendicular_distance = (
            eye_vector[0] * nose_vector[1]
            - eye_vector[1] * nose_vector[0]
        ) / eye_distance
        pitch = math.degrees(
            math.atan2(perpendicular_distance, eye_distance)
        )
        # Image Y grows downward, so reverse pitch for intuitive head motion.
        return -pitch, roll

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

        # Mechanical axis mixing verified on the assembled neck:
        #   roll  = common-mode motion (both servos in the same direction)
        #   pitch = differential motion (servos in opposite directions)
        pitch_motion = pitch_normalized * self.args.servo_pitch_travel
        roll_motion = roll_normalized * self.args.servo_roll_travel
        servo1 = SERVO1_CENTER + roll_motion + pitch_motion
        servo2 = SERVO2_CENTER + roll_motion - pitch_motion
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
                face_landmarks = (
                    result.multi_face_landmarks[0].landmark
                    if face_found else None
                )
                now = time.monotonic()
                servo1 = self.last_servo1
                servo2 = self.last_servo2
                pitch_delta = 0.0
                roll_delta = 0.0

                if face_found:
                    pose = self.estimate_pose(
                        face_landmarks, width, height
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
                    if face_landmarks is not None:
                        for index in POSE_INDICES:
                            point = face_landmarks[index]
                            position = (
                                int(point.x * width), int(point.y * height)
                            )
                            cv2.circle(frame, position, 5, (0, 255, 255), -1)

                        left_eye = np.array(
                            (
                                face_landmarks[LEFT_EYE_INDEX].x * width,
                                face_landmarks[LEFT_EYE_INDEX].y * height,
                            )
                        )
                        right_eye = np.array(
                            (
                                face_landmarks[RIGHT_EYE_INDEX].x * width,
                                face_landmarks[RIGHT_EYE_INDEX].y * height,
                            )
                        )
                        nose_xy = np.array(
                            (
                                face_landmarks[NOSE_INDEX].x * width,
                                face_landmarks[NOSE_INDEX].y * height,
                            )
                        )
                        eye_vector = right_eye - left_eye
                        eye_length_squared = float(np.dot(eye_vector, eye_vector))
                        projection = left_eye
                        if eye_length_squared > 1.0:
                            amount = float(
                                np.dot(nose_xy - left_eye, eye_vector)
                                / eye_length_squared
                            )
                            projection = left_eye + eye_vector * amount
                        cv2.line(
                            frame, tuple(left_eye.astype(int)),
                            tuple(right_eye.astype(int)), (0, 255, 0), 2
                        )
                        cv2.line(
                            frame, tuple(projection.astype(int)),
                            tuple(nose_xy.astype(int)), (255, 0, 255), 2
                        )

                        nose = face_landmarks[1]
                        nose_point = (
                            int(nose.x * width), int(nose.y * height)
                        )
                        arrow_end = (
                            int(nose_point[0] + roll_delta * 4.0),
                            int(nose_point[1] + pitch_delta * 4.0),
                        )
                        cv2.arrowedLine(
                            frame, nose_point, arrow_end, (255, 100, 0), 3,
                            tipLength=0.22
                        )

                    status = (
                        f"calibrating {self.calibration_remaining}"
                        if self.calibration_remaining > 0
                        else (
                            f"pitch {pitch_delta:+.1f} deg  "
                            f"roll {roll_delta:+.1f} deg"
                        )
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
                    cv2.putText(
                        frame,
                        "FACE TRACKING" if attach else "NO FACE / DETACHED",
                        (20, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 255, 0) if attach else (0, 0, 255), 2
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
    parser.add_argument("--eye-min", type=float, default=5.0)
    parser.add_argument("--eye-max", type=float, default=15.0)
    parser.add_argument("--heart-brightness", type=float, default=5.0)
    parser.add_argument("--pitch-limit-degrees", type=float, default=10.0)
    parser.add_argument("--roll-limit-degrees", type=float, default=10.0)
    parser.add_argument("--servo-pitch-travel", type=float, default=18.0)
    parser.add_argument("--servo-roll-travel", type=float, default=18.0)
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
