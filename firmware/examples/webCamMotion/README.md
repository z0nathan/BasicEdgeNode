# Webcam motion bridge

This example makes the ESP32-C3 a hardware follower. The laptop bridge owns
all behavior:

- MediaPipe Face Mesh estimates a person's webcam head pitch and roll;
- pitch and roll are smoothed and mapped into the safe servo ranges;
- laptop output audio or microphone magnitude controls both eye LEDs (0–15);
- the bridge sends heart LED brightness, eye brightness, servo targets, and
  servo attach state to the ESP32-C3;
- the ESP32-C3 reports a GPIO10 switch press as a Space key on the laptop;
- a one-second serial timeout turns LEDs off and detaches both servos.

The retained legacy wiring is GPIO 1/0/3 for the three LEDs, GPIO 4/5 for the
servos, and GPIO10 for the switch. Confirm the pinout before flashing a
different hardware revision.

## Build and flash the follower

```bash
cd firmware/examples/webCamMotion
source /path/to/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyACM0 flash
```

Use the board's COM port instead of `/dev/ttyACM0` on Windows. Do not run an
ESP-IDF serial monitor at the same time as the bridge.

## Install the bridge

Ubuntu:

```bash
cd firmware/examples/webCamMotion
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python webcam_bridge.py --port /dev/ttyACM0
```

Windows PowerShell:

```powershell
cd firmware/examples/webCamMotion
py -m venv .venv
.venv\Scripts\Activate.ps1
py -m pip install -r requirements.txt
py webcam_bridge.py --port COM5
```

Look naturally toward the camera during the first 30 frames. In the preview,
press `C` to recalibrate and `Q` or Escape to quit.

By default, laptop output audio drives the eyes. To use a microphone:

```bash
python webcam_bridge.py --port /dev/ttyACM0 --audio-source microphone
```

Useful adjustments:

```bash
python webcam_bridge.py \
  --pitch-limit-degrees 15 \
  --roll-limit-degrees 15 \
  --pose-smoothing 0.18 \
  --audio-gain 12
```

Smaller pitch/roll limits produce more servo travel for the same human head
movement. Use `--invert-pitch` or `--invert-roll` if an axis follows backward.
