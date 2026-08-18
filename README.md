# BEN — Basic Edge Node

**A minimal desktop companion robotics platform.**

<img width="763" height="870" alt="KakaoTalk_20260815_001912404" src="https://github.com/user-attachments/assets/c1ed3716-c423-47f7-a8aa-0f5b8f87f93d" />


BEN is a small desktop robot built around a simple idea: how far can a desktop companion be simplified while still remaining expressive?

It keeps only the essentials:

- 2 DOF
- Single-button input
- USB-powered
- LED eyes and heart

Design files are provided as-is. Feel free to build on them, modify them, or just poke around.

## Hardware

BEN is built around an **ESP32-C3 SuperMini** and a custom flexible PCB (FPCB).

| Component          | Specification                    |
| ------------------ | -------------------------------- |
| MCU                | ESP32-C3 SuperMini               |
| Degrees of freedom | 2 DOF (roll&pitch)                           |
| Actuators          | 2 × DM-S0020 2g micro servo      |
| Input              | 1 × Kailh Choc V2 mechanical switch |
| LEDs | 3 × LUXEON 3535L LED3 × LUXEON 3535L LED |
| Power              | 5 V USB C                         |
| PCB                | Custom FPCB                      |
| Mechanical parts   | 3D printed                       |

The two servos provide the neck motion, while the three LEDs provide simple visual expression.

The button on top of the head acts as BEN's primary physical input.

## Pinout

| Function             | ESP32-C3        |
| -------------------- | --------------- |
| Left / Right Eye LED | GPIO 0 / GPIO 3 |
| Heart LED            | GPIO 1          |
| Servo 1              | GPIO 4          |
| Servo 2              | GPIO 5          |
| Head Button          | GPIO 10         |
| Servo Power          | 5 V             |
| Logic Power          | 3.3 V           |
| Common Ground        | GND             |

The LEDs are connected through current-limiting resistors and can be brightness-controlled using PWM.

## Electronics

The complete editable electronics design is provided as a KiCad project:

```text
BEN.kicad_pro
BEN.kicad_sch
BEN.kicad_pcb
```

The files were converted and verified using **KiCad 9**.

The FPCB integrates the controller, LED connections, servo connections, and button interface into a single compact board designed to fit inside BEN.

## Mechanical Design

BEN uses a compact 2-DOF neck mechanism driven by two micro servos.

The mechanism was designed to minimize the number of actuators and mechanical components while still allowing expressive head movement.

STEP files are provided for modification and fabrication.

## Building BEN

### Electronics

Open:

```text
hardware/electronics/BEN.kicad_pro
```

with **KiCad 9 or later** to inspect or modify the FPCB.

The provided `.kicad_sch` and `.kicad_pcb` files contain the editable schematic and PCB design.

### Mechanical Parts

STEP files are provided under:

```text
hardware/CAD/
```

They can be modified in most CAD software or used to generate files for 3D printing.

### Firmware

The original button-triggered hardware test is provided as a standalone
ESP-IDF project in [`firmware/examples/internalTest/`](firmware/examples/internalTest/). Its local README contains
build, flash, and legacy test-pin instructions.

The [`firmware/examples/webCamMotion/`](firmware/examples/webCamMotion/)
example follows webcam head pitch and roll while laptop audio controls eye
brightness.

Feel free to modify it, add new behaviors, or use BEN as a platform for your own desktop robotics experiments.


## Building guide

A step-by-step build guide, including fabrication, assembly, wiring, and setup, is available on Instructables:


## Third-Party Resources

The project uses the following third-party resources:

* **Kailh Choc switch footprint** — [KiSwitch](https://github.com/kiswitch/kiswitch/tree/main/library/footprints/Switch_Keyboard_Kailh.pretty)
* **ESP32-C3 SuperMini symbol/footprint** — [SnapMagic / SnapEDA](https://www.snapeda.com/parts/ESP32-C3%20SuperMini_TH/Espressif%20Systems/view-part/)
* **ESP-IDF** — [Espressif IoT Development Framework](https://github.com/espressif/esp-idf), used to build the example firmware and distributed under the Apache License 2.0.
* **MediaPipe** — [Google MediaPipe](https://github.com/google-ai-edge/mediapipe), used for webcam face landmarks under the Apache License 2.0.
* **OpenCV** — [OpenCV](https://opencv.org/), used for camera capture and head-pose geometry under the Apache License 2.0.
* **NumPy** — [NumPy](https://numpy.org/), used for pose and audio calculations in the webcam bridge.
* **SoundCard** — [SoundCard](https://github.com/bastibe/SoundCard), used to capture laptop output or microphone audio.
* **pySerial** — [pySerial](https://github.com/pyserial/pyserial), used for the laptop-to-ESP32 connection.
* **pynput** — [pynput](https://github.com/moses-palmer/pynput), used to convert the physical switch event into a Space key press.

These resources remain subject to their respective original licenses and terms.

## License

BEN uses separate licenses for hardware and software.

### Hardware

CAD files, schematics, PCB designs, and other hardware design files are licensed under the **CERN Open Hardware Licence Version 2 — Weakly Reciprocal (CERN-OHL-W-2.0)**.

Commercial use and modification are allowed under the terms of the license.

See `LICENSE-HARDWARE` for details.

### Software

Firmware and software are licensed under the **MIT License**.

Commercial use and modification are allowed under the terms of the license.

See `LICENSE-SOFTWARE` for details.

---

**BEN — Basic Embodied Node**

Minimal hardware. Open design. Have fun with it.
