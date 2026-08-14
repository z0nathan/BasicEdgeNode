# BEN — Edge Node

**A minimal desktop companion robotics platform.**

BEN is a small desktop robot built around a simple idea: how far can a desktop companion be simplified while still remaining expressive?

It keeps only the essentials:

- 2 DOF
- Single-button input
- USB-powered
- LED eyes and heart

The hardware and software are open source and intended to be easy to modify, rebuild, and experiment with.

## Hardware

BEN is built around an **ESP32-C3 SuperMini** and a custom flexible PCB (FPCB).

| Component          | Specification                    |
| ------------------ | -------------------------------- |
| MCU                | ESP32-C3 SuperMini               |
| Degrees of freedom | 2 DOF                            |
| Actuators          | 2 × micro servo                  |
| Input              | 1 × Kailh Choc mechanical switch |
| Eyes               | 2 × brightness-controllable LEDs |
| Heart              | 1 × brightness-controllable LED  |
| Power              | 5 V USB                          |
| PCB                | Custom FPCB                      |
| Mechanical parts   | 3D printed                       |

The two servos provide the neck motion, while the three LEDs provide simple visual expression.

The button on top of the head acts as BEN's primary physical input.

## Pinout

| Function             | ESP32-C3        |
| -------------------- | --------------- |
| Left / Right Eye LED | GPIO 0 / GPIO 1 |
| Heart LED            | GPIO 2          |
| Servo 1              | GPIO 4          |
| Servo 2              | GPIO 5          |
| Head Button          | GPIO 6          |
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

## Repository Structure

```text
BasicEdgeNode/
├── README.md
├── LICENSE-HARDWARE
├── LICENSE-SOFTWARE
│
├── hardware/
│   ├── CAD/
│   │   └── *.step
│   │
│   └── electronics/
│       ├── BEN.kicad_pro
│       ├── BEN.kicad_sch
│       └── BEN.kicad_pcb
│
└── firmware/
```

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

Firmware is provided in the `firmware/` directory.

Feel free to modify it, add new behaviors, or use BEN as a platform for your own desktop robotics experiments.

## Third-Party Resources

The electronics design uses the following third-party resources:

* **Kailh Choc switch footprint** — [KiSwitch](https://github.com/kiswitch/kiswitch/tree/main/library/footprints/Switch_Keyboard_Kailh.pretty)
* **ESP32-C3 SuperMini symbol/footprint** — [SnapMagic / SnapEDA](https://www.snapeda.com/parts/ESP32-C3%20SuperMini_TH/Espressif%20Systems/view-part/)

These resources remain subject to their respective original licenses and terms.

## License

BEN uses separate licenses for hardware and software.

### Hardware

CAD files, schematics, PCB designs, and other hardware design files are licensed under the **CERN Open Hardware Licence Version 2 — Weakly Reciprocal (CERN-OHL-W-2.0)**.

Commercial use and modification are allowed under the terms of the license.

See `LICENSE-HARDWARE` for details.

### Software

Firmware and software are licensed under the **GNU General Public License v3.0 (GPL-3.0)**.

Commercial use and modification are allowed under the terms of the license.

See `LICENSE-SOFTWARE` for details.

## Acknowledgements

Special thanks to **JLCPCB** for helping with the FPCB fabrication.

---

**BEN — Basic Embodied Node**

Minimal hardware. Open design. Have fun with it.
