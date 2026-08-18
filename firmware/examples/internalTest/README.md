# BEN internal hardware test

This standalone ESP-IDF project preserves the original button-triggered LED
and two-servo test sequence.

## Build

```bash
source /path/to/esp-idf/export.sh
cd firmware/examples/internalTest
idf.py build
```

To flash an attached ESP32-C3:

```bash
idf.py -p /dev/ttyACM0 flash monitor
```

Close any other serial monitor before flashing. Replace the serial port with
the correct COM port when building on Windows.

## Pin assignment

| Function | GPIO |
| --- | ---: |
| Eye LEDs | 0, 1 |
| Heart LED | 2 |
| Servos | 4, 5 |
| Switch | 6 |

This matches the BEN hardware pinout in the repository's main README.
