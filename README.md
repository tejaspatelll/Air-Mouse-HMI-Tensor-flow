# AirMouse-ESP32 📐

> A dual-core ESP32 + MPU-6050 based Bluetooth LE air-mouse that seamlessly switches between free-space mouse control and AI-powered gesture shortcuts, complete with rich haptic feedback.

![AirMouse banner](docs/img/hero.jpg)

## Table of Contents

1. Features
2. Hardware Required
3. Circuit / Pin-out
4. Getting Started
   1. Toolchain
   2. Library Installation
   3. Building & Flashing
5. Operating the AirMouse
   1. Modes & Gestures
   2. Haptic Patterns
6. Troubleshooting & FAQ
7. Contributing
8. License

---

## 1. Features

- **6-DoF motion tracking** with the MPU-6050 IMU (3-axis gyro + 3-axis accel).
- **Edge Impulse TinyML model** recognising air-drawn gestures (circle, up, down …).
- **Dual-core architecture** – TensorFlow inference on core-0, real-time mouse on core-1.
- **Automatic mode switching** between "Mouse" (device oriented flat) and "Gesture" (vertical).
- **BLE HID Mouse** (works on Windows, macOS, Linux, Android, iPadOS, RPi …).
- **Touch-pad click sensor** with single-tap, double-tap (right-click) & long-press drag.
- **Rich haptic feedback** for every interaction (connect, click, drag, gesture OK, error…).

---

## 2. Hardware Required

| Qty | Part                           | Example                           | Notes                           |
| --- | ------------------------------ | --------------------------------- | ------------------------------- |
| 1   | ESP32-DevKit v1                | [link](https://www.espressif.com) | Any ESP32-WROOM/-S3 works       |
| 1   | MPU-6050 IMU module            | GY-521 breakout                   | Mounted 3.3 V                   |
| 1   | Vibration motor + driver/FET   | ERM or LRA                        | Connected to `GPIO13`           |
| 1   | Tactile push-button            | restart                           | `GPIO25`, active-low            |
| 1   | Conductive pad / copper tape   | touch click                       | Attached to `TOUCH3` / `GPIO15` |
| …   | 3.7 V Li-Ion battery + charger | optional                          | Portable build                  |

> Total BOM ≈ 10-15 €.

---

## 3. Circuit / Pin-out

```
ESP32             MPU-6050      Other
-----             --------      -----
23  (GPIO23)  ->  SDA
19  (GPIO19)  ->  SCL
22  (GPIO22)  ->  **GND** (pulled low to power module ground)
13  (GPIO13)  ->  Haptic motor driver
15  (GPIO15/T3) -> Touch pad (active-low <40)
25  (GPIO25)  ->  Reset button (active-low)
3V3          ->  VCC (IMU & motor driver)
GND          ->  Common ground
```

Schematic PDF and KiCad files can be found in [`docs/hardware`](docs/hardware).

---

## 4. Getting Started

### 4.1 Toolchain

- [Arduino IDE ≥ 2.2](https://arduino.cc) (or PlatformIO)
- ESP32 Board package ≥ v2.0.9 (Preferences → Additional board URLs <https://espressif.github.io/arduino-esp32/package_esp32_index.json>)

### 4.2 Library Installation

Install the following libraries via the Arduino Library Manager or as ZIP files:

- **BleMouse** by T-vK (ID: 284838)
- Edge-Impulse generated library `minerfrands-project-1_inferencing.zip` (Menu → Sketch → Include Library → Add .ZIP …)

The rest (`Wire.h`, `freertos/…`) ship with the ESP32 core.

### 4.3 Building & Flashing

1. Clone this repository:
   ```bash
   git clone https://github.com/tejaspatelll/AirMouse-ESP32.git
   cd AirMouse-ESP32
   ```
2. Open `esp32_MPU6050_Tensorlite.ino` in Arduino IDE.
3. Select _Tools → Board → ESP32 Dev Module_ (or your variant).
4. Set the correct _COM/tty_ port, _Flash size_ & speed.
5. Click **Upload**. On first flash press the BOOT button when the IDE says _Connecting…_.
6. Open the Serial Monitor @115200 baud to view logs.

---

## 5. Operating the AirMouse

### 5.1 Modes & Gestures

| Condition                 | Mode             | Actions                                            |
| ------------------------- | ---------------- | -------------------------------------------------- |
| Device flat (within ±20°) | **Mouse Mode**   | Gyro controls cursor; touch pad for clicks & drag  |
| Device vertical           | **Gesture Mode** | Draw gestures in the air → _Back_, _Forward_, etc. |

Supported gestures (editable in Edge Impulse):

- **Circle** → Browser _Back_
- **Up / Down swipe** → Browser _Forward_

### 5.2 Haptic Patterns

| Pattern    | Meaning                  |
| ---------- | ------------------------ |
| 1 × 600 ms | BLE connected            |
| 1 × 350 ms | Tap / click              |
| 2 × 350 ms | Drag start / mode switch |
| 3 × 350 ms | Gesture accepted         |
| 4 × 350 ms | Error / restart          |

---

## 6. Troubleshooting & FAQ

**The IMU isn't detected** – Ensure `GPIO22` is pulled low and that SDA/SCL lines are not swapped.

**BLE device not discovered** – Make sure flashing completed, power-cycle ESP32, and that the BLE HID limit of your OS is not reached.

**Cursor drifts** – Hold device still during the _Calibrating..._ phase at startup. If drift persists, increase `numReadings` in `calibrateSensors()`.

For more see [`docs/troubleshooting.md`](docs/troubleshooting.md).

---

## 7. Contributing

Pull requests are welcome! Please open an issue first to discuss major changes. See the [contributing guide](CONTRIBUTING.md).

---

## 8. License

This project is licensed under the MIT License – see the [LICENSE](LICENSE) file for details.
