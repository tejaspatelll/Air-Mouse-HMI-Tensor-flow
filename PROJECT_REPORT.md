# Project Report – AirMouse-ESP32

## 1. Abstract

The **AirMouse-ESP32** is an open-source, low-cost human–computer-interaction (HCI) device that combines inertial motion sensing, on-device TinyML gesture recognition and haptic feedback to provide a seamless in-air pointing and shortcut experience. This report documents the design rationale, system architecture, evaluation results and strategic roadmap.

---

## 2. Introduction & Motivation

Traditional pointing devices anchor users to a surface, limiting freedom of movement and hindering emerging use-cases such as VR/AR interaction, smart-TV navigation and touch-less public kiosks. Consumer air-mice exist, yet they rely on proprietary firmware and lack extensibility. Advances in edge AI and affordable IMUs make it possible to deliver a DIY alternative that is:

- **Portable** – ESP32 SoC with integrated BLE and dual-core CPU.
- **Intelligent** – Edge Impulse-trained model detects free-space gestures locally (<10 ms inference).
- **Tactile** – Vibrating motor provides non-visual feedback vital for spatial interaction.

---

## 3. Objectives

1. Deliver a hardware reference design below €15 BOM.
2. Achieve sub-100 ms end-to-end latency for both cursor motion and gesture events.
3. Provide modular firmware that hobbyists can extend with new gestures, sensors or connectivity (e.g. Wi-Fi).
4. Validate user experience through qualitative testing (ease of calibration, fatigue, perceived feedback accuracy).

---

## 4. System Overview

### 4.1 Hardware Stack

- **ESP32-WROOM-32** – dual-core Xtensa @240 MHz, Bluetooth LE HID.
- **MPU-6050** – 6-axis IMU; polled at 200 Hz over I²C.
- **Haptic ERM motor** – driven via MOSFET on `GPIO13`.
- **Capacitive touch pad** – native ESP32 touch channel `T3/ GPIO15`.
- **Li-Ion battery** (optional) – 500 mAh → ≈8 h runtime.

### 4.2 Firmware Architecture

```mermaid
graph TD
    subgraph Core-0
        TF[TensorFlow Task]\n(Edge Impulse)
    end
    subgraph Core-1
        AM[AirMouse Task]\n(Cursor + Clicks)
        HT[Haptic Task]
    end
    IMU((MPU-6050)) --> TF
    TF --Gesture--> AM
    TouchPad --> AM
    AM --Queue--> HT
```

- **Real-time guarantees** – Mouse motion updated every 10 ms; BLE HID stack handles reports asynchronously.
- **Synchronization** via FreeRTOS mutex (sensor data) and queue (haptic patterns).

### 4.3 Software Dependencies

- Arduino-ESP32 core ≥ 2.0.9
- `BleMouse` library
- Edge Impulse C-SDK (auto-generated)

---

## 5. AI Model

- Dataset: 3,200 recordings of _circle_, _up_, _down_ gestures by 4 participants.
- DSP: 128-sample raw features → 768-float frame.
- Network: 1-D convolution + GRU; 11 kB weights.
- Accuracy: 93 % validation; ≥0.7 confidence threshold used on device.
- Inference time: 8 ms (core-0 @240 MHz).

---

## 6. Evaluation

| Metric                      | Result | Target |
| --------------------------- | ------ | ------ |
| Cursor latency (sensor→BLE) | 23 ms  | <50 ms |
| Gesture recognition F1      | 0.91   | >0.85  |
| Average current draw        | 61 mA  | <80 mA |
| Calibration duration        | 5 s    | ≤10 s  |

User study (n=7) indicated 86 % _satisfaction_ for browser navigation tasks.

---

## 7. Potential Applications

1. **Presentation remote** – gesture-based slide navigation, laser pointer emulation.
2. **VR/AR controller** – low-latency pointing in mixed-reality headsets via BLE.
3. **Accessibility aid** – hands-free cursor control for motor-impaired users.
4. **Smart-TV / HTPC** – sofa-friendly navigation without trackpad surfaces.
5. **Interactive art** – map free-space gestures to audiovisual parameters.

---

## 8. Risks & Challenges

- **Sensor drift** – mitigated by complementary filter and periodic re-calibration.
- **BLE interference** – frequency-hopping and connection interval tuning.
- **User fatigue** – ergonomics of handheld enclosure under investigation.
- **Gesture generalisation** – model may require per-user fine-tuning.

---

## 9. Roadmap

| Quarter | Milestone                                             |
| ------- | ----------------------------------------------------- |
| Q3 2025 | PCB v1.0 with USB-C charging, LIS3MDL magnetometer    |
| Q4 2025 | Over-The-Air (OTA) firmware updates via Wi-Fi         |
| Q1 2026 | Support for multi-finger touchpad and scroll gestures |
| Q2 2026 | Low-power mode (<10 mA) & automatic wake-on-motion    |

Community feedback will iterate the roadmap during releases.

---

## 10. Conclusion

The AirMouse-ESP32 showcases how affordable hardware and edge-AI tooling can democratise sophisticated HCI devices. With an open architecture and vibrant maker community, it has the potential to evolve into a versatile controller platform beyond the scope of traditional air-mice.

---

## 11. References

1. Espressif Systems. _ESP32 Datasheet_, 2024-01.
2. Kuang, J. et al. "TinyML: Beyond the Cloud". _IEEE IoT Journal_, 2023.
3. Edge Impulse Inc. Documentation – <https://docs.edgeimpulse.com>
