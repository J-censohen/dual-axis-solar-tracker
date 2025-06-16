# Dual Axis Solar Tracker ☀️📡

An Arduino-based dual-axis solar tracker that uses a 28BYJ-48 stepper motor (with ULN2003 driver) and an SG90 servo motor. It dynamically follows the sun using 4 LDRs and PID control logic to maximize light capture. Ideal for prototyping efficient solar panel mounts.

---

## 🔧 Features

- **Dual-axis control** (pan & tilt)
- **PID-based sun tracking** for smooth responsiveness
- **Light-deadzone filtering** to prevent jitter
- **Safety constraints**:
  - Tilt: 5°–175°
  - Pan: ±90° from center
- **Optimized for 28BYJ + SG90 hardware**
- Median-filtered LDR readings for noise reduction

---

## 🧠 Tech Stack

- Arduino Uno
- 28BYJ-48 Stepper Motor + ULN2003 Driver
- SG90 Servo Motor
- 4x LDRs with voltage dividers
- PID control (custom implementation)
- AccelStepper & Servo libraries

---

## 🖼️ Media

| Tilt Tracking Demo | Final Build |
|--------------------|-------------|
| ![demo](media/tilt-demo.gif) | ![hardware](media/final-build.jpg) |

---

## 🧪 PID Tuning

See `docs/pid-tuning.md` for details on:
- PID constants
- Tuning methodology
- Effect of light fluctuations and deadzone thresholds

---

## 🧾 Bill of Materials (BOM)

See `hardware/parts-list.md`.

---

## 📁 Folder Structure

