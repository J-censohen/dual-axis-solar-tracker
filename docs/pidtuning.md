# PID Tuning for Dual-Axis Solar Tracker

## Overview
This solar tracker uses two separate PID controllers for:
- **Tilt (elevation)** using an SG90 servo motor
- **Pan (azimuth)** using a 28BYJ-48 stepper motor

Careful tuning of the PID parameters was critical for smooth, responsive motion without jitter or overcorrection.

## Tilt PID Parameters
```cpp
const float tilt_Kp = 250;
const float tilt_Ki = 0.1;
const float tilt_Kd = 20;
```

- **Kp (Proportional)**: Controls how aggressively the tilt adjusts to light imbalance.
- **Ki (Integral)**: Helps eliminate long-term drift when light levels are steady.
- **Kd (Derivative)**: Dampens fast swings when sudden shadows or movement occurs.

## Pan PID Parameters
```cpp
const float pan_Kp = 800;
const float pan_Ki = 0.1;
const float pan_Kd = 30;
```

- Pan axis requires more torque and has a slower dynamic response.
- Kp is high to ensure firm correction when misaligned.
- Damping via Kd prevents overshooting and noisy oscillation.

## Methodology
- PID gains were tuned manually based on real-time behavior of the system under changing light sources.
- Used serial output to analyze system behavior and tweak responsiveness.
- Prioritized **tilt-first logic** to avoid unnecessary power usage from pan motor.

## Notes
- These values work well under indoor flashlight simulation. Outdoor sunlight may require tuning adjustments.
- Future improvements may include:
  - Auto-tuning PID loops
  - Adaptive control based on ambient conditions

