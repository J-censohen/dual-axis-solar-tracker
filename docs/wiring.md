# Light Normalization Logic

## Overview
To determine how much the solar panel is misaligned from the strongest light source, four Light Dependent Resistors (LDRs) are arranged in a quadrant layout:
- **Top Left (TL)** — A0
- **Top Right (TR)** — A1
- **Bottom Right (BR)** — A2
- **Bottom Left (BL)** — A3

These readings are used to calculate normalized light differences in both the **azimuth (horizontal)** and **elevation (vertical)** directions.

## Median Filtering
Each analog reading is median-filtered using three quick samples to reduce noise:
```cpp
int readMedian(int pin) {
  int a = analogRead(pin);
  int b = analogRead(pin);
  int c = analogRead(pin);

  int smallest = min(a, min(b,c));
  int largest  = max(a, max(b,c));
  return a + b + c - smallest - largest;
}
```
This prevents occasional electrical noise from causing sudden erroneous motor movements.

## Normalized Error Calculation
Two key normalized values are computed from the four sensors:

```cpp
float azNorm = float((TR + BR) - (TL + BL)) / (TL + TR + BL + BR);
float elNorm = float((TR + TL) - (BR + BL)) / (TL + TR + BL + BR);
```

### Azimuth (Pan) Normalization `azNorm`
- Compares right-side light intensity (TR + BR) to left-side (TL + BL)
- Positive `azNorm` means light is stronger on the right → pan right
- Negative `azNorm` means stronger on the left → pan left

### Elevation (Tilt) Normalization `elNorm`
- Compares top light intensity (TR + TL) to bottom (BR + BL)
- Positive `elNorm` means light is higher → tilt up
- Negative `elNorm` means light is lower → tilt down

## Why Normalize?
By dividing by the total light:
- The values become scale-invariant (0 to ±1), regardless of ambient brightness.
- This allows tuning PID controllers in a consistent range.
- Also introduces a natural **deadzone**, reducing jitter under even lighting.

## Notes
- Normalization assumes balanced and calibrated LDRs. Significant sensor mismatch may introduce bias.
- Values are only meaningful when the total light sum is above a certain threshold — consider adding a low-light cutoff to reduce noise at night.

