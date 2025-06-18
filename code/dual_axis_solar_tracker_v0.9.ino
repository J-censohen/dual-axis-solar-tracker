// Dual-Axis Solar Tracker with PID Control
// Uses LDR sensors to track sunlight and align a solar panel via servo (tilt) and stepper motor (pan)

#include <Servo.h>
#include <AccelStepper.h>
#include <Arduino.h>

// ----------------------------
// LDR Sensor Pins (Analog)
// ----------------------------
const int LDR_TL = A0;  // Top-left
const int LDR_TR = A1;  // Top-right
const int LDR_BR = A2;  // Bottom-right
const int LDR_BL = A3;  // Bottom-left

// ----------------------------
// Actuator Pins
// ----------------------------
const int servo = 9;       // PWM pin for tilt servo
const int step1 = 2;       // Stepper motor IN1
const int step2 = 3;       // Stepper motor IN2
const int step3 = 4;       // Stepper motor IN3
const int step4 = 5;       // Stepper motor IN4

// ----------------------------
// Control Parameters
// ----------------------------
const float norm_deadzone = 0.01;   // Ignore small light difference
const float max_panspeed = 1000.0;  // Max speed (steps/s) for panning
const float max_tiltspeed = 15.0;   // Max speed (deg/s) for tilting
const float tilt_first = 0.02;      // Require tilt to settle before panning

const int min_tilt = 5;             // Min servo angle
const int max_tilt = 175;           // Max servo angle
const int max_steps = 400;          // Pan limits (positive and negative)

// ----------------------------
// Utility: Get Median of 3 Reads
// ----------------------------
int readMedian(int pin) {
  int a = analogRead(pin);
  int b = analogRead(pin);
  int c = analogRead(pin);

  int smallest = min(a, min(b, c));
  int largest = max(a, max(b, c));
  return a + b + c - smallest - largest;
}

// ----------------------------
// PID Controller Class
// ----------------------------
class PID {
public:
  PID(float Kp, float Ki, float Kd, float outMin, float outMax)
    : Kp(Kp), Ki(Ki), Kd(Kd),
      prevError(0), integral(0),
      outMin(outMin), outMax(outMax) {}

  float compute(float error, float dt) {
    integral += error * dt;
    float deriv = (error - prevError) / dt;
    prevError = error;

    float output = Kp * error + Ki * integral + Kd * deriv;
    return constrain(output, outMin, outMax);
  }

private:
  float Kp, Ki, Kd;
  float prevError;
  float integral;
  float outMin, outMax;
};

// ----------------------------
// Hardware Setup
// ----------------------------
Servo tiltServo;
AccelStepper panStepper(AccelStepper::HALF4WIRE, step4, step2, step3, step1);

// PID Tuning Parameters
const float pan_Kp = 800;
const float pan_Ki = 0.1;
const float pan_Kd = 30;

const float tilt_Kp = 250;
const float tilt_Ki = 0.1;
const float tilt_Kd = 20;

PID panPID(pan_Kp, pan_Ki, pan_Kd, -max_panspeed, +max_panspeed);
PID tiltPID(tilt_Kp, tilt_Ki, tilt_Kd, -max_tiltspeed, +max_tiltspeed);

// System State
float currentTiltAngle = 95;    // Start angle for tilt
long panPosition = 0;           // Stepper position
unsigned long lastTime = 0;     // Time tracking for PID

// ----------------------------
// Arduino Setup
// ----------------------------
void setup() {
  Serial.begin(115200);

  tiltServo.attach(servo);
  tiltServo.write(currentTiltAngle);

  panStepper.setMaxSpeed(max_panspeed);
  panStepper.setAcceleration(200.0);
  panStepper.setCurrentPosition(0);

  lastTime = millis();
}

// ----------------------------
// Main Control Loop
// ----------------------------
void loop() {
  // Time Delta Calculation
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // Sensor Readings
  int TL = readMedian(LDR_TL);
  int TR = readMedian(LDR_TR);
  int BL = readMedian(LDR_BL);
  int BR = readMedian(LDR_BR);

  // Output raw sensor values
  Serial.print("TL="); Serial.print(TL);
  Serial.print(" TR="); Serial.print(TR);
  Serial.print(" BL="); Serial.print(BL);
  Serial.print(" BR="); Serial.println(BR);

  // Compute Normalized Errors
  float azNorm = float((TR + BR) - (TL + BL)) / (TL + TR + BL + BR); // Pan (Azimuth)
  float elNorm = float((TR + TL) - (BR + BL)) / (TL + TR + BL + BR); // Tilt (Elevation)

  float tiltSpeed = 0;

  // ----------------------------
  // Tilt Adjustment (Servo)
  // ----------------------------
  if (abs(elNorm) > norm_deadzone) {
    tiltSpeed = tiltPID.compute(elNorm, dt);
    currentTiltAngle += tiltSpeed * dt;
    currentTiltAngle = constrain(currentTiltAngle, min_tilt, max_tilt);
    tiltServo.write((int)(currentTiltAngle + 0.5));
  }

  // ----------------------------
  // Pan Adjustment (Stepper)
  // ----------------------------
  float panSpeed = 0;
  if (abs(elNorm) < tilt_first && abs(azNorm) > norm_deadzone) {
    panSpeed = panPID.compute(azNorm, dt);
    panStepper.setSpeed(panSpeed);
    
    long nextPos = panStepper.currentPosition() + panSpeed * dt;
    if (abs(nextPos) <= max_steps) {
      panStepper.runSpeed();
    } else {
      panStepper.setSpeed(0); // Limit range
    }
  } else {
    panStepper.setSpeed(0); // Wait for tilt to finish
  }

  // ----------------------------
  // Debug Output
  // ----------------------------
  Serial.print(" azNorm=");   Serial.print(azNorm,  3);
  Serial.print(" panSpd=");   Serial.print(panSpeed, 1);
  Serial.print(" elNorm=");   Serial.print(elNorm,  3);
  Serial.print(" tiltSpd=");  Serial.print(tiltSpeed,1);
  Serial.print(" angle=");    Serial.print(currentTiltAngle);
  Serial.print(" panStep=");  Serial.println(panStepper.currentPosition());

  delay(20); // Small delay for stability
}
