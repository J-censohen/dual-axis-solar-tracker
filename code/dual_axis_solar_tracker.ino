#include <Servo.h>
#include <AccelStepper.h>
#include <Arduino.h>

const int LDR_TL = A0;
const int LDR_TR = A1;
const int LDR_BR = A2;
const int LDR_BL = A3;

const int servo = 9;
const int step1 = 2;
const int step2 = 3;
const int step3 = 4;
const int step4 = 5;

const float norm_deadzone = 0.01;
const float max_panspeed = 1000.0;
const float max_tiltspeed = 15.0;
const float tilt_first = 0.02;

const int min_tilt = 5;
const int max_tilt = 175;

const int max_steps = 400;

int readMedian(int pin) {
  int a = analogRead(pin);
  int b = analogRead(pin);
  int c = analogRead(pin);
  
  int smallest = min(a, min(b,c));
  int largest = max(a, max(b,c));
  
  return a + b + c - smallest - largest;
  }

class PID {
public:
  PID(float Kp, float Ki, float Kd, float outMin, float outMax)
    : Kp(Kp), Ki(Ki), Kd(Kd),
      prevError(0), integral(0),
      outMin(outMin), outMax(outMax) {}

  float compute(float error, float dt) {
    integral    += error * dt;                        
    float deriv = (error - prevError) / dt;            
    prevError    = error;                              

    float output = Kp * error + Ki * integral + Kd * deriv;
    return constrain(output, outMin, outMax);
  }

private:
  float Kp, Ki, Kd;     
  float prevError;       
  float integral;        
  float outMin, outMax;  
};

Servo tiltServo;

AccelStepper panStepper(AccelStepper:: HALF4WIRE, step4, step2, step3, step1);

const float pan_Kp = 800;
const float pan_Ki = 0.1;
const float pan_Kd = 30;

const float tilt_Kp = 250;
const float tilt_Ki = 0.1;
const float tilt_Kd = 20;

PID panPID(pan_Kp, pan_Ki, pan_Kd, -max_panspeed, +max_panspeed);
PID tiltPID(tilt_Kp, tilt_Ki, tilt_Kd, -max_tiltspeed, +max_tiltspeed);

float currentTiltAngle = 95;
long panPosition = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);

  tiltServo.attach(servo);
  tiltServo.write(currentTiltAngle);

  panStepper.setMaxSpeed(max_panspeed);
  panStepper.setAcceleration(200.0);
  panStepper.setCurrentPosition(0);

  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now-lastTime)/1000.0;
  lastTime = now;

  int TL = readMedian(LDR_TL);
  int TR = readMedian(LDR_TR);
  int BL = readMedian(LDR_BL);
  int BR = readMedian(LDR_BR);

  Serial.print("TL="); Serial.print(TL);
  Serial.print(" TR="); Serial.print(TR);
  Serial.print(" BL="); Serial.print(BL);
  Serial.print(" BR="); Serial.println(BR);

  float azNorm = float((TR+BR) - (TL+BL)) / (TL+TR+BL+BR);
  float elNorm = float((TR+TL) - (BL+BR)) / (TL+TR+BL+BR);

float tiltSpeed = 0;

  if (abs(elNorm) > norm_deadzone) {
    tiltSpeed = tiltPID.compute(elNorm, dt);
    currentTiltAngle += tiltSpeed * dt;
    currentTiltAngle = constrain(currentTiltAngle, min_tilt, max_tilt);
    tiltServo.write((int)(currentTiltAngle + 0.5));
  }

  float panSpeed = 0;
  if (abs(elNorm) < tilt_first && abs(azNorm) > norm_deadzone) {
    panSpeed = panPID.compute(azNorm, dt);
    panStepper.setSpeed(panSpeed);
    long nextPos = panStepper.currentPosition() + panSpeed * dt;
    if (abs(nextPos) <= max_steps) {
      panStepper.runSpeed();
    } else {
      panStepper.setSpeed(0);
    }
  } else {
    panStepper.setSpeed(0); } 

  Serial.print("azNorm=");   Serial.print(azNorm,  3);
  Serial.print(" panSpd=");  Serial.print(panSpeed, 1);
  Serial.print(" elNorm=");  Serial.print(elNorm,  3);
  Serial.print(" tiltSpd="); Serial.print(tiltSpeed,1);
  Serial.print(" angle=");   Serial.print(currentTiltAngle);
  Serial.print(" panStep ="); Serial.println(panStepper.currentPosition());
  delay(20);

}
