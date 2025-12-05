#include <Arduino.h>
#include <SoftwareSerial.h>

/*
  STM32F103C8T6 - QTR8A (PA0..PA7) + DRV8833 + HC-05
  - Auto-calibration: runs at startup for CALIB_MS (or via 'c' command)
  - Normalizes sensor readings 0..1000 using calibrated min/max
  - IIR smoothing on normalized sensor values
  - PID control with anti-windup
  - Lost-line handling: use lastPosition to search, timeout stop
  - BT commands:
      c        -> recalibrate (3s)
      s        -> print current calibration
      P<x>     -> set Kp (e.g. P0.5)
      I<x>     -> set Ki
      D<x>     -> set Kd
*/

uint8_t sensorPins[] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7};
#define NUM_SENSORS 8

int rawValues[NUM_SENSORS];
int normValues[NUM_SENSORS];   // 0..1000 after calibration
float smoothValues[NUM_SENSORS]; // IIR smoothed

// Calibration arrays
int calMin[NUM_SENSORS];
int calMax[NUM_SENSORS];

#define CALIB_MS 3000    // duration of automatic calibration at startup (ms)
#define SAMPLE_AVG 5     // samples per analogRead average

// HC-05
SoftwareSerial BTSerial(PB11, PB10); // RX, TX

// DRV8833 motor driver pins (adjust if needed)
#define IN1_L PB0
#define IN2_L PB1
#define IN1_R PB6
#define IN2_R PB7

// PID params (defaults)
float Kp = 0.25;
float Ki = 0.00;
float Kd = 4.5;

// runtime PID state
float integral = 0.0;
int lastError = 0;
unsigned long lastLoopMs = 0;

// base speed 0..255
int baseSpeed = 150; // tune as needed

// smoothing alpha for IIR (0..1). higher -> less smoothing
const float SMOOTH_ALPHA = 0.35f;

// lost-line handling
bool lostLine = false;
int lastPosition = -1;
unsigned long lostStartMs = 0;
const unsigned long LOST_TIMEOUT_MS = 1200; // if can't find line in this time -> stop

// helper for reading averaged ADC
int readSensorRaw(uint8_t pin) {
  long sum = 0;
  for (int i = 0; i < SAMPLE_AVG; ++i) {
    sum += analogRead(pin);
  }
  return (int)(sum / SAMPLE_AVG);
}

// map with clamp for floats
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return (out_min + out_max) * 0.5f;
  float v = (x - in_min) / (in_max - in_min);
  if (v < 0) v = 0;
  if (v > 1) v = 1;
  return out_min + v * (out_max - out_min);
}

// perform calibration for durationMillis ms (collect min/max)
void calibrateSensors(unsigned long durationMillis) {
  // initialize min/max
  for (int i = 0; i < NUM_SENSORS; ++i) {
    calMin[i] = 4095;
    calMax[i] = 0;
  }
  unsigned long t0 = millis();
  while (millis() - t0 < durationMillis) {
    for (int i = 0; i < NUM_SENSORS; ++i) {
      int v = readSensorRaw(sensorPins[i]);
      if (v < calMin[i]) calMin[i] = v;
      if (v > calMax[i]) calMax[i] = v;
    }
    delay(5);
  }
  // safety: expand the range a bit to avoid zero-span
  for (int i = 0; i < NUM_SENSORS; ++i) {
    if (calMax[i] - calMin[i] < 30) {
      // small range, expand artificially
      int c = (calMax[i] + calMin[i]) / 2;
      calMax[i] = min(4095, c + 40);
      calMin[i] = max(0, c - 40);
    }
  }

  // initialize smoothed values based on midpoint
  for (int i = 0; i < NUM_SENSORS; ++i) {
    normValues[i] = (int)fmap((float)((calMax[i] + calMin[i]) / 2.0f), calMin[i], calMax[i], 0, 1000);
    smoothValues[i] = normValues[i];
  }
}

// normalize raw readouts to 0..1000 using calMin/calMax
void normalizeSensors() {
  for (int i = 0; i < NUM_SENSORS; ++i) {
    int raw = rawValues[i];
    int n = (int)fmap((float)raw, (float)calMin[i], (float)calMax[i], 0.0f, 1000.0f);
    normValues[i] = n;
    // IIR smoothing
    smoothValues[i] = SMOOTH_ALPHA * (float)n + (1.0f - SMOOTH_ALPHA) * smoothValues[i];
  }
}

// read all raw sensors, normalize & smooth
void sampleSensors() {
  for (int i = 0; i < NUM_SENSORS; ++i) {
    rawValues[i] = readSensorRaw(sensorPins[i]); // 0..4095
  }
  normalizeSensors();
}

// compute line position using weighted average on smoothed normalized values
// returns -1 if nothing seen (sum too small)
int readLinePosition() {
  float weighted = 0.0f;
  float sum = 0.0f;
  for (int i = 0; i < NUM_SENSORS; ++i) {
    float v = smoothValues[i]; // 0..1000 float
    // optional: threshold small values to 0
    if (v < 30.0f) v = 0.0f;
    weighted += v * (i * 1000.0f);
    sum += v;
  }
  if (sum < 5.0f) return -1; // nothing significant
  float pos = weighted / sum; // 0..7000
  return (int)(pos + 0.5f);
}

// motor helpers (Arduino analogWrite on STM32 typically 0..255)
void motorLeft(int speed) { // speed -255..255
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(IN2_L, LOW);
    analogWrite(IN1_L, speed);
  } else {
    digitalWrite(IN1_L, LOW);
    analogWrite(IN2_L, -speed);
  }
}
void motorRight(int speed) { // speed -255..255
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(IN2_R, LOW);
    analogWrite(IN1_R, speed);
  } else {
    digitalWrite(IN1_R, LOW);
    analogWrite(IN2_R, -speed);
  }
}

void stopMotors() {
  analogWrite(IN1_L, 0); digitalWrite(IN2_L, LOW);
  analogWrite(IN1_R, 0); digitalWrite(IN2_R, LOW);
}

// process BT commands (non-blocking)
void processBTCommands() {
  if (!BTSerial.available()) return;
  String cmd = BTSerial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("c")) {
    BTSerial.println("CALIBRATE");
    calibrateSensors(CALIB_MS);
    BTSerial.println("CAL DONE");
    // print calibration ranges
    for (int i = 0; i < NUM_SENSORS; ++i) {
      char buf[48];
      sprintf(buf, "s%d min=%d max=%d", i, calMin[i], calMax[i]);
      BTSerial.println(buf);
    }
    return;
  }

  if (cmd.equalsIgnoreCase("s")) {
    for (int i = 0; i < NUM_SENSORS; ++i) {
      char buf[64];
      sprintf(buf, "s%d raw=%d norm=%d smooth=%.1f min=%d max=%d",
              i, rawValues[i], normValues[i], smoothValues[i], calMin[i], calMax[i]);
      BTSerial.println(buf);
    }
    return;
  }

  // set PID: expecting P<number> or I<number> or D<number>
  if ((cmd[0] == 'P' || cmd[0] == 'p') && cmd.length() > 1) {
    float v = cmd.substring(1).toFloat();
    Kp = v;
    BTSerial.print("Kp=");
    BTSerial.println(Kp, 4);
    return;
  }
  if ((cmd[0] == 'I' || cmd[0] == 'i') && cmd.length() > 1) {
    float v = cmd.substring(1).toFloat();
    Ki = v;
    BTSerial.print("Ki=");
    BTSerial.println(Ki, 6);
    return;
  }
  if ((cmd[0] == 'D' || cmd[0] == 'd') && cmd.length() > 1) {
    float v = cmd.substring(1).toFloat();
    Kd = v;
    BTSerial.print("Kd=");
    BTSerial.println(Kd, 4);
    return;
  }

  BTSerial.print("Unknown cmd: ");
  BTSerial.println(cmd);
}

void setup() {
  BTSerial.begin(9600);
  Serial.begin(115200);

  // sensor pins
  for (int i = 0; i < NUM_SENSORS; ++i) {
    pinMode(sensorPins[i], INPUT_ANALOG);
  }

  // motor pins
  pinMode(IN1_L, OUTPUT); pinMode(IN2_L, OUTPUT);
  pinMode(IN1_R, OUTPUT); pinMode(IN2_R, OUTPUT);
  stopMotors();

  BTSerial.println("STM32 QTR8A AutoCal + Anti-Loss READY");
  Serial.println("Starting calibration... move robot left-right over line");

  // automatic calibration at startup
  calibrateSensors(CALIB_MS);
  BTSerial.println("Auto-calibration done. Send 'c' to recalibrate.");

  lastLoopMs = millis();
}

void loop() {
  // process BT commands first
  processBTCommands();

  // sample sensors and compute position
  sampleSensors();
  int pos = readLinePosition(); // -1 if none
  int center = (NUM_SENSORS - 1) * 1000 / 2; // 3500 for 8 sensors

  // debug line summary every loop via BT (you can change frequency)
  // build compact line string
  {
    char tmp[200];
    int n = snprintf(tmp, sizeof(tmp), "POS=%d center=%d Kp=%.3f Ki=%.6f Kd=%.3f",
                     pos, center, Kp, Ki, Kd);
    BTSerial.println(tmp);
  }

  unsigned long now = millis();
  float dt = (now - lastLoopMs) / 1000.0f;
  if (dt <= 0) dt = 0.001f;

  if (pos < 0) {
    // LOST LINE
    if (!lostLine) {
      lostLine = true;
      lostStartMs = now;
      BTSerial.println("LOST line - searching...");
    }
    // if we had a lastPosition, rotate toward it
    if (lastPosition >= 0) {
      if (lastPosition < center) {
        // turn left slowly (left backward, right forward)
        motorLeft(-120);
        motorRight(120);
      } else {
        // turn right slowly
        motorLeft(120);
        motorRight(-120);
      }
    } else {
      // no history: spin in place slowly
      motorLeft(-100); motorRight(100);
    }

    if (now - lostStartMs > LOST_TIMEOUT_MS) {
      // give up and stop
      stopMotors();
      BTSerial.println("LOST timeout - stopped");
    }

    // skip PID update
    lastLoopMs = now;
    delay(25);
    return;
  } else {
    // have line
    if (lostLine) {
      lostLine = false;
      BTSerial.println("Line reacquired");
    }
    lastPosition = pos;
  }

  // compute error normalized to -1..1
  float normFactor = (float)center;
  float error = ((float)pos - (float)center) / normFactor; // -1..1 approx

  // PID
  // P
  float P = error;
  // I with anti-windup
  integral += error * dt;
  // clamp integral
  float ItermMax = 10.0f;
  if (integral > ItermMax) integral = ItermMax;
  if (integral < -ItermMax) integral = -ItermMax;
  // D
  float derivative = (error - (float)lastError) / dt;

  float output = Kp * P + Ki * integral + Kd * derivative;
  lastError = (int)(error * 1000.0f); // store scaled

  // map output to motor differential - use baseSpeed as center
  // output roughly between -some..some. scale it
  float scale = 1.0f; // adjust if output too small/large
  float left = (float)baseSpeed + output * (float)baseSpeed * scale;
  float right = (float)baseSpeed - output * (float)baseSpeed * scale;

  // convert 0..255 domain and apply
  int leftPWM  = (int)constrain(left, -255, 255);
  int rightPWM = (int)constrain(right, -255, 255);

  motorLeft(leftPWM);
  motorRight(rightPWM);

  lastLoopMs = now;

  delay(15); // control loop ~60Hz
}
