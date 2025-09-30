/*
  Medical Line-Follower Robot with Vital Signs Collection
  -------------------------------------------------------
  Features (as implemented here)
  • Differential drive using L298N (2 DC motors).
  • Two front IR reflectance sensors for black-line tracking.
    - Forward when neither sensor sees black
    - Correct left/right when only one sensor sees black
    - Full stop when both sensors see black (perpendicular line / stop mark)
  • At stop mark: ultrasonic distance check (HC-SR04). If an obstacle (patient) is
    detected closer than OBSTACLE_DISTANCE_CM, proceed to vital-signs workflow.
  • Vital signs: deploy a small lever via a servo to aim MAX30100 at the finger,
    acquire SpO2 and Heart Rate for a short window, compute robust averages.
  • Bluetooth (HC-05) transmission: human-readable + CSV line for easy parsing in MIT App.
  • Clear state machine; all “magic numbers” are named constants below.

  Assumptions & Notes
  • IR sensor modules: many output LOW on black and HIGH on white; set IR_ACTIVE_LOW
    accordingly after a quick bench test.
  • MAX30100 uses I2C. Install library “arduino-MAX30100” (PulseOximeter) by oxullo.
    In Library Manager search: “MAX30100 PulseOximeter”.
  • HC-05 wired to SoftwareSerial (RX=10, TX=11) at 9600 baud (default). Adjust if you
    configured your HC-05 differently.
  • Date/time are NOT added on Arduino (no RTC). Your MIT App should timestamp on receipt,
    then push to DB (per project design). 
  • Tune speeds/thresholds/angles below to match your chassis & sensor placement.

  Hardware pins (change to match your wiring)
  - L298N:
      ENA -> MOTOR_LEFT_EN (PWM),   IN1 -> MOTOR_LEFT_IN1,  IN2 -> MOTOR_LEFT_IN2
      ENB -> MOTOR_RIGHT_EN (PWM),  IN3 -> MOTOR_RIGHT_IN1, IN4 -> MOTOR_RIGHT_IN2
  - IR line sensors: IR_LEFT_PIN, IR_RIGHT_PIN (digital)
  - Ultrasonic: US_TRIG_PIN, US_ECHO_PIN
  - Servo: SERVO_PIN
  - HC-05 Bluetooth: BT_RX_PIN (Arduino receives), BT_TX_PIN (Arduino transmits)

  © You. Use freely in your lab/class.
*/

#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// ---------- Third-party library for MAX30100 ----------
#include <PulseOximeter.h>   // Install “MAX30100 PulseOximeter” by oxullo

// ===================== Tunable Constants =====================

// --- Line tracking & logic ---
const bool  IR_ACTIVE_LOW          = true;   // true if sensor reads LOW on black; flip if opposite
const int   IR_LEFT_PIN            = 2;
const int   IR_RIGHT_PIN           = 3;

// --- Motors (L298N) ---
const int   MOTOR_LEFT_EN          = 5;      // PWM
const int   MOTOR_LEFT_IN1         = 6;
const int   MOTOR_LEFT_IN2         = 7;

const int   MOTOR_RIGHT_EN         = 9;      // PWM
const int   MOTOR_RIGHT_IN1        = 8;
const int   MOTOR_RIGHT_IN2        = 4;

// Motor speeds (0..255). Tune for your chassis.
const uint8_t SPEED_FWD            = 160;
const uint8_t SPEED_TURN           = 140;
const uint8_t SPEED_CORRECTION     = 130;

// --- Ultrasonic (HC-SR04) ---
const int   US_TRIG_PIN            = 12;
const int   US_ECHO_PIN            = A0;
const float SOUND_SPEED_CM_PER_US  = 0.0343f / 2.0f;   // /2 for round-trip
const float OBSTACLE_DISTANCE_CM   = 40.0f;            // trigger distance

// --- Servo (finger lever) ---
const int   SERVO_PIN              = A1;
const int   SERVO_STOW_ANGLE       = 10;    // lever parked
const int   SERVO_MEASURE_ANGLE    = 70;    // lever toward the patient (tune)

// --- MAX30100 oximeter ---
PulseOximeter pox;
const uint32_t OXI_WARMUP_MS       = 1500;  // allow sensor to stabilize after deployment
const uint32_t MEASURE_WINDOW_MS   = 8000;  // total acquisition time
const uint8_t  OXI_LED_CURRENT     = 0x1F;  // default (library uses internal defaults too)

// --- Bluetooth (HC-05, 9600 default) ---
const int   BT_RX_PIN              = 10;   // Arduino RX  (to HC-05 TX)
const int   BT_TX_PIN              = 11;   // Arduino TX  (to HC-05 RX)
const long  BT_BAUD                = 9600;

// --- Safety / timeouts ---
const uint32_t STOP_SETTLE_MS      = 400;    // small pause after stopping on the cross line
const uint32_t POST_MEASURE_PAUSE  = 600;    // pause before retracting servo

// ===================== Globals =====================
SoftwareSerial BTSerial(BT_RX_PIN, BT_TX_PIN);
Servo fingerServo;

volatile bool beatDetected = false;
float lastHR = 0.0f, lastSpO2 = 0.0f;

// For averaging during the measurement window
float sumHR = 0.0f, sumSpO2 = 0.0f;
uint16_t cntHR = 0, cntSpO2 = 0;

// ===================== Utility: Motor Control =====================
void motorsStop() {
  analogWrite(MOTOR_LEFT_EN, 0);
  analogWrite(MOTOR_RIGHT_EN, 0);
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
}

void motorsForward(uint8_t speedL, uint8_t speedR) {
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, HIGH);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
  analogWrite(MOTOR_LEFT_EN, speedL);
  analogWrite(MOTOR_RIGHT_EN, speedR);
}

void motorsBackward(uint8_t speedL, uint8_t speedR) {
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, HIGH);
  analogWrite(MOTOR_LEFT_EN, speedL);
  analogWrite(MOTOR_RIGHT_EN, speedR);
}

void motorsTurnLeft() {
  // Right wheel forward, left wheel backward for a tight left correction
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  digitalWrite(MOTOR_RIGHT_IN1, HIGH);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
  analogWrite(MOTOR_LEFT_EN, SPEED_CORRECTION);
  analogWrite(MOTOR_RIGHT_EN, SPEED_TURN);
}

void motorsTurnRight() {
  // Left wheel forward, right wheel backward for a tight right correction
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, HIGH);
  analogWrite(MOTOR_LEFT_EN, SPEED_TURN);
  analogWrite(MOTOR_RIGHT_EN, SPEED_CORRECTION);
}

// ===================== Utility: Sensors =====================
bool readIR(int pin) {
  int v = digitalRead(pin);
  return IR_ACTIVE_LOW ? (v == LOW) : (v == HIGH); // true means "black detected"
}

float readDistanceCm() {
  // Standard HC-SR04 pulse cycle
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);

  long duration = pulseIn(US_ECHO_PIN, HIGH, 30000UL); // timeout ~30ms (~5m)
  if (duration == 0) return NAN;
  return duration * SOUND_SPEED_CM_PER_US;
}

// ===================== Oximeter callbacks =====================
void onBeatDetected() {
  beatDetected = true;
}

// ===================== State Machine =====================
enum State {
  LINE_FOLLOW = 0,
  INTERSECTION_STOP,
  OBSTACLE_CHECK,
  DEPLOY_SENSOR,
  OXI_WARMUP,
  MEASURE,
  TRANSMIT,
  RETRACT_SENSOR,
  RESUME_DRIVE
};

State state = LINE_FOLLOW;
uint32_t stateTS = 0;

// ===================== Setup =====================
void setup() {
  // Pins
  pinMode(IR_LEFT_PIN,  INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);

  pinMode(MOTOR_LEFT_EN,  OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);

  pinMode(MOTOR_RIGHT_EN,  OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);

  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);

  fingerServo.attach(SERVO_PIN);
  fingerServo.write(SERVO_STOW_ANGLE);

  // Serial for debug; Bluetooth for app
  Serial.begin(115200);
  BTSerial.begin(BT_BAUD);

  // Oximeter init
  if (!pox.begin()) {
    Serial.println(F("[OXI] MAX30100 init FAILED (check wiring & pullups)."));
  } else {
    Serial.println(F("[OXI] MAX30100 init OK."));
    pox.setIRLedCurrent(OXI_LED_CURRENT);  // optional tweak
    pox.setOnBeatDetectedCallback(onBeatDetected);
  }

  motorsStop();
  delay(300);
  Serial.println(F("[SYS] Ready."));
}

// ===================== Loop =====================
void loop() {
  // Always keep oximeter serviced (non-blocking)
  pox.update();

  switch (state) {

    case LINE_FOLLOW: {
      bool leftBlack  = readIR(IR_LEFT_PIN);
      bool rightBlack = readIR(IR_RIGHT_PIN);

      if (!leftBlack && !rightBlack) {
        // Both see white → drive straight
        motorsForward(SPEED_FWD, SPEED_FWD);
      } else if (rightBlack && !leftBlack) {
        // Right sees black → correct left
        motorsTurnLeft();
      } else if (leftBlack && !rightBlack) {
        // Left sees black → correct right
        motorsTurnRight();
      } else { 
        // Both see black → stop mark
        motorsStop();
        state = INTERSECTION_STOP;
        stateTS = millis();
        Serial.println(F("[NAV] Stop line detected."));
      }
    } break;

    case INTERSECTION_STOP: {
      if (millis() - stateTS >= STOP_SETTLE_MS) {
        state = OBSTACLE_CHECK;
      }
    } break;

    case OBSTACLE_CHECK: {
      float d = readDistanceCm();
      Serial.print(F("[NAV] Distance cm: ")); Serial.println(d);

      if (!isnan(d) && d <= OBSTACLE_DISTANCE_CM) {
        Serial.println(F("[NAV] Obstacle/patient found → proceeding to measurement."));
        state = DEPLOY_SENSOR;
      } else {
        Serial.println(F("[NAV] No obstacle → resume driving."));
        state = RESUME_DRIVE;
      }
    } break;

    case DEPLOY_SENSOR: {
      fingerServo.write(SERVO_MEASURE_ANGLE);
      stateTS = millis();
      state = OXI_WARMUP;

      // Reset accumulators
      sumHR = sumSpO2 = 0.0f;
      cntHR = cntSpO2 = 0;
      lastHR = lastSpO2 = 0.0f;
      beatDetected = false;

      Serial.println(F("[OXI] Sensor deployed; warming up…"));
    } break;

    case OXI_WARMUP: {
      if (millis() - stateTS >= OXI_WARMUP_MS) {
        stateTS = millis();
        state = MEASURE;
        Serial.println(F("[OXI] Warm-up done. Measuring…"));
      }
    } break;

    case MEASURE: {
      // Read current estimates
      float hr   = pox.getHeartRate();
      float spo2 = pox.getSpO2();

      // Accept sane values only
      if (hr > 25.0f && hr < 220.0f)   { sumHR += hr;   cntHR++;   lastHR = hr; }
      if (spo2 > 70.0f && spo2 <= 100.0f) { sumSpO2 += spo2; cntSpO2++; lastSpO2 = spo2; }

      if (millis() - stateTS >= MEASURE_WINDOW_MS) {
        state = TRANSMIT;
        Serial.println(F("[OXI] Measurement window ended."));
      }
    } break;

    case TRANSMIT: {
      motorsStop(); // just in case
      delay(POST_MEASURE_PAUSE);

      float avgHR   = (cntHR   > 0) ? (sumHR   / cntHR)   : NAN;
      float avgSpO2 = (cntSpO2 > 0) ? (sumSpO2 / cntSpO2) : NAN;

      // Human-readable
      Serial.println(F("----- MEASUREMENT RESULT -----"));
      Serial.print(F("Heart Rate (bpm): ")); Serial.println(isnan(avgHR) ? lastHR : avgHR, 1);
      Serial.print(F("SpO2 (%): "));         Serial.println(isnan(avgSpO2) ? lastSpO2 : avgSpO2, 1);

      // Over Bluetooth (both pretty + CSV)
      BTSerial.println(F("Vital signs acquired:"));
      BTSerial.print  (F("HR_bpm="));
      BTSerial.println(isnan(avgHR) ? lastHR : avgHR, 1);
      BTSerial.print  (F("SpO2_pct="));
      BTSerial.println(isnan(avgSpO2) ? lastSpO2 : avgSpO2, 1);

      // CSV (easy parse in MIT App: split by comma, then by '=')
      BTSerial.print(F("CSV,HR_bpm="));
      BTSerial.print(isnan(avgHR) ? lastHR : avgHR, 1);
      BTSerial.print(F(",SpO2_pct="));
      BTSerial.println(isnan(avgSpO2) ? lastSpO2 : avgSpO2, 1);

      state = RETRACT_SENSOR;
    } break;

    case RETRACT_SENSOR: {
      fingerServo.write(SERVO_STOW_ANGLE);
      delay(400);
      state = RESUME_DRIVE;
    } break;

    case RESUME_DRIVE: {
      // Roll back onto the main line and continue mission
      motorsForward(SPEED_FWD, SPEED_FWD);
      state = LINE_FOLLOW;
    } break;
  }
}
