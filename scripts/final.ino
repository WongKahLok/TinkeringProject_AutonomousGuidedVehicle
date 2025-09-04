/******************************************************
 * ESP32 Auto Car Carrier w/ IR Beacon Follow + Avoid
 * Using TB6612FNG motor driver (Left=Channel A, Right=Channel B)
 * With Debug Output
 ******************************************************/

#include <Arduino.h>
#include <IRremote.hpp>

// =================== Pins ===================
#define IR_RECEIVE_PIN 4

// Ultrasonic (HC-SR04)
#define TRIG_PIN 5
#define ECHO_PIN 18

// TB6612FNG Motor driver pins
#define PWMA 42   // PWM speed control Left
#define AIN1 40   // Left motor forward
#define AIN2 41   // Left motor backward
#define PWMB 21   // PWM speed control Right
#define BIN1 19   // Right motor forward
#define BIN2 20   // Right motor backward
#define STBY 48   // Standby pin

// =================== Globals ===================
unsigned long lastIRTime = 0;
uint32_t lastIRCommand = 0;
const unsigned long IR_DEBOUNCE_MS = 200;

long duration;
float distance_cm = -1;

enum State { FOLLOW, AVOID, STOP };
State currentState = FOLLOW;

unsigned long lastMeasurement = 0;
unsigned long lastAvoidAction = 0;
bool avoidanceInProgress = false;
int  avoidanceStep = 0;

const float OBSTACLE_THRESHOLD = 20.0;  // cm
const float SAFE_DISTANCE      = 30.0;  // cm

int FORWARD_SPEED = 180;
int TURN_SPEED    = 150;
int REVERSE_SPEED = 150;
int SEARCH_SPEED  = 100;

// =================== Helpers ===================

float getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1;
  return duration * 0.0343f / 2.0f;
}

void driveMotors(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  digitalWrite(STBY, HIGH);

  // Left
  if (leftSpeed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(0, leftSpeed);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(0, abs(leftSpeed));
  }

  // Right
  if (rightSpeed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(1, rightSpeed);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(1, abs(rightSpeed));
  }

  Serial.printf("[MOTOR] Left=%d, Right=%d\n", leftSpeed, rightSpeed);
}

void stopMotors() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(STBY, LOW);

  Serial.println("[MOTOR] STOP");
}

// =================== IR Handling ===================

void interpretCommand(uint32_t cmd) {
  switch (cmd) {
    case 0x40: case 0x45: case 0x12:
      Serial.println("[IR] STOP command received");
      currentState = STOP;
      stopMotors();
      break;

    case 0x16: case 0x0C:
      Serial.println("[IR] FOLLOW command received");
      currentState = FOLLOW;
      break;

    case 0x18: case 0x46:
      FORWARD_SPEED = min(FORWARD_SPEED + 10, 255);
      TURN_SPEED    = min(TURN_SPEED + 10, 255);
      REVERSE_SPEED = min(REVERSE_SPEED + 10, 255);
      SEARCH_SPEED  = min(SEARCH_SPEED + 10, 255);
      Serial.printf("[IR] SPEED UP: F:%d T:%d R:%d S:%d\n",
                    FORWARD_SPEED, TURN_SPEED, REVERSE_SPEED, SEARCH_SPEED);
      break;

    case 0x19: case 0x47:
      FORWARD_SPEED = max(FORWARD_SPEED - 10, 0);
      TURN_SPEED    = max(TURN_SPEED - 10, 0);
      REVERSE_SPEED = max(REVERSE_SPEED - 10, 0);
      SEARCH_SPEED  = max(SEARCH_SPEED - 10, 0);
      Serial.printf("[IR] SPEED DOWN: F:%d T:%d R:%d S:%d\n",
                    FORWARD_SPEED, TURN_SPEED, REVERSE_SPEED, SEARCH_SPEED);
      break;

    default:
      Serial.printf("[IR] Unknown Command 0x%lX\n", (unsigned long)cmd);
      break;
  }
}

bool readIRBeacon() {
  if (!IrReceiver.decode()) return false;
  unsigned long now = millis();
  auto &data = IrReceiver.decodedIRData;

  bool valid = (data.protocol != UNKNOWN && data.command != 0);
  bool fresh = (now - lastIRTime > IR_DEBOUNCE_MS) || (data.command != lastIRCommand);

  if (valid && fresh) {
    Serial.printf("[IR] Protocol=%s, Command=0x%X\n",
                  getProtocolString(data.protocol), data.command);
    interpretCommand(data.command);
    lastIRTime = now;
    lastIRCommand = data.command;
    IrReceiver.resume();
    return true;
  }
  IrReceiver.resume();
  return false;
}

// =================== Behaviors ===================

void followHuman() {
  static unsigned long lastSearchTime = 0;
  static bool searchRight = true;

  if (readIRBeacon() && currentState == FOLLOW) {
    Serial.println("[FOLLOW] Beacon detected → FORWARD");
    driveMotors(FORWARD_SPEED, FORWARD_SPEED);
    lastSearchTime = millis();
  } else {
    if (currentState != FOLLOW) {
      stopMotors();
      return;
    }

    if (millis() - lastSearchTime > 2000) {
      Serial.println("[FOLLOW] Searching for beacon...");
      if (searchRight) {
        driveMotors(SEARCH_SPEED, -SEARCH_SPEED);
      } else {
        driveMotors(-SEARCH_SPEED, SEARCH_SPEED);
      }
      delay(300);
      stopMotors();
      searchRight = !searchRight;
      lastSearchTime = millis();
    } else {
      Serial.println("[FOLLOW] No beacon detected → STOP");
      stopMotors();
    }
  }
}

void avoidObstacle() {
  if (!avoidanceInProgress) {
    avoidanceInProgress = true;
    avoidanceStep = 0;
    lastAvoidAction = millis();
    Serial.println("[AVOID] Starting obstacle avoidance...");
  }

  unsigned long t = millis();
  switch (avoidanceStep) {
    case 0:
      stopMotors();
      if (t - lastAvoidAction > 200) { avoidanceStep = 1; lastAvoidAction = t; }
      break;
    case 1:
      Serial.println("[AVOID] Reversing...");
      driveMotors(-REVERSE_SPEED, -REVERSE_SPEED);
      if (t - lastAvoidAction > 500) { avoidanceStep = 2; lastAvoidAction = t; }
      break;
    case 2:
      Serial.println("[AVOID] Turning right...");
      driveMotors(TURN_SPEED, -TURN_SPEED);
      if (t - lastAvoidAction > 800) { avoidanceStep = 3; lastAvoidAction = t; }
      break;
    case 3:
      Serial.println("[AVOID] Moving forward...");
      driveMotors(FORWARD_SPEED, FORWARD_SPEED);
      if (t - lastAvoidAction > 600) { avoidanceStep = 4; lastAvoidAction = t; }
      break;
    case 4:
      Serial.println("[AVOID] Correcting direction (left)...");
      driveMotors(-TURN_SPEED, TURN_SPEED);
      if (t - lastAvoidAction > 400) {
        stopMotors();
        avoidanceInProgress = false;
        avoidanceStep = 0;
        Serial.println("[AVOID] Obstacle avoidance complete");
        currentState = FOLLOW;
      }
      break;
  }
}

// =================== Setup / Loop ===================

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[SETUP] ESP32 Auto Carrier Starting...");

  IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);
  Serial.println("[SETUP] IR Receiver ready");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  ledcAttachPin(PWMA, 0);
  ledcAttachPin(PWMB, 1);
  ledcSetup(0, 1000, 8);
  ledcSetup(1, 1000, 8);

  stopMotors();
  Serial.println("[SETUP] Motor driver ready");
}

void loop() {
  unsigned long now = millis();

  if (now - lastMeasurement > 50) {
    distance_cm = getUltrasonicDistance();
    lastMeasurement = now;
    if (distance_cm < 0) Serial.println("[ULTRASONIC] No reading");
    else Serial.printf("[ULTRASONIC] Distance=%.1f cm\n", distance_cm);
  }

  if (!avoidanceInProgress) {
    if (distance_cm > 0 && distance_cm < OBSTACLE_THRESHOLD) {
      if (currentState != AVOID) Serial.println("[STATE] Switching to AVOID");
      currentState = AVOID;
    } else if (distance_cm < 0 || distance_cm > SAFE_DISTANCE) {
      if (currentState != STOP && currentState != FOLLOW) {
        Serial.println("[STATE] Switching to FOLLOW");
      }
      if (currentState != STOP) currentState = FOLLOW;
    }
  }

  switch (currentState) {
    case FOLLOW:
      followHuman();
      break;
    case AVOID:
      avoidObstacle();
      break;
    case STOP:
      stopMotors();
      if (IrReceiver.decode()) {
        auto &data = IrReceiver.decodedIRData;
        if (data.protocol != UNKNOWN && data.command != 0) {
          interpretCommand(data.command);
          lastIRTime = now;
          lastIRCommand = data.command;
        }
        IrReceiver.resume();
      }
      break;
  }

  delay(10);
}
