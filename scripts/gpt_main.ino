#include <IRremote.hpp>

// ================== Pin Definitions ==================
#define IR_PIN 4
#define TRIG_PIN 16
#define ECHO_PIN 17
#define IR_RECV_PIN 4

// Motor driver pins (example: L298N – adjust as needed)
#define ENA 25  // PWM speed control Left
#define IN1 26  // Left motor forward
#define IN2 27  // Left motor backward
#define ENB 32  // PWM speed control Right
#define IN3 33  // Right motor forward
#define IN4 34  // Right motor backward

// ================== Globals ==================
long duration;
float distance_cm;
IRrecv irrecv(IR_RECV_PIN);
decode_results results;

enum State { FOLLOW, AVOID, STOP };
State currentState = FOLLOW;

// ================== Functions ==================
float getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout 30ms
  return duration * 0.0343 / 2; // cm
}

void driveMotors(int leftSpeed, int rightSpeed) {
  // Clamp values 0–255
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Left motor
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(0, leftSpeed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(0, -leftSpeed);
  }

  // Right motor
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    ledcWrite(1, rightSpeed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    ledcWrite(1, -rightSpeed);
  }
}

void stopMotors() {
  driveMotors(0, 0);
}

// Function: read IR signal
bool readIR() {
  if (irrecv.decode(&results)) {
    // You can print the raw value for debugging
    Serial.print("IR code received: ");
    Serial.println(results.value, HEX);

    irrecv.resume();  // ready for next
    return true;      // signal detected
  }
  return false;       // no signal
}

// Function: follow human with single IR receiver
void followHuman() {
  if (readIR()) {
    Serial.println("Beacon detected → move forward");
    driveMotors(180, 180); // straight forward
  } else {
    Serial.println("No beacon detected → stop/search");
    stopMotors();
    // optional: rotate slowly to search for beacon
    // driveMotors(100, -100); // spin in place
  }
}


void avoidObstacle() {
  Serial.println("Avoiding obstacle...");
  stopMotors();
  delay(200);
  // Simple avoidance: reverse + turn
  driveMotors(-150, -150);
  delay(500);
  driveMotors(150, -150); // turn right
  delay(500);
  stopMotors();
}

// ================== Setup ==================
void setup() {
  Serial.begin(115200);

  // IR
  IrSender.begin(IR_PIN);

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Motors
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcAttachPin(ENA, 0); // Channel 0
  ledcAttachPin(ENB, 1); // Channel 1
  ledcSetup(0, 1000, 8); // 1kHz, 8-bit
  ledcSetup(1, 1000, 8);
}

// ================== Loop ==================
void loop() {
  distance_cm = getUltrasonicDistance();
  Serial.print("Distance: "); Serial.println(distance_cm);

  if (distance_cm > 0 && distance_cm < 20) {
    currentState = AVOID;
  } else {
    currentState = FOLLOW;
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
      break;
  }

  delay(50); // Loop delay
}
