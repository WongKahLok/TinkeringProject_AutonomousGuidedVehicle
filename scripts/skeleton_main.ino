#include <IRremote.hpp>

// ========== Pins ==========
#define IR_PIN 4
#define TRIG_PIN 16
#define ECHO_PIN 17
// motor driver pins...

// ========== Globals ==========
long duration;
float distance_cm;

enum State { FOLLOW, AVOID, STOP };
State currentState = FOLLOW;

// ========== Ultrasonic Function ==========
float getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 30000);
  return duration * 0.0343 / 2;
}

// ========== Motor Control ==========
void driveMotors(int leftSpeed, int rightSpeed) {
  /* ... */ 
}

void stopMotors() {
  driveMotors(0, 0); 
}

// ========== Behaviors ==========
void followHuman() {
  // Replace with IR receiver tracking logic
  Serial.println("Following human...");
  driveMotors(180, 180);
}

void avoidObstacle() {
  Serial.println("Avoiding obstacle...");
  stopMotors();
  delay(200);
  driveMotors(-150, -150);
  delay(500);
  driveMotors(150, -150);
  delay(500);
  stopMotors();
}

// ========== Setup ==========
void setup() {
  Serial.begin(115200);

  // IR setup
  IrSender.begin(IR_PIN);

  // Ultrasonic setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Motor setup...
}

// ========== Loop ==========
void loop() {
  distance_cm = getUltrasonicDistance();
  Serial.print("Distance: "); Serial.println(distance_cm);

  if (distance_cm > 0 && distance_cm < 20) {
    currentState = AVOID;
  } else {
    currentState = FOLLOW;
  }

  switch (currentState) {
    case FOLLOW: followHuman(); break;
    case AVOID: avoidObstacle(); break;
    case STOP: stopMotors(); break;
  }

  delay(50);
}
