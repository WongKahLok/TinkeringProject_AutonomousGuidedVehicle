#include <IRremote.hpp>

// ========== Pins ==========
#define IR_PIN 4
#define TRIG_PIN 16
#define ECHO_PIN 17

// Motor driver pins (L298N)
#define ENA 25  // PWM speed control Left
#define IN1 26  // Left motor forward
#define IN2 27  // Left motor backward
#define ENB 32  // PWM speed control Right
#define IN3 33  // Right motor forward
#define IN4 34  // Right motor backward

// ========== Globals ==========
IRrecv irrecv(IR_PIN);  // Create IR receiver object
decode_results results; // IR results structure

long duration;
float distance_cm;

enum State { FOLLOW, AVOID, STOP };
State currentState = FOLLOW;

// Timing variables for non-blocking behavior
unsigned long lastMeasurement = 0;
unsigned long lastAvoidAction = 0;
bool avoidanceInProgress = false;
int avoidanceStep = 0;

// Distance thresholds
const float OBSTACLE_THRESHOLD = 20.0;  // cm
const float SAFE_DISTANCE = 30.0;       // cm

// Motor speeds
const int FORWARD_SPEED = 180;
const int TURN_SPEED = 150;
const int REVERSE_SPEED = 150;
const int SEARCH_SPEED = 100;


// ========== Ultrasonic Function ==========
float getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) {
    return -1; // No echo received (timeout)
  }
  
  return duration * 0.0343 / 2;
}

// ========== Motor Control ==========
void driveMotors(int leftSpeed, int rightSpeed) {
  
  // Clamp values -255 to 255
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  // Left motor control
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(0, leftSpeed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(0, abs(leftSpeed));
  }
  
  // Right motor control
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    ledcWrite(1, rightSpeed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    ledcWrite(1, abs(rightSpeed));
  }
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}}

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

// ========== Behaviors ==========
void followHuman() {
  static unsigned long lastSearchTime = 0;
  static bool searchDirection = true; // true = right, false = left
  
  if (readIR()) {
    Serial.println("Beacon detected → moving forward");
    driveMotors(FORWARD_SPEED, FORWARD_SPEED);
    lastSearchTime = millis(); // Reset search timer
  } else {
    // No beacon detected - search behavior
    if (millis() - lastSearchTime > 2000) { // Search every 2 seconds
      Serial.println("Searching for beacon...");
      
      if (searchDirection) {
        driveMotors(SEARCH_SPEED, -SEARCH_SPEED); // Turn right
      } else {
        driveMotors(-SEARCH_SPEED, SEARCH_SPEED); // Turn left
      }
      
      delay(300); // Brief turn
      stopMotors();
      searchDirection = !searchDirection; // Alternate search direction
      lastSearchTime = millis();
    } else {
      Serial.println("No beacon detected → stopping");
      stopMotors();
    }
  }
}

void avoidObstacle() {
// default sequence: stop → reverse → turn right → forward → correct direction
  
  if (!avoidanceInProgress) {
    // Start avoidance sequence
    avoidanceInProgress = true;
    avoidanceStep = 0;
    lastAvoidAction = millis();
    Serial.println("Starting obstacle avoidance...");
  }
  
  unsigned long currentTime = millis();
  
  switch (avoidanceStep) {
    case 0: // Stop
      stopMotors();
      if (currentTime - lastAvoidAction > 200) {
        avoidanceStep = 1;
        lastAvoidAction = currentTime;
      }
      break;
      
    case 1: // Reverse
      Serial.println("Reversing...");
      driveMotors(-REVERSE_SPEED, -REVERSE_SPEED);
      if (currentTime - lastAvoidAction > 500) {
        avoidanceStep = 2;
        lastAvoidAction = currentTime;
      }
      break;
      
    case 2: // Turn right
      Serial.println("Turning right...");
      driveMotors(TURN_SPEED, -TURN_SPEED);
      if (currentTime - lastAvoidAction > 800) {
        avoidanceStep = 3;
        lastAvoidAction = currentTime;
      }
      break;
      
    case 3: // Move forward briefly
      Serial.println("Moving forward...");
      driveMotors(FORWARD_SPEED, FORWARD_SPEED);
      if (currentTime - lastAvoidAction > 600) {
        avoidanceStep = 4;
        lastAvoidAction = currentTime;
      }
      break;
      
    case 4: // Turn left to resume original direction
      Serial.println("Correcting direction...");
      driveMotors(-TURN_SPEED, TURN_SPEED);
      if (currentTime - lastAvoidAction > 400) {
        // End avoidance sequence
        stopMotors();
        avoidanceInProgress = false;
        avoidanceStep = 0;
        Serial.println("Obstacle avoidance complete");
      }
      break;
  }
}


// ========== Setup ==========
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 AGV Starting...");
  
  // IR receiver setup 
  irrecv.enableIRIn(); // Start the IR receiver
  
  // Ultrasonic sensor setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Motor driver setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // PWM setup for motor speed control
  ledcAttachPin(ENA, 0); // Channel 0 for left motor
  ledcAttachPin(ENB, 1); // Channel 1 for right motor
  ledcSetup(0, 1000, 8); // 1kHz frequency, 8-bit resolution
  ledcSetup(1, 1000, 8);
  
  // Initialize motors to stop
  stopMotors();
  
  Serial.println("Setup complete. AGV ready!");
}

// ========== Main Loop ==========
void loop() {
  unsigned long currentTime = millis();
  
  // Read ultrasonic sensor (non-blocking with timing control)
  if (currentTime - lastMeasurement > 50) { // Update every 50ms
    distance_cm = getUltrasonicDistance();
    lastMeasurement = currentTime;
    
    Serial.print("Distance: ");
    if (distance_cm < 0) {
      Serial.println("No reading");
    } else {
      Serial.print(distance_cm);
      Serial.println(" cm");
    }
  }
  
  // State machine logic
  if (!avoidanceInProgress) {
    if (distance_cm > 0 && distance_cm < OBSTACLE_THRESHOLD) {
      currentState = AVOID;
    } else if (distance_cm < 0 || distance_cm > SAFE_DISTANCE) {
      currentState = FOLLOW;
    }
  }
  
  // Execute current state behavior
  switch (currentState) {
    case FOLLOW:
      if (!avoidanceInProgress) {
        followHuman();
      }
      break;
      
    case AVOID:
      avoidObstacle();
      break;
      
    case STOP:
      stopMotors();
      break;
  }
  
  // Small delay to prevent overwhelming the processor
  delay(10);
}
