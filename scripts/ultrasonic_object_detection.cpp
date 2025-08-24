/*
 * HC-SR04 Ultrasonic Sensor Test for ESP32-S3
 * This code tests only the ultrasonic sensor functionality
 * 
 * Connections:
 * HC-SR04 VCC -> ESP32 3.3V (or 5V if available)
 * HC-SR04 GND -> ESP32 GND
 * HC-SR04 TRIG -> ESP32 Pin 16
 * HC-SR04 ECHO -> ESP32 Pin 17
 */

// Pin definitions for ESP32-S3
const int TRIG_PIN = 16;             // Ultrasonic trigger pin
const int ECHO_PIN = 17;             // Ultrasonic echo pin

// Variables for distance measurement
long duration;
float distance_cm;
float distance_inches;

// Function declarations
float getUltrasonicDistance();
void displayDistanceBar(float dist);
void testSensorConnection();

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Wait for Serial Monitor to connect
  delay(1000);
  
  Serial.println("=================================");
  Serial.println("HC-SR04 Ultrasonic Sensor Test");
  Serial.println("ESP32-S3 WROOM-1");
  Serial.println("=================================");
  Serial.println("TRIG Pin: " + String(TRIG_PIN));
  Serial.println("ECHO Pin: " + String(ECHO_PIN));
  Serial.println("=================================");
  Serial.println();
  
  // Test sensor initialization
  testSensorConnection();
}

void loop() {
  // Get distance measurement
  distance_cm = getUltrasonicDistance();
  distance_inches = distance_cm * 0.393701; // Convert to inches
  
  // Display results
  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.print(" cm  |  ");
  Serial.print(distance_inches);
  Serial.print(" inches");
  
  // Add status indicators
  if (distance_cm <= 0 || distance_cm > 400) {
    Serial.print("  [OUT OF RANGE]");
  } else if (distance_cm < 5) {
    Serial.print("  [TOO CLOSE]");
  } else if (distance_cm < 20) {
    Serial.print("  [OBSTACLE DETECTED!]");
  } else if (distance_cm < 50) {
    Serial.print("  [CLOSE OBJECT]");
  } else {
    Serial.print("  [CLEAR PATH]");
  }
  
  Serial.println();
  
  // Add a visual bar graph for distance
  displayDistanceBar(distance_cm);
  
  delay(500); // Update every 500ms
}

float getUltrasonicDistance() {
  // Clear the trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Send 10μs pulse to trigger pin
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read the echo pin (timeout after 30ms to prevent hanging)
  duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  // Calculate distance in cm
  // Speed of sound = 343 m/s = 0.0343 cm/μs
  // Distance = (duration/2) * 0.0343
  float distance = duration * 0.0343 / 2;
  
  return distance;
}

void displayDistanceBar(float dist) {
  Serial.print("Distance Bar: [");
  
  // Create a visual bar (scale: 0-100cm)
  int barLength = 20; // Total bar length
  int filledBars = 0;
  
  if (dist > 0 && dist <= 100) {
    filledBars = (int)(dist / 5); // Each bar represents 5cm
    if (filledBars > barLength) filledBars = barLength;
  }
  
  for (int i = 0; i < barLength; i++) {
    if (i < filledBars) {
      Serial.print("█");
    } else {
      Serial.print("░");
    }
  }
  
  Serial.print("] ");
  Serial.print("(0-100cm scale)");
  Serial.println();
  Serial.println("---");
}

void testSensorConnection() {
  Serial.println("Testing sensor connection...");
  
  // Take 5 quick readings to test
  bool sensorWorking = true;
  int validReadings = 0;
  
  for (int i = 0; i < 5; i++) {
    float testDistance = getUltrasonicDistance();
    Serial.print("Test reading " + String(i + 1) + ": ");
    Serial.print(testDistance);
    Serial.println(" cm");
    
    if (testDistance > 0 && testDistance <= 400) {
      validReadings++;
    }
    
    delay(200);
  }
  
  Serial.println();
  if (validReadings >= 3) {
    Serial.println("✓ SENSOR TEST PASSED - HC-SR04 is working correctly!");
    Serial.println("Valid readings: " + String(validReadings) + "/5");
  } else {
    Serial.println("✗ SENSOR TEST FAILED - Check connections!");
    Serial.println("Valid readings: " + String(validReadings) + "/5");
    Serial.println();
    Serial.println("Troubleshooting tips:");
    Serial.println("1. Check VCC connection (3.3V or 5V)");
    Serial.println("2. Check GND connection");
    Serial.println("3. Verify TRIG pin connection (Pin " + String(TRIG_PIN) + ")");
    Serial.println("4. Verify ECHO pin connection (Pin " + String(ECHO_PIN) + ")");
    Serial.println("5. Ensure no loose connections");
    Serial.println("6. Try different pins if needed");
  }
  
  Serial.println();
  Serial.println("Starting continuous measurements...");
  Serial.println("=================================");
  Serial.println();
}
