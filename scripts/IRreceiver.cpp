#include <IRremote.h>

// Define the pin where IR receiver is connected
#define IR_RECEIVE_PIN 4

// Variables to prevent spam
unsigned long lastReceiveTime = 0;
uint32_t lastCommand = 0;
const unsigned long DEBOUNCE_TIME = 200; // 200ms debounce

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000); // Give time for serial to initialize
  
  // Initialize the IR receiver
  IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK); // Disable LED feedback to reduce noise
  
  Serial.println("=================================");
  Serial.println("ESP32 IR Receiver - Clean Version");
  Serial.println("=================================");
  Serial.println("Ready! Press remote buttons...");
  Serial.println();
}

void loop() {
  // Check if IR data is received
  if (IrReceiver.decode()) {
    
    // Get current time
    unsigned long currentTime = millis();
    
    // Filter out noise and repeated signals
    if (IrReceiver.decodedIRData.protocol != UNKNOWN && 
        IrReceiver.decodedIRData.command != 0 &&
        (currentTime - lastReceiveTime > DEBOUNCE_TIME || 
         IrReceiver.decodedIRData.command != lastCommand)) {
      
      // Print the received IR code
      Serial.println("--- IR Signal Received ---");
      Serial.print("Protocol: ");
      Serial.println(getProtocolString(IrReceiver.decodedIRData.protocol));
      
      Serial.print("Address: 0x");
      Serial.println(IrReceiver.decodedIRData.address, HEX);
      
      Serial.print("Command: 0x");
      Serial.println(IrReceiver.decodedIRData.command, HEX);
      
      // Check for specific commands
      interpretCommand(IrReceiver.decodedIRData.command);
      
      Serial.println("-------------------------");
      Serial.println();
      
      // Update tracking variables
      lastReceiveTime = currentTime;
      lastCommand = IrReceiver.decodedIRData.command;
    }
    
    // Resume receiving the next IR signal
    IrReceiver.resume();
  }
  
  // Small delay to prevent overwhelming
  delay(50);
}

void interpretCommand(uint32_t command) {
  // Common IR remote commands (adjust these based on your remote)
  switch(command) {
    case 0x40:
    case 0x45:
    case 0x12: // Various power button codes
      Serial.println("Action: POWER");
      break;
      
    case 0x18:
    case 0x46: // Volume up codes
      Serial.println("Action: VOLUME UP");
      break;
      
    case 0x19:
    case 0x47: // Volume down codes
      Serial.println("Action: VOLUME DOWN");
      break;
      
    case 0x52:
    case 0x43: // Channel up codes
      Serial.println("Action: CHANNEL UP");
      break;
      
    case 0x53:
    case 0x44: // Channel down codes
      Serial.println("Action: CHANNEL DOWN");
      break;
      
    case 0x16:
    case 0x0C: // OK/Enter button codes
      Serial.println("Action: OK/ENTER");
      break;
      
    default:
      Serial.print("Action: UNKNOWN (Code: 0x");
      Serial.print(command, HEX);
      Serial.println(")");
      break;
  }
}

// Function to pause/resume IR receiver
void pauseIR() {
  IrReceiver.stop();
  Serial.println("IR Receiver PAUSED");
}

void resumeIR() {
  IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);
  Serial.println("IR Receiver RESUMED");
}
