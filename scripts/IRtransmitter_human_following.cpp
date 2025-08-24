#include <IRremote.hpp>

#define IR_PIN  4  // DAT pin of your IR LED connected to GPIO 4

void setup() {
  Serial.begin(115200);

  // Start IR sender without LED feedback
  IrSender.begin(IR_PIN);

  Serial.println("Starting continuous IR send...");
}

void loop() {
  // Send NEC protocol with address=0x00, command=0x10
  IrSender.sendNEC(0x00, 0x10, 0); // last '0' means no repeats
  delay(100); // short delay between sends
}
