#include <Servo.h>

#define MAX_SIGNAL 2000 // Standard max pulse length in microseconds
#define MIN_SIGNAL 1000 // Standard min pulse length in microseconds
#define ESC_PIN 9       // Pin connected to the ESC signal wire

Servo esc;

int current_throttle = 1000;

void setup() {
  Serial.begin(9600);
  esc.attach(ESC_PIN);

  Serial.println("--- ESC Calibration Started ---");
  Serial.println("1. Disconnect the LiPo battery.");
  Serial.println("2. Type any character and press Enter to send MAX signal...");
  
  // Wait for user to type something to proceed
  wait_for_input();

  // Step 1: Send High Signal
  Serial.println("Sending MAX signal (2000us).");
  esc.writeMicroseconds(MAX_SIGNAL);

  Serial.println("3. NOW connect the LiPo battery.");
  Serial.println("4. Wait for the ESC to beep (usually 2 short beeps).");
  Serial.println("5. Type any character and press Enter to send MIN signal...");

  // Wait for user to connect battery and confirm
  wait_for_input();

  // Step 2: Send Low Signal
  Serial.println("Sending MIN signal (1000us).");
  esc.writeMicroseconds(MIN_SIGNAL);

  delay(4000);

  Serial.println("6. Wait for a long confirmation beep.");
  Serial.println("Calibration Complete! You can now use the Serial Monitor to test.");
}

void loop() {
  if (Serial.available() > 0) {
    int val = Serial.parseInt();
    if (val >= 1000 && val <= 2000) {
      Serial.print("Setting throttle to: ");
      Serial.println(val);
      current_throttle = val;
    }
  }
  esc.writeMicroseconds(current_throttle);
}

// Helper function to pause until Serial input is received
void wait_for_input() {
  while (Serial.available() == 0) {
    // Do nothing until input arrives
  }
  while (Serial.available() > 0) {
    Serial.read(); // Clear the buffer
  }
}