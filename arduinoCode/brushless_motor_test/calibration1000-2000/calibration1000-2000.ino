#include <Servo.h>

Servo esc; // Create a Servo object to control the ESC
int pwmValue = 1000; // Initial PWM value

void setup() {
  Serial.begin(9600); // Initialize Serial communication
  esc.attach(9);      // Attach ESC to pin 9

  // Initialize ESC
  Serial.println("Initializing ESC...");
  pwmValue = 1000; // Start with minimum throttle
  esc.writeMicroseconds(pwmValue);
  Serial.print("PWM Value: ");
  Serial.println(pwmValue);


  Serial.println("ESC Initialized.");
  Serial.println("Enter 'L' for 1000 µs or 'H' for 2000 µs.");
}

void loop() {
  // Check if data is available in Serial Monitor
  if (Serial.available() > 0) {
    char input = Serial.read(); // Read the incoming byte

    if (input == 'L' || input == 'l') {
      pwmValue = 1000; // Set PWM to 1000 µs
      esc.writeMicroseconds(pwmValue);
      Serial.println("Set PWM to 1000 µs (Low).");
    } else if (input == 'H' || input == 'h') {
      pwmValue = 2000; // Set PWM to 2000 µs
      esc.writeMicroseconds(pwmValue);
      Serial.println("Set PWM to 2000 µs (High).");
    } else {
      Serial.println("Invalid input. Use 'L' for 1000 µs or 'H' for 2000 µs.");
    }
  }
}
