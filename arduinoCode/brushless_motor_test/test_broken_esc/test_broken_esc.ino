#include <Servo.h>

Servo esc; // Create a Servo object to control the ESC

int pwmValue = 1000; // Variable to store the current PWM value

void setup() {
  Serial.begin(9600); // Initialize Serial Monitor communication
  esc.attach(9);      // Attach ESC to pin 9

  // Initialize ESC with a sequence of PWM values
  Serial.println("Initializing ESC...");
  
  pwmValue = 2000; // Initialize at 0
  esc.writeMicroseconds(pwmValue);
  Serial.print("PWM Value: ");
  Serial.println(pwmValue);
  delay(4000);

  pwmValue = 1000; // Set to 1000µs
  esc.writeMicroseconds(pwmValue);
  Serial.print("PWM Value: ");
  Serial.println(pwmValue);
  delay(2000);

  Serial.println("ESC Initialized.");
}

void loop() {
  // Example: Changing the PWM dynamically
  pwmValue = 1100; // Back to 1000µs
  esc.writeMicroseconds(pwmValue);
  Serial.print("PWM Value: ");
  Serial.println(pwmValue);
  delay(2000);
}
