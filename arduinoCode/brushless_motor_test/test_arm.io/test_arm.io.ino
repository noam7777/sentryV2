#include <Servo.h>

Servo esc; // Create a Servo object to control the ESC
int pwmValue = 1000; // Starting PWM value
int increment = 10;   // Step size for increasing/decreasing the PWM
int delayTime = 10;  // Delay time between steps in milliseconds

void setup() {
  Serial.begin(9600); // Initialize Serial Monitor
  esc.attach(9);      // Attach ESC to pin 9

  // Initialize ESC at minimum throttle
  Serial.println("Initializing ESC...");
  esc.writeMicroseconds(pwmValue);
  Serial.print("PWM Value : ");
  Serial.println(pwmValue);
  delay(3000);
  Serial.println("ESC Initialized.");
}

void loop() {
  // Gradually increase PWM from 1000 to 2000
  for (pwmValue = 1000; pwmValue <= 1300; pwmValue += increment) {
    esc.writeMicroseconds(pwmValue); // Send PWM signal to ESC
    Serial.print("PWM Value (Increasing): ");
    Serial.println(pwmValue);
    delay(delayTime); // Wait for a short period
  }

  // Gradually decrease PWM from 2000 back to 1000
  for (pwmValue = 1400; pwmValue >= 1000; pwmValue -= increment) {
    esc.writeMicroseconds(pwmValue); // Send PWM signal to ESC
    Serial.print("PWM Value (Decreasing): ");
    Serial.println(pwmValue);
    delay(delayTime); // Wait for a short period
  }
  delay(5000);
}
