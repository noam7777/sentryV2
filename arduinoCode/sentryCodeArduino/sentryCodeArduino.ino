#include <Servo.h> // Include the Servo library
int pidLoopDelayMs = 50;

// Define a ServoController class
class ServoController {
private:
  Servo servo;         // Servo object
  int pin;             // PWM pin for the servo
  float currentPos;    // Current position of the servo (degrees)
  float velocity;      // Current velocity command (degrees per second)
  float minAngle;      // Minimum angle for the servo
  float maxAngle;      // Maximum angle for the servo
  bool isBackwards;    // Flip the direction of commands if true

public:
  // Constructor
  ServoController(int pin, float minAngle = 0, float maxAngle = 180, bool isBackwards = false) 
    : pin(pin), currentPos((minAngle + maxAngle) / 2), velocity(0), 
      minAngle(minAngle), maxAngle(maxAngle), isBackwards(isBackwards) {}

  // Attach the servo to the specified pin
  void attach() {
    servo.attach(pin);
    servo.write(currentPos); // Set initial position to the midpoint
  }

  // Set a velocity command for the servo (degrees per second)
  void setVelocity(float vel) {
    velocity = isBackwards ? -vel : vel; // Flip the direction if assembled backwards
  }

  // Update the servo position based on the velocity
  void update() {
    float deltaTime = (float)pidLoopDelayMs / 1000;

    // Update the current position based on the velocity
    currentPos += velocity * deltaTime;

    // Constrain the position to the servo's limits
    currentPos = constrain(currentPos, minAngle, maxAngle);

    // Write the updated position to the servo
    servo.write(currentPos);
  }
};

// Create ServoController objects for each servo
ServoController elevationServo(3, 69, 167); // Elevation Servo on pin 3 with limits 69° to 167°
ServoController azimuthServo(5, 5, 175, true); // Azimuth Servo on pin 5, assembled backwards

void setup() {
  elevationServo.attach();
  azimuthServo.attach();

  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    // Read a string from the serial port
    String command = Serial.readStringUntil('\n');
    command.trim();

    // Parse the command to extract elevation and azimuth velocities
    int separatorIndex = command.indexOf(',');
    if (separatorIndex > 0) {
      float elevationRateCmd = command.substring(0, separatorIndex).toFloat();
      float azimuthRateCmd = command.substring(separatorIndex + 1).toFloat();

      // Set velocities
      elevationServo.setVelocity(elevationRateCmd);
      azimuthServo.setVelocity(azimuthRateCmd);
    }
  }

  // Update the servos
  elevationServo.update();
  azimuthServo.update();

  delay(pidLoopDelayMs); // Delay for smoother updates
}
