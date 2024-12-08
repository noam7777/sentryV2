#include <Servo.h> // Include the Servo library

// shot mechanism parameters:
int durationForMotorsToSpeedUpMs = 1000;
int durationForTriggerToMoveFarwardMs = 700;
int triggerServoForwardPos = 90;
int triggerServoBackPos = 10;
int flywhillIdlePwm = 990;
int flywhillSpinPwm = 1150;

int pidLoopDelayMs = 50;

// Define a ServoController class
class ServoController {
private:
  Servo servo;         // Servo object
  int pin;             // PWM pin for the servo
  float currentPos;    // Current position of the servo (degrees)
  float velocity;      // Current velocity command (degrees per second)
  unsigned long lastUpdateTime; // Last time the servo position was updated
  float minAngle;      // Minimum angle for the servo
  float maxAngle;      // Maximum angle for the servo
  bool isBackwards;    // Flip the direction of commands if true

public:
  // Constructor
  ServoController(int pin, float minAngle = 0, float maxAngle = 180, bool isBackwards = false) 
    : pin(pin), currentPos((minAngle + maxAngle) / 2), velocity(0), 
      lastUpdateTime(0), minAngle(minAngle), maxAngle(maxAngle), 
      isBackwards(isBackwards) {}

  // Attach the servo to the specified pin
  void attach() {
    servo.attach(pin);
    servo.write(currentPos); // Set initial position to the midpoint
    lastUpdateTime = millis(); // Initialize the timer
  }

  // Set a velocity command for the servo (degrees per second)
  void setVelocity(int vel) {
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

  // Get the current position of the servo
  float getPosition() {
    return currentPos;
  }

  // Set the minimum and maximum angles for the servo
  void setAngleLimits(float minA, float maxA) {
    minAngle = minA;
    maxAngle = maxA;
  }

  // Set whether the servo is assembled backwards
  void setBackwards(bool backwards) {
    isBackwards = backwards;
  }
};

// Create ServoController objects for each servo
ServoController elevationServo(3, 69, 167); // elevation Servo on pin 3 with limits 69° to 167
ServoController azimuthServo(5, 5, 175); // azimuth Servo on pin 5, assembled backwards
Servo triggerServo;
Servo esc; // Create a Servo object to control the ESC

void armEscs(void) {
  // Initialize ESC at minimum throttle
  // restarting esc
  esc.writeMicroseconds(0);
  delay(2000); // Wait for a short period
  
  // Initialize ESC at minimum throttle
  esc.writeMicroseconds(flywhillIdlePwm);
  delay(5000); // Wait for a short period
}

int shotStateMachineEnterStateTimeStamp;

enum shotStates_e {
  SHOT_STATE_IDLE,
  SHOT_STATE_SPIN_FLYWHEEL,
  SHOT_STATE_TRIGGER_FARWARD
};

shotStates_e shotStates = SHOT_STATE_IDLE;
void setShotStateToSpinFlywheel(void) {
  esc.write(flywhillSpinPwm);
  triggerServo.write(triggerServoBackPos);
  shotStateMachineEnterStateTimeStamp = millis();
  shotStates = SHOT_STATE_SPIN_FLYWHEEL;
}

void setShotStateToTriggerForward(void) {
  triggerServo.write(triggerServoForwardPos);
  shotStateMachineEnterStateTimeStamp = millis();
  shotStates = SHOT_STATE_TRIGGER_FARWARD;
}

void setShotStateToIdle(void) {
  triggerServo.write(triggerServoBackPos);
  esc.write(flywhillIdlePwm);

  shotStateMachineEnterStateTimeStamp = millis();
  shotStates = SHOT_STATE_IDLE;
}


void shootNow(void) {
  setShotStateToSpinFlywheel();
}

void handleShot(void) {
  switch(shotStates){
    case SHOT_STATE_IDLE:
      break;

    case SHOT_STATE_SPIN_FLYWHEEL:
      if (millis() > (shotStateMachineEnterStateTimeStamp + durationForMotorsToSpeedUpMs)) {
        setShotStateToTriggerForward();
      }
      break;

    case SHOT_STATE_TRIGGER_FARWARD:
      if (millis() > (shotStateMachineEnterStateTimeStamp + durationForTriggerToMoveFarwardMs)) {
        setShotStateToIdle();
      }
      break;
  }
}

void setup() {
  elevationServo.attach();
  azimuthServo.attach();
  triggerServo.attach(6);
  armEscs();
  setShotStateToIdle();
  Serial.begin(9600); // For debugging
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
      int elevationRateCmd = command.substring(0, separatorIndex).toInt();
      int azimuthRateCmd = command.substring(separatorIndex + 1).toInt();

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