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
  bool isBackwards;    // Flip the direction if assembled backwards

public:
  ServoController(int pin, float minAngle = 0, float maxAngle = 180, bool isBackwards = false) 
    : pin(pin), currentPos((minAngle + maxAngle) / 2), velocity(0), 
      minAngle(minAngle), maxAngle(maxAngle), isBackwards(isBackwards) {}

  void attach() {
    servo.attach(pin);
    servo.write(currentPos); // Set initial position to the midpoint
  }

  void setVelocity(int velDeciDegreesPerSec) {
    float velDegreesPerSec = velDeciDegreesPerSec / 10.0; // Convert to degrees per second
    velocity = isBackwards ? -velDegreesPerSec : velDegreesPerSec; // Flip the direction if assembled backwards
  }

  void update() {
    float deltaTime = (float)pidLoopDelayMs / 1000;
    currentPos += velocity * deltaTime;
    currentPos = constrain(currentPos, minAngle, maxAngle);
    servo.write(currentPos);
  }

  float getPosition() {
    return currentPos;
  }
};

// Define a TriggerAndESC class
class TriggerAndESC {
private:
  Servo triggerServo;
  Servo esc;
  int flywheelIdlePwm;
  int flywheelSpinPwm;
  int triggerForwardPos;
  int triggerBackPos;
  int durationForMotorsToSpeedUpMs;
  int durationForTriggerToMoveForwardMs;
  int durationForTriggerToMoveBackwardMs = 500; // New state duration
  int stateEnterTimestamp;
  bool shootCommandActive;

  enum ShotStates {
    SHOT_STATE_IDLE,
    SHOT_STATE_SPIN_FLYWHEEL,
    SHOT_STATE_TRIGGER_FORWARD,
    SHOT_STATE_TRIGGER_BACKWARD
  };

  ShotStates currentState;

public:
  TriggerAndESC(int triggerPin, int escPin, int idlePwm, int spinPwm, int forwardPos, int backPos, int motorSpeedUpMs, int triggerMoveForwardMs)
    : flywheelIdlePwm(idlePwm), flywheelSpinPwm(spinPwm), 
      triggerForwardPos(forwardPos), triggerBackPos(backPos), 
      durationForMotorsToSpeedUpMs(motorSpeedUpMs), 
      durationForTriggerToMoveForwardMs(triggerMoveForwardMs),
      currentState(SHOT_STATE_IDLE), shootCommandActive(false) {
    triggerServo.attach(triggerPin);
    esc.attach(escPin);
  }

  void armESC() {
    esc.writeMicroseconds(0);
    delay(2000);
    esc.writeMicroseconds(flywheelIdlePwm);
    delay(5000);
  }

  void setStateToSpinFlywheel() {
    esc.writeMicroseconds(flywheelSpinPwm);
    triggerServo.write(triggerBackPos);
    stateEnterTimestamp = millis();
    currentState = SHOT_STATE_SPIN_FLYWHEEL;
  }

  void setStateToTriggerForward() {
    triggerServo.write(triggerForwardPos);
    stateEnterTimestamp = millis();
    currentState = SHOT_STATE_TRIGGER_FORWARD;
  }

  void setStateToTriggerBackward() {
    triggerServo.write(triggerBackPos);
    esc.writeMicroseconds(flywheelIdlePwm); // Set ESC to idle
    stateEnterTimestamp = millis();
    currentState = SHOT_STATE_TRIGGER_BACKWARD;
  }

  void setStateToIdle() {
    stateEnterTimestamp = millis();
    currentState = SHOT_STATE_IDLE;
  }

  void handleShot() {
    switch (currentState) {
      case SHOT_STATE_IDLE:
        break;
      case SHOT_STATE_SPIN_FLYWHEEL:
        if (millis() > (stateEnterTimestamp + durationForMotorsToSpeedUpMs)) {
          setStateToTriggerForward();
        }
        break;
      case SHOT_STATE_TRIGGER_FORWARD:
        if (millis() > (stateEnterTimestamp + durationForTriggerToMoveForwardMs)) {
          setStateToTriggerBackward();
        }
        break;
      case SHOT_STATE_TRIGGER_BACKWARD:
        if (millis() > (stateEnterTimestamp + durationForTriggerToMoveBackwardMs)) {
          if (shootCommandActive) {
            setStateToTriggerForward();
          } else {
            setStateToIdle();
          }
        }
        break;
    }
  }

  void shootNow() {
    shootCommandActive = true;
    if (currentState == SHOT_STATE_IDLE) {
      setStateToSpinFlywheel();
    }
  }

  void stopShootCommand() {
    shootCommandActive = false;
  }
};

// Create ServoController objects
ServoController elevationServo(3, 69, 167);
ServoController azimuthServo(5, 5, 175);

// Create TriggerAndESC object
TriggerAndESC triggerAndESC(6, 9, 990, 1150, 90, 10, 2000, 2000);

void setup() {
  elevationServo.attach();
  azimuthServo.attach();
  triggerAndESC.armESC();
  triggerAndESC.setStateToIdle();
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    int firstSeparatorIndex = command.indexOf(',');
    int secondSeparatorIndex = command.indexOf(',', firstSeparatorIndex + 1);

    if (firstSeparatorIndex > 0 && secondSeparatorIndex > 0) {
      int elevationRateCmd = command.substring(0, firstSeparatorIndex).toInt();
      int azimuthRateCmd = command.substring(firstSeparatorIndex + 1, secondSeparatorIndex).toInt();
      int shouldShoot = command.substring(secondSeparatorIndex + 1).toInt();

      // Debugging: Print received values
      Serial.print("Elevation: ");
      Serial.print(elevationRateCmd);
      Serial.print(" Azimuth: ");
      Serial.print(azimuthRateCmd);
      Serial.print(" Shoot: ");
      Serial.println(shouldShoot);

      elevationServo.setVelocity(elevationRateCmd);
      azimuthServo.setVelocity(azimuthRateCmd);

      if (shouldShoot == 1) {
        triggerAndESC.shootNow();
      } else {
        triggerAndESC.stopShootCommand();
      }
    }
  }

  elevationServo.update();
  azimuthServo.update();
  triggerAndESC.handleShot();
  delay(pidLoopDelayMs);
}
