#include <Servo.h> // Include the Servo library

enum shotStates_e {
  SHOT_STATE_IDLE,
  SHOT_STATE_SPIN_FLYWHEEL,
  SHOT_STATE_TRIGGER_FARWARD
};

shotStates_e shotStates = SHOT_STATE_IDLE;


enum gunStates_e {
  GUN_STATE_SHUTDOWN = 0,   // pwm 0 to the gun and centrelize.                         set shot state to idle if not in idle
  GUN_STATE_DISARM = 1,     // pwm 0 to the gun and follow elevation azimuth commands.  set shot state to idle if not in idle
  GUN_STATE_ARM = 2,        // hold fire, arm esc if need to.                           set shot state to idle if not in idle
  GUN_STATE_FIRE = 3,       // regular shoot command
  GUN_STATE_RELOADED = 4    // not a mode, this just update the shotFiredCounter to 0.        set shot state to idle if not in idle
};

gunStates_e gunState = GUN_STATE_SHUTDOWN;
gunStates_e gunStatePrev = GUN_STATE_SHUTDOWN;


// general parameters
int maxTargetLossTime = 1000;
int shotFiredCounter = 0;
int maxShotsInMagazin = 3;

// shot mechanism parameters:
int durationForMotorsToSpeedUpMs = 1000;
int durationForTriggerToMoveFarwardMs = 700;
int triggerServoForwardPos = 70;
int triggerServoBackPos = 13;
int flywhillIdlePwm = 990;
int flywhillSpinPwm = 1150;

int pidLoopDelayMs = 50;

uint32_t shotStateMachineEnterStateTimeStamp;



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
  bool isBackwards;    // Flip the direction if assembled backwards

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
  void centrelize() {
    setVelocity(0);
    currentPos = this->minAngle + (this->maxAngle - this->minAngle)/2;
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
  if ((gunState == GUN_STATE_SHUTDOWN) || (gunState == GUN_STATE_DISARM)) {
    // Initialize ESC at minimum throttle
    esc.writeMicroseconds(0);
    Serial.println("esc.writeMicroseconds(0)");

    delay(2000); // Wait for a short period
    
    // Initialize ESC at minimum throttle
    esc.writeMicroseconds(flywhillIdlePwm);
    Serial.println("esc.writeMicroseconds(flywhillIdlePwm)");
    delay(5000); // Wait for a short period
    Serial.println("esc is armed");
  }
}

void disarmEscs(void) {
  esc.writeMicroseconds(0);
  if((gunStatePrev != GUN_STATE_SHUTDOWN) || (gunState != GUN_STATE_DISARM)) {
    Serial.println("esc is disarmed");
  }
}


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
  shotFiredCounter ++;
}

void setShotStateToIdle(void) {
  triggerServo.write(triggerServoBackPos);
  esc.write(flywhillIdlePwm);

  shotStateMachineEnterStateTimeStamp = millis();
  shotStates = SHOT_STATE_IDLE;
}


void shootNow(void) {
  if (shotStates == SHOT_STATE_IDLE){
    setShotStateToSpinFlywheel();
  }
}

void handleShot(void) {
  if (gunState == GUN_STATE_DISARM || gunState == GUN_STATE_DISARM){
    setShotStateToIdle();
    return;
  }

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

static void setGunStateToShutdown(void) {
    disarmEscs();
    elevationServo.centrelize();
    azimuthServo.centrelize();
    gunState = GUN_STATE_SHUTDOWN;
}

void setup() {
  Serial.begin(9600); // For debugging
  elevationServo.attach();
  azimuthServo.attach();
  triggerServo.attach(6);
  esc.attach(9);
  shotFiredCounter = 0;
  setGunStateToShutdown();
}

void loop() {
  // Check if data is available on the serial port
  static int32_t lastRecivedCommandTimestamp = 0;
  if (Serial.available() > 0) {
    lastRecivedCommandTimestamp = millis();

    // Read a string from the serial port
    String command = Serial.readStringUntil('\n');
    command.trim();

    // Parse the command to extract elevation, azimuth velocities, and shoot command
    int firstSeparatorIndex = command.indexOf(',');
    int secondSeparatorIndex = command.indexOf(',', firstSeparatorIndex + 1);

    if (firstSeparatorIndex > 0 && secondSeparatorIndex > 0) {


      int elevationRateCmd = command.substring(0, firstSeparatorIndex).toInt();
      int azimuthRateCmd = command.substring(firstSeparatorIndex + 1, secondSeparatorIndex).toInt();
      int gunCommand = command.substring(secondSeparatorIndex + 1).toInt();


      // Set velocities
      elevationServo.setVelocity(elevationRateCmd);
      azimuthServo.setVelocity(azimuthRateCmd);

        // Handle shoot command
      if (gunCommand == GUN_STATE_RELOADED) {
        shotFiredCounter = 0;
      }
      if (shotFiredCounter > maxShotsInMagazin){
        setGunStateToShutdown();
      }
      else {
        switch (gunCommand) {
          case GUN_STATE_SHUTDOWN:
            Serial.print("4 ");
            setGunStateToShutdown();
            break;
          case GUN_STATE_DISARM:
            Serial.print("5 ");
            disarmEscs();
            gunState = GUN_STATE_DISARM;
            break;
          case GUN_STATE_ARM:
            Serial.print("6 ");
            armEscs();
            gunState = GUN_STATE_ARM;
            break;
          case GUN_STATE_FIRE:
            Serial.print("7 ");
            shootNow();
            gunState = GUN_STATE_FIRE;
            break;
          case GUN_STATE_RELOADED:
            Serial.print("8 ");
            default:
            break;
        }
      }
    }
  }
  else if ((millis() - lastRecivedCommandTimestamp) > maxTargetLossTime) {
      Serial.println("9 ");
      setGunStateToShutdown();
  }

  // Update the servos
  elevationServo.update();
  azimuthServo.update();
  handleShot();
  gunStatePrev = gunState;
  delay(pidLoopDelayMs); // Delay for smoother updates
}
