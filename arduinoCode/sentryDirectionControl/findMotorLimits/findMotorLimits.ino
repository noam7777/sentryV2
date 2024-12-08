#include <Servo.h> // Include the Servo library
Servo servo;

void setup() {
    servo.attach(5);

}

void loop() {
  // put your main code here, to run repeatedly:
    static bool once = false;
    servo.write(69); // Set initial position to the midpoint
    delay(2000);
    servo.write(167); // Set initial position to the midpoint
    delay(2000);



}
