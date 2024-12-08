#include <Arduino.h>

void setup() {
  Serial.begin(9600); // Communication with BLHeli Configurator
  Serial1.begin(115200); // Communication with ESC (adjust if needed)
}

void loop() {
  if (Serial.available()) {
    Serial1.write(Serial.read());
  }
  if (Serial1.available()) {
    Serial.write(Serial1.read());
  }
}