// #include "ServoController.h"
#include "MotorController.h"


MotorController motor1(1);
MotorController motor2(2);

void setup() {
  // nothing needed; constructor handles pinMode
}

void loop() {
  motor1.moveMotor(1, 255);
  delay(1000);
  motor1.moveMotor(1, 0);
  delay(1000);
  motor2.moveMotor(1, 255);
  delay(1000);
  motor2.moveMotor(1, 0);
  delay(1000);
}