// #include "ServoController.h"
#include "MotorController.h"
#include <IRremote.hpp>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
#define IR_RECEIVE_PIN 8

MotorController motor1(1);
MotorController motor2(2);

float setpoint = 2;      // Desired target value 
float input = 0;         // Current value from sensor
float output = 0;        // Output to actuator 

// float Kp = 1.5;
// float Ki = 1;
// float Kd = 0.1;

float error;
float lastError = 0;
float integral = 0;
float derivative;

unsigned long lastTime;

void setup() {
  // Serial.begin(9600); // // Establish serial communication
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver
  Wire.begin();
  mpu.initialize();
  // if (!mpu.testConnection()) {
  //   Serial.println("MPU6050 connection failed!");
  //   while (1);
  // }
  // Serial.println("MPU6050 ready");
  lastTime = millis();
}
float getTiltAngle(float gyroRate, float deltaTime) {
  static float angle = 0;
  static float alpha = 0.96;

  int16_t accXraw = mpu.getAccelerationX();
  int16_t accZraw = mpu.getAccelerationZ();

  float accX = accXraw / 16384.0;
  float accZ = accZraw / 16384.0;

  float accAngle = atan2(accX, accZ) * 180 / PI;

  angle = alpha * (angle + gyroRate * deltaTime) + (1 - alpha) * accAngle;
  // Serial.print("Angle = ");
  // Serial.print(angle);  // Kp - Ki - Kd
  return angle;
}

int computePid(float Kp, float Ki, float Kd) {
  static unsigned long lastTime = millis();
  unsigned long now = millis();
  float deltaTime = (now - lastTime) / 1000.0;
  lastTime = now;

  int16_t gyroXraw = mpu.getRotationX();
  float gyroRate = gyroXraw / 131.0;

  input = getTiltAngle(gyroRate, deltaTime);

  error = setpoint - input;
  
    integral += error * deltaTime;
    derivative = -gyroRate; // use gyro as angular velocity for Kd

    output = Kp * error + Ki * integral + Kd * derivative;
    output = constrain(output, -255, 255);
  lastError = error;

  return output;
}

void loop() {

  
    if (IrReceiver.decode()) {
    uint32_t rawCode = IrReceiver.decodedIRData.decodedRawData;

    Serial.println(rawCode, HEX); // just to see it still

    // Example: check if "Power" button was pressed
    if (rawCode == 0x9F600707) { // forward
      motor1.moveMotor(1, 255);
      motor2.moveMotor(1, 255);
      // do something
    }
    else if (rawCode == 0x9E610707) { // backward
      motor1.moveMotor(0, 255);
      motor2.moveMotor(0, 255);
    }
    else if(rawCode == 0x9D620707){ // right
      motor1.moveMotor(0, 255);
      motor2.moveMotor(1, 255);
    }
    else if(rawCode == 0x9A650707){ // left
      motor1.moveMotor(1, 255);
      motor2.moveMotor(0, 255);
    }
    else { // left
      motor1.moveMotor(1, 0);
      motor2.moveMotor(1, 0);
    }
    IrReceiver.resume(); // Ready for next signal
  }
  int nSpeed = computePid(160 , 0 , 100); // Kp - Ki - Kd
  int direction = 1;
  if(nSpeed < 0)
  {
    direction = 0;
    nSpeed = abs(nSpeed); 
  }
  motor1.moveMotor(direction, nSpeed);
  motor2.moveMotor(direction, nSpeed);
  
  // Serial.print(" | ");
  // Serial.print("Output = ");
  // Serial.println(nSpeed);  // Kp - Ki - Kd
}