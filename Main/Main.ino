// #include "ServoController.h"
#include "MotorController.h"
#include <IRremote.hpp>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
#define IR_RECEIVE_PIN 8

MotorController motor1(1);
MotorController motor2(2);

float setpoint = -7;      // Desired target value (e.g., angle, speed)
float input = 0;         // Current value from sensor
float output = 0;        // Output to actuator (e.g., motor power)

// float Kp = 1.5;
// float Ki = 1;
// float Kd = 0.1;

float error;
float lastError = 0;
float integral = 0;
float derivative;

unsigned long lastTime;

void setup() {
  Serial.begin(9600); // // Establish serial communication
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  Serial.println("MPU6050 ready");
  lastTime = millis();
}
float getTiltAngle() {
  static unsigned long lastTime = millis();
  static float angle = 0;

  float alpha = 0.96;
  float accX, accZ, gyroX;
  float accAngle, elapsedTime;

  // Time step
  unsigned long currentTime = millis();
  elapsedTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // Read raw values
  int16_t accXraw = mpu.getAccelerationX();
  int16_t accZraw = mpu.getAccelerationZ();
  int16_t gyroXraw = mpu.getRotationX();

  // Convert to real-world units
  accX = accXraw / 16384.0;
  accZ = accZraw / 16384.0;
  gyroX = gyroXraw / 131.0;

  // Calculate tilt angle from accelerometer
  accAngle = atan2(accX, accZ) * 180 / PI;

  // Complementary filter
  angle = alpha * (angle + gyroX * elapsedTime) + (1 - alpha) * accAngle;

  return angle;
}
int computePid(float Kp,float Ki, float Kd){
  input = getTiltAngle();
  unsigned long now = millis();
  float deltaTime = (now - lastTime) / 1000.0; // convert ms to seconds
  lastTime = now;
  error = setpoint - input;
  integral += error * deltaTime;
  derivative = (error - lastError) / deltaTime;

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
  int nSpeed = computePid(14 , 13.4 , 15); // Kp - Ki - Kd
  int direction = 1;
  if(nSpeed < 0)
  {
    direction = 0;
    nSpeed = abs(nSpeed);
  }
  motor1.moveMotor(direction, nSpeed);
  motor2.moveMotor(direction, nSpeed);
  Serial.print("Angle = ");
  Serial.print(getTiltAngle());  // Kp - Ki - Kd
  Serial.print(" | ");
  Serial.print("Output = ");
  Serial.println(nSpeed);  // Kp - Ki - Kd
}