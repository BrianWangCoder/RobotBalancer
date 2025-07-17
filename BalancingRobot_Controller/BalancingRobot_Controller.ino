#include "MotorController.h"
#include <IRremote.hpp>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>

MPU6050 mpu;

#define IR_RECEIVE_PIN 8

MotorController motor1(1);
MotorController motor2(2);

float setpoint = 3; 
float input = 0;
float output = 0;

float error;
float lastError = 0;
float integral = 0;
float derivative;

unsigned long lastTime;

Quaternion q;
VectorFloat gravity;
float ypr[3];

uint8_t fifoBuffer[64];      
uint16_t packetSize;        

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  Serial.println("Initializing DMP...");
  if (mpu.dmpInitialize() != 0) {
    Serial.println("DMP init failed");
    while (1);
  }

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();

  Serial.println("DMP ready!");
  lastTime = millis();
}

float getTiltAngle() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    float pitch = ypr[1] * 180 / M_PI; // Convert to degrees
    return pitch;
  }
  return input; 
}

int computePid(float Kp, float Ki, float Kd) {
  unsigned long now = millis();
  float deltaTime = (now - lastTime) / 1000.0;
  lastTime = now;

  input = getTiltAngle();
  error = setpoint - input;

  if (abs(error) < 1) return 0;

  integral += error * deltaTime;
  derivative = (error - lastError) / deltaTime;
  lastError = error;

  output = Kp * error + Ki * integral + Kd * derivative;
  output = constrain(output, -255, 255);

  return output;
}

void loop() {
  int nSpeed = computePid(24 , 17, 2.7); // Kp = 23 , Ki = 17 , Kd = 2-3  = 2.7

  int direction = 1;
  if (nSpeed < 0) {
    direction = 0;
    nSpeed = abs(nSpeed);
  }

  motor1.moveMotor(direction, nSpeed);
  motor2.moveMotor(direction, nSpeed);

  Serial.print("Pitch: ");
  Serial.print(input);
  Serial.print(" | Output: ");
  Serial.println(nSpeed);

  delay(5);
}
