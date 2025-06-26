// #include "ServoController.h"
#include "MotorController.h"
#include <IRremote.hpp>

#define IR_RECEIVE_PIN 8
MotorController motor1(1);
MotorController motor2(2);

void setup() {
  Serial.begin(115200); // // Establish serial communication
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver
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
}