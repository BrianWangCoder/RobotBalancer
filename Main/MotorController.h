#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H
class MotorController {
  private:
    int directionPin2;
    int directionPin1;
    int speedPin;

  public:
    MotorController(int motorNumber) {
      switch(motorNumber){
        case 1:
          directionPin1 = 3;
          directionPin2 = 2;
          speedPin = 9;
          pinMode(directionPin1,OUTPUT);
          pinMode(directionPin2,OUTPUT);
          pinMode(speedPin,OUTPUT);
        break;
        case 2:
          directionPin1 = 4;
          directionPin2 = 5;
          speedPin = 10;
          pinMode(directionPin1,OUTPUT);
          pinMode(directionPin2,OUTPUT);
          pinMode(speedPin,OUTPUT);
        break;
      }

    }

    void moveMotor(int direction,int speed){
      digitalWrite(directionPin1,direction);
      digitalWrite(directionPin2,!direction);
      analogWrite(speedPin,speed);
    }
};
#endif