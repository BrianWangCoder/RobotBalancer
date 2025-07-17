#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H
class MotorController {
  private:
    int directionPin;
    int speedPin;

  public:
    MotorController(int motorNumber) {
      switch(motorNumber){
        case 1:
          directionPin = 4;
          speedPin = 5;
          pinMode(directionPin,OUTPUT);
          pinMode(speedPin,OUTPUT);
        break;
        case 2:
          directionPin = 7;
          speedPin = 6;
          pinMode(directionPin,OUTPUT);
          pinMode(speedPin,OUTPUT);
        break;
      }

    }

    void moveMotor(int direction,int speed){
      digitalWrite(directionPin,direction);
      analogWrite(speedPin,speed);
    }
};
#endif