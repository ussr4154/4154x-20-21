#include "main.h"
#define lineSensorThreshold 2850

//Helpers
void setIntakeFront(int power){
  rightIntake = power;
  leftIntake = power;
  towerIntakeBottom = (power/3);
  towerIntakeTop = 0;

}

void setIntakeShoot(int power){
  towerIntakeBottom = power;
  towerIntakeTop = power;
  rightIntake = 0;
  leftIntake = 0;
}

void setIntakeALL(int power){
  leftIntake = power;
  rightIntake = power;
  towerIntakeTop = power;
  towerIntakeBottom = power;
}

void setIntakeEjector(int power){

  leftIntake = power;
  rightIntake = power;
  towerIntakeBottom = power;
  towerIntakeTop = - power;
}

void setIntakeIndex(int power){

  if(ballPresent()){
    towerIntakeTop = 0;
    towerIntakeBottom = 0;
    leftIntake = power;
    rightIntake = power;
  }
  else{
    leftIntake = power;
    rightIntake = power;
    towerIntakeBottom = power;
    towerIntakeTop = - 0;
  }

}

bool ballPresent(){

    bool ball = false;
    if(topBallSensor.get_value() <= lineSensorThreshold){
      bool ball = true;
    }
    return ball;
}

//Driver Control
void setIntakeMotors(){

  if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
      setIntakeFront(127);
}
  else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      setIntakeEjector(127);
}
  else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
      setIntakeALL(127);
}
  else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      setIntakeALL(-127);
}
  else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
      setIntakeShoot(127);
  }
  else {
    setIntakeALL(0);
  }
}
