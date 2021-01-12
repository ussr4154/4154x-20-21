#include "main.h"

//Helpers
void setIntakeCentral(int power){
  towerIntakeTop = power;
  towerIntakeBottom = power;
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

//Driver Control
void setIntakeMotors(){
  /*if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
      setIntakeCentral(127);
}*/ if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      setIntakeEjector(127);
}
  else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
      setIntakeALL(127);
}
  else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      setIntakeALL(-127);
}
  else {
    setIntakeALL(0);
  }
}
