#include "main.h"

//Helpers
void setIntakeBottom(int power){
  leftIntake = power;
  rightIntake = power;
}

void setIntakeALL(int power){
  leftIntake = power;
  rightIntake = power;
  towerIntake = power;
}

//Driver Control
void setIntakeMotorsBottom(){
  if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
      setIntakeBottom(127);
} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      setIntakeBottom(-127);
}
}

void setIntakeMotorsALL(){
  if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
      setIntakeALL(127);
} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      setIntakeALL(-127);
}
}
