#include "main.h"

void initialize() {

}

void disabled() {}


void competition_initialize() {}


void autonomous() {}


void opcontrol() {
  while(true){
    //Code for Drivetrain
    setDriveMotors();
    setIntakeMotorsALL();
    setIntakeMotorsBottom();
    //Code for Intakes;


    pros::delay(10);
  }
}
