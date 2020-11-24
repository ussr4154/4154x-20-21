#include "main.h"
#include "autoSelect/selection.h"

void initialize() {

  //Brakes
  frontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  frontRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  backLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  backRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  leftIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rightIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  towerIntakeTop.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  towerIntakeBottom.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

}

void disabled() {}

void competition_initialize() {
  selector::init();
}


void autonomous() {
  redAuton();

}
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
