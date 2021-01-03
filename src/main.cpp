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

  //Un-Note for competiton ONLY
  /**
  int redHomeRow = 1
  int redMiddleTower = 2
  int redNothing = 3
  int blueHomeRow = -1
  int blueMiddleTower = -2
  int blueNothing = -3
  int skills = 0

  if(selector::auton == redHomeRow || selector::auton == redMiddleTower)
    redAuton();
  if(selector::auton == blueHomeRow || selector::auton == blueMiddleTower)
    blueAuton();
  if(selector::auton == blueNothing || selector::auton == redNothing)
    nothingToSeeHere();
  if(selector::auton == skills)
    skillsAuton();
  **/

}
void opcontrol() {
  while(true){
    //Code for Drivetrain
    setDriveMotors();
    //Code for Intakes;
    setIntakeMotorsALL();
    setIntakeMotorsCentral();

    pros::delay(10);
  }
}
