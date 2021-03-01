#include "main.h"
#include "autoSelect/selection.h"
#define redHomeRow 1
#define redTwoPoint 2
#define redNothing 3
#define blueHomeRow -1
#define blueTwoPoint -2
#define blueNothing 3
#define skills 0

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

void disabled(){

}

void competition_initialize() {
  selector::init();
}


void autonomous() {
 //Selector | If no program is chosen, defaults to skills
  if(selector::auton == 1)
    homeRowAuton();
  if(selector::auton == -1)
    homeRowAuton();
  if(selector::auton == 2)
    twoPointAuton();
  if (selector::auton == -2)
    twoPointAuton();
  if(selector::auton == -3)
    nothingToSeeHere();
  if(selector::auton == 3)
    nothingToSeeHere();
  if(selector::auton == 0)
    skillsAuton();
}

void opcontrol() {
  while(true){
    ballPresent();
    //Code for Drivetrain
    setDriveMotors();
    //Code for Intakes;
    setIntakeMotors();
    pros::delay(10);
  }
}
