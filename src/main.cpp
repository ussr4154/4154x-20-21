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
  int redLeft = 1
  int redRight = 2
  int redNothing = 3
  int blueLeft = -1
  int blueRight = -2
  int blueNothing = -3
  int skills = 0

  if(selector::auton == redLeft || selector::auton == redRight)
    redAuton();
  if(selector::auton == blueLeft || selector::auton == blueRight)
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
    setIntakeMotorsALL();
    setIntakeMotorsBottom();
    //Code for Intakes;


    pros::delay(10);
  }
}
