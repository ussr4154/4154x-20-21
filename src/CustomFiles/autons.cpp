#include "main.h"
#include "autoSelect/selection.h"
using namespace okapi;

//Macros | Automatic Functions

void deploy(){

  towerIntakeTop.move(-127);
  towerIntakeBottom.move(-127);
  pros::delay(300);
  towerIntakeTop.move(0);
  towerIntakeBottom.move(0);

}

void frontIntakesOn(){
  leftIntake = 127;
  rightIntake = 127;
  towerIntakeBottom = 127;
}

void allIntakesOff(){
  leftIntake.move(0);
  rightIntake.move(0);
  towerIntakeTop.move(0);
  towerIntakeBottom.move(0);
}

void frontIntakesSpit(){
  leftIntake.move(-127);
  rightIntake.move(-127);
  towerIntakeBottom.move(-127);
}

void score(){
  towerIntakeTop.move(127);
  towerIntakeBottom.move(127);;
  leftIntake.move(127);
  rightIntake.move(127);
  pros::delay(1000);
  towerIntakeTop.move(0);
  towerIntakeBottom.move(0);
  leftIntake.move(0);
  rightIntake.move(0);
}


//Chassis Odometry & Motion Profiling Builders
  std::shared_ptr<OdomChassisController> chassis =
    ChassisControllerBuilder()
      .withMotors({5,4},{7,6}) // Left = 5,4 | Right = 7,6
      .withDimensions(AbstractMotor::gearset::blue, {{4_in, 12.5_in}, imev5BlueTPR})
        /*.withSensors(
          ADIEncoder{'G', 'H'}, //Left Encoder = A,B (Reverse)
          ADIEncoder{'E', 'F',true} // Right Encoder = C,D // Middle Encoder = E,F
        )*/
        // green gearset, tracking wheel diameter (2.75 in), track (7 in), and TPR (360)
        // 1 inch middle encoder distance, and 2.75 inch middle wheel diameter
        .withDimensions(AbstractMotor::gearset::blue, {{2.75_in, 6.5_in, 6_in, 2.75_in}, quadEncoderTPR})
        .withOdometry() // use the same scales as the chassis (above)
        .buildOdometry(); // Builds the odom chassis

  std::shared_ptr<AsyncMotionProfileController> profileController =
    AsyncMotionProfileControllerBuilder()
      .withLimits({
          6.0, // Maximum linear velocity of the Chassis in m/s
          5.0, // Maximum linear acceleration of the Chassis in m/s/s
          7.0 // Maximum linear jerk of the Chassis in m/s/s/s
          })
      .withOutput(chassis)
      .buildMotionProfileController();





//Autonomous Programs
void twoPointAuton(){

    deploy();

    chassis->setState({0_ft,0_ft});

    frontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    frontRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    backLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    backRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    chassis->setMaxVelocity(600);

    profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {4.5_ft, 0_ft, 0_deg}}, "goal2Mid");
    profileController->setTarget("goal2Mid");
    profileController->waitUntilSettled();

    chassis->turnAngle(-175_deg);

    frontIntakesOn();

    profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {3.25_ft, 0_ft, 0_deg}}, "goal2Score");
    profileController->setTarget("goal2Score");
    profileController->waitUntilSettled();

    score();

    profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {3.5_ft, 0_ft, 0_deg}}, "goal2Backup");
    profileController->setTarget("goal2Backup",true);
    profileController->waitUntilSettled();

    frontIntakesSpit();

    pros::delay(500);

    allIntakesOff();

}

void homeRowAuton(){

  deploy();

  chassis->setState({0_ft,0_ft});

  frontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  frontRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  backLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  backRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  chassis->setMaxVelocity(600);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {4.5_ft, 0_ft, 0_deg}}, "goal2Mid");
  profileController->setTarget("goal2Mid");
  profileController->waitUntilSettled();

  chassis->turnAngle(-175_deg);

  frontIntakesOn();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {3.25_ft, 0_ft, 0_deg}}, "goal2Score");
  profileController->setTarget("goal2Score");
  profileController->waitUntilSettled();

  score();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {3.5_ft, 0_ft, 0_deg}}, "goal2Backup");
  profileController->setTarget("goal2Backup",true);
  profileController->waitUntilSettled();

  frontIntakesSpit();

  pros::delay(500);

  chassis->turnAngle(-283_deg);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {11.5_ft, 0_ft, 0_deg}}, "crossHomeZone");
  profileController->setTarget("crossHomeZone");
  profileController->waitUntilSettled();

  chassis->turnAngle(115_deg);

  frontIntakesOn();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {3.75_ft, 0_ft, 0_deg}}, "goal3Score");
  profileController->setTarget("goal3Score");
  profileController->waitUntilSettled();

  score();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {3.5_ft, 0_ft, 0_deg}}, "goal3Backup");
  profileController->setTarget("goal3Backup",true);
  profileController->waitUntilSettled();

}

void skillsAuton(){

  chassis->setMaxVelocity(600);

  frontIntakesOn();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {8.5_ft, 0_ft, 0_deg}}, "firstGoalGrab");
  profileController->setTarget("firstGoalGrab");
  profileController->waitUntilSettled();

  allIntakesOff();

  chassis->turnAngle(276_deg);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {3.95_ft, 0_ft, 0_deg}}, "firstGoalScore");
  profileController->setTarget("firstGoalScore");
  profileController->waitUntilSettled();

  score();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {1_ft, 0_ft, 0_deg}}, "firstGoalBackup");
  profileController->setTarget("firstGoalBackup",true);
  profileController->waitUntilSettled();

  chassis->turnAngle(-195_deg);

}

void nothingToSeeHere(){

}
