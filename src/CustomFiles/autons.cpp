#include "main.h"
#include "autoSelect/selection.h"
using namespace okapi;

//Macros | Automatic Functions

void deploy(){

  towerIntakeTop.move(-127);
  towerIntakeBottom.move(-127);
  pros::delay(500);
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

void allIntakesOn(){
  leftIntake.move(127);
  rightIntake.move(127);
  towerIntakeTop.move(127);
  towerIntakeBottom.move(127);
}

void frontIntakesSpit(){
  leftIntake.move(-127);
  rightIntake.move(-127);
  towerIntakeBottom.move(-127);
}

void allIntakesSpit(){
  leftIntake.move(-127);
  rightIntake.move(-127);
  towerIntakeTop.move(-127);
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

void scoreTwo(){
  towerIntakeTop.move(127);
  towerIntakeBottom.move(127);;
  leftIntake.move(127);
  rightIntake.move(127);
  pros::delay(1500);
  towerIntakeTop.move(0);
  towerIntakeBottom.move(0);
  leftIntake.move(0);
  rightIntake.move(0);
}

void scoreNoDescore(int time){
  towerIntakeTop.move(127);
  towerIntakeBottom.move(127);;
  leftIntake.move(-127);
  rightIntake.move(-127);
  pros::delay(time);
  towerIntakeTop.move(0);
  towerIntakeBottom.move(0);
  leftIntake.move(0);
  rightIntake.move(0);
}

void turnRight(){}

void turnLeft(){}
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

  chassis->turnAngle(-170_deg);

  frontIntakesOn();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {3.45_ft, 0_ft, 0_deg}}, "goal2Score");
  profileController->setTarget("goal2Score");
  profileController->waitUntilSettled();

  score();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {7.35_ft, 0_ft, 0_deg}}, "goal2Backup");
  profileController->setTarget("goal2Backup",true);
  profileController->waitUntilSettled();

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

  chassis->turnAngle(-170_deg);

  frontIntakesOn();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {3.45_ft, 0_ft, 0_deg}}, "goal2Score");
  profileController->setTarget("goal2Score");
  profileController->waitUntilSettled();

  score();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {7.35_ft, 0_ft, 0_deg}}, "goal2Backup");
  profileController->setTarget("goal2Backup",true);
  profileController->waitUntilSettled();

  allIntakesOn();

  pros::delay(500);

  chassis->turnAngle(-214_deg);

  allIntakesOff();

  frontIntakesOn();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {13.65_ft, 0_ft, 0_deg}}, "crossHomeZone");
  profileController->setTarget("crossHomeZone");
  profileController->waitUntilSettled();

  scoreTwo();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {3.5_ft, 0_ft, 0_deg}}, "goal3Backup");
  profileController->setTarget("goal3Backup",true);
  profileController->waitUntilSettled();

}

void skillsAuton(){

  deploy();

  chassis->setMaxVelocity(600);

  frontIntakesOn();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {8.3_ft, 0_ft, 0_deg}}, "firstGoalGrab");
  profileController->setTarget("firstGoalGrab");
  profileController->waitUntilSettled();

  allIntakesOff();

  chassis->turnAngle(275_deg);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {3.95_ft, 0_ft, 0_deg}}, "firstGoalScore");
  profileController->setTarget("firstGoalScore");
  profileController->waitUntilSettled();

  scoreNoDescore(2000);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {0.8_ft, 0_ft, 0_deg}}, "firstGoalBackup");
  profileController->setTarget("firstGoalBackup",true);
  profileController->waitUntilSettled();

  allIntakesOn();

  pros::delay(300);

  chassis->turnAngle(-180_deg);

  allIntakesOff();

  frontIntakesOn();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {6.4_ft, 0_ft, 0_deg}}, "secondGoalDrive");
  profileController->setTarget("secondGoalDrive");
  profileController->waitUntilSettled();

  allIntakesOff();

  chassis->turnAngle(120_deg);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {2_ft, 0_ft, 0_deg}}, "secondGoalScore");
  profileController->setTarget("secondGoalScore");
  profileController->waitUntilSettled();

  scoreNoDescore(1000);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {2.6_ft, 0_ft, 0_deg}}, "secondGoalBack");
  profileController->setTarget("secondGoalBack",true);
  profileController->waitUntilSettled();

  chassis->turnAngle(-110_deg);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {2.9_ft, 0_ft, 0_deg}}, "thirdGoalCross");
  profileController->setTarget("thirdGoalCross",true);
  profileController->waitUntilSettled();

  chassis->turnAngle(-180_deg);

  frontIntakesOn();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {3.8_ft, 0_ft, 0_deg}}, "thirdGoalAlign");
  profileController->setTarget("thirdGoalAlign",true);
  profileController->waitUntilSettled();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {9_ft, 0_ft, 0_deg}}, "thirdGoalGrab");
  profileController->setTarget("thirdGoalGrab");
  profileController->waitUntilSettled();

  chassis->setMaxVelocity(400);

  chassis->turnAngle(180_deg);

  chassis->turnAngle(70_deg);

  chassis->setMaxVelocity(600);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {5_ft, 0_ft, 0_deg}}, "thirdGoalScore");
  profileController->setTarget("thirdGoalScore");
  profileController->waitUntilSettled();

  scoreNoDescore(2000);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {2_ft, 0_ft, 0_deg}}, "thirdGoalBack");
  profileController->setTarget("thirdGoalBack",true);
  profileController->waitUntilSettled();




}

void nothingToSeeHere(){

}
