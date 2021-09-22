#include "main.h"
#include "autoSelect/selection.h"
using namespace okapi;

//Macros | Automatic Functions

void frontIntakesOn(){
  leftIntake = 120;
  rightIntake = 120;
  towerIntakeBottom = 60;
}

void sideIntakesOn(){
  leftIntake = 127;
  rightIntake = 127;
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

void allIntakesEject(){
  setIntakeEjector(127);
}

void deploy(){
  frontIntakesSpit();
  towerIntakeEjector(127);
  pros::delay(200);
  setIntakeEjector(0);
  pros::delay(10);
  towerIntakeEjector(127);
  pros::delay(150);
  allIntakesOff();

}

void score(){
  towerIntakeTop.move(-127);
  towerIntakeBottom.move(-127);
  pros::delay(50);
  towerIntakeTop.move(127);
  towerIntakeBottom.move(127);
  leftIntake.move(127);
  rightIntake.move(127);
  pros::delay(600);
  towerIntakeTop.move(0);
  towerIntakeBottom.move(0);
  leftIntake.move(0);
  rightIntake.move(0);
}

void scoreTwo(){
  towerIntakeTop.move(-127);
  towerIntakeBottom.move(-127);
  pros::delay(100);
  towerIntakeTop.move(127);
  towerIntakeBottom.move(127);;
  leftIntake.move(127);
  rightIntake.move(127);
  pros::delay(1100);
  towerIntakeTop.move(0);
  towerIntakeBottom.move(0);
  leftIntake.move(0);
  rightIntake.move(0);
}

void scoreNoDescore(int time){
  towerIntakeTop.move(-127);
  towerIntakeBottom.move(-127);
  pros::delay(50);
  towerIntakeTop.move(127);
  towerIntakeBottom.move(127);;
  leftIntake.move(-20);
  rightIntake.move(-20);
  pros::delay(time);
  towerIntakeTop.move(0);
  towerIntakeBottom.move(0);
  leftIntake.move(0);
  rightIntake.move(0);
}

void turnRight(int degree){

  float kP = 0.3;
  float kD = 0.000;
  int startHeading = inertial.get_heading();
  float prevError = 0;

  while(inertial.get_heading() < startHeading + degree){
    double currentHeading = inertial.get_heading();
    float error = (startHeading + degree) - currentHeading;
    //float integral = integral + error;
    float derivative = error - prevError;
    prevError = error;

    //if(error == 0)
    //  integral = 0;

    int power = error*kP + derivative*kD;

    frontLeft = power;
    frontRight = -power;
    backLeft = power;
    backRight = -power;

    pros::delay(10);
  }

  frontLeft = 0;
  frontRight = 0;
  backLeft = 0;
  backRight = 0;

}

void turnLeft(int degree){

  float kP = 0.000;
  float kI = 0.000;
  float kD = 0.000;
  int startHeading = inertial.get_heading();
  float prevError = 0;

  while(inertial.get_heading() > startHeading - degree){
    double currentHeading = inertial.get_heading();
    float error = (startHeading - degree) - currentHeading;
    float derivative = prevError - error;
    prevError = error;

    float power = error*kP + derivative*kD;

    frontLeft = -power;
    frontRight = power;
    backLeft = -power;
    backRight = power;

    pros::delay(10);
  }

  frontLeft = 0;
  frontRight = 0;
  backLeft = 0;
  backRight = 0;

}

void turnYawRight(int degree){

  float kP = 0.09;
  float kD = 0.000;
  frontRight.tare_position();
  float prevError = 0;

  while(frontRight.get_position() < degree){
    float error = degree - frontRight.get_position();
    //float integral = integral + error;
    float derivative = error - prevError;
    prevError = error;

    //if(error == 0)
    //  integral = 0;

    int power = error*kP - derivative*kD;

    frontLeft = power;
    frontRight = -power;
    backLeft = power;
    backRight = -power;

    pros::delay(10);
  }

  frontLeft = 0;
  frontRight = 0;
  backLeft = 0;
  backRight = 0;

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
        // blue gearset, tracking wheel diameter (2.75 in), track (7 in), and TPR (360)
        // 1 inch middle encoder distance, and 2.75 inch middle wheel diameter
        .withDimensions(AbstractMotor::gearset::blue, {{2.75_in, 6.5_in, 6_in, 2.75_in}, quadEncoderTPR})
        .withOdometry() // use the same scales as the chassis (above)
        .buildOdometry(); // Builds the odom chassis

  std::shared_ptr<AsyncMotionProfileController> profileController =
    AsyncMotionProfileControllerBuilder()
      .withLimits({
          10.0, // Maximum linear velocity of the Chassis in m/s
          3.0, // Maximum linear acceleration of the Chassis in m/s/s
          10.0 // Maximum linear jerk of the Chassis in m/s/s/s
          })
      .withOutput(chassis)
      .buildMotionProfileController();





//Autonomous Programs
void twoPointAuton(){

    deploy();

    frontIntakesSpit();

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

    chassis->setMaxVelocity(300);

    chassis->turnAngle(-200_deg);

    chassis->setMaxVelocity(600);

    frontIntakesOn();

    profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {3.7_ft, 0_ft, 0_deg}}, "goal2Score");
    profileController->setTarget("goal2Score");
    profileController->waitUntilSettled();

    pros::delay(500);

    scoreNoDescore(1000);

    scoreNoDescore(300);

    frontIntakesSpit();

    pros::delay(200);

    profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {4.55_ft, 0_ft, 0_deg}}, "goal2Backup");
    profileController->setTarget("goal2Backup",true);
    profileController->waitUntilSettled();

    allIntakesOff();

}

void homeRowAuton(){

  deploy();

  frontIntakesSpit();

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

  chassis->setMaxVelocity(300);

  chassis->turnAngle(-200_deg);

  chassis->setMaxVelocity(600);

  frontIntakesOn();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {3.7_ft, 0_ft, 0_deg}}, "goal2Score");
  profileController->setTarget("goal2Score");
  profileController->waitUntilSettled();

  pros::delay(500);

  scoreNoDescore(1000);

  scoreNoDescore(300);

  frontIntakesSpit();

  pros::delay(200);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {7.55_ft, 0_ft, 0_deg}}, "goal2Backup");
  profileController->setTarget("goal2Backup",true);
  profileController->waitUntilSettled();

  allIntakesOn();

  chassis->setMaxVelocity(300);

  chassis->turnAngle(-242_deg);

  chassis->setMaxVelocity(600);

  allIntakesOff();

  allIntakesOn();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {11.4_ft, 5_ft, 0_deg}}, "crossHomeZone");
  profileController->setTarget("crossHomeZone");
  profileController->waitUntilSettled();

  scoreNoDescore(900);

  scoreNoDescore(600);

  frontIntakesSpit();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {3.5_ft, 0_ft, 0_deg}}, "goal3Backup");
  profileController->setTarget("goal3Backup",true);
  profileController->waitUntilSettled();

}

void skillsFirstHalf(){

  deploy();

  frontIntakesSpit();

  pros::delay(300);

  chassis->setMaxVelocity(600);

  frontIntakesOn();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {3_ft, 0_ft, 0_deg}}, "firstGoalGrab");
  profileController->setTarget("firstGoalGrab");
  profileController->waitUntilSettled();

  chassis->setMaxVelocity(300);

  chassis->turnAngle(-341_deg);

  chassis->setMaxVelocity(600);

  allIntakesOff();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {3.6_ft, 0_ft, 0_deg}}, "firstGoalScore");
  profileController->setTarget("firstGoalScore");
  profileController->waitUntilSettled();

  scoreNoDescore(800);

  scoreNoDescore(600);

  frontIntakesSpit();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {4.4_ft, 0_ft, 0_deg}}, "firstGoalBackup");
  profileController->setTarget("firstGoalBackup",true);
  profileController->waitUntilSettled();

  allIntakesOn();

  pros::delay(300);

  chassis->setMaxVelocity(300);

  chassis->turnAngle(92_deg);

  chassis->setMaxVelocity(600);

  allIntakesOff();

  frontIntakesOn();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {3.65_ft, 0_ft, 0_deg}}, "secondGoalDrive");
  profileController->setTarget("secondGoalDrive");
  profileController->waitUntilSettled();

  chassis->setMaxVelocity(600);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {7.9_ft, 0_ft, 0_deg}}, "secondGoalCross");
  profileController->setTarget("secondGoalCross", true);
  profileController->waitUntilSettled();

  chassis->setMaxVelocity(270);

  chassis->turnAngle(-221_deg);

  chassis->setMaxVelocity(600);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {2.9_ft, 0_ft, 0_deg}}, "secondGoalScore");
  profileController->setTarget("secondGoalScore");

  allIntakesOn();

  pros::delay(150);

  allIntakesOff();

  profileController->waitUntilSettled();

  scoreNoDescore(800);

  scoreNoDescore(600);

  frontIntakesSpit();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {1.05_ft, 0_ft, 0_deg}}, "secondGoalBackup");
  profileController->setTarget("secondGoalBackup",true);
  profileController->waitUntilSettled();

  allIntakesOn();

  pros::delay(300);

  chassis->setMaxVelocity(200);

  chassis->turnAngle(-232_deg);

  allIntakesOff();

  frontIntakesOn();

  chassis->setMaxVelocity(400);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {10.1_ft, 0_ft, 0_deg}}, "thirdGoalDrive");
  profileController->setTarget("thirdGoalDrive");
  profileController->waitUntilSettled();

  chassis->setMaxVelocity(200);

  allIntakesOff();

  chassis->turnAngle(125_deg);

  chassis->setMaxVelocity(600);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {1.9_ft, 0_ft, 0_deg}}, "thirdGoalScore");
  profileController->setTarget("thirdGoalScore");
  profileController->waitUntilSettled();

  scoreNoDescore(1000);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {2.8_ft, 0_ft, 0_deg}}, "thirdGoalBack");
  profileController->setTarget("thirdGoalBack",true);
  profileController->waitUntilSettled();

  chassis->setMaxVelocity(200);

  chassis->turnAngle(-128_deg);

  chassis->setMaxVelocity(600);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {2.8_ft, 0_ft, 0_deg}}, "fourthGoalCross");
  profileController->setTarget("fourthGoalCross",true);
  profileController->waitUntilSettled();

  chassis->setMaxVelocity(300);

  chassis->turnAngle(-216_deg);

  chassis->setMaxVelocity(600);

  frontIntakesOn();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {3_ft, 0_ft, 0_deg}}, "fourthGoalAlign");
  profileController->setTarget("fourthGoalAlign",true);
  profileController->waitUntilSettled();

  allIntakesOff();
}

void skillsSecondHalf(){

    frontIntakesOn();

    profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {8.4_ft, 0_ft, 0_deg}}, "fourthGoalGrab");
    profileController->setTarget("fourthGoalGrab");
    profileController->waitUntilSettled();

    pros::delay(200);

    allIntakesEject();

    pros::delay(500);

    allIntakesOff();

    chassis->setMaxVelocity(300);

    chassis->turnAngle(221_deg);

    //chassis->turnAngle(90_deg);

    chassis->setMaxVelocity(600);

    allIntakesOff();

    frontIntakesOn();

    profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {3.9_ft, 0_ft, 0_deg}}, "fourthGoalScore");
    profileController->setTarget("fourthGoalScore");

    pros::delay(700);

    frontIntakesOn();

    pros::delay(500);

    profileController->waitUntilSettled();

    allIntakesOff();

    scoreNoDescore(800);

    profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {2.6_ft, 0_ft, 0_deg}}, "fourthGoalBack");
    profileController->setTarget("fourthGoalBack",true);
    profileController->waitUntilSettled();

    chassis->setMaxVelocity(200);

    chassis->turnAngle(-223_deg);

    chassis->setMaxVelocity(600);

    frontIntakesOn();

    profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {6_ft, 0_ft, 0_deg}}, "fifthGoalPickup");
    profileController->setTarget("fifthGoalPickup");
    profileController->waitUntilSettled();

    allIntakesOff();

    profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {2_ft, 0_ft, 0_deg}}, "fifthGoalLineup");
    profileController->setTarget("fifthGoalLineup", true);
    profileController->waitUntilSettled();

    chassis->setMaxVelocity(300);

    chassis->turnAngle(92_deg);

    chassis->setMaxVelocity(600);

    profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {4.9_ft, 0_ft, 0_deg}}, "fifthGoalScore");
    profileController->setTarget("fifthGoalScore");
    profileController->waitUntilSettled();

    score();
    /*
    profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {5_ft, 0_ft, 0_deg}}, "fifthGoalBack");
    profileController->setTarget("fifthGoalBack", true);
    profileController->waitUntilSettled();

    chassis->setMaxVelocity(300);

    chassis->turnAngle(-265_deg);
    */

}

void skillsAuton(){
  skillsFirstHalf();
  skillsSecondHalf();
}

void nothingToSeeHere(){

  deploy();

  pros::delay(300);

  chassis->setMaxVelocity(600);

  frontIntakesOn();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {3.2_ft, 0_ft, 0_deg}}, "firstGoalGrab");
  profileController->setTarget("firstGoalGrab");
  profileController->waitUntilSettled();

  chassis->setMaxVelocity(300);

  chassis->turnAngle(320_deg);

  chassis->setMaxVelocity(600);

  allIntakesOn();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {3.8_ft, 0_ft, 0_deg}}, "firstGoalScore");
  profileController->setTarget("firstGoalScore");
  profileController->waitUntilSettled();

  scoreNoDescore(800);

  scoreNoDescore(600);

  frontIntakesSpit();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {4.4_ft, 0_ft, 0_deg}}, "firstGoalBackup");
  profileController->setTarget("firstGoalBackup",true);
  profileController->waitUntilSettled();

  allIntakesOff();
}

void testerAuton(){

  turnYawRight(400);
}
