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
  leftIntake = 120;
  rightIntake = 120;
  towerIntakeBottom = 90;
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
  leftIntake.move(-40);
  rightIntake.move(-40);
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
    {{0_ft, 0_ft, 0_deg}, {5_ft, 0_ft, 0_deg}}, "goal2Mid");
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

  scoreNoDescore(1000);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {7.35_ft, 0_ft, 0_deg}}, "goal2Backup");
  profileController->setTarget("goal2Backup",true);
  profileController->waitUntilSettled();

  allIntakesOn();

  pros::delay(500);

  chassis->turnAngle(-222_deg);

  allIntakesOff();

  frontIntakesOn();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {13.65_ft, 0_ft, 0_deg}}, "crossHomeZone");
  profileController->setTarget("crossHomeZone");
  profileController->waitUntilSettled();

  scoreNoDescore(2000);

  frontIntakesSpit();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {3.5_ft, 0_ft, 0_deg}}, "goal3Backup");
  profileController->setTarget("goal3Backup",true);
  profileController->waitUntilSettled();

}

void skillsFirstHalf(){

  deploy();

  chassis->setMaxVelocity(600);

  frontIntakesOn();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {8.4_ft, 0_ft, 0_deg}}, "firstGoalGrab");
  profileController->setTarget("firstGoalGrab");
  profileController->waitUntilSettled();

  chassis->setMaxVelocity(200);

  chassis->turnAngle(314_deg);

  chassis->setMaxVelocity(600);

  allIntakesOff();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {4.1_ft, 0_ft, 0_deg}}, "firstGoalScore");
  profileController->setTarget("firstGoalScore");
  profileController->waitUntilSettled();

  scoreNoDescore(700);

  pros::delay(500);

  scoreNoDescore(1000);

  frontIntakesSpit();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {1_ft, 0_ft, 0_deg}}, "firstGoalBackup");
  profileController->setTarget("firstGoalBackup",true);
  profileController->waitUntilSettled();

  allIntakesOn();

  pros::delay(300);

  chassis->setMaxVelocity(200);

  chassis->turnAngle(-232_deg);

  chassis->setMaxVelocity(600);

  allIntakesOff();

  frontIntakesOn();

  chassis->setMaxVelocity(400);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {10.4_ft, 0_ft, 0_deg}}, "secondGoalDrive");
  profileController->setTarget("secondGoalDrive");
  profileController->waitUntilSettled();

  chassis->setMaxVelocity(200);

  allIntakesOff();

  chassis->turnAngle(125_deg);

  chassis->setMaxVelocity(600);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {1.6_ft, 0_ft, 0_deg}}, "secondGoalScore");
  profileController->setTarget("secondGoalScore");
  profileController->waitUntilSettled();

  scoreNoDescore(1000);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {2.8_ft, 0_ft, 0_deg}}, "secondGoalBack");
  profileController->setTarget("secondGoalBack",true);
  profileController->waitUntilSettled();

  chassis->setMaxVelocity(200);

  chassis->turnAngle(-125_deg);

  chassis->setMaxVelocity(600);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {2.9_ft, 0_ft, 0_deg}}, "thirdGoalCross");
  profileController->setTarget("thirdGoalCross",true);
  profileController->waitUntilSettled();

  chassis->setMaxVelocity(200);

  chassis->turnAngle(-225_deg);

  chassis->setMaxVelocity(600);

  frontIntakesOn();

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {2.9_ft, 0_ft, 0_deg}}, "thirdGoalAlign");
  profileController->setTarget("thirdGoalAlign",true);
  profileController->waitUntilSettled();

  allIntakesOff();
}

void skillsSecondHalf(){

    frontIntakesOn();

    profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {8.2_ft, 0_ft, 0_deg}}, "thirdGoalGrab");
    profileController->setTarget("thirdGoalGrab");
    profileController->waitUntilSettled();

    pros::delay(200);

    allIntakesOff();

    chassis->setMaxVelocity(200);

    chassis->turnAngle(223_deg);

    //chassis->turnAngle(90_deg);

    chassis->setMaxVelocity(600);

    allIntakesOff();

    frontIntakesOn();

    profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {3.6_ft, 0_ft, 0_deg}}, "thirdGoalScore");
    profileController->setTarget("thirdGoalScore");

    pros::delay(700);

    frontIntakesOn();

    pros::delay(500);

    profileController->waitUntilSettled();

    allIntakesOff();

    scoreNoDescore(2000);

    profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {2.7_ft, 0_ft, 0_deg}}, "thirdGoalBack");
    profileController->setTarget("thirdGoalBack",true);
    profileController->waitUntilSettled();

    pros::delay(2500);

    chassis->setMaxVelocity(200);

    chassis->turnAngle(-223_deg);

    chassis->setMaxVelocity(600);

    frontIntakesOn();

    profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {6_ft, 0_ft, 0_deg}}, "fourthGoalPickup");
    profileController->setTarget("fourthGoalPickup");
    profileController->waitUntilSettled();

    allIntakesOff();

    chassis->setMaxVelocity(300);

    chassis->turnAngle(120_deg);

    chassis->setMaxVelocity(600);

    profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {4_ft, 0_ft, 0_deg}}, "fourthGoalScore");
    profileController->setTarget("fourthGoalScore");
    profileController->waitUntilSettled();

    score();
}

void skillsAuton(){

  skillsFirstHalf();
  skillsSecondHalf();
}

void nothingToSeeHere(){

}

void testerAuton(){

  turnYawRight(400);
}
