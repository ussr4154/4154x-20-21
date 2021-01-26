#include "main.h"
#include "autoSelect/selection.h"
using namespace okapi;

//Chassis Odometry & Motion Profiling Builders
  std::shared_ptr<OdomChassisController> chassis =
    ChassisControllerBuilder()
      .withMotors({5,4},{7,6}) // Left = 5,4 | Right = 7,6
      .withDimensions(AbstractMotor::gearset::blue, {{4_in, 12.5_in}, imev5BlueTPR})
        .withSensors(
          ADIEncoder{'G', 'H'}, //Left Encoder = A,B (Reverse)
          ADIEncoder{'E', 'F',true} // Right Encoder = C,D // Middle Encoder = E,F
        )
        // green gearset, tracking wheel diameter (2.75 in), track (7 in), and TPR (360)
        // 1 inch middle encoder distance, and 2.75 inch middle wheel diameter
        .withDimensions(AbstractMotor::gearset::blue, {{2.75_in, 6.5_in, 6_in, 2.75_in}, quadEncoderTPR})
        .withOdometry() // use the same scales as the chassis (above)
        .buildOdometry(); // Builds the odom chassis

  std::shared_ptr<AsyncMotionProfileController> profileController =
    AsyncMotionProfileControllerBuilder()
      .withLimits({
          3.0, // Maximum linear velocity of the Chassis in m/s
          2.0, // Maximum linear acceleration of the Chassis in m/s/s
          7.0 // Maximum linear jerk of the Chassis in m/s/s/s
          })
      .withOutput(chassis)
      .buildMotionProfileController();



//Autonomous Programs
void redAuton(){

}
void blueAuton(){

}
void skillsAuton(){

  deploy();

  frontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  frontRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  backLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  backRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  chassis->setMaxVelocity(600);

/*  chassis->setState({0_in, 0_in, 0_deg});

  chassis->driveToPoint({3_ft, 0_ft});

  chassis->driveToPoint({-3_ft,2_ft});
*/
  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {3_ft, 0_ft, 0_deg}}, "goal1Mid");
  profileController->setTarget("goal1Midpoint");
  profileController->waitUntilSettled();

  chassis->turnAngle(-180_deg);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {2_ft, 0_ft, 0_deg}}, "goal1Score");
  profileController->setTarget("goal1Score");
  profileController->waitUntilSettled();
}

void nothingToSeeHere(){

}

//Macros | Automatic Functions

void deploy(){

  towerIntakeTop.move(127);
  towerIntakeBottom.move(127);
  pros::delay(500);
  towerIntakeTop.move(0);
  towerIntakeBottom.move(0);

}

void score(){
  towerIntakeTop.move(127);
  towerIntakeBottom.move(127);
  pros::delay(1000);
  towerIntakeTop.move(0);
  towerIntakeBottom.move(0);
}
