#include "main.h"
#include "autoSelect/selection.h"
using namespace okapi;

//Chassis Odometry & Motion Profiling Builders
  std::shared_ptr<OdomChassisController> chassis =
    ChassisControllerBuilder()
      .withMotors({5,4},{7,6}) // Left = 5,4 | Right = 7,6
      .withGains(
          {0.001, 0.001, 0.00001}, // distance controller gains
          {0.001, 0.001, 0.00001}, // turn controller gains
          {0, 0, 0}  // angle controller gains (helps drive straight)
        )
        .withSensors(
          ADIEncoder{'G', 'H'}, //Left Encoder = A,B (Reverse)
          ADIEncoder{'E', 'F',true},  // Right Encoder = C,D
          ADIEncoder{'C', 'D', true}  // Middle Encoder = E,F
        )
        // green gearset, tracking wheel diameter (2.75 in), track (7 in), and TPR (360)
        // 1 inch middle encoder distance, and 2.75 inch middle wheel diameter
        .withDimensions(AbstractMotor::gearset::blue, {{2.75_in, 6.5_in, 6_in, 2.75_in}, quadEncoderTPR})
        .withOdometry() // use the same scales as the chassis (above)
        .buildOdometry(); // Builds the odom chassis

  std::shared_ptr<AsyncMotionProfileController> profileController =
    AsyncMotionProfileControllerBuilder()
      .withLimits({
          1.0, // Maximum linear velocity of the Chassis in m/s
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

  chassis->setState({0_in, 0_in, 0_deg});

  chassis->driveToPoint({1_ft,1_ft});
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
