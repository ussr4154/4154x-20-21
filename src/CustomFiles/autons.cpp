#include "main.h"
using namespace okapi;

//Chassis Odometry & Motion Profiling Builders
  std::shared_ptr<OdomChassisController> chassis =
    ChassisControllerBuilder()
      .withMotors({1,3},{2,4}) // Left = 1,3 | Right = 2,4
      .withGains(
          {0.001, 0, 0.0001}, // distance controller gains
          {0.001, 0, 0.0001}, // turn controller gains
          {0.001, 0, 0.0001}  // angle controller gains (helps drive straight)
        )
        .withSensors(
          ADIEncoder{'A', 'B', true}, //Left Encoder = A,B (Reverse)
          ADIEncoder{'C', 'D'},  // Right Encoder = C,D
          ADIEncoder{'E', 'F'}  // Middle Encoder = E,F
        )
        // green gearset, tracking wheel diameter (2.75 in), track (7 in), and TPR (360)
        // 1 inch middle encoder distance, and 2.75 inch middle wheel diameter
        .withDimensions(AbstractMotor::gearset::green, {{2.75_in, 7_in, 1_in, 2.75_in}, quadEncoderTPR})
        .withOdometry() // use the same scales as the chassis (above)
        .buildOdometry(); // Builds the odom chassis

  std::shared_ptr<AsyncMotionProfileController> profileController =
    AsyncMotionProfileControllerBuilder()
      .withLimits({
          1.0, // Maximum linear velocity of the Chassis in m/s
          2.0, // Maximum linear acceleration of the Chassis in m/s/s
          5.0 // Maximum linear jerk of the Chassis in m/s/s/s
          })
      .withOutput(chassis)
      .buildMotionProfileController();



//Autonomous Programs
void redAuton(){
}
void blueAuton(){

}
void skillsAuton(){

}
