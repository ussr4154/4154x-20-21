#include "main.h"

//Motors
pros::Motor frontLeft(1, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor frontRight(2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor backLeft(3, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor backRight(4, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor leftIntake(5, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor rightIntake(6, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor towerIntakeTop(7, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor towerIntakeBottom(8, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

//Sensors
/*
pros::ADIEncoder leftTracker('A','B', true); //Reversed
pros::ADIEncoder rightTracker('E','F');
pros::ADIEncoder middleTracker('C','D');
pros::ADIAnalogIn topBallSensor('G');
pros::ADIAnalogIn middleBallSensor('H');
pros::ADIDigitalIn bottomBallSwitch('H');
*/

//Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
