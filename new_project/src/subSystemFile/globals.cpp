#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"

// pros::Motor leftFront(1);
// pros::Motor leftBack(2);
// pros::Motor rightFront(3);
// pros::Motor rightBack(4);
//
//
// 6 motors
pros::MotorGroup leftDrive({-4, -5, 6}, pros::v5::MotorGear::blue); //-4,-5,6
//
pros::MotorGroup rightDrive({-1, 2, 3}, pros::v5::MotorGear::blue); //-1,2,3
// pros::MotorGroup leftDrive({7, 8});
// pros::MotorGroup rightDrive({5, 6});
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Imu imu(7);

// blue set
// pros::MotorGroup bluemotors({1, 2}, pros::E_MOTOR_GEAR_BLUE);
