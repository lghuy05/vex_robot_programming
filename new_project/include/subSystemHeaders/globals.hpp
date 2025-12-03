#pragma once
#include "main.h"
#include "pros/imu.hpp"

// 4 motors
// extern pros::Motor leftFront;
// extern pros::Motor leftBack;
// extern pros::Motor rightFront;
// extern pros::Motor rightBack;

// 6motors
extern pros::MotorGroup leftDrive;
extern pros::MotorGroup rightDrive;
extern pros::Controller master;
extern pros::Imu imu;
extern pros::Motor intake_motor;
extern pros::Motor intake_motor2;
extern pros::Motor intake_motor3;
