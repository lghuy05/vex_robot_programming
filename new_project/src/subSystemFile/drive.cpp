#include "../../include/subSystemHeaders/drive.hpp"
#include "../../include/subSystemHeaders/globals.hpp"
#include "main.h"
#include "pros/misc.h"

void setDriveMotor(int left, int right) {
  leftDrive.move(left);
  rightDrive.move(right);
}


void setDrive() {
  // tank control mechanism
  // int right = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
  // int left = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  // setDriveMotor(left, right);

  // arcade control mechanism
  int power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
  leftDrive.move(power + turn);
  rightDrive.move(power - turn);
}

void setDrivewithsensor() {
  // Arcade control
  int power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
  const double speed_counter = 0.4;
  power = power * speed_counter;

  turn = turn * speed_counter;
  // IMU Auto-Straighten Feature
  static double initial_heading = 0.0;
  static bool heading_locked = false;

  // When driver stops turning, lock current heading
  if (abs(turn) < 10 && !heading_locked) {
    initial_heading = imu.get_heading();
    heading_locked = true;
  }

  // If driver starts turning again, unlock heading
  if (abs(turn) > 15) {
    heading_locked = false;
  }

  // Apply auto-straighten if heading is locked
  if (heading_locked && abs(power) > 20) {
    double current_heading = imu.get_heading();
    double error = initial_heading - current_heading;
    int correction = error * 0.4; // Adjust this gain as needed

    // Add correction to turn
    turn += correction;
  }

  // Set motor power with IMU correction
  leftDrive.move(power + turn);
  rightDrive.move(power - turn);

  // Display IMU data on brain for debugging
  static int counter = 0;
  if (counter++ % 10 == 0) { // Update every 500ms to avoid lag
    double heading = imu.get_heading();
    double pitch = imu.get_pitch();
    double roll = imu.get_roll();

    pros::lcd::print(3, "H:%.1f P:%.1f R:%.1f", heading, pitch, roll);
  }
}
