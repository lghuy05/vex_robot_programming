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
