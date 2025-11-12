#include "../../include/subSystemHeaders/drive.hpp"
#include "main.h"
#include "subSystemHeaders/globals.hpp"

// void goForward() {
//   // leftDrive.move(50);
//   // rightDrive.move(50);
//   // pros::delay(2000);
//   // leftDrive.move(0);
//   // rightDrive.move(0);
//   // pros::delay(1000);
//
//   leftDrive.move_relative(720, 50); // 2 wheel revolutions
//   rightDrive.move_relative(720, 50);
//   while (!leftDrive.get_position(725) && !rightDrive.get_position(725)) {
//     pros::delay(10);
//   }
// }
//
// void turnLeft() {
//   leftDrive.move(-50);
//   rightDrive.move(50);
//   pros::delay(1000);
//   leftDrive.move(0);
//   rightDrive.move(0);
//   pros::delay(1000);
// }
//
// void goSquare() {
//   for (int i = 0; i <= 3; i++) {
//     goForward();
//     turnLeft();
//   }
// }
