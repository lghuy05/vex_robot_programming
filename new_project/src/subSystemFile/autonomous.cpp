#include "main.h"
#include "subSystemHeaders/globals.hpp"
#include "subSystemHeaders/pid_controller.hpp"

void moveForward(double inches) {
  PIDController pid(0.1, 0.001, 0.01); // You'll need to tune these
  const double TICKS_PER_INCH = 42.5;  // Adjust based on your wheel size

  leftDrive.tare_position();
  rightDrive.tare_position();

  double target_ticks = inches * TICKS_PER_INCH;

  while (true) {
    double left_pos = leftDrive.get_position();
    double right_pos = rightDrive.get_position();
    double avg_pos = (left_pos + right_pos) / 2.0;

    double error = target_ticks - avg_pos;

    // Stop when close enough to target
    if (fabs(error) < 20)
      break;

    double output = pid.calculate(error);

    // Limit output to motor range
    if (output > 60)
      output = 60;
    if (output < -60)
      output = -60;

    leftDrive.move(output);
    rightDrive.move(output);

    pros::delay(10);
  }

  leftDrive.move(0);
  rightDrive.move(0);
}

void turnDegrees(double degrees) {
  PIDController turn_pid(1.0, 0.01, 0.2); // Different gains for turning

  imu.tare_heading(); // Reset IMU to 0

  while (true) {
    double current_heading = imu.get_heading();
    double error = degrees - current_heading;

    // Handle crossing 360/0 boundary
    if (error > 180)
      error -= 360;
    if (error < -180)
      error += 360;

    if (fabs(error) < 2)
      break; // Stop when close enough

    double output = turn_pid.calculate(error);

    if (output > 50)
      output = 50;
    if (output < -50)
      output = -50;
    leftDrive.move(-output);
    rightDrive.move(output);

    pros::delay(10);
  }

  leftDrive.move(0);
  rightDrive.move(0);
}

void myAutonomous() {
  // Simple autonomous routine
  moveForward(36); // Move 24 inches forward
  turnDegrees(90); // Turn 90 degrees
  moveForward(12); // Move 12 inches forward
}
