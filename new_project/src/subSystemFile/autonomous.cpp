#include "main.h"
#include "pros/rtos.hpp"
#include "subSystemHeaders/globals.hpp"
#include "subSystemHeaders/pid_controller.hpp"
typedef struct {
  float current;
  float kP;
  float kI;
  float kD;
  float target;
  float error;
  float integral;
  float derivative;
  float lastError;
  float threshold;
  int lastTime;
} PID;

// Simple PID calculation function
float calculatePID(PID *pid, float target, float current,
                   float kILimit = 50.0) {
  pid->current = current;
  pid->error = target - pid->current;

  // Integral term with anti-windup
  if (abs(pid->error) < kILimit) {
    pid->integral += pid->error;
  } else {
    pid->integral = 0; // Reset integral when far from target
  }

  // Derivative term
  pid->derivative = pid->error - pid->lastError;
  pid->lastError = pid->error;

  // PID calculation
  return (pid->error * pid->kP) + (pid->integral * pid->kI) +
         (pid->derivative * pid->kD);
}

void grab_ball(double slow_distance_inches = 6.0, int intake_speed = 80,
               int slow_speed = 30) {
  // This function: Move slowly while intaking for exactly 1 second

  PID pid = {0};
  pid.kP = 0.2;
  pid.kI = 0.001;
  pid.kD = 0.05;

  // Calculate TICKS_PER_INCH
  double wheel_diameter = 2.7; // Your measured value
  double circumference = 3.14159 * wheel_diameter;
  const double TICKS_PER_INCH = 360 / circumference;

  leftDrive.tare_position();
  rightDrive.tare_position();

  double target_ticks = slow_distance_inches * TICKS_PER_INCH;
  int intake_timer = 0;
  bool intake_started = false;

  while (true) {
    double avg_pos =
        (leftDrive.get_position() + rightDrive.get_position()) / 2.0;
    double output = calculatePID(&pid, target_ticks, avg_pos);

    // Check if we've reached target
    if (fabs(pid.error) < 10) {
      break;
    }

    // Limit to slow speed
    if (output > slow_speed)
      output = slow_speed;
    if (output < -slow_speed)
      output = -slow_speed;

    // Start intake when we start moving slowly
    if (!intake_started) {
      intake_motor.move(intake_speed);
      intake_motor2.move(intake_speed);
      intake_started = true;
      intake_timer = 0;
    }

    // Run intake for exactly 1000ms (1 second)
    intake_timer += 10; // We delay 10ms each loop
    if (intake_timer >= 1000) {
      intake_motor.move(0);
      intake_motor2.move(0);
    }

    // Apply to motors
    leftDrive.move(output);
    rightDrive.move(output);

    pros::delay(10);
  }

  // Stop drive motors
  leftDrive.move(0);
  rightDrive.move(0);

  // Make sure intake is off
  intake_motor.move(0);
  intake_motor2.move(0);
}

// Modified moveForward with PID struct
void moveForward(double inches) {
  PID pid = {0};               // Initialize all to 0
  pid.kP = 0.2;                // Start with this - will need tuning
  pid.kI = 0.001;              // Small integral gain
  pid.kD = 0.01;               // Small derivative gain
  double wheel_diameter = 2.7; // MEASURE AND PUT YOUR WHEEL SIZE HERE!
  double circumference = 3.14159 * wheel_diameter;
  double TICKS_PER_REVOLUTION = 360; // Standard for V5 motors
  const double TICKS_PER_INCH = TICKS_PER_REVOLUTION / circumference;

  leftDrive.tare_position();
  rightDrive.tare_position();

  double target_ticks = inches * TICKS_PER_INCH;

  while (true) {
    double left_pos = leftDrive.get_position();
    double right_pos = rightDrive.get_position();
    double avg_pos = (left_pos + right_pos) / 2.0;

    // Calculate PID output
    double output = calculatePID(&pid, target_ticks, avg_pos);

    // Check if we've reached target
    if (fabs(pid.error) < 10) { // 10 ticks tolerance
      break;
    }

    // Limit output to prevent overshoot
    if (output > 60)
      output = 60;
    if (output < -60)
      output = -60;

    // Apply to motors
    leftDrive.move(output);
    rightDrive.move(output);

    pros::delay(10);
  }

  // Stop motors
  leftDrive.move(0);
  rightDrive.move(0);
}

// Modified turnDegrees with PID struct
void turnDegrees(double degrees) {
  PID pid = {0};
  pid.kP = 1.5;   // Higher P for turning
  pid.kI = 0.005; // Very small I
  pid.kD = 0.5;   // Higher D to prevent overshoot

  imu.tare_heading(); // Reset to 0

  while (true) {
    double current_heading = imu.get_heading();

    // Handle 360/0 wrap-around
    double error = degrees - current_heading;
    if (error > 180)
      error -= 360;
    if (error < -180)
      error += 360;

    // Use a helper variable for the PID target
    double target = degrees;

    // Calculate PID output
    double output = calculatePID(&pid, 0, error);

    // Check if we're close enough
    if (fabs(error) < 2.0) { // 2 degree tolerance
      break;
    }

    // Limit turn speed
    if (output > 50)
      output = 50;
    if (output < -50)
      output = -50;

    // Turn in place
    leftDrive.move(-output);
    rightDrive.move(output);

    pros::delay(10);
  }

  leftDrive.move(0);
  rightDrive.move(0);
}
// Add this to the END of autonomous.cpp
void myAutonomous() {
  pros::lcd::print(0, "Moving to ball area");
  moveForward(24); // Move 24 inches precisely to ball

  // 2. Slow down AND run intake SIMULTANEOUSLY for 1 second (timed)
  pros::lcd::print(0, "Grabbing ball");
  intake_motor.move(20);  // Intake on
  intake_motor2.move(20); // Intake on
  leftDrive.move(25);     // Slow speed
  rightDrive.move(25);    // Slow speed

  // Wait 1 second (both running together)
  pros::delay(1000);

  leftDrive.move(0);
  rightDrive.move(0);
  // 3. Stop intake but keep moving slowly
  intake_motor.move(0);
  intake_motor2.move(0);
  pros::delay(2000);
  // 5. Optional: Back up using moveForward
  moveForward(-24); // Back up 12 inches
}
