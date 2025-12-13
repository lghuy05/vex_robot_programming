#include "main.h"
#include "pros/rtos.hpp"
#include "subSystemHeaders/globals.hpp"
#include "subSystemHeaders/pid_controller.hpp"
#include <ios>
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

void moveForward(double inches, int max_speed = 60) {
  // 6-WHEEL PID FOR STRAIGHT MOVEMENT
  PID pid = {0};
  pid.kP = 0.12;   // Low P for smooth acceleration
  pid.kI = 0.0003; // Very small I
  pid.kD = 0.06;   // Damping

  // Wheel measurements for 6-wheel
  double wheel_diameter = 2.7; // Adjust based on your wheels
  double circumference = 3.14159 * wheel_diameter;
  const double TICKS_PER_INCH = 360 / circumference;

  // Reset sensors
  leftDrive.tare_position();
  rightDrive.tare_position();
  imu.tare_heading();

  double initial_heading = imu.get_heading();
  double target_ticks = inches * TICKS_PER_INCH;

  // 6-WHEEL SPECIFIC
  const double MAX_CORRECTION = 15;              // Less correction for 6 wheels
  const double DEADZONE = 2.0;                   // Ignore small heading errors
  const double SPEED_FACTOR = max_speed / 127.0; // Scale output

  pid.lastError = 0;
  pid.integral = 0;

  // DECLARE good_count here (at function scope)
  int good_count = 0;

  while (true) {
    // Get current position
    double left_pos = leftDrive.get_position();
    double right_pos = rightDrive.get_position();
    double avg_pos = (left_pos + right_pos) / 2.0;

    // PID for distance
    pid.current = avg_pos;
    pid.error = target_ticks - pid.current;

    // Calculate PID
    double output = (pid.error * pid.kP) + (pid.integral * pid.kI) +
                    ((pid.error - pid.lastError) * pid.kD);
    pid.lastError = pid.error;

    // Integral with limits
    if (fabs(pid.error) < 50) {
      pid.integral += pid.error;
      if (pid.integral > 100)
        pid.integral = 100;
      if (pid.integral < -100)
        pid.integral = -100;
    }

    // Apply speed limit
    output *= SPEED_FACTOR;
    if (output > max_speed)
      output = max_speed;
    if (output < -max_speed)
      output = -max_speed;

    // === HEADING CORRECTION FOR 6 WHEELS ===
    double current_heading = imu.get_heading();
    double heading_error = initial_heading - current_heading;

    // Wrap heading error
    if (heading_error > 180)
      heading_error -= 360;
    if (heading_error < -180)
      heading_error += 360;

    // Deadzone
    if (fabs(heading_error) < DEADZONE)
      heading_error = 0;

    // Progressive correction - less at start/end
    double progress = fabs(pid.error) / fabs(target_ticks);
    double correction = heading_error * 0.15; // Gentle correction

    if (progress < 0.3 || progress > 0.7) {
      correction *= 0.6; // Even gentler at start/end
    }

    // Limit correction
    if (correction > MAX_CORRECTION)
      correction = MAX_CORRECTION;
    if (correction < -MAX_CORRECTION)
      correction = -MAX_CORRECTION;

    // Apply correction
    double left_output = output + correction;
    double right_output = output - correction;

    // Final power limits
    if (left_output > max_speed)
      left_output = max_speed;
    if (left_output < -max_speed)
      left_output = -max_speed;
    if (right_output > max_speed)
      right_output = max_speed;
    if (right_output < -max_speed)
      right_output = -max_speed;

    // Exit condition
    if (fabs(pid.error) < 8 && fabs(heading_error) < 3) {
      good_count++;
      if (good_count > 5)
        break;
    } else {
      good_count = 0;
    }

    // Move 6 wheels
    leftDrive.move(left_output);
    rightDrive.move(right_output);

    // Debug output
    pros::lcd::print(4, "Dist: %.1f/%.1f", avg_pos / TICKS_PER_INCH, inches);
    pros::lcd::print(5, "Err: %.1f HC: %.1f", pid.error, heading_error);

    pros::delay(20);
  }

  // Stop with braking
  leftDrive.move(-5);
  rightDrive.move(-5);
  pros::delay(30);

  leftDrive.move(0);
  rightDrive.move(0);

  pros::lcd::print(6, "Move Complete");
}

void turnDegrees(double degrees, int max_power = 35) {
  // IMU IS REVERSED - FIXED VERSION
  PID pid = {0};
  pid.kP = 0.25;
  pid.kI = 0.0002;
  pid.kD = 1.8;

  imu.tare_heading();
  double target_heading = degrees;

  pid.lastError = 0;
  pid.lastTime = pros::millis();
  pid.integral = 0;

  const double MIN_TURN_POWER = 10;
  const double DEADBAND = 1.5;

  int settle_count = 0;
  double last_output = 0;
  int loop_count = 0;
  int start_time = pros::millis();

  pros::lcd::print(2, "Turn: %.1f° (REVERSED IMU)", degrees);

  while (true) {
    // Time delta
    int currentTime = pros::millis();
    float dt = (currentTime - pid.lastTime) / 1000.0;
    if (dt <= 0)
      dt = 0.01;
    pid.lastTime = currentTime;

    // === CRITICAL: INVERT IMU READING ===
    double raw_heading = imu.get_heading();
    double current = 360.0 - raw_heading; // INVERT HERE
    if (current >= 360.0)
      current -= 360.0;
    if (current < 0)
      current += 360.0;

    // Calculate error with shortest path
    double error = target_heading - current;

    // Shortest path wrap-around
    if (error > 180) {
      error -= 360;
    } else if (error < -180) {
      error += 360;
    }

    // DEBUG - show both raw and corrected
    pros::lcd::print(3, "Raw:%.1f Corr:%.1f", raw_heading, current);
    pros::lcd::print(4, "Error: %.1f", error);

    // === PID (standard from here) ===
    double P = error * pid.kP;

    // I term
    if (fabs(error) < 30) {
      pid.integral += error * dt;
      if (pid.integral > 25)
        pid.integral = 25;
      if (pid.integral < -25)
        pid.integral = -25;
    } else {
      pid.integral = 0;
    }
    double I = pid.integral * pid.kI;

    // D term
    double derivative = (error - pid.lastError) / dt;
    double D = derivative * pid.kD;
    pid.lastError = error;

    double output = P + I + D;

    // Smoothing
    double max_change = 15.0 * dt;
    if (fabs(output - last_output) > max_change) {
      output = last_output + (output > last_output ? max_change : -max_change);
    }
    last_output = output;

    // Ramp up
    if (loop_count < 10) {
      output *= (loop_count / 10.0);
    }
    loop_count++;

    // Power limits
    if (output > max_power)
      output = max_power;
    if (output < -max_power)
      output = -max_power;

    // Minimum power
    if (fabs(output) < MIN_TURN_POWER && fabs(error) > 5) {
      output = (output > 0 ? MIN_TURN_POWER : -MIN_TURN_POWER);
    }

    // === CRITICAL: REVERSE MOTOR DIRECTION TOO ===
    // Since IMU is reversed, our control direction is also reversed
    // If error is positive, we need to turn clockwise (right motor forward,
    // left backward) But with reversed IMU, this logic flips

    // Apply with reversed correction
    leftDrive.move(output);   // Was -output
    rightDrive.move(-output); // Was output

    // Exit condition
    if (fabs(error) < DEADBAND) {
      settle_count++;
      if (settle_count > 5) {
        break;
      }
    } else {
      settle_count = 0;
    }

    // Timeout (3 seconds max)
    if (pros::millis() - start_time > 3000) {
      pros::lcd::print(5, "TIMEOUT");
      break;
    }

    pros::delay(10);
  }

  // Stop
  leftDrive.move(0);
  rightDrive.move(0);

  // Final reading
  double final_raw = imu.get_heading();
  double final_corrected = 360.0 - final_raw;
  pros::lcd::print(6, "Final: %.1f°", final_corrected);
}

void timedDriveWithIntake(double seconds, int drive_power = 60,
                          int intake_power = 80) {
  // Start both
  leftDrive.move(drive_power);
  rightDrive.move(drive_power);
  intake_motor.move(intake_power);
  intake_motor2.move(intake_power);
  intake_motor3.move(-intake_power);

  // Run for specified time
  pros::delay(seconds * 1000);

  // Stop both
  leftDrive.move(0);
  rightDrive.move(0);
  intake_motor.move(0);
  intake_motor2.move(0);
  intake_motor3.move(0);
}
void timedDrive(int power, int milliseconds) {
  leftDrive.move(power);
  rightDrive.move(power);
  pros::delay(milliseconds);
  leftDrive.move(0);
  rightDrive.move(0);
}

// Turn in place for a specific time
void timedTurn(int power, int milliseconds) {
  // Positive power = turn right (clockwise)
  // Negative power = turn left (counter-clockwise)
  leftDrive.move(power);
  rightDrive.move(-power); // Opposite direction for turning
  pros::delay(milliseconds);
  leftDrive.move(0);
  rightDrive.move(0);
}

void togglePiston() { piston1.toggle(); }
// Add this to the END of autonomous.cpp
void myAutonomous() {
  timedTurn(-26, 882);
  timedDrive(33, 3700);
  timedTurn(30, 1600);

  leftDrive.move(20);
  rightDrive.move(20);
  pros::delay(1400);
  leftDrive.move(20);
  rightDrive.move(20);
  intake_motor.move(-100);
  intake_motor2.move(-100);
  pros::delay(2000);
  //
  // leftDrive.move(0);
  // rightDrive.move(0);
  // intake_motor.move(0);
  // intake_motor2.move(0);
  timedTurn(25, 500);
  timedDrive(30, 2000);
  timedTurn(-30, 1700);
  timedDrive(10, 1000);
  togglePiston();
  pros::delay(1500);
  intake_motor.move(-100);
  intake_motor2.move(-100);
  intake_motor3.move(100);
  pros::delay(3000);
  piston1.set_value(false);
  //
  // 2. Slow down AND run intake SIMULTANEOUSLY for 1 second (timed)
  // pros::lcd::print(0, "Grabbing ball");
  // intake_motor.move(20);   // Intake on
  // intake_motor2.move(20);  // Intake on
  // intake_motor3.move(-20); // Intake on
  //
  // moveForward(30);
  // // Wait 1 second (both running together)
  // pros::delay(1000);
  //
  // leftDrive.move(0);
  // rightDrive.move(0);
  // // 3. Stop intake but keep moving slowly
  // intake_motor.move(0);
  // intake_motor2.move(0);
  // pros::delay(2000);
  // // 5. Optional: Back up using moveForward
  // moveForward(-24); // Back up 12 inches
}
