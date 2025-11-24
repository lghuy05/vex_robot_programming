#pragma once

class PIDController {
private:
  double kP, kI, kD;
  double integral = 0;
  double previous_error = 0;
  double integral_limit = 1000; // Prevent integral windup

public:
  PIDController(double p, double i, double d) : kP(p), kI(i), kD(d) {}

  double calculate(double error) {
    integral += error;
    // Limit integral to prevent windup
    if (integral > integral_limit)
      integral = integral_limit;
    if (integral < -integral_limit)
      integral = -integral_limit;

    double derivative = error - previous_error;
    previous_error = error;

    return (kP * error) + (kI * integral) + (kD * derivative);
  }

  void reset() {
    integral = 0;
    previous_error = 0;
  }

  void setGains(double p, double i, double d) {
    kP = p;
    kI = i;
    kD = d;
  }
};
