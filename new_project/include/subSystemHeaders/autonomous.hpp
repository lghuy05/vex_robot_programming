#include "main.h"
#pragma once

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

// Function declarations
float calculatePID(PID *pid, float target, float current, float kILimit = 50.0);
void moveForward(double inches);
void turnDegrees(double degrees);
void grab_ball(double distance_inches = 12.0, int intake_speed = 80,
               int drive_speed = 30);
void myAutonomous();
