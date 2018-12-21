// Copyright 2014 Jacob Niehus
// jacob.niehus@gmail.com
// Do not distribute without permission.

#ifndef IMU_H
#define IMU_H

#include "Arduino.h"

// Calibrate gyros
void calibrateGyros();

// Function to return measured tilt rate (rad/sec)
double getTiltRate();

// Function to return measured yaw rate (rad/sec)
double getYawRate();

// Function to return measured vertical accel (g)
double getVertAccel();

// Function to return measured longitudinal accel (g)
double getLongAccel();

#endif
