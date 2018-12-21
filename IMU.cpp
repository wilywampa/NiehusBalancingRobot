// Copyright 2014 Jacob Niehus
// jacob.niehus@gmail.com
// Do not distribute without permission.

#include "IMU.h"

// IMU pin definitions
const int tiltGyroPin  = 2;
const int yawGyroPin   = 3;
const int vertAccelPin = 0;
const int longAccelPin = 1;

// Constants
const int numCalibrationSamples = 50;
const int maxClibrationRange = 5;

// Defines for IMU calibration
#define VREF 3.3
#define GYRO_VOLTS_PER_DEG 0.0036
#define GYRO_VOLTS_PER_DEG_YAW (GYRO_VOLTS_PER_DEG/4.5)
// #define DEG_TO_RAD 0.017453292519943

// Calibration points
double tiltCenter;
double yawCenter;

// Calibrate gyros
void calibrateGyros()
{
  int tiltTotal = 0, tiltMin = 1023, tiltMax = 0;
  int yawTotal = 0, yawMin = 1023, yawMax = 0;
  for (int i = 0; i < numCalibrationSamples; i++)
  {
    int tilt = analogRead(tiltGyroPin);
    tiltTotal += tilt;
    if (tilt > tiltMax)
    {
      tiltMax = tilt;
    }
    if (tilt < tiltMin)
    {
      tiltMin = tilt;
    }
    int yaw = analogRead(yawGyroPin);
    yawTotal += yaw;
    if (yaw > yawMax)
    {
      yawMax = yaw;
    }
    if (yaw < yawMin)
    {
      yawMin = yaw;
    }
    if (  ((tiltMax - tiltMin) > maxClibrationRange)
       || ((yawMax - yawMin) > maxClibrationRange))
    {
      i = -1;
      tiltTotal = 0;
      yawTotal = 0;
      tiltMin = 1023;
      tiltMax = 0;
      yawMin = 1023;
      yawMax = 0;
    }
    delay(2);
  }
  tiltCenter = (double)tiltTotal / (double)numCalibrationSamples;
  yawCenter = (double)yawTotal / (double)numCalibrationSamples;
}

// Function to return measured tilt rate (rad/sec)
double getTiltRate()
{
  return ((double)analogRead(tiltGyroPin) - tiltCenter) * VREF / 1024.0 / GYRO_VOLTS_PER_DEG * DEG_TO_RAD;
}

// Function to return measured yaw rate (rad/sec)
double getYawRate()
{
  return (yawCenter - (double)analogRead(yawGyroPin)) * VREF / 1024.0 / GYRO_VOLTS_PER_DEG_YAW * DEG_TO_RAD;
}

// Function to return measured vertical accel (g)
double getVertAccel()
{
  return map(analogRead(vertAccelPin), 513, 339, -1000, 1000) / 1000.0;
}

// Function to return measured longitudinal accel (g)
double getLongAccel()
{
  return map(analogRead(longAccelPin), 357, 529, -1000, 1000) / 1000.0;
}
