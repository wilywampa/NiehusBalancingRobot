// Copyright 2014 Jacob Niehus
// jacob.niehus@gmail.com
// Do not distribute without permission.

#ifndef ENCODERINTERFACE_H
#define ENCODERINTERFACE_H

#include "Arduino.h"

// Initialize encoders
void initializeEncoders();

// Update encoders
void updateEncoders();

// Update speed
void updateSpeedA(int clicks);
void updateSpeedB(int clicks);

// Speed accessor functions return speed in clicks/sec (16 clicks per encoder wheel, 464 clicks per wheel)
double getSpeedA();
double getSpeedB();

long int getPositionA();
long int getPositionB();

#endif
