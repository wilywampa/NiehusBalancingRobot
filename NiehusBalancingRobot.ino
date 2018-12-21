// Copyright 2014 Jacob Niehus
// jacob.niehus@gmail.com
// Do not distribute without permission.

#include "IMU.h"
#include "PID_v1.h"
#include "EncoderInterface.h"
#include "DualVNH5019MotorShield.h"
#include <PinChangeInt.h>
#include <AdaEncoder.h>
#include <Streaming.h>
#include "Wire.h"
#include "SecondOrderFilter.h"
SecondOrderFilter notchFilter;
SecondOrderFilter notchFilterYaw;
#include "WiiChuck.h"
WiiChuck chuck = WiiChuck();

const int deltaTimeMillis = 5;
#define DTS_PER_ENCODER_UPDATE 10
unsigned long lastExecutionTimeMicros;
bool giveUp = false;
DualVNH5019MotorShield motorShield;

// PID variables
double speedSetpoint = 0, speedInput = 0, speedOutput = 0;
PID speedPID(&speedInput, &speedOutput, &speedSetpoint, 0.00006, 0.00002, 0.000001, DIRECT);

double tiltSetpoint = 0, tiltInput = 0, tiltOutput = 0;
PID tiltPID(&tiltInput, &tiltOutput, &tiltSetpoint, 65, 10, 0.0001, DIRECT);

double rateSetpoint = 0, rateInput = 0, rateOutput = 0, rateOutputFiltered = 0;
PID ratePID(&rateInput, &rateOutput, &rateSetpoint, 50, 15, 0.2, REVERSE);

double yawSetpoint = 0, yawInput = 0, yawOutput = 0, yawOutputFiltered = 0;
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, 70, 50, 0, REVERSE);

#define MOTOR_KP 0.0015
#define MOTOR_KI 0.0002
#define MOTOR_KD 0.0003
#define MOTOR_LIMIT 400.0

double motorAsetpoint = 0, motorAinput = 0, motorAoutput = 0;
PID motorApid(&motorAinput, &motorAoutput, &motorAsetpoint, MOTOR_KP, MOTOR_KI, MOTOR_KD, REVERSE);

double motorBsetpoint = 0, motorBinput = 0, motorBoutput = 0;
PID motorBpid(&motorBinput, &motorBoutput, &motorBsetpoint, MOTOR_KP, MOTOR_KI, MOTOR_KD, REVERSE);

// Tilt filter variables
double tiltAngle = 0;
double tiltRateBias = 0;

// Initial setup
void setup()
{
  Serial.begin(115200);

  analogReference(EXTERNAL);

  // Notch filters - 10 Hz critical frequency - 0.25 Hz bandwidth
  notchFilter.setFilterCoefficients(1.000000, -1.894854, 0.992367,
                                    0.995878, -1.894272, 0.995878);
  notchFilterYaw.setFilterCoefficients(1.000000, -1.894854, 0.992367,
                                       0.995878, -1.894272, 0.995878);

  delay(100);
  // Wii Nunchuck initializiation
  chuck.begin();
  chuck.update();
  do
  {
    delay(20);
    chuck.update();
    Serial << "chuck.buttonC = " << chuck.buttonC << endl;
    Serial << "chuck.buttonZ = " << chuck.buttonZ << endl;
  }
  while (!(chuck.buttonC && chuck.buttonZ));
  chuck.calibrateJoy();

  calibrateGyros();

  motorShield.init();

  initializeEncoders();

  // PID setup
  speedPID.SetOutputLimits(-0.1, 0.1);
  speedPID.SetSampleTime(deltaTimeMillis * DTS_PER_ENCODER_UPDATE);
  speedPID.SetMode(AUTOMATIC);

  tiltPID.SetOutputLimits(-5, 5);
  tiltPID.SetSampleTime(deltaTimeMillis);
  tiltPID.SetMode(AUTOMATIC);

  ratePID.SetOutputLimits(-MOTOR_LIMIT, MOTOR_LIMIT);
  ratePID.SetSampleTime(deltaTimeMillis);
  ratePID.SetMode(AUTOMATIC);

  yawPID.SetOutputLimits(-MOTOR_LIMIT, MOTOR_LIMIT);
  yawPID.SetSampleTime(deltaTimeMillis);
  yawPID.SetMode(AUTOMATIC);

  motorApid.SetOutputLimits(-MOTOR_LIMIT, MOTOR_LIMIT);
  motorApid.SetSampleTime(deltaTimeMillis * DTS_PER_ENCODER_UPDATE);
  motorApid.SetMode(AUTOMATIC);

  motorBpid.SetOutputLimits(-MOTOR_LIMIT, MOTOR_LIMIT);
  motorBpid.SetSampleTime(deltaTimeMillis * DTS_PER_ENCODER_UPDATE);
  motorBpid.SetMode(AUTOMATIC);

  tiltAngle = atan2(-getLongAccel(),getVertAccel());

  lastExecutionTimeMicros = micros();
}

// Execution time checking
int t0, t1, dt, dtMax;

// Main loop
void loop()
{
  // Main timing loop
  if (micros() - lastExecutionTimeMicros >= deltaTimeMillis * 1000)
  {
    lastExecutionTimeMicros = micros();

    t0 = micros();

    // Update tilt filter
    double tiltRateMeas = getTiltRate();
    double tiltRate = tiltRateMeas + tiltRateBias;
    double tiltAngleEst = tiltAngle + (double)deltaTimeMillis / 1000.0 * tiltRate;
    double tiltAngleMeas = atan2(-getLongAccel(),getVertAccel());
    double tiltAngleError = tiltAngleMeas - tiltAngleEst;
    double tiltRateBiasGain = 0.0001;
    double tau = 0.6;
    double weight = tau / (tau + (double)deltaTimeMillis / 1000.0);
    tiltRateBias += tiltRateBiasGain * (double)deltaTimeMillis * tiltAngleError;
    tiltAngle = weight * tiltAngleEst + (1.0 - weight) * tiltAngleMeas;

    // Stop trying to balance if tilt angle grows too large
    if (fabs(tiltAngle) > 0.5)
    {
      giveUp = true;
    }

    updateEncoders();

    chuck.update();

    // Update controller
    speedSetpoint = (double)chuck.readJoyY() * 10.0;
    speedInput = (getSpeedA() - getSpeedB()) / 2.0;
    speedPID.Compute();

    tiltSetpoint = speedOutput;
    tiltInput = tiltAngle;
    tiltPID.Compute();

    rateSetpoint = tiltOutput;
    rateInput = tiltRate;
    ratePID.Compute();

    yawSetpoint = (double)chuck.readJoyX() / 50.0;
    yawInput = getYawRate();
    yawPID.Compute();

    //motorAsetpoint += -rateOutput * 4000.0 * (double)deltaTimeMillis / 1000.0;
    //motorAinput = getPositionA();
    //motorApid.Compute();

    //motorBsetpoint += rateOutput * 4000.0 * (double)deltaTimeMillis / 1000.;
    //motorBinput = getPositionB();
    //motorBpid.Compute();

    rateOutputFiltered = notchFilter.stepFilter(rateOutput);
    yawOutputFiltered = notchFilterYaw.stepFilter(yawOutput);

    // Set output commands
    if (giveUp)
    {
      motorShield.setM1Speed(0);
      motorShield.setM2Speed(0);
    }
    else
    {
      motorShield.setM1Speed(-rateOutput + yawOutput);
      motorShield.setM2Speed( rateOutput + yawOutput);
    }

    // Print serial data
    if (dtMax >= deltaTimeMillis * 1000)
    {
      Serial << "Execution time overrun - max dt = " << dtMax << " micros" << endl;
    }
    else
    {
      // Serial << speedSetpoint << endl;
      Serial << tiltSetpoint << "," << tiltInput << "," << tiltOutput << endl;
      // Serial << tiltAngle << endl;
      // Serial << rateSetpoint << rateInput << rateOutput << rateOutputFiltered << endl;
      // Serial << rateInput << "," << yawInput << endl;
      // Serial << rateOutput << "," << yawOutput << endl;
    }

    // Measure execution time and maximum execution time
    t1 = micros();
    dt = t1 - t0;
    if (dt > dtMax)
    {
      dtMax = dt;
    }
  }
}

