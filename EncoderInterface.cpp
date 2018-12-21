// Copyright 2014 Jacob Niehus
// jacob.niehus@gmail.com
// Do not distribute without permission.

#include "EncoderInterface.h"
#include <AdaEncoder.h>

const int deltaTimeMillis = 5;
#define DTS_PER_ENCODER_UPDATE 10

#define a_PINA 3
#define a_PINB 5
#define b_PINA 11
#define b_PINB 13

double speedA = 0;
double speedB = 0;

int encoderUpdateCounter = 0;
int clicksSinceUpdateA = 0;
int clicksSinceUpdateB = 0;
long int totalClicksA = 0;
long int totalClicksB = 0;

// Initialize encoders
void initializeEncoders()
{
  AdaEncoder::addEncoder('a', a_PINA, a_PINB);
  AdaEncoder::addEncoder('b', b_PINA, b_PINB);
}

// Update encoders
void updateEncoders()
{
  // Need to repeat twice in case both encoders updated
  int8_t clicksA = 0;
  int8_t clicksB = 0;
  for (int i = 0; i < 2; i++)
  {
    encoder *thisEncoder;
    thisEncoder = AdaEncoder::genie();
    if (thisEncoder != NULL)
    {
      if (thisEncoder->id == 'a')
      {
        clicksA = thisEncoder->clicks;
      }
      else
      {
        clicksB = thisEncoder->clicks;
      }
      thisEncoder->clicks = 0;
    }
  }
  clicksSinceUpdateA += clicksA;
  clicksSinceUpdateB += clicksB;
  
  totalClicksA += clicksA;
  totalClicksB += clicksB;
  
  // Update speeds when counter reaches 
  encoderUpdateCounter++;
  if (encoderUpdateCounter == DTS_PER_ENCODER_UPDATE)
  {
    updateSpeedA(clicksSinceUpdateA);
    updateSpeedB(clicksSinceUpdateB);
    encoderUpdateCounter = 0;
    clicksSinceUpdateA = 0;
    clicksSinceUpdateB = 0;
  }
}

// Update speed
void updateSpeedA(int clicks)
{
  speedA = (double)clicks / ((double)deltaTimeMillis / 1000.0 * (double)DTS_PER_ENCODER_UPDATE);
}
void updateSpeedB(int clicks)
{
  speedB = (double)clicks / ((double)deltaTimeMillis / 1000.0 * (double)DTS_PER_ENCODER_UPDATE);
}

// Speed accessor functions return speed in clicks/sec (16 clicks per encoder wheel, 464 clicks per wheel)
double getSpeedA(){return speedA;}
double getSpeedB(){return speedB;}

long int getPositionA(){return totalClicksA;}
long int getPositionB(){return totalClicksB;}
