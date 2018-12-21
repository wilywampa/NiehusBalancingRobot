// Copyright 2014 Jacob Niehus
// jacob.niehus@gmail.com
// Do not distribute without permission.

#include "SecondOrderFilter.h"

SecondOrderFilter::SecondOrderFilter()
{
  initialize();
}

void SecondOrderFilter::initialize()
{
  for (int i = 0; i < 3; i++)
  {
    a[i] = 0;
    b[i] = 0;
    z[i] = 0;
  }

  y = 0;
  
  setFilterCoefficients(1, 0, 0, 1, 0, 0);
}

void SecondOrderFilter::setFilterCoefficients(double a0, double a1, double a2,
                                              double b0, double b1, double b2)
{
  a[0] = a0;
  a[1] = a1;
  a[2] = a2;
  b[0] = b0;
  b[1] = b1;
  b[2] = b2;
}

double SecondOrderFilter::stepFilter(const double x)
{
  z[0] = x - a[2] * z[2] - a[1] * z[1];
  y = b[2] * z[2] + b[1] * z[1] + b[0] * z[0];
  z[2] = z[1];
  z[1] = z[0];
  
  return y;
}

