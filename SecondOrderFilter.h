// Copyright 2014 Jacob Niehus
// jacob.niehus@gmail.com
// Do not distribute without permission.

#ifndef SECONDORDERFILTER_H
#define SECONDORDERFILTER_H

class SecondOrderFilter
{
  public:
  
    SecondOrderFilter();
    void initialize();
    void setFilterCoefficients(double a0, double a1, double a2,
                               double b0, double b1, double b2);
    double stepFilter(const double x);
    double getFilterOutput(){return y;}
  
  private:
  
    double a[3]; // Filter coefficients
    double b[3]; // Filter coefficients
    double y;    // Filter output
    double z[3]; // Internal filter state
};

#endif

