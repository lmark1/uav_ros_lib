/*
 * nonlinear_filters.cpp
 *
 *  Created on: Oct 11, 2018
 *      Author: lmark
 */

#include <uav_ros_lib/nonlinear_filters.hpp>
#include <iostream>
#include <cmath>

double nonlinear_filters::saturation(double value, double lowLimit, double highLimit)
{
  if (value > highLimit) { return highLimit; }
  if (value < lowLimit) { return lowLimit; }
  return value;
}

double nonlinear_filters::deadzone(double value, double lowLimit, double highLimit)
{
  if (value < highLimit && value > lowLimit) { return 0; }
  if (value >= highLimit) { return value - highLimit; }
  if (value <= lowLimit) { return value - lowLimit; }
}

double nonlinear_filters::filterPT1(double previousValue,
  double currentValue,
  double T,
  double Ts,
  double K)
{
  double a = T / (T + Ts);
  double b = K * Ts / (T + Ts);
  return (a * previousValue + b * currentValue);
}