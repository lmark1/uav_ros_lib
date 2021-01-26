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

double nonlinear_filters::interpolate_heading(double start, double end, double coeff)
{
  double shortest_angle =
    std::fmod((std::fmod(end - start, M_PI * 2.0)) + M_PI * 3.0, M_PI * 2) - M_PI;
  return start * (1 - coeff) + std::fmod(shortest_angle * coeff , 2 * M_PI);
}