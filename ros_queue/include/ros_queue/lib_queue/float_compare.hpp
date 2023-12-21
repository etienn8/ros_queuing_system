#pragma once

#include <limits>
#include <cmath>
#include <algorithm>


// source: https://stackoverflow.com/questions/4915462/how-should-i-do-floating-point-comparison
bool float_compare(float a, float b, float epsilon = 128 * std::numeric_limits<float>::epsilon(), float abs_th = std::numeric_limits<float>::min())
{
  if (a == b) return true;

  float diff = std::abs(a-b);
  float norm = std::min((std::abs(a) + std::abs(b)), std::numeric_limits<float>::max());
  return diff < std::max(abs_th, epsilon * norm);
}
