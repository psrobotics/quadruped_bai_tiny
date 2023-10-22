#include "quick_math.h"

float quick_sin(float theta)
{
    while (theta < 0.0f) theta += 6.28318530718f;
    while (theta >= 6.28318530718f) theta -= 6.28318530718f;    
    return sin_table[(int) (multiplier*theta)] ;
}

float quick_cos(float theta)
{
  float rl = quick_sin(theta + pi/2);
  return rl;
}
