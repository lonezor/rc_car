
#include "interpolation.h"

void lerp_update(double* value, double target, double min_value)
{
  *value += (target - *value) * LERP_STEP_SIZE;

  if (*value < min_value) {
    *value = min_value;
  }
}