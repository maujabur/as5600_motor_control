#include "BoundedInteger.h"

#include <math.h>

bool BoundedInteger::roundToUint32(float input, uint32_t minimum,
                                   uint32_t maximum, uint32_t* output) {
  if (!output || !isfinite(input) || minimum > maximum) return false;
  const double rounded = floor((double)input + 0.5);
  if (rounded < (double)minimum || rounded > (double)maximum) return false;
  *output = (uint32_t)rounded;
  return true;
}
