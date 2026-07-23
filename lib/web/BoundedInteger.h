#pragma once

#include <stdint.h>

namespace BoundedInteger {

bool roundToUint32(float input, uint32_t minimum, uint32_t maximum,
                   uint32_t* output);

}  // namespace BoundedInteger
