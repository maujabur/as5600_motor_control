#include <BoundedInteger.h>

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

int main() {
  uint32_t value = 0;
  assert(BoundedInteger::roundToUint32(100.4f, 0, 100, &value));
  assert(value == 100);
  assert(!BoundedInteger::roundToUint32(100.6f, 0, 100, &value));
  assert(!BoundedInteger::roundToUint32(256.0f, 0, 100, &value));
  assert(!BoundedInteger::roundToUint32(257.0f, 1, 20, &value));
  assert(!BoundedInteger::roundToUint32(-1.0f, 0, 3600000, &value));
  assert(!BoundedInteger::roundToUint32(NAN, 0, 100, &value));
  assert(!BoundedInteger::roundToUint32(INFINITY, 0, 100, &value));
  assert(!BoundedInteger::roundToUint32(1.0e20f, 0, UINT32_MAX, &value));
  puts("bounded integer tests passed");
}
