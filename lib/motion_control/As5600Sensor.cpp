#include "As5600Sensor.h"

bool As5600Sensor::probe(TwoWire& wire, uint8_t address) {
  wire_ = &wire;
  address_ = address;
  uint16_t raw_angle = 0;
  return readRawAngle(&raw_angle);
}

bool As5600Sensor::readRawAngle(uint16_t* raw_angle) {
  if (!wire_ || !raw_angle) return false;

  wire_->beginTransmission(address_);
  wire_->write(REG_RAW_ANGLE_H);
  if (wire_->endTransmission(false) != 0) return false;

  const int bytes = wire_->requestFrom((int)address_, 2);
  if (bytes != 2) return false;

  const uint8_t high_byte = (uint8_t)wire_->read();
  const uint8_t low_byte = (uint8_t)wire_->read();

  *raw_angle = (uint16_t)(((uint16_t)high_byte << 8) | low_byte) & 0x0FFF;
  return true;
}
