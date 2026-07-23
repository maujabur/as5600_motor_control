#include "Mt6701Sensor.h"

bool Mt6701Sensor::probe(TwoWire& wire, uint8_t address) {
  if (address != DEFAULT_I2C_ADDR && address != ALTERNATE_I2C_ADDR) return false;

  wire_ = &wire;
  address_ = address;
  uint16_t raw_angle = 0;
  return readRawAngle(&raw_angle);
}

bool Mt6701Sensor::readRegister(uint8_t register_address, uint8_t* value) {
  if (!wire_ || !value) return false;

  wire_->beginTransmission(address_);
  wire_->write(register_address);
  if (wire_->endTransmission(false) != 0) return false;

  const int bytes = wire_->requestFrom((int)address_, 1);
  if (bytes != 1) return false;

  *value = (uint8_t)wire_->read();
  return true;
}

bool Mt6701Sensor::readRawAngle(uint16_t* raw_angle) {
  if (!wire_ || !raw_angle) return false;
  uint8_t high_byte = 0;
  uint8_t low_byte = 0;
  if (!readRegister(REG_ANGLE_HIGH, &high_byte)) return false;
  if (!readRegister(REG_ANGLE_LOW, &low_byte)) return false;
  *raw_angle = (uint16_t)(((uint16_t)high_byte << 6) |
                         ((uint16_t)low_byte >> 2)) & 0x3FFF;
  return true;
}
