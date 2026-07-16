#pragma once

#include "AngleSensor.h"

class Mt6701Sensor final : public AngleSensor {
 public:
  static constexpr uint8_t DEFAULT_I2C_ADDR = 0x06;
  static constexpr uint8_t ALTERNATE_I2C_ADDR = 0x46;
  static constexpr uint8_t REG_ANGLE_HIGH = 0x03;
  static constexpr uint8_t REG_ANGLE_LOW = 0x04;

  bool probe(TwoWire& wire, uint8_t address) override;
  bool readRawAngle(uint16_t* raw_angle) override;
  const char* name() const override { return "MT6701"; }
  AngleSensorType type() const override { return AngleSensorType::Mt6701; }
  uint8_t address() const override { return address_; }
  uint16_t countsPerTurn() const override { return 16384; }

 private:
  bool readRegister(uint8_t register_address, uint8_t* value);
  TwoWire* wire_ = nullptr;
  uint8_t address_ = DEFAULT_I2C_ADDR;
};
