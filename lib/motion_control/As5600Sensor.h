#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "AngleSensor.h"

class As5600Sensor final : public AngleSensor {
 public:
  static constexpr uint8_t DEFAULT_I2C_ADDR = 0x36;
  static constexpr uint8_t REG_RAW_ANGLE_H = 0x0C;

  bool probe(TwoWire& wire, uint8_t address) override;
  bool readRawAngle(uint16_t* raw_angle) override;
  const char* name() const override { return "AS5600"; }
  AngleSensorType type() const override { return AngleSensorType::As5600; }
  uint8_t address() const override { return address_; }
  uint16_t countsPerTurn() const override { return 4096; }

 private:
  TwoWire* wire_ = nullptr;
  uint8_t address_ = DEFAULT_I2C_ADDR;
};
