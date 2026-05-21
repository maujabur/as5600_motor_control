#pragma once

#include <Arduino.h>
#include <Wire.h>

class As5600Sensor {
 public:
  static constexpr uint8_t DEFAULT_I2C_ADDR = 0x36;

  As5600Sensor() = default;

  bool begin(TwoWire& wire, uint8_t sda_pin, uint8_t scl_pin,
             uint8_t address = DEFAULT_I2C_ADDR);

  bool detected() const { return detected_; }
  uint8_t address() const { return address_; }

  bool readRawAngle(uint16_t* raw_angle);
  bool readAngleDeg(float* angle_deg);

 private:
  static constexpr uint8_t REG_RAW_ANGLE_H = 0x0C;

  TwoWire* wire_ = nullptr;
  uint8_t address_ = DEFAULT_I2C_ADDR;
  bool detected_ = false;
};
