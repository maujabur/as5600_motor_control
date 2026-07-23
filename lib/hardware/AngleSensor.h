#pragma once

#include <Arduino.h>
#include <Wire.h>

enum class AngleSensorType { None, As5600, Mt6701 };

class AngleSensor {
 public:
  virtual ~AngleSensor() = default;
  virtual bool probe(TwoWire& wire, uint8_t address) = 0;
  virtual bool readRawAngle(uint16_t* raw_angle) = 0;
  virtual const char* name() const = 0;
  virtual AngleSensorType type() const = 0;
  virtual uint8_t address() const = 0;
  virtual uint16_t countsPerTurn() const = 0;

  bool readAngleDeg(float* angle_deg) {
    if (!angle_deg) return false;
    uint16_t raw_angle = 0;
    if (!readRawAngle(&raw_angle)) return false;
    *angle_deg = (float)raw_angle * 360.0f / (float)countsPerTurn();
    return true;
  }
};
