#pragma once

#include <WebServer.h>

#include <DeviceSettings.h>

struct OperationResult {
  bool ok = false;
  int http_status = 500;
  const char* message = "Falha interna";
};

struct ApplicationStatus {
  uint8_t unit = 1;
  bool running = false;
  bool move_active = false;
  const char* phase = "STOPPED";
  uint8_t step = 0;
  bool sensor_available = false;
  const char* sensor_type = "NONE";
  uint8_t sensor_address = 0;
  const char* sensor_state = "DETECTING";
  uint8_t sensor_failures = 0;
  float angle_deg = 0.0f;
  float max_rpm = 0.0f;
  bool ota_busy = false;
  bool stalled = false;
};

class WebControlActions {
 public:
  virtual ~WebControlActions() = default;
  virtual ApplicationStatus status() const = 0;
  virtual DeviceSettings settings() const = 0;
  virtual OperationResult replaceSettings(const DeviceSettings& settings) = 0;
  virtual OperationResult setRunning(bool running) = 0;
  virtual OperationResult manualMove(const MotionRequest& request) = 0;
  virtual OperationResult adjustTo(bool startup_position) = 0;
};

class WebControlServer {
 public:
  WebControlServer(uint16_t port, WebControlActions& actions);
  void begin();
  void handleClient();
  bool started() const { return started_; }

 private:
  bool parseNumber(const char* name, float* value) const;
  bool parseDirection(const String& text, MotionDirection* direction) const;
  bool parseSequenceStep(uint8_t index, float max_rpm, MotionStep* step) const;
  const char* directionText(MotionDirection direction) const;
  void appendSequenceStepJson(String& json, const MotionStep& step) const;
  void sendError(int status, const char* message);
  void sendOperationError(const OperationResult& result);
  void sendStatus(int status = 200);
  void sendSettings();
  void sendSequence();

  WebServer server_;
  WebControlActions& actions_;
  bool started_ = false;
};
