#pragma once

#include <AdrcPositionController.h>
#include <AngleSensorManager.h>
#include <HBridgeMotorDriver.h>
#include <MotionCoordinator.h>
#include <MotionSequenceController.h>
#include <MotionTelemetry.h>
#include <NetworkServices.h>
#include <PreferencesSettingsStore.h>
#include <WebControlServer.h>

class MotorControlApplication final : public WebControlActions,
                                      public NetworkServiceEvents {
 public:
  MotorControlApplication();

  void setup();
  void update(uint32_t now_ms);

  ApplicationStatus status() const override;
  DeviceSettings settings() const override;
  OperationResult replaceSettings(const DeviceSettings& settings) override;
  OperationResult setRunning(bool running) override;
  OperationResult manualMove(const MotionRequest& request) override;
  OperationResult adjustTo(bool startup_position) override;
  void onOtaStart() override;

 private:
  void applySettings(const DeviceSettings& settings);
  void loadSettings();
  bool saveSettings();
  void setRunning(bool running, bool persist);

  DeviceSettings settings_ = DeviceSettings::defaults();
  AngleSensorManager sensor_;
  AdrcPositionController position_controller_;
  HBridgeMotorDriver motor_;
  MotionCoordinator motion_;
  MotionSequenceController sequence_;
  PreferencesSettingsStore settings_store_;
  MotionTelemetry telemetry_;
  NetworkServices network_;
  WebControlServer web_;
  bool settings_store_ready_ = false;
  bool run_when_sensor_detected_ = false;
  bool stall_fault_ = false;
};
