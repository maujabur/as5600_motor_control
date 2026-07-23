#include <Arduino.h>

#include <AngleSensorManager.h>
#include <AdrcPositionController.h>
#include <HBridgeMotorDriver.h>
#include <MotionCoordinator.h>
#include <MotionSequenceController.h>
#include <MotionTelemetry.h>
#include <NetworkServices.h>
#include <PreferencesSettingsStore.h>
#include <WebControlServer.h>

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#if __has_include("wifi_credentials.h")
#include "wifi_credentials.h"
#else
#define WIFI_STA_SSID ""
#define WIFI_STA_PASSWORD ""
#endif

#ifndef MOTOR_CONTROL_UNIT
#define MOTOR_CONTROL_UNIT 1
#endif

static_assert(MOTOR_CONTROL_UNIT >= 1 && MOTOR_CONTROL_UNIT <= 99,
              "MOTOR_CONTROL_UNIT deve estar entre 1 e 99");

constexpr uint8_t MOTOR_CONTROL_UNIT_NUMBER = MOTOR_CONTROL_UNIT;
constexpr char OTA_AP_PASSWORD[] = "+5511981550110";
constexpr char OTA_PASSWORD[]    = "as5600-update";
constexpr uint8_t OTA_BUTTON_PIN = 7;

// Waveshare ESP32-S3-Zero + L298N
// IN1/IN2 = motor A,  IN3/IN4 = motor B  (A é o padrao deste projeto)
constexpr uint8_t  MOTOR_A_IN1            = 1;
constexpr uint8_t  MOTOR_A_IN2            = 2;
constexpr uint8_t  MOTOR_B_IN1            = 3;
constexpr uint8_t  MOTOR_B_IN2            = 4;

constexpr uint32_t SERIAL_BAUD            = 115200;
constexpr uint32_t SERIAL_STARTUP_WAIT_MS = 3000;
constexpr uint32_t PWM_DEFAULT_FREQUENCY_HZ = 500;
constexpr uint32_t PWM_MIN_FREQUENCY_HZ     = 500;
constexpr uint32_t PWM_MAX_FREQUENCY_HZ     = 20000;
constexpr uint8_t  PWM_RESOLUTION_BITS      = 8;

constexpr uint8_t  I2C_SDA_PIN            = 5;
constexpr uint8_t  I2C_SCL_PIN            = 6;
constexpr float    DEFAULT_MOVE_MAX_RPM   = 1.8f;
constexpr float    DEFAULT_MAX_TARGET_RPM = 2.4f;
float              g_default_move_max_rpm = DEFAULT_MOVE_MAX_RPM;

AngleSensorManager g_angle_sensor;
AdrcPositionController   g_position_servo;
HBridgeMotorDriver       g_motor_driver({
  MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_B_IN1, MOTOR_B_IN2});
MotionCoordinator        g_motion_coordinator(
  g_angle_sensor, g_position_servo, g_motor_driver);
MotionSequenceController g_sequence_motion(g_motion_coordinator);
bool                     g_run_when_sensor_detected = false;
bool                    g_adrc_stall_fault = false;
MotionTelemetry         g_telemetry;
uint32_t                g_pwm_frequency_hz = PWM_DEFAULT_FREQUENCY_HZ;
uint8_t                 g_power_limit_percent = 100;

bool setPwmFrequencyHz(uint32_t hz);

DeviceSettings g_settings = DeviceSettings::defaults();
PreferencesSettingsStore g_settings_store;
bool g_settings_store_ready = false;

class MainNetworkEvents final : public NetworkServiceEvents {
 public:
  void onOtaStart() override;
};

MainNetworkEvents g_network_events;
NetworkServices g_network({
  MOTOR_CONTROL_UNIT_NUMBER, OTA_BUTTON_PIN,
  WIFI_STA_SSID, WIFI_STA_PASSWORD, OTA_AP_PASSWORD, OTA_PASSWORD},
  g_network_events);

class MainWebActions final : public WebControlActions {
 public:
  ApplicationStatus status() const override;
  DeviceSettings settings() const override;
  OperationResult replaceSettings(const DeviceSettings& settings) override;
  OperationResult setRunning(bool running) override;
  OperationResult manualMove(const MotionRequest& request) override;
  OperationResult adjustTo(bool startup_position) override;
};

MainWebActions g_web_actions;
WebControlServer g_web_server(80, g_web_actions);

void applyDeviceSettings(const DeviceSettings& settings) {
  g_settings = settings;
  g_position_servo.setSettings(g_settings.position);
  g_sequence_motion.setConfig(g_settings.sequence);
  g_power_limit_percent = g_settings.motor.power_limit_percent;
  g_pwm_frequency_hz = g_settings.motor.pwm_frequency_hz;
  g_angle_sensor.setFailureLimit(g_settings.sensor.failure_limit);
  g_motor_driver.setSettings(g_settings.motor);
}
bool saveSettingsSnapshot() {
  if (!g_settings_store_ready) return false;
  return g_settings_store.save(g_settings);
}

void loadDeviceSettings() {
  g_settings_store_ready = g_settings_store.begin();
  if (!g_settings_store_ready) {
    Serial.println("AVISO: nao foi possivel abrir a configuracao persistente");
  }
  applyDeviceSettings(g_settings_store_ready
                        ? g_settings_store.load()
                        : DeviceSettings::defaults());
}

void setRepetitiveRunning(bool running, bool persist = true) {
  g_sequence_motion.setRunning(running, millis());
  if (persist && running != g_settings.running) {
    g_settings.running = running;
    saveSettingsSnapshot();
  }
}
ApplicationStatus MainWebActions::status() const {
  const MotionStatus& motion = g_motion_coordinator.status();
  const char* sensor_state = "DETECTING";
  switch (g_angle_sensor.state()) {
    case AngleSensorManager::State::Active: sensor_state = "ACTIVE"; break;
    case AngleSensorManager::State::Lost: sensor_state = "LOST"; break;
    case AngleSensorManager::State::Detecting: break;
  }
  ApplicationStatus value;
  value.unit = MOTOR_CONTROL_UNIT_NUMBER;
  value.running = g_sequence_motion.running();
  value.move_active = motion.active;
  value.phase = g_sequence_motion.phaseText();
  value.step = g_sequence_motion.currentStep();
  value.sensor_available = motion.sensor_available;
  value.sensor_type = g_angle_sensor.sensorName();
  value.sensor_address = g_angle_sensor.sensorAddress();
  value.sensor_state = sensor_state;
  value.sensor_failures = g_angle_sensor.failureCount();
  value.angle_deg = motion.angle_deg;
  value.max_rpm = g_position_servo.settings().max_target_rpm;
  value.ota_busy = g_network.otaBusy();
  value.stalled = g_adrc_stall_fault;
  return value;
}

DeviceSettings MainWebActions::settings() const {
  return g_settings;
}

OperationResult MainWebActions::replaceSettings(
    const DeviceSettings& settings) {
  if (!validateDeviceSettings(settings)) {
    return {false, 400, "Configuracao invalida"};
  }
  if (!g_settings_store_ready || !g_settings_store.save(settings)) {
    return {false, 500, "Falha ao salvar configuracao"};
  }
  applyDeviceSettings(settings);
  return {true, 200, ""};
}

OperationResult MainWebActions::setRunning(bool running) {
  if (g_network.otaBusy()) {
    return {false, 409, "Atualizacao OTA em andamento"};
  }
  if (running && !g_motion_coordinator.status().sensor_available) {
    return {false, 409, "Sensor angular nao detectado"};
  }
  if (running) g_adrc_stall_fault = false;
  setRepetitiveRunning(running);
  return {true, 200, ""};
}

OperationResult MainWebActions::manualMove(const MotionRequest& request) {
  if (g_sequence_motion.running() || g_position_servo.isActive() ||
      g_network.otaBusy()) {
    return {false, 409,
            "Movimento avulso permitido somente com o motor parado"};
  }
  if (!g_motion_coordinator.status().sensor_available) {
    return {false, 409, "Sensor angular nao detectado"};
  }
  g_adrc_stall_fault = false;
  if (!g_motion_coordinator.startMove(request)) {
    return {false, 503, "Falha ao iniciar movimento"};
  }
  return {true, 200, ""};
}

OperationResult MainWebActions::adjustTo(bool startup_position) {
  if (g_sequence_motion.running() || g_position_servo.isActive()) {
    return {false, 409, "Ajuste permitido somente com o motor parado"};
  }
  if (g_network.otaBusy()) {
    return {false, 409, "Atualizacao OTA em andamento"};
  }
  if (!g_motion_coordinator.status().sensor_available) {
    return {false, 409, "Sensor angular nao detectado"};
  }
  const MotionStep& step = startup_position
    ? g_settings.sequence.startup_step
    : g_settings.sequence.steps[0];
  g_adrc_stall_fault = false;
  if (!g_motion_coordinator.startMove(
        {step.target_deg, step.rpm, MotionDirection::Shortest})) {
    return {false, 503, "Falha ao iniciar movimento"};
  }
  return {true, 200, ""};
}
// ─── utilitarios ────────────────────────────────────────────────────────────

bool setPwmFrequencyHz(uint32_t hz) {
  MotorDriverSettings settings = g_motor_driver.settings();
  settings.pwm_frequency_hz = hz;
  settings.pwm_resolution_bits = PWM_RESOLUTION_BITS;
  settings.power_limit_percent = g_power_limit_percent;
  if (!g_motor_driver.setSettings(settings)) return false;
  g_pwm_frequency_hz = hz;
  return true;
}

void stopMotorForOta() {
  g_sequence_motion.stop();
  g_run_when_sensor_detected = false;
  g_motion_coordinator.cancelMove();
}

void MainNetworkEvents::onOtaStart() {
  stopMotorForOta();
}
// ─── texto de estado ────────────────────────────────────────────────────────

// ─── setup / loop ───────────────────────────────────────────────────────────

void setup() {
  Serial.begin(SERIAL_BAUD);
  MotorDriverSettings motor_settings;
  motor_settings.pwm_frequency_hz = g_pwm_frequency_hz;
  motor_settings.pwm_resolution_bits = PWM_RESOLUTION_BITS;
  motor_settings.power_limit_percent = g_power_limit_percent;
  if (!g_motor_driver.begin(motor_settings)) {
    Serial.println("ERRO: falha ao inicializar ponte H");
  }
  delay(SERIAL_STARTUP_WAIT_MS);

  loadDeviceSettings();
  setPwmFrequencyHz(g_pwm_frequency_hz);

  g_angle_sensor.begin(Wire, I2C_SDA_PIN, I2C_SCL_PIN, 400000);
  if (g_network.begin()) {
    g_web_server.begin();
  } else {
    Serial.println("ERRO: rede, painel web e OTA indisponiveis");
  }

  Serial.println("\n=== Motor PWM Tester ===");
  Serial.println("Placa: Waveshare ESP32-S3-Zero  |  Motor padrao: IN3/IN4");
  Serial.printf("PWM: freq=%u Hz  resolucao=%u bits\n", g_pwm_frequency_hz, PWM_RESOLUTION_BITS);
  Serial.printf("I2C: SDA=%u SCL=%u\n", I2C_SDA_PIN, I2C_SCL_PIN);
  if (g_angle_sensor.active()) {
    Serial.printf("%s detectado no endereco 0x%02X\n",
                  g_angle_sensor.sensorName(), g_angle_sensor.sensorAddress());
  } else {
    Serial.println("Sensor angular nao detectado; procurando periodicamente");
  }
  Serial.println("ADRC pronto (motor nominal 2 rpm)");

  if (g_settings.running) {
    if (g_angle_sensor.active()) {
      Serial.println("Ciclo persistente: running=on; iniciando homing no ponto inicial");
      setRepetitiveRunning(true, false);
    } else {
      g_run_when_sensor_detected = true;
      Serial.println("Ciclo persistente: aguardando sensor angular");
    }
  }

  Serial.println("Serial: comandos desativados; interface disponivel apenas via web");
}

void loop() {
  const uint32_t now_ms = millis();
  g_network.update(now_ms);
  g_web_server.handleClient();
  g_motion_coordinator.updateSensor(now_ms);
  const uint32_t recovered_pause_ms =
    g_motion_coordinator.consumeRecoveredPauseMs();
  if (recovered_pause_ms > 0) {
    g_sequence_motion.resumeAfterPause(recovered_pause_ms);
  }
  if (g_run_when_sensor_detected &&
      g_motion_coordinator.status().sensor_available &&
      !g_network.otaBusy()) {
    g_run_when_sensor_detected = false;
    setRepetitiveRunning(true, false);
    Serial.println("Sensor detectado; iniciando ciclo persistente");
  }
  if (!g_motion_coordinator.status().paused_for_sensor) {
    g_sequence_motion.update(now_ms);
  }
  g_motion_coordinator.updateControl(now_ms);
  if (g_motion_coordinator.status().stalled && !g_adrc_stall_fault) {
    g_adrc_stall_fault = true;
    setRepetitiveRunning(false);
    Serial.println("ERRO ADRC: stall detectado; motor e ciclo desativados");
  }
  g_telemetry.update(g_motion_coordinator.status(), now_ms);

}
