#include "MotorControlApplication.h"

#include <Arduino.h>
#include <Wire.h>

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

namespace {
constexpr uint8_t kUnitNumber = MOTOR_CONTROL_UNIT;
constexpr char kOtaApPassword[] = "+5511981550110";
constexpr char kOtaPassword[] = "as5600-update";
constexpr uint8_t kOtaButtonPin = 7;
constexpr uint8_t kMotorAIn1 = 1;
constexpr uint8_t kMotorAIn2 = 2;
constexpr uint8_t kMotorBIn1 = 3;
constexpr uint8_t kMotorBIn2 = 4;
constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kSerialStartupWaitMs = 3000;
constexpr uint32_t kDefaultPwmFrequencyHz = 500;
constexpr uint8_t kPwmResolutionBits = 8;
constexpr uint8_t kI2cSdaPin = 5;
constexpr uint8_t kI2cSclPin = 6;
}  // namespace

MotorControlApplication::MotorControlApplication()
    : motor_({kMotorAIn1, kMotorAIn2, kMotorBIn1, kMotorBIn2}),
      motion_(sensor_, position_controller_, motor_),
      sequence_(motion_),
      network_({kUnitNumber, kOtaButtonPin, WIFI_STA_SSID, WIFI_STA_PASSWORD,
                kOtaApPassword, kOtaPassword},
               *this),
      web_(80, *this) {}

void MotorControlApplication::applySettings(const DeviceSettings& settings) {
  settings_ = settings;
  position_controller_.setSettings(settings_.position);
  sequence_.setConfig(settings_.sequence);
  sensor_.setFailureLimit(settings_.sensor.failure_limit);
  motor_.setSettings(settings_.motor);
}

void MotorControlApplication::loadSettings() {
  settings_store_ready_ = settings_store_.begin();
  if (!settings_store_ready_) {
    Serial.println("AVISO: nao foi possivel abrir a configuracao persistente");
  }
  applySettings(settings_store_ready_ ? settings_store_.load()
                                      : DeviceSettings::defaults());
}

bool MotorControlApplication::setRunning(bool running, bool persist) {
  if (!persist || running == settings_.running) {
    sequence_.setRunning(running, millis());
    return true;
  }

  DeviceSettings candidate = settings_;
  candidate.running = running;
  if (!running) sequence_.setRunning(false, millis());
  if (!settings_store_ready_ || !settings_store_.save(candidate)) return false;

  settings_ = candidate;
  if (running) sequence_.setRunning(true, millis());
  return true;
}

void MotorControlApplication::setup() {
  Serial.begin(kSerialBaud);
  MotorDriverSettings motor_settings;
  motor_settings.pwm_frequency_hz = kDefaultPwmFrequencyHz;
  motor_settings.pwm_resolution_bits = kPwmResolutionBits;
  motor_settings.power_limit_percent = 100;
  if (!motor_.begin(motor_settings)) {
    Serial.println("ERRO: falha ao inicializar ponte H");
  }
  delay(kSerialStartupWaitMs);

  loadSettings();
  sensor_.begin(Wire, kI2cSdaPin, kI2cSclPin, 400000);
  if (network_.begin()) {
    web_.begin();
  } else {
    Serial.println("ERRO: rede, painel web e OTA indisponiveis");
  }

  Serial.println("\n=== Motor PWM Tester ===");
  Serial.println("Placa: Waveshare ESP32-S3-Zero  |  Motor padrao: IN3/IN4");
  Serial.printf("PWM: freq=%u Hz  resolucao=%u bits\n",
                settings_.motor.pwm_frequency_hz,
                settings_.motor.pwm_resolution_bits);
  Serial.printf("I2C: SDA=%u SCL=%u\n", kI2cSdaPin, kI2cSclPin);
  if (sensor_.active()) {
    Serial.printf("%s detectado no endereco 0x%02X\n", sensor_.sensorName(),
                  sensor_.sensorAddress());
  } else {
    Serial.println("Sensor angular nao detectado; procurando periodicamente");
  }
  Serial.println("ADRC pronto (motor nominal 2 rpm)");

  if (settings_.running) {
    if (sensor_.active()) {
      Serial.println(
          "Ciclo persistente: running=on; iniciando homing no ponto inicial");
      setRunning(true, false);
    } else {
      run_when_sensor_detected_ = true;
      Serial.println("Ciclo persistente: aguardando sensor angular");
    }
  }
  Serial.println("Serial: comandos desativados; interface disponivel apenas via web");
}

void MotorControlApplication::update(uint32_t now_ms) {
  network_.update(now_ms);
  web_.handleClient();
  motion_.updateSensor(now_ms);

  const uint32_t recovered_pause_ms = motion_.consumeRecoveredPauseMs();
  if (recovered_pause_ms > 0) {
    sequence_.resumeAfterPause(recovered_pause_ms);
  }
  if (run_when_sensor_detected_ && motion_.status().sensor_available &&
      !network_.otaBusy()) {
    run_when_sensor_detected_ = false;
    setRunning(true, false);
    Serial.println("Sensor detectado; iniciando ciclo persistente");
  }
  if (!motion_.status().paused_for_sensor) {
    sequence_.update(now_ms);
  }
  motion_.updateControl(now_ms);
  if (motion_.status().stalled && !stall_fault_) {
    stall_fault_ = true;
    setRunning(false, true);
    Serial.println("ERRO ADRC: stall detectado; motor e ciclo desativados");
  }
  telemetry_.update(motion_.status(), now_ms);
}

ApplicationStatus MotorControlApplication::status() const {
  const MotionStatus& motion_status = motion_.status();
  const char* sensor_state = "DETECTING";
  switch (sensor_.state()) {
    case AngleSensorManager::State::Active: sensor_state = "ACTIVE"; break;
    case AngleSensorManager::State::Lost: sensor_state = "LOST"; break;
    case AngleSensorManager::State::Detecting: break;
  }
  ApplicationStatus value;
  value.unit = kUnitNumber;
  value.running = sequence_.running();
  value.move_active = motion_status.active;
  value.phase = sequence_.phaseText();
  value.step = sequence_.currentStep();
  value.sensor_available = motion_status.sensor_available;
  value.sensor_type = sensor_.sensorName();
  value.sensor_address = sensor_.sensorAddress();
  value.sensor_state = sensor_state;
  value.sensor_failures = sensor_.failureCount();
  value.angle_deg = motion_status.angle_deg;
  value.max_rpm = position_controller_.settings().max_target_rpm;
  value.ota_busy = network_.otaBusy();
  value.stalled = stall_fault_;
  return value;
}

DeviceSettings MotorControlApplication::settings() const {
  return settings_;
}

OperationResult MotorControlApplication::replaceSettings(
    const DeviceSettings& settings) {
  if (!validateDeviceSettings(settings)) {
    return {false, 400, "Configuracao invalida"};
  }
  if (!settings_store_ready_ || !settings_store_.save(settings)) {
    return {false, 500, "Falha ao salvar configuracao"};
  }
  applySettings(settings);
  return {true, 200, ""};
}

OperationResult MotorControlApplication::setRunning(bool running) {
  if (network_.otaBusy()) {
    return {false, 409, "Atualizacao OTA em andamento"};
  }
  if (running && !motion_.status().sensor_available) {
    return {false, 409, "Sensor angular nao detectado"};
  }
  if (running) stall_fault_ = false;
  if (!setRunning(running, true)) {
    return {false, 500, "Falha ao salvar configuracao"};
  }
  return {true, 200, ""};
}

OperationResult MotorControlApplication::manualMove(
    const MotionRequest& request) {
  if (sequence_.running() || position_controller_.isActive() ||
      network_.otaBusy()) {
    return {false, 409,
            "Movimento avulso permitido somente com o motor parado"};
  }
  if (!motion_.status().sensor_available) {
    return {false, 409, "Sensor angular nao detectado"};
  }
  stall_fault_ = false;
  if (!motion_.startMove(request)) {
    return {false, 503, "Falha ao iniciar movimento"};
  }
  return {true, 200, ""};
}

OperationResult MotorControlApplication::adjustTo(bool startup_position) {
  if (sequence_.running() || position_controller_.isActive()) {
    return {false, 409, "Ajuste permitido somente com o motor parado"};
  }
  if (network_.otaBusy()) {
    return {false, 409, "Atualizacao OTA em andamento"};
  }
  if (!motion_.status().sensor_available) {
    return {false, 409, "Sensor angular nao detectado"};
  }
  const MotionStep& step = startup_position ? settings_.sequence.startup_step
                                            : settings_.sequence.steps[0];
  stall_fault_ = false;
  if (!motion_.startMove(
          {step.target_deg, step.rpm, MotionDirection::Shortest})) {
    return {false, 503, "Falha ao iniciar movimento"};
  }
  return {true, 200, ""};
}

void MotorControlApplication::onOtaStart() {
  setRunning(false, false);
  run_when_sensor_detected_ = false;
  motion_.cancelMove();
}
