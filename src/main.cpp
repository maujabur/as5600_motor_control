#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include <AngleMath.h>
#include <AngleSensorManager.h>
#include <AdrcPositionController.h>
#include <HBridgeMotorDriver.h>
#include <MotionCoordinator.h>
#include <MotionSequenceController.h>

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "repetitive_motion_web_page.h"
#include "control_settings_web_page.h"

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
char OTA_AP_SSID[20] = {0};
constexpr char OTA_AP_PASSWORD[] = "+5511981550110";
char OTA_HOSTNAME[28] = {0};
constexpr char OTA_PASSWORD[]    = "as5600-update";
constexpr uint32_t WIFI_STA_CONNECT_TIMEOUT_MS = 12000;
// Para seis unidades: 01/04 -> canal 1, 02/05 -> canal 6, 03/06 -> canal 11.
// Os tres canais nao se sobrepoem e recebem exatamente duas unidades cada.
constexpr uint8_t WIFI_AP_CHANNEL_SLOT = (MOTOR_CONTROL_UNIT_NUMBER - 1U) % 3U;
constexpr uint8_t WIFI_AP_CHANNEL = WIFI_AP_CHANNEL_SLOT == 0U ? 1U
                                    : WIFI_AP_CHANNEL_SLOT == 1U ? 6U : 11U;
constexpr uint8_t OTA_BUTTON_PIN = 7;
constexpr uint32_t OTA_BUTTON_HOLD_MS = 1500;

// Waveshare ESP32-S3-Zero + L298N
// IN1/IN2 = motor A,  IN3/IN4 = motor B  (A é o padrao deste projeto)
constexpr uint8_t  MOTOR_A_IN1            = 1;
constexpr uint8_t  MOTOR_A_IN2            = 2;
constexpr uint8_t  MOTOR_B_IN1            = 3;
constexpr uint8_t  MOTOR_B_IN2            = 4;

constexpr uint32_t SERIAL_BAUD            = 115200;
constexpr uint32_t SERIAL_STARTUP_WAIT_MS = 3000;
constexpr uint32_t MOVE_RPM_TELEMETRY_WINDOW_MS = 80;
constexpr uint32_t MOVE_DEBUG_LOG_PERIOD_MS = 2000;
constexpr uint32_t PWM_DEFAULT_FREQUENCY_HZ = 500;
constexpr uint32_t PWM_MIN_FREQUENCY_HZ     = 500;
constexpr uint32_t PWM_MAX_FREQUENCY_HZ     = 20000;
constexpr uint8_t  PWM_RESOLUTION_BITS      = 8;

constexpr uint8_t  I2C_SDA_PIN            = 5;
constexpr uint8_t  I2C_SCL_PIN            = 6;
constexpr float    DEFAULT_MOVE_MAX_RPM   = 1.8f;
constexpr float    DEFAULT_MAX_TARGET_RPM = 2.4f;
float              g_default_move_max_rpm = DEFAULT_MOVE_MAX_RPM;

bool g_ota_mode_active = false;
bool g_ota_update_in_progress = false;
bool g_ota_button_pressed = false;
uint32_t g_ota_button_pressed_ms = 0;

AngleSensorManager g_angle_sensor;
AdrcPositionController   g_position_servo;
HBridgeMotorDriver       g_motor_driver({
  MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_B_IN1, MOTOR_B_IN2});
MotionCoordinator        g_motion_coordinator(
  g_angle_sensor, g_position_servo, g_motor_driver);
MotionSequenceController g_sequence_motion(g_motion_coordinator);
bool                     g_run_when_sensor_detected = false;
bool                    g_move_done_reported = false;
bool                    g_adrc_stall_fault = false;
bool                    g_move_tracking_active = false;
float                   g_move_peak_rpm_abs = 0.0f;
float                   g_move_peak_rpm_signed = 0.0f;
float                   g_move_last_rpm_signed = 0.0f;
float                   g_move_last_nonzero_rpm_signed = 0.0f;
bool                    g_move_prev_sample_valid = false;
float                   g_move_prev_deg = 0.0f;
uint32_t                g_move_prev_ms = 0;
uint32_t                g_move_start_ms = 0;
float                   g_move_rpm_sum = 0.0f;
uint32_t                g_move_rpm_samples = 0;
float                   g_move_total_abs_delta_deg = 0.0f;
float                   g_move_total_net_delta_deg = 0.0f;
float                   g_move_total_progress_deg = 0.0f;
float                   g_move_start_accumulated_deg = 0.0f;
bool                    g_move_start_accumulated_captured = false;
float                   g_move_rpm_window_delta_deg = 0.0f;
uint32_t                g_move_rpm_window_dt_ms = 0;
int16_t                 g_move_peak_pwm_adrc_abs = 0;
int16_t                 g_move_peak_pwm_out_abs = 0;
float                   g_move_pwm_out_abs_sum = 0.0f;
uint32_t                g_move_pwm_out_samples = 0;
uint32_t                g_move_pwm_out_sat_samples = 0;
uint32_t                g_pwm_frequency_hz = PWM_DEFAULT_FREQUENCY_HZ;
uint8_t                 g_power_limit_percent = 100;
uint32_t                g_move_debug_last_print_ms = 0;

float clampf(float v, float lo, float hi);
bool setPwmFrequencyHz(uint32_t hz);

Preferences g_repetitive_preferences;
bool g_repetitive_preferences_ready = false;
bool g_repetitive_run_on_boot = false;
bool g_persisted_repetitive_running = false;
WebServer g_web_server(80);
bool g_web_server_started = false;

constexpr uint32_t SEQUENCE_STORAGE_VERSION = 1;
struct StoredMotionSequence {
  uint32_t version;
  MotionSequenceConfig config;
};

void loadRepetitiveMotionPreferences() {
  g_repetitive_preferences_ready =
    g_repetitive_preferences.begin("repeat_motion", false);
  if (!g_repetitive_preferences_ready) {
    Serial.println("AVISO: nao foi possivel abrir a configuracao persistente do ciclo");
    return;
  }

  AdrcPositionSettings adrc = g_position_servo.settings();
  adrc.control_bandwidth = clampf(g_repetitive_preferences.getFloat(
    "adrc_wc", adrc.control_bandwidth), 1.0f, 100.0f);
  adrc.observer_bandwidth = clampf(g_repetitive_preferences.getFloat(
    "adrc_wo", adrc.observer_bandwidth), 1.0f, 300.0f);
  adrc.plant_gain = clampf(g_repetitive_preferences.getFloat(
    "adrc_b0", adrc.plant_gain), 1.0f, 2000.0f);
  adrc.max_target_rpm = clampf(g_repetitive_preferences.getFloat(
    "max_rpm", adrc.max_target_rpm), 0.1f, 10.0f);
  adrc.physical_max_rpm = clampf(g_repetitive_preferences.getFloat(
    "phys_rpm", adrc.physical_max_rpm), 0.1f, 10.0f);
  adrc.max_target_rpm = fminf(adrc.max_target_rpm, adrc.physical_max_rpm);
  adrc.stop_window_deg = clampf(g_repetitive_preferences.getFloat(
    "stop_win", adrc.stop_window_deg), 0.2f, 20.0f);
  adrc.accel_ramp_ms = (uint16_t)constrain(
    g_repetitive_preferences.getUInt("accel_ms", adrc.accel_ramp_ms), 50U, 2000U);
  adrc.decel_ramp_ms = (uint16_t)constrain(
    g_repetitive_preferences.getUInt("decel_ms", adrc.decel_ramp_ms), 50U, 2000U);
  adrc.kick_pwm_percent = clampf(g_repetitive_preferences.getFloat(
    "kick_pct", adrc.kick_pwm_percent), 0.0f, 100.0f);
  adrc.kick_ms = (uint16_t)constrain(
    g_repetitive_preferences.getUInt("kick_ms", adrc.kick_ms), 0U, 1000U);
  adrc.samples_to_stop = (uint16_t)constrain(
    g_repetitive_preferences.getUInt("stop_samples", adrc.samples_to_stop), 1U, 20U);
  adrc.minimum_drive_pwm_percent = clampf(g_repetitive_preferences.getFloat(
    "min_pwm", adrc.minimum_drive_pwm_percent), 0.0f, 45.0f);
  adrc.stall_timeout_ms = (uint16_t)constrain(
    g_repetitive_preferences.getUInt("stall_ms", adrc.stall_timeout_ms), 100U, 10000U);
  adrc.stall_velocity_deg_s = clampf(g_repetitive_preferences.getFloat(
    "stall_vel", adrc.stall_velocity_deg_s), 0.1f, 20.0f);
  adrc.velocity_window_ms = (uint16_t)constrain(
    g_repetitive_preferences.getUInt("vel_win", adrc.velocity_window_ms), 20U, 1000U);
  adrc.velocity_num_samples = (uint8_t)constrain(
    g_repetitive_preferences.getUInt("vel_samples", adrc.velocity_num_samples), 2U, 20U);
  g_position_servo.setSettings(adrc);
  g_power_limit_percent = (uint8_t)constrain(
    g_repetitive_preferences.getUInt("power_limit", g_power_limit_percent), 0U, 100U);
  g_pwm_frequency_hz = constrain(
    g_repetitive_preferences.getUInt("pwm_hz", g_pwm_frequency_hz),
    PWM_MIN_FREQUENCY_HZ, PWM_MAX_FREQUENCY_HZ);
  g_angle_sensor.setFailureLimit((uint8_t)constrain(
    g_repetitive_preferences.getUInt("sensor_fail", 3), 1U, 20U));

  StoredMotionSequence stored{};
  if (g_repetitive_preferences.getBytesLength("sequence") == sizeof(stored) &&
      g_repetitive_preferences.getBytes("sequence", &stored, sizeof(stored)) == sizeof(stored) &&
      stored.version == SEQUENCE_STORAGE_VERSION &&
      stored.config.step_count >= 1 &&
      stored.config.step_count <= MOTION_SEQUENCE_MAX_STEPS) {
    g_sequence_motion.setConfig(stored.config);
  } else {
    MotionSequenceConfig sequence;
    sequence.startup_step = {0.0f, 1.0f, 1000,
                             MotionDirection::Shortest};
    sequence.step_count = 2;
    sequence.steps[0] = {180.0f, 1.0f, 2000,
                         MotionDirection::Clockwise};
    sequence.steps[1] = {0.0f, 1.0f, 1000,
                         MotionDirection::CounterClockwise};
    g_sequence_motion.setConfig(sequence);
  }
  g_repetitive_run_on_boot = g_repetitive_preferences.getBool("running", false);
  g_persisted_repetitive_running = g_repetitive_run_on_boot;

}

void saveRepetitiveMotionConfig() {
  if (!g_repetitive_preferences_ready) return;
  StoredMotionSequence stored{SEQUENCE_STORAGE_VERSION, g_sequence_motion.config()};
  g_repetitive_preferences.putBytes("sequence", &stored, sizeof(stored));
}

void saveAdrcSettings() {
  if (!g_repetitive_preferences_ready) return;
  const AdrcPositionSettings& s = g_position_servo.settings();
  g_repetitive_preferences.putFloat("adrc_wc", s.control_bandwidth);
  g_repetitive_preferences.putFloat("adrc_wo", s.observer_bandwidth);
  g_repetitive_preferences.putFloat("adrc_b0", s.plant_gain);
}

void saveControlSettings() {
  if (!g_repetitive_preferences_ready) return;
  const AdrcPositionSettings& s = g_position_servo.settings();
  g_repetitive_preferences.putFloat("adrc_wc", s.control_bandwidth);
  g_repetitive_preferences.putFloat("adrc_wo", s.observer_bandwidth);
  g_repetitive_preferences.putFloat("adrc_b0", s.plant_gain);
  g_repetitive_preferences.putFloat("max_rpm", s.max_target_rpm);
  g_repetitive_preferences.putFloat("phys_rpm", s.physical_max_rpm);
  g_repetitive_preferences.putFloat("stop_win", s.stop_window_deg);
  g_repetitive_preferences.putUInt("accel_ms", s.accel_ramp_ms);
  g_repetitive_preferences.putUInt("decel_ms", s.decel_ramp_ms);
  g_repetitive_preferences.putFloat("kick_pct", s.kick_pwm_percent);
  g_repetitive_preferences.putUInt("kick_ms", s.kick_ms);
  g_repetitive_preferences.putUInt("stop_samples", s.samples_to_stop);
  g_repetitive_preferences.putFloat("min_pwm", s.minimum_drive_pwm_percent);
  g_repetitive_preferences.putUInt("stall_ms", s.stall_timeout_ms);
  g_repetitive_preferences.putFloat("stall_vel", s.stall_velocity_deg_s);
  g_repetitive_preferences.putUInt("vel_win", s.velocity_window_ms);
  g_repetitive_preferences.putUInt("vel_samples", s.velocity_num_samples);
  g_repetitive_preferences.putUInt("power_limit", g_power_limit_percent);
  g_repetitive_preferences.putUInt("pwm_hz", g_pwm_frequency_hz);
  g_repetitive_preferences.putUInt("sensor_fail", g_angle_sensor.failureLimit());
}

void setRepetitiveRunning(bool running, bool persist = true) {
  g_sequence_motion.setRunning(running, millis());
  if (persist && g_repetitive_preferences_ready &&
      running != g_persisted_repetitive_running) {
    g_repetitive_preferences.putBool("running", running);
    g_persisted_repetitive_running = running;
  }
}

bool parseWebNumber(const char* name, float* value) {
  if (!value || !g_web_server.hasArg(name)) return false;
  const String text = g_web_server.arg(name);
  char* end = nullptr;
  const float parsed = strtof(text.c_str(), &end);
  if (end == text.c_str() || !end || *end != '\0' || !isfinite(parsed)) return false;
  *value = parsed;
  return true;
}

const char* angleSensorStateText(AngleSensorManager::State state) {
  switch (state) {
    case AngleSensorManager::State::Active: return "ACTIVE";
    case AngleSensorManager::State::Lost: return "LOST";
    case AngleSensorManager::State::Detecting: return "DETECTING";
  }
  return "DETECTING";
}

void sendWebStatus(int status_code = 200) {
  const MotionStatus& motion = g_motion_coordinator.status();
  const bool sensor_ok = motion.sensor_available;
  String json;
  json.reserve(420);
  json += F("{\"unit\":");
  json += String(MOTOR_CONTROL_UNIT_NUMBER);
  json += F(",\"running\":");
  json += g_sequence_motion.running() ? F("true") : F("false");
  json += F(",\"moveActive\":");
  json += g_position_servo.isActive() ? F("true") : F("false");
  json += F(",\"phase\":\"");
  json += g_sequence_motion.phaseText();
  json += F("\",\"step\":");
  json += String(g_sequence_motion.currentStep());
  json += F(",\"sensor\":");
  json += sensor_ok ? F("true") : F("false");
  json += F(R"json(,"sensorType":")json");
  json += g_angle_sensor.sensorName();
  json += F(R"json(","sensorAddress":)json");
  json += String(g_angle_sensor.sensorAddress());
  json += F(R"json(,"sensorState":")json");
  json += angleSensorStateText(g_angle_sensor.state());
  json += F(R"json(","sensorFailures":)json");
  json += String(g_angle_sensor.failureCount());
  json += F(",\"angle\":"); json += String(motion.angle_deg, 2);
  json += F(",\"maxRpm\":"); json += String(g_position_servo.settings().max_target_rpm, 2);
  json += F(",\"otaBusy\":");
  json += g_ota_update_in_progress ? F("true") : F("false");
  json += F(",\"stall\":");
  json += g_adrc_stall_fault ? F("true") : F("false");
  json += '}';
  g_web_server.send(status_code, "application/json", json);
}

void sendWebError(int status_code, const char* message) {
  String json = F("{\"error\":\"");
  json += message;
  json += F("\"}");
  g_web_server.send(status_code, "application/json", json);
}

void sendWebSettings() {
  const AdrcPositionSettings& s = g_position_servo.settings();
  String json;
  json.reserve(360);
  json += F("{\"canEdit\":");
  json += (!g_sequence_motion.running() && !g_position_servo.isActive() &&
           !g_ota_update_in_progress) ? F("true") : F("false");
  json += F(",\"wc\":"); json += String(s.control_bandwidth, 2);
  json += F(",\"wo\":"); json += String(s.observer_bandwidth, 2);
  json += F(",\"b0\":"); json += String(s.plant_gain, 2);
  json += F(",\"maxRpm\":"); json += String(s.max_target_rpm, 2);
  json += F(",\"physRpm\":"); json += String(s.physical_max_rpm, 2);
  json += F(",\"powerLimit\":"); json += String(g_power_limit_percent);
  json += F(",\"pwmHz\":"); json += String(g_pwm_frequency_hz);
  json += F(",\"stopWindow\":"); json += String(s.stop_window_deg, 2);
  json += F(",\"stopSamples\":"); json += String(s.samples_to_stop);
  json += F(",\"accelMs\":"); json += String(s.accel_ramp_ms);
  json += F(",\"decelMs\":"); json += String(s.decel_ramp_ms);
  json += F(",\"kickPct\":"); json += String(s.kick_pwm_percent, 1);
  json += F(",\"kickMs\":"); json += String(s.kick_ms);
  json += F(",\"minPwm\":"); json += String(s.minimum_drive_pwm_percent, 1);
  json += F(",\"stallMs\":"); json += String(s.stall_timeout_ms);
  json += F(",\"stallVel\":"); json += String(s.stall_velocity_deg_s, 2);
  json += F(",\"velWindow\":"); json += String(s.velocity_window_ms);
  json += F(",\"velSamples\":"); json += String(s.velocity_num_samples);
  json += F(",\"sensorFailures\":"); json += String(g_angle_sensor.failureLimit());
  json += '}';
  g_web_server.send(200, "application/json", json);
}

const char* motionDirectionText(MotionDirection direction) {
  switch (direction) {
    case MotionDirection::Clockwise: return "cw";
    case MotionDirection::CounterClockwise: return "ccw";
    case MotionDirection::Shortest: return "shortest";
  }
  return "shortest";
}

bool parseMotionDirection(const String& text, MotionDirection* direction) {
  if (!direction) return false;
  if (text == "shortest") *direction = MotionDirection::Shortest;
  else if (text == "cw") *direction = MotionDirection::Clockwise;
  else if (text == "ccw") *direction = MotionDirection::CounterClockwise;
  else return false;
  return true;
}

void appendSequenceStepJson(String& json, const MotionStep& step) {
  json += F("{\"target\":"); json += String(step.target_deg, 2);
  json += F(",\"rpm\":"); json += String(step.rpm, 2);
  json += F(",\"dwell\":"); json += String(step.dwell_ms);
  json += F(",\"direction\":\""); json += motionDirectionText(step.direction);
  json += F("\"}");
}

void sendWebSequence() {
  const MotionSequenceConfig& config = g_sequence_motion.config();
  String json;
  json.reserve(1900);
  json += F("{\"canEdit\":");
  json += (!g_sequence_motion.running() && !g_position_servo.isActive() &&
           !g_ota_update_in_progress) ? F("true") : F("false");
  json += F(",\"maxRpm\":");
  json += String(g_position_servo.settings().max_target_rpm, 2);
  json += F(",\"startup\":");
  appendSequenceStepJson(json, config.startup_step);
  json += F(",\"steps\":[");
  for (uint8_t i = 0; i < config.step_count; ++i) {
    if (i) json += ',';
    appendSequenceStepJson(json, config.steps[i]);
  }
  json += F("]}");
  g_web_server.send(200, "application/json", json);
}

bool parseWebSequenceStep(uint8_t index, MotionStep* step) {
  if (!step) return false;
  const String prefix = "s" + String(index) + "_";
  float target = 0.0f, rpm = 0.0f, dwell = 0.0f;
  if (!parseWebNumber((prefix + "target").c_str(), &target) ||
      !parseWebNumber((prefix + "rpm").c_str(), &rpm) ||
      !parseWebNumber((prefix + "dwell").c_str(), &dwell) ||
      !g_web_server.hasArg(prefix + "direction")) return false;
  MotionDirection direction;
  if (!parseMotionDirection(g_web_server.arg(prefix + "direction"), &direction)) return false;
  const float max_rpm = g_position_servo.settings().max_target_rpm;
  if (target < 0.0f || target >= 360.0f || rpm < 0.1f || rpm > max_rpm ||
      dwell < 0.0f || dwell > 3600000.0f) return false;
  *step = {target, rpm, (uint32_t)lroundf(dwell), direction};
  return true;
}

void setupWebControl() {
  if (g_web_server_started) return;

  g_web_server.on("/", HTTP_GET, []() {
    g_web_server.send_P(200, "text/html; charset=utf-8", REPETITIVE_MOTION_WEB_PAGE);
  });
  g_web_server.on("/settings", HTTP_GET, []() {
    g_web_server.send_P(200, "text/html; charset=utf-8", CONTROL_SETTINGS_WEB_PAGE);
  });
  g_web_server.on("/api/settings", HTTP_GET, []() { sendWebSettings(); });
  g_web_server.on("/api/settings", HTTP_POST, []() {
    if (g_sequence_motion.running() || g_position_servo.isActive() ||
        g_ota_update_in_progress) {
      sendWebError(409, "Pare o motor antes de alterar os ajustes");
      return;
    }
    float wc, wo, b0, max_rpm, phys_rpm, power_limit, pwm_hz;
    float stop_window, stop_samples, accel_ms, decel_ms, kick_pct, kick_ms;
    float min_pwm, stall_ms, stall_vel, vel_window, vel_samples, sensor_failures;
    if (!parseWebNumber("wc", &wc) || !parseWebNumber("wo", &wo) ||
        !parseWebNumber("b0", &b0) || !parseWebNumber("maxRpm", &max_rpm) ||
        !parseWebNumber("physRpm", &phys_rpm) ||
        !parseWebNumber("powerLimit", &power_limit) ||
        !parseWebNumber("pwmHz", &pwm_hz) ||
        !parseWebNumber("stopWindow", &stop_window) ||
        !parseWebNumber("stopSamples", &stop_samples) ||
        !parseWebNumber("accelMs", &accel_ms) ||
        !parseWebNumber("decelMs", &decel_ms) ||
        !parseWebNumber("kickPct", &kick_pct) ||
        !parseWebNumber("kickMs", &kick_ms) ||
        !parseWebNumber("minPwm", &min_pwm) ||
        !parseWebNumber("stallMs", &stall_ms) ||
        !parseWebNumber("stallVel", &stall_vel) ||
        !parseWebNumber("velWindow", &vel_window) ||
        !parseWebNumber("velSamples", &vel_samples) ||
        !parseWebNumber("sensorFailures", &sensor_failures)) {
      sendWebError(400, "Parametros invalidos ou incompletos");
      return;
    }
    if (wc < 1.0f || wc > 100.0f || wo < 1.0f || wo > 300.0f ||
        b0 < 1.0f || b0 > 2000.0f || max_rpm < 0.1f || max_rpm > 10.0f ||
        phys_rpm < 0.1f || phys_rpm > 10.0f || max_rpm > phys_rpm ||
        power_limit < 0.0f || power_limit > 100.0f ||
        pwm_hz < PWM_MIN_FREQUENCY_HZ || pwm_hz > PWM_MAX_FREQUENCY_HZ ||
        stop_window < 0.2f || stop_window > 20.0f ||
        stop_samples < 1.0f || stop_samples > 20.0f ||
        accel_ms < 50.0f || accel_ms > 2000.0f ||
        decel_ms < 50.0f || decel_ms > 2000.0f ||
        kick_pct < 0.0f || kick_pct > 100.0f ||
        kick_ms < 0.0f || kick_ms > 1000.0f ||
        min_pwm < 0.0f || min_pwm > 45.0f ||
        stall_ms < 100.0f || stall_ms > 10000.0f ||
        stall_vel < 0.1f || stall_vel > 20.0f ||
        vel_window < 20.0f || vel_window > 1000.0f ||
        vel_samples < 2.0f || vel_samples > 20.0f ||
        sensor_failures < 1.0f || sensor_failures > 20.0f ||
        sensor_failures != floorf(sensor_failures)) {
      sendWebError(422, "Valor fora da faixa ou RPM maxima acima da RPM fisica");
      return;
    }

    AdrcPositionSettings s = g_position_servo.settings();
    s.control_bandwidth = wc;
    s.observer_bandwidth = wo;
    s.plant_gain = b0;
    s.max_target_rpm = max_rpm;
    s.physical_max_rpm = phys_rpm;
    s.stop_window_deg = stop_window;
    s.samples_to_stop = (uint16_t)lroundf(stop_samples);
    s.accel_ramp_ms = (uint16_t)lroundf(accel_ms);
    s.decel_ramp_ms = (uint16_t)lroundf(decel_ms);
    s.kick_pwm_percent = kick_pct;
    s.kick_ms = (uint16_t)lroundf(kick_ms);
    s.minimum_drive_pwm_percent = min_pwm;
    s.stall_timeout_ms = (uint16_t)lroundf(stall_ms);
    s.stall_velocity_deg_s = stall_vel;
    s.velocity_window_ms = (uint16_t)lroundf(vel_window);
    s.velocity_num_samples = (uint8_t)lroundf(vel_samples);
    g_position_servo.setSettings(s);
    g_angle_sensor.setFailureLimit((uint8_t)lroundf(sensor_failures));
    g_power_limit_percent = (uint8_t)lroundf(power_limit);
    if (!setPwmFrequencyHz((uint32_t)lroundf(pwm_hz))) {
      sendWebError(500, "Falha ao aplicar frequencia PWM");
      return;
    }

    MotionSequenceConfig sequence = g_sequence_motion.config();
    sequence.startup_step.rpm = fminf(sequence.startup_step.rpm, max_rpm);
    for (uint8_t i = 0; i < sequence.step_count; ++i) {
      sequence.steps[i].rpm = fminf(sequence.steps[i].rpm, max_rpm);
    }
    g_sequence_motion.setConfig(sequence);
    saveRepetitiveMotionConfig();
    saveControlSettings();
    sendWebSettings();
  });
  g_web_server.on("/api/status", HTTP_GET, []() { sendWebStatus(); });
  g_web_server.on("/api/sequence", HTTP_GET, []() { sendWebSequence(); });
  g_web_server.on("/api/sequence", HTTP_POST, []() {
    if (g_sequence_motion.running() || g_position_servo.isActive() ||
        g_ota_update_in_progress) {
      sendWebError(409, "Pare o motor antes de alterar a sequencia");
      return;
    }
    float count_value = 0.0f;
    if (!parseWebNumber("count", &count_value) || count_value < 1.0f ||
        count_value > MOTION_SEQUENCE_MAX_STEPS ||
        count_value != floorf(count_value)) {
      sendWebError(422, "Quantidade de passos invalida");
      return;
    }
    MotionSequenceConfig candidate;
    candidate.step_count = (uint8_t)count_value;
    if (!parseWebSequenceStep(0, &candidate.startup_step)) {
      sendWebError(422, "Passo inicial invalido");
      return;
    }
    for (uint8_t i = 1; i <= candidate.step_count; ++i) {
      if (!parseWebSequenceStep(i, &candidate.steps[i - 1])) {
        sendWebError(422, "Passo do ciclo invalido");
        return;
      }
    }
    g_sequence_motion.setConfig(candidate);
    saveRepetitiveMotionConfig();
    sendWebSequence();
  });
  g_web_server.on("/api/run", HTTP_POST, []() {
    if (!g_web_server.hasArg("running")) {
      sendWebError(400, "Parametro running ausente");
      return;
    }
    const bool requested = g_web_server.arg("running") == "1";
    if (requested && g_ota_update_in_progress) {
      sendWebError(409, "Atualizacao OTA em andamento");
      return;
    }
    if (requested && !g_angle_sensor.active()) {
      sendWebError(409, "Sensor angular nao detectado");
      return;
    }
    if (requested) g_adrc_stall_fault = false;
    setRepetitiveRunning(requested);
    sendWebStatus();
  });
  g_web_server.on("/api/adjust", HTTP_POST, []() {
    if (g_sequence_motion.running() || g_position_servo.isActive()) {
      sendWebError(409, "Ajuste permitido somente com o motor parado");
      return;
    }
    if (g_ota_update_in_progress) {
      sendWebError(409, "Atualizacao OTA em andamento");
      return;
    }
    if (!g_angle_sensor.active()) {
      sendWebError(409, "Sensor angular nao detectado");
      return;
    }
    if (!g_web_server.hasArg("target")) {
      sendWebError(400, "Destino ausente");
      return;
    }
    const MotionSequenceConfig& sequence = g_sequence_motion.config();
    g_adrc_stall_fault = false;
    const String target = g_web_server.arg("target");
    if (target == "start") {
      g_motion_coordinator.startMove({
        sequence.startup_step.target_deg,
        sequence.startup_step.rpm,
        MotionDirection::Shortest});
    } else if (target == "end") {
      const MotionStep& end_step = sequence.steps[0];
      g_motion_coordinator.startMove({
        end_step.target_deg, end_step.rpm, MotionDirection::Shortest});
    } else {
      sendWebError(400, "Destino invalido");
      return;
    }
    sendWebStatus();
  });
  g_web_server.on("/api/manual-move", HTTP_POST, []() {
    if (g_sequence_motion.running() || g_position_servo.isActive() ||
        g_ota_update_in_progress) {
      sendWebError(409, "Movimento avulso permitido somente com o motor parado");
      return;
    }
    if (!g_angle_sensor.active()) {
      sendWebError(409, "Sensor angular nao detectado");
      return;
    }

    float target_deg = 0.0f;
    float rpm = 0.0f;
    if (!parseWebNumber("target", &target_deg) ||
        !parseWebNumber("rpm", &rpm)) {
      sendWebError(400, "Destino ou RPM invalido");
      return;
    }
    const float max_rpm = g_position_servo.settings().max_target_rpm;
    if (target_deg < 0.0f || target_deg >= 360.0f ||
        rpm < 0.1f || rpm > max_rpm) {
      sendWebError(422, "Destino ou RPM fora da faixa permitida");
      return;
    }

    g_adrc_stall_fault = false;
    if (!g_motion_coordinator.startMove(
          {target_deg, rpm, MotionDirection::Shortest})) {
      sendWebError(503, "Falha ao iniciar movimento");
      return;
    }
    g_move_done_reported = false;
    g_move_start_ms = millis();
    sendWebStatus();
  });
  g_web_server.on("/api/config", HTTP_POST, []() {
    float start = 0.0f, end = 0.0f, rpm_out = 0.0f, rpm_back = 0.0f;
    float wait_start = 0.0f, wait_end = 0.0f;
    if (!parseWebNumber("start", &start) || !parseWebNumber("end", &end) ||
        !parseWebNumber("rpmOut", &rpm_out) || !parseWebNumber("rpmBack", &rpm_back) ||
        !parseWebNumber("waitStart", &wait_start) || !parseWebNumber("waitEnd", &wait_end)) {
      sendWebError(400, "Parametros invalidos ou incompletos");
      return;
    }
    const float max_rpm = g_position_servo.settings().max_target_rpm;
    if (start < 0.0f || start >= 360.0f || end < 0.0f || end >= 360.0f ||
        rpm_out < 0.1f || rpm_out > max_rpm || rpm_back < 0.1f || rpm_back > max_rpm ||
        wait_start < 0.0f || wait_start > 3600000.0f ||
        wait_end < 0.0f || wait_end > 3600000.0f) {
      sendWebError(422, "Valor fora da faixa permitida");
      return;
    }
    MotionSequenceConfig sequence = g_sequence_motion.config();
    sequence.startup_step = {
      start, rpm_back, (uint32_t)lroundf(wait_start),
      MotionDirection::Shortest};
    sequence.step_count = 2;
    sequence.steps[0] = {
      end, rpm_out, (uint32_t)lroundf(wait_end),
      MotionDirection::Clockwise};
    sequence.steps[1] = {
      start, rpm_back, (uint32_t)lroundf(wait_start),
      MotionDirection::CounterClockwise};
    g_sequence_motion.setConfig(sequence);
    saveRepetitiveMotionConfig();
    sendWebStatus();
  });
  g_web_server.onNotFound([]() {
    g_web_server.sendHeader("Location", "/", true);
    g_web_server.send(302, "text/plain", "");
  });
  g_web_server.begin();
  g_web_server_started = true;
  Serial.println("WEB: painel disponivel em http://192.168.4.1/");
}

// ─── utilitarios ────────────────────────────────────────────────────────────

float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

bool setPwmFrequencyHz(uint32_t hz) {
  MotorDriverSettings settings = g_motor_driver.settings();
  settings.pwm_frequency_hz = hz;
  settings.pwm_resolution_bits = PWM_RESOLUTION_BITS;
  settings.power_limit_percent = g_power_limit_percent;
  if (!g_motor_driver.setSettings(settings)) return false;
  g_pwm_frequency_hz = hz;
  return true;
}

void updateMotionDiagnostics(uint32_t now_ms) {
  const MotionStatus& motion = g_motion_coordinator.status();
  if (motion.paused_for_sensor) return;

  if (!motion.active) {
    g_move_tracking_active = false;
    g_move_prev_sample_valid = false;
    g_move_total_abs_delta_deg = 0.0f;
    g_move_total_net_delta_deg = 0.0f;
    g_move_total_progress_deg = 0.0f;
    g_move_start_accumulated_captured = false;
    g_move_rpm_window_delta_deg = 0.0f;
    g_move_rpm_window_dt_ms = 0;
    g_move_debug_last_print_ms = 0;
    return;
  }

  if (!g_move_tracking_active) {
    g_move_tracking_active = true;
    g_move_peak_rpm_abs = 0.0f;
    g_move_peak_rpm_signed = 0.0f;
    g_move_last_rpm_signed = 0.0f;
    g_move_last_nonzero_rpm_signed = 0.0f;
    g_move_rpm_sum = 0.0f;
    g_move_rpm_samples = 0;
    g_move_total_abs_delta_deg = 0.0f;
    g_move_total_net_delta_deg = 0.0f;
    g_move_total_progress_deg = 0.0f;
    g_move_start_accumulated_captured = false;
    g_move_rpm_window_delta_deg = 0.0f;
    g_move_rpm_window_dt_ms = 0;
    g_move_peak_pwm_adrc_abs = 0;
    g_move_peak_pwm_out_abs = 0;
    g_move_pwm_out_abs_sum = 0.0f;
    g_move_pwm_out_samples = 0;
    g_move_pwm_out_sat_samples = 0;
    g_move_prev_sample_valid = false;
    g_move_debug_last_print_ms = millis();
  }

  const float current_deg = motion.angle_deg;

  // Acumula deslocamento angular absoluto para diagnostico de movimentos curtos.
  if (g_move_prev_sample_valid) {
    const uint32_t dt_ms = now_ms - g_move_prev_ms;
    if (dt_ms > 0) {
      const float delta_deg =
        AngleMath::shortestDelta(g_move_prev_deg, current_deg);
      // Para rpm_medio, considera deslocamento quase integral; rejeita apenas glitches grosseiros.
      if (fabsf(delta_deg) <= 20.0f) {
        g_move_total_abs_delta_deg += fabsf(delta_deg);
        g_move_total_net_delta_deg += delta_deg;
        const float cmd_rpm = g_position_servo.commandedRpm();
        const float sign = (cmd_rpm >= 0.0f) ? 1.0f : -1.0f;
        const float projected_progress = sign * delta_deg;
        if (projected_progress > 0.0f) {
          g_move_total_progress_deg += projected_progress;
        }
      }

      // Nao computa rpm_pico durante KICK para evitar inflar metricas de regime.
      if (!g_position_servo.isKicking()) {
        const AdrcPositionSettings& cfg = g_position_servo.settings();
        // Para rpm_pico, usa filtro dinamico mais estrito.
        float max_delta_plausible_deg = 3.0f;
        if (cfg.physical_max_rpm > 0.1f) {
          const float rpm_guard = cfg.physical_max_rpm * 1.8f;
          max_delta_plausible_deg = (rpm_guard * 360.0f * (float)dt_ms) / 60000.0f + 0.8f;
        }
        // Pico de RPM por janela temporal para reduzir efeito de quantizacao.
        if (fabsf(delta_deg) <= max_delta_plausible_deg) {
          g_move_rpm_window_delta_deg += delta_deg;
          g_move_rpm_window_dt_ms += dt_ms;
        }
        if (g_move_rpm_window_dt_ms >= MOVE_RPM_TELEMETRY_WINDOW_MS) {
          const float rpm_window =
            (g_move_rpm_window_delta_deg * 60000.0f) / (360.0f * (float)g_move_rpm_window_dt_ms);
          const float abs_rpm_window = fabsf(rpm_window);
          float rpm_peak_limit = fmaxf(g_position_servo.maxSpeedRpm() * 2.20f, 1.0f);
          if (cfg.physical_max_rpm > 0.1f) {
            rpm_peak_limit = fmaxf(rpm_peak_limit, cfg.physical_max_rpm * 1.40f + 0.20f);
          }
          rpm_peak_limit = fminf(rpm_peak_limit, 30.0f);

          if (abs_rpm_window <= rpm_peak_limit) {
            if (abs_rpm_window > g_move_peak_rpm_abs) {
              g_move_peak_rpm_abs = abs_rpm_window;
              g_move_peak_rpm_signed = rpm_window;
            }
            if (abs_rpm_window > 0.2f) {
              g_move_last_nonzero_rpm_signed = rpm_window;
            }
          }

          g_move_rpm_window_delta_deg = 0.0f;
          g_move_rpm_window_dt_ms = 0;
        }
      } else {
        // Reinicia a janela para nao carregar amostras de kick para o pico de regime.
        g_move_rpm_window_delta_deg = 0.0f;
        g_move_rpm_window_dt_ms = 0;
      }
    }
  }
  g_move_prev_deg = current_deg;
  g_move_prev_ms = now_ms;
  g_move_prev_sample_valid = true;

  const int16_t pwm_adrc_abs = (int16_t)abs(g_position_servo.pwmOutput());
  if (pwm_adrc_abs > g_move_peak_pwm_adrc_abs) {
    g_move_peak_pwm_adrc_abs = pwm_adrc_abs;
  }
  const int16_t pwm_out_abs =
    (int16_t)abs(motion.pwm_percent);
  if (pwm_out_abs > g_move_peak_pwm_out_abs) {
    g_move_peak_pwm_out_abs = pwm_out_abs;
  }

  // Captura posição acumulada inicial (odômetro) na primeira iteração do movimento.
  if (!g_move_start_accumulated_captured && g_position_servo.isActive()) {
    g_move_start_accumulated_deg = g_position_servo.accumulatedDeg();
    g_move_start_accumulated_captured = true;
  }

  if (g_position_servo.isActive()) {
    // Mantem RPM instantaneo para debug/diagnostico.
    g_move_last_rpm_signed = g_position_servo.measuredRpm();

    // Debug periodico sem liberar entrada serial: status reduzido a cada 10s.
    if (g_move_debug_last_print_ms == 0 || (now_ms - g_move_debug_last_print_ms) >= MOVE_DEBUG_LOG_PERIOD_MS) {
      g_move_debug_last_print_ms = now_ms;
      Serial.printf("DBG: move t=%u ms  rpm_inst=%.2f  rpm_raw=%.2f  rpm_cmd=%.2f  erro_rpm=%.2f  pwm=%d%%\n",
                    now_ms - g_move_start_ms,
                    g_position_servo.measuredRpm(),
                    g_position_servo.measuredRpmRaw(),
                    g_position_servo.commandedRpm(),
                    g_position_servo.lastVelocityError(),
                    g_position_servo.pwmOutput());
    }

    g_move_pwm_out_abs_sum += pwm_out_abs;
    g_move_pwm_out_samples++;
    if (pwm_out_abs >= 99.0f) {
      g_move_pwm_out_sat_samples++;
    }
  }

  if (!motion.active && !g_move_done_reported) {
    g_move_done_reported = true;
    g_move_tracking_active = false;
    g_move_prev_sample_valid = false;
    g_move_debug_last_print_ms = 0;
    const uint32_t elapsed_ms = millis() - g_move_start_ms;
    // rpm_medio calculado a partir do odômetro interno do controlador (mais preciso
    // que acumulação de deltas externos, que perde progressão durante fase STOPPING).
    const float total_displacement_deg = g_move_start_accumulated_captured
                                           ? fabsf(g_position_servo.accumulatedDeg() - g_move_start_accumulated_deg)
                                           : fabsf(g_move_total_net_delta_deg);
    const float rpm_medio = (elapsed_ms > 0)
                              ? (total_displacement_deg * 60000.0f) / (360.0f * (float)elapsed_ms)
                              : 0.0f;
    if (g_move_peak_rpm_abs < 0.05f && rpm_medio > 0.05f) {
      const float peak_sign = (g_move_last_nonzero_rpm_signed < 0.0f) ? -1.0f : 1.0f;
      g_move_peak_rpm_abs = rpm_medio;
      g_move_peak_rpm_signed = peak_sign * rpm_medio;
    }
    const float pwm_adrc_pico_pct = clampf((float)g_move_peak_pwm_adrc_abs, 0.0f, 100.0f);
    const float pwm_out_pico_pct = (float)g_move_peak_pwm_out_abs;
    const float pwm_out_medio_pct = (g_move_pwm_out_samples > 0)
                                       ? (g_move_pwm_out_abs_sum / (float)g_move_pwm_out_samples)
                                       : 0.0f;
    const float pwm_out_sat_pct = (g_move_pwm_out_samples > 0)
                                     ? ((float)g_move_pwm_out_sat_samples * 100.0f / (float)g_move_pwm_out_samples)
                                     : 0.0f;
    const float ang_ini = g_move_start_accumulated_captured ? g_move_start_accumulated_deg : 0.0f;
    const float desl_signed = g_move_start_accumulated_captured
                                ? (g_position_servo.accumulatedDeg() - g_move_start_accumulated_deg)
                                : g_move_total_net_delta_deg;
    const float desl_abs = fabsf(desl_signed);
    Serial.printf("OK: alvo atingido em %.2f deg (ang_ini=%.2f deg, desl=%.2f deg, desl_abs=%.2f deg, tempo=%u ms, rpm_pico=%.2f, rpm_medio=%.2f, pwm_adrc_pico=%.1f%%, pwm_out_pico=%.1f%%, pwm_out_med=%.1f%%, pwm_sat=%.0f%%)\n",
                  current_deg, ang_ini, desl_signed, desl_abs, elapsed_ms, g_move_peak_rpm_signed, rpm_medio,
                  pwm_adrc_pico_pct, pwm_out_pico_pct, pwm_out_medio_pct, pwm_out_sat_pct);
  }
}

void stopMotorForOta() {
  g_sequence_motion.stop();
  g_run_when_sensor_detected = false;
  g_move_tracking_active = false;
  g_motion_coordinator.cancelMove();
}

bool setupOtaAndWebServices() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    g_ota_update_in_progress = true;
    stopMotorForOta();
    Serial.println("OTA: atualizacao iniciada; motores desativados");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA: atualizacao concluida; reiniciando");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    const unsigned int percent = total == 0 ? 0 : (progress * 100U) / total;
    Serial.printf("\rOTA: %u%%", percent);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("\nOTA: erro %u; reinicie a placa antes de operar o motor\n",
                  (unsigned int)error);
  });
  ArduinoOTA.begin();
  setupWebControl();

  return true;
}

bool setupOtaAccessPoint() {
  snprintf(OTA_AP_SSID, sizeof(OTA_AP_SSID), "Motor-Control-%02u",
           MOTOR_CONTROL_UNIT_NUMBER);
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(OTA_AP_SSID, OTA_AP_PASSWORD, WIFI_AP_CHANNEL)) {
    Serial.println("OTA: falha ao criar ponto de acesso");
    return false;
  }

  // HT40 pode fazer alguns scanners mostrarem o canal central (por exemplo 9
  // para um AP cujo canal primario e 11). Fixamos HT20 e reafirmamos o canal.
  if (!WiFi.softAPbandwidth(WIFI_BW_HT20)) {
    Serial.println("OTA/WEB: falha ao fixar largura WiFi em 20 MHz");
    WiFi.softAPdisconnect(true);
    return false;
  }
  const esp_err_t channel_result =
    esp_wifi_set_channel(WIFI_AP_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (channel_result != ESP_OK) {
    Serial.printf("OTA/WEB: falha ao fixar canal WiFi: %s\n",
                  esp_err_to_name(channel_result));
    WiFi.softAPdisconnect(true);
    return false;
  }

  uint8_t actual_channel = 0;
  wifi_second_chan_t actual_secondary = WIFI_SECOND_CHAN_NONE;
  wifi_bandwidth_t actual_bandwidth = WIFI_BW_HT20;
  const esp_err_t channel_read_result =
    esp_wifi_get_channel(&actual_channel, &actual_secondary);
  const esp_err_t bandwidth_read_result =
    esp_wifi_get_bandwidth(WIFI_IF_AP, &actual_bandwidth);
  if (channel_read_result != ESP_OK || bandwidth_read_result != ESP_OK ||
      actual_channel != WIFI_AP_CHANNEL || actual_secondary != WIFI_SECOND_CHAN_NONE ||
      actual_bandwidth != WIFI_BW_HT20) {
    Serial.printf("OTA/WEB: configuracao WiFi divergente (canal=%u secundario=%d largura=%d)\n",
                  actual_channel, (int)actual_secondary, (int)actual_bandwidth);
    WiFi.softAPdisconnect(true);
    return false;
  }

  setupOtaAndWebServices();

  Serial.printf("OTA/WEB AP: SSID=%s  canal=%u  largura=20MHz  IP=%s  OTA=3232 WEB=80\n",
                OTA_AP_SSID, actual_channel, WiFi.softAPIP().toString().c_str());
  return true;
}

bool setupStationOrAccessPoint() {
  snprintf(OTA_AP_SSID, sizeof(OTA_AP_SSID), "Motor-Control-%02u",
           MOTOR_CONTROL_UNIT_NUMBER);
  snprintf(OTA_HOSTNAME, sizeof(OTA_HOSTNAME), "as5600-motor-%02u",
           MOTOR_CONTROL_UNIT_NUMBER);

  if (strlen(WIFI_STA_SSID) > 0) {
    Serial.printf("WiFi: conectando a %s", WIFI_STA_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(OTA_HOSTNAME);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASSWORD);
    const uint32_t started_ms = millis();
    while (WiFi.status() != WL_CONNECTED &&
           millis() - started_ms < WIFI_STA_CONNECT_TIMEOUT_MS) {
      delay(250);
      Serial.print('.');
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      setupOtaAndWebServices();
      Serial.printf("OTA/WEB STA: SSID=%s  IP=%s  OTA=3232 WEB=80\n",
                    WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
      return true;
    }

    Serial.println("WiFi: rede nao encontrada; iniciando AP de contingencia");
    WiFi.disconnect(true);
    delay(100);
  } else {
    Serial.println("WiFi: credenciais STA ausentes; iniciando AP de contingencia");
  }

  return setupOtaAccessPoint();
}

void handleOtaMaintenanceMode() {
  if (g_ota_mode_active) {
    ArduinoOTA.handle();
    if (g_web_server_started) g_web_server.handleClient();
    return;
  }

  const bool button_pressed = digitalRead(OTA_BUTTON_PIN) == LOW;
  if (!button_pressed) {
    g_ota_button_pressed = false;
    return;
  }

  const uint32_t now = millis();
  if (!g_ota_button_pressed) {
    g_ota_button_pressed = true;
    g_ota_button_pressed_ms = now;
    return;
  }

  if (now - g_ota_button_pressed_ms < OTA_BUTTON_HOLD_MS) {
    return;
  }

  g_ota_mode_active = true;
  Serial.println("OTA/WEB: ativando AP; controle do motor permanece disponivel");
  if (!setupOtaAccessPoint()) {
    Serial.println("OTA/WEB: falha ao ativar AP");
  }
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

  AdrcPositionSettings pos_settings;

  // ADRC: valores iniciais extraidos do prototipo V2.5. A escala de b0 segue
  // PWM 8-bit internamente, embora a ponte H continue recebendo percentual.
  pos_settings.control_bandwidth = 25.0f;
  pos_settings.observer_bandwidth = 80.0f;
  pos_settings.plant_gain = 250.0f;
  
  pos_settings.max_target_rpm = DEFAULT_MAX_TARGET_RPM;
  pos_settings.physical_max_rpm = 3.0f;
  pos_settings.stop_window_deg = 1.0f;
  pos_settings.accel_ramp_ms = 250;
  pos_settings.decel_ramp_ms = 220;
  pos_settings.kick_pwm_percent = 85.0f;
  pos_settings.kick_ms = 180;
  pos_settings.samples_to_stop = 3;
  pos_settings.velocity_window_ms = 400;
  pos_settings.velocity_num_samples = 8;
  pos_settings.minimum_drive_pwm_percent = 24.0f;
  
  g_position_servo.setSettings(pos_settings);
  loadRepetitiveMotionPreferences();
  setPwmFrequencyHz(g_pwm_frequency_hz);

  g_angle_sensor.begin(Wire, I2C_SDA_PIN, I2C_SCL_PIN, 400000);

  pinMode(OTA_BUTTON_PIN, INPUT_PULLUP);

  WiFi.mode(WIFI_OFF);

  // Tenta a rede local primeiro. Se ela nao estiver disponivel, cria o AP de
  // contingencia. Somente uma transferencia OTA interrompe o motor.
  g_ota_mode_active = setupStationOrAccessPoint();

  Serial.println("\n=== Motor PWM Tester ===");
  Serial.println("Placa: Waveshare ESP32-S3-Zero  |  Motor padrao: IN3/IN4");
  Serial.printf("PWM: freq=%u Hz  resolucao=%u bits\n", g_pwm_frequency_hz, PWM_RESOLUTION_BITS);
  Serial.printf("I2C: SDA=%u SCL=%u\n", I2C_SDA_PIN, I2C_SCL_PIN);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi: conectado a %s  painel=http://%s/\n",
                  WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
  } else {
    Serial.printf("WiFi: AP de contingencia  SSID=%s  canal=%u  painel=http://192.168.4.1/\n",
                  OTA_AP_SSID, WIFI_AP_CHANNEL);
  }
  if (g_angle_sensor.active()) {
    Serial.printf("%s detectado no endereco 0x%02X\n",
                  g_angle_sensor.sensorName(), g_angle_sensor.sensorAddress());
  } else {
    Serial.println("Sensor angular nao detectado; procurando periodicamente");
  }
  Serial.println("ADRC pronto (motor nominal 2 rpm)");

  if (g_repetitive_run_on_boot) {
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
  handleOtaMaintenanceMode();

  // Mantem a serial exclusivamente para logs; qualquer entrada e descartada.
  while (Serial.available() > 0) Serial.read();
  const uint32_t now_ms = millis();
  g_motion_coordinator.updateSensor(now_ms);
  const uint32_t recovered_pause_ms =
    g_motion_coordinator.consumeRecoveredPauseMs();
  if (recovered_pause_ms > 0) {
    g_sequence_motion.resumeAfterPause(recovered_pause_ms);
    g_move_prev_sample_valid = false;
    g_move_rpm_window_delta_deg = 0.0f;
    g_move_rpm_window_dt_ms = 0;
  }
  if (g_run_when_sensor_detected &&
      g_motion_coordinator.status().sensor_available &&
      !g_ota_update_in_progress) {
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
  updateMotionDiagnostics(now_ms);

}
