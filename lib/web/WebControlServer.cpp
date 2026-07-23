#include "WebControlServer.h"

#include "BoundedInteger.h"
#include "control_settings_web_page.h"
#include "repetitive_motion_web_page.h"

#include <math.h>
#include <stdlib.h>

WebControlServer::WebControlServer(uint16_t port, WebControlActions& actions)
    : server_(port), actions_(actions) {}

bool WebControlServer::parseNumber(const char* name, float* value) const {
  if (!value || !server_.hasArg(name)) return false;
  const String text = server_.arg(name);
  char* end = nullptr;
  const float parsed = strtof(text.c_str(), &end);
  if (end == text.c_str() || !end || *end != '\0' || !isfinite(parsed)) {
    return false;
  }
  *value = parsed;
  return true;
}

bool WebControlServer::parseDirection(
    const String& text, MotionDirection* direction) const {
  if (!direction) return false;
  if (text == "shortest") *direction = MotionDirection::Shortest;
  else if (text == "cw") *direction = MotionDirection::Clockwise;
  else if (text == "ccw") *direction = MotionDirection::CounterClockwise;
  else return false;
  return true;
}

const char* WebControlServer::directionText(MotionDirection direction) const {
  switch (direction) {
    case MotionDirection::Clockwise: return "cw";
    case MotionDirection::CounterClockwise: return "ccw";
    case MotionDirection::Shortest: return "shortest";
  }
  return "shortest";
}

bool WebControlServer::parseSequenceStep(
    uint8_t index, float max_rpm, MotionStep* step) const {
  if (!step) return false;
  const String prefix = "s" + String(index) + "_";
  float target = 0.0f;
  float rpm = 0.0f;
  float dwell = 0.0f;
  if (!parseNumber((prefix + "target").c_str(), &target) ||
      !parseNumber((prefix + "rpm").c_str(), &rpm) ||
      !parseNumber((prefix + "dwell").c_str(), &dwell) ||
      !server_.hasArg(prefix + "direction")) {
    return false;
  }
  MotionDirection direction;
  uint32_t dwell_ms = 0;
  if (!parseDirection(server_.arg(prefix + "direction"), &direction) ||
      target < 0.0f || target >= 360.0f || rpm < 0.1f || rpm > max_rpm ||
      !BoundedInteger::roundToUint32(dwell, 0, 3600000, &dwell_ms)) {
    return false;
  }
  *step = {target, rpm, dwell_ms, direction};
  return true;
}

void WebControlServer::appendSequenceStepJson(
    String& json, const MotionStep& step) const {
  json += F("{\"target\":"); json += String(step.target_deg, 2);
  json += F(",\"rpm\":"); json += String(step.rpm, 2);
  json += F(",\"dwell\":"); json += String(step.dwell_ms);
  json += F(",\"direction\":\""); json += directionText(step.direction);
  json += F("\"}");
}

void WebControlServer::sendError(int status, const char* message) {
  String json = F("{\"error\":\"");
  json += message;
  json += F("\"}");
  server_.send(status, "application/json", json);
}

void WebControlServer::sendOperationError(const OperationResult& result) {
  sendError(result.http_status, result.message);
}

void WebControlServer::sendStatus(int status_code) {
  const ApplicationStatus status = actions_.status();
  String json;
  json.reserve(420);
  json += F("{\"unit\":"); json += String(status.unit);
  json += F(",\"running\":"); json += status.running ? F("true") : F("false");
  json += F(",\"moveActive\":"); json += status.move_active ? F("true") : F("false");
  json += F(",\"phase\":\""); json += status.phase;
  json += F("\",\"step\":"); json += String(status.step);
  json += F(",\"sensor\":"); json += status.sensor_available ? F("true") : F("false");
  json += F(",\"sensorType\":\""); json += status.sensor_type;
  json += F("\",\"sensorAddress\":"); json += String(status.sensor_address);
  json += F(",\"sensorState\":\""); json += status.sensor_state;
  json += F("\",\"sensorFailures\":"); json += String(status.sensor_failures);
  json += F(",\"angle\":"); json += String(status.angle_deg, 2);
  json += F(",\"maxRpm\":"); json += String(status.max_rpm, 2);
  json += F(",\"otaBusy\":"); json += status.ota_busy ? F("true") : F("false");
  json += F(",\"stall\":"); json += status.stalled ? F("true") : F("false");
  json += '}';
  server_.send(status_code, "application/json", json);
}

void WebControlServer::sendSettings() {
  const ApplicationStatus status = actions_.status();
  const DeviceSettings settings = actions_.settings();
  const AdrcPositionSettings& s = settings.position;
  String json;
  json.reserve(360);
  json += F("{\"canEdit\":");
  json += (!status.running && !status.move_active && !status.ota_busy)
            ? F("true") : F("false");
  json += F(",\"wc\":"); json += String(s.control_bandwidth, 2);
  json += F(",\"wo\":"); json += String(s.observer_bandwidth, 2);
  json += F(",\"b0\":"); json += String(s.plant_gain, 2);
  json += F(",\"maxRpm\":"); json += String(s.max_target_rpm, 2);
  json += F(",\"physRpm\":"); json += String(s.physical_max_rpm, 2);
  json += F(",\"powerLimit\":"); json += String(settings.motor.power_limit_percent);
  json += F(",\"pwmHz\":"); json += String(settings.motor.pwm_frequency_hz);
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
  json += F(",\"sensorFailures\":"); json += String(settings.sensor.failure_limit);
  json += '}';
  server_.send(200, "application/json", json);
}

void WebControlServer::sendSequence() {
  const ApplicationStatus status = actions_.status();
  const DeviceSettings settings = actions_.settings();
  const MotionSequenceConfig& config = settings.sequence;
  String json;
  json.reserve(1900);
  json += F("{\"canEdit\":");
  json += (!status.running && !status.move_active && !status.ota_busy)
            ? F("true") : F("false");
  json += F(",\"maxRpm\":"); json += String(settings.position.max_target_rpm, 2);
  json += F(",\"startup\":");
  appendSequenceStepJson(json, config.startup_step);
  json += F(",\"steps\":[");
  for (uint8_t i = 0; i < config.step_count; ++i) {
    if (i) json += ',';
    appendSequenceStepJson(json, config.steps[i]);
  }
  json += F("]}");
  server_.send(200, "application/json", json);
}

void WebControlServer::begin() {
  if (started_) return;

  server_.on("/", HTTP_GET, [this]() {
    server_.send_P(200, "text/html; charset=utf-8", REPETITIVE_MOTION_WEB_PAGE);
  });
  server_.on("/settings", HTTP_GET, [this]() {
    server_.send_P(200, "text/html; charset=utf-8", CONTROL_SETTINGS_WEB_PAGE);
  });
  server_.on("/api/status", HTTP_GET, [this]() { sendStatus(); });
  server_.on("/api/settings", HTTP_GET, [this]() { sendSettings(); });
  server_.on("/api/settings", HTTP_POST, [this]() {
    const ApplicationStatus status = actions_.status();
    if (status.running || status.move_active || status.ota_busy) {
      sendError(409, "Pare o motor antes de alterar os ajustes");
      return;
    }
    float wc, wo, b0, max_rpm, phys_rpm, power_limit, pwm_hz;
    float stop_window, stop_samples, accel_ms, decel_ms, kick_pct, kick_ms;
    float min_pwm, stall_ms, stall_vel, vel_window, vel_samples, sensor_failures;
    if (!parseNumber("wc", &wc) || !parseNumber("wo", &wo) ||
        !parseNumber("b0", &b0) || !parseNumber("maxRpm", &max_rpm) ||
        !parseNumber("physRpm", &phys_rpm) ||
        !parseNumber("powerLimit", &power_limit) || !parseNumber("pwmHz", &pwm_hz) ||
        !parseNumber("stopWindow", &stop_window) ||
        !parseNumber("stopSamples", &stop_samples) ||
        !parseNumber("accelMs", &accel_ms) || !parseNumber("decelMs", &decel_ms) ||
        !parseNumber("kickPct", &kick_pct) || !parseNumber("kickMs", &kick_ms) ||
        !parseNumber("minPwm", &min_pwm) || !parseNumber("stallMs", &stall_ms) ||
        !parseNumber("stallVel", &stall_vel) ||
        !parseNumber("velWindow", &vel_window) ||
        !parseNumber("velSamples", &vel_samples) ||
        !parseNumber("sensorFailures", &sensor_failures)) {
      sendError(400, "Parametros invalidos ou incompletos");
      return;
    }
    uint32_t stop_samples_value, accel_ms_value, decel_ms_value;
    uint32_t kick_ms_value, stall_ms_value, vel_window_value;
    uint32_t vel_samples_value, power_limit_value, pwm_hz_value;
    uint32_t sensor_failures_value;
    if (!BoundedInteger::roundToUint32(stop_samples, 1, 20,
                                       &stop_samples_value) ||
        !BoundedInteger::roundToUint32(accel_ms, 50, 2000,
                                       &accel_ms_value) ||
        !BoundedInteger::roundToUint32(decel_ms, 50, 2000,
                                       &decel_ms_value) ||
        !BoundedInteger::roundToUint32(kick_ms, 0, 1000, &kick_ms_value) ||
        !BoundedInteger::roundToUint32(stall_ms, 100, 10000,
                                       &stall_ms_value) ||
        !BoundedInteger::roundToUint32(vel_window, 20, 1000,
                                       &vel_window_value) ||
        !BoundedInteger::roundToUint32(vel_samples, 2, 20,
                                       &vel_samples_value) ||
        !BoundedInteger::roundToUint32(power_limit, 0, 100,
                                       &power_limit_value) ||
        !BoundedInteger::roundToUint32(pwm_hz, 500, 20000,
                                       &pwm_hz_value) ||
        !BoundedInteger::roundToUint32(sensor_failures, 1, 20,
                                       &sensor_failures_value)) {
      sendError(422, "Valor inteiro fora da faixa permitida");
      return;
    }
    DeviceSettings candidate = actions_.settings();
    AdrcPositionSettings& p = candidate.position;
    p.control_bandwidth = wc;
    p.observer_bandwidth = wo;
    p.plant_gain = b0;
    p.max_target_rpm = max_rpm;
    p.physical_max_rpm = phys_rpm;
    p.stop_window_deg = stop_window;
    p.samples_to_stop = (uint16_t)stop_samples_value;
    p.accel_ramp_ms = (uint16_t)accel_ms_value;
    p.decel_ramp_ms = (uint16_t)decel_ms_value;
    p.kick_pwm_percent = kick_pct;
    p.kick_ms = (uint16_t)kick_ms_value;
    p.minimum_drive_pwm_percent = min_pwm;
    p.stall_timeout_ms = (uint16_t)stall_ms_value;
    p.stall_velocity_deg_s = stall_vel;
    p.velocity_window_ms = (uint16_t)vel_window_value;
    p.velocity_num_samples = (uint8_t)vel_samples_value;
    candidate.motor.power_limit_percent = (uint8_t)power_limit_value;
    candidate.motor.pwm_frequency_hz = pwm_hz_value;
    candidate.sensor.failure_limit = (uint8_t)sensor_failures_value;
    candidate.sequence.startup_step.rpm =
      fminf(candidate.sequence.startup_step.rpm, max_rpm);
    for (uint8_t i = 0; i < candidate.sequence.step_count; ++i) {
      candidate.sequence.steps[i].rpm =
        fminf(candidate.sequence.steps[i].rpm, max_rpm);
    }
    if (!validateDeviceSettings(candidate)) {
      sendError(422, "Valor fora da faixa ou RPM maxima acima da RPM fisica");
      return;
    }
    const OperationResult result = actions_.replaceSettings(candidate);
    if (!result.ok) { sendOperationError(result); return; }
    sendSettings();
  });

  server_.on("/api/sequence", HTTP_GET, [this]() { sendSequence(); });
  server_.on("/api/sequence", HTTP_POST, [this]() {
    const ApplicationStatus status = actions_.status();
    if (status.running || status.move_active || status.ota_busy) {
      sendError(409, "Pare o motor antes de alterar a sequencia");
      return;
    }
    float count_value = 0.0f;
    if (!parseNumber("count", &count_value) || count_value < 1.0f ||
        count_value > MOTION_SEQUENCE_MAX_STEPS || count_value != floorf(count_value)) {
      sendError(422, "Quantidade de passos invalida");
      return;
    }
    DeviceSettings candidate = actions_.settings();
    candidate.sequence.step_count = (uint8_t)count_value;
    if (!parseSequenceStep(0, candidate.position.max_target_rpm,
                           &candidate.sequence.startup_step)) {
      sendError(422, "Passo inicial invalido");
      return;
    }
    for (uint8_t i = 1; i <= candidate.sequence.step_count; ++i) {
      if (!parseSequenceStep(i, candidate.position.max_target_rpm,
                             &candidate.sequence.steps[i - 1])) {
        sendError(422, "Passo do ciclo invalido");
        return;
      }
    }
    const OperationResult result = actions_.replaceSettings(candidate);
    if (!result.ok) { sendOperationError(result); return; }
    sendSequence();
  });

  server_.on("/api/run", HTTP_POST, [this]() {
    if (!server_.hasArg("running")) {
      sendError(400, "Parametro running ausente");
      return;
    }
    const bool requested = server_.arg("running") == "1";
    const OperationResult result = actions_.setRunning(requested);
    if (!result.ok) { sendOperationError(result); return; }
    sendStatus();
  });

  server_.on("/api/adjust", HTTP_POST, [this]() {
    const ApplicationStatus status = actions_.status();
    if (status.running || status.move_active) {
      sendError(409, "Ajuste permitido somente com o motor parado");
      return;
    }
    if (status.ota_busy) { sendError(409, "Atualizacao OTA em andamento"); return; }
    if (!status.sensor_available) { sendError(409, "Sensor angular nao detectado"); return; }
    if (!server_.hasArg("target")) { sendError(400, "Destino ausente"); return; }
    const String target = server_.arg("target");
    if (target != "start" && target != "end") {
      sendError(400, "Destino invalido");
      return;
    }
    const OperationResult result = actions_.adjustTo(target == "start");
    if (!result.ok) { sendOperationError(result); return; }
    sendStatus();
  });

  server_.on("/api/manual-move", HTTP_POST, [this]() {
    const ApplicationStatus status = actions_.status();
    if (status.running || status.move_active || status.ota_busy) {
      sendError(409, "Movimento avulso permitido somente com o motor parado");
      return;
    }
    if (!status.sensor_available) { sendError(409, "Sensor angular nao detectado"); return; }
    float target_deg = 0.0f;
    float rpm = 0.0f;
    if (!parseNumber("target", &target_deg) || !parseNumber("rpm", &rpm)) {
      sendError(400, "Destino ou RPM invalido");
      return;
    }
    if (target_deg < 0.0f || target_deg >= 360.0f ||
        rpm < 0.1f || rpm > status.max_rpm) {
      sendError(422, "Destino ou RPM fora da faixa permitida");
      return;
    }
    const OperationResult result = actions_.manualMove(
      {target_deg, rpm, MotionDirection::Shortest});
    if (!result.ok) { sendOperationError(result); return; }
    sendStatus();
  });

  server_.on("/api/config", HTTP_POST, [this]() {
    const ApplicationStatus status = actions_.status();
    if (status.running || status.move_active || status.ota_busy) {
      sendError(409, "Pare o motor antes de alterar a sequencia");
      return;
    }
    float start, end, rpm_out, rpm_back, wait_start, wait_end;
    if (!parseNumber("start", &start) || !parseNumber("end", &end) ||
        !parseNumber("rpmOut", &rpm_out) || !parseNumber("rpmBack", &rpm_back) ||
        !parseNumber("waitStart", &wait_start) || !parseNumber("waitEnd", &wait_end)) {
      sendError(400, "Parametros invalidos ou incompletos");
      return;
    }
    uint32_t wait_start_ms = 0;
    uint32_t wait_end_ms = 0;
    if (!BoundedInteger::roundToUint32(wait_start, 0, 3600000,
                                       &wait_start_ms) ||
        !BoundedInteger::roundToUint32(wait_end, 0, 3600000,
                                       &wait_end_ms)) {
      sendError(422, "Intervalo fora da faixa permitida");
      return;
    }
    DeviceSettings candidate = actions_.settings();
    candidate.sequence.startup_step = {
      start, rpm_back, wait_start_ms, MotionDirection::Shortest};
    candidate.sequence.step_count = 2;
    candidate.sequence.steps[0] = {
      end, rpm_out, wait_end_ms, MotionDirection::Clockwise};
    candidate.sequence.steps[1] = {
      start, rpm_back, wait_start_ms, MotionDirection::CounterClockwise};
    if (!validateDeviceSettings(candidate)) {
      sendError(422, "Valor fora da faixa permitida");
      return;
    }
    const OperationResult result = actions_.replaceSettings(candidate);
    if (!result.ok) { sendOperationError(result); return; }
    sendStatus();
  });

  server_.onNotFound([this]() {
    server_.sendHeader("Location", "/", true);
    server_.send(302, "text/plain", "");
  });
  server_.begin();
  started_ = true;
}

void WebControlServer::handleClient() {
  if (started_) server_.handleClient();
}
