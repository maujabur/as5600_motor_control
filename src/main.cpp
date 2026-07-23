#include <Arduino.h>

#include <AngleMath.h>
#include <AngleSensorManager.h>
#include <AdrcPositionController.h>
#include <HBridgeMotorDriver.h>
#include <MotionCoordinator.h>
#include <MotionSequenceController.h>
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

void saveRepetitiveMotionConfig() {
  g_settings.sequence = g_sequence_motion.config();
  saveSettingsSnapshot();
}

void saveAdrcSettings() {
  g_settings.position = g_position_servo.settings();
  saveSettingsSnapshot();
}

void saveControlSettings() {
  g_settings.position = g_position_servo.settings();
  g_settings.sequence = g_sequence_motion.config();
  g_settings.motor = g_motor_driver.settings();
  g_settings.sensor.failure_limit = g_angle_sensor.failureLimit();
  saveSettingsSnapshot();
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
  g_move_done_reported = false;
  g_move_start_ms = millis();
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
  g_move_done_reported = false;
  g_move_start_ms = millis();
  return {true, 200, ""};
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
  // Mantem a serial exclusivamente para logs; qualquer entrada e descartada.
  while (Serial.available() > 0) Serial.read();
  const uint32_t now_ms = millis();
  g_network.update(now_ms);
  g_web_server.handleClient();
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
  updateMotionDiagnostics(now_ms);

}
