/***************************************************
  AS7265X dual-sensor outdoor NDVI meter (ESP32)

  - Bulbs OFF (ambient light)
  - Uses channels I (red-ish) and V (NIR-ish) for NDVI
  - UP sensor: incident light (sky) on Wire1 (default SDA/SCL)
  - DOWN sensor: reflected light (turf) on Wire (custom GPIOs on PCB)
  - Shows NDVI + battery % on ST7789 240x240 LCD (Pimoroni 1.3")
  - Measures when BTN_MEASURE is pressed OR serial 'm' received
  - Single measurement per press: UP then DOWN, ratio-based NDVI
  - No grey-panel calibration — reflectance ratios used directly

  Serial commands (115200 baud):
    m  or  M  — trigger a measurement (newline also accepted)

  IMPORTANT (Pimoroni 1.3" ST7789 240x240 SPI):
  - No RST pin -> TFT_RST = -1
  - BL is backlight enable, drive separately

  Hot-plug behaviour:
  - Sketch will boot if sensors are missing
  - It keeps retrying init in the background

  REQUIRES: SparkFun AS7265X library patched to accept a TwoWire&
  argument in begin(). The stock library always uses Wire; without
  this patch both sensors share the same bus and collide.
****************************************************/

#include "SparkFun_AS7265X.h"
#include "as7265x_calibration_data.h"
#include <Wire.h>
#include <WiFi.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ---------------- I2C ----------------
// DOWN sensor on custom pins (cannot change due to PCB)
#define I2C1_SDA_PIN 33
#define I2C1_SCL_PIN 13

// UP sensor on default SDA/SCL (Wire1)
#define I2C2_SDA_PIN SDA
#define I2C2_SCL_PIN SCL

AS7265X sensor_DOWN;
AS7265X sensor_UP;

Preferences calibration_preferences;

bool up_ok   = false;
bool down_ok = false;

// Retry logic
unsigned long last_sensor_retry_ms = 0;
const unsigned long SENSOR_RETRY_PERIOD_MS = 1000;

// ---------------- ST7789 240x240 LCD ----------------
#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 240

#define TFT_CS  A0
#define TFT_DC  A1
#define TFT_RST -1
#define TFT_BL  A5

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// Colors (565)
#define BLACK  0x0000
#define GREEN  0x07E0
#define RED    0xF800
#define YELLOW 0xFFE0
#define CYAN   0x07FF
#define GREY   0x8410
#define WHITE  0xFFFF

// ---------------- Button ----------------
#define BTN_MEASURE A2
bool lastMeasureState = false;

static constexpr const char *CALIBRATION_FW_ID = "Manti-S-Cal";
static constexpr const char *CALIBRATION_NAMESPACE = "mantiscal";
static constexpr const char *CALIBRATION_KEY = "blob";
static constexpr uint32_t CALIBRATION_MAGIC = 0x4D534341UL;
static constexpr uint16_t CALIBRATION_VERSION = 2;
static constexpr size_t SERIAL_COMMAND_MAX_TOKENS =
  (4 + (2 * AS7265X_CAL_POINT_COUNT) > 2 + AS7265X_CHANNEL_COUNT)
    ? 4 + (2 * AS7265X_CAL_POINT_COUNT)
    : 2 + AS7265X_CHANNEL_COUNT;
static constexpr size_t RED_CHANNEL_INDEX = 9;
static constexpr size_t NIR_CHANNEL_INDEX = 14;

static const char *CHANNEL_LABELS[AS7265X_CHANNEL_COUNT] = {
  "A", "B", "C", "D", "E", "F", "G", "H", "R",
  "I", "S", "J", "T", "U", "V", "W", "K", "L"
};

static const uint16_t CHANNEL_WAVELENGTHS[AS7265X_CHANNEL_COUNT] = {
  410, 435, 460, 485, 510, 535, 560, 585, 610,
  645, 680, 705, 730, 760, 810, 860, 900, 940
};

static float UP_DARK_OFFSETS[AS7265X_CHANNEL_COUNT] = {0.0f};
static float DOWN_DARK_OFFSETS[AS7265X_CHANNEL_COUNT] = {0.0f};
static bool calibration_loaded_from_flash = false;

struct CalibrationBlob {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  float up_inputs[AS7265X_CHANNEL_COUNT][AS7265X_CAL_POINT_COUNT];
  float up_factors[AS7265X_CHANNEL_COUNT][AS7265X_CAL_POINT_COUNT];
  float down_inputs[AS7265X_CHANNEL_COUNT][AS7265X_CAL_POINT_COUNT];
  float down_factors[AS7265X_CHANNEL_COUNT][AS7265X_CAL_POINT_COUNT];
  float up_dark_offsets[AS7265X_CHANNEL_COUNT];
  float down_dark_offsets[AS7265X_CHANNEL_COUNT];
};

struct SensorCalibrationView {
  const char *name;
  AS7265X *sensor;
  bool *ready_flag;
  float (*inputs)[AS7265X_CAL_POINT_COUNT];
  float (*factors)[AS7265X_CAL_POINT_COUNT];
  float *dark_offsets;
};

// ---------------- Battery ----------------
#define BATT A3

// ADC correction factor measured empirically for this board.
// Multiply the raw ESP32 ADC voltage by this to get true voltage.
// Re-measure with a multimeter if the board changes.
#define ADC_CORRECTION_FACTOR 1.065f

float batt_v_avg = 0.0f;
int battery_soc  = 0;

// Battery is sampled periodically in the main loop.
unsigned long last_batt_sample_ms = 0;
const unsigned long BATT_SAMPLE_PERIOD_MS = 5000;

static const float voltagePoints[] = {
  3.30f, 3.45f, 3.55f, 3.68f, 3.73f, 3.79f,
  3.85f, 3.92f, 4.00f, 4.08f, 4.12f, 4.20f
};

static const float socPoints[] = {
   0,  10,  20,  30,  40,  50,
  60,  70,  80,  90,  95, 100
};

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float voltageToPercent(float v) {
  const int N = sizeof(voltagePoints) / sizeof(voltagePoints[0]);
  if (v <= voltagePoints[0])     return 0.0f;
  if (v >= voltagePoints[N - 1]) return 100.0f;
  for (int i = 0; i < N - 1; i++) {
    if (v >= voltagePoints[i] && v <= voltagePoints[i + 1]) {
      float t = (v - voltagePoints[i]) / (voltagePoints[i + 1] - voltagePoints[i]);
      return socPoints[i] + t * (socPoints[i + 1] - socPoints[i]);
    }
  }
  return 0.0f;
}

void configure_batt() {
  pinMode(BATT, INPUT);
  analogSetPinAttenuation(BATT, ADC_11db);
}

int read_voltage_percent() {
  uint16_t raw = analogRead(BATT);
  float voltage = (raw / 4095.0f) * 3.3f * ADC_CORRECTION_FACTOR * 2.0f;

  if (batt_v_avg == 0.0f) {
    batt_v_avg = voltage;
  } else {
    batt_v_avg = 0.8f * batt_v_avg + 0.2f * voltage;
  }

  return (int)clampf(voltageToPercent(batt_v_avg), 0.0f, 100.0f);
}

// ---------------- NDVI ----------------
String message_NDVI = "0.00";
float ndvi = 0.0f;
bool last_measurement_low_light = false;
uint32_t measurement_sequence = 0;
unsigned long last_measurement_ms = 0;

// Raw count threshold below which incident light is too low to trust.
const float MIN_INCIDENT_SIGNAL = 75.0f;  // raw counts (0-65535)
const float UP_RED_FROM_NIR_SLOPE = 6.3534f;
const float UP_RED_FROM_NIR_INTERCEPT = -2571.7f;

// Last raw incident readings — used for low-light heuristic.
float last_red_UP = 0.0f;
float last_nir_UP = 0.0f;

// Last calibrated readings for all four channels — displayed on the NDVI screen.
float last_calc_iU = 0.0f;
float last_calc_vU = 0.0f;
float last_calc_iD = 0.0f;
float last_calc_vD = 0.0f;

// ---------------- BLE ----------------
static const char *BLE_DEVICE_NAME  = "Manti-S-00001";
static const char *BLE_SERVICE_UUID = "9f4a6d90-f740-4d10-a947-8c6c20f4f9d1";
static const char *BLE_STATUS_UUID  = "2d86686f-34f8-4f1b-a4ac-f127f90f7df0";

BLEServer         *ble_server                = nullptr;
BLEService        *ble_service               = nullptr;
BLECharacteristic *ble_status_characteristic = nullptr;

static SemaphoreHandle_t ble_mutex = nullptr;
static bool _ble_connected         = false;
static bool _ble_state_dirty       = false;
static bool _ble_ui_dirty          = false;

bool ble_was_connected = false;

static inline void ble_set_connected(bool v) {
  xSemaphoreTake(ble_mutex, portMAX_DELAY);
  _ble_connected   = v;
  _ble_state_dirty = true;
  _ble_ui_dirty    = true;
  xSemaphoreGive(ble_mutex);
}
static inline bool ble_take_state_dirty() {
  xSemaphoreTake(ble_mutex, portMAX_DELAY);
  bool d = _ble_state_dirty;
  _ble_state_dirty = false;
  xSemaphoreGive(ble_mutex);
  return d;
}
static inline bool ble_take_ui_dirty() {
  xSemaphoreTake(ble_mutex, portMAX_DELAY);
  bool d = _ble_ui_dirty;
  _ble_ui_dirty = false;
  xSemaphoreGive(ble_mutex);
  return d;
}
static inline bool ble_is_connected() {
  xSemaphoreTake(ble_mutex, portMAX_DELAY);
  bool v = _ble_connected;
  xSemaphoreGive(ble_mutex);
  return v;
}

enum ScreenMode {
  SCREEN_MODE_STATUS,
  SCREEN_MODE_NDVI,
  SCREEN_MODE_MEASURING
};

ScreenMode current_screen_mode  = SCREEN_MODE_STATUS;
int current_measuring_sample    = 0;
int current_measuring_total     = 1;

class MantisBLEServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) {
    (void)server;
    ble_set_connected(true);
  }
  void onDisconnect(BLEServer *server) {
    ble_set_connected(false);
    server->startAdvertising();
  }
};

// ---------------- Forward declarations ----------------
void drawHeaderStatus();
void drawStatusScreen();
void drawNDVIScreen();
void drawMeasuringScreen(int currentSample, int totalSamples);
void redrawCurrentScreen();
void setupBLE();
void serviceBLE();
void updateBLEStatusValue();
bool poll_serial_commands();

bool get_sensor_calibration_view(const char *sensor_name, SensorCalibrationView &view) {
  if (strcmp(sensor_name, "up") == 0) {
    view = {"up", &sensor_UP, &up_ok, UP_CAL_INPUTS, UP_CAL_FACTORS, UP_DARK_OFFSETS};
    return true;
  }
  if (strcmp(sensor_name, "down") == 0) {
    view = {"down", &sensor_DOWN, &down_ok, DOWN_CAL_INPUTS, DOWN_CAL_FACTORS, DOWN_DARK_OFFSETS};
    return true;
  }
  return false;
}

void fill_calibration_blob(CalibrationBlob &blob) {
  blob.magic = CALIBRATION_MAGIC;
  blob.version = CALIBRATION_VERSION;
  blob.reserved = 0;
  memcpy(blob.up_inputs, UP_CAL_INPUTS, sizeof(UP_CAL_INPUTS));
  memcpy(blob.up_factors, UP_CAL_FACTORS, sizeof(UP_CAL_FACTORS));
  memcpy(blob.down_inputs, DOWN_CAL_INPUTS, sizeof(DOWN_CAL_INPUTS));
  memcpy(blob.down_factors, DOWN_CAL_FACTORS, sizeof(DOWN_CAL_FACTORS));
  memcpy(blob.up_dark_offsets, UP_DARK_OFFSETS, sizeof(UP_DARK_OFFSETS));
  memcpy(blob.down_dark_offsets, DOWN_DARK_OFFSETS, sizeof(DOWN_DARK_OFFSETS));
}

void apply_calibration_blob(const CalibrationBlob &blob) {
  memcpy(UP_CAL_INPUTS, blob.up_inputs, sizeof(UP_CAL_INPUTS));
  memcpy(UP_CAL_FACTORS, blob.up_factors, sizeof(UP_CAL_FACTORS));
  memcpy(DOWN_CAL_INPUTS, blob.down_inputs, sizeof(DOWN_CAL_INPUTS));
  memcpy(DOWN_CAL_FACTORS, blob.down_factors, sizeof(DOWN_CAL_FACTORS));
  memcpy(UP_DARK_OFFSETS, blob.up_dark_offsets, sizeof(UP_DARK_OFFSETS));
  memcpy(DOWN_DARK_OFFSETS, blob.down_dark_offsets, sizeof(DOWN_DARK_OFFSETS));
}

bool load_calibration_from_flash() {
  CalibrationBlob blob;
  if (!calibration_preferences.begin(CALIBRATION_NAMESPACE, true)) return false;
  size_t stored_size = calibration_preferences.getBytesLength(CALIBRATION_KEY);
  if (stored_size != sizeof(CalibrationBlob)) {
    calibration_preferences.end();
    return false;
  }
  size_t read_size = calibration_preferences.getBytes(CALIBRATION_KEY, &blob, sizeof(blob));
  calibration_preferences.end();
  if (read_size != sizeof(blob)) return false;
  if (blob.magic != CALIBRATION_MAGIC || blob.version != CALIBRATION_VERSION) return false;
  apply_calibration_blob(blob);
  calibration_loaded_from_flash = true;
  return true;
}

bool save_calibration_to_flash() {
  CalibrationBlob blob;
  fill_calibration_blob(blob);
  if (!calibration_preferences.begin(CALIBRATION_NAMESPACE, false)) return false;
  size_t written = calibration_preferences.putBytes(CALIBRATION_KEY, &blob, sizeof(blob));
  calibration_preferences.end();
  if (written != sizeof(blob)) return false;
  calibration_loaded_from_flash = true;
  return true;
}

const char *calibration_source_name() {
  return calibration_loaded_from_flash ? "flash" : "header";
}

void read_sensor_channels(AS7265X &sensor, uint16_t values[AS7265X_CHANNEL_COUNT]) {
  values[0] = sensor.getA();
  values[1] = sensor.getB();
  values[2] = sensor.getC();
  values[3] = sensor.getD();
  values[4] = sensor.getE();
  values[5] = sensor.getF();
  values[6] = sensor.getG();
  values[7] = sensor.getH();
  values[8] = sensor.getR();
  values[9] = sensor.getI();
  values[10] = sensor.getS();
  values[11] = sensor.getJ();
  values[12] = sensor.getT();
  values[13] = sensor.getU();
  values[14] = sensor.getV();
  values[15] = sensor.getW();
  values[16] = sensor.getK();
  values[17] = sensor.getL();
}

float interpolate_calibration_factor(const float inputs[AS7265X_CAL_POINT_COUNT], const float factors[AS7265X_CAL_POINT_COUNT], float value) {
  if (value <= inputs[0]) return factors[0];

  const size_t last = AS7265X_CAL_POINT_COUNT - 1;
  if (value >= inputs[last]) {
    float x1 = inputs[last - 1];
    float x2 = inputs[last];
    float y1 = factors[last - 1];
    float y2 = factors[last];
    if (fabsf(x2 - x1) < 1e-6f) return y2;
    return y1 + ((value - x1) / (x2 - x1)) * (y2 - y1);
  }

  for (size_t idx = 1; idx < AS7265X_CAL_POINT_COUNT; ++idx) {
    if (value <= inputs[idx]) {
      float x1 = inputs[idx - 1];
      float x2 = inputs[idx];
      float y1 = factors[idx - 1];
      float y2 = factors[idx];
      if (fabsf(x2 - x1) < 1e-6f) return y2;
      return y1 + ((value - x1) / (x2 - x1)) * (y2 - y1);
    }
  }

  return factors[last];
}

float apply_channel_calibration(const SensorCalibrationView &view, size_t channel_index, float raw_value) {
  float input_value = raw_value - view.dark_offsets[channel_index];
  if (input_value < 0.0f) input_value = 0.0f;
  float factor = interpolate_calibration_factor(view.inputs[channel_index], view.factors[channel_index], input_value);
  return input_value * factor;
}

void print_status_block() {
  Serial.println("BEGIN_STATUS");
  Serial.print("fw=");
  Serial.println(CALIBRATION_FW_ID);
  Serial.print("calibration_source=");
  Serial.println(calibration_source_name());
  Serial.print("up_sensor=");
  Serial.println(up_ok ? "ready" : "missing");
  Serial.print("down_sensor=");
  Serial.println(down_ok ? "ready" : "missing");
  Serial.println("END_STATUS");
}

void print_calibration_status() {
  Serial.print("Calibration source: ");
  Serial.println(calibration_source_name());
}

void print_caldump(const SensorCalibrationView &view) {
  Serial.print("BEGIN_CALDUMP,");
  Serial.println(view.name);
  for (size_t channel = 0; channel < AS7265X_CHANNEL_COUNT; ++channel) {
    Serial.print(CHANNEL_LABELS[channel]);
    for (size_t point = 0; point < AS7265X_CAL_POINT_COUNT; ++point) {
      Serial.print(",");
      Serial.print(view.inputs[channel][point], 6);
    }
    for (size_t point = 0; point < AS7265X_CAL_POINT_COUNT; ++point) {
      Serial.print(",");
      Serial.print(view.factors[channel][point], 6);
    }
    Serial.println();
  }
  Serial.print("END_CALDUMP,");
  Serial.println(view.name);
}

void print_caldarkdump(const SensorCalibrationView &view) {
  Serial.print("BEGIN_CALDARK,");
  Serial.println(view.name);
  for (size_t channel = 0; channel < AS7265X_CHANNEL_COUNT; ++channel) {
    Serial.print(CHANNEL_LABELS[channel]);
    Serial.print(",");
    Serial.print(CHANNEL_WAVELENGTHS[channel]);
    Serial.print(",");
    Serial.println(view.dark_offsets[channel], 6);
  }
  Serial.print("END_CALDARK,");
  Serial.println(view.name);
}

void print_raw_channels_legacy(const char *label, const uint16_t values[AS7265X_CHANNEL_COUNT]) {
  Serial.println(label);
  for (size_t channel = 0; channel < AS7265X_CHANNEL_COUNT; ++channel) {
    Serial.print(CHANNEL_LABELS[channel]);
    Serial.print(" (");
    Serial.print(CHANNEL_WAVELENGTHS[channel]);
    Serial.print(" nm): ");
    Serial.println(values[channel]);
  }
}

bool parse_float_token(const char *token, float &value) {
  char *end = nullptr;
  value = strtof(token, &end);
  return end != token && end != nullptr && *end == '\0';
}

bool parse_int_token(const char *token, int &value) {
  char *end = nullptr;
  long parsed = strtol(token, &end, 10);
  if (end == token || end == nullptr || *end != '\0') return false;
  value = (int)parsed;
  return true;
}

void handle_calwrite_command(char *tokens[], size_t token_count) {
  if (token_count != (3 + (2 * AS7265X_CAL_POINT_COUNT))) {
    Serial.println("ERR calwrite usage");
    return;
  }

  SensorCalibrationView view;
  if (!get_sensor_calibration_view(tokens[1], view)) {
    Serial.println("ERR calwrite sensor");
    return;
  }

  int channel_index = -1;
  if (!parse_int_token(tokens[2], channel_index) || channel_index < 0 || channel_index >= (int)AS7265X_CHANNEL_COUNT) {
    Serial.println("ERR calwrite channel");
    return;
  }

  for (size_t point = 0; point < AS7265X_CAL_POINT_COUNT; ++point) {
    float parsed = 0.0f;
    if (!parse_float_token(tokens[3 + point], parsed)) {
      Serial.println("ERR calwrite input");
      return;
    }
    view.inputs[channel_index][point] = parsed;
  }

  for (size_t point = 0; point < AS7265X_CAL_POINT_COUNT; ++point) {
    float parsed = 0.0f;
    if (!parse_float_token(tokens[3 + AS7265X_CAL_POINT_COUNT + point], parsed)) {
      Serial.println("ERR calwrite factor");
      return;
    }
    view.factors[channel_index][point] = parsed;
  }

  Serial.print("OK calwrite ");
  Serial.print(view.name);
  Serial.print(" ");
  Serial.println(channel_index);
}

void handle_darkwrite_command(char *tokens[], size_t token_count) {
  if (token_count != 20) {
    Serial.println("ERR darkwrite usage");
    return;
  }

  SensorCalibrationView view;
  if (!get_sensor_calibration_view(tokens[1], view)) {
    Serial.println("ERR darkwrite sensor");
    return;
  }

  for (size_t channel = 0; channel < AS7265X_CHANNEL_COUNT; ++channel) {
    float parsed = 0.0f;
    if (!parse_float_token(tokens[2 + channel], parsed)) {
      Serial.println("ERR darkwrite value");
      return;
    }
    view.dark_offsets[channel] = parsed;
  }

  Serial.print("OK darkwrite ");
  Serial.println(view.name);
}

bool process_serial_command(const String &raw_command) {
  String command = raw_command;
  command.trim();
  if (command.length() == 0) return false;
  if (command.equalsIgnoreCase("m")) return true;

  char buffer[512];
  if (command.length() >= sizeof(buffer)) {
    Serial.println("ERR command too long");
    return false;
  }
  command.toCharArray(buffer, sizeof(buffer));

  char *tokens[SERIAL_COMMAND_MAX_TOKENS] = {nullptr};
  size_t token_count = 0;
  char *saveptr = nullptr;
  char *token = strtok_r(buffer, " ", &saveptr);
  while (token != nullptr && token_count < SERIAL_COMMAND_MAX_TOKENS) {
    tokens[token_count++] = token;
    token = strtok_r(nullptr, " ", &saveptr);
  }
  if (token_count == 0) return false;

  if (strcmp(tokens[0], "status") == 0) {
    print_status_block();
    return false;
  }
  if (strcmp(tokens[0], "calstatus") == 0) {
    print_calibration_status();
    return false;
  }
  if (strcmp(tokens[0], "calsave") == 0) {
    if (save_calibration_to_flash()) {
      Serial.println("OK calsave");
    } else {
      Serial.println("ERR calsave");
    }
    return false;
  }
  if (strcmp(tokens[0], "calwrite") == 0) {
    handle_calwrite_command(tokens, token_count);
    return false;
  }
  if (strcmp(tokens[0], "darkwrite") == 0) {
    handle_darkwrite_command(tokens, token_count);
    return false;
  }
  if (strcmp(tokens[0], "caldump") == 0 && token_count == 2) {
    SensorCalibrationView view;
    if (!get_sensor_calibration_view(tokens[1], view)) {
      Serial.println("ERR caldump sensor");
      return false;
    }
    print_caldump(view);
    return false;
  }
  if (strcmp(tokens[0], "caldarkdump") == 0 && token_count == 2) {
    SensorCalibrationView view;
    if (!get_sensor_calibration_view(tokens[1], view)) {
      Serial.println("ERR caldarkdump sensor");
      return false;
    }
    print_caldarkdump(view);
    return false;
  }

  Serial.print("ERR unknown ");
  Serial.println(command);
  return false;
}

bool poll_serial_commands() {
  static String serial_buffer;
  bool triggered = false;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      triggered = process_serial_command(serial_buffer) || triggered;
      serial_buffer = "";
      continue;
    }
    serial_buffer += c;
  }

  return triggered;
}

// ---------------- Sensor configuration ----------------
void configure_sensor_common(AS7265X &s) {
  s.disableIndicator();
  s.disableBulb(AS7265x_LED_WHITE);
  s.disableBulb(AS7265x_LED_IR);
  s.disableBulb(AS7265x_LED_UV);
  s.setIntegrationCycles(100);  // ~280 ms integration
  s.setGain(AS7265X_GAIN_37X);
}

bool try_init_up_sensor() {
  if (up_ok) return true;
  if (!sensor_UP.begin(Wire1)) return false;
  configure_sensor_common(sensor_UP);
  return true;
}

bool try_init_down_sensor() {
  if (down_ok) return true;
  if (!sensor_DOWN.begin(Wire)) return false;
  configure_sensor_common(sensor_DOWN);
  return true;
}

void i2c_init_buses_once() {
  Wire.begin(I2C1_SDA_PIN, I2C1_SCL_PIN, 400000);
  delay(10);
  Wire1.begin(I2C2_SDA_PIN, I2C2_SCL_PIN, 400000);
  delay(10);
}

void retry_sensor_init_periodic() {
  unsigned long now = millis();
  if (now - last_sensor_retry_ms < SENSOR_RETRY_PERIOD_MS) return;
  last_sensor_retry_ms = now;

  bool up_now = try_init_up_sensor();
  bool dn_now = try_init_down_sensor();

  if (up_now != up_ok || dn_now != down_ok) {
    up_ok   = up_now;
    down_ok = dn_now;
    updateBLEStatusValue();
    drawStatusScreen();
  } else {
    up_ok   = up_now;
    down_ok = dn_now;
  }
}

// ---------------- UI ----------------
void drawHeaderStatus() {
  bool connected = ble_is_connected();
  tft.setCursor(10, 10);
  tft.setTextColor(connected ? CYAN : GREY);
  tft.setTextSize(2);
  tft.print("BLE");

  if (connected) {
    tft.fillCircle(55, 17, 4, CYAN);
  } else {
    tft.drawCircle(55, 17, 4, GREY);
  }

  tft.setCursor(140, 10);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("BAT:");
  tft.print(battery_soc);
  tft.print("%");
}

void drawStatusScreen() {
  current_screen_mode = SCREEN_MODE_STATUS;
  tft.fillScreen(BLACK);
  drawHeaderStatus();

  tft.setCursor(10, 50);
  tft.setTextSize(3);

  if (!up_ok) {
    tft.setTextColor(RED);
    tft.print("UP: MISSING");
  }

  tft.setCursor(10, 90);
  if (!down_ok) {
    tft.setTextColor(RED);
    tft.print("DN: MISSING");
  }
}

bool isLowLight() {
  return last_measurement_low_light;
}

void drawCenteredText(const String &text, int16_t centerY, uint8_t textSize, uint16_t color) {
  int16_t x1 = 0;
  int16_t y1 = 0;
  uint16_t w = 0;
  uint16_t h = 0;
  tft.setTextSize(textSize);
  tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((SCREEN_WIDTH - (int16_t)w) / 2, centerY - ((int16_t)h / 2));
  tft.setTextColor(color);
  tft.print(text);
}

void drawNDVIScreen() {
  current_screen_mode = SCREEN_MODE_NDVI;
  tft.fillScreen(BLACK);
  drawHeaderStatus();

  if (!up_ok || !down_ok) {
    tft.setCursor(10, 95);
    tft.setTextColor(RED);
    tft.setTextSize(3);
    tft.println("CONNECT");
    tft.setCursor(10, 130);
    tft.println("SENSOR");
    tft.setCursor(10, 200);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.println("Retrying...");
    return;
  }

  drawCenteredText(message_NDVI, 118, 9, WHITE);

  if (isLowLight()) {
    drawCenteredText("LOW LIGHT", 180, 3, WHITE);
  }
}

void drawMeasuringScreen(int currentSample, int totalSamples) {
  current_screen_mode      = SCREEN_MODE_MEASURING;
  current_measuring_sample = currentSample;
  current_measuring_total  = totalSamples;
  tft.fillScreen(BLACK);
  drawHeaderStatus();

  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.setCursor(20, 100);
  tft.println("MEASURING");
}

void redrawCurrentScreen() {
  if (current_screen_mode == SCREEN_MODE_MEASURING) {
    drawMeasuringScreen(current_measuring_sample, current_measuring_total);
  } else if (current_screen_mode == SCREEN_MODE_NDVI) {
    drawNDVIScreen();
  } else {
    drawStatusScreen();
  }
}

void updateBLEStatusValue() {
  if (ble_status_characteristic == nullptr) return;

  String payload = "NDVI=";
  payload += message_NDVI;
  payload += ",SEQ=";
  payload += measurement_sequence;
  payload += ",MS=";
  payload += last_measurement_ms;
  payload += ",BAT=";
  payload += battery_soc;
  payload += ",UP=";
  payload += (up_ok  ? "1" : "0");
  payload += ",DN=";
  payload += (down_ok ? "1" : "0");
  payload += ",LOW=";
  payload += (last_measurement_low_light ? "1" : "0");

  ble_status_characteristic->setValue(payload.c_str());

  if (ble_is_connected()) {
    ble_status_characteristic->notify();
  }
}

void setupBLE() {
  BLEDevice::init(BLE_DEVICE_NAME);

  ble_server = BLEDevice::createServer();
  ble_server->setCallbacks(new MantisBLEServerCallbacks());

  ble_service = ble_server->createService(BLE_SERVICE_UUID);

  ble_status_characteristic = ble_service->createCharacteristic(
    BLE_STATUS_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );

  updateBLEStatusValue();
  ble_service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
}

void serviceBLE() {
  if (ble_take_state_dirty()) {
    bool connected = ble_is_connected();
    if (!connected && ble_was_connected) {
      Serial.println("BLE disconnected");
    } else if (connected && !ble_was_connected) {
      Serial.println("BLE connected");
    }
    ble_was_connected = connected;
    updateBLEStatusValue();
  }

  if (ble_take_ui_dirty()) {
    redrawCurrentScreen();
  }
}

// ---------------- Measurement ----------------
bool take_measurements_and_compute_ndvi() {
  if (!up_ok || !down_ok) return false;

  drawMeasuringScreen(1, 1);

  sensor_UP.takeMeasurements();
  sensor_DOWN.takeMeasurements();
  uint16_t up_values[AS7265X_CHANNEL_COUNT];
  uint16_t down_values[AS7265X_CHANNEL_COUNT];
  read_sensor_channels(sensor_UP, up_values);
  read_sensor_channels(sensor_DOWN, down_values);

  uint16_t iU_raw = up_values[RED_CHANNEL_INDEX];
  uint16_t vU_raw = up_values[NIR_CHANNEL_INDEX];
  uint16_t iD_raw = down_values[RED_CHANNEL_INDEX];
  uint16_t vD_raw = down_values[NIR_CHANNEL_INDEX];

  last_red_UP = (float)iU_raw;
  last_nir_UP = (float)vU_raw;

  SensorCalibrationView up_view;
  SensorCalibrationView down_view;
  get_sensor_calibration_view("up", up_view);
  get_sensor_calibration_view("down", down_view);

  float iU = apply_channel_calibration(up_view, RED_CHANNEL_INDEX, (float)iU_raw);
  float vU = apply_channel_calibration(up_view, NIR_CHANNEL_INDEX, (float)vU_raw);
  float iD = apply_channel_calibration(down_view, RED_CHANNEL_INDEX, (float)iD_raw);
  float vD = apply_channel_calibration(down_view, NIR_CHANNEL_INDEX, (float)vD_raw);
  float derived_iU = (UP_RED_FROM_NIR_SLOPE * vU) + UP_RED_FROM_NIR_INTERCEPT;
  if (derived_iU < 0.0f) derived_iU = 0.0f;

  (void)iU;
  last_calc_iU = derived_iU;
  last_calc_vU = vU;
  last_calc_iD = iD;
  last_calc_vD = vD;

  last_measurement_low_light =
    (derived_iU < MIN_INCIDENT_SIGNAL) ||
    (vU < MIN_INCIDENT_SIGNAL);

  float red_refl = (derived_iU > 0.0f) ? (iD / derived_iU) : 0.0f;
  float nir_refl = (vU > 0.0f) ? (vD / vU) : 0.0f;
  float denom = nir_refl + red_refl;
  ndvi = (denom > 0.0f) ? ((nir_refl - red_refl) / denom) : 0.0f;
  ndvi = clampf(ndvi, -1.0f, 1.0f);

  message_NDVI = String(ndvi, 2);
  measurement_sequence++;
  last_measurement_ms = millis();

  battery_soc = read_voltage_percent();
  updateBLEStatusValue();

  print_raw_channels_legacy("UP raw:", up_values);
  Serial.println("------------------------------------");
  print_raw_channels_legacy("DOWN raw:", down_values);

  return true;
}

// ================= SETUP & LOOP =================
void setup() {
  Serial.begin(115200);
  Serial.println("Manti NDVI meter ready. Send 'm' to measure.");

  WiFi.mode(WIFI_OFF);

  ble_mutex = xSemaphoreCreateMutex();

  pinMode(BTN_MEASURE, INPUT_PULLUP);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  configure_batt();
  battery_soc = read_voltage_percent();

  tft.init(SCREEN_WIDTH, SCREEN_HEIGHT);
  tft.setRotation(2);
  tft.fillScreen(BLACK);

  setupBLE();
  i2c_init_buses_once();
  load_calibration_from_flash();

  up_ok   = try_init_up_sensor();
  down_ok = try_init_down_sensor();
  updateBLEStatusValue();

  if (up_ok && down_ok) {
    drawNDVIScreen();
  } else {
    drawStatusScreen();
  }
}

void loop() {
  serviceBLE();
  retry_sensor_init_periodic();

  unsigned long now = millis();
  if (now - last_batt_sample_ms >= BATT_SAMPLE_PERIOD_MS) {
    last_batt_sample_ms = now;
    battery_soc = read_voltage_percent();
  }

  bool btn_now       = (digitalRead(BTN_MEASURE) == LOW);
  bool button_trigger = (!lastMeasureState && btn_now);
  bool serial_trigger = poll_serial_commands();
  lastMeasureState    = btn_now;

  if (button_trigger || serial_trigger) {
    if (serial_trigger) Serial.println(">> Serial trigger received.");

    up_ok   = false;
    down_ok = false;
    up_ok   = try_init_up_sensor();
    down_ok = try_init_down_sensor();
    updateBLEStatusValue();

    if (take_measurements_and_compute_ndvi()) {
      drawNDVIScreen();
    } else {
      drawStatusScreen();
    }
  }

  delay(20);
}
