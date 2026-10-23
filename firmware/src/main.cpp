/***************************************************
  AS7265X dual-sensor outdoor NDVI meter (ESP32)

  - Bulbs OFF (ambient light)
  - Uses channels I (red-ish) and V (NIR-ish) for NDVI
  - UP sensor: incident light (sky) on Wire1 (default SDA/SCL)
  - DOWN sensor: reflected light (turf) on Wire (custom GPIOs on PCB)
  - Shows NDVI + battery % on ST7789 240x240 LCD (Pimoroni 1.3")
  - Short button press measures; 5-second hold opens setup; serial 'm' measures
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
#include "manti_features.h"
#include "portal_ota.h"
#if __has_include("release_version.h")
#include "release_version.h"
#endif
#include <Wire.h>
#include <WiFi.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLE2902.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_ota_ops.h>

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
static constexpr uint32_t BUTTON_DEBOUNCE_MS = 30;
static constexpr uint32_t PORTAL_HOLD_MS = 5000;
bool buttonRawDown = false;
bool buttonDown = false;
bool buttonActionHandled = false;
uint32_t buttonRawChangedAt = 0;
uint32_t buttonPressStartedAt = 0;

static constexpr const char *CALIBRATION_FW_ID = "Manti-S-Cal";
#ifndef MANTI_FIRMWARE_VERSION
#define MANTI_FIRMWARE_VERSION "v0.3.3-dev4"
#endif
static constexpr const char *FIRMWARE_VERSION = MANTI_FIRMWARE_VERSION;
#ifndef MANTI_FIRMWARE_BUILD_DATE
#define MANTI_FIRMWARE_BUILD_DATE __DATE__
#endif
static constexpr const char *FIRMWARE_BUILD_DATE = MANTI_FIRMWARE_BUILD_DATE;
static constexpr const char *CALIBRATION_NAMESPACE = "mantiscal";
static constexpr const char *CALIBRATION_KEY = "blob";
static constexpr const char *CALIBRATION_PARTITION = "caldata";
static constexpr uint32_t CALIBRATION_MAGIC = 0x4D534341UL;
static constexpr uint16_t CALIBRATION_VERSION = 5;
static constexpr bool ENABLE_16X_CALIBRATION = MANTI_ENABLE_16X_CALIBRATION != 0;
static constexpr size_t SERIAL_COMMAND_MAX_TOKENS = 2;
static constexpr size_t RED_CHANNEL_INDEX = 9;
static constexpr size_t NIR_CHANNEL_INDEX = 14;
static constexpr uint8_t UP_INTEGRATION_CYCLES = 100;
static constexpr uint8_t UP_LOW_LIGHT_INTEGRATION_CYCLES = 255;
static constexpr uint8_t UP_LOW_LIGHT_EXPOSURES = 3;
static constexpr uint8_t DOWN_INTEGRATION_CYCLES = 255;
static constexpr uint8_t DOWN_LOW_LIGHT_EXPOSURES = 3;
static constexpr uint32_t UP_MIN_RAW_SIGNAL = 100;
static constexpr uint32_t DOWN_MIN_RAW_SIGNAL = 100;
static constexpr float UP_DIFFUSER_TRANSMISSION_RED = 0.0470f;
static constexpr float UP_DIFFUSER_TRANSMISSION_NIR = 0.0564f;
static constexpr float BASE_SENSOR_GAIN = 3.7f;
static constexpr float HIGH_SENSOR_GAIN = 16.0f;
static constexpr int RUNTIME_GAIN_ATTEMPTS = ENABLE_16X_CALIBRATION ? 2 : 1;
static constexpr int MEASUREMENT_PROGRESS_TOTAL =
  (RUNTIME_GAIN_ATTEMPTS * (1 + 1 + UP_LOW_LIGHT_EXPOSURES)) +
  (RUNTIME_GAIN_ATTEMPTS * (1 + DOWN_LOW_LIGHT_EXPOSURES));
static constexpr int UP_PROGRESS_COMPLETE =
  RUNTIME_GAIN_ATTEMPTS * (1 + 1 + UP_LOW_LIGHT_EXPOSURES);

enum CalibrationProfileIndex : uint8_t {
  PROFILE_UP_100X1 = 0,
  PROFILE_UP_255X1,
  PROFILE_UP_255X3,
  PROFILE_DOWN_255X1,
  PROFILE_DOWN_255X3,
  PROFILE_UP_100X1_G16,
  PROFILE_UP_255X1_G16,
  PROFILE_UP_255X3_G16,
  PROFILE_DOWN_255X1_G16,
  PROFILE_DOWN_255X3_G16,
  CALIBRATION_PROFILE_COUNT
};

struct CalibrationProfileDefinition {
  const char *id;
  bool is_up;
  uint8_t integration_cycles;
  uint8_t exposures;
  uint8_t gain_setting;
};

static const CalibrationProfileDefinition CALIBRATION_PROFILES[CALIBRATION_PROFILE_COUNT] = {
  {"up_100x1", true, UP_INTEGRATION_CYCLES, 1, AS7265X_GAIN_37X},
  {"up_255x1", true, UP_LOW_LIGHT_INTEGRATION_CYCLES, 1, AS7265X_GAIN_37X},
  {"up_255x3", true, UP_LOW_LIGHT_INTEGRATION_CYCLES, UP_LOW_LIGHT_EXPOSURES, AS7265X_GAIN_37X},
  {"down_255x1", false, DOWN_INTEGRATION_CYCLES, 1, AS7265X_GAIN_37X},
  {"down_255x3", false, DOWN_INTEGRATION_CYCLES, DOWN_LOW_LIGHT_EXPOSURES, AS7265X_GAIN_37X},
  {"up_100x1_g16", true, UP_INTEGRATION_CYCLES, 1, AS7265X_GAIN_16X},
  {"up_255x1_g16", true, UP_LOW_LIGHT_INTEGRATION_CYCLES, 1, AS7265X_GAIN_16X},
  {"up_255x3_g16", true, UP_LOW_LIGHT_INTEGRATION_CYCLES, UP_LOW_LIGHT_EXPOSURES, AS7265X_GAIN_16X},
  {"down_255x1_g16", false, DOWN_INTEGRATION_CYCLES, 1, AS7265X_GAIN_16X},
  {"down_255x3_g16", false, DOWN_INTEGRATION_CYCLES, DOWN_LOW_LIGHT_EXPOSURES, AS7265X_GAIN_16X},
};

static_assert(CALIBRATION_PROFILE_COUNT <= 16, "Calibration mask supports at most 16 profiles");

static const char *CHANNEL_LABELS[AS7265X_CHANNEL_COUNT] = {
  "A", "B", "C", "D", "E", "F", "G", "H", "R",
  "I", "S", "J", "T", "U", "V", "W", "K", "L"
};

static const uint16_t CHANNEL_WAVELENGTHS[AS7265X_CHANNEL_COUNT] = {
  410, 435, 460, 485, 510, 535, 560, 585, 610,
  645, 680, 705, 730, 760, 810, 860, 900, 940
};

static float CAL_PROFILE_RED_DARK_OFFSETS[CALIBRATION_PROFILE_COUNT] = {};
static float CAL_PROFILE_NIR_DARK_OFFSETS[CALIBRATION_PROFILE_COUNT] = {};
static uint16_t calibration_valid_mask = 0;
static bool calibration_loaded_from_flash = false;

// This type matches the existing on-flash schema; it is allocated only during boot.
struct LegacyCalibrationBlob {
  uint32_t magic;
  uint16_t version;
  uint8_t profile_count;
  uint8_t reserved;
  uint16_t valid_mask;
  float inputs[CALIBRATION_PROFILE_COUNT][AS7265X_CHANNEL_COUNT][AS7265X_CAL_POINT_COUNT];
  float factors[CALIBRATION_PROFILE_COUNT][AS7265X_CHANNEL_COUNT][AS7265X_CAL_POINT_COUNT];
  float dark_offsets[CALIBRATION_PROFILE_COUNT][AS7265X_CHANNEL_COUNT];
};

static_assert(sizeof(LegacyCalibrationBlob) == 9372, "Legacy calibration layout changed");

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
String message_raw_diffuser_NDVI = "0.00";
float raw_diffuser_ndvi = 0.0f;
float last_raw_diffuser_iU = 0.0f;
float last_raw_diffuser_vU = 0.0f;
float last_raw_diffuser_iD = 0.0f;
float last_raw_diffuser_vD = 0.0f;
bool last_measurement_low_light = false;
uint8_t last_up_integration_cycles = UP_INTEGRATION_CYCLES;
uint8_t last_up_exposure_count = 1;
uint8_t last_down_exposure_count = 1;
uint8_t last_up_gain_setting = AS7265X_GAIN_37X;
uint8_t last_down_gain_setting = AS7265X_GAIN_37X;
CalibrationProfileIndex last_up_profile = PROFILE_UP_100X1;
CalibrationProfileIndex last_down_profile = PROFILE_DOWN_255X1;
bool last_up_saturated = false;
bool last_down_saturated = false;
uint32_t measurement_sequence = 0;
unsigned long last_measurement_ms = 0;

// ---------------- BLE ----------------
#ifdef MANTI_DEVICE_NUMBER
static_assert(MANTI_DEVICE_NUMBER >= 1 && MANTI_DEVICE_NUMBER <= 99999,
              "MANTI_DEVICE_NUMBER must be between 1 and 99999");
#endif
static char BLE_DEVICE_NAME[sizeof("Manti-S-FFFFFFFFFFFF")] = {};

void loadDeviceIdentity() {
  Preferences identity;
  uint32_t deviceNumber = 0;
  if (identity.begin("mantid", false)) {
    deviceNumber = identity.getUInt("number", 0);
#ifdef MANTI_DEVICE_NUMBER
    if (deviceNumber == 0) {
      deviceNumber = MANTI_DEVICE_NUMBER;
      identity.putUInt("number", deviceNumber);
    }
#endif
    identity.end();
  }
  if (deviceNumber >= 1 && deviceNumber <= 99999) {
    snprintf(BLE_DEVICE_NAME, sizeof(BLE_DEVICE_NAME), "Manti-S-%05lu", (unsigned long)deviceNumber);
  } else {
    // Generic release images use the factory MAC until USB provisioning assigns a number.
    snprintf(BLE_DEVICE_NAME, sizeof(BLE_DEVICE_NAME), "Manti-S-%012llX",
             (unsigned long long)ESP.getEfuseMac());
  }
}
static const char *BLE_SERVICE_UUID = "9f4a6d90-f740-4d10-a947-8c6c20f4f9d1";
static const char *BLE_STATUS_UUID  = "2d86686f-34f8-4f1b-a4ac-f127f90f7df0";

BLEServer         *ble_server                = nullptr;
BLEService        *ble_service               = nullptr;
BLECharacteristic *ble_status_characteristic = nullptr;
bool blePausedForPortal = false;

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
  SCREEN_MODE_MEASURING,
  SCREEN_MODE_PORTAL
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
    if (!blePausedForPortal) server->startAdvertising();
  }
};

// ---------------- Forward declarations ----------------
void drawHeaderStatus();
void drawStatusScreen();
void drawNDVIScreen();
void drawMeasuringScreen(int currentSample, int totalSamples);
void drawPortalScreen();
void updateMeasuringProgress(int currentSample, int totalSamples);
void redrawCurrentScreen();
void setupBLE();
void serviceBLE();
void updateBLEStatusValue();
bool poll_serial_commands();

bool get_calibration_profile_index(const char *profile_id, CalibrationProfileIndex &index) {
  for (uint8_t i = 0; i < CALIBRATION_PROFILE_COUNT; ++i) {
    if (strcmp(profile_id, CALIBRATION_PROFILES[i].id) == 0) {
      index = (CalibrationProfileIndex)i;
      return true;
    }
  }
  return false;
}

bool calibration_profile_is_valid(CalibrationProfileIndex index) {
  return (calibration_valid_mask & (uint16_t)(1U << index)) != 0;
}

bool load_calibration_from_partition(const char *partition_label) {
  if (!calibration_preferences.begin(CALIBRATION_NAMESPACE, true, partition_label)) return false;
  size_t stored_size = calibration_preferences.getBytesLength(CALIBRATION_KEY);
  if (stored_size == sizeof(LegacyCalibrationBlob)) {
    LegacyCalibrationBlob *blob = static_cast<LegacyCalibrationBlob *>(malloc(stored_size));
    if (blob != nullptr) {
      size_t read_size = calibration_preferences.getBytes(CALIBRATION_KEY, blob, stored_size);
      if (read_size == stored_size && blob->magic == CALIBRATION_MAGIC &&
          blob->version == CALIBRATION_VERSION &&
          blob->profile_count == CALIBRATION_PROFILE_COUNT) {
        calibration_valid_mask = blob->valid_mask;
        for (uint8_t profile = 0; profile < CALIBRATION_PROFILE_COUNT; ++profile) {
          CAL_PROFILE_RED_DARK_OFFSETS[profile] = blob->dark_offsets[profile][RED_CHANNEL_INDEX];
          CAL_PROFILE_NIR_DARK_OFFSETS[profile] = blob->dark_offsets[profile][NIR_CHANNEL_INDEX];
        }
        calibration_loaded_from_flash = true;
      }
      free(blob);
    }
  }
  calibration_preferences.end();
  return calibration_loaded_from_flash;
}

bool load_calibration_from_flash() {
  // Read only profile flags and I/V dark offsets from existing schema-5 data.
  // Never rewrite the legacy calibration partition.
  return load_calibration_from_partition(CALIBRATION_PARTITION);
}

const char *calibration_source_name() {
  return calibration_loaded_from_flash ? "flash" : "none";
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

bool capture_profile_raw(
  CalibrationProfileIndex profile_index,
  uint32_t values[AS7265X_CHANNEL_COUNT],
  bool &saturated,
  int *measurement_progress = nullptr,
  int measurement_progress_total = 0
) {
  const CalibrationProfileDefinition &profile = CALIBRATION_PROFILES[profile_index];
  AS7265X &sensor = profile.is_up ? sensor_UP : sensor_DOWN;
  if ((profile.is_up && !up_ok) || (!profile.is_up && !down_ok)) return false;

  memset(values, 0, sizeof(uint32_t) * AS7265X_CHANNEL_COUNT);
  saturated = false;
  sensor.setIntegrationCycles(profile.integration_cycles);
  sensor.setGain(profile.gain_setting);

  uint16_t frame[AS7265X_CHANNEL_COUNT];
  for (uint8_t exposure = 0; exposure < profile.exposures; ++exposure) {
    sensor.takeMeasurements();
    read_sensor_channels(sensor, frame);
    for (size_t channel = 0; channel < AS7265X_CHANNEL_COUNT; ++channel) {
      values[channel] += frame[channel];
      if (frame[channel] == UINT16_MAX) saturated = true;
    }
    if (measurement_progress != nullptr && measurement_progress_total > 0) {
      (*measurement_progress)++;
      updateMeasuringProgress(*measurement_progress, measurement_progress_total);
    }
  }
  return true;
}

float sensor_gain_value(uint8_t gain_setting) {
  return gain_setting == AS7265X_GAIN_16X ? HIGH_SENSOR_GAIN : BASE_SENSOR_GAIN;
}

const char *sensor_gain_label(uint8_t gain_setting) {
  return gain_setting == AS7265X_GAIN_16X ? "16x" : "3.7x";
}

bool raw_signal_is_low(
  const uint32_t values[AS7265X_CHANNEL_COUNT],
  uint32_t minimum_raw_signal
) {
  return values[RED_CHANNEL_INDEX] < minimum_raw_signal ||
    values[NIR_CHANNEL_INDEX] < minimum_raw_signal;
}

bool capture_adaptive_profiles(
  const CalibrationProfileIndex base_profiles[],
  const CalibrationProfileIndex high_profiles[],
  size_t profile_count,
  uint32_t selected_values[AS7265X_CHANNEL_COUNT],
  uint32_t minimum_raw_signal,
  CalibrationProfileIndex &selected_profile,
  uint8_t &selected_gain_setting,
  bool &selected_saturated,
  int &measurement_progress
) {
  uint32_t base_values[AS7265X_CHANNEL_COUNT] = {0};
  uint32_t high_values[AS7265X_CHANNEL_COUNT] = {0};

  for (size_t index = 0; index < profile_count; ++index) {
    CalibrationProfileIndex base_profile = base_profiles[index];
    CalibrationProfileIndex high_profile = high_profiles[index];
    bool base_saturated = false;
    if (!capture_profile_raw(
          base_profile,
          base_values,
          base_saturated,
          &measurement_progress,
          MEASUREMENT_PROGRESS_TOTAL
        )) return false;

    if (base_saturated || !raw_signal_is_low(base_values, minimum_raw_signal)) {
      memcpy(selected_values, base_values, sizeof(base_values));
      selected_profile = base_profile;
      selected_gain_setting = CALIBRATION_PROFILES[base_profile].gain_setting;
      selected_saturated = base_saturated;
      return true;
    }

    bool final_profile = index + 1 == profile_count;
    if (ENABLE_16X_CALIBRATION && calibration_profile_is_valid(high_profile)) {
      bool high_saturated = false;
      if (!capture_profile_raw(
            high_profile,
            high_values,
            high_saturated,
            &measurement_progress,
            MEASUREMENT_PROGRESS_TOTAL
          )) return false;

      bool high_is_usable = !high_saturated &&
        (!raw_signal_is_low(high_values, minimum_raw_signal) || final_profile);
      if (high_is_usable) {
        memcpy(selected_values, high_values, sizeof(high_values));
        selected_profile = high_profile;
        selected_gain_setting = CALIBRATION_PROFILES[high_profile].gain_setting;
        selected_saturated = false;
        return true;
      }

      if (final_profile) {
        // A saturated 16x result is invalid; retain the same profile at 3.7x.
        memcpy(selected_values, base_values, sizeof(base_values));
        selected_profile = base_profile;
        selected_gain_setting = CALIBRATION_PROFILES[base_profile].gain_setting;
        selected_saturated = base_saturated;
        return true;
      }
    }

    if (final_profile) {
      memcpy(selected_values, base_values, sizeof(base_values));
      selected_profile = base_profile;
      selected_gain_setting = CALIBRATION_PROFILES[base_profile].gain_setting;
      selected_saturated = base_saturated;
      return true;
    }
  }

  return false;
}

float dark_corrected_calibration_input(
  CalibrationProfileIndex profile,
  size_t channel_index,
  float raw_value
) {
  float dark_offset = channel_index == RED_CHANNEL_INDEX
    ? CAL_PROFILE_RED_DARK_OFFSETS[profile]
    : CAL_PROFILE_NIR_DARK_OFFSETS[profile];
  float input_value = raw_value - dark_offset;
  return input_value > 0.0f ? input_value : 0.0f;
}

float convert_up_input_to_down_profile(
  const CalibrationProfileDefinition &up_profile,
  const CalibrationProfileDefinition &down_profile,
  float up_input,
  float diffuser_transmission
) {
  if (diffuser_transmission <= 0.0f) return 0.0f;
  float integration_ratio =
    ((float)down_profile.integration_cycles + 1.0f) /
    ((float)up_profile.integration_cycles + 1.0f);
  float exposure_ratio = (float)down_profile.exposures / (float)up_profile.exposures;
  float gain_ratio = sensor_gain_value(down_profile.gain_setting) /
    sensor_gain_value(up_profile.gain_setting);
  return (up_input / diffuser_transmission) * integration_ratio * exposure_ratio * gain_ratio;
}

float calculate_raw_diffuser_ndvi(
  const CalibrationProfileDefinition &up_profile,
  const CalibrationProfileDefinition &down_profile,
  float up_red_input,
  float up_nir_input,
  float down_red_input,
  float down_nir_input
) {
  float equivalent_up_red = convert_up_input_to_down_profile(
    up_profile,
    down_profile,
    up_red_input,
    UP_DIFFUSER_TRANSMISSION_RED
  );
  float equivalent_up_nir = convert_up_input_to_down_profile(
    up_profile,
    down_profile,
    up_nir_input,
    UP_DIFFUSER_TRANSMISSION_NIR
  );
  float red_reflectance = equivalent_up_red > 0.0f
    ? down_red_input / equivalent_up_red
    : 0.0f;
  float nir_reflectance = equivalent_up_nir > 0.0f
    ? down_nir_input / equivalent_up_nir
    : 0.0f;
  float denominator = nir_reflectance + red_reflectance;
  float result = denominator > 0.0f
    ? (nir_reflectance - red_reflectance) / denominator
    : 0.0f;
  return clampf(result, -1.0f, 1.0f);
}

void print_status_block() {
  Serial.println("BEGIN_STATUS");
  Serial.print("fw=");
  Serial.println(CALIBRATION_FW_ID);
  Serial.print("firmware_version=");
  Serial.println(FIRMWARE_VERSION);
  Serial.print("firmware_build_date=");
  Serial.println(FIRMWARE_BUILD_DATE);
  Serial.print("calibration_source=");
  Serial.println(calibration_source_name());
  Serial.print("calibration_schema=");
  Serial.println(CALIBRATION_VERSION);
  Serial.print("calibration_valid_mask=");
  Serial.println(calibration_valid_mask);
  Serial.print("calibration_16x_enabled=");
  Serial.println(ENABLE_16X_CALIBRATION ? 1 : 0);
  const uint16_t high_gain_mask =
    (1U << PROFILE_UP_100X1_G16) |
    (1U << PROFILE_UP_255X1_G16) |
    (1U << PROFILE_UP_255X3_G16) |
    (1U << PROFILE_DOWN_255X1_G16) |
    (1U << PROFILE_DOWN_255X3_G16);
  Serial.print("calibration_16x_ready=");
  Serial.println((calibration_valid_mask & high_gain_mask) == high_gain_mask ? 1 : 0);
  Serial.print("up_profile=");
  Serial.println(CALIBRATION_PROFILES[last_up_profile].id);
  Serial.print("up_calibration_mode=");
  Serial.println("not-used");
  Serial.print("up_transfer_profile=");
  Serial.println("none");
  Serial.print("up_integration_cycles=");
  Serial.println(last_up_integration_cycles);
  Serial.print("up_exposures=");
  Serial.println(last_up_exposure_count);
  Serial.print("up_gain=");
  Serial.println(sensor_gain_label(last_up_gain_setting));
  Serial.print("down_integration_cycles=");
  Serial.println(DOWN_INTEGRATION_CYCLES);
  Serial.print("down_exposures=");
  Serial.println(last_down_exposure_count);
  Serial.print("down_gain=");
  Serial.println(sensor_gain_label(last_down_gain_setting));
  Serial.print("down_profile=");
  Serial.println(CALIBRATION_PROFILES[last_down_profile].id);
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

void print_raw_profile_block(
  CalibrationProfileIndex profile_index,
  const uint32_t values[AS7265X_CHANNEL_COUNT],
  bool saturated
) {
  const CalibrationProfileDefinition &profile = CALIBRATION_PROFILES[profile_index];
  Serial.print("BEGIN_RAW,");
  Serial.println(profile.id);
  Serial.print("meta,integration_cycles,");
  Serial.println(profile.integration_cycles);
  Serial.print("meta,exposures,");
  Serial.println(profile.exposures);
  Serial.print("meta,gain,");
  Serial.println(sensor_gain_label(profile.gain_setting));
  Serial.print("meta,saturated,");
  Serial.println(saturated ? 1 : 0);
  for (size_t channel = 0; channel < AS7265X_CHANNEL_COUNT; ++channel) {
    Serial.print(CHANNEL_LABELS[channel]);
    Serial.print(",");
    Serial.print(CHANNEL_WAVELENGTHS[channel]);
    Serial.print(",");
    Serial.println(values[channel]);
  }
  Serial.print("END_RAW,");
  Serial.println(profile.id);
}

void handle_capture_command(const char *profile_id) {
  CalibrationProfileIndex profile_index;
  if (!get_calibration_profile_index(profile_id, profile_index)) {
    Serial.println("ERR capture profile");
    return;
  }
  uint32_t values[AS7265X_CHANNEL_COUNT];
  bool saturated = false;
  if (!capture_profile_raw(profile_index, values, saturated)) {
    Serial.println("ERR capture sensor");
    return;
  }
  print_raw_profile_block(profile_index, values, saturated);
}

void print_raw_channels_legacy(const char *label, const uint32_t values[AS7265X_CHANNEL_COUNT]) {
  Serial.println(label);
  for (size_t channel = 0; channel < AS7265X_CHANNEL_COUNT; ++channel) {
    Serial.print(CHANNEL_LABELS[channel]);
    Serial.print(" (");
    Serial.print(CHANNEL_WAVELENGTHS[channel]);
    Serial.print(" nm): ");
    Serial.println(values[channel]);
  }
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
  if (strcmp(tokens[0], "capture") == 0 && token_count == 2) {
    handle_capture_command(tokens[1]);
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
void configure_sensor_common(AS7265X &s, uint8_t integration_cycles) {
  s.disableIndicator();
  s.disableBulb(AS7265x_LED_WHITE);
  s.disableBulb(AS7265x_LED_IR);
  s.disableBulb(AS7265x_LED_UV);
  s.setIntegrationCycles(integration_cycles);
  s.setGain(AS7265X_GAIN_37X);
}

bool try_init_up_sensor() {
  if (up_ok) return true;
  if (!sensor_UP.begin(Wire1)) return false;
  configure_sensor_common(sensor_UP, UP_INTEGRATION_CYCLES);
  return true;
}

bool try_init_down_sensor() {
  if (down_ok) return true;
  if (!sensor_DOWN.begin(Wire)) return false;
  configure_sensor_common(sensor_DOWN, DOWN_INTEGRATION_CYCLES);
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

void drawCenteredTextAtX(
  const String &text,
  int16_t centerX,
  int16_t centerY,
  uint8_t textSize,
  uint16_t color
) {
  int16_t x1 = 0;
  int16_t y1 = 0;
  uint16_t w = 0;
  uint16_t h = 0;
  tft.setTextSize(textSize);
  tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor(centerX - ((int16_t)w / 2), centerY - ((int16_t)h / 2));
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

  drawCenteredText("NDVI", 65, 2, GREY);
  drawCenteredText(message_raw_diffuser_NDVI, 125, 7, CYAN);

  if (isLowLight()) {
    drawCenteredText("LOW CONFIDENCE", 218, 2, YELLOW);
  }
}

void updateMeasuringProgress(int currentSample, int totalSamples) {
  if (totalSamples < 1) totalSamples = 1;
  if (currentSample < 0) currentSample = 0;
  if (currentSample > totalSamples) currentSample = totalSamples;

  current_screen_mode      = SCREEN_MODE_MEASURING;
  current_measuring_sample = currentSample;
  current_measuring_total  = totalSamples;

  const int barX = 20;
  const int barY = 120;
  const int barW = 200;
  const int barH = 20;
  const int fillW = (currentSample * (barW - 2)) / totalSamples;

  tft.drawRect(barX, barY, barW, barH, WHITE);
  tft.fillRect(barX + 1, barY + 1, barW - 2, barH - 2, BLACK);
  tft.fillRect(barX + 1, barY + 1, fillW, barH - 2, GREEN);

  const int percent = (currentSample * 100) / totalSamples;
  tft.fillRect(0, 145, SCREEN_WIDTH, 50, BLACK);
  drawCenteredText(String(percent) + "%", 170, 2, YELLOW);
}

void drawMeasuringScreen(int currentSample, int totalSamples) {
  current_screen_mode = SCREEN_MODE_MEASURING;
  tft.fillScreen(BLACK);
  drawHeaderStatus();

  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.setCursor(20, 60);
  tft.println("MEASURING");

  updateMeasuringProgress(currentSample, totalSamples);
}

void redrawCurrentScreen() {
  if (current_screen_mode == SCREEN_MODE_MEASURING) {
    drawMeasuringScreen(current_measuring_sample, current_measuring_total);
  } else if (current_screen_mode == SCREEN_MODE_NDVI) {
    drawNDVIScreen();
  } else if (current_screen_mode == SCREEN_MODE_PORTAL) {
    drawPortalScreen();
  } else {
    drawStatusScreen();
  }
}

void drawPortalScreen() {
  current_screen_mode = SCREEN_MODE_PORTAL;
  tft.fillScreen(BLACK);
  tft.setTextColor(CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 12);
  tft.println("SETUP / UPDATE");
  tft.setTextColor(WHITE);
  tft.setCursor(10, 52);
  tft.println(portalSsid());
  tft.setCursor(10, 91);
  tft.println(portalPassword());
  tft.setTextColor(YELLOW);
  tft.setCursor(10, 132);
  tft.println("192.168.4.1");
  tft.setTextSize(1);
  tft.setCursor(10, 183);
  tft.println(portalMessage());
  tft.setCursor(10, 220);
  tft.println("Press button to exit");
}

void updateBLEStatusValue() {
  if (ble_status_characteristic == nullptr) return;

  String payload = "NDVI=";
  payload += message_raw_diffuser_NDVI;
  payload += ",RUI=";
  payload += String(last_raw_diffuser_iU, 2);
  payload += ",RUV=";
  payload += String(last_raw_diffuser_vU, 2);
  payload += ",RDI=";
  payload += String(last_raw_diffuser_iD, 2);
  payload += ",RDV=";
  payload += String(last_raw_diffuser_vD, 2);
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
  payload += ",UI=";
  payload += (unsigned int)last_up_integration_cycles;
  payload += ",UX=";
  payload += (unsigned int)last_up_exposure_count;
  payload += ",UG=";
  payload += sensor_gain_value(last_up_gain_setting);
  payload += ",DX=";
  payload += (unsigned int)last_down_exposure_count;
  payload += ",DG=";
  payload += sensor_gain_value(last_down_gain_setting);
  payload += ",SAT=";
  payload += (unsigned int)((last_up_saturated ? 1 : 0) | (last_down_saturated ? 2 : 0));
  payload += ",CVM=";
  payload += (unsigned int)calibration_valid_mask;

  ble_status_characteristic->setValue(payload.c_str());

  if (ble_is_connected()) {
    ble_status_characteristic->notify();
  }
}

void setupBLE() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(247);

  ble_server = BLEDevice::createServer();
  ble_server->setCallbacks(new MantisBLEServerCallbacks());

  ble_service = ble_server->createService(BLE_SERVICE_UUID);

  ble_status_characteristic = ble_service->createCharacteristic(
    BLE_STATUS_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  ble_status_characteristic->addDescriptor(new BLE2902());

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

  int measurement_progress = 0;
  drawMeasuringScreen(measurement_progress, MEASUREMENT_PROGRESS_TOTAL);

  uint32_t up_values[AS7265X_CHANNEL_COUNT] = {0};
  uint32_t down_values[AS7265X_CHANNEL_COUNT] = {0};

  static const CalibrationProfileIndex up_base_profiles[] = {
    PROFILE_UP_100X1,
    PROFILE_UP_255X1,
    PROFILE_UP_255X3
  };
  static const CalibrationProfileIndex up_high_profiles[] = {
    PROFILE_UP_100X1_G16,
    PROFILE_UP_255X1_G16,
    PROFILE_UP_255X3_G16
  };
  if (!capture_adaptive_profiles(
        up_base_profiles,
        up_high_profiles,
        sizeof(up_base_profiles) / sizeof(up_base_profiles[0]),
        up_values,
        UP_MIN_RAW_SIGNAL,
        last_up_profile,
        last_up_gain_setting,
        last_up_saturated,
        measurement_progress
      )) return false;

  if (measurement_progress < UP_PROGRESS_COMPLETE) {
    measurement_progress = UP_PROGRESS_COMPLETE;
    updateMeasuringProgress(measurement_progress, MEASUREMENT_PROGRESS_TOTAL);
  }

  const CalibrationProfileDefinition &up_profile = CALIBRATION_PROFILES[last_up_profile];
  last_up_integration_cycles = up_profile.integration_cycles;
  last_up_exposure_count = up_profile.exposures;

  static const CalibrationProfileIndex down_base_profiles[] = {
    PROFILE_DOWN_255X1,
    PROFILE_DOWN_255X3
  };
  static const CalibrationProfileIndex down_high_profiles[] = {
    PROFILE_DOWN_255X1_G16,
    PROFILE_DOWN_255X3_G16
  };
  if (!capture_adaptive_profiles(
        down_base_profiles,
        down_high_profiles,
        sizeof(down_base_profiles) / sizeof(down_base_profiles[0]),
        down_values,
        DOWN_MIN_RAW_SIGNAL,
        last_down_profile,
        last_down_gain_setting,
        last_down_saturated,
        measurement_progress
      )) return false;
  if (measurement_progress < MEASUREMENT_PROGRESS_TOTAL) {
    measurement_progress = MEASUREMENT_PROGRESS_TOTAL;
    updateMeasuringProgress(measurement_progress, MEASUREMENT_PROGRESS_TOTAL);
  }
  last_down_exposure_count = CALIBRATION_PROFILES[last_down_profile].exposures;

  bool up_low = raw_signal_is_low(up_values, UP_MIN_RAW_SIGNAL);
  bool down_low = raw_signal_is_low(down_values, DOWN_MIN_RAW_SIGNAL);
  last_measurement_low_light = up_low || down_low;

  const CalibrationProfileDefinition &down_profile = CALIBRATION_PROFILES[last_down_profile];

  uint32_t iU_raw = up_values[RED_CHANNEL_INDEX];
  uint32_t vU_raw = up_values[NIR_CHANNEL_INDEX];

  uint32_t iD_raw = down_values[RED_CHANNEL_INDEX];
  uint32_t vD_raw = down_values[NIR_CHANNEL_INDEX];
  float iU_input = dark_corrected_calibration_input(last_up_profile, RED_CHANNEL_INDEX, (float)iU_raw);
  float vU_input = dark_corrected_calibration_input(last_up_profile, NIR_CHANNEL_INDEX, (float)vU_raw);
  float iD_input = dark_corrected_calibration_input(last_down_profile, RED_CHANNEL_INDEX, (float)iD_raw);
  float vD_input = dark_corrected_calibration_input(last_down_profile, NIR_CHANNEL_INDEX, (float)vD_raw);
  last_raw_diffuser_iU = iU_input;
  last_raw_diffuser_vU = vU_input;
  last_raw_diffuser_iD = iD_input;
  last_raw_diffuser_vD = vD_input;
  raw_diffuser_ndvi = calculate_raw_diffuser_ndvi(
    up_profile,
    down_profile,
    iU_input,
    vU_input,
    iD_input,
    vD_input
  );
  message_raw_diffuser_NDVI = String(raw_diffuser_ndvi, 2);
  measurement_sequence++;
  last_measurement_ms = millis();

  battery_soc = read_voltage_percent();
  updateBLEStatusValue();

  Serial.print("UP accumulation: ");
  Serial.print(last_up_exposure_count);
  Serial.print(" x ");
  Serial.print(last_up_integration_cycles);
  Serial.println(" cycles");
  Serial.print("UP gain: ");
  Serial.println(sensor_gain_label(last_up_gain_setting));
  Serial.print("UP profile: ");
  Serial.println(up_profile.id);
  Serial.print("Raw + diffuser NDVI: ");
  Serial.println(message_raw_diffuser_NDVI);
  print_raw_channels_legacy("UP raw:", up_values);
  Serial.println("------------------------------------");
  Serial.print("DOWN accumulation: ");
  Serial.print(last_down_exposure_count);
  Serial.print(" x ");
  Serial.print(DOWN_INTEGRATION_CYCLES);
  Serial.println(" cycles");
  Serial.print("DOWN gain: ");
  Serial.println(sensor_gain_label(last_down_gain_setting));
  Serial.print("DOWN profile: ");
  Serial.println(CALIBRATION_PROFILES[last_down_profile].id);
  print_raw_channels_legacy("DOWN raw:", down_values);

  return true;
}

// ================= SETUP & LOOP =================
void setup() {
  Serial.begin(115200);
  Serial.println("Manti NDVI meter ready. Send 'm' to measure.");
  Serial.print("Firmware: ");
  Serial.print(FIRMWARE_VERSION);
  Serial.print(" (");
  Serial.print(FIRMWARE_BUILD_DATE);
  Serial.println(")");

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

  loadDeviceIdentity();

  // Read legacy metadata before BLE allocates heap for its stack.
  load_calibration_from_flash();
  setupBLE();
  i2c_init_buses_once();

  up_ok   = try_init_up_sensor();
  down_ok = try_init_down_sensor();
  updateBLEStatusValue();

  if (up_ok && down_ok) {
    drawNDVIScreen();
  } else {
    drawStatusScreen();
  }

  buttonRawDown = digitalRead(BTN_MEASURE) == LOW;
  buttonDown = buttonRawDown;
  buttonRawChangedAt = millis();
  buttonPressStartedAt = buttonRawChangedAt;

  const esp_partition_t *runningPartition = esp_ota_get_running_partition();
  esp_ota_img_states_t otaState;
  if (runningPartition &&
      esp_ota_get_state_partition(runningPartition, &otaState) == ESP_OK &&
      otaState == ESP_OTA_IMG_PENDING_VERIFY) {
    esp_ota_mark_app_valid_cancel_rollback();
  }
}

void loop() {
  if (!blePausedForPortal) serviceBLE();
  if (portalActive()) {
    String previousMessage = portalMessage();
    portalLoop();
    if (!portalActive()) {
      // Reboot restores the normal BLE service and measurement screen cleanly.
      if (blePausedForPortal) { ESP.restart(); return; }
      if (up_ok && down_ok) drawNDVIScreen();
      else drawStatusScreen();
    } else if (portalMessage() != previousMessage) {
      drawPortalScreen();
    }
  }
  retry_sensor_init_periodic();

  unsigned long now = millis();
  if (now - last_batt_sample_ms >= BATT_SAMPLE_PERIOD_MS) {
    last_batt_sample_ms = now;
    battery_soc = read_voltage_percent();
  }

  bool button_trigger = false;
  bool rawDown = digitalRead(BTN_MEASURE) == LOW;
  if (rawDown != buttonRawDown) {
    buttonRawDown = rawDown;
    buttonRawChangedAt = now;
  }
  if (rawDown != buttonDown && now - buttonRawChangedAt >= BUTTON_DEBOUNCE_MS) {
    buttonDown = rawDown;
    if (buttonDown) {
      buttonPressStartedAt = now;
      buttonActionHandled = false;
      if (portalActive()) {
        portalEnd();
        buttonActionHandled = true;
        if (blePausedForPortal) { ESP.restart(); return; }
        if (up_ok && down_ok) drawNDVIScreen();
        else drawStatusScreen();
      }
    } else if (!buttonActionHandled && !portalActive()) {
      button_trigger = true;
    }
  }
  if (buttonDown && !buttonActionHandled && !portalActive() &&
      now - buttonPressStartedAt >= PORTAL_HOLD_MS) {
    buttonActionHandled = true;
    // BLE and TLS compete for the ESP32's heap; BLE resumes after setup exits.
    blePausedForPortal = true;
    BLEDevice::deinit(false);
    ble_status_characteristic = nullptr;
    ble_service = nullptr;
    ble_server = nullptr;
    ble_set_connected(false);
    Serial.printf("Setup heap after BLE pause: %u\n", ESP.getFreeHeap());
    if (portalBegin(BLE_DEVICE_NAME, FIRMWARE_VERSION)) drawPortalScreen();
    else ESP.restart();
  }
  bool serial_trigger = poll_serial_commands();

  if (button_trigger || serial_trigger) {
    if (serial_trigger) Serial.println(">> Serial trigger received.");

    up_ok   = false;
    down_ok = false;
    up_ok   = try_init_up_sensor();
    down_ok = try_init_down_sensor();
    updateBLEStatusValue();

    if (take_measurements_and_compute_ndvi()) {
      if (portalActive()) drawPortalScreen();
      else drawNDVIScreen();
    } else {
      if (portalActive()) drawPortalScreen();
      else drawStatusScreen();
    }
  }

  delay(20);
}
