/*************************************************** 
  AS7265X dual-sensor outdoor NDVI meter
  - Bulbs OFF (ambient light)
  - Uses channels I (red-ish) and V (NIR-ish) for NDVI
  - UP sensor: incident light (sky)
  - DOWN sensor: reflected light (turf)
  - Prints all channels A–L, R–W
  - Shows NDVI + battery % on ST7789 240x240 LCD
  - Simple interface: large NDVI value or "LOW LIGHT"
  - Measure ONLY when A0 is pressed
****************************************************/

#include "SparkFun_AS7265X.h"
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

// ---------------- I2C ----------------
#define I2C1_SDA_PIN D7   // DOWN sensor
#define I2C1_SCL_PIN A4
#define I2C2_SDA_PIN SDA  // UP sensor
#define I2C2_SCL_PIN SCL

AS7265X sensor_DOWN;
AS7265X sensor_UP;

// ---------------- ST7789 240x240 LCD ----------------
#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 240
#define TFT_CS   D6  // Chip select
#define TFT_RST  D3  // Reset pin
#define TFT_DC   D2  // Data/Command pin

#define BLACK   0x0000
#define GREEN   0x07E0
#define RED     0xF800
#define YELLOW  0xFFE0  
#define WHITE   0xFFFF
#define BLUE    0x001F

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// ---------------- Button ----------------
#define BTN_MEASURE A0
bool lastMeasureState = false;

// ---------------- Battery (single LiPo 3.7V) ----------------
#define BATT A3

float soc_avg = 0.0;
int battery_soc = 0;

void configure_batt() {
  pinMode(BATT, INPUT);
  analogSetAttenuation(ADC_11db); // ESP32 ADC range up to ~3.6V at pin
}

// Piecewise LiPo voltage -> SoC (%) table (approx, under light load)
float voltagePoints[] = {
  3.30, // 0%
  3.45, // 10%
  3.55, // 20%
  3.68, // 30%
  3.73, // 40%
  3.79, // 50%
  3.85, // 60%
  3.92, // 70%
  4.00, // 80%
  4.08, // 90%
  4.12, // 95%
  4.20  // 100%
};

float socPoints[] = {
   0,
  10,
  20,
  30,
  40,
  50,
  60,
  70,
  80,
  90,
  95,
 100
};

// Map battery voltage to approximate SoC using linear interpolation
float voltageToPercent(float v) {
  // Clamp to min/max in table
  if (v <= voltagePoints[0]) return 0.0f;
  if (v >= voltagePoints[11]) return 100.0f;

  // Find segment
  for (int i = 0; i < 11; i++) {
    float v1 = voltagePoints[i];
    float v2 = voltagePoints[i + 1];
    if (v >= v1 && v <= v2) {
      float s1 = socPoints[i];
      float s2 = socPoints[i + 1];
      float t = (v - v1) / (v2 - v1);
      return s1 + t * (s2 - s1);
    }
  }
  return 0.0f; // shouldn't reach here
}

int read_voltage() {
  float raw = analogRead(BATT);
  // Same divider + fudge as your original code
  float voltage = (raw / 4095.0f) * 3.3f * 1.065f * 2.0f; // estimated battery voltage
  Serial.println("Voltage");
  Serial.println(raw);
  Serial.println(voltage);

  // Smooth the voltage
  if (soc_avg == 0.0f) {
    soc_avg = voltage;
  } else {
    soc_avg = 0.8f * soc_avg + 0.2f * voltage;
  }

  float pct = voltageToPercent(soc_avg);

  Serial.print("Battery raw V: ");
  Serial.print(voltage, 3);
  Serial.print(" V  | avg: ");
  Serial.print(soc_avg, 3);
  Serial.print(" V  | SoC: ");
  Serial.print(pct, 1);
  Serial.println(" %");

  // Clamp and convert to int
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return (int)pct;
}

// ---------------- NDVI ----------------
String message_NDVI = "---";
float ndvi = 0.0;

// ================= TRIAD SETUP =================
void setup_triad() {
  Serial.println("AS7265x Spectral Triad - outdoor mode (ambient, I/V NDVI)");

  // I2C for DOWN sensor
  Wire.begin(I2C1_SDA_PIN, I2C1_SCL_PIN, 100000);
  delay(10);

  // I2C for UP sensor
  Wire1.begin(I2C2_SDA_PIN, I2C2_SCL_PIN, 100000);
  delay(10);

  // Init sensors
  if (!sensor_DOWN.begin(Wire)) {
    Serial.println("Error: DOWN sensor not found!");
    while (1) delay(100);
  }
  if (!sensor_UP.begin(Wire1)) {
    Serial.println("Error: UP sensor not found!");
    while (1) delay(100);
  }

  // Turn off status LEDs
  sensor_UP.disableIndicator();
  sensor_DOWN.disableIndicator();

  // Ensure bulbs are OFF for outdoor NDVI
  sensor_UP.disableBulb(AS7265x_LED_WHITE);
  sensor_DOWN.disableBulb(AS7265x_LED_WHITE);

  // Integration time & gain (tune if needed)
  // setIntegrationCycles(N) -> N * 2.8 ms
  sensor_UP.setIntegrationCycles(50);    // ~140 ms
  sensor_DOWN.setIntegrationCycles(50);

  // Gains: AS7265X_GAIN_1X, _37X, _16X, _64X
  sensor_UP.setGain(AS7265X_GAIN_37X);
  sensor_DOWN.setGain(AS7265X_GAIN_37X);

  Serial.println("Triad setup complete.");
}

// ================= MEASURE & NDVI (I/V) =================
void getTriadReadings() {
  // ---- Ambient measurement UP (incident) ----
  sensor_UP.takeMeasurements();
  float aU = sensor_UP.getCalibratedA();
  float bU = sensor_UP.getCalibratedB();
  float cU = sensor_UP.getCalibratedC();
  float dU = sensor_UP.getCalibratedD();
  float eU = sensor_UP.getCalibratedE();
  float fU = sensor_UP.getCalibratedF();
  float gU = sensor_UP.getCalibratedG();
  float hU = sensor_UP.getCalibratedH();
  float iU = sensor_UP.getCalibratedI(); // RED (chosen)
  float jU = sensor_UP.getCalibratedJ();
  float kU = sensor_UP.getCalibratedK();
  float lU = sensor_UP.getCalibratedL();
  float rU = sensor_UP.getCalibratedR();
  float sU = sensor_UP.getCalibratedS();
  float tU = sensor_UP.getCalibratedT();
  float uU = sensor_UP.getCalibratedU();
  float vU = sensor_UP.getCalibratedV(); // NIR (chosen)
  float wU = sensor_UP.getCalibratedW();

  // ---- Ambient measurement DOWN (reflected) ----
  sensor_DOWN.takeMeasurements();
  float aD = sensor_DOWN.getCalibratedA();
  float bD = sensor_DOWN.getCalibratedB();
  float cD = sensor_DOWN.getCalibratedC();
  float dD = sensor_DOWN.getCalibratedD();
  float eD = sensor_DOWN.getCalibratedE();
  float fD = sensor_DOWN.getCalibratedF();
  float gD = sensor_DOWN.getCalibratedG();
  float hD = sensor_DOWN.getCalibratedH();
  float iD = sensor_DOWN.getCalibratedI(); // RED (down)
  float jD = sensor_DOWN.getCalibratedJ();
  float kD = sensor_DOWN.getCalibratedK();
  float lD = sensor_DOWN.getCalibratedL();
  float rD = sensor_DOWN.getCalibratedR();
  float sD = sensor_DOWN.getCalibratedS();
  float tD = sensor_DOWN.getCalibratedT();
  float uD = sensor_DOWN.getCalibratedU();
  float vD = sensor_DOWN.getCalibratedV(); // NIR (down)
  float wD = sensor_DOWN.getCalibratedW();

  // ---- NDVI using I (red) and V (NIR) ----
  float red_UP = iU;
  float nir_UP = vU;
  float red_DN = iD;
  float nir_DN = vD;

  float red_ratio = (red_UP != 0.0f) ? (red_DN / red_UP) : 0.0f;
  float nir_ratio = (nir_UP != 0.0f) ? (nir_DN / nir_UP) : 0.0f;

  float denom = nir_ratio + red_ratio;
  if (denom != 0.0f) {
    ndvi = (nir_ratio - red_ratio) / denom;
  } else {
    ndvi = 0.0f;
  }
  message_NDVI = String(ndvi, 3);

  // Update battery reading each time we measure
  battery_soc = read_voltage();

  // ---- SERIAL OUTPUT ----
  Serial.println("===================================================");
  Serial.println("UP SENSOR (TOP) - ambient, all channels");
  Serial.println(" A–L:");
  Serial.printf("  %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
                aU,bU,cU,dU,eU,fU,gU,hU,iU,jU,kU,lU);
  Serial.println(" R–W:");
  Serial.printf("  %.4f %.4f %.4f %.4f %.4f %.4f\n", rU,sU,tU,uU,vU,wU);

  Serial.println();
  Serial.println("DOWN SENSOR (BOTTOM) - ambient, all channels");
  Serial.println(" A–L:");
  Serial.printf("  %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
                aD,bD,cD,dD,eD,fD,gD,hD,iD,jD,kD,lD);
  Serial.println(" R–W:");
  Serial.printf("  %.4f %.4f %.4f %.4f %.4f %.4f\n", rD,sD,tD,uD,vD,wD);

  Serial.println();
  Serial.println(" iD, vD:");
  Serial.printf("  %.4f %.4f\n", iD, vD);

  Serial.print("NDVI (I/V, ambient): ");
  Serial.println(message_NDVI);
  Serial.print("Battery: ");
  Serial.print(battery_soc);
  Serial.println(" %");
  Serial.println("===================================================");
}

// ================= LCD DISPLAY (Simplified Interface) =================
void drawNDVIScreen() {
  tft.fillScreen(BLACK);

  // Battery indicator (top-right)
  tft.setCursor(150, 10);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("BAT: ");
  tft.print(battery_soc);
  tft.println("%");

  // Check for low light conditions
  bool lowLight = false;
  if (message_NDVI != "---") {
    // Consider low light if readings are very low
    // This is a simple check - you may want to adjust thresholds
    lowLight = (ndvi < -0.5 || ndvi > 1.0); // Invalid NDVI range suggests poor lighting
  }

  if (lowLight) {
    // Show LOW LIGHT warning
    tft.setCursor(30, 100);
    tft.setTextColor(RED);
    tft.setTextSize(4);
    tft.println("LOW LIGHT");
  } else {
    // Show NDVI value (large and centered)
    tft.setCursor(50, 100);
    if (ndvi >= 0.5) {
      tft.setTextColor(GREEN);  // Good vegetation
    } else if (ndvi >= 0.2) {
      tft.setTextColor(YELLOW); // Moderate vegetation
    } else {
      tft.setTextColor(WHITE);  // Low vegetation or non-vegetation
    }
    tft.setTextSize(5);
    tft.println(message_NDVI);
  }

  // Instruction text (bottom)
  tft.setCursor(30, 200);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Press to measure");
}

// ================= SETUP & LOOP =================
void setup(void) {
  Serial.begin(9600);
  WiFi.mode(WIFI_OFF);
  setCpuFrequencyMhz(160);
  pinMode(BTN_MEASURE, INPUT_PULLUP);

  configure_batt();
  battery_soc = read_voltage(); // initial reading

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(BLACK);

  setup_triad();
  drawNDVIScreen();
}

void loop() {
  bool current = (digitalRead(BTN_MEASURE) == LOW);
  if (!lastMeasureState && current) {
    getTriadReadings();
    drawNDVIScreen();
  }
  lastMeasureState = current;
  delay(20);
}
