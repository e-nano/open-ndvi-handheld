# Firmware - Open NDVI Handheld

- This folder now contains the ESP32 firmware structure copied from `Manti-S_CalibrationCode`.

## Layout

- `platformio.ini`
  - PlatformIO project config for `adafruit_feather_esp32_v2`
- `src/main.cpp`
  - Main ESP32 application
- `src/as7265x_calibration_data.h`
  - Embedded calibration tables
- `lib/SparkFun_Spectral_Triad_AS7265X/`
  - Local patched AS7265X library used by the firmware
- `ndvi_handheld/ndvi_handheld.ino`
  - Older Arduino sketch retained for reference

## Notes

- The active PlatformIO firmware is the `src/` project, not the older `.ino` sketch.
- The copied firmware includes:
  - Dual AS7265X sensor handling
  - ST7789 display output
  - Battery monitoring
  - BLE status service
  - Calibration storage via `Preferences`

## Build

```bash
cd /home/robert/Documents/Manti-S/open-ndvi-handheld/firmware
pio run
```

## Upload

```bash
cd /home/robert/Documents/Manti-S/open-ndvi-handheld/firmware
pio run --target upload
```
