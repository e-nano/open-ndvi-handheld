# Firmware - Open NDVI Handheld

This directory contains the firmware for the ESP32 Feather V2 based NDVI handheld device.

## Overview

The firmware is designed to work with both Arduino IDE and PlatformIO development environments. It interfaces with the SparkFun AS7265x Triad spectral sensor to measure light in multiple wavelengths and compute NDVI values.

## Hardware Requirements

- ESP32 Feather V2 (Adafruit)
- SparkFun AS7265x Triad Spectral Sensor
- Custom PCB (see `hardware/` directory)
- Optional: Display for real-time readings

## Development Options

### Arduino IDE
- Open `ndvi_handheld/ndvi_handheld.ino` in Arduino IDE
- Install required libraries (see Dependencies below)
- Select "Adafruit ESP32 Feather" as board
- Upload to device

### PlatformIO
- Use `platformio.ini` configuration from the firmware/ directory
- Run `pio run` to build
- Run `pio run --target upload` to flash

## Dependencies

Required libraries:
- SparkFun AS7265x Arduino Library (for spectral sensors)
- Adafruit ST7735 and ST7789 Library (for 240x240 LCD display)
- Adafruit GFX Library (graphics support)
- Wire (ESP32 built-in I2C)
- WiFi (ESP32 built-in)
- SPI (ESP32 built-in for display)

## Firmware Features

- Multi-spectral light measurement
- NDVI calculation algorithms
- Serial output for data logging
- WiFi connectivity for remote data transfer
- Power management for battery operation
- Calibration routines

## Configuration

Configuration constants are defined at the top of `ndvi_handheld/ndvi_handheld.ino`:
- Pin assignments (I2C, SPI, button, battery)
- Display settings (screen dimensions, colors)
- Sensor settings (integration cycles, gain)
- Battery voltage calibration table

## NDVI Calculation

The firmware implements standard NDVI calculation:
```
NDVI = (NIR - Red) / (NIR + Red)
```

Where:
- NIR = Near-infrared reflectance (~800-900nm)
- Red = Red light reflectance (~600-700nm)

## Usage

1. Power on the device
2. Wait for sensor initialization
3. Point at vegetation sample
4. Press measure button or wait for auto-measurement
5. Read NDVI value from display or serial output

## Serial Output Format

The device outputs measurement data in a human-readable text format:

```
UP SENSOR (TOP) - ambient, all channels
 A–L:
  0.1234 0.2345 0.3456 0.4567 0.5678 0.6789 0.7890 0.8901 0.9012 1.0123 1.1234 1.2345
 R–W:
  1.3456 1.4567 1.5678 1.6789 1.7890 1.8901

DOWN SENSOR (BOTTOM) - ambient, all channels
 A–L:
  0.0987 0.1876 0.2765 0.3654 0.4543 0.5432 0.6321 0.7210 0.8099 0.9088 1.0077 1.1066
 R–W:
  1.2055 1.3044 1.4033 1.5022 1.6011 1.7000

 iD, vD:
  0.8099 1.6011

NDVI (I/V, ambient): 0.328
Battery: 85 %
```

**Output Details:**
- All spectral channel readings A-L and R-W for both UP and DOWN sensors
- Key values iD (red channel) and vD (NIR channel) used for NDVI calculation
- Calculated NDVI value (range -1 to +1)
- Battery charge percentage

## Data Output

The AS7265x sensors are factory-calibrated and provide spectral irradiance readings. No user calibration is required - simply power on and start measuring.

## License

This firmware is licensed under the MIT License. See `LICENSES/LICENSE_FIRMWARE_MIT.txt`.
