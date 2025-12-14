# Assembly Notes - Open NDVI Handheld

## Overview
The Open NDVI Handheld uses a modular assembly approach with a custom carrier PCB that connects all major components via headers and connectors.

## Components Required
- ESP32 Feather V2 (Adafruit #5400)
- 2x SparkFun AS7265x Triad Sensors (SEN-15050)  
- Pimoroni 1.3" SPI LCD 240x240 (ST7789)
- Pimoroni LiPo Battery Pack
- Custom Carrier PCB
- Power switch
- Momentary push button
- Headers and connectors
- 3D printed enclosure

## Assembly Steps
1. Install headers on carrier PCB
2. Mount ESP32 Feather V2 to carrier board
3. Connect both AS7265x sensors via I2C:
   - UP sensor: incident light measurement
   - DOWN sensor: reflected light measurement  
4. Connect ST7789 display via SPI
5. Wire power switch and measurement button
6. Connect LiPo battery
7. Test all connections
8. Install in 3D printed enclosure

## Connections Summary
- ESP32 Feather V2: Main processing unit
- AS7265x UP sensor: I2C2 (SDA, SCL pins)
- AS7265x DOWN sensor: I2C1 (D7, A4 pins)
- ST7789 Display: SPI (CS=D6, DC=D2, RST=D3)
- Button: Pin A0 (with pullup)
- Battery Monitor: Pin A3

## Notes
- Modular design allows easy component replacement
- All major components are socketed/removable
- Carrier PCB simplifies wiring and assembly
- 3D printed enclosure protects components

## License
Assembly documentation licensed under CC BY-SA 4.0
