# Hardware - Open NDVI Handheld

This directory contains all hardware design files for the Open NDVI Handheld device.

## Overview

The hardware consists of a custom carrier PCB designed to connect modular components: ESP32 Feather V2, dual AS7265x spectral sensors, ST7789 240x240 LCD display, battery, switches, and connectors. This modular approach simplifies assembly, troubleshooting, and component replacement.

## Directory Structure

- `fusion_eagle/` - Autodesk Fusion/Eagle design files
- `gerbers/` - Manufacturing files for PCB production
- `bom/` - Bill of Materials
- `outputs/` - Generated documentation (schematics, assembly drawings)
- `datasheets/` - Component datasheets (AS7265x, ESP32, SSD1351, etc.)

## Design Files

### Fusion/Eagle Files
- `ndvi_v1.sch` - Schematic design
- `ndvi_v1.brd` - PCB layout
- `ndvi_v1.f3d` - Fusion 360 3D model
- `ndvi_v1.f2d` - Fusion 360 2D drawings

### Manufacturing Outputs
- `gerbers/ndvi_gerbers.zip` - Gerber files for PCB fabrication
- `bom/bom.csv` - Complete bill of materials
- `outputs/schematic.pdf` - Printable schematic
- `outputs/pcb_top.pdf` - Top layer assembly drawing
- `outputs/pcb_bottom.pdf` - Bottom layer assembly drawing

## Key Components

- **ESP32 Feather V2** (Adafruit #5400) - Main microcontroller with WiFi/Bluetooth
- **2x AS7265x Triad Sensors** (SparkFun SEN-15050) - 18-channel spectral sensors
- **ST7789 240x240 LCD** (Pimoroni 1.3" SPI) - Color display with simplified interface
- **LiPo Battery** (Pimoroni) - Rechargeable power source
- **Power Switch** - On/off control
- **Momentary Button** - Measurement trigger (A0)
- **Carrier PCB** - Custom board connecting all modules
- **Headers/Connectors** - Modular connections for easy assembly

## PCB Specifications

- 4-layer PCB design
- Dimensions: TBD
- Thickness: 1.6mm standard
- Finish: HASL or ENIG
- Via size: 0.2mm minimum

## Assembly Notes

See `mechanical/assembly_notes.md` for complete assembly instructions.

## License

Hardware designs are licensed under CERN OHL-W v2. See `LICENSES/LICENSE_HARDWARE_CERN-OHL-W.txt`.
