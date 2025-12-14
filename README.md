# Open NDVI Handheld

**A reference open-source handheld NDVI instrument**

Open-source handheld NDVI device based on the ESP32 Feather V2 and AS7265x Triad spectral sensors, intended as a transparent reference design for vegetation sensing, experimentation, and system integration.

## What is this project?

This repository contains a fully open hardware and software implementation of a handheld NDVI (Normalized Difference Vegetation Index) measurement device.

It is designed to:

- Expose how NDVI measurements are actually produced
- Enable validation, comparison, and experimentation
- Serve as a building block for larger sensing systems

The device measures reflected light in visible and near-infrared bands and computes NDVI locally, producing structured digital output suitable for further analysis or integration.

## Why E-Nano is releasing this

E-Nano develops sensing systems and automation platforms that turn physical measurements into structured, actionable data across multiple industries (sports surfaces, agriculture, construction, and environmental monitoring).

In practice, we repeatedly encountered the same problem:

**Many vegetation indices are treated as "black boxes", making validation, comparison, and integration difficult.**

This project exists to address that problem by providing:

- A transparent reference implementation
- A known, inspectable signal path
- A device that can be understood, modified, and extended

This open handheld NDVI tool is not a commercial product, but a reference instrument that complements E-Nano's wider ecosystem.

## What NDVI means (briefly)

NDVI compares how vegetation reflects:

- **Red light** (absorbed for photosynthesis)
- **Near-infrared light** (strongly reflected by healthy plant structure)

The resulting value typically ranges from:

- **High NDVI (≈ 0.5–0.9)** → healthy, dense vegetation
- **Moderate NDVI (≈ 0.2–0.5)** → stressed or sparse vegetation
- **Low / negative NDVI** → soil, hard surfaces, water

NDVI is an indicator, not a diagnosis, and its reliability depends heavily on measurement context and methodology.

## What this device is intended for

This project is intended for:

- Engineers building or validating sensing systems
- Researchers and students studying vegetation indices
- Turf, agronomy, or environmental professionals exploring NDVI behaviour
- Developers integrating NDVI into larger data pipelines
- Anyone who wants to understand NDVI beyond a single numeric output

It is not intended to replace certified instruments or professional judgement.

## What you will find in this repository

This repository contains everything required to understand and reproduce the device:

### Firmware
- Arduino-based ESP32 code
- NDVI calculation logic
- Sensor configuration and data output

### Hardware
- Full schematic and PCB design (Fusion / Eagle)
- Manufacturing files (Gerbers, BOM)
- Modular carrier-board architecture

### Mechanical
- 3D-printable enclosure (STL + STEP)
- Assembly notes

### Documentation
- Assembly and wiring guides
- NDVI theory and limitations
- Example output data

Nothing is hidden behind proprietary firmware or closed hardware.

## Quick Start Guide

### 📦 **Components Required (~$244 total)**

| Component | Cost | Supplier |
|-----------|------|----------|
| [ESP32 Feather V2](https://www.adafruit.com/product/5400) | $22.50 | Adafruit |
| [2x AS7265x Sensors](https://www.sparkfun.com/products/15050) | $74.95 each | SparkFun |
| [240x240 LCD Display](https://shop.pimoroni.com/products/1-3-spi-colour-lcd-240x240-breakout) | $16.50 | Pimoroni |
| [LiPo Battery](https://shop.pimoroni.com/products/lipo-battery-pack) | $12.00 | Pimoroni |
| Custom Carrier PCB | ~$20 | PCB Manufacturer |
| 3D Printed Enclosure | ~$15 | 3D Printing Service |
| Switches & Connectors | ~$8 | Generic |

### 🔧 **Build Process**

1. **Order components** and send `hardware/gerbers/` files to PCB manufacturer
2. **Assemble** carrier PCB with all modules (see [Assembly Guide](docs/assembly_guide.md))
3. **Program firmware** using Arduino IDE (see setup below)
4. **Test device** and take first measurements

### 💻 **Arduino IDE Setup**

1. **Install Arduino IDE** from [arduino.cc](https://www.arduino.cc/)

2. **Add ESP32 Board Support:**
   - File → Preferences
   - Additional Board Manager URLs: `https://espressif.github.io/arduino-esp32/package_esp32_dev_index.json`
   - Tools → Board → Boards Manager → Search "esp32" → Install

3. **Install Libraries** (Library Manager):
   - SparkFun AS7265x Arduino Library
   - Adafruit ST7735 and ST7789 Library
   - Adafruit GFX Library

4. **Programming:**
   - Connect ESP32 via USB
   - Board: "Adafruit ESP32 Feather"
   - Open: `firmware/ndvi_handheld/ndvi_handheld.ino`
   - Upload ⬆️

### 📱 **Using the Device**

**Display shows:**
- **Large NDVI number** (color-coded: green=healthy, yellow=moderate, white=sparse)
- **Battery percentage**
- **"LOW LIGHT"** warning when too dark for accurate readings

## Design philosophy

Key principles behind this project:

**Transparency over optimisation**
Readability and traceability are prioritised over maximum performance.

**Reference, not product**
This is a baseline instrument meant to be built upon.

**Interoperability**
Data output is designed to be easy to log, parse, and integrate.

**Modularity**
Firmware, hardware, and enclosure can all be adapted independently.

## Relationship to E-Nano's commercial systems

This open project:

- Does not include E-Nano's proprietary analysis pipelines
- Does not include commercial calibration workflows
- Does not claim agronomic decision-making accuracy

E-Nano's commercial offerings build on similar sensing principles, but add:

- Controlled calibration
- Repeatability guarantees
- Quality assurance
- Integration into larger automation and analysis platforms

This repository exists to enable understanding and experimentation, not to replace professional systems.

## Repository structure (high level)

```
open-ndvi-handheld/
├── firmware/        # ESP32 firmware (Arduino)
├── hardware/        # Schematic, PCB, BOM, Gerbers
├── mechanical/      # 3D-printable enclosure
├── docs/            # Assembly, theory, documentation
└── LICENSES/        # Multi-license structure
```

## Licensing

Different parts of the project use appropriate open licences:

- **Firmware**: MIT
- **Hardware (schematic & PCB)**: CERN OHL-W v2
- **Mechanical design**: CC BY-SA 4.0
- **Documentation**: CC BY 4.0

See the `LICENSES/` directory for full texts.

## Documentation

- 🔧 [Assembly Guide](docs/assembly_guide.md) - Detailed assembly instructions
- 📐 [Wiring & Pinout](docs/wiring_and_pinout.md) - Technical connections
- 🧪 [NDVI Theory](docs/ndvi_theory.md) - Scientific background
- ❓ [FAQ](docs/faq.md) - Common questions and troubleshooting

## Contributing

Contributions are welcome, particularly:

- Documentation improvements
- Validation experiments
- Firmware extensions
- Mechanical or hardware variants

Please read:

- [CONTRIBUTING.md](CONTRIBUTING.md)
- [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md)

## Disclaimer

This is an experimental reference instrument provided as-is.

It is not certified, not safety-critical, and not intended to be used as the sole basis for agronomic or environmental decisions.

Released by E-Nano to support transparency, learning, and open experimentation in vegetation sensing.
