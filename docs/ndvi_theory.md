# NDVI Explanation and Measurement Principles

## Purpose of this document

This document explains what NDVI is, how it is measured, and how it is implemented in this device.

The goal is not to present NDVI as a black-box metric, but to clarify:

- What physical quantities are being measured
- How those measurements are combined
- What assumptions are involved
- Where limitations arise

This aligns with the project's intent as a reference and learning instrument, not a certified diagnostic tool.

## What NDVI is

NDVI (Normalized Difference Vegetation Index) is a numerical indicator that relates plant health to how vegetation interacts with light.

It is based on two well-established observations:

- **Healthy vegetation absorbs red light** for photosynthesis
- **Healthy vegetation reflects near-infrared (NIR) light** due to internal leaf structure

NDVI combines these two effects into a single normalized value:

```
NDVI = (NIR − RED) / (NIR + RED)
```

The resulting value typically ranges between –1 and +1.

## Interpreting NDVI values

While exact thresholds depend on context, typical interpretations are:

**0.5 – 0.9**
Dense, healthy vegetation

**0.2 – 0.5**
Sparse, stressed, or partially senescent vegetation

**0.0 – 0.2**
Very sparse vegetation or mixed soil/background

**< 0.0**
Non-vegetated surfaces such as soil, concrete, or water

NDVI indicates relative vegetation condition, not a specific diagnosis.

## What is actually measured

**NDVI is not measured directly.**
It is computed from measurements of reflected light.

This device measures:

- Spectral intensity in discrete wavelength bands
- Using calibrated photodiode-based spectral sensors
- Across both visible and near-infrared regions

Each measurement represents relative reflected light intensity, not absolute radiance.

## Why this device uses two spectral sensors

### The challenge of outdoor measurements

In real outdoor conditions, illumination is constantly changing due to:

- Sun angle
- Cloud cover
- Shadows
- Atmospheric conditions

If illumination changes, reflected light measurements change — even if vegetation does not.

### Downward-facing sensor (surface reflectance)

The downward-facing sensor measures light reflected from the surface beneath the device.

This sensor captures:

- Reflected red light
- Reflected near-infrared light
- A mixture of vegetation and background within its field of view

This measurement contains the vegetation signal of interest.

### Upward-facing sensor (incident illumination)

The upward-facing sensor measures the ambient light illuminating the scene.

This sensor captures:

- The spectral composition of incoming light
- Short-term changes caused by clouds or sun angle
- Overall illumination intensity

This represents what light is available at the time of measurement.

### Why both are used together

Using only a downward-facing sensor assumes stable illumination, which is rarely true outdoors.

By measuring both:

- **Incident light** (upward-facing sensor)
- **Reflected light** (downward-facing sensor)

the system can normalise reflected measurements against current lighting conditions.

This improves:

- Measurement repeatability
- Comparability between consecutive readings
- Interpretation under changing weather conditions

This approach mirrors techniques used in professional field spectrometers and handheld vegetation sensors.

## How NDVI is computed in this device

At a high level, the process is:

1. Measure incident spectral light (upward sensor)
2. Measure reflected spectral light (downward sensor)
3. Select appropriate bands to represent RED and NIR
4. Apply normalisation and averaging
5. Compute NDVI using the standard formula

The exact band selection and processing steps are documented in the firmware.

This is a band-based approximation of NDVI, not a hyperspectral implementation.

## Assumptions and limitations

It is important to understand what this design assumes:

- Illumination is reasonably uniform within the sensor field of view
- The measured area is dominated by the surface of interest
- Sensor temperature and electronics remain stable over short intervals
- Measurements are relative, not absolutely calibrated radiance

Known limitations include:

- Mixed pixels (vegetation + soil)
- Strong shadows or specular reflections
- Very sparse or very dense canopies
- Artificial or highly directional lighting

The upward-facing sensor improves consistency, but does not eliminate all sources of error.

## What this device does not claim

This device does not claim:

- Absolute radiometric calibration
- Satellite-level NDVI equivalence
- Disease detection or agronomic prescription
- Certified measurement accuracy

It is intended as a transparent reference implementation, not a decision-making authority.

## Why this explanation matters

NDVI values are often treated as single numbers without context.

This project intentionally exposes:

- The physics behind the measurement
- The assumptions involved
- The reasons values may vary

Understanding these factors leads to better interpretation, better validation, and more responsible use of NDVI data.

## Summary

This device implements NDVI using discrete spectral measurements of red and near-infrared light, combined with incident-light normalisation via a dual-sensor design. The approach improves repeatability in real-world conditions while remaining transparent about assumptions and limitations. The goal is understanding and experimentation, not black-box output.

---

*License: CC BY 4.0*
