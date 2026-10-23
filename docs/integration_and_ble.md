# Integration time and BLE API

This document describes the active PlatformIO firmware in
`firmware/src/main.cpp`. The older Arduino sketch in
`firmware/ndvi_handheld/` does not implement this protocol.

## Low-light measurement profiles

Both AS7265x sensors start each profile at 3.7x gain. If either NDVI band is
low, firmware retries that profile at 16x before increasing integration.
The saved profile-valid mask controls 16x eligibility. Saturated 16x profiles
are skipped; the NDVI calculation normalises the selected gains.

An AS7265x integration cycle is approximately 2.8 ms. Six-channel one-shot
mode converts two banks, so one exposure takes approximately:

```text
exposure time = 2 x 2.8 ms x (integration cycles + 1)
```

The times below are sensor conversion estimates. Display, I2C, and firmware
overhead add some delay.

| Level | Profile | Cycles | Exposures | Approx. integration | Purpose |
|---|---|---:|---:|---:|---|
| 1 | `up_100x1` | 100 | 1 | 566 ms | Normal-light UP reading |
| 2 | `up_255x1` | 255 | 1 | 1.434 s | First low-light retry |
| 3 | `up_255x3` | 255 | 3, summed | 4.301 s | Maximum low-light accumulation |

The adaptive UP sequence is:

1. Capture `up_100x1` at 3.7x, then at 16x if raw red or NIR is below 100.
2. If still low, repeat that gain sequence for `up_255x1`.
3. If still low, repeat it for `up_255x3`.
4. Use only the final selected profile and gain for NDVI.

Level 3 is three separate 255-cycle exposures added together. It is a sum, not
an average. Each profile can have its own saved red/NIR dark offsets.

The DOWN sensor starts with `down_255x1` at 3.7x, tries it at 16x if low, then
does the same for `down_255x3` if needed. DOWN escalation is based on the DOWN
signal, not the UP result. A saturated 16x result is not selected; firmware
continues or falls back to the final profile at 3.7x.

| DOWN profile | Cycles | Exposures | Approx. integration |
|---|---:|---:|---:|
| `down_255x1` | 255 | 1 | 1.434 s |
| `down_255x3` | 255 | 3, summed | 4.301 s |

Retries are cumulative. A measurement that tries every gain and profile takes
about 24 seconds of sensor conversion time. Typical measurements stop earlier.

`LOW=1` means the final UP or DOWN profile has a direct raw I or V count below
100. The firmware
still calculates NDVI, but an app should label or reject that result as low
confidence.

Automatic 16x attempts can be disabled reversibly by setting
`MANTI_ENABLE_16X_CALIBRATION=0` in `firmware/src/manti_features.h` and rebuilding.
The saved `caldata` partition is unchanged.

## Displayed NDVI calculation

The displayed result and BLE `NDVI` field use the dark-corrected RAW+DF inputs.
The UP inputs are divided by the characterised diffuser transmission and
normalised to the selected DOWN integration, exposure, and gain profile. No
channel calibration factor is applied. Saved schema-5 factor/input tables are
ignored; only the profile-valid mask and red/NIR dark offsets are read at boot.
The four inputs are included in BLE as
`RUI`, `RUV`, `RDI`, and `RDV`.

## BLE GATT interface

| Item | Value |
|---|---|
| Advertising name | `Manti-S-<stored number>` after USB provisioning; otherwise `Manti-S-<factory MAC>` |
| Service UUID | `9f4a6d90-f740-4d10-a947-8c6c20f4f9d1` |
| Status characteristic UUID | `2d86686f-34f8-4f1b-a4ac-f127f90f7df0` |
| Characteristic properties | Read, Notify (CCCD) |
| Encoding | UTF-8/ASCII text |
| Authentication | None |
| Write or remote-measure command | None |

Discover by service UUID rather than device name. The device number is stored
in NVS on its first USB-provisioned boot and survives later OTA updates.

Measurements are started with the physical button or `m` over the 115200-baud
USB serial connection. BLE only exposes the latest status. The device resumes
advertising after a disconnect.

### Payload format

The characteristic contains one comma-separated record with `KEY=VALUE`
fields and no newline:

```text
NDVI=0.67,RUI=845.20,RUV=1240.10,RDI=312.40,RDV=910.30,SEQ=42,MS=183921,BAT=84,UP=1,DN=1,LOW=0,UI=255,UX=1,UG=16.00,DX=1,DG=16.00,SAT=0,CVM=1023
```

| Field | Type | Meaning |
|---|---|---|
| `NDVI` | decimal | Latest RAW+DF NDVI, formatted to two decimal places and clamped to -1.00...1.00 |
| `RUI` | decimal | Dark-corrected UP red channel I input used by RAW+DF NDVI |
| `RUV` | decimal | Dark-corrected UP NIR channel V input used by RAW+DF NDVI |
| `RDI` | decimal | Dark-corrected DOWN red channel I input used by RAW+DF NDVI |
| `RDV` | decimal | Dark-corrected DOWN NIR channel V input used by RAW+DF NDVI |
| `SEQ` | unsigned integer | Measurement counter; increments after each completed measurement |
| `MS` | unsigned integer | Device uptime in milliseconds when that measurement completed |
| `BAT` | integer | Estimated battery charge, 0...100 percent |
| `UP` | `0` or `1` | UP sensor is available |
| `DN` | `0` or `1` | DOWN sensor is available |
| `LOW` | `0` or `1` | Final UP or DOWN signal is still below the low-light threshold |
| `UI` | integer | Integration cycles used by the selected UP profile: 100 or 255 |
| `UX` | integer | Exposures summed in the selected UP profile: 1 or 3 |
| `UG` | decimal | Selected UP gain: 3.70 or 16.00 |
| `DX` | integer | Exposures summed in the selected DOWN profile: 1 or 3; DOWN always uses 255 cycles |
| `DG` | decimal | Selected DOWN gain: 3.70 or 16.00 |
| `SAT` | bit mask | Saturation: bit 0 = UP, bit 1 = DOWN |
| `CVM` | bit mask | Profiles with stored calibration marked valid |

`SAT` is set if any spectral channel reaches 65535 in any exposure. Decode it
with `(SAT & 1) != 0` for UP and `(SAT & 2) != 0` for DOWN.

`CVM` uses these bits:

| Bit | Value | Profile |
|---:|---:|---|
| 0 | 1 | `up_100x1` |
| 1 | 2 | `up_255x1` |
| 2 | 4 | `up_255x3` |
| 3 | 8 | `down_255x1` |
| 4 | 16 | `down_255x3` |
| 5 | 32 | `up_100x1_g16` |
| 6 | 64 | `up_255x1_g16` |
| 7 | 128 | `up_255x3_g16` |
| 8 | 256 | `down_255x1_g16` |
| 9 | 512 | `down_255x3_g16` |

`CVM=31` means all five 3.7x profiles are valid. `CVM=1023` means all ten
dual-gain profiles are valid. Match `UI`, `UX`, `UG`, `DX`, and `DG` to the
selected profiles before checking their bits.

### Validating a reading

Treat the payload as a usable measurement only when:

- `SEQ` is greater than zero; the initial `NDVI=0.00` is a placeholder.
- `SEQ` differs from the last record your app stored.
- `UP=1` and `DN=1`.
- `LOW=0`.
- The selected UP and DOWN profile bits are present in `CVM`.
- The relevant `SAT` bits are clear.

Keep the raw status record alongside the NDVI value so later analysis can see
the light profile and quality flags.

### Parser example

The protocol is intentionally order-independent. Ignore unknown keys so newer
firmware can add fields without breaking an app.

```javascript
function parseMantiStatus(bytes) {
  const text = new TextDecoder().decode(bytes);
  const fields = Object.fromEntries(
    text.split(",").map((item) => {
      const separator = item.indexOf("=");
      if (separator < 1) throw new Error(`Invalid field: ${item}`);
      return [item.slice(0, separator), item.slice(separator + 1)];
    })
  );

  const required = [
    "NDVI", "RUI", "RUV", "RDI", "RDV", "SEQ", "MS", "BAT", "UP", "DN", "LOW",
    "UI", "UX", "UG", "DX", "DG", "SAT", "CVM"
  ];
  if (!required.every((key) => key in fields)) {
    throw new Error("Incomplete Manti-S status payload");
  }

  return {
    ndvi: Number(fields.NDVI),
    rawUpRed: Number(fields.RUI),
    rawUpNir: Number(fields.RUV),
    rawDownRed: Number(fields.RDI),
    rawDownNir: Number(fields.RDV),
    sequence: Number(fields.SEQ),
    measuredAtUptimeMs: Number(fields.MS),
    batteryPercent: Number(fields.BAT),
    upReady: fields.UP === "1",
    downReady: fields.DN === "1",
    lowLight: fields.LOW === "1",
    upIntegrationCycles: Number(fields.UI),
    upExposures: Number(fields.UX),
    upGain: Number(fields.UG),
    downExposures: Number(fields.DX),
    downGain: Number(fields.DG),
    saturationMask: Number(fields.SAT),
    calibrationValidMask: Number(fields.CVM),
    raw: text
  };
}
```

### Connection flow

1. Scan for the service UUID.
2. Connect and discover the service and status characteristic.
3. Read the characteristic once for initial state.
4. Subscribe to notifications if supported by the client stack.
5. Parse only complete records containing all required fields.
6. Use `SEQ` to de-duplicate updates.
7. Reconnect and repeat discovery after a disconnect.

### Notification compatibility

The complete status record is longer than the 20-byte value available with the
default 23-byte ATT MTU. Firmware configures a 247-byte local MTU and provides
a Client Characteristic Configuration Descriptor (CCCD). The client must still
negotiate a sufficiently large MTU before subscribing to notifications.

For compatibility with the current firmware:

- Use a characteristic read as the authoritative way to obtain the full record.
- If notifications work, verify that every required key is present; otherwise
  discard the notification and perform a read.
- Polling `SEQ` with periodic reads is the safest cross-platform approach.

An application must not silently accept a truncated notification. Negotiate an
ATT MTU of at least 200 bytes before treating full-record notifications as
portable.

## Protocol limitations

- There is no BLE command to start a measurement.
- There is no timestamp or real-time clock; `MS` is uptime, not wall time.
- There is no device identifier in the payload.
- There is no protocol-version field.
- There is no authentication, encryption requirement, or bonding.
- The four dark-corrected NDVI-band inputs are available through BLE. All 18
  raw spectral channels can be exported with serial `capture <profile>` for
  diffuser analysis outside the product.

Apps should timestamp records on receipt, associate the connection with their
own device identifier, ignore unknown fields, and retain the full original
payload.
