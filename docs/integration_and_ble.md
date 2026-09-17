# Integration time and BLE API

This document describes the active PlatformIO firmware in
`firmware/src/main.cpp`. The older Arduino sketch in
`firmware/ndvi_handheld/` does not implement this protocol.

## Low-light measurement profiles

Both AS7265x sensors run at 3.7x gain. The firmware adapts the upward-facing
(`UP`) sensor through three profiles and uses two profiles for the
downward-facing (`DOWN`) sensor.

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

1. Capture `up_100x1`.
2. If its dark-corrected 810 nm signal is below 75 raw counts, discard it and
   capture `up_255x1`.
3. If that result is still below 75 counts, discard it and capture
   `up_255x3`.
4. Use only the final selected profile for calibration and NDVI.

Level 3 is three separate 255-cycle exposures added together. It is a sum, not
an average. Each profile therefore has its own calibration and dark-offset
data.

The DOWN sensor always starts with `down_255x1`. If either its dark-corrected
645 nm or 810 nm channel is below 75 counts, that reading is discarded and
`down_255x3` is captured. DOWN escalation is based on the DOWN signal, not the
UP result.

| DOWN profile | Cycles | Exposures | Approx. integration |
|---|---:|---:|---:|
| `down_255x1` | 255 | 1 | 1.434 s |
| `down_255x3` | 255 | 3, summed | 4.301 s |

Retries are cumulative. A measurement that reaches the longest UP and DOWN
profiles takes about 12 seconds of sensor conversion time because the shorter
attempts are captured first and discarded.

`LOW=1` means that at least one sensor remains below its threshold after its
final profile. The firmware still calculates NDVI, but an app should label or
reject that result as low confidence.

## BLE GATT interface

| Item | Value |
|---|---|
| Advertising name | `Manti-S-00001` |
| Service UUID | `9f4a6d90-f740-4d10-a947-8c6c20f4f9d1` |
| Status characteristic UUID | `2d86686f-34f8-4f1b-a4ac-f127f90f7df0` |
| Characteristic properties | Read, Notify |
| Encoding | UTF-8/ASCII text |
| Authentication | None |
| Write or remote-measure command | None |

Discover by service UUID rather than device name. The compiled name is not
guaranteed to be unique if several devices use the default firmware.

Measurements are started with the physical button or `m` over the 115200-baud
USB serial connection. BLE only exposes the latest status. The device resumes
advertising after a disconnect.

### Payload format

The characteristic contains one comma-separated record with `KEY=VALUE`
fields and no newline:

```text
NDVI=0.67,SEQ=42,MS=183921,BAT=84,UP=1,DN=1,LOW=0,UI=255,UX=1,DX=1,SAT=0,CVM=31
```

| Field | Type | Meaning |
|---|---|---|
| `NDVI` | decimal | Latest NDVI, formatted to two decimal places and clamped to -1.00...1.00 |
| `SEQ` | unsigned integer | Measurement counter; increments after each completed measurement |
| `MS` | unsigned integer | Device uptime in milliseconds when that measurement completed |
| `BAT` | integer | Estimated battery charge, 0...100 percent |
| `UP` | `0` or `1` | UP sensor is available |
| `DN` | `0` or `1` | DOWN sensor is available |
| `LOW` | `0` or `1` | Final UP or DOWN signal is still below the low-light threshold |
| `UI` | integer | Integration cycles used by the selected UP profile: 100 or 255 |
| `UX` | integer | Exposures summed in the selected UP profile: 1 or 3 |
| `DX` | integer | Exposures summed in the selected DOWN profile: 1 or 3; DOWN always uses 255 cycles |
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

For example, `CVM=31` means all five profiles are marked valid. Match `UI`,
`UX`, and `DX` to the selected profiles before checking their bits.

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
    "NDVI", "SEQ", "MS", "BAT", "UP", "DN", "LOW",
    "UI", "UX", "DX", "SAT", "CVM"
  ];
  if (!required.every((key) => key in fields)) {
    throw new Error("Incomplete Manti-S status payload");
  }

  return {
    ndvi: Number(fields.NDVI),
    sequence: Number(fields.SEQ),
    measuredAtUptimeMs: Number(fields.MS),
    batteryPercent: Number(fields.BAT),
    upReady: fields.UP === "1",
    downReady: fields.DN === "1",
    lowLight: fields.LOW === "1",
    upIntegrationCycles: Number(fields.UI),
    upExposures: Number(fields.UX),
    downExposures: Number(fields.DX),
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
default 23-byte ATT MTU. The current firmware does not explicitly request a
larger local MTU or add a Client Characteristic Configuration Descriptor
(CCCD). Consequently, notification behaviour can vary between BLE clients.

For compatibility with the current firmware:

- Use a characteristic read as the authoritative way to obtain the full record.
- If notifications work, verify that every required key is present; otherwise
  discard the notification and perform a read.
- Polling `SEQ` with periodic reads is the safest cross-platform approach.

An application must not silently accept a truncated notification. A future
firmware protocol should add a CCCD and negotiate an ATT MTU of at least 128
bytes before full-record notifications are treated as portable.

## Protocol limitations

- There is no BLE command to start a measurement.
- There is no timestamp or real-time clock; `MS` is uptime, not wall time.
- There is no device identifier in the payload.
- There is no protocol-version field.
- There is no authentication, encryption requirement, or bonding.
- Raw spectral channels and calibration tables are available over USB serial,
  not through this BLE characteristic.

Apps should timestamp records on receipt, associate the connection with their
own device identifier, ignore unknown fields, and retain the full original
payload.
