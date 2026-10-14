# Firmware - Open NDVI Handheld

- This folder now contains the ESP32 firmware structure copied from `Manti-S_CalibrationCode`.

## Layout

- `platformio.ini`
  - PlatformIO project config for `adafruit_feather_esp32_v2`
- `src/main.cpp`
  - Main ESP32 application
- `src/as7265x_calibration_data.h`
  - Embedded calibration tables
- `src/manti_features.h`
  - Reversible runtime feature switches
- `src/portal_ota.cpp`
  - Device Wi-Fi portal, network setup, and manual OTA update
- `src/github_roots.h`
  - TLS trust roots used for GitHub Releases
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
  - Calibration storage via `Preferences` in a dedicated 64 KB `caldata` NVS partition

## Wi-Fi portal and manual updates

- Normal startup shows the measurement screen with Wi-Fi off, even without
  stored credentials. A short measure-button press starts a measurement when
  released.
- Hold the measure button for 5 seconds during normal operation to open setup.
  Release it, then press it again to close setup and return to the measurement
  screen. The portal and Wi-Fi turn off when setup closes.
- Only in setup mode, the screen shows the device Wi-Fi name, a fresh 8-character WPA2 password, and
  `http://192.168.4.1/`. Connect a phone to that network and open the address
  if the captive-portal popup does not appear.
- The portal scans nearby networks. Credentials are saved in the default NVS
  partition only after a successful connection. The device keeps its own Wi-Fi
  network active during setup, so the portal remains available. Press **Finish
  setup** in the portal or press the device button to close it. Serial
  measurements remain available while setup is open.
- Press **Check for update**, then **Install update**. Checking never installs.
  Installation streams the latest public GitHub Release `firmware.bin` into the
  inactive app slot and verifies its size and SHA-256 against `manifest.txt`
  before selecting it for boot. Use USB power during an update.
- GitHub must be reachable and its TLS certificate must chain to a root in
  `src/github_roots.h`. The portal is only served over the device's password
  protected Wi-Fi network; it is not served on the joined LAN.
- The bootloader keeps the previous app slot. The new app confirms its first
  boot after display, BLE, calibration, and sensor initialisation completes.

### Initial USB migration

Existing devices need one USB flash of this firmware and partition table before
OTA is available. `app0`, `spiffs`, `coredump`, and `caldata` retain their old
offsets. The new `app1` occupies `0x500000..0x7fffff` on the 8 MB Feather V2.
Back up each device's calibration before the USB flash and do not use a full
flash erase. The first boot stores the USB-provisioned device number in default
NVS; generic release images then preserve that number. A new board flashed
directly with a generic image uses its factory MAC in the BLE and Wi-Fi name.
Before flashing, send `status` over USB serial and confirm
`calibration_schema=5`, `calibration_source=flash`, and the expected
`calibration_valid_mask`. This firmware ignores older calibration schemas;
their flash bytes may remain present but their factors will not be loaded.

### Publishing an update

- Merge and review the firmware change first.
- Push a version tag such as `v0.3.0`. The `Release firmware` workflow builds
  the active PlatformIO project, embeds the tag as its version, and publishes
  `firmware.bin` plus `manifest.txt` in a public GitHub Release.
- The portal points at the latest published release. Test a release on one
  device before using it across the fleet.

## Integration timing

- Each integration level starts at 3.7x and, if low, retries at 16x before
  moving to the next level.
- UP uses 100x1, 255x1, then 255x3 profiles as required.
- DOWN uses 255x1, then 255x3 profiles as required.
- In normal light, DOWN captures one exposure.
- If DOWN red or NIR remains low, DOWN captures and sums three new exposures.
- BLE reports integration/exposures as `UI`/`UX`/`DX` and selected gains as
  `UG`/`DG`.
- Serial reports the summed raw channels and accumulation count for both sensors.
- Low-light decisions monitor direct raw counts before calibration:
  - UP: channel I or V below 100.
  - DOWN: channel I or V below 100.
- See [Integration Time and BLE API](../docs/integration_and_ble.md) for timing,
  thresholds, UUIDs, payload fields, and app integration guidance.

## Calibration profiles

- Calibration schema 5 stores separate 3.7x and 16x tables and dark offsets.
- Runtime keeps full 16x counts and applies the matching 16x table.
- The five 3.7x profiles are:
  - `up_100x1`
  - `up_255x1`
  - `up_255x3`
  - `down_255x1`
  - `down_255x3`
- Five matching 16x profiles append `_g16` to those IDs.
- Factors interpolate only inside measured ranges and clamp at the endpoints;
  runtime does not extrapolate calibration curves.
- UP uses its selected table while both I and V are in range. If either is out
  of range, both measured channels are corrected through a matching-gain DOWN
  table after removing the characterised diffuser attenuation (`I=0.0470`,
  `V=0.0564`) and normalising integration and exposure count.
- Serial status reports `up_calibration_mode` and `up_transfer_profile`.
- `CVM=31` means all five 3.7x profiles are valid; `CVM=1023` means all ten
  dual-gain profiles are valid.
- Schema-2, schema-3 and schema-4 flash tables are deliberately ignored.
  All profiles remain invalid until fresh schema-5 tables are calibrated and saved.

## Reversing or disabling 16x

Set this feature flag in `src/manti_features.h`, then rebuild:

```cpp
#define MANTI_ENABLE_16X_CALIBRATION 0
```

This disables automatic 16x measurement attempts. It does not delete 16x
tables, and forced `_g16` captures remain available to the calibration tool.

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

During upload, PlatformIO reads the ESP32 factory MAC and uses
`tools/device_registry.json` to assign its BLE name:

- A new board receives the next sequential name (`Manti-S-00002`, etc.).
- Reflashing a known board reuses its existing name.
- If several serial devices are connected, select one with
  `pio run --target upload --upload-port /dev/ttyACM0`.
- Keep `tools/device_registry.json` under version control and backed up; it is
  the source of truth for allocated device numbers.
