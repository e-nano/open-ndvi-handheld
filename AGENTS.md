# Agent instructions

- Keep responses short. Prefer bullets.
- The active firmware is `firmware/src/` (PlatformIO), not the older `.ino` sketch.

## When the user says "deploy" or "release the firmware"

This authorizes the agent to commit and push the intended firmware changes to
`https://github.com/e-nano/open-ndvi-handheld`, push a new stable `vX.Y.Z` tag,
and verify the GitHub Release. Do not ask for those steps again. The phrase does
not authorize flashing a connected device or installing an OTA update: device
installation is started manually in its portal.

1. Inspect `git status`, the diff, `origin`, existing tags, and GitHub Releases.
   Identify the intended changes. Never use `git add .` on a dirty tree; exclude
   backups, testbeds, generated files, and unrelated user work.
2. Check that the repository and release assets are publicly downloadable.
   The firmware fetches these exact URLs without GitHub authentication:
   `https://github.com/e-nano/open-ndvi-handheld/releases/latest/download/manifest.txt`
   and the matching `firmware.bin`. If the repository is private, resolve the
   distribution design before publishing; a private release cannot serve this
   firmware's OTA client.
3. Review calibration and device behavior changes against the last deployed
   release: `firmware/src/as7265x_calibration_data.h`, calibration schema and
   loader, measurement code, sensor library, device identity, and partition
   offsets. Preserve the `caldata` partition at `0x400000` (64 KB). Explain any
   intentional change; a successful build alone does not prove calibration or
   measurement equivalence. If those areas changed, validate on one device and
   compare its calibration backup and post-update status before fleet use.
4. Run `pio run` from `firmware/`, check `git diff --check`, and run relevant
   checks for the changed code. Compare the live TLS chains for the GitHub
   manifest and redirected release assets with `firmware/src/github_roots.h`;
   a missing root makes the portal report a manifest request failure. Do not
   use full-chip erase during deployment.
5. Commit only the reviewed files. Push the commit to `origin/main` (or merge
   the reviewed branch into `main` and push). The release workflow must exist
   in the tagged commit. Never force-push or move an existing release tag.
6. Choose the next unused stable version tag (`vX.Y.Z`; for the first release
   of the current OTA work, `v0.3.0` if still unused). Create the tag on the
   pushed commit and push it. Tag pushes run
   `.github/workflows/release_firmware.yml`, which builds the firmware with the
   tag as its version and publishes `firmware.bin` plus `manifest.txt`.
7. Wait for the GitHub Action to finish. Verify the Release is public, both
   assets exist, and `manifest.txt` version, size, and SHA-256 match the
   downloadable `firmware.bin`. Do not report success until these checks pass.
8. Report the commit, tag, release link, tests, and any calibration or behavior
   risk. Explain that users install it by opening the device Wi-Fi portal,
   pressing **Check for update**, then **Install update**. Test one device before
   wider rollout. Never trigger installation just because a release exists.

If credentials, network access, or a required safety check block a step, state
the exact blocker and leave the release unpublished or clearly marked incomplete.

## USB flash for a device without OTA

A request to **flash this device** authorizes a USB flash of the identified unit
after the checks below. A request for instructions alone does not. Do the
read-only checks and backup before asking the user to decide a specific
calibration or hardware incompatibility. Never treat "deploy" as permission to
flash a device.

1. Ask only for facts you cannot discover: which physical unit should be
   updated, whether its USB data cable is connected to the agent's computer or
   the user's computer, the user's operating system if remote, and whether
   they want the latest stable release or a specified local build. If several
   ports or units exist, ask which one.
   If the agent cannot access USB, give OS-specific steps and request the
   command output; do not claim to have inspected or flashed the device.
2. Check that the intended source contains `firmware/platformio.ini`,
   `firmware/src/portal_ota.cpp`, and `firmware/partitions_manti.csv`. A clean
   older checkout may not contain OTA. Confirm the target source, commit or
   release before building; a standalone `firmware.bin` is insufficient for
   the first USB migration because the partition table must also be installed.
3. Prepare the host if needed. Check for Git, Python 3, PlatformIO Core, and
   serial-port access. If the checkout is missing, obtain the selected source
   from `https://github.com/e-nano/open-ndvi-handheld` and verify it contains
   the OTA files above. If PlatformIO is absent, create a Python virtual
   environment, activate it using the user's OS instructions, and run
   `python -m pip install platformio==6.1.19`; then check `pio --version`.
   `platformio.ini` pins the ESP32 platform to 6.13.0. Build with `pio run`
   from `firmware/`. For a stable release build, use its tagged source and
   generate `firmware/src/release_version.h` with the selected tag and date,
   as `.github/workflows/release_firmware.yml` does; check that the built
   version is the one requested. On Linux, check serial permissions; on
   Windows, identify the COM port and driver. If no port appears, check the
   data cable, power, and USB driver before proceeding.
4. Identify the board read-only: list serial ports, read its factory MAC with
   PlatformIO's esptool, check flash capacity (`flash_id`), and match the MAC
   and physical label to `firmware/tools/device_registry.json`. This project
   targets the 8 MB Adafruit Feather ESP32 V2. Ask for the existing device
   number if its MAC is absent from the registry; add the correct mapping
   before upload so provisioning does not assign a new identity silently.
5. Capture the baseline. If supported, send `status` at 115200 baud and record
   firmware version, calibration source/schema/valid mask, and sensor status.
   Read the old partition table at `0x8000` (4 KB). Save and SHA-256 hash the
   `caldata` partition at `0x400000` (64 KB) when that partition exists. If the
   layout or calibration location is unknown, read back the entire flash first
   (8 MB only after confirming 8 MB capacity). Store backups outside Git.
   Example esptool commands, replacing `<port>` and `<esptool.py>` with the
   detected values:

   ```text
   python <esptool.py> --port <port> read_mac
   python <esptool.py> --port <port> flash_id
   python <esptool.py> --port <port> read_flash 0x8000 0x1000 partition_before.bin
   python <esptool.py> --port <port> read_flash 0x400000 0x10000 caldata_before.bin
   python <esptool.py> --port <port> read_flash 0 0x800000 full_before.bin
   ```

6. Check compatibility before writing. The current loader requires calibration
   schema 5; schema 2/3/4 bytes may survive but will **not** load. A missing
   `status` response does not prove calibration is absent. Check old source or
   calibration records if needed. If schema, location, source, flash size, or
   partition offsets differ, explain the actual consequence and ask the user
   whether to migrate/recalibrate before flashing. Do not promise unchanged
   measurement behavior without a comparison. Do not write to `caldata` or use
   `erase_flash` as part of routine migration.
7. For a compatible board, confirm the new partition table retains `nvs`,
   `app0`, `spiffs`, `coredump`, and `caldata` at their old offsets and adds
   `app1` at `0x500000`. From `firmware/`, use
   `pio run --target upload --upload-port <port>`; this uploads the bootloader,
   partition table and app, and provisions the device number. Select the port
   explicitly. Do not upload the old `.ino` sketch or only a generic OTA binary.
8. After boot, read `status` again. Confirm the intended version and device
   name, `calibration_source=flash`, `calibration_schema=5`, expected valid
   mask, and both sensors ready. Read back `caldata` and compare its SHA-256 to
   the backup. Run one measurement, then test the 5-second button hold, setup
   page, and next-press exit. OTA remains a manual portal action. If checks
   fail, preserve backups and report the exact difference before further
   writes or restoration.
