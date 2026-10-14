Import("env")

import json
import os
import re
import subprocess
from pathlib import Path

from SCons.Script import COMMAND_LINE_TARGETS
from serial.tools import list_ports


PROJECT_DIR = Path(env.subst("$PROJECT_DIR")).resolve()
TOOLS_DIR = PROJECT_DIR / "tools"
if not (TOOLS_DIR / "device_registry.json").exists():
    TOOLS_DIR = PROJECT_DIR.parent / "open-ndvi-handheld" / "firmware" / "tools"
REGISTRY_PATH = TOOLS_DIR / "device_registry.json"
MAC_PATTERN = re.compile(r"MAC:\s*([0-9a-fA-F:]{17})")


def select_upload_port():
    configured = env.subst("$UPLOAD_PORT").strip()
    if configured and "$" not in configured and "*" not in configured:
        return configured

    candidates = [
        port.device
        for port in list_ports.comports()
        if "ACM" in port.device or "USB" in port.device
    ]
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise RuntimeError("No Manti-S serial port found for device-ID assignment")
    raise RuntimeError(
        "Multiple serial devices found; select the target with --upload-port: "
        + ", ".join(candidates)
    )


def read_factory_mac(port):
    platform = env.PioPlatform()
    esptool_path = Path(platform.get_package_dir("tool-esptoolpy")) / "esptool.py"
    python_executable = env.subst("$PYTHONEXE")
    result = subprocess.run(
        [python_executable, str(esptool_path), "--port", port, "read_mac"],
        check=False,
        capture_output=True,
        text=True,
    )
    output = result.stdout + result.stderr
    match = MAC_PATTERN.search(output)
    if result.returncode != 0 or match is None:
        raise RuntimeError(f"Could not read ESP32 factory MAC on {port}:\n{output}")
    return match.group(1).lower()


def load_registry():
    if not REGISTRY_PATH.exists():
        return {"next_device_number": 1, "devices": {}}
    with REGISTRY_PATH.open("r", encoding="utf-8") as registry_file:
        registry = json.load(registry_file)
    if not isinstance(registry.get("devices"), dict):
        raise RuntimeError(f"Invalid device registry: {REGISTRY_PATH}")
    return registry


def save_registry(registry):
    temporary_path = REGISTRY_PATH.with_suffix(".json.tmp")
    with temporary_path.open("w", encoding="utf-8") as registry_file:
        json.dump(registry, registry_file, indent=2, sort_keys=True)
        registry_file.write("\n")
    os.replace(temporary_path, REGISTRY_PATH)


def assign_device_number(mac_address):
    registry = load_registry()
    devices = registry["devices"]
    if mac_address in devices:
        return int(devices[mac_address]["device_number"]), False

    device_number = int(registry.get("next_device_number", 1))
    if device_number < 1 or device_number > 99999:
        raise RuntimeError("Manti-S device-number range 00001..99999 is exhausted")

    devices[mac_address] = {
        "device_name": f"Manti-S-{device_number:05d}",
        "device_number": device_number,
    }
    registry["next_device_number"] = device_number + 1
    save_registry(registry)
    return device_number, True


if "upload" in COMMAND_LINE_TARGETS:
    upload_port = select_upload_port()
    factory_mac = read_factory_mac(upload_port)
    device_number, newly_assigned = assign_device_number(factory_mac)
    action = "assigned" if newly_assigned else "reusing"
    print(
        f"Manti-S device ID: {action} Manti-S-{device_number:05d} "
        f"for {factory_mac} on {upload_port}"
    )
    # The first USB boot stores this number in NVS. Release builds remain generic.
    env.Append(CPPDEFINES=[("MANTI_DEVICE_NUMBER", device_number)])
