"""Build the ESP32 Arduino 2.x flash-resident TLS root bundle.

Format: ESP-IDF 4.4 esp_crt_bundle (count, then sorted subject/key pairs).
See https://github.com/espressif/esp-idf/blob/release/v4.4/components/mbedtls/esp_crt_bundle/gen_crt_bundle.py
"""

import argparse
import re
import struct
from pathlib import Path

from cryptography import x509
from cryptography.hazmat.primitives import serialization


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pem", type=Path, help="PEM CA root store")
    parser.add_argument("output", type=Path, help="output bundle binary")
    args = parser.parse_args()

    source = args.pem.read_bytes()
    blocks = re.findall(
        rb"-----BEGIN CERTIFICATE-----.*?-----END CERTIFICATE-----", source, re.DOTALL
    )
    if not blocks:
        raise SystemExit("No PEM certificates found")
    roots = [x509.load_pem_x509_certificate(block) for block in blocks]
    names = [root.subject.rfc4514_string() for root in roots]
    for required in (
        "USERTrust ECC Certification Authority",
        "Sectigo Public Server Authentication Root E46",
    ):
        if not any(required in name for name in names):
            raise SystemExit(f"Required GitHub root missing: {required}")

    entries = []
    for root in roots:
        subject = root.subject.public_bytes()
        key = root.public_key().public_bytes(
            serialization.Encoding.DER, serialization.PublicFormat.SubjectPublicKeyInfo
        )
        entries.append((subject, key))
    entries.sort(key=lambda entry: entry[0])
    if len({subject for subject, _ in entries}) != len(entries):
        raise SystemExit("Duplicate root subjects in input")
    payload = bytearray(struct.pack(">H", len(entries)))
    for subject, key in entries:
        payload.extend(struct.pack(">HH", len(subject), len(key)))
        payload.extend(subject)
        payload.extend(key)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(payload)
    print(f"Wrote {len(entries)} roots, {len(payload)} bytes to {args.output}")


if __name__ == "__main__":
    main()
