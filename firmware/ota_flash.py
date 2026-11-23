"""Wireless HTTP Web OTA uploader for p25_haier_c3 (works on LAN 192.168.1.25 or AP 192.168.4.1)."""

import sys
import urllib.request
import uuid
from pathlib import Path

BIN_PATH = (
    Path(__file__).resolve().parent
    / "p25_haier_c3"
    / "build"
    / "esp32.esp32.esp32c3"
    / "p25_haier_c3.ino.bin"
)


def main() -> None:
    ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.25"
    if not BIN_PATH.exists():
        print(f"[ERROR] Binary not found: {BIN_PATH}\nRun compile_and_flash.bat first.")
        sys.exit(1)

    data = BIN_PATH.read_bytes()
    boundary = f"----P25OTA{uuid.uuid4().hex}"
    body = (
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="firmware"; filename="{BIN_PATH.name}"\r\n'
        f"Content-Type: application/octet-stream\r\n\r\n"
    ).encode("utf-8") + data + f"\r\n--{boundary}--\r\n".encode("utf-8")

    url = f"http://{ip}/update"
    print(f"[OTA] Uploading {BIN_PATH.name} ({len(data):,} bytes) -> {url} ...")
    req = urllib.request.Request(
        url,
        data=body,
        method="POST",
        headers={
            "Content-Type": f"multipart/form-data; boundary={boundary}",
            "Origin": f"http://{ip}",
            "Referer": url,
        },
    )
    with urllib.request.urlopen(req, timeout=60) as resp:
        print(f"[OK] HTTP {resp.status}: {resp.read().decode('utf-8', 'ignore').strip()}")


if __name__ == "__main__":
    main()
