"""Compress web_ui_src.html into PROGMEM gzip byte array web_ui.h for ESP32-C3."""

import gzip
from pathlib import Path

DIR = Path(__file__).resolve().parent
SRC = DIR / "web_ui_src.html"
DST = DIR / "web_ui.h"

raw = SRC.read_bytes()
gz = gzip.compress(raw, compresslevel=9)

lines = [
    "#pragma once",
    "#include <Arduino.h>",
    f"// Generated from web_ui_src.html ({len(raw):,} bytes -> {len(gz):,} bytes gzip)",
    f"const size_t P25_WEB_UI_GZ_LEN = {len(gz)};",
    "const uint8_t P25_WEB_UI_GZ[] PROGMEM = {",
]

for i in range(0, len(gz), 20):
    chunk = ", ".join(f"0x{b:02X}" for b in gz[i : i + 20])
    lines.append(f"  {chunk},")

lines.append("};")
DST.write_text("\n".join(lines) + "\n", encoding="utf-8")
print(f"[PACK] {SRC.name}: {len(raw):,} B -> {len(gz):,} B gzip ({len(gz)*100.0/len(raw):.1f}%) -> {DST.name}")
