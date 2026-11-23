"""Haier hOn (E++2.18) UART Protocol Codec for HSU-15VQRA03T / TF / TC.

Pure Python protocol module (zero hardware/serial dependencies) covering:
- CRC-16/ARC (Big-Endian) and 8-bit checksum
- Byte stuffing / unstuffing (0xFF -> 0xFF 0x55 after header)
- Frame parser with strict CRC+Checksum-verified single-0xFF recovery
- Decoders for Device Version (0x62), User Data (6D 01), BigData (7D 01 / 6D FE),
  and Hardware Alarms (0x74)
- 12-byte Control Block encoder (60 01)
"""

from typing import Any, Dict, List, Optional

PROTOCOL_TABLES: Dict[str, Any] = {
    "meta": {
        "model": "Haier HSU-15VQRA03T (Indoor: TF / Outdoor: TC, 14,700 BTU)",
        "protocol": "hOn (E++2.18)",
        "baud": 9600,
        "framing": "8N1",
        "flag": "0x40 (CRC-16/ARC BE + 0xFF->0xFF55 byte-stuffing)",
    },
    "modes": {
        "AUTO": 0x00,
        "COOL": 0x01,
        "DRY": 0x02,
        "HEALTHY_DRY": 0x03,
        "HEAT": 0x04,
        "ENERGY_SAVING": 0x05,
        "FAN": 0x06,
    },
    "fan_modes": {
        "HIGH": 0x01,
        "MID": 0x02,
        "LOW": 0x03,
        "AUTO": 0x05,
    },
    "vertical_swing": {
        "DEFAULT": 0x00,
        "HEALTH_UP": 0x01,
        "MAX_UP": 0x02,
        "HEALTH_DOWN": 0x03,
        "UP": 0x04,
        "CENTER": 0x06,
        "DOWN": 0x08,
        "MAX_DOWN": 0x0A,
        "AUTO": 0x0C,
        "AUTO_SPECIAL": 0x0E,
    },
    "horizontal_swing": {
        "CENTER": 0x00,
        "MAX_LEFT": 0x03,
        "LEFT": 0x04,
        "RIGHT": 0x05,
        "MAX_RIGHT": 0x06,
        "AUTO": 0x07,
    },
    "eco_levels_b7_bits3_5": {
        "OFF": 0x00,
        "L1": 0x28,
        "L2": 0x30,
        "L3": 0x38,
    },
    "subcommands": {
        "GET_USER_DATA": "4D01",
        "GET_BIG_DATA": "4DFE",
        "POWER_ON_SINGLE": "5D010001",
        "POWER_OFF_SINGLE": "5D010000",
        "SELF_CLEAN_ON_SINGLE": "5D0D0001",
        "SELF_CLEAN_OFF_SINGLE": "5D0D0000",
        "SET_GROUP_PARAMETERS": "6001",
    },
}

MODE_NAMES = {v: k for k, v in PROTOCOL_TABLES["modes"].items()}
FAN_NAMES = {v: k for k, v in PROTOCOL_TABLES["fan_modes"].items()}
VSWING_NAMES = {v: k for k, v in PROTOCOL_TABLES["vertical_swing"].items()}
HSWING_NAMES = {v: k for k, v in PROTOCOL_TABLES["horizontal_swing"].items()}
ECO_B7_TO_LVL = {0x00: 0, 0x28: 1, 0x30: 2, 0x38: 3}
ECO_LVL_TO_B7 = {0: 0x00, 1: 0x28, 2: 0x30, 3: 0x38}

HON_ALARM_MESSAGES = [
    "Outdoor module failure",
    "Outdoor defrost sensor failure",
    "Outdoor compressor exhaust sensor failure",
    "Outdoor EEPROM abnormality",
    "Indoor coil sensor failure (E2)",
    "Indoor-outdoor communication failure (E7)",
    "Power supply overvoltage protection",
    "Communication failure panel-indoor",
    "Outdoor compressor overheat protection",
    "Outdoor environmental sensor abnormality",
    "Full water protection",
    "Indoor EEPROM failure",
    "Outdoor out air sensor failure",
    "CBD and module communication failure",
    "Indoor DC fan failure (E14)",
    "Outdoor DC fan failure",
    "Door switch failure",
    "Dust filter needs cleaning reminder",
    "Water shortage protection",
    "Humidity sensor failure",
    "Indoor temperature sensor failure (E1)",
    "Manipulator limit failure",
    "Indoor PM2.5 sensor failure",
    "Outdoor PM2.5 sensor failure",
    "Indoor heating overload/high load alarm",
    "Outdoor AC current protection",
    "Outdoor compressor operation abnormality (F3)",
    "Outdoor DC current protection",
    "Outdoor no-load failure",
    "CT current abnormality",
    "Indoor cooling freeze protection",
    "High and low pressure protection",
    "Compressor out air temperature too high",
    "Outdoor evaporator sensor failure (F1)",
    "Outdoor cooling overload",
    "Water pump drainage failure",
    "Three-phase power supply failure",
    "Four-way valve failure",
    "External alarm/scraper flow switch failure",
    "Temperature cutoff protection alarm",
    "Different mode operation failure",
    "Electronic expansion valve failure",
    "Dual heat source sensor Tw failure",
    "Communication failure with wired controller",
    "Indoor unit address duplication failure",
    "50Hz zero crossing failure",
    "Outdoor unit failure",
    "Formaldehyde sensor failure",
    "VOC sensor failure",
    "CO2 sensor failure",
    "Firewall failure",
]

DEFAULT_CONTROL_RAW = b"\x08\x06\x21\x00\x02\x01\x00\x00\x08\x00\x00\x00"


def crc16_arc(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc


def stuff_bytes(raw: bytes) -> bytes:
    """Insert 0x55 after any 0xFF byte per HaierFrame::fill_buffer."""
    out = bytearray()
    for b in raw:
        out.append(b)
        if b == 0xFF:
            out.append(0x55)
    return bytes(out)


def build_frame(frame_type: int, payload: bytes, flag: int = 0x40) -> bytes:
    length = 8 + len(payload)
    body = bytes([length, flag, 0, 0, 0, 0, 0, frame_type]) + payload
    chk = sum(body) & 0xFF
    tail = bytes([chk])
    if flag & 0x40:
        c = crc16_arc(body)
        tail += bytes([(c >> 8) & 0xFF, c & 0xFF])
    return b"\xFF\xFF" + stuff_bytes(body + tail)


def _try_extract_at(raw: bytes, start_body_idx: int, header_start_idx: int, recovered: bool) -> Optional[tuple[Dict[str, Any], int]]:
    """Attempt to unstuff and validate a Haier frame whose body starts at start_body_idx."""
    n = len(raw)
    j = start_body_idx
    logical = bytearray()
    while j < n and len(logical) < 3:
        b = raw[j]
        logical.append(b)
        j += 2 if (b == 0xFF and j + 1 < n and raw[j + 1] == 0x55) else 1
    if len(logical) < 3:
        return None
    length = logical[0]
    flag = logical[1]
    if length < 8 or length > 120:
        return None
    if recovered and not (flag & 0x40):
        # For single-FF recovery we strictly require 16-bit CRC + 8-bit checksum
        return None
    need_total = length + (3 if (flag & 0x40) else 1)
    while j < n and len(logical) < need_total:
        b = raw[j]
        logical.append(b)
        j += 2 if (b == 0xFF and j + 1 < n and raw[j + 1] == 0x55) else 1
    if len(logical) < need_total:
        return None
    body = bytes(logical[:length])
    chk_ok = (sum(body) & 0xFF) == logical[length]
    crc_ok = True
    if flag & 0x40:
        exp_crc = (logical[length + 1] << 8) | logical[length + 2]
        crc_ok = crc16_arc(body) == exp_crc
    if recovered and not (chk_ok and crc_ok):
        return None
    frame = {
        "hex": raw[header_start_idx:j].hex(),
        "type": f"0x{body[7]:02X}",
        "flag": f"0x{flag:02X}",
        "data": body[8:length],
        "chk_ok": chk_ok,
        "crc_ok": crc_ok,
        "recovered_single_ff": recovered,
    }
    return frame, j


def parse_frames(raw: bytes, allow_single_ff_recovery: bool = True) -> List[Dict[str, Any]]:
    """Parse Haier hOn frames from raw UART bytes.

    Supports both standard 0xFF 0xFF headers and strict CRC+CHK-verified
    single-0xFF recovery (fixes USB-TTL leading-0xFF byte drops).
    """
    frames: List[Dict[str, Any]] = []
    i = 0
    n = len(raw)
    while i + 9 <= n:
        # 1. Standard 0xFF 0xFF header
        if raw[i] == 0xFF and i + 1 < n and raw[i + 1] == 0xFF and (i + 2 >= n or raw[i + 2] != 0x55):
            extracted = _try_extract_at(raw, i + 2, i, recovered=False)
            if extracted is not None:
                fr, next_i = extracted
                frames.append(fr)
                i = next_i
                continue
            # Might be incomplete trailing frame; if length byte is valid and we ran out of bytes, stop
            if i + 3 < n and 8 <= raw[i + 2] <= 120:
                break
        # 2. Strict single-0xFF recovery (only when 24-bit CHK + CRC-16/ARC both pass)
        elif (
            allow_single_ff_recovery
            and raw[i] == 0xFF
            and i + 2 < n
            and raw[i + 1] != 0xFF
            and raw[i + 1] != 0x55
            and 8 <= raw[i + 1] <= 120
            and (raw[i + 2] & 0x40) != 0
        ):
            extracted = _try_extract_at(raw, i + 1, i, recovered=True)
            if extracted is not None:
                fr, next_i = extracted
                frames.append(fr)
                i = next_i
                continue
        i += 1
    return frames


def decode_version(data: bytes) -> Dict[str, Any]:
    def clean(b: bytes) -> str:
        return b.split(b"\x00", 1)[0].decode("ascii", errors="replace")

    return {
        "protocol_version": clean(data[0:8]) if len(data) >= 8 else "",
        "software_version": clean(data[8:16]) if len(data) >= 16 else "",
        "encryption_hex": data[16:19].hex() if len(data) >= 19 else "",
        "hardware_version": clean(data[19:27]) if len(data) >= 27 else "",
        "device_name": clean(data[28:36]) if len(data) >= 36 else "",
    }


def decode_user_data(data: bytes) -> Dict[str, Any]:
    """Decode control + sensor bytes from either 6D 01 (36B) or 7D 01 (50B)."""
    if len(data) < 14:
        return {"raw_hex": data.hex(), "note": "short_payload"}
    c = data[2:14]  # 12-byte control block in E++2.18
    b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11 = c
    vswing_raw = b1 & 0x0F
    hswing_raw = b7 & 0x07
    fan_raw = b2 & 0x07
    mode_raw = (b2 >> 5) & 0x07
    res: Dict[str, Any] = {
        "target_temp": 16 + b0 + (0.5 if (b4 & 0x04) else 0.0),
        "vertical_swing": VSWING_NAMES.get(vswing_raw, f"0x{vswing_raw:02X}"),
        "vertical_swing_raw": vswing_raw,
        "fan_mode": FAN_NAMES.get(fan_raw, f"0x{fan_raw:02X}"),
        "fan_mode_raw": fan_raw,
        "special_mode": (b2 >> 3) & 0x03,
        "ac_mode": MODE_NAMES.get(mode_raw, f"0x{mode_raw:02X}"),
        "ac_mode_raw": mode_raw,
        "display_status": bool((b4 >> 1) & 1),
        "half_degree": bool((b4 >> 2) & 1),
        "steri_clean": bool((b4 >> 7) & 1),
        "ac_power": bool(b5 & 1),
        "health_mode": bool((b5 >> 1) & 1),
        "fast_mode": bool((b5 >> 3) & 1),
        "quiet_mode": bool((b5 >> 4) & 1),
        "sleep_mode": bool((b5 >> 5) & 1),
        "lock_remote": bool((b5 >> 6) & 1),
        "beeper_muted": bool((b5 >> 7) & 1),
        "horizontal_swing": HSWING_NAMES.get(hswing_raw, f"0x{hswing_raw:02X}"),
        "horizontal_swing_raw": hswing_raw,
        "eco_level": ECO_B7_TO_LVL.get(b7 & 0x38, 0),
        "self_cleaning": bool((b9 >> 4) & 1),
        "light_status": bool((b9 >> 5) & 1),
        "energy_saving": (b7 & 0x38) != 0 or bool((b9 >> 6) & 1),
        "extra_bytes_hex": f"b3={b3:02x} b8={b8:02x} b10={b10:02x} b11={b11:02x}",
        "raw_control_hex": c.hex(),
    }
    s = data[14:36]
    if len(s) >= 6:
        res["sensors"] = {
            "room_temp_c": round(s[0] * 0.5, 1) if s[0] > 0 else None,
            "room_humidity_pct": s[1] if s[1] <= 100 else None,
            "outdoor_temp_c": (s[2] - 64) if s[2] > 0 else None,
            "ac_type": "Cool Only" if (s[3] & 0x80) else "Heat & Cool",
            "error_code": s[4],
            "operation_source": ["Other", "Remote", "Button", "ESP/UART"][s[5] & 0x03],
            "operation_mode_hk": ["Cool", "Dry", "Heat", "Fan"][(s[5] >> 2) & 0x03],
            "raw_sensors_hex": s.hex(),
        }
    return res


def decode_big_data(data: bytes) -> Dict[str, Any]:
    """Decode 14-byte BigData block from 7D 01 (at offset 36) or 6D FE (at offset 2)."""
    if data[:2] == b"\x7D\x01" and len(data) >= 50:
        b = data[36:50]
    elif data[:2] == b"\x6D\xFE" and len(data) >= 16:
        b = data[2:16]
    else:
        return {"raw_hex": data.hex()}
    if len(b) < 14:
        return {"raw_hex": data.hex()}
    return {
        "power_w": (b[0] << 8) | b[1],
        "indoor_coil_temp_c": round(b[2] * 0.5 - 20.0, 1) if b[2] > 0 else None,
        "outdoor_out_air_c": (b[3] - 64) if b[3] > 0 else None,
        "outdoor_coil_temp_c": (b[4] - 64) if b[4] > 0 else None,
        "outdoor_in_air_c": (b[5] - 64) if b[5] > 0 else None,
        "outdoor_defrost_c": (b[6] - 64) if b[6] > 0 else None,
        "compressor_freq_hz": b[7],
        "compressor_current_a": round(((b[8] << 8) | b[9]) * 0.1, 1),
        "outdoor_fan_status": b[10] & 0x03,
        "defrost_status": (b[10] >> 2) & 0x03,
        "compressor_status": b[11] & 0x03,
        "indoor_fan_status": (b[11] >> 2) & 0x03,
        "four_way_valve_status": (b[11] >> 4) & 0x03,
        "eev_open_degree": (b[12] << 8) | b[13],
        "raw_bigdata_hex": b.hex(),
    }


def decode_alarm_status(data: bytes) -> Dict[str, Any]:
    payload = data[2:] if len(data) > 2 else data
    active = []
    code = 0
    for i in range(len(payload) - 1, -1, -1):
        byte_val = payload[i]
        for bit in range(8):
            if (byte_val >> bit) & 1:
                msg = HON_ALARM_MESSAGES[code] if code < len(HON_ALARM_MESSAGES) else f"Unknown alarm #{code}"
                active.append({"code": code, "message": msg})
            code += 1
    return {"active_count": len(active), "active_alarms": active, "raw_alarm_hex": data.hex()}


def encode_control_bytes(state: Dict[str, Any], prev_raw: Optional[bytes] = None) -> bytes:
    """Encode a 12-byte hOn control block (60 01 payload)."""
    buf = bytearray(prev_raw if prev_raw and len(prev_raw) == 12 else DEFAULT_CONTROL_RAW)

    temp = max(16.0, min(30.0, float(state.get("target_temp", 24))))
    buf[0] = int(temp) - 16

    vswing_default = PROTOCOL_TABLES["vertical_swing"].get(state.get("vertical_swing", "CENTER"), 0x06)
    vswing = int(state["vertical_swing_raw"]) if "vertical_swing_raw" in state else vswing_default
    buf[1] = (buf[1] & 0xF0) | (vswing & 0x0F)

    mode_str = state.get("ac_mode", "COOL")
    mode = int(state["ac_mode_raw"]) if "ac_mode_raw" in state else PROTOCOL_TABLES["modes"].get(mode_str, 0x01)
    fan_str = state.get("fan_mode", "AUTO")
    fan = int(state["fan_mode_raw"]) if "fan_mode_raw" in state else PROTOCOL_TABLES["fan_modes"].get(fan_str, 0x05)
    if mode == 0x06 and fan == 0x05:
        fan = 0x02  # Fan-only mode does not accept AUTO fan speed
    spec = int(state.get("special_mode", 0))
    buf[2] = (fan & 0x07) | ((spec & 0x03) << 3) | ((mode & 0x07) << 5)

    self_clean = bool(state.get("self_cleaning", False))
    steri_clean = bool(state.get("steri_clean", False))
    ac_power = bool(state.get("ac_power", True))
    if self_clean or steri_clean:
        ac_power = True  # Cleaning cycle requires indoor unit powered ON

    # Clear byte 4 before setting flags (per hon_climate.cpp line 532)
    b4 = 0
    if state.get("display_status", True):
        b4 |= 0x02
    if (temp % 1.0) >= 0.5:
        b4 |= 0x04
    if steri_clean:
        b4 |= 0x80
    buf[4] = b4

    fast = bool(state.get("fast_mode", False)) if (ac_power and mode != 0x06 and not self_clean) else False
    quiet = bool(state.get("quiet_mode", False)) if (ac_power and mode != 0x06 and not self_clean) else False
    sleep = bool(state.get("sleep_mode", False)) if (ac_power and not self_clean) else False

    b5 = 0
    if ac_power:
        b5 |= 0x01
    if state.get("health_mode", False):
        b5 |= 0x02
    if fast:
        b5 |= 0x08
    if quiet:
        b5 |= 0x10
    if sleep:
        b5 |= 0x20
    if state.get("lock_remote", False):
        b5 |= 0x40
    if state.get("beeper_muted", False):
        b5 |= 0x80
    buf[5] = b5

    hswing_default = PROTOCOL_TABLES["horizontal_swing"].get(state.get("horizontal_swing", "CENTER"), 0x00)
    hswing = int(state["horizontal_swing_raw"]) if "horizontal_swing_raw" in state else hswing_default
    eco_lvl = 0 if (not ac_power or self_clean) else int(state.get("eco_level", 0))
    eco_bits = ECO_LVL_TO_B7.get(eco_lvl, 0x00)
    buf[7] = (buf[7] & 0xC0) | eco_bits | (hswing & 0x07)

    # Byte 9: Only set self_cleaning (0x10) / light_status (0x20) explicitly.
    # HSU-15VQRA03T uses b4 bit 1 for Display LED and always clears b9 bit 5.
    b9 = buf[9] & ~0x70
    if self_clean:
        b9 |= 0x10
    if state.get("light_status", False) and not self_clean:
        b9 |= 0x20
    buf[9] = b9

    return bytes(buf)


def compute_byte_diffs(old_hex: str, new_hex: str) -> List[Dict[str, Any]]:
    old_b = bytes.fromhex(old_hex) if old_hex else b""
    new_b = bytes.fromhex(new_hex) if new_hex else b""
    diffs: List[Dict[str, Any]] = []
    for idx in range(max(len(old_b), len(new_b))):
        ov = old_b[idx] if idx < len(old_b) else None
        nv = new_b[idx] if idx < len(new_b) else None
        if ov != nv:
            xor_v = (ov ^ nv) if (ov is not None and nv is not None) else None
            diffs.append({
                "byte_idx": idx,
                "old": f"0x{ov:02X}" if ov is not None else "--",
                "new": f"0x{nv:02X}" if nv is not None else "--",
                "xor": f"0x{xor_v:02X}" if xor_v is not None else "--",
                "bits": f"{ov:08b} -> {nv:08b}" if (ov is not None and nv is not None) else "",
            })
    return diffs
