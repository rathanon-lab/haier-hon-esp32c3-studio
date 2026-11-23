"""Serial Transport, Auto-Sync, Auto-Reconnect & Compound Control Engine for Haier AC."""

from __future__ import annotations

import collections
import json
import threading
import time
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any, Callable, Deque, Dict, List, Optional

import serial
import serial.tools.list_ports

from cl_manager import SelfCleanManager, describe_snapshot
from protocol import (
    build_frame,
    decode_alarm_status,
    decode_big_data,
    decode_user_data,
    decode_version,
    encode_control_bytes,
    parse_frames,
)


class HaierSerialManager:
    def __init__(
        self,
        log_file: Path,
        cl_mgr: SelfCleanManager,
        serial_factory: Optional[Callable[[str], Any]] = None,
    ) -> None:
        self.lock = threading.RLock()
        self.log_file = log_file
        self.cl_mgr = cl_mgr
        self._serial_factory = serial_factory

        self.port: Optional[str] = None
        self.ser: Optional[Any] = None
        self.device_info: Dict[str, Any] = {}
        self.last_state: Dict[str, Any] = {}
        self.last_big_data: Dict[str, Any] = {}
        self.last_control_raw: Optional[bytes] = None
        self.last_user_raw_hex: str = ""
        self.last_big_raw_hex: str = ""
        self.baseline_user_hex: str = ""
        self.baseline_big_hex: str = ""
        self.active_timer: Optional[Dict[str, Any]] = None
        self.sticky_beeper_muted: bool = False
        self.auto_sync_enabled: bool = True
        self.smart_rule_enabled: bool = False
        self.last_alarms: Dict[str, Any] = {"active_count": 0, "active_alarms": []}

        self.stats: Dict[str, Any] = {
            "tx_count": 0,
            "rx_ok": 0,
            "rx_recovered_single_ff": 0,
            "rx_raw_failed": 0,
            "serial_errors": 0,
            "reconnect_count": 0,
            "last_rx_at": None,
            "last_error": None,
        }
        self._last_reconnect_attempt_ts: float = 0.0
        self._bigdata_supported: bool = True

        # O(1) in-memory ring buffer so /api/logs doesn't re-read multi-MB JSONL every 1.2s
        self._log_lock = threading.Lock()
        self.recent_logs: Deque[Dict[str, Any]] = collections.deque(maxlen=250)
        self._preload_recent_logs()

    def _preload_recent_logs(self) -> None:
        if not self.log_file.exists():
            return
        try:
            # Read only tail (up to last ~256KB) for fast startup even on huge logs
            size = self.log_file.stat().st_size
            read_from = max(0, size - 262144)
            with self.log_file.open("rb") as f:
                if read_from > 0:
                    f.seek(read_from)
                    f.readline()  # discard partial first line
                tail = f.read().decode("utf-8", errors="ignore").splitlines()
            with self._log_lock:
                for ln in tail[-250:]:
                    ln = ln.strip()
                    if not ln:
                        continue
                    try:
                        self.recent_logs.append(json.loads(ln))
                    except Exception:
                        pass
        except Exception:
            pass

    def get_recent_logs(self, limit: int = 50, log_filter: str = "all") -> List[Dict[str, Any]]:
        with self._log_lock:
            items = list(self.recent_logs)
        if log_filter == "commands":
            items = [
                x for x in items
                if x.get("dir") != "RX" or not str(x.get("note", "")).startswith("auto_sync")
            ]
        elif log_filter == "cl":
            items = [
                x for x in items
                if str(x.get("dir", "")).startswith("CL_")
                or (isinstance(x.get("decoded"), dict) and x["decoded"].get("self_cleaning"))
            ]
        elif log_filter == "errors":
            items = [
                x for x in items
                if "ERR" in str(x.get("dir", ""))
                or x.get("dir") == "RX_RAW"
                or "recovered=1" in str(x.get("note", ""))
            ]
        return list(reversed(items[-limit:]))

    def clear_logs(self) -> None:
        with self._log_lock:
            self.recent_logs.clear()
        if self.log_file.exists():
            self.log_file.write_text("", encoding="utf-8")

    def log_event(self, direction: str, frame_hex: str, decoded: Any, note: str = "") -> Dict[str, Any]:
        entry = {
            "ts": datetime.now().isoformat(timespec="seconds"),
            "dir": direction,
            "hex": frame_hex,
            "decoded": decoded,
            "note": note,
        }
        with self._log_lock:
            self.recent_logs.append(entry)
        try:
            self.log_file.parent.mkdir(parents=True, exist_ok=True)
            with self.log_file.open("a", encoding="utf-8") as f:
                f.write(json.dumps(entry, ensure_ascii=False) + "\n")
        except Exception:
            pass
        return entry

    def _open_port(self, port: str) -> Any:
        if self._serial_factory is not None:
            return self._serial_factory(port)
        return serial.Serial(port, 9600, timeout=0.04, write_timeout=0.5)

    def close_port(self) -> None:
        with self.lock:
            if self.ser:
                try:
                    if self.ser.is_open:
                        self.ser.close()
                except Exception:
                    pass
            self.ser = None

    def is_connected(self) -> bool:
        return bool(self.ser and getattr(self.ser, "is_open", False))

    def auto_detect_port(self) -> Dict[str, Any]:
        with self.lock:
            self.close_port()
            ports = list(serial.tools.list_ports.comports())
            results = []
            probe = build_frame(0x61, b"\x00\x07", 0x40)
            for p in ports:
                info = {
                    "port": p.device,
                    "desc": p.description,
                    "hwid": p.hwid,
                    "status": "skipped",
                    "ac_found": False,
                }
                try:
                    s = self._open_port(p.device)
                    s.reset_input_buffer()
                    s.write(probe)
                    t0 = time.time()
                    rx = bytearray()
                    while time.time() - t0 < 0.6:
                        c = s.read(256)
                        if c:
                            rx.extend(c)
                            if parse_frames(bytes(rx)):
                                time.sleep(0.02)
                                rx.extend(s.read(256))
                                break
                    parsed = parse_frames(bytes(rx))
                    if parsed and any(fr["type"] == "0x62" for fr in parsed):
                        fr62 = next(fr for fr in parsed if fr["type"] == "0x62")
                        self.ser = s
                        self.port = p.device
                        self.device_info = decode_version(fr62["data"])
                        self.stats["last_error"] = None
                        info["status"] = "AC_CONNECTED"
                        info["ac_found"] = True
                        info["device"] = self.device_info
                        self.log_event("AUTO_DETECT", fr62["hex"], self.device_info, note=f"Found AC on {p.device}")
                        results.append(info)
                        return {"ok": True, "selected": p.device, "device": self.device_info, "scanned": results}
                    s.close()
                    info["status"] = f"open_ok_no_ac (rx={len(rx)}B)"
                except Exception as e:
                    info["status"] = f"error: {e}"
                results.append(info)
            return {"ok": False, "selected": None, "scanned": results}

    def connect(self, port: str) -> None:
        with self.lock:
            self.close_port()
            self.port = port
            self.ser = self._open_port(port)
            self.stats["last_error"] = None

    def try_auto_reconnect(self, min_interval_s: float = 5.0) -> bool:
        """Attempt non-blocking periodic reconnect after a USB/Serial error."""
        now = time.time()
        if now - self._last_reconnect_attempt_ts < min_interval_s:
            return False
        self._last_reconnect_attempt_ts = now
        with self.lock:
            if self.is_connected():
                return True
            # First try last known port quickly
            if self.port:
                try:
                    s = self._open_port(self.port)
                    s.reset_input_buffer()
                    s.write(build_frame(0x61, b"\x00\x07", 0x40))
                    t0 = time.time()
                    rx = bytearray()
                    while time.time() - t0 < 0.55:
                        c = s.read(256)
                        if c:
                            rx.extend(c)
                            if parse_frames(bytes(rx)):
                                break
                    parsed = parse_frames(bytes(rx))
                    if parsed and any(fr["type"] == "0x62" for fr in parsed):
                        fr62 = next(fr for fr in parsed if fr["type"] == "0x62")
                        self.ser = s
                        self.device_info = decode_version(fr62["data"])
                        self.stats["reconnect_count"] += 1
                        self.stats["last_error"] = None
                        self.log_event("AUTO_RECONNECT", fr62["hex"], self.device_info, note=f"Reconnected on {self.port}")
                        return True
                    s.close()
                except Exception:
                    pass
            # Fallback: full port scan
            det = self.auto_detect_port()
            if det.get("ok"):
                self.stats["reconnect_count"] += 1
                self.log_event("AUTO_RECONNECT", "", {"port": det["selected"]}, note="Reconnected via auto-detect")
                return True
            return False

    def send_and_recv(
        self,
        pkt: bytes,
        wait_s: float = 0.45,
        note: str = "",
        log_tx: bool = True,
        log_rx_only_if_changed: bool = False,
        observe_cl: bool = True,
    ) -> List[Dict[str, Any]]:
        cl_directive: Optional[Dict[str, Any]] = None
        with self.lock:
            try:
                if not self.is_connected():
                    if not self.port:
                        raise RuntimeError("No COM port selected. Click Auto-Detect first.")
                    self.ser = self._open_port(self.port)
                self.ser.reset_input_buffer()
                self.ser.write(pkt)
                self.stats["tx_count"] += 1
                if log_tx:
                    self.log_event("TX", pkt.hex(), {"len": len(pkt)}, note=note)
                t0 = time.time()
                rx = bytearray()
                while time.time() - t0 < wait_s:
                    chunk = self.ser.read(256)
                    if chunk:
                        rx.extend(chunk)
                        if parse_frames(bytes(rx)):
                            time.sleep(0.02)
                            extra = self.ser.read(256)
                            if extra:
                                rx.extend(extra)
                            break
            except (serial.SerialException, OSError) as e:
                self.close_port()
                self.stats["serial_errors"] += 1
                self.stats["last_error"] = str(e)
                self.log_event("SERIAL_ERR", "", {"error": str(e)}, note=note)
                raise RuntimeError(f"Serial error on {self.port} (auto-closed for recovery): {e}") from e

            parsed = parse_frames(bytes(rx))
            for fr in parsed:
                self.stats["rx_ok"] += 1
                self.stats["last_rx_at"] = datetime.now().isoformat(timespec="seconds")
                if fr.get("recovered_single_ff"):
                    self.stats["rx_recovered_single_ff"] += 1

                dec: Any = {"raw_data": fr["data"].hex()}
                changed = True
                rec_tag = " recovered=1" if fr.get("recovered_single_ff") else ""

                if fr["type"] == "0x62":
                    dec = decode_version(fr["data"])
                    self.device_info = dec
                elif fr["type"] == "0x02" and fr["data"].startswith(b"\x6D\x01"):
                    new_hex = fr["data"].hex()
                    changed = new_hex != self.last_user_raw_hex
                    self.last_user_raw_hex = new_hex
                    dec = decode_user_data(fr["data"])
                    dec["beeper_muted"] = self.sticky_beeper_muted
                    self.last_state = dec
                    self.last_control_raw = fr["data"][2:14]
                    if observe_cl:
                        cl_directive = self.cl_mgr.observe_frame(self.last_state, self.last_big_data)
                elif fr["type"] == "0x02" and fr["data"].startswith(b"\x7D\x01") and len(fr["data"]) >= 50:
                    # 7D 01 carries BOTH control+sensors [2:36] AND big_data [36:50] in one frame!
                    user_slice = fr["data"][:36]
                    user_hex_norm = "6d01" + user_slice[2:].hex()
                    user_changed = user_hex_norm != self.last_user_raw_hex
                    self.last_user_raw_hex = user_hex_norm
                    user_dec = decode_user_data(user_slice)
                    user_dec["beeper_muted"] = self.sticky_beeper_muted
                    self.last_state = user_dec
                    self.last_control_raw = fr["data"][2:14]

                    big_hex = fr["data"].hex()
                    big_changed = big_hex != self.last_big_raw_hex
                    self.last_big_raw_hex = big_hex
                    big_dec = decode_big_data(fr["data"])
                    self.last_big_data = big_dec

                    changed = big_changed
                    dec = big_dec
                    if user_changed:
                        self.log_event(
                            "RX",
                            fr["hex"],
                            user_dec,
                            note=f"{note} type=0x02(7D01->user) crc={fr['crc_ok']}{rec_tag}",
                        )
                    if observe_cl:
                        cl_directive = self.cl_mgr.observe_frame(self.last_state, self.last_big_data)
                elif fr["type"] == "0x02" and fr["data"].startswith(b"\x6D\xFE"):
                    new_hex = fr["data"].hex()
                    changed = new_hex != self.last_big_raw_hex
                    self.last_big_raw_hex = new_hex
                    dec = decode_big_data(fr["data"])
                    self.last_big_data = dec
                    if observe_cl:
                        cl_directive = self.cl_mgr.observe_frame(None, self.last_big_data)
                elif fr["type"] == "0x74":
                    dec = decode_alarm_status(fr["data"])
                    self.last_alarms = dec
                elif fr["type"] == "0x03":
                    dec = {"rejected_0x03": True, "raw": fr["data"].hex()}
                elif fr["type"] == "0x7D":
                    dec = {"ack": True}

                if (not log_rx_only_if_changed) or changed:
                    self.log_event(
                        "RX",
                        fr["hex"],
                        dec,
                        note=f"{note} type={fr['type']} crc={fr['crc_ok']}{rec_tag}",
                    )
            if rx and not parsed:
                self.stats["rx_raw_failed"] += 1
                self.log_event("RX_RAW", bytes(rx).hex(), {"unparsed": True}, note=note)

        # Execute post-CL action outside serial lock if CL just completed naturally
        if cl_directive and cl_directive.get("event") == "cl_completed":
            self.execute_post_cl_action(cl_directive)

        return parsed

    def poll_status(self, include_bigdata: bool = True, note_prefix: str = "poll", log_tx: bool = True, log_rx_only_if_changed: bool = False) -> None:
        if include_bigdata and self._bigdata_supported:
            frames = self.send_and_recv(
                build_frame(0x01, b"\x4D\xFE", 0x40),
                wait_s=0.40,
                note=f"{note_prefix}_big_data" if log_tx else note_prefix,
                log_tx=log_tx,
                log_rx_only_if_changed=log_rx_only_if_changed,
            )
            if any(f["type"] == "0x02" and f["data"].startswith(b"\x7D\x01") and len(f["data"]) >= 50 for f in frames):
                return
        # Fallback to 4D 01 if 7D 01 wasn't returned
        self.send_and_recv(
            build_frame(0x01, b"\x4D\x01", 0x40),
            wait_s=0.40,
            note=f"{note_prefix}_user_data" if log_tx else note_prefix,
            log_tx=log_tx,
            log_rx_only_if_changed=log_rx_only_if_changed,
        )

    def set_power(self, on: bool, note_prefix: str = "power", observe_cl: bool = True) -> Dict[str, Any]:
        with self.lock:
            pkt = build_frame(0x01, b"\x5D\x01\x00" + (b"\x01" if on else b"\x00"), 0x40)
            frames = self.send_and_recv(
                pkt,
                wait_s=0.45,
                note=f"{note_prefix}_5D01_{'ON' if on else 'OFF'}",
                observe_cl=observe_cl,
            )
            if not frames or any(f["type"] == "0x03" for f in frames):
                if not self.last_state:
                    self.send_and_recv(build_frame(0x01, b"\x4D\x01", 0x40), note="pre_power_poll", observe_cl=observe_cl)
                merged = dict(self.last_state)
                merged["ac_power"] = on
                merged["beeper_muted"] = self.sticky_beeper_muted
                if not on:
                    merged["self_cleaning"] = False
                    merged["steri_clean"] = False
                ctrl_bytes = encode_control_bytes(merged, self.last_control_raw)
                pkt = build_frame(0x01, b"\x60\x01" + ctrl_bytes, 0x40)
                self.send_and_recv(
                    pkt,
                    wait_s=0.45,
                    note=f"{note_prefix}_6001_fallback_{'ON' if on else 'OFF'}",
                    observe_cl=observe_cl,
                )
            else:
                self.send_and_recv(
                    build_frame(0x01, b"\x4D\x01", 0x40),
                    wait_s=0.45,
                    note="post_power_poll",
                    observe_cl=observe_cl,
                )
            return {"ok": True, "tx_hex": pkt.hex(), "state": self.last_state, "big_data": self.last_big_data}

    def apply_control(
        self,
        updates: Dict[str, Any],
        note: str = "set_group_6001_12B",
        observe_cl: bool = True,
    ) -> Dict[str, Any]:
        with self.lock:
            if not self.last_state:
                self.send_and_recv(build_frame(0x01, b"\x4D\x01", 0x40), note="pre_control_poll", observe_cl=observe_cl)
            upd = dict(updates)
            if "beeper_muted" in upd:
                self.sticky_beeper_muted = bool(upd["beeper_muted"])
            merged = dict(self.last_state)
            merged["beeper_muted"] = self.sticky_beeper_muted
            merged.update(upd)
            if upd.get("fan_mode") in ("LOW", "MID", "HIGH"):
                merged["fast_mode"] = False
                merged["quiet_mode"] = False
            if "ac_mode" in upd and "ac_mode_raw" not in upd:
                merged.pop("ac_mode_raw", None)
            if "fan_mode" in upd and "fan_mode_raw" not in upd:
                merged.pop("fan_mode_raw", None)
            if "vertical_swing" in upd and "vertical_swing_raw" not in upd:
                merged.pop("vertical_swing_raw", None)
            if "horizontal_swing" in upd and "horizontal_swing_raw" not in upd:
                merged.pop("horizontal_swing_raw", None)

            ctrl_bytes = encode_control_bytes(merged, self.last_control_raw)
            pkt = build_frame(0x01, b"\x60\x01" + ctrl_bytes, 0x40)
            frames = self.send_and_recv(pkt, wait_s=0.45, note=note, observe_cl=observe_cl)
            if self.last_state:
                self.last_state["beeper_muted"] = self.sticky_beeper_muted
            return {
                "ok": True,
                "tx_hex": pkt.hex(),
                "frames": [{"type": f["type"], "hex": f["hex"]} for f in frames],
                "state": self.last_state,
                "big_data": self.last_big_data,
            }

    def schedule_dry_off(self, minutes: int = 10, note_prefix: str = "dry_coil", observe_cl: bool = True) -> Dict[str, Any]:
        with self.lock:
            self.apply_control(
                {
                    "ac_power": True,
                    "ac_mode": "FAN",
                    "fan_mode": "HIGH",
                    "fast_mode": False,
                    "quiet_mode": False,
                    "sleep_mode": False,
                    "eco_level": 0,
                    "self_cleaning": False,
                    "steri_clean": False,
                },
                note=f"{note_prefix}_{minutes}m_fan_high",
                observe_cl=observe_cl,
            )
            trig = datetime.now() + timedelta(minutes=minutes)
            self.active_timer = {
                "action": "OFF",
                "hours": round(minutes / 60, 2),
                "label": f"DRY&OFF ({minutes}m)",
                "trigger_ts": trig.timestamp(),
                "trigger_at": trig.strftime("%H:%M:%S"),
            }
            self.log_event("TIMER", "", self.active_timer, note=f"{note_prefix}_scheduled_{minutes}m")
            return {"ok": True, "state": self.last_state, "big_data": self.last_big_data, "timer": self.active_timer}

    def start_self_clean(self, post_action: Optional[str] = None) -> Dict[str, Any]:
        """Start Self-Clean (CL) cleanly from either OFF or ON state, preserving Pre-CL snapshot."""
        with self.lock:
            if not self.last_state:
                self.poll_status(include_bigdata=True, note_prefix="pre_cl_poll", log_tx=True)
            # Clear any competing power timer while starting CL
            if self.active_timer:
                self.log_event("CL_INFO", "", {"cleared_timer": self.active_timer}, note="cleared_timer_for_cl")
                self.active_timer = None

            sess = self.cl_mgr.begin_session(
                current_state=self.last_state,
                post_action=post_action,
                trigger_source="web_ui",
            )
            self.log_event(
                "CL_START",
                "",
                {
                    "pre_cl_summary": sess["pre_cl_summary"],
                    "post_action": sess["post_action"],
                    "started_from_off": not bool((sess.get("pre_cl_state") or {}).get("ac_power", False)),
                },
                note="start_self_clean",
            )

            # Attempt 1: Preserve current AC mode while powering ON + setting self_cleaning=True
            res = self.apply_control(
                {
                    "ac_power": True,
                    "self_cleaning": True,
                    "steri_clean": False,
                    "fast_mode": False,
                    "quiet_mode": False,
                    "sleep_mode": False,
                    "eco_level": 0,
                    "light_status": False,
                },
                note="cl_start_6001",
                observe_cl=False,
            )

            # Attempt 2 (fallback if AC didn't engage CL): ESPHome DRY+22C CL profile
            if not self.last_state.get("self_cleaning"):
                res = self.apply_control(
                    {
                        "ac_power": True,
                        "ac_mode": "DRY",
                        "target_temp": 22.0,
                        "vertical_swing": "CENTER",
                        "horizontal_swing": "CENTER",
                        "self_cleaning": True,
                        "steri_clean": False,
                        "fast_mode": False,
                        "quiet_mode": False,
                        "sleep_mode": False,
                        "eco_level": 0,
                        "light_status": False,
                    },
                    note="cl_start_6001_dry_fallback",
                    observe_cl=False,
                )

            # Attempt 3 (fallback): Single-parameter 5D 0D 00 01 (SELF_CLEANING)
            if not self.last_state.get("self_cleaning"):
                self.send_and_recv(
                    build_frame(0x01, b"\x5D\x0D\x00\x01", 0x40),
                    wait_s=0.45,
                    note="cl_start_5D0D_fallback",
                    observe_cl=False,
                )
                self.send_and_recv(build_frame(0x01, b"\x4D\x01", 0x40), wait_s=0.45, note="cl_start_verify", observe_cl=False)

            if not self.last_state.get("self_cleaning"):
                self.cl_mgr.rollback_failed_start()
                self.log_event("CL_ERR", "", {"error": "AC did not confirm self_cleaning=True"}, note="cl_start_rejected")
                return {
                    "ok": False,
                    "error": "แอร์ไม่ตอบรับคำสั่ง Self-Clean (CL) — ตรวจสอบสถานะแอร์หรือลองใหม่อีกครั้ง",
                    "state": self.last_state,
                    "big_data": self.last_big_data,
                    "cl": self.cl_mgr.get_public_status(),
                }

            self.cl_mgr.observe_frame(self.last_state, self.last_big_data)
            return {
                "ok": True,
                "tx_hex": res.get("tx_hex", ""),
                "state": self.last_state,
                "big_data": self.last_big_data,
                "cl": self.cl_mgr.get_public_status(),
            }

    def _run_restore_snapshot(self, pre_cl_state: Optional[Dict[str, Any]], note_prefix: str) -> str:
        """Restore the exact pre-CL AC state (OFF -> power OFF; ON -> restore mode/temp/fan/swing/eco)."""
        if not pre_cl_state:
            # Unknown pre-CL state -> safest default is Power OFF (same as ESPHome hOn)
            if self.last_state.get("ac_power", True):
                self.set_power(False, note_prefix=f"{note_prefix}_safe_off", observe_cl=False)
                return "ปิดเครื่องอัตโนมัติ (OFF — ไม่พบสถานะก่อนทำ CL)"
            return "แอร์ปิดเครื่องเรียบร้อยแล้ว (OFF)"

        if not pre_cl_state.get("ac_power", False):
            # AC was OFF before CL -> restore back to OFF!
            if self.last_state.get("ac_power", True):
                self.set_power(False, note_prefix=f"{note_prefix}_restore_off", observe_cl=False)
                return f"ปิดเครื่องกลับอัตโนมัติ (คืนสถานะก่อนทำ CL: {describe_snapshot(pre_cl_state)})"
            return f"แอร์ปิดเครื่องเรียบร้อยแล้ว (ตรงตามสถานะก่อนทำ CL: OFF)"

        # AC was ON before CL -> restore full pre-CL settings!
        restore_upd = dict(pre_cl_state)
        restore_upd["self_cleaning"] = False
        restore_upd["steri_clean"] = False
        restore_upd["beeper_muted"] = self.sticky_beeper_muted
        self.apply_control(restore_upd, note=f"{note_prefix}_restore_on", observe_cl=False)
        return f"คืนค่าเดิมอัตโนมัติ: {describe_snapshot(pre_cl_state)}"

    def execute_post_cl_action(self, directive: Dict[str, Any]) -> Dict[str, Any]:
        """Run automatic post-CL action when CL completes naturally."""
        post_action = directive.get("post_action", "restore_prev")
        pre_cl_state = directive.get("pre_cl_state")
        summary = ""

        # Brief settle delay so indoor MCU finishes its internal end-of-CL transition
        time.sleep(0.4)

        try:
            with self.lock:
                if post_action == "restore_prev":
                    summary = self._run_restore_snapshot(pre_cl_state, note_prefix="cl_done")
                elif post_action == "power_off":
                    if self.last_state.get("ac_power", True):
                        self.set_power(False, note_prefix="cl_done_off", observe_cl=False)
                        summary = "ปิดเครื่องอัตโนมัติ (OFF)"
                    else:
                        summary = "แอร์ปิดเครื่องเรียบร้อยแล้ว (OFF)"
                elif post_action == "dry_off_10m":
                    self.schedule_dry_off(minutes=10, note_prefix="cl_done_dry", observe_cl=False)
                    summary = "เป่าคอยล์ต่อโหมด FAN HIGH 10 นาที -> ปิดเครื่องอัตโนมัติ"
                else:  # stay_on
                    summary = "คงสถานะแอร์ไว้ตามเดิม (รีเซ็ตเฉพาะหน้าจอ/ปุ่มกลับหน้ามาตรฐาน)"
        except Exception as e:
            summary = f"ทำคำสั่งหลัง CL ไม่สำเร็จ: {e}"
            self.log_event("CL_ERR", "", {"error": str(e), "post_action": post_action}, note="cl_post_action_err")

        record = self.cl_mgr.finalize_session(
            status="completed",
            post_action_taken=post_action,
            post_action_summary=summary,
            final_state=self.last_state,
        )
        self.log_event("CL_COMPLETE", "", record, note=f"post_action={post_action}")
        return record

    def cancel_self_clean(self, mode: str = "restore") -> Dict[str, Any]:
        """Cancel active CL cycle and restore pre-CL state (or dry 5m -> OFF / immediate OFF)."""
        with self.lock:
            pub = self.cl_mgr.get_public_status()
            sess = pub.get("session") or {}
            pre_cl_state = sess.get("pre_cl_state") or self.cl_mgr.last_non_cl_state

            # Always clear self_cleaning bit first if still active
            if mode == "dry_5m":
                self.schedule_dry_off(minutes=5, note_prefix="cl_cancel_dry", observe_cl=False)
                summary = "ยกเลิก CL และเปลี่ยนเป็นเป่าแห้ง FAN HIGH 5 นาที -> ปิดเครื่องอัตโนมัติ"
                action_tag = "cancel_dry_5m"
            elif mode == "power_off":
                self.apply_control({"self_cleaning": False, "steri_clean": False}, note="cl_cancel_clear_bit", observe_cl=False)
                self.set_power(False, note_prefix="cl_cancel_off", observe_cl=False)
                summary = "ยกเลิก CL และปิดเครื่อง (OFF) เรียบร้อย"
                action_tag = "cancel_power_off"
            else:  # restore (default)
                if pre_cl_state and not pre_cl_state.get("ac_power", False):
                    # Started from OFF -> clear CL bit and turn OFF!
                    self.apply_control({"self_cleaning": False, "steri_clean": False}, note="cl_cancel_clear_bit", observe_cl=False)
                    self.set_power(False, note_prefix="cl_cancel_restore_off", observe_cl=False)
                    summary = f"ยกเลิก CL และปิดเครื่องกลับอัตโนมัติ (คืนสถานะก่อนทำ CL: OFF)"
                else:
                    summary = "ยกเลิก CL — " + self._run_restore_snapshot(pre_cl_state, note_prefix="cl_cancel")
                action_tag = "cancel_restore"

            record = self.cl_mgr.finalize_session(
                status="cancelled",
                post_action_taken=action_tag,
                post_action_summary=summary,
                final_state=self.last_state,
            )
            self.log_event("CL_CANCEL", "", record, note=f"cancel_mode={mode}")
            return {
                "ok": True,
                "state": self.last_state,
                "big_data": self.last_big_data,
                "timer": self.active_timer,
                "cl": self.cl_mgr.get_public_status(),
            }
