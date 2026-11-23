"""Automated verification suite for p25_esp32_haier_ac (Protocol, Self-Clean Lifecycle, Serial Manager, API)."""

from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_DIR))

from cl_manager import SelfCleanManager
from protocol import (
    build_frame,
    decode_big_data,
    decode_user_data,
    encode_control_bytes,
    parse_frames,
)
from serial_manager import HaierSerialManager


class FakeHaierACSerial:
    """In-memory emulator of Haier HSU-15VQRA03T UART board for deterministic testing."""

    def __init__(self, initial_ctrl: bytes | None = None) -> None:
        self.is_open = True
        self.ctrl = bytearray(initial_ctrl or bytes.fromhex("090625000200000008000000"))  # OFF, COOL 25C, AUTO
        self.room_temp_raw = 64  # 32.0C
        self.outdoor_temp_raw = 96  # 32C
        self.coil_raw = 102  # 31.0C
        self.pipe_raw = 138  # 74C
        self.power_w = 0
        self.freq_hz = 0
        self.amp_10 = 0
        self.comp_status = 0
        self.ifan_status = 0
        self.rx_buf = bytearray()
        self.tx_history: list[bytes] = []
        self.drop_next_leading_ff = False

    def reset_input_buffer(self) -> None:
        self.rx_buf.clear()

    def close(self) -> None:
        self.is_open = False

    def _build_sensors(self) -> bytes:
        s = bytearray(22)
        s[0] = self.room_temp_raw
        s[1] = 0
        s[2] = self.outdoor_temp_raw
        s[3] = 0x80
        s[4] = 0
        s[5] = 0x03
        return bytes(s)

    def _build_bigdata(self) -> bytes:
        b = bytearray(14)
        b[0] = (self.power_w >> 8) & 0xFF
        b[1] = self.power_w & 0xFF
        b[2] = self.coil_raw
        b[3] = self.pipe_raw
        b[7] = self.freq_hz
        b[8] = (self.amp_10 >> 8) & 0xFF
        b[9] = self.amp_10 & 0xFF
        b[10] = 0x02
        b[11] = (self.comp_status & 0x03) | ((self.ifan_status & 0x03) << 2) | (2 << 4)
        return bytes(b)

    def _queue_frame(self, fr: bytes) -> None:
        if self.drop_next_leading_ff:
            self.drop_next_leading_ff = False
            self.rx_buf.extend(fr[1:])
        else:
            self.rx_buf.extend(fr)

    def write(self, data: bytes) -> int:
        self.tx_history.append(data)
        parsed = parse_frames(data)
        if not parsed:
            return len(data)
        fr = parsed[0]
        ftype = int(fr["type"], 16)
        payload = fr["data"]
        if ftype == 0x61:
            ver = b"E++2.18\x00" + b"21060800" + b"\xf1\x00\x00" + b"21030803\x01" + b"U-ACG\x00\x00\x00"
            self._queue_frame(build_frame(0x62, ver, 0x40))
        elif ftype == 0x01 and payload == b"\x4D\x01":
            self._queue_frame(build_frame(0x02, b"\x6D\x01" + bytes(self.ctrl) + self._build_sensors(), 0x40))
        elif ftype == 0x01 and payload == b"\x4D\xFE":
            self._queue_frame(
                build_frame(0x02, b"\x7D\x01" + bytes(self.ctrl) + self._build_sensors() + self._build_bigdata(), 0x40)
            )
        elif ftype == 0x01 and payload.startswith(b"\x5D\x01\x00") and len(payload) == 4:
            if payload[3] == 1:
                self.ctrl[5] |= 0x01
            else:
                self.ctrl[5] &= ~0x01
                self.ctrl[9] &= ~0x10
            self._queue_frame(build_frame(0x02, b"\x6D\x01" + bytes(self.ctrl) + self._build_sensors(), 0x40))
        elif ftype == 0x01 and payload.startswith(b"\x60\x01") and len(payload) == 14:
            new_c = bytearray(payload[2:14])
            # Mimic real HSU-15VQRA03T quirks: clears beeper bit (b5 & 0x80) and light_status (b9 & 0x20) in status reply
            new_c[5] &= ~0x80
            new_c[9] &= ~0x20
            self.ctrl = new_c
            self._queue_frame(build_frame(0x02, b"\x6D\x01" + bytes(self.ctrl) + self._build_sensors(), 0x40))
        return len(data)

    def read(self, size: int = 256) -> bytes:
        out = bytes(self.rx_buf[:size])
        del self.rx_buf[:size]
        return out


class TestProtocol(unittest.TestCase):
    def test_recover_all_real_rx_raw_frames_from_log(self):
        log_path = PROJECT_DIR / "logs" / "haier_frames.jsonl"
        if not log_path.exists():
            self.skipTest("No real haier_frames.jsonl present")
        total = 0
        recovered = 0
        for ln in log_path.read_text(encoding="utf-8").splitlines():
            if not ln.strip():
                continue
            row = json.loads(ln)
            if row.get("dir") == "RX_RAW":
                total += 1
                raw = bytes.fromhex(row["hex"])
                parsed = parse_frames(raw, allow_single_ff_recovery=True)
                self.assertEqual(len(parsed), 1, f"Failed to recover RX_RAW at {row['ts']}")
                self.assertTrue(parsed[0]["chk_ok"])
                self.assertTrue(parsed[0]["crc_ok"])
                self.assertTrue(parsed[0]["recovered_single_ff"])
                recovered += 1
        self.assertGreater(total, 0)
        self.assertEqual(recovered, total)

    def test_single_ff_rejects_corrupted_crc(self):
        good = build_frame(0x01, b"\x4D\x01", 0x40)
        dropped = bytearray(good[1:])
        # Flip a bit in the payload -> checksum & CRC must reject!
        dropped[5] ^= 0x01
        self.assertEqual(parse_frames(bytes(dropped), allow_single_ff_recovery=True), [])

    def test_vertical_swing_0x00_and_b9_light_status(self):
        # Exact frame from today's log with V=0x00
        ctrl_raw = bytes.fromhex("0900c2000201000008000000")
        user = decode_user_data(b"\x6d\x01" + ctrl_raw + bytes(22))
        self.assertEqual(user["vertical_swing"], "DEFAULT")
        self.assertEqual(user["vertical_swing_raw"], 0)

        # Encoding with display_status=True must NOT set b9 bit 0x20
        enc = encode_control_bytes({"ac_power": True, "ac_mode": "COOL", "display_status": True}, ctrl_raw)
        self.assertEqual(enc[4] & 0x02, 0x02)
        self.assertEqual(enc[9] & 0x20, 0x00)


class TestSelfCleanLifecycle(unittest.TestCase):
    def _make_env(self, tmpdir: str, initial_ctrl_hex: str = "090625000200000008000000"):
        tmp = Path(tmpdir)
        cl = SelfCleanManager(tmp / "cl_session.json", tmp / "cl_history.jsonl")
        fake_ser = FakeHaierACSerial(bytes.fromhex(initial_ctrl_hex))
        mgr = HaierSerialManager(tmp / "frames.jsonl", cl_mgr=cl, serial_factory=lambda _p: fake_ser)
        mgr.connect("COM_MOCK")
        return cl, mgr, fake_ser

    def test_replay_real_today_cl_cycle_phases_and_metrics(self):
        log_path = PROJECT_DIR / "logs" / "haier_frames.jsonl"
        if not log_path.exists():
            self.skipTest("No real haier_frames.jsonl present")
        with tempfile.TemporaryDirectory() as td:
            cl = SelfCleanManager(Path(td) / "sess.json", Path(td) / "hist.jsonl")
            directive = None
            last_state = None
            last_bd = None
            seen_phases = []
            for ln in log_path.read_text(encoding="utf-8").splitlines():
                if not ln.strip():
                    continue
                row = json.loads(ln)
                ts = row.get("ts", "")
                if not ("2026-11-20T14:05:45" <= ts <= "2026-11-20T14:26:45"):
                    continue
                if row.get("dir") != "RX":
                    continue
                dec = row.get("decoded") or {}
                if "self_cleaning" in dec:
                    last_state = dec
                elif "power_w" in dec:
                    last_bd = dec
                else:
                    continue
                from datetime import datetime
                ep = datetime.fromisoformat(ts).timestamp()
                d = cl.observe_frame(last_state, last_bd, now_ts=ep)
                pub = cl.get_public_status(now_ts=ep)
                if pub["active"] and pub["session"]:
                    ph = pub["session"]["phase"]
                    if not seen_phases or seen_phases[-1] != ph:
                        seen_phases.append(ph)
                if d:
                    directive = d
                    break

            self.assertIsNotNone(directive, "Expected CL completion directive at 14:26:24+")
            self.assertEqual(directive["event"], "cl_completed")
            for expected in ("FREEZING", "WASHING", "DRYING"):
                self.assertIn(expected, seen_phases)
            # Metrics check
            pub = cl.get_public_status()
            m = pub["session"]["metrics"]
            self.assertEqual(m["min_coil_c"], 0.0)
            self.assertEqual(m["peak_power_w"], 1020)
            self.assertEqual(m["peak_freq_hz"], 80)
            self.assertGreaterEqual(m["peak_pipe_c"], 96)

    def test_start_cl_from_off_auto_restores_to_off(self):
        """User scenario: AC was OFF -> Start CL -> CL runs -> CL finishes -> AC auto-turns back OFF!"""
        with tempfile.TemporaryDirectory() as td:
            cl, mgr, fake = self._make_env(td, "090625000200000008000000")  # OFF, COOL 25C
            mgr.poll_status(include_bigdata=True)
            self.assertFalse(mgr.last_state["ac_power"])

            # Start CL from OFF
            res = mgr.start_self_clean(post_action="restore_prev")
            self.assertTrue(res["ok"])
            self.assertTrue(mgr.last_state["ac_power"])
            self.assertTrue(mgr.last_state["self_cleaning"])
            pub = cl.get_public_status()
            self.assertTrue(pub["active"])
            self.assertFalse(pub["session"]["pre_cl_state"]["ac_power"])

            # Phase 1: Freezing (comp ON, indoor fan OFF)
            fake.comp_status = 1; fake.ifan_status = 0; fake.freq_hz = 80; fake.power_w = 910; fake.coil_raw = 40  # 0.0C
            mgr.poll_status(include_bigdata=True)
            self.assertEqual(cl.get_public_status()["session"]["phase"], "FREEZING")

            # Phase 2: Washing (comp ON, indoor fan ON)
            fake.ifan_status = 1; fake.power_w = 1020; fake.coil_raw = 80  # 20.0C
            mgr.poll_status(include_bigdata=True)
            self.assertEqual(cl.get_public_status()["session"]["phase"], "WASHING")

            # Phase 3: Drying (comp OFF, indoor fan ON)
            fake.comp_status = 0; fake.freq_hz = 0; fake.power_w = 0
            mgr.poll_status(include_bigdata=True)
            self.assertEqual(cl.get_public_status()["session"]["phase"], "DRYING")

            # CL finishes on AC hardware (clears b9 0x10 bit, but stays ac_power=True just like real HSU-15VQRA03T!)
            fake.ctrl[9] &= ~0x10
            # Poll 1 (debounce)
            mgr.poll_status(include_bigdata=True)
            # Poll 2 (confirms completion -> auto executes Power OFF!)
            mgr.poll_status(include_bigdata=True)

            self.assertFalse(mgr.last_state["ac_power"], "AC MUST auto-power OFF when started from OFF!")
            self.assertFalse(mgr.last_state["self_cleaning"])
            final_pub = cl.get_public_status()
            self.assertFalse(final_pub["active"])
            self.assertTrue(final_pub["unacked_completion"])
            self.assertEqual(final_pub["last_completed"]["status"], "completed")
            self.assertEqual(len(cl.get_history()), 1)

    def test_start_cl_from_on_cool_eco_auto_restores_settings(self):
        """User scenario: AC was ON (COOL 25C, ECO L2) -> Start CL -> CL finishes -> restores COOL 25C ECO L2!"""
        # b0=0x09 (25C), b1=0x06 (CENTER), b2=0x25 (COOL+AUTO), b4=0x02, b5=0x01 (ON), b7=0x30 (ECO L2 + CENTER)
        with tempfile.TemporaryDirectory() as td:
            cl, mgr, fake = self._make_env(td, "090625000201003008000000")
            mgr.poll_status(include_bigdata=True)
            self.assertTrue(mgr.last_state["ac_power"])
            self.assertEqual(mgr.last_state["ac_mode"], "COOL")
            self.assertEqual(mgr.last_state["eco_level"], 2)

            res = mgr.start_self_clean(post_action="restore_prev")
            self.assertTrue(res["ok"])
            self.assertTrue(mgr.last_state["self_cleaning"])
            self.assertEqual(mgr.last_state["eco_level"], 0)  # ECO cleared during CL

            # Simulate AC switching mode to FAN MID during/after CL and clearing CL bit
            fake.ctrl[2] = 0xC2  # FAN + MID
            fake.ctrl[9] &= ~0x10
            mgr.poll_status(include_bigdata=True)
            mgr.poll_status(include_bigdata=True)

            self.assertTrue(mgr.last_state["ac_power"])
            self.assertEqual(mgr.last_state["ac_mode"], "COOL")
            self.assertEqual(mgr.last_state["target_temp"], 25.0)
            self.assertEqual(mgr.last_state["eco_level"], 2)
            self.assertFalse(mgr.last_state["self_cleaning"])

    def test_remote_triggered_cl_from_off_auto_restores_off(self):
        """IR Remote starts CL while AC was OFF -> App tracks & turns AC back OFF on finish!"""
        with tempfile.TemporaryDirectory() as td:
            cl, mgr, fake = self._make_env(td, "090625000200000008000000")  # OFF
            mgr.poll_status(include_bigdata=True)
            self.assertFalse(cl.get_public_status()["active"])

            # User presses CL on physical IR remote
            fake.ctrl[5] |= 0x01
            fake.ctrl[9] |= 0x10
            mgr.poll_status(include_bigdata=True)
            pub = cl.get_public_status()
            self.assertTrue(pub["active"])
            self.assertEqual(pub["session"]["trigger_source"], "remote_detected")
            self.assertFalse(pub["session"]["pre_cl_state"]["ac_power"])

            # CL finishes on hardware
            fake.ctrl[9] &= ~0x10
            mgr.poll_status(include_bigdata=True)
            mgr.poll_status(include_bigdata=True)

            self.assertFalse(mgr.last_state["ac_power"])
            self.assertFalse(cl.get_public_status()["active"])

    def test_cancel_cl_from_off_restores_off(self):
        with tempfile.TemporaryDirectory() as td:
            cl, mgr, fake = self._make_env(td, "090625000200000008000000")  # OFF
            mgr.poll_status(include_bigdata=True)
            mgr.start_self_clean(post_action="restore_prev")
            self.assertTrue(mgr.last_state["self_cleaning"])

            mgr.cancel_self_clean(mode="restore")
            self.assertFalse(mgr.last_state["ac_power"])
            self.assertFalse(mgr.last_state["self_cleaning"])
            pub = cl.get_public_status()
            self.assertFalse(pub["active"])
            self.assertEqual(pub["last_completed"]["status"], "cancelled")

    def test_persistence_across_restart_mid_cl(self):
        with tempfile.TemporaryDirectory() as td:
            sess_f = Path(td) / "cl_session.json"
            hist_f = Path(td) / "cl_history.jsonl"
            cl1 = SelfCleanManager(sess_f, hist_f)
            cl1.begin_session({"ac_power": False, "ac_mode": "COOL", "target_temp": 25.0}, post_action="restore_prev")

            # Simulate server restart: brand new manager reads persisted cl_session.json
            cl2 = SelfCleanManager(sess_f, hist_f)
            pub = cl2.get_public_status()
            self.assertTrue(pub["active"])
            self.assertFalse(pub["session"]["pre_cl_state"]["ac_power"])

    def test_single_ff_recovery_and_sticky_beeper_in_serial_manager(self):
        with tempfile.TemporaryDirectory() as td:
            cl, mgr, fake = self._make_env(td, "090625000201000008000000")
            mgr.apply_control({"beeper_muted": True})
            self.assertTrue(mgr.last_state["beeper_muted"])

            # Next auto-sync drops leading 0xFF AND AC reports b5 & 0x80 == 0
            fake.drop_next_leading_ff = True
            mgr.poll_status(include_bigdata=True, log_tx=False, log_rx_only_if_changed=True)
            self.assertEqual(mgr.stats["rx_recovered_single_ff"], 1)
            self.assertEqual(mgr.stats["rx_raw_failed"], 0)
            self.assertTrue(mgr.last_state["beeper_muted"], "Sticky beeper_muted must survive auto_sync!")


class TestFastApiRoutes(unittest.TestCase):
    def test_endpoints_smoke(self):
        os.environ["HAIER_NO_BG_WORKERS"] = "1"
        from fastapi.testclient import TestClient
        import app as app_mod

        orig_log_file = app_mod.mgr.log_file
        orig_sess_file = app_mod.cl_mgr.session_file
        with tempfile.TemporaryDirectory() as td:
            app_mod.mgr.log_file = Path(td) / "smoke_frames.jsonl"
            app_mod.cl_mgr.session_file = Path(td) / "smoke_cl_session.json"
            try:
                with TestClient(app_mod.app) as client:
                    r = client.get("/")
                    self.assertEqual(r.status_code, 200)
                    self.assertIn("โหมดล้างคอยล์", r.text)
                    self.assertIn("หน้ามาตรฐาน", r.text)

                    st = client.get("/api/state").json()
                    self.assertIn("cl", st)
                    self.assertIn("stats", st)

                    cfg = client.post("/api/cl/config", json={"post_action": "restore_prev", "apply_to_default": True})
                    self.assertEqual(cfg.status_code, 200)
                    self.assertEqual(cfg.json()["cl"]["default_post_action"], "restore_prev")

                    bad = client.post("/api/cl/config", json={"post_action": "invalid_mode"})
                    self.assertEqual(bad.status_code, 400)

                    hist = client.get("/api/cl/history").json()
                    self.assertTrue(hist["ok"])
                    self.assertGreaterEqual(len(hist["items"]), 1, "Expected seeded history from real 2026-11-20 CL log!")
            finally:
                app_mod.mgr.log_file = orig_log_file
                app_mod.cl_mgr.session_file = orig_sess_file


if __name__ == "__main__":
    unittest.main(verbosity=2)
