"""Self-Clean (CL) Lifecycle Manager for Haier HSU-15VQRA03T.

Tracks the complete Self-Clean cycle:
1. Captures Pre-CL AC state snapshot (whether started via Web UI or IR remote)
   and persists it to logs/cl_session.json so state survives restarts.
2. Detects live physical CL phases calibrated from HSU-15VQRA03T telemetry:
   STARTING -> FREEZING (comp ON, indoor fan OFF, coil -> 0C)
            -> WASHING  (comp ON, indoor fan ON, defrost/wash)
            -> DRYING   (comp OFF, indoor fan ON, post-wash blow)
            -> COMPLETED
3. On CL completion (self_cleaning True -> False), triggers the configured
   post-CL action:
   - restore_prev (default): Restore exact Pre-CL state (e.g. OFF -> OFF, ON COOL 25C -> COOL 25C)
   - power_off             : Always turn AC OFF
   - dry_off_10m           : Blow FAN HIGH for 10 min then auto-OFF
   - stay_on               : Do not send post-CL control command; just reset UI to Standard Screen
4. Logs completed/cancelled runs to logs/cl_history.jsonl.
"""

from __future__ import annotations

import json
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

ESTIMATED_CL_DURATION_SEC = 1230  # ~20m 30s measured on HSU-15VQRA03T (14:06:01 -> 14:26:24 = 1223s)

VALID_POST_ACTIONS = ("restore_prev", "power_off", "dry_off_10m", "stay_on")

POST_ACTION_LABELS: Dict[str, str] = {
    "restore_prev": "คืนค่าก่อนทำ CL อัตโนมัติ (ถ้าเดิมปิด -> ปิดกลับ)",
    "power_off": "ปิดเครื่องอัตโนมัติ (OFF)",
    "dry_off_10m": "เป่าคอยล์ต่อ 10 นาที (FAN HIGH) -> ปิดเครื่อง",
    "stay_on": "คงสถานะเดิมของแอร์ไว้ (รีเซ็ตเฉพาะหน้าจอ/ปุ่ม)",
}

PHASE_META: Dict[str, Dict[str, Any]] = {
    "IDLE": {"step": 0, "label": "พร้อมใช้งาน", "desc": "ไม่ได้อยู่ในโหมดล้างคอยล์"},
    "STARTING": {"step": 1, "label": "กำลังเริ่มต้น (Starting)", "desc": "เปิดระบบล้างคอยล์และเตรียมสตาร์ทคอมเพรสเซอร์"},
    "FREEZING": {"step": 2, "label": "เคลือบน้ำแข็ง (Frosting)", "desc": "พัดลมในห้องหยุด • คอมฯ เร่งสร้างเกล็ดน้ำแข็งจับฝุ่น (~12 นาที)"},
    "WASHING": {"step": 3, "label": "ละลายน้ำแข็งล้างคราบ (Defrost & Wash)", "desc": "พัดลมเริ่มหมุน • ละลายเกล็ดน้ำแข็งชะล้างคราบสกปรก (~5 นาที)"},
    "DRYING": {"step": 4, "label": "เป่าคอยล์แห้ง (Drying)", "desc": "คอมฯ ตัดแล้ว • พัดลมเป่าไล่ความชื้นออกจากคอยล์ (~3 นาที)"},
    "FINISHING": {"step": 5, "label": "กำลังคืนสถานะ...", "desc": "CL เสร็จสิ้น • กำลังดำเนินการหลัง CL และรีเซ็ตปุ่มกลับหน้ามาตรฐาน"},
    "COMPLETED": {"step": 5, "label": "เสร็จสมบูรณ์ (Completed)", "desc": "ล้างคอยล์เสร็จเรียบร้อย"},
    "CANCELLED": {"step": 0, "label": "ยกเลิกแล้ว (Cancelled)", "desc": "ยกเลิกกลางคันและคืนสถานะเรียบร้อย"},
}

SNAPSHOT_KEYS = (
    "ac_power",
    "ac_mode",
    "ac_mode_raw",
    "target_temp",
    "fan_mode",
    "fan_mode_raw",
    "vertical_swing",
    "vertical_swing_raw",
    "horizontal_swing",
    "horizontal_swing_raw",
    "eco_level",
    "health_mode",
    "fast_mode",
    "quiet_mode",
    "sleep_mode",
    "display_status",
    "lock_remote",
    "beeper_muted",
)


def extract_snapshot(state: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """Extract clean restoreable control parameters from a decoded 6D01/7D01 state."""
    if not state or "ac_power" not in state:
        return None
    snap: Dict[str, Any] = {}
    for k in SNAPSHOT_KEYS:
        if k in state:
            snap[k] = state[k]
    snap["self_cleaning"] = False
    snap["steri_clean"] = False
    return snap


def describe_snapshot(snap: Optional[Dict[str, Any]]) -> str:
    if not snap:
        return "ไม่ทราบสถานะก่อนทำ CL (ค่าปลอดภัย: ปิดเครื่อง OFF)"
    mode = snap.get("ac_mode", "COOL")
    temp = snap.get("target_temp", 25)
    fan = snap.get("fan_mode", "AUTO")
    if not snap.get("ac_power", False):
        return f"ปิดเครื่อง (OFF) [โหมดเดิม: {mode} {temp}°C]"
    parts = [f"เปิดเครื่อง ({mode}"]
    if mode != "FAN":
        parts[0] += f" {temp}°C"
    parts[0] += f" | Fan {fan})"
    if snap.get("eco_level", 0) > 0:
        parts.append(f"ECO L{snap['eco_level']}")
    if snap.get("fast_mode"):
        parts.append("TURBO")
    elif snap.get("quiet_mode"):
        parts.append("QUIET")
    if snap.get("sleep_mode"):
        parts.append("SLEEP")
    if snap.get("health_mode"):
        parts.append("UV/Health")
    return " ".join(parts)


def format_duration(seconds: Optional[float]) -> str:
    if seconds is None or seconds < 0:
        return "--"
    sec = int(round(seconds))
    mins, rem = divmod(sec, 60)
    hrs, mins = divmod(mins, 60)
    if hrs > 0:
        return f"{hrs} ชม. {mins} น. {rem} วิ."
    if mins > 0:
        return f"{mins} น. {rem} วิ."
    return f"{rem} วิ."


class SelfCleanManager:
    """Thread-safe Self-Clean lifecycle manager with JSON persistence."""

    def __init__(self, session_file: Path, history_file: Path) -> None:
        self.lock = threading.Lock()
        self.session_file = session_file
        self.history_file = history_file
        self.default_post_action: str = "restore_prev"
        self.last_non_cl_state: Optional[Dict[str, Any]] = None
        self.completion_seq: int = 0
        self.active_session: Optional[Dict[str, Any]] = None
        self.last_completed: Optional[Dict[str, Any]] = None
        self.unacked_completion: bool = False
        self._pending_finalize: bool = False
        self._load_persisted()

    def _load_persisted(self) -> None:
        if not self.session_file.exists():
            return
        try:
            data = json.loads(self.session_file.read_text(encoding="utf-8"))
            pa = data.get("default_post_action", "restore_prev")
            if pa in VALID_POST_ACTIONS:
                self.default_post_action = pa
            self.completion_seq = int(data.get("completion_seq", 0))
            self.last_non_cl_state = data.get("last_non_cl_state")
            self.active_session = data.get("active_session")
            self.last_completed = data.get("last_completed")
            self.unacked_completion = bool(data.get("unacked_completion", False))
        except Exception:
            pass

    def _save_persisted_locked(self) -> None:
        payload = {
            "default_post_action": self.default_post_action,
            "completion_seq": self.completion_seq,
            "last_non_cl_state": self.last_non_cl_state,
            "active_session": self.active_session,
            "last_completed": self.last_completed,
            "unacked_completion": self.unacked_completion,
            "updated_at": datetime.now().isoformat(timespec="seconds"),
        }
        try:
            self.session_file.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.session_file.with_suffix(".tmp")
            tmp.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
            tmp.replace(self.session_file)
        except Exception:
            pass

    def _append_history_locked(self, record: Dict[str, Any]) -> None:
        try:
            self.history_file.parent.mkdir(parents=True, exist_ok=True)
            with self.history_file.open("a", encoding="utf-8") as f:
                f.write(json.dumps(record, ensure_ascii=False) + "\n")
        except Exception:
            pass

    def _new_session_dict(
        self,
        pre_cl_state: Optional[Dict[str, Any]],
        trigger_source: str,
        post_action: str,
        now_ts: float,
    ) -> Dict[str, Any]:
        return {
            "active": True,
            "status": "running",
            "phase": "STARTING",
            "trigger_source": trigger_source,
            "post_action": post_action,
            "started_at": datetime.fromtimestamp(now_ts).isoformat(timespec="seconds"),
            "started_ts": now_ts,
            "pre_cl_state": pre_cl_state,
            "pre_cl_summary": describe_snapshot(pre_cl_state),
            "metrics": {
                "min_coil_c": None,
                "min_room_c": None,
                "peak_pipe_c": None,
                "peak_power_w": 0,
                "peak_freq_hz": 0,
                "saw_comp_on": False,
                "saw_freezing": False,
                "saw_washing": False,
                "saw_drying": False,
                "phases_seen": ["STARTING"],
            },
            "_off_confirm_count": 0,
            "_first_off_ts": None,
        }

    def set_post_action(self, post_action: str, apply_to_default: bool = True) -> Dict[str, Any]:
        if post_action not in VALID_POST_ACTIONS:
            raise ValueError(f"Invalid post_action: {post_action}. Must be one of {VALID_POST_ACTIONS}")
        with self.lock:
            if apply_to_default:
                self.default_post_action = post_action
            if self.active_session and self.active_session.get("active"):
                self.active_session["post_action"] = post_action
            self._save_persisted_locked()
            return self.get_public_status_locked()

    def ack_completion(self) -> Dict[str, Any]:
        with self.lock:
            self.unacked_completion = False
            self._save_persisted_locked()
            return self.get_public_status_locked()

    def begin_session(
        self,
        current_state: Optional[Dict[str, Any]],
        post_action: Optional[str] = None,
        trigger_source: str = "web_ui",
        now_ts: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Start tracking a new CL session, capturing pre-CL state."""
        ts = now_ts if now_ts is not None else time.time()
        with self.lock:
            pa = post_action if (post_action in VALID_POST_ACTIONS) else self.default_post_action
            if post_action in VALID_POST_ACTIONS:
                self.default_post_action = post_action
            # Prefer current non-CL state; if already CL, keep previously recorded pre_cl_state
            if current_state and not current_state.get("self_cleaning"):
                snap = extract_snapshot(current_state)
            elif self.active_session and self.active_session.get("pre_cl_state"):
                snap = self.active_session["pre_cl_state"]
            else:
                snap = self.last_non_cl_state
            self.active_session = self._new_session_dict(snap, trigger_source, pa, ts)
            self.unacked_completion = False
            self._pending_finalize = False
            self._save_persisted_locked()
            return dict(self.active_session)

    def rollback_failed_start(self) -> None:
        """Clear active session if AC rejected the CL start command."""
        with self.lock:
            self.active_session = None
            self._pending_finalize = False
            self._save_persisted_locked()

    def _update_phase_and_metrics_locked(
        self,
        sess: Dict[str, Any],
        state: Optional[Dict[str, Any]],
        big_data: Optional[Dict[str, Any]],
        now_ts: float,
    ) -> None:
        m = sess["metrics"]
        if state:
            rt = (state.get("sensors") or {}).get("room_temp_c")
            if rt is not None and (m["min_room_c"] is None or rt < m["min_room_c"]):
                m["min_room_c"] = rt
        if big_data:
            coil = big_data.get("indoor_coil_temp_c")
            if coil is not None and (m["min_coil_c"] is None or coil < m["min_coil_c"]):
                m["min_coil_c"] = coil
            pipe = big_data.get("outdoor_out_air_c")
            if pipe is not None and (m["peak_pipe_c"] is None or pipe > m["peak_pipe_c"]):
                m["peak_pipe_c"] = pipe
            pw = big_data.get("power_w")
            if pw is not None and pw > (m["peak_power_w"] or 0):
                m["peak_power_w"] = pw
            hz = big_data.get("compressor_freq_hz")
            if hz is not None and hz > (m["peak_freq_hz"] or 0):
                m["peak_freq_hz"] = hz

            comp_on = (big_data.get("compressor_status") == 1) or ((hz or 0) > 0)
            ifan_on = big_data.get("indoor_fan_status") == 1
            elapsed = max(0.0, now_ts - sess["started_ts"])

            cur_phase = sess.get("phase", "STARTING")
            next_phase = cur_phase

            if comp_on and not ifan_on:
                m["saw_comp_on"] = True
                m["saw_freezing"] = True
                next_phase = "FREEZING"
            elif comp_on and ifan_on:
                m["saw_comp_on"] = True
                m["saw_washing"] = True
                next_phase = "WASHING"
            elif (not comp_on) and ifan_on and (m["saw_comp_on"] or m["saw_freezing"] or m["saw_washing"] or elapsed >= 60):
                m["saw_drying"] = True
                next_phase = "DRYING"
            elif cur_phase == "STARTING" and comp_on:
                m["saw_comp_on"] = True
                next_phase = "FREEZING"

            # Prevent phase regression (e.g. DRYING -> STARTING)
            order = {"STARTING": 1, "FREEZING": 2, "WASHING": 3, "DRYING": 4, "FINISHING": 5}
            if order.get(next_phase, 0) >= order.get(cur_phase, 0):
                sess["phase"] = next_phase
                if next_phase not in m["phases_seen"]:
                    m["phases_seen"].append(next_phase)

    def observe_frame(
        self,
        state: Optional[Dict[str, Any]],
        big_data: Optional[Dict[str, Any]],
        now_ts: Optional[float] = None,
    ) -> Optional[Dict[str, Any]]:
        """Process a newly received AC status/bigdata update.

        Returns a post-CL completion directive dict ONLY ONCE when CL naturally
        transitions from active -> finished (confirmed), so the caller can execute
        the post-CL UART action outside locks and then call complete_session().
        """
        ts = now_ts if now_ts is not None else time.time()
        with self.lock:
            if not state or "self_cleaning" not in state:
                # Still update BigData metrics if CL is already active
                if self.active_session and self.active_session.get("active") and big_data:
                    self._update_phase_and_metrics_locked(self.active_session, None, big_data, ts)
                return None

            is_cl = bool(state.get("self_cleaning")) or bool(state.get("steri_clean"))

            if is_cl:
                if not self.active_session or not self.active_session.get("active"):
                    # CL started externally (e.g., physical IR remote) or resumed
                    snap = self.last_non_cl_state
                    self.active_session = self._new_session_dict(
                        pre_cl_state=snap,
                        trigger_source="remote_detected",
                        post_action=self.default_post_action,
                        now_ts=ts,
                    )
                    self.unacked_completion = False
                    self._pending_finalize = False
                self.active_session["_off_confirm_count"] = 0
                self.active_session["_first_off_ts"] = None
                self._update_phase_and_metrics_locked(self.active_session, state, big_data, ts)
                self._save_persisted_locked()
                return None

            # is_cl is False
            if self.active_session and self.active_session.get("active") and not self._pending_finalize:
                self.active_session["_off_confirm_count"] = self.active_session.get("_off_confirm_count", 0) + 1
                if self.active_session.get("_first_off_ts") is None:
                    self.active_session["_first_off_ts"] = ts
                # Confirm immediately if AC already powered off, or after 2 polls / >=1.2s
                confirmed = (
                    (not state.get("ac_power", True))
                    or self.active_session["_off_confirm_count"] >= 2
                    or (ts - self.active_session["_first_off_ts"]) >= 1.2
                )
                if confirmed:
                    self._pending_finalize = True
                    self.active_session["phase"] = "FINISHING"
                    self._save_persisted_locked()
                    return {
                        "event": "cl_completed",
                        "post_action": self.active_session.get("post_action", self.default_post_action),
                        "pre_cl_state": self.active_session.get("pre_cl_state"),
                        "current_state": dict(state),
                    }
                return None

            # Not in CL -> keep rolling snapshot of normal AC state!
            snap = extract_snapshot(state)
            if snap is not None:
                self.last_non_cl_state = snap
            return None

    def finalize_session(
        self,
        status: str,
        post_action_taken: str,
        post_action_summary: str,
        final_state: Optional[Dict[str, Any]] = None,
        now_ts: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Mark the active CL session completed or cancelled, persist to history, and clear active state."""
        ts = now_ts if now_ts is not None else time.time()
        with self.lock:
            sess = self.active_session or {}
            started_ts = sess.get("started_ts", ts)
            duration_sec = max(0, int(round(ts - started_ts)))
            self.completion_seq += 1
            record = {
                "seq": self.completion_seq,
                "status": status,  # "completed" | "cancelled"
                "trigger_source": sess.get("trigger_source", "web_ui"),
                "started_at": sess.get("started_at", datetime.fromtimestamp(started_ts).isoformat(timespec="seconds")),
                "finished_at": datetime.fromtimestamp(ts).isoformat(timespec="seconds"),
                "duration_sec": duration_sec,
                "duration_text": format_duration(duration_sec),
                "pre_cl_state": sess.get("pre_cl_state"),
                "pre_cl_summary": sess.get("pre_cl_summary", describe_snapshot(sess.get("pre_cl_state"))),
                "post_action": post_action_taken,
                "post_action_summary": post_action_summary,
                "final_state_summary": describe_snapshot(extract_snapshot(final_state)) if final_state else "--",
                "metrics": sess.get("metrics", {}),
            }
            self.last_completed = record
            self.active_session = None
            self.unacked_completion = True
            self._pending_finalize = False
            if final_state and not final_state.get("self_cleaning"):
                snap = extract_snapshot(final_state)
                if snap is not None:
                    self.last_non_cl_state = snap
            self._append_history_locked(record)
            self._save_persisted_locked()
            return record

    def get_public_status(self, now_ts: Optional[float] = None) -> Dict[str, Any]:
        with self.lock:
            return self.get_public_status_locked(now_ts)

    def get_public_status_locked(self, now_ts: Optional[float] = None) -> Dict[str, Any]:
        ts = now_ts if now_ts is not None else time.time()
        active = bool(self.active_session and self.active_session.get("active"))
        sess_pub: Optional[Dict[str, Any]] = None
        if active and self.active_session:
            s = self.active_session
            elapsed = max(0, int(round(ts - s["started_ts"])))
            eta = max(0, ESTIMATED_CL_DURATION_SEC - elapsed)
            pct = min(99, int(round((elapsed / ESTIMATED_CL_DURATION_SEC) * 100)))
            phase = s.get("phase", "STARTING")
            pmeta = PHASE_META.get(phase, PHASE_META["STARTING"])
            sess_pub = {
                "active": True,
                "status": s.get("status", "running"),
                "phase": phase,
                "phase_step": pmeta["step"],
                "phase_label": pmeta["label"],
                "phase_desc": pmeta["desc"],
                "trigger_source": s.get("trigger_source", "web_ui"),
                "post_action": s.get("post_action", self.default_post_action),
                "post_action_label": POST_ACTION_LABELS.get(s.get("post_action", self.default_post_action), ""),
                "started_at": s.get("started_at"),
                "elapsed_sec": elapsed,
                "elapsed_text": format_duration(elapsed),
                "eta_sec": eta,
                "eta_text": format_duration(eta),
                "progress_pct": pct,
                "pre_cl_state": s.get("pre_cl_state"),
                "pre_cl_summary": s.get("pre_cl_summary"),
                "metrics": s.get("metrics", {}),
            }
        return {
            "active": active,
            "default_post_action": self.default_post_action,
            "post_action_options": [
                {"id": k, "label": POST_ACTION_LABELS[k]} for k in VALID_POST_ACTIONS
            ],
            "session": sess_pub,
            "last_completed": self.last_completed,
            "unacked_completion": self.unacked_completion,
            "completion_seq": self.completion_seq,
        }

    def get_history(self, limit: int = 30) -> List[Dict[str, Any]]:
        if not self.history_file.exists():
            return []
        rows: List[Dict[str, Any]] = []
        try:
            for ln in self.history_file.read_text(encoding="utf-8").splitlines():
                ln = ln.strip()
                if not ln:
                    continue
                try:
                    rows.append(json.loads(ln))
                except Exception:
                    pass
        except Exception:
            return []
        return list(reversed(rows[-limit:]))

    def seed_history_from_frame_log(self, frame_log_file: Path) -> int:
        """One-time scan of haier_frames.jsonl to backfill past CL runs when cl_history.jsonl is empty."""
        if self.history_file.exists() and self.history_file.stat().st_size > 0:
            return 0
        if not frame_log_file.exists():
            return 0

        imported = 0
        in_cl = False
        start_ts_str = ""
        start_epoch = 0.0
        pre_snap: Optional[Dict[str, Any]] = None
        last_non_cl: Optional[Dict[str, Any]] = None
        metrics: Dict[str, Any] = {}

        def _reset_metrics() -> Dict[str, Any]:
            return {
                "min_coil_c": None,
                "min_room_c": None,
                "peak_pipe_c": None,
                "peak_power_w": 0,
                "peak_freq_hz": 0,
                "phases_seen": ["STARTING"],
            }

        try:
            with frame_log_file.open("r", encoding="utf-8") as f:
                for ln in f:
                    ln = ln.strip()
                    if not ln:
                        continue
                    try:
                        entry = json.loads(ln)
                    except Exception:
                        continue
                    if entry.get("dir") != "RX":
                        continue
                    dec = entry.get("decoded")
                    if not isinstance(dec, dict):
                        continue
                    ts_str = entry.get("ts", "")
                    if "self_cleaning" in dec:
                        is_cl = bool(dec.get("self_cleaning"))
                        if is_cl and not in_cl:
                            in_cl = True
                            start_ts_str = ts_str
                            try:
                                start_epoch = datetime.fromisoformat(ts_str).timestamp()
                            except Exception:
                                start_epoch = 0.0
                            pre_snap = last_non_cl
                            metrics = _reset_metrics()
                        if is_cl:
                            rt = (dec.get("sensors") or {}).get("room_temp_c")
                            if rt is not None and (metrics["min_room_c"] is None or rt < metrics["min_room_c"]):
                                metrics["min_room_c"] = rt
                        elif in_cl and not is_cl:
                            try:
                                end_epoch = datetime.fromisoformat(ts_str).timestamp()
                            except Exception:
                                end_epoch = start_epoch
                            dur = max(0, int(round(end_epoch - start_epoch)))
                            rec = {
                                "seq": imported + 1,
                                "status": "completed",
                                "trigger_source": "imported_from_log",
                                "started_at": start_ts_str,
                                "finished_at": ts_str,
                                "duration_sec": dur,
                                "duration_text": format_duration(dur),
                                "pre_cl_state": pre_snap,
                                "pre_cl_summary": describe_snapshot(pre_snap),
                                "post_action": "legacy_none",
                                "post_action_summary": "บันทึกจาก Log เดิม (ก่อนอัปเกรดระบบ Auto-Restore)",
                                "final_state_summary": describe_snapshot(extract_snapshot(dec)),
                                "metrics": metrics,
                            }
                            with self.lock:
                                self._append_history_locked(rec)
                            imported += 1
                            in_cl = False
                        else:
                            last_non_cl = extract_snapshot(dec)
                    elif in_cl and "power_w" in dec:
                        coil = dec.get("indoor_coil_temp_c")
                        if coil is not None and (metrics["min_coil_c"] is None or coil < metrics["min_coil_c"]):
                            metrics["min_coil_c"] = coil
                        pipe = dec.get("outdoor_out_air_c")
                        if pipe is not None and (metrics["peak_pipe_c"] is None or pipe > metrics["peak_pipe_c"]):
                            metrics["peak_pipe_c"] = pipe
                        pw = dec.get("power_w") or 0
                        if pw > metrics["peak_power_w"]:
                            metrics["peak_power_w"] = pw
                        hz = dec.get("compressor_freq_hz") or 0
                        if hz > metrics["peak_freq_hz"]:
                            metrics["peak_freq_hz"] = hz
                        comp_on = (dec.get("compressor_status") == 1) or (hz > 0)
                        ifan_on = dec.get("indoor_fan_status") == 1
                        if comp_on and not ifan_on and "FREEZING" not in metrics["phases_seen"]:
                            metrics["phases_seen"].append("FREEZING")
                        elif comp_on and ifan_on and "WASHING" not in metrics["phases_seen"]:
                            metrics["phases_seen"].append("WASHING")
                        elif (not comp_on) and ifan_on and "FREEZING" in metrics["phases_seen"] and "DRYING" not in metrics["phases_seen"]:
                            metrics["phases_seen"].append("DRYING")
        except Exception:
            pass
        return imported
