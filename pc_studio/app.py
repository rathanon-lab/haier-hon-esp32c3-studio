"""p25_esp32_haier_ac — Haier AC hOn (E++2.18) Web Studio & Self-Clean Automation Server."""

from __future__ import annotations

import collections
import json
import os
import threading
import time
import urllib.request
from contextlib import asynccontextmanager
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional

import serial.tools.list_ports
import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse, PlainTextResponse
from pydantic import BaseModel

from cl_manager import SelfCleanManager
from protocol import PROTOCOL_TABLES, build_frame, compute_byte_diffs
from serial_manager import HaierSerialManager

BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"
LOGS_DIR = BASE_DIR / "logs"
LOGS_DIR.mkdir(parents=True, exist_ok=True)

LOG_FILE = LOGS_DIR / "haier_frames.jsonl"
DHT_LOG_FILE = LOGS_DIR / "dht_compare_log.jsonl"
DIFF_FILE = LOGS_DIR / "remote_diff_map.json"
CL_SESSION_FILE = LOGS_DIR / "cl_session.json"
CL_HISTORY_FILE = LOGS_DIR / "cl_history.jsonl"

cl_mgr = SelfCleanManager(session_file=CL_SESSION_FILE, history_file=CL_HISTORY_FILE)
mgr = HaierSerialManager(log_file=LOG_FILE, cl_mgr=cl_mgr)

ext_dht_state: Dict[str, Any] = {"ok": False, "ip": "192.168.1.25", "data": None, "error": "init"}
_dht_lock = threading.Lock()
_recent_dht_logs: Deque[Dict[str, Any]] = collections.deque(maxlen=200)
_stop_event = threading.Event()


def _preload_dht_logs() -> None:
    if not DHT_LOG_FILE.exists():
        return
    try:
        lines = DHT_LOG_FILE.read_text(encoding="utf-8", errors="ignore").splitlines()[-200:]
        with _dht_lock:
            _recent_dht_logs.clear()
            for ln in lines:
                ln = ln.strip()
                if not ln:
                    continue
                try:
                    _recent_dht_logs.append(json.loads(ln))
                except Exception:
                    pass
    except Exception:
        pass


def timer_worker() -> None:
    # Auto-connect to AC immediately on startup so headless CL monitoring works even without browser open
    if mgr.auto_sync_enabled and not mgr.is_connected():
        try:
            if mgr.try_auto_reconnect(min_interval_s=0.0):
                mgr.poll_status(include_bigdata=True, note_prefix="startup_poll", log_tx=True)
        except Exception:
            pass

    while not _stop_event.wait(1.0):
        # 1. Execute scheduled ON/OFF timer (guarded: never fire while Self-Clean is active)
        t = mgr.active_timer
        if t and datetime.now().timestamp() >= t["trigger_ts"]:
            if not mgr.last_state.get("self_cleaning"):
                action = t["action"]
                mgr.active_timer = None
                try:
                    mgr.set_power(action == "ON", note_prefix=f"timer_auto_{action}")
                except Exception as e:
                    mgr.log_event("TIMER_ERR", "", {"error": str(e)}, note=action)

        # 2. Auto-Sync & Auto-Reconnect
        if mgr.auto_sync_enabled:
            if not mgr.is_connected():
                mgr.try_auto_reconnect(min_interval_s=5.0)
                continue
            try:
                mgr.poll_status(
                    include_bigdata=True,
                    note_prefix="auto_sync",
                    log_tx=False,
                    log_rx_only_if_changed=True,
                )
                # 3. Smart Rule: Auto-ECO <= 27C (STRICTLY guarded against firing during Self-Clean!)
                st = mgr.last_state
                if (
                    mgr.smart_rule_enabled
                    and st.get("ac_power")
                    and not st.get("self_cleaning")
                    and not st.get("steri_clean")
                    and st.get("ac_mode") == "COOL"
                ):
                    rt = (st.get("sensors") or {}).get("room_temp_c")
                    if rt is not None and rt <= 27.0 and st.get("eco_level", 0) != 3:
                        mgr.apply_control(
                            {"eco_level": 3, "fan_mode": "LOW", "fast_mode": False, "quiet_mode": False},
                            note="smart_rule_eco_L3",
                        )
            except Exception:
                pass


def _poll_ext_dht_loop() -> None:
    tick = 0
    while not _stop_event.wait(2.5):
        ip = ext_dht_state["ip"]
        try:
            req = urllib.request.Request(f"http://{ip}/api/sensors", method="GET")
            with urllib.request.urlopen(req, timeout=1.5) as resp:
                data = json.loads(resp.read().decode("utf-8"))
                ext_dht_state["data"] = data
                ext_dht_state["ok"] = True
                ext_dht_state["error"] = None
                tick += 1
                if tick % 2 == 1:  # log every ~5s
                    s1 = data.get("s1") or {}
                    s2 = data.get("s2") or {}
                    haier_room = (mgr.last_state or {}).get("sensors", {}).get("room_temp_c")
                    s1_t = s1.get("temp_c") if s1.get("ok") else None
                    s2_t = s2.get("temp_c") if s2.get("ok") else None
                    row = {
                        "ts": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                        "s1_temp_c": s1_t,
                        "s1_hum_pct": s1.get("humidity_pct") if s1.get("ok") else None,
                        "s1_err": s1.get("err"),
                        "s2_temp_c": s2_t,
                        "s2_hum_pct": s2.get("humidity_pct") if s2.get("ok") else None,
                        "s2_err": s2.get("err"),
                        "haier_room_c": haier_room,
                        "delta_s1_haier": round(s1_t - haier_room, 1) if (s1_t is not None and haier_room is not None) else None,
                        "delta_s2_haier": round(s2_t - haier_room, 1) if (s2_t is not None and haier_room is not None) else None,
                        "delta_s1_s2": round(s1_t - s2_t, 1) if (s1_t is not None and s2_t is not None) else None,
                    }
                    with _dht_lock:
                        _recent_dht_logs.append(row)
                    with DHT_LOG_FILE.open("a", encoding="utf-8") as f:
                        f.write(json.dumps(row, ensure_ascii=False) + "\n")
        except Exception as exc:
            ext_dht_state["ok"] = False
            ext_dht_state["error"] = str(exc)


@asynccontextmanager
async def lifespan(_app: FastAPI):
    _stop_event.clear()
    _preload_dht_logs()
    cl_mgr.seed_history_from_frame_log(LOG_FILE)
    if os.environ.get("HAIER_NO_BG_WORKERS") != "1":
        threading.Thread(target=timer_worker, daemon=True, name="haier-timer-sync").start()
        threading.Thread(target=_poll_ext_dht_loop, daemon=True, name="haier-ext-dht").start()
    yield
    _stop_event.set()
    mgr.close_port()


app = FastAPI(title="p25 ESP32 Haier AC Lab (HSU-15VQRA03T)", lifespan=lifespan)


# ==============================================================================
# Request Schemas
# ==============================================================================
class ConnectReq(BaseModel):
    port: str


class ControlReq(BaseModel):
    updates: Dict[str, Any]


class PowerReq(BaseModel):
    on: bool


class TimerReq(BaseModel):
    action: str
    hours: float = 0.5


class DiffSaveReq(BaseModel):
    label: str
    user_diffs: List[Dict[str, Any]]
    big_diffs: List[Dict[str, Any]]
    raw_user_hex: str


class ClStartReq(BaseModel):
    post_action: Optional[str] = None


class ClCancelReq(BaseModel):
    mode: str = "restore"  # "restore" | "dry_5m" | "power_off"


class ClConfigReq(BaseModel):
    post_action: str
    apply_to_default: bool = True


def _full_state_payload(extra: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    payload: Dict[str, Any] = {
        "ok": mgr.is_connected(),
        "connected": mgr.is_connected(),
        "port": mgr.port,
        "device": mgr.device_info,
        "state": mgr.last_state,
        "big_data": mgr.last_big_data,
        "alarms": mgr.last_alarms,
        "timer": mgr.active_timer,
        "auto_sync": mgr.auto_sync_enabled,
        "smart_rule": mgr.smart_rule_enabled,
        "ext_dht": ext_dht_state,
        "cl": cl_mgr.get_public_status(),
        "stats": mgr.stats,
    }
    if extra:
        payload.update(extra)
    return payload


# ==============================================================================
# Serial & Connection Endpoints
# ==============================================================================
@app.get("/api/ports")
def api_ports():
    ports = [{"device": p.device, "desc": p.description, "hwid": p.hwid} for p in serial.tools.list_ports.comports()]
    return {"ports": ports, "current": mgr.port, "connected": mgr.is_connected()}


@app.post("/api/auto_detect")
def api_auto_detect():
    return mgr.auto_detect_port()


@app.post("/api/connect")
def api_connect(req: ConnectReq):
    try:
        mgr.connect(req.port)
        return {"ok": True, "port": req.port}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.post("/api/reconnect")
@app.post("/api/reset_all")
def api_reconnect():
    """Safely close and re-detect COM port WITHOUT wiping historical logs."""
    mgr.close_port()
    det = mgr.auto_detect_port()
    if det.get("ok"):
        return api_poll(include_bigdata=True)
    return {"ok": False, "connected": False, "detect": det}


# ==============================================================================
# Self-Clean (CL) Lifecycle Endpoints
# ==============================================================================
@app.get("/api/cl/status")
def api_cl_status():
    return {"ok": True, "cl": cl_mgr.get_public_status()}


@app.post("/api/cl/start")
def api_cl_start(req: ClStartReq = ClStartReq()):
    try:
        res = mgr.start_self_clean(post_action=req.post_action)
        return _full_state_payload(res)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/cl/cancel")
def api_cl_cancel(req: ClCancelReq = ClCancelReq()):
    try:
        res = mgr.cancel_self_clean(mode=req.mode)
        return _full_state_payload(res)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/cl/config")
def api_cl_config(req: ClConfigReq):
    try:
        cl_pub = cl_mgr.set_post_action(req.post_action, apply_to_default=req.apply_to_default)
        mgr.log_event("CL_CONFIG", "", {"post_action": req.post_action}, note="update_post_cl_action")
        return {"ok": True, "cl": cl_pub}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.post("/api/cl/ack")
def api_cl_ack():
    return {"ok": True, "cl": cl_mgr.ack_completion()}


@app.get("/api/cl/history")
def api_cl_history(limit: int = 30):
    return {"ok": True, "items": cl_mgr.get_history(limit=limit)}


# ==============================================================================
# AC State, Control & Smart Automation Endpoints
# ==============================================================================
@app.get("/api/state")
def api_state():
    return _full_state_payload()


@app.post("/api/auto_sync")
def api_auto_sync(enabled: bool = True):
    mgr.auto_sync_enabled = enabled
    return {"ok": True, "auto_sync": mgr.auto_sync_enabled}


@app.post("/api/smart_rule")
def api_smart_rule(enabled: bool = False):
    mgr.smart_rule_enabled = enabled
    mgr.log_event("SMART_RULE", "", {"enabled": enabled}, note="toggle_auto_eco_27c")
    return {"ok": True, "smart_rule": mgr.smart_rule_enabled}


@app.post("/api/dry_off")
def api_dry_off(minutes: int = 10):
    try:
        res = mgr.schedule_dry_off(minutes=minutes)
        return _full_state_payload(res)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/alarms")
def api_alarms():
    try:
        frames = mgr.send_and_recv(build_frame(0x73, b"", 0x40), wait_s=0.5, note="get_alarm_0x73")
        return _full_state_payload({"ok": True, "frames": [{"type": f["type"], "hex": f["hex"]} for f in frames]})
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/poll")
def api_poll(include_bigdata: bool = True):
    try:
        if not mgr.port or not mgr.is_connected():
            det = mgr.auto_detect_port()
            if not det["ok"]:
                return _full_state_payload({"ok": False, "detect": det})
        if not mgr.device_info:
            mgr.send_and_recv(build_frame(0x61, b"\x00\x07", 0x40), note="handshake_0x61")
        mgr.poll_status(include_bigdata=include_bigdata, note_prefix="poll", log_tx=True)
        return _full_state_payload({
            "ok": True,
            "raw_user_hex": mgr.last_user_raw_hex,
            "raw_big_hex": mgr.last_big_raw_hex,
        })
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/power")
def api_power(req: PowerReq):
    try:
        res = mgr.set_power(req.on)
        return _full_state_payload(res)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/control")
def api_control(req: ControlReq):
    try:
        # Route legacy self_cleaning toggle smoothly through SelfCleanManager!
        if "self_cleaning" in req.updates and len(req.updates) == 1:
            if req.updates["self_cleaning"]:
                return _full_state_payload(mgr.start_self_clean())
            return _full_state_payload(mgr.cancel_self_clean(mode="restore"))
        res = mgr.apply_control(req.updates)
        return _full_state_payload(res)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/timer")
def api_timer(req: TimerReq):
    if req.action == "CANCEL":
        mgr.active_timer = None
        mgr.log_event("TIMER", "", {"cancelled": True}, note="timer_cancel")
        return {"ok": True, "timer": None}
    hrs = max(0.5, min(24.0, round(req.hours * 2) / 2))
    trig = datetime.now() + timedelta(hours=hrs)
    mgr.active_timer = {
        "action": req.action,
        "hours": hrs,
        "trigger_ts": trig.timestamp(),
        "trigger_at": trig.strftime("%H:%M:%S"),
    }
    mgr.log_event("TIMER", "", mgr.active_timer, note=f"timer_set_{req.action}_{hrs}h")
    return {"ok": True, "timer": mgr.active_timer}


# ==============================================================================
# Remote Diff Scanner Endpoints
# ==============================================================================
@app.post("/api/diff/baseline")
def api_diff_baseline():
    res = api_poll(include_bigdata=True)
    if not res.get("ok"):
        return res
    mgr.baseline_user_hex = mgr.last_user_raw_hex
    mgr.baseline_big_hex = mgr.last_big_raw_hex
    return {"ok": True, "baseline_user_hex": mgr.baseline_user_hex, "baseline_big_hex": mgr.baseline_big_hex}


@app.post("/api/diff/step")
def api_diff_step():
    res = api_poll(include_bigdata=True)
    if not res.get("ok"):
        return res
    u_diffs = compute_byte_diffs(mgr.baseline_user_hex, mgr.last_user_raw_hex) if mgr.baseline_user_hex else []
    b_diffs = compute_byte_diffs(mgr.baseline_big_hex, mgr.last_big_raw_hex) if mgr.baseline_big_hex else []
    return _full_state_payload({
        "ok": True,
        "user_diffs": u_diffs,
        "big_diffs": b_diffs,
        "raw_user_hex": mgr.last_user_raw_hex,
        "raw_big_hex": mgr.last_big_raw_hex,
    })


@app.post("/api/diff/save")
def api_diff_save(req: DiffSaveReq):
    existing = json.loads(DIFF_FILE.read_text(encoding="utf-8")) if DIFF_FILE.exists() else []
    existing.append({"ts": datetime.now().isoformat(timespec="seconds"), **req.model_dump()})
    DIFF_FILE.write_text(json.dumps(existing, indent=2, ensure_ascii=False), encoding="utf-8")
    mgr.baseline_user_hex = mgr.last_user_raw_hex
    mgr.baseline_big_hex = mgr.last_big_raw_hex
    return {"ok": True, "count": len(existing)}


@app.get("/api/diff/list")
def api_diff_list():
    if not DIFF_FILE.exists():
        return {"items": []}
    return {"items": json.loads(DIFF_FILE.read_text(encoding="utf-8"))}


# ==============================================================================
# Logs, External ESP DHT & UI Endpoints
# ==============================================================================
@app.get("/api/logs")
def api_logs(limit: int = 50, filter: str = "all"):
    return {"logs": mgr.get_recent_logs(limit=limit, log_filter=filter)}


@app.delete("/api/logs")
def api_clear_logs():
    mgr.clear_logs()
    return {"ok": True}


@app.post("/api/ext_dht/ip")
def api_set_ext_dht_ip(ip: str = "192.168.1.29"):
    ext_dht_state["ip"] = ip.strip()
    return {"ok": True, "ip": ext_dht_state["ip"]}


@app.get("/api/ext_dht/logs")
def api_get_dht_logs(limit: int = 60):
    with _dht_lock:
        rows = list(reversed(list(_recent_dht_logs)[-limit:]))
    return {"rows": rows}


@app.get("/api/ext_dht/logs.csv")
def api_get_dht_logs_csv():
    header = "ts,s1_temp_c,s1_hum_pct,s1_err,s2_temp_c,s2_hum_pct,s2_err,haier_room_c,delta_s1_haier,delta_s2_haier,delta_s1_s2\n"
    if not DHT_LOG_FILE.exists():
        return PlainTextResponse(header, media_type="text/csv")
    out = [header]
    for ln in DHT_LOG_FILE.read_text(encoding="utf-8").splitlines():
        try:
            r = json.loads(ln)
            out.append(",".join("" if r.get(k) is None else str(r.get(k)) for k in [
                "ts", "s1_temp_c", "s1_hum_pct", "s1_err", "s2_temp_c", "s2_hum_pct", "s2_err",
                "haier_room_c", "delta_s1_haier", "delta_s2_haier", "delta_s1_s2",
            ]) + "\n")
        except Exception:
            pass
    return PlainTextResponse("".join(out), media_type="text/csv")


@app.delete("/api/ext_dht/logs")
def api_clear_dht_logs():
    with _dht_lock:
        _recent_dht_logs.clear()
    if DHT_LOG_FILE.exists():
        DHT_LOG_FILE.write_text("", encoding="utf-8")
    return {"ok": True}


@app.get("/api/table")
def api_table():
    return JSONResponse(PROTOCOL_TABLES)


@app.get("/", response_class=HTMLResponse)
def index():
    html_file = STATIC_DIR / "index.html"
    return html_file.read_text(encoding="utf-8")


if __name__ == "__main__":
    print("=== Haier AC + ESP32 DHT Studio ===")
    print("Local PC : http://127.0.0.1:8025")
    print("Mobile   : http://192.168.1.2:8025")
    uvicorn.run(app, host="0.0.0.0", port=8025)
