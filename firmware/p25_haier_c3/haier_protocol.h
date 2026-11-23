#pragma once
#include <Arduino.h>
#include <functional>

// ============================================================================
// Haier hOn (E++2.18) UART Protocol Engine for ESP32-C3 (HSU-15VQRA03T)
// Verified 1:1 with p25_esp32_haier_ac/protocol.py & cl_manager.py:
//   - UART: 9600 8N1 (default RX=GPIO 3, TX=GPIO 4, configurable in NVS)
//   - Frame: FF FF <len> <flag=0x40> 00 00 00 00 00 <type> <payload...> <chk> <crc_hi> <crc_lo>
//   - Byte stuffing: 0xFF -> 0xFF 0x55 anywhere after FF FF header
//   - Strict CRC-16/ARC + 8-bit Checksum verified single-0xFF recovery
//   - Single-frame 4D FE -> 7D 01 dual decode (Control+Sensors + BigData)
// ============================================================================

struct HaierControlState {
  bool valid = false;
  bool acPower = false;
  uint8_t modeRaw = 0x01;       // 0=AUTO, 1=COOL, 2=DRY, 3=HEALTHY_DRY, 4=HEAT, 5=ENERGY_SAVING, 6=FAN
  uint8_t fanRaw = 0x05;        // 1=HIGH, 2=MID, 3=LOW, 5=AUTO
  float targetTemp = 25.0f;     // 16.0..30.0 in 0.5 steps
  uint8_t vSwingRaw = 0x06;     // 0x00..0x0E
  uint8_t hSwingRaw = 0x00;     // 0x00..0x07
  uint8_t ecoLevel = 0;         // 0=OFF, 1=L1(0x28), 2=L2(0x30), 3=L3(0x38)
  bool displayStatus = true;    // b4 bit 1
  bool steriClean = false;      // b4 bit 7
  bool healthMode = false;      // b5 bit 1
  bool fastMode = false;        // b5 bit 3
  bool quietMode = false;       // b5 bit 4
  bool sleepMode = false;       // b5 bit 5
  bool lockRemote = false;      // b5 bit 6
  bool beeperMuted = false;     // b5 bit 7 (sticky)
  bool selfCleaning = false;    // b9 bit 4
  bool lightStatus = false;     // b9 bit 5
  uint8_t rawControl[12] = {0x08, 0x06, 0x21, 0x00, 0x02, 0x01, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00};

  // Sensors (from 6D 01 / 7D 01 bytes [14:36])
  bool hasSensors = false;
  float roomTempC = NAN;
  int roomHumPct = -1;
  int outdoorTempC = -999;
  uint8_t errorCode = 0;
  uint8_t opSource = 0; // 0=Other, 1=Remote, 2=Button, 3=ESP/UART
};

struct HaierBigData {
  bool valid = false;
  uint16_t powerW = 0;
  float indoorCoilC = NAN;
  int outdoorOutAirC = -999;
  int outdoorCoilC = -999;
  int outdoorInAirC = -999;
  int outdoorDefrostC = -999;
  uint8_t compFreqHz = 0;
  float compCurrentA = 0.0f;
  uint8_t outFanStatus = 0;
  uint8_t defrostStatus = 0;
  uint8_t compStatus = 0;
  uint8_t inFanStatus = 0;
  uint8_t fourWayStatus = 0;
  uint16_t eevOpen = 0;
};

struct HaierDeviceInfo {
  bool valid = false;
  String protocolVer;
  String softwareVer;
  String hardwareVer;
  String deviceName;
};

struct HaierSelfCleanTracker {
  bool active = false;
  String phase = "IDLE";          // IDLE, STARTING, FREEZING, WASHING, DRYING, COMPLETED
  String postAction = "restore_prev"; // restore_prev, power_off, dry_off_10m, stay_on
  unsigned long startedMs = 0;
  uint8_t offConfirmCount = 0;
  bool sawCompOn = false;
  float minCoilC = 999.0f;
  int peakPipeC = -999;
  uint16_t peakPowerW = 0;
  bool hasPreSnap = false;
  HaierControlState preSnap;
  bool unackedDone = false;
  String lastSummary = "";
  uint32_t completionSeq = 0;
};

class HaierAcController {
public:
  HaierControlState state;
  HaierBigData big;
  HaierDeviceInfo devInfo;
  HaierSelfCleanTracker cl;

  bool stickyBeeperMuted = false;
  bool smartEcoEnabled = false;
  float smartEcoThresholdC = 27.0f;
  bool simMode = false; // Debug AC Simulator (default OFF for real hardware safety)

  uint32_t txCount = 0;
  uint32_t rxOkCount = 0;
  uint32_t rxRecoveredSingleFF = 0;
  uint32_t rxErrorCount = 0;
  unsigned long lastRxMs = 0;
  uint16_t lastRttMs = 0;
  String lastRxHex;
  String lastNote;

  // Alarms (0x73 -> 0x74)
  int alarmCount = 0;
  String lastAlarmHex = "";

  // On-board countdown timer (TIME ON / TIME OFF / Dry&Off 10m)
  bool timerActive = false;
  bool timerTurnOn = false;
  unsigned long timerFireAtMs = 0;
  String timerLabel = "";

  std::function<void(const String&, const String&)> onLog = nullptr;

  void emitLog(const String& cat, const String& msg) {
    if (onLog) onLog(cat, msg);
  }

  int rxPin = 4;
  int txPin = 3;
  unsigned long coolStartedMs = 0;

  bool isCompWaitingDelay() const {
    if (!state.valid || !state.acPower || state.selfCleaning) return false;
    if (state.modeRaw == 6) return false; // FAN only
    if (big.valid && (big.compStatus == 1 || big.compFreqHz > 0)) return false;
    return (coolStartedMs > 0) && ((millis() - coolStartedMs) < 210000UL);
  }

  void begin(int rx, int tx) {
    rxPin = rx;
    txPin = tx;
    Serial1.end();
    delay(20);
    Serial1.begin(9600, SERIAL_8N1, rxPin, txPin);
    Serial1.setRxBufferSize(512);
    delay(30);
    while (Serial1.available()) Serial1.read();
  }

  bool isOnline() const {
    if (simMode) return true;
    return (lastRxMs > 0) && ((millis() - lastRxMs) < 75000UL);
  }

  void setSimMode(bool enable) {
    if (simMode == enable) return;
    simMode = enable;
    if (simMode) {
      devInfo.valid = true;
      devInfo.deviceName = "HSU-15VQRA03T (SIM)";
      devInfo.protocolVer = "E++2.18-SIM";
      state.valid = true;
      state.hasSensors = true;
      state.acPower = true;
      state.modeRaw = 1; // COOL
      state.fanRaw = 5;  // AUTO
      state.targetTemp = 25.0f;
      state.vSwingRaw = 6;
      state.hSwingRaw = 0;
      state.ecoLevel = 0;
      state.displayStatus = true;
      state.quietMode = false;
      state.fastMode = false;
      state.sleepMode = false;
      state.healthMode = false;
      state.lockRemote = false;
      state.selfCleaning = false;
      state.roomTempC = 30.5f;
      state.outdoorTempC = 33;
      state.opSource = 3;
      cl.active = false;
      cl.phase = "IDLE";
      updateSimPhysics();
      emitLog("SYS", "Debug AC Simulator ENABLED (Virtual HSU-15VQRA03T)");
    } else {
      lastRxMs = 0;
      state.valid = false;
      big.valid = false;
      devInfo.valid = false;
      devInfo.deviceName = "";
      devInfo.protocolVer = "";
      cl.active = false;
      cl.phase = "IDLE";
      emitLog("SYS", "Debug AC Simulator DISABLED (Real UART mode)");
    }
  }

  void updateSimPhysics() {
    if (!simMode) return;
    big.valid = true;
    rxOkCount++;
    lastRxMs = millis();

    if (cl.active && state.selfCleaning) {
      unsigned long sec = (millis() - cl.startedMs) / 1000UL;
      if (sec < 5) {
        cl.phase = "STARTING";
        big.compFreqHz = 30; big.powerW = 420; big.inFanStatus = 0; big.compStatus = 1; big.indoorCoilC = 24.0f;
      } else if (sec < 15) {
        cl.phase = "FREEZING";
        big.compFreqHz = 80; big.powerW = 880; big.inFanStatus = 0; big.compStatus = 1; big.indoorCoilC = 0.0f;
      } else if (sec < 25) {
        cl.phase = "WASHING";
        big.compFreqHz = 78; big.powerW = 1010; big.inFanStatus = 1; big.compStatus = 1; big.indoorCoilC = 18.5f;
      } else if (sec < 35) {
        cl.phase = "DRYING";
        big.compFreqHz = 0; big.powerW = 28; big.inFanStatus = 1; big.compStatus = 0; big.indoorCoilC = 26.0f;
      } else {
        cl.active = false;
        state.selfCleaning = false;
        cl.phase = "COMPLETED";
        pendingPostClAction = true;
      }
      big.outdoorOutAirC = (big.compFreqHz > 0) ? 94 : 42;
      big.compCurrentA = big.powerW / 225.0f;
      return;
    }

    if (!state.acPower) {
      big.powerW = 2; big.compFreqHz = 0; big.compCurrentA = 0.0f;
      big.compStatus = 0; big.inFanStatus = 0;
      big.indoorCoilC = state.roomTempC; big.outdoorOutAirC = 34;
    } else if (state.modeRaw == 6) { // FAN ONLY
      big.powerW = (state.fanRaw == 1) ? 38 : (state.fanRaw == 2) ? 26 : 18;
      big.compFreqHz = 0; big.compCurrentA = big.powerW / 225.0f;
      big.compStatus = 0; big.inFanStatus = 1;
      big.indoorCoilC = state.roomTempC; big.outdoorOutAirC = 34;
    } else {
      int baseW = (state.ecoLevel == 3) ? 350 : (state.ecoLevel == 2) ? 520 : (state.ecoLevel == 1) ? 690 : (state.fastMode ? 1120 : 890);
      int deltaW = (int)((25.0f - state.targetTemp) * 35.0f);
      big.powerW = (uint16_t)constrain(baseW + deltaW, 240, 1380);
      big.compFreqHz = (uint8_t)constrain(big.powerW / 12, 20, 88);
      big.compCurrentA = big.powerW / 225.0f;
      big.compStatus = 1; big.inFanStatus = 1;
      big.indoorCoilC = (state.ecoLevel == 3) ? 16.5f : 11.0f;
      big.outdoorOutAirC = 72 + (big.compFreqHz / 4);
    }
  }

  static uint16_t crc16Arc(const uint8_t* data, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
      crc ^= data[i];
      for (int b = 0; b < 8; b++) {
        crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
      }
    }
    return crc;
  }

  static size_t buildFrame(uint8_t frameType, const uint8_t* payload, size_t payloadLen, uint8_t* outBuf, size_t outMax, uint8_t flag = 0x40) {
    uint8_t body[128];
    uint8_t length = (uint8_t)(8 + payloadLen);
    if (length + 4 > sizeof(body)) return 0;

    body[0] = length;
    body[1] = flag;
    body[2] = 0; body[3] = 0; body[4] = 0; body[5] = 0; body[6] = 0;
    body[7] = frameType;
    if (payloadLen > 0) memcpy(&body[8], payload, payloadLen);

    uint8_t chk = 0;
    for (uint8_t i = 0; i < length; i++) chk = (uint8_t)(chk + body[i]);
    body[length] = chk;
    size_t rawLen = length + 1;
    if (flag & 0x40) {
      uint16_t c = crc16Arc(body, length);
      body[length + 1] = (uint8_t)((c >> 8) & 0xFF);
      body[length + 2] = (uint8_t)(c & 0xFF);
      rawLen += 2;
    }

    size_t pos = 0;
    if (pos + 2 > outMax) return 0;
    outBuf[pos++] = 0xFF;
    outBuf[pos++] = 0xFF;
    for (size_t i = 0; i < rawLen; i++) {
      if (pos + 2 > outMax) return 0;
      outBuf[pos++] = body[i];
      if (body[i] == 0xFF) outBuf[pos++] = 0x55;
    }
    return pos;
  }

  bool probeVersion() {
    if (simMode) return true;
    const uint8_t pl[2] = {0x00, 0x07};
    return sendAndRecv(0x61, pl, 2, 450, "probe_ver");
  }

  bool pollStatus() {
    if (simMode) {
      updateSimPhysics();
      return true;
    }
    const uint8_t plBig[2] = {0x4D, 0xFE};
    if (sendAndRecv(0x01, plBig, 2, 420, "poll_7D01")) {
      return true;
    }
    const uint8_t plUsr[2] = {0x4D, 0x01};
    return sendAndRecv(0x01, plUsr, 2, 420, "poll_6D01");
  }

  bool setPower(bool on) {
    if (simMode) {
      txCount++;
      state.acPower = on;
      if (!on) { state.selfCleaning = false; cl.active = false; }
      updateSimPhysics();
      return true;
    }
    uint8_t pl[4] = {0x5D, 0x01, 0x00, (uint8_t)(on ? 0x01 : 0x00)};
    bool ok = sendAndRecv(0x01, pl, 4, 480, on ? "power_on" : "power_off");
    if (!ok) {
      // Fallback via full 60 01 block
      HaierControlState next = state;
      next.acPower = on;
      next.beeperMuted = stickyBeeperMuted;
      ok = sendControlState(next, on ? "power_on_6001" : "power_off_6001");
    }
    return ok;
  }

  bool checkAlarms() {
    if (simMode) {
      txCount++;
      alarmCount = 0;
      emitLog("AC", "Alarms check (0x74 SIM): active=0 OK");
      return true;
    }
    const uint8_t pl[2] = {0x00, 0x00};
    return sendAndRecv(0x73, pl, 0, 480, "check_alarms");
  }

  void setCountdownTimer(bool turnOn, uint32_t seconds, const String& label) {
    timerActive = true;
    timerTurnOn = turnOn;
    timerFireAtMs = millis() + (seconds * 1000UL);
    timerLabel = label;
    emitLog("CMD", "Timer armed: " + label + " (" + String(seconds) + "s)");
  }

  void cancelCountdownTimer() {
    if (timerActive) {
      timerActive = false;
      timerLabel = "";
      emitLog("CMD", "Timer cancelled");
    }
  }

  bool startDryAndOff10m() {
    HaierControlState next = state;
    next.acPower = true;
    next.modeRaw = 0x06; // FAN
    next.fanRaw = 0x01;  // HIGH
    next.ecoLevel = 0;
    next.selfCleaning = false;
    bool ok = sendControlState(next, "dry_off_10m");
    if (ok) {
      setCountdownTimer(false, 600, "Dry & Off 10m");
    }
    return ok;
  }

  bool startSelfClean(const String& postAction) {
    if (postAction == "restore_prev" || postAction == "power_off" || postAction == "dry_off_10m" || postAction == "stay_on") {
      cl.postAction = postAction;
    }
    if (!cl.active && state.valid) {
      cl.preSnap = state;
      cl.preSnap.selfCleaning = false;
      cl.preSnap.steriClean = false;
      cl.hasPreSnap = true;
    }
    cl.offConfirmCount = 0;
    cl.sawCompOn = false;
    cl.minCoilC = 999.0f;
    cl.peakPipeC = -999;
    cl.peakPowerW = 0;
    cl.unackedDone = false;

    if (simMode) {
      txCount++;
      state.acPower = true;
      state.selfCleaning = true;
      cl.active = true;
      cl.startedMs = millis();
      cl.phase = "STARTING";
      updateSimPhysics();
      return true;
    }
    // Primary: 60 01 with acPower=true + selfCleaning=true (verified on HSU-15VQRA03T)
    HaierControlState next = state;
    next.acPower = true;
    next.selfCleaning = true;
    next.steriClean = false;
    next.fastMode = false;
    next.quietMode = false;
    next.sleepMode = false;
    next.ecoLevel = 0;
    bool ok = sendControlState(next, "cl_start_6001");
    if (!ok || !state.selfCleaning) {
      const uint8_t pl[4] = {0x5D, 0x0D, 0x00, 0x01};
      ok = sendAndRecv(0x01, pl, 4, 480, "cl_start_5D0D");
    }
    if (ok && state.selfCleaning) {
      cl.active = true;
      cl.startedMs = millis();
      cl.phase = "STARTING";
      emitLog("CL", "Self-Clean STARTED (prePower=" + String(cl.hasPreSnap && cl.preSnap.acPower ? "ON" : "OFF") + ", post=" + cl.postAction + ")");
      return true;
    }
    return false;
  }

  bool stopSelfClean() {
    bool ok = true;
    if (simMode) {
      txCount++;
      state.selfCleaning = false;
    } else {
      HaierControlState clr = state;
      clr.selfCleaning = false;
      clr.steriClean = false;
      ok = sendControlState(clr, "cl_stop_6001");
    }
    cl.active = false;
    cl.phase = "IDLE";
    cl.offConfirmCount = 0;
    if (cl.hasPreSnap && cl.postAction == "restore_prev") {
      restorePreCleanSnapshot();
      cl.lastSummary = cl.preSnap.acPower ? "ยกเลิก CL -> คืนสถานะแอร์ก่อนล้างแล้ว" : "ยกเลิก CL -> ปิดเครื่องกลับ (OFF) แล้ว";
    } else if (cl.postAction == "power_off") {
      setPower(false);
      cl.lastSummary = "ยกเลิก CL -> ปิดเครื่อง (OFF) แล้ว";
    } else {
      cl.lastSummary = "ยกเลิก CL แล้ว";
    }
    cl.unackedDone = true;
    cl.completionSeq++;
    emitLog("CL", cl.lastSummary);
    if (simMode) updateSimPhysics();
    return ok;
  }

  bool sendControlState(const HaierControlState& desired, const char* note = "ctrl_6001") {
    if (simMode) {
      txCount++;
      state = desired;
      state.valid = true;
      state.hasSensors = true;
      state.targetTemp = roundf(constrain(desired.targetTemp, 16.0f, 30.0f));
      if (state.modeRaw == 6 && state.fanRaw == 5) state.fanRaw = 2;
      if (state.selfCleaning || state.steriClean) state.acPower = true;
      if (!state.acPower || state.selfCleaning) { state.ecoLevel = 0; state.fastMode = false; state.quietMode = false; state.sleepMode = false; }
      if (state.ecoLevel > 0) { state.fastMode = false; state.quietMode = false; }
      if (state.fastMode || state.quietMode) { state.ecoLevel = 0; }
      state.beeperMuted = stickyBeeperMuted;
      lastNote = note;
      updateSimPhysics();
      return true;
    }
    uint8_t c[12];
    memcpy(c, state.valid ? state.rawControl : desired.rawControl, 12);

    // HSU-15VQRA03T accepts integer target temperatures (16..30C)
    float temp = roundf(constrain(desired.targetTemp, 16.0f, 30.0f));
    c[0] = (uint8_t)((int)temp - 16);
    c[1] = (uint8_t)((c[1] & 0xF0) | (desired.vSwingRaw & 0x0F));

    uint8_t mode = desired.modeRaw & 0x07;
    uint8_t fan = desired.fanRaw & 0x07;
    if (mode == 0x06 && fan == 0x05) fan = 0x02; // Fan-only cannot use AUTO fan
    c[2] = (uint8_t)(fan | ((mode) << 5));

    bool sc = desired.selfCleaning;
    bool steri = desired.steriClean;
    bool pwr = (sc || steri) ? true : desired.acPower;

    uint8_t b4 = 0;
    if (desired.displayStatus) b4 |= 0x02;
    if (steri) b4 |= 0x80;
    c[4] = b4;

    uint8_t eco = (pwr && !sc && mode != 0x06) ? desired.ecoLevel : 0;
    bool fast = (pwr && mode != 0x06 && !sc && eco == 0) ? desired.fastMode : false;
    bool quiet = (pwr && mode != 0x06 && !sc && eco == 0 && !fast) ? desired.quietMode : false;
    bool sleep = (pwr && !sc) ? desired.sleepMode : false;

    uint8_t b5 = 0;
    if (pwr) b5 |= 0x01;
    if (desired.healthMode) b5 |= 0x02;
    if (fast) b5 |= 0x08;
    if (quiet) b5 |= 0x10;
    if (sleep) b5 |= 0x20;
    if (desired.lockRemote) b5 |= 0x40;
    if (desired.beeperMuted || stickyBeeperMuted) b5 |= 0x80;
    c[5] = b5;

    uint8_t ecoBits = 0x00;
    if (eco == 1) ecoBits = 0x28;
    else if (eco == 2) ecoBits = 0x30;
    else if (eco == 3) ecoBits = 0x38;
    c[7] = (uint8_t)((c[7] & 0xC0) | ecoBits | (desired.hSwingRaw & 0x07));

    uint8_t b9 = (uint8_t)(c[9] & ~0x70);
    if (sc) b9 |= 0x10;
    c[9] = b9;

    uint8_t pl[14];
    pl[0] = 0x60;
    pl[1] = 0x01;
    memcpy(&pl[2], c, 12);
    return sendAndRecv(0x01, pl, 14, 420, note);
  }

  void runSmartEcoCheck(float externalRoomTempC) {
    if (!smartEcoEnabled || !state.valid) return;
    if (!state.acPower || state.selfCleaning || state.steriClean || state.modeRaw != 0x01) return;

    float refTemp = !isnan(externalRoomTempC) ? externalRoomTempC : state.roomTempC;
    if (isnan(refTemp)) return;

    if (refTemp <= smartEcoThresholdC && state.ecoLevel != 3) {
      HaierControlState next = state;
      next.ecoLevel = 3;
      next.fanRaw = 0x03; // LOW
      next.fastMode = false;
      next.quietMode = false;
      sendControlState(next, "smart_eco_L3");
    }
  }

  static const char* modeName(uint8_t m) {
    switch (m) {
      case 0: return "AUTO";
      case 1: return "COOL";
      case 2: return "DRY";
      case 3: return "HEALTHY_DRY";
      case 4: return "HEAT";
      case 5: return "ENERGY_SAVING";
      case 6: return "FAN";
      default: return "UNKNOWN";
    }
  }

  static const char* fanName(uint8_t f) {
    switch (f) {
      case 1: return "HIGH";
      case 2: return "MID";
      case 3: return "LOW";
      case 5: return "AUTO";
      default: return "AUTO";
    }
  }

  static const char* vSwingName(uint8_t v) {
    switch (v) {
      case 0x00: return "DEFAULT";
      case 0x01: return "HEALTH_UP";
      case 0x02: return "MAX_UP";
      case 0x03: return "HEALTH_DOWN";
      case 0x04: return "UP";
      case 0x06: return "CENTER";
      case 0x08: return "DOWN";
      case 0x0A: return "MAX_DOWN";
      case 0x0C: return "AUTO";
      case 0x0E: return "AUTO_SPECIAL";
      default: return "CENTER";
    }
  }

  static const char* hSwingName(uint8_t h) {
    switch (h) {
      case 0x00: return "CENTER";
      case 0x03: return "MAX_LEFT";
      case 0x04: return "LEFT";
      case 0x05: return "RIGHT";
      case 0x06: return "MAX_RIGHT";
      case 0x07: return "AUTO";
      default: return "CENTER";
    }
  }

  static const char* opSourceName(uint8_t s) {
    static const char* k[] = {"Other", "Remote", "Button", "ESP/UART"};
    return k[s & 0x03];
  }

private:
  bool sendAndRecv(uint8_t frameType, const uint8_t* payload, size_t payloadLen, uint32_t waitMs, const char* note) {
    uint8_t txBuf[160];
    size_t txLen = buildFrame(frameType, payload, payloadLen, txBuf, sizeof(txBuf), 0x40);
    if (txLen == 0) return false;

    while (Serial1.available()) Serial1.read();
    Serial1.write(txBuf, txLen);
    Serial1.flush();
    txCount++;
    lastNote = note;

    uint8_t rxBuf[256];
    size_t rxLen = 0;
    unsigned long t0 = millis();
    unsigned long lastByteMs = t0;

    while ((millis() - t0) < waitMs && rxLen < sizeof(rxBuf)) {
      bool gotBytes = false;
      while (Serial1.available() && rxLen < sizeof(rxBuf)) {
        rxBuf[rxLen++] = (uint8_t)Serial1.read();
        lastByteMs = millis();
        gotBytes = true;
      }
      if (rxLen >= 12 && (rxLen >= 48 || (!gotBytes && (millis() - lastByteMs) > 15))) {
        if (parseFrames(rxBuf, rxLen)) {
          lastRttMs = (uint16_t)min(9999UL, millis() - t0);
          return true;
        }
      }
      delay(2);
    }

    if (rxLen > 0) {
      if (parseFrames(rxBuf, rxLen)) {
        lastRttMs = (uint16_t)min(9999UL, millis() - t0);
        return true;
      }
      rxErrorCount++;
    }
    return false;
  }

  bool tryExtractAt(const uint8_t* raw, size_t n, size_t startBodyIdx, bool recovered, size_t& nextIdx) {
    uint8_t logical[140];
    size_t logLen = 0;
    size_t j = startBodyIdx;

    while (j < n && logLen < 3) {
      uint8_t b = raw[j];
      logical[logLen++] = b;
      j += (b == 0xFF && j + 1 < n && raw[j + 1] == 0x55) ? 2 : 1;
    }
    if (logLen < 3) return false;

    uint8_t length = logical[0];
    uint8_t flag = logical[1];
    if (length < 8 || length > 120) return false;
    if (recovered && !(flag & 0x40)) return false;

    size_t needTotal = length + ((flag & 0x40) ? 3 : 1);
    if (needTotal > sizeof(logical)) return false;

    while (j < n && logLen < needTotal) {
      uint8_t b = raw[j];
      logical[logLen++] = b;
      j += (b == 0xFF && j + 1 < n && raw[j + 1] == 0x55) ? 2 : 1;
    }
    if (logLen < needTotal) return false;

    uint8_t chkSum = 0;
    for (uint8_t k = 0; k < length; k++) chkSum = (uint8_t)(chkSum + logical[k]);
    bool chkOk = (chkSum == logical[length]);
    bool crcOk = true;
    if (flag & 0x40) {
      uint16_t expCrc = ((uint16_t)logical[length + 1] << 8) | logical[length + 2];
      crcOk = (crc16Arc(logical, length) == expCrc);
    }
    if (!chkOk || !crcOk) return false;

    rxOkCount++;
    if (recovered) rxRecoveredSingleFF++;
    lastRxMs = millis();

    // Save short hex preview
    char hexBuf[65];
    size_t hpos = 0;
    size_t previewLen = min((size_t)24, (size_t)length);
    for (size_t k = 0; k < previewLen && hpos + 2 < sizeof(hexBuf); k++) {
      hpos += snprintf(hexBuf + hpos, sizeof(hexBuf) - hpos, "%02X", logical[k]);
    }
    lastRxHex = String(hexBuf);

    uint8_t ftype = logical[7];
    const uint8_t* data = &logical[8];
    size_t dataLen = length - 8;

    handleDecodedFrame(ftype, data, dataLen);
    nextIdx = j;
    return true;
  }

  bool parseFrames(const uint8_t* raw, size_t n) {
    bool foundAny = false;
    size_t i = 0;
    while (i + 9 <= n) {
      size_t nextIdx = i + 1;
      if (raw[i] == 0xFF && i + 1 < n && raw[i + 1] == 0xFF && (i + 2 >= n || raw[i + 2] != 0x55)) {
        if (tryExtractAt(raw, n, i + 2, false, nextIdx)) {
          foundAny = true;
          i = nextIdx;
          continue;
        }
      } else if (raw[i] == 0xFF && i + 2 < n && raw[i + 1] != 0xFF && raw[i + 1] != 0x55 &&
                 raw[i + 1] >= 8 && raw[i + 1] <= 120 && (raw[i + 2] & 0x40)) {
        if (tryExtractAt(raw, n, i + 1, true, nextIdx)) {
          foundAny = true;
          i = nextIdx;
          continue;
        }
      }
      i++;
    }
    return foundAny;
  }

  static String cleanAscii(const uint8_t* p, size_t len) {
    String s;
    for (size_t i = 0; i < len && p[i] != 0; i++) {
      if (p[i] >= 32 && p[i] <= 126) s += (char)p[i];
    }
    return s;
  }

  void decodeUserBlock(const uint8_t* data, size_t len) {
    if (len < 14) return;
    bool prevValid = state.valid;
    bool prevPwr = state.acPower;
    uint8_t prevMode = state.modeRaw;
    bool prevCl = state.valid ? state.selfCleaning : false;
    HaierControlState prevStateCopy = state;

    const uint8_t* c = &data[2];
    memcpy(state.rawControl, c, 12);
    state.valid = true;
    state.targetTemp = 16.0f + c[0] + ((c[4] & 0x04) ? 0.5f : 0.0f);
    state.vSwingRaw = c[1] & 0x0F;
    state.fanRaw = c[2] & 0x07;
    state.modeRaw = (c[2] >> 5) & 0x07;
    state.displayStatus = (c[4] & 0x02) != 0;
    state.steriClean = (c[4] & 0x80) != 0;
    state.acPower = (c[5] & 0x01) != 0;
    state.healthMode = (c[5] & 0x02) != 0;
    state.fastMode = (c[5] & 0x08) != 0;
    state.quietMode = (c[5] & 0x10) != 0;
    state.sleepMode = (c[5] & 0x20) != 0;
    state.lockRemote = (c[5] & 0x40) != 0;
    state.beeperMuted = stickyBeeperMuted;
    state.hSwingRaw = c[7] & 0x07;
    uint8_t ecoBits = c[7] & 0x38;
    state.ecoLevel = (ecoBits == 0x28) ? 1 : (ecoBits == 0x30) ? 2 : (ecoBits == 0x38) ? 3 : 0;
    state.selfCleaning = (c[9] & 0x10) != 0;
    state.lightStatus = (c[9] & 0x20) != 0;

    if (len >= 20) {
      const uint8_t* s = &data[14];
      state.hasSensors = true;
      state.roomTempC = (s[0] > 0) ? (s[0] * 0.5f) : NAN;
      state.roomHumPct = (s[1] <= 100) ? (int)s[1] : -1;
      state.outdoorTempC = (s[2] > 0) ? ((int)s[2] - 64) : -999;
      state.errorCode = s[4];
      state.opSource = s[5] & 0x03;
    }

    // Track cooling mode start for 3-min compressor anti-short-cycle indicator
    if (state.acPower && state.modeRaw != 6 && (!prevPwr || prevMode == 6 || coolStartedMs == 0)) {
      coolStartedMs = millis();
    } else if (!state.acPower || state.modeRaw == 6) {
      coolStartedMs = 0;
    }

    // Observe Self-Clean lifecycle transitions (handles both Web UI & IR remote triggers)
    if (state.selfCleaning) {
      if (!cl.active) {
        if (prevValid && !prevCl) {
          cl.preSnap = prevStateCopy;
          cl.preSnap.selfCleaning = false;
          cl.preSnap.steriClean = false;
          cl.hasPreSnap = true;
        }
        cl.active = true;
        cl.startedMs = millis();
        cl.phase = "STARTING";
        cl.offConfirmCount = 0;
        cl.sawCompOn = false;
        cl.minCoilC = 999.0f;
        cl.peakPipeC = -999;
        cl.peakPowerW = 0;
        cl.unackedDone = false;
      }
      cl.offConfirmCount = 0;
    } else if (cl.active) {
      cl.offConfirmCount++;
      if (!state.acPower || cl.offConfirmCount >= 2) {
        cl.active = false;
        cl.phase = "COMPLETED";
        cl.offConfirmCount = 0;
        pendingPostClAction = true;
      }
    }
  }

  void decodeBigBlock(const uint8_t* b) {
    big.valid = true;
    big.powerW = ((uint16_t)b[0] << 8) | b[1];
    big.indoorCoilC = (b[2] > 0) ? (b[2] * 0.5f - 20.0f) : NAN;
    big.outdoorOutAirC = (b[3] > 0) ? ((int)b[3] - 64) : -999;
    big.outdoorCoilC = (b[4] > 0) ? ((int)b[4] - 64) : -999;
    big.outdoorInAirC = (b[5] > 0) ? ((int)b[5] - 64) : -999;
    big.outdoorDefrostC = (b[6] > 0) ? ((int)b[6] - 64) : -999;
    big.compFreqHz = b[7];
    big.compCurrentA = (((uint16_t)b[8] << 8) | b[9]) * 0.1f;
    big.outFanStatus = b[10] & 0x03;
    big.defrostStatus = (b[10] >> 2) & 0x03;
    big.compStatus = b[11] & 0x03;
    big.inFanStatus = (b[11] >> 2) & 0x03;
    big.fourWayStatus = (b[11] >> 4) & 0x03;
    big.eevOpen = ((uint16_t)b[12] << 8) | b[13];

    if (cl.active && state.selfCleaning) {
      if (!isnan(big.indoorCoilC) && big.indoorCoilC < cl.minCoilC) cl.minCoilC = big.indoorCoilC;
      if (big.outdoorOutAirC > cl.peakPipeC) cl.peakPipeC = big.outdoorOutAirC;
      if (big.powerW > cl.peakPowerW) cl.peakPowerW = big.powerW;
      bool compOn = (big.compStatus == 1) || (big.compFreqHz > 0);
      if (compOn && big.inFanStatus == 0) { cl.sawCompOn = true; cl.phase = "FREEZING"; }
      else if (compOn && big.inFanStatus > 0) { cl.sawCompOn = true; cl.phase = "WASHING"; }
      else if (!compOn && big.inFanStatus > 0 && (cl.sawCompOn || (millis() - cl.startedMs) > 60000UL)) cl.phase = "DRYING";
    }
  }

  void handleDecodedFrame(uint8_t ftype, const uint8_t* data, size_t dataLen) {
    if (ftype == 0x62 && dataLen >= 27) {
      devInfo.valid = true;
      devInfo.protocolVer = cleanAscii(&data[0], 8);
      devInfo.softwareVer = cleanAscii(&data[8], 8);
      devInfo.hardwareVer = cleanAscii(&data[19], 8);
      if (dataLen >= 36) devInfo.deviceName = cleanAscii(&data[28], 8);
    } else if (ftype == 0x02 && dataLen >= 14) {
      if (data[0] == 0x6D && data[1] == 0x01) {
        decodeUserBlock(data, dataLen);
      } else if (data[0] == 0x7D && data[1] == 0x01 && dataLen >= 50) {
        decodeUserBlock(data, 36);
        decodeBigBlock(&data[36]);
      } else if (data[0] == 0x6D && data[1] == 0xFE && dataLen >= 16) {
        decodeBigBlock(&data[2]);
      }
    } else if (ftype == 0x74) {
      alarmCount = 0;
      char hbuf[32];
      size_t hp = 0;
      size_t startIdx = (dataLen > 2) ? 2 : 0;
      for (size_t k = startIdx; k < dataLen; k++) {
        uint8_t b = data[k];
        for (int bit = 0; bit < 8; bit++) {
          if ((b >> bit) & 1) alarmCount++;
        }
        if (hp + 2 < sizeof(hbuf)) hp += snprintf(hbuf + hp, sizeof(hbuf) - hp, "%02X", b);
      }
      lastAlarmHex = String(hbuf);
      emitLog(alarmCount > 0 ? "ERR" : "AC", "Alarms check (0x74): active=" + String(alarmCount));
    }
  }

  void restorePreCleanSnapshot() {
    if (!cl.hasPreSnap) {
      setPower(false);
      return;
    }
    if (!cl.preSnap.acPower) {
      setPower(false);
    } else {
      sendControlState(cl.preSnap, "cl_post_restore");
    }
  }

public:
  bool pendingPostClAction = false;

  void processDeferredActions() {
    if (pendingPostClAction) {
      pendingPostClAction = false;
      delay(350);
      if (cl.postAction == "restore_prev") {
        restorePreCleanSnapshot();
        cl.lastSummary = (!cl.hasPreSnap || !cl.preSnap.acPower)
          ? "CL เสร็จสมบูรณ์ -> ปิดเครื่องกลับ (OFF) อัตโนมัติ"
          : "CL เสร็จสมบูรณ์ -> คืนค่าก่อนล้างอัตโนมัติแล้ว";
      } else if (cl.postAction == "power_off") {
        setPower(false);
        cl.lastSummary = "CL เสร็จสมบูรณ์ -> ปิดเครื่อง (OFF) อัตโนมัติ";
      } else if (cl.postAction == "dry_off_10m") {
        startDryAndOff10m();
        cl.lastSummary = "CL เสร็จสมบูรณ์ -> เป่าคอยล์ต่อ 10 น. แล้วปิด";
      } else {
        cl.lastSummary = "CL เสร็จสมบูรณ์ -> คงสถานะแอร์ไว้ตามเดิม";
      }
      cl.unackedDone = true;
      cl.completionSeq++;
      emitLog("CL", cl.lastSummary);
    }
    if (timerActive && (long)(millis() - timerFireAtMs) >= 0) {
      if (!state.selfCleaning) {
        bool turnOn = timerTurnOn;
        String lbl = timerLabel;
        timerActive = false;
        timerLabel = "";
        emitLog("CMD", "Timer fired (" + lbl + ") -> Power " + String(turnOn ? "ON" : "OFF"));
        setPower(turnOn);
      }
    }
  }
};
