#pragma once
#include <Arduino.h>
#include <LittleFS.h>
#include <time.h>
#include "haier_protocol.h"

// ============================================================================
// Advanced If-Else Automation Rule Engine & Presets for ESP32-C3 (v4.0)
// - Stores up to 8 customizable IF-THEN rule blocks in LittleFS (/rules_v4.bin)
// - Runs autonomously 24/7 on ESP32-C3 (even when PC/Phone browser is closed!)
// - Supports:
//     IF: Xiaomi Temp <= / >=, Xiaomi Humidity >= / <=, Local Time == HH:MM
//     THEN: Power (OFF/ON/Keep), Mode (COOL/DRY/FAN/AUTO), Target Temp (16..30),
//           Fan Speed (LOW/MID/HIGH/AUTO), Perf (Normal/Turbo/Quiet/ECO L1-L3)
// - Built-in cooldown (default 180s) & strict Self-Clean (CL) safety guard
// ============================================================================

struct __attribute__((packed)) AutoRule {
  uint8_t  id;
  uint8_t  enabled;       // 0/1
  uint8_t  condType;      // 0=temp<=, 1=temp>=, 2=hum>=, 3=hum<=, 4=time_hhmm
  uint8_t  onlyWhenOn;    // 1=require AC ON before firing (ignored for time_hhmm)
  int16_t  thresholdX10;  // Temp*10 (e.g. 265=26.5C), Hum*10 (700=70%), or HH*60+MM for time
  uint8_t  actPower;      // 0=OFF, 1=ON, 2=keep
  uint8_t  actMode;       // 0=AUTO, 1=COOL, 2=DRY, 6=FAN, 255=keep
  uint8_t  actTemp;       // 16..30 or 0=keep
  uint8_t  actFan;        // 1=HIGH, 2=MID, 3=LOW, 5=AUTO, 255=keep
  uint8_t  actPerf;       // 0=Normal, 1=Turbo, 2=Quiet, 3=ECO1, 4=ECO2, 5=ECO3, 255=keep
  uint16_t cooldownSec;   // Min seconds before same rule fires again (default 180)
  char     name[64];      // UTF-8 safe Thai/English label
};

class AutomationRulesEngine {
public:
  static const int MAX_RULES = 8;
  AutoRule rules[MAX_RULES];
  int ruleCount = 0;
  bool masterEnabled = true;

  void begin() {
    if (!loadFromFs()) {
      seedDefaults();
      saveToFs();
    }
  }

  void seedDefaults() {
    ruleCount = 0;
    addRuleInternal(false, 0, 1, 265, 1, 1, 26, 3, 5, 300, "เย็นแล้ว -> ECO L3 + Low");
    addRuleInternal(false, 2, 1, 720, 1, 2, 25, 5, 0, 300, "ชื้น >= 72% -> โหมด DRY");
    addRuleInternal(false, 3, 1, 550, 1, 6,  0, 2, 0, 300, "แห้ง <= 55% -> เป่าลม FAN");
    addRuleInternal(false, 4, 0, 360, 0, 255, 0, 255, 255, 90, "ปิดแอร์อัตโนมัติ 06:00");
  }

  void setMasterEnabled(bool en) {
    masterEnabled = en;
    saveToFs();
  }

  void resetDefaults() {
    seedDefaults();
    saveToFs();
  }

  int activeRuleCount() const {
    if (!masterEnabled) return 0;
    int c = 0;
    for (int i = 0; i < ruleCount; i++) if (rules[i].enabled) c++;
    return c;
  }

  bool upsertRule(int id, bool enabled, uint8_t condType, bool onlyWhenOn, int16_t thresholdX10,
                  uint8_t actPower, uint8_t actMode, uint8_t actTemp, uint8_t actFan,
                  uint8_t actPerf, uint16_t cooldownSec, const String& name) {
    int slot = -1;
    if (id > 0) {
      for (int i = 0; i < ruleCount; i++) {
        if (rules[i].id == id) { slot = i; break; }
      }
    }
    if (slot < 0) {
      if (ruleCount >= MAX_RULES) return false;
      slot = ruleCount++;
      uint8_t maxId = 0;
      for (int i = 0; i < slot; i++) if (rules[i].id > maxId) maxId = rules[i].id;
      rules[slot].id = maxId + 1;
      lastFiredMs[slot] = 0;
      lastFiredMin[slot] = -1;
    }
    AutoRule& r = rules[slot];
    r.enabled = enabled ? 1 : 0;
    r.condType = condType;
    r.onlyWhenOn = onlyWhenOn ? 1 : 0;
    r.thresholdX10 = thresholdX10;
    r.actPower = actPower;
    r.actMode = actMode;
    r.actTemp = actTemp;
    r.actFan = actFan;
    r.actPerf = actPerf;
    r.cooldownSec = (cooldownSec < 30) ? 30 : cooldownSec;
    copyUtf8Safe(r.name, sizeof(r.name), name.c_str());
    saveToFs();
    return true;
  }

  bool toggleRule(int id, bool enabled) {
    for (int i = 0; i < ruleCount; i++) {
      if (rules[i].id == id) {
        rules[i].enabled = enabled ? 1 : 0;
        saveToFs();
        return true;
      }
    }
    return false;
  }

  bool deleteRule(int id) {
    for (int i = 0; i < ruleCount; i++) {
      if (rules[i].id == id) {
        for (int j = i + 1; j < ruleCount; j++) {
          rules[j - 1] = rules[j];
          lastFiredMs[j - 1] = lastFiredMs[j];
          lastFiredMin[j - 1] = lastFiredMin[j];
        }
        ruleCount--;
        saveToFs();
        return true;
      }
    }
    return false;
  }

  // Evaluate enabled rules (call every ~5-10s in loop)
  void evaluate(HaierAcController& ac, bool xOk, float xTempC, float xHumPct) {
    if (!masterEnabled || !ac.state.valid || ac.state.selfCleaning || ac.state.steriClean) return;

    float refTemp = (xOk && !isnan(xTempC)) ? xTempC : ac.state.roomTempC;
    float refHum = (xOk && !isnan(xHumPct)) ? xHumPct : NAN;

    time_t nowUtc = time(nullptr);
    bool hasNtp = (nowUtc > 1700000000);
    int curLocalMin = -1;
    if (hasNtp) {
      time_t loc = nowUtc + 7L * 3600L;
      struct tm t;
      gmtime_r(&loc, &t);
      curLocalMin = t.tm_hour * 60 + t.tm_min;
    }

    unsigned long nowMs = millis();
    for (int i = 0; i < ruleCount; i++) {
      const AutoRule& r = rules[i];
      if (!r.enabled) continue;
      if (r.condType != 4 && r.onlyWhenOn && !ac.state.acPower) continue;
      if (lastFiredMs[i] > 0 && (nowMs - lastFiredMs[i]) < (unsigned long)r.cooldownSec * 1000UL) continue;

      bool matched = false;
      if (r.condType == 0 && !isnan(refTemp)) matched = (refTemp <= (r.thresholdX10 / 10.0f));
      else if (r.condType == 1 && !isnan(refTemp)) matched = (refTemp >= (r.thresholdX10 / 10.0f));
      else if (r.condType == 2 && !isnan(refHum)) matched = (refHum >= (r.thresholdX10 / 10.0f));
      else if (r.condType == 3 && !isnan(refHum)) matched = (refHum <= (r.thresholdX10 / 10.0f));
      else if (r.condType == 4 && curLocalMin >= 0) {
        if (curLocalMin == r.thresholdX10 && lastFiredMin[i] != curLocalMin) {
          matched = true;
          lastFiredMin[i] = curLocalMin;
        }
      }
      if (!matched) continue;

      // Check if desired state already matches current AC state (avoid redundant UART sends!)
      if (r.actPower == 0) {
        if (!ac.state.acPower) continue;
        lastFiredMs[i] = nowMs;
        ac.setPower(false);
        ac.emitLog("RULE", String("Rule #") + r.id + " [" + r.name + "] -> Power OFF");
        return; // Max 1 rule action per cycle
      }

      HaierControlState next = ac.state;
      if (r.actPower == 1) next.acPower = true;
      if (r.actMode != 255) next.modeRaw = r.actMode;
      if (r.actTemp >= 16 && r.actTemp <= 30) next.targetTemp = (float)r.actTemp;
      if (r.actFan != 255) next.fanRaw = r.actFan;
      if (r.actPerf != 255) {
        next.fastMode = (r.actPerf == 1);
        next.quietMode = (r.actPerf == 2);
        next.ecoLevel = (r.actPerf >= 3 && r.actPerf <= 5) ? (r.actPerf - 2) : 0;
      }

      bool changed = (next.acPower != ac.state.acPower) ||
                     (next.modeRaw != ac.state.modeRaw) ||
                     ((int)roundf(next.targetTemp) != (int)roundf(ac.state.targetTemp)) ||
                     (next.fanRaw != ac.state.fanRaw) ||
                     (next.fastMode != ac.state.fastMode) ||
                     (next.quietMode != ac.state.quietMode) ||
                     (next.ecoLevel != ac.state.ecoLevel);
      if (!changed) continue;

      lastFiredMs[i] = nowMs;
      ac.sendControlState(next, "auto_rule");
      ac.emitLog("RULE", String("Rule #") + r.id + " [" + r.name + "] fired -> " +
                         HaierAcController::modeName(next.modeRaw) + " " + String((int)next.targetTemp) + "C");
      return;
    }
  }

  String toJson() const {
    String j = "{\"master\":" + String(masterEnabled ? "true" : "false") + ",\"rules\":[";
    for (int i = 0; i < ruleCount; i++) {
      const AutoRule& r = rules[i];
      if (i > 0) j += ",";
      j += "{\"id\":" + String(r.id) + ",\"en\":" + String(r.enabled ? "true" : "false");
      j += ",\"cond\":" + String(r.condType) + ",\"on_only\":" + String(r.onlyWhenOn ? "true" : "false");
      j += ",\"th\":" + String(r.thresholdX10) + ",\"pwr\":" + String(r.actPower);
      j += ",\"mode\":" + String(r.actMode) + ",\"temp\":" + String(r.actTemp);
      j += ",\"fan\":" + String(r.actFan) + ",\"perf\":" + String(r.actPerf);
      j += ",\"cd\":" + String(r.cooldownSec) + ",\"name\":\"";
      for (size_t k = 0; k < strlen(r.name); k++) {
        uint8_t b = (uint8_t)r.name[k];
        if (b == '"' || b == '\\') j += '\\';
        if (b >= 32) j += (char)b;
      }
      j += "\"}";
    }
    j += "]}";
    return j;
  }

private:
  unsigned long lastFiredMs[MAX_RULES] = {0};
  int lastFiredMin[MAX_RULES] = {-1, -1, -1, -1, -1, -1, -1, -1};

  static void copyUtf8Safe(char* dst, size_t dstSize, const char* src) {
    if (!dst || dstSize == 0) return;
    memset(dst, 0, dstSize);
    if (!src) return;
    size_t n = strnlen(src, dstSize - 1);
    memcpy(dst, src, n);
    dst[n] = '\0';
    // If truncated mid UTF-8 sequence, trim trailing incomplete bytes cleanly
    if (src[n] != '\0' && n > 0) {
      size_t i = n;
      while (i > 0 && ((uint8_t)dst[i - 1] & 0xC0) == 0x80) i--;
      if (i > 0) {
        uint8_t lead = (uint8_t)dst[i - 1];
        size_t need = (lead >= 0xF0) ? 4 : (lead >= 0xE0) ? 3 : (lead >= 0xC0) ? 2 : 1;
        if ((n - (i - 1)) < need) dst[i - 1] = '\0';
      }
    }
  }

  void addRuleInternal(bool en, uint8_t cond, uint8_t onOnly, int16_t th,
                       uint8_t pwr, uint8_t mode, uint8_t temp, uint8_t fan,
                       uint8_t perf, uint16_t cd, const char* nm) {
    if (ruleCount >= MAX_RULES) return;
    AutoRule& r = rules[ruleCount];
    memset(&r, 0, sizeof(AutoRule));
    r.id = ruleCount + 1;
    r.enabled = en ? 1 : 0;
    r.condType = cond;
    r.onlyWhenOn = onOnly;
    r.thresholdX10 = th;
    r.actPower = pwr;
    r.actMode = mode;
    r.actTemp = temp;
    r.actFan = fan;
    r.actPerf = perf;
    r.cooldownSec = cd;
    copyUtf8Safe(r.name, sizeof(r.name), nm);
    ruleCount++;
  }

  bool loadFromFs() {
    if (!LittleFS.exists("/rules_v4.bin")) return false;
    File f = LittleFS.open("/rules_v4.bin", "r");
    if (!f) return false;
    uint8_t hdr[2];
    if (f.read(hdr, 2) != 2 || hdr[0] != 0xA5) { f.close(); return false; }
    masterEnabled = (hdr[1] == 1);
    ruleCount = 0;
    AutoRule r;
    while (ruleCount < MAX_RULES && f.read((uint8_t*)&r, sizeof(r)) == sizeof(r)) {
      rules[ruleCount++] = r;
    }
    f.close();
    return ruleCount > 0;
  }

  void saveToFs() {
    File f = LittleFS.open("/rules_v4.bin", "w");
    if (!f) return;
    uint8_t hdr[2] = {0xA5, (uint8_t)(masterEnabled ? 1 : 0)};
    f.write(hdr, 2);
    for (int i = 0; i < ruleCount; i++) {
      f.write((const uint8_t*)&rules[i], sizeof(AutoRule));
    }
    f.close();
  }
};
