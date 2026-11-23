#pragma once
#include <Arduino.h>
#include <LittleFS.h>
#include <time.h>

// ============================================================================
// Multi-Sensor Time-Series Climate, Power & Energy Analytics Store v4.0
//   - Tier 1 (/hist15_v4.bin)  : 15-min buckets, 7 days (672 x 20B = 13.4 KB)
//     Stores ALL sensors: Xiaomi Temp/Hum, AC Room, Outdoor, Indoor Coil,
//     Discharge Pipe, Comp Freq Hz, Avg Power W, Target Temp, Mode/Flags
//   - Tier 2 (/histday_v4.bin) : Daily summary, up to 3 years (1095 x 32B = 35 KB)
//     Stores Min/Max/Avg Temp & Hum, AC ON min, Real Integrated Wh, 24h Mask
//   - Tier 3 (/sessions_v4.bin): Recent 24 AC ON->OFF usage sessions (384 B)
//   - Supports Day (0..6d ago), Week (7d), Month (30d), Year (12m),
//     Single-Day Delete, Clear All (clean RAM reset!), and CSV Export!
// ============================================================================

struct __attribute__((packed)) Hist15mRec {
  uint32_t epoch;
  int16_t  xTempX100;       // Xiaomi room temp x100 (INT16_MIN = invalid)
  uint16_t xHumX100;        // Xiaomi hum x100 (0xFFFF = invalid)
  int16_t  acRoomX10;       // AC room temp x10 (INT16_MIN = invalid)
  int16_t  outTempX10;      // Outdoor ambient x10 (INT16_MIN = invalid)
  int16_t  coilX10;         // Indoor coil temp x10 (INT16_MIN = invalid)
  int16_t  pipeC;           // Comp discharge pipe C (INT16_MIN = invalid)
  uint16_t avgPowerW;       // Avg AC power W
  uint8_t  avgFreqHz;       // Avg comp freq Hz
  uint8_t  targetAndFlags;  // bits0..4: target-16 (31=OFF), bit5:acOn, bit6:turbo, bit7:eco
};

struct __attribute__((packed)) HistDayRec {
  uint16_t dayIndex;        // days since 2020-01-01 (local GMT+7)
  int16_t  tMinX100;
  int16_t  tMaxX100;
  int16_t  tAvgX100;
  uint16_t hMinX100;
  uint16_t hMaxX100;
  uint16_t hAvgX100;
  uint16_t acOnMinutes;
  uint32_t energyWh;        // Real integrated Watt-hours
  uint32_t activeHoursMask; // bit 0..23 = local hour AC was ON
  uint16_t avgOnWatts;      // Avg power while ON
  int16_t  peakPipeC;       // Peak discharge pipe temp
  int16_t  minCoilX10;      // Min indoor coil temp x10
  uint16_t reserved;
};

struct __attribute__((packed)) AcSessionRec {
  uint32_t startEpoch;
  uint32_t endEpoch;
  uint32_t energyWh;
  uint8_t  modeRaw;
  uint8_t  targetTemp;
  uint16_t avgPowerW;
};

class ClimateHistoryStore {
public:
  static const int MAX_15M = 672;     // 7 days on Flash
  static const int RAM_15M = 96;      // Last 24h in RAM (1.92 KB)
  static const int RAM_DAYS = 60;     // Last 60 days in RAM (1.92 KB)
  static const int MAX_SESSIONS = 24; // Last 24 ON->OFF sessions (384 B)
  static const uint32_t EPOCH_2020 = 1577836800UL;

  float tariffBahtPerKwh = 4.18f;

  void begin() {
    // Clean legacy v3 files if present to reclaim LittleFS space
    if (LittleFS.exists("/hist_15m.bin")) LittleFS.remove("/hist_15m.bin");
    if (LittleFS.exists("/hist_day.bin")) LittleFS.remove("/hist_day.bin");
    loadFromFs();
  }

  static uint16_t epochToDayIndex(uint32_t utcEpoch) {
    int64_t localSec = (int64_t)utcEpoch + 7L * 3600L - (int64_t)EPOCH_2020;
    return (localSec > 0) ? (uint16_t)(localSec / 86400L) : 0;
  }

  static uint8_t epochToLocalHour(uint32_t utcEpoch) {
    return (uint8_t)(((utcEpoch + 7UL * 3600UL) % 86400UL) / 3600UL);
  }

  static String dayIndexToDateStr(uint16_t dayIdx) {
    time_t t = (time_t)(EPOCH_2020 + (uint32_t)dayIdx * 86400UL);
    struct tm tmVal;
    gmtime_r(&t, &tmVal);
    char buf[16];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d", tmVal.tm_year + 1900, tmVal.tm_mon + 1, tmVal.tm_mday);
    return String(buf);
  }

  static String epochToLocalTimeStr(uint32_t utcEpoch, bool withDate = false) {
    time_t t = (time_t)(utcEpoch + 7UL * 3600UL);
    struct tm tmVal;
    gmtime_r(&t, &tmVal);
    char buf[24];
    if (withDate) {
      snprintf(buf, sizeof(buf), "%02d/%02d %02d:%02d", tmVal.tm_mon + 1, tmVal.tm_mday, tmVal.tm_hour, tmVal.tm_min);
    } else {
      snprintf(buf, sizeof(buf), "%02d:%02d", tmVal.tm_hour, tmVal.tm_min);
    }
    return String(buf);
  }

  void observe(
      bool xOk, float xTempC, float xHumPct,
      bool acOnline, bool acPower, uint8_t modeRaw, float targetTemp,
      bool fastMode, uint8_t ecoLevel,
      float acRoomC, int outTempC, float coilC, int pipeC,
      uint16_t powerW, uint8_t freqHz) {

    bool hasAnyTemp = (xOk && !isnan(xTempC)) || (acOnline && !isnan(acRoomC));
    if (!hasAnyTemp && !acOnline) return;

    time_t nowUtc = time(nullptr);
    bool hasNtp = (nowUtc > 1700000000);
    uint32_t nowTs = hasNtp ? (uint32_t)nowUtc : (1763900000UL + (millis() / 1000UL));
    uint32_t bucket15 = nowTs / 900UL;
    uint16_t dayIdx = epochToDayIndex(nowTs);
    uint8_t localHr = epochToLocalHour(nowTs);

    unsigned long nowMs = millis();
    float dtSec = (lastObsMs > 0 && (nowMs - lastObsMs) < 180000UL) ? ((nowMs - lastObsMs) / 1000.0f) : 10.0f;
    lastObsMs = nowMs;

    uint16_t effPowerW = (acOnline && acPower) ? powerW : 0;
    float deltaWh = (effPowerW * dtSec) / 3600.0f;

    // Track AC ON -> OFF sessions
    observeSession(nowTs, acOnline && acPower, modeRaw, (uint8_t)roundf(constrain(targetTemp, 16.0f, 30.0f)), effPowerW, deltaWh);

    // 15-min bucket rollover
    if (curBucket15 != bucket15) {
      if (curBucketCount > 0) commitCurrentBucket();
      resetBucketAccum(bucket15);
    }

    if (xOk && !isnan(xTempC)) { curSumXT += (int32_t)roundf(xTempC * 100.0f); curCntXT++; }
    if (xOk && !isnan(xHumPct)) { curSumXH += (uint32_t)constrain((int)roundf(xHumPct * 100.0f), 0, 10000); curCntXH++; }
    if (acOnline && !isnan(acRoomC)) { curSumAcR += (int32_t)roundf(acRoomC * 10.0f); curCntAcR++; }
    if (acOnline && outTempC > -100) { curSumOut += (int32_t)(outTempC * 10); curCntOut++; }
    if (acOnline && !isnan(coilC)) { curSumCoil += (int32_t)roundf(coilC * 10.0f); curCntCoil++; }
    if (acOnline && pipeC > -100) { curSumPipe += pipeC; curCntPipe++; }
    if (acOnline) {
      curSumW += effPowerW;
      curSumHz += (acPower ? freqHz : 0);
      curCntPwr++;
      if (acPower) curAcOnTicks++;
      uint8_t tOff = acPower ? (uint8_t)constrain((int)roundf(targetTemp) - 16, 0, 14) : 31;
      curTargetFlags = (tOff & 0x1F) | (acPower ? 0x20 : 0) | (fastMode ? 0x40 : 0) | (ecoLevel > 0 ? 0x80 : 0);
    }
    curBucketCount++;

    // Update Today's Daily Summary (prefer Xiaomi room temp, fallback to AC room temp)
    float refTemp = (xOk && !isnan(xTempC)) ? xTempC : acRoomC;
    int16_t t100 = !isnan(refTemp) ? (int16_t)roundf(refTemp * 100.0f) : INT16_MIN;
    uint16_t h100 = (xOk && !isnan(xHumPct)) ? (uint16_t)constrain((int)roundf(xHumPct * 100.0f), 0, 10000) : 0xFFFF;
    int16_t coil10 = (acOnline && !isnan(coilC)) ? (int16_t)roundf(coilC * 10.0f) : INT16_MIN;
    updateTodaySummary(dayIdx, localHr, dtSec, t100, h100, acOnline && acPower, effPowerW, deltaWh, pipeC, coil10);

    if (!firstBootCommitted && curBucketCount >= 1) {
      firstBootCommitted = true;
      commitCurrentBucket();
    }
  }

  void flushCurrentDay() {
    if (countDays > 0) persistLatestDay(ramDays[countDays - 1]);
  }

  void clearAll() {
    count15m = 0;
    head15m = 0;
    countDays = 0;
    sessionCount = 0;
    sessionHead = 0;
    sessActive = false;
    firstBootCommitted = false;
    curBucket15 = 0;
    resetBucketAccum(0);
    accDayIdx = 0;
    accDaySumT = 0; accDaySumH = 0; accDayCntT = 0; accDayCntH = 0;
    accDayOnSec = 0; accDayWh = 0.0f; accDaySumOnW = 0; accDayCntOnW = 0;
    lastObsMs = 0;
    if (LittleFS.exists("/hist15_v4.bin")) LittleFS.remove("/hist15_v4.bin");
    if (LittleFS.exists("/histday_v4.bin")) LittleFS.remove("/histday_v4.bin");
    if (LittleFS.exists("/sessions_v4.bin")) LittleFS.remove("/sessions_v4.bin");
  }

  // Delete a single day cleanly (both daily summary and 15-min slots for that day)
  bool deleteDay(uint16_t targetDayIdx) {
    bool removedAny = false;
    // 1. Remove from RAM daily array
    int newDayCnt = 0;
    for (int i = 0; i < countDays; i++) {
      if (ramDays[i].dayIndex == targetDayIdx) {
        removedAny = true;
        continue;
      }
      ramDays[newDayCnt++] = ramDays[i];
    }
    countDays = newDayCnt;
    if (accDayIdx == targetDayIdx) {
      accDayIdx = 0;
      accDaySumT = 0; accDaySumH = 0; accDayCntT = 0; accDayCntH = 0;
      accDayOnSec = 0; accDayWh = 0.0f; accDaySumOnW = 0; accDayCntOnW = 0;
    }

    // Rewrite /histday_v4.bin without targetDayIdx
    if (LittleFS.exists("/histday_v4.bin")) {
      File fin = LittleFS.open("/histday_v4.bin", "r");
      File fout = LittleFS.open("/histday_v4.tmp", "w");
      if (fin && fout) {
        HistDayRec d;
        while (fin.read((uint8_t*)&d, sizeof(d)) == sizeof(d)) {
          if (d.dayIndex == targetDayIdx) { removedAny = true; continue; }
          fout.write((const uint8_t*)&d, sizeof(d));
        }
      }
      if (fin) fin.close();
      if (fout) fout.close();
      LittleFS.remove("/histday_v4.bin");
      LittleFS.rename("/histday_v4.tmp", "/histday_v4.bin");
    }

    // 2. Remove matching 15-min slots from /hist15_v4.bin and reload RAM ring
    if (LittleFS.exists("/hist15_v4.bin")) {
      File fin = LittleFS.open("/hist15_v4.bin", "r");
      File fout = LittleFS.open("/hist15_v4.tmp", "w");
      if (fin && fout) {
        Hist15mRec r;
        while (fin.read((uint8_t*)&r, sizeof(r)) == sizeof(r)) {
          if (epochToDayIndex(r.epoch) == targetDayIdx) { removedAny = true; continue; }
          fout.write((const uint8_t*)&r, sizeof(r));
        }
      }
      if (fin) fin.close();
      if (fout) fout.close();
      LittleFS.remove("/hist15_v4.bin");
      LittleFS.rename("/hist15_v4.tmp", "/hist15_v4.bin");
    }
    if (curBucket15 > 0 && epochToDayIndex(curBucketEpoch) == targetDayIdx) {
      resetBucketAccum(0);
      curBucket15 = 0;
    }
    // Reload 15m RAM ring cleanly
    count15m = 0; head15m = 0;
    load15mFromFs();
    return removedAny;
  }

  String buildHistoryJson(const String& range, int dayOffset,
                          bool xOk, float xT, float xH,
                          bool acOn, float acR, int outT, float coilC, int pipeC,
                          uint16_t liveW, uint8_t liveHz, float liveTarget) const {
    if (range == "week")  return buildDaysJson(7);
    if (range == "month") return buildDaysJson(30);
    if (range == "year")  return buildYearJson();
    return buildDayJson(dayOffset, xOk, xT, xH, acOn, acR, outT, coilC, pipeC, liveW, liveHz, liveTarget);
  }

  String buildCsv() const {
    String out = "date,t_min_c,t_max_c,t_avg_c,h_min_pct,h_max_pct,h_avg_pct,ac_on_min,energy_kwh,cost_thb,avg_on_w,peak_pipe_c,min_coil_c\n";
    for (int i = 0; i < countDays; i++) {
      const HistDayRec& d = ramDays[i];
      float kwh = d.energyWh / 1000.0f;
      float cost = kwh * tariffBahtPerKwh;
      out += dayIndexToDateStr(d.dayIndex) + ",";
      out += (d.tMinX100 != INT16_MIN ? String(d.tMinX100 / 100.0f, 1) : "") + ",";
      out += (d.tMaxX100 != INT16_MIN ? String(d.tMaxX100 / 100.0f, 1) : "") + ",";
      out += (d.tAvgX100 != INT16_MIN ? String(d.tAvgX100 / 100.0f, 1) : "") + ",";
      out += (d.hMinX100 != 0xFFFF ? String(d.hMinX100 / 100.0f, 0) : "") + ",";
      out += (d.hMaxX100 != 0xFFFF ? String(d.hMaxX100 / 100.0f, 0) : "") + ",";
      out += (d.hAvgX100 != 0xFFFF ? String(d.hAvgX100 / 100.0f, 0) : "") + ",";
      out += String(d.acOnMinutes) + ",";
      out += String(kwh, 3) + ",";
      out += String(cost, 2) + ",";
      out += String(d.avgOnWatts) + ",";
      out += (d.peakPipeC > -100 ? String(d.peakPipeC) : "") + ",";
      out += (d.minCoilX10 != INT16_MIN ? String(d.minCoilX10 / 10.0f, 1) : "") + "\n";
    }
    return out;
  }

private:
  Hist15mRec ram15m[RAM_15M];
  int head15m = 0;
  int count15m = 0;

  HistDayRec ramDays[RAM_DAYS];
  int countDays = 0;

  AcSessionRec ramSessions[MAX_SESSIONS];
  int sessionHead = 0;
  int sessionCount = 0;

  // Live AC session tracker
  bool sessActive = false;
  uint32_t sessStartEpoch = 0;
  float sessWh = 0.0f;
  uint64_t sessSumW = 0;
  uint32_t sessCntW = 0;
  uint8_t sessMode = 1;
  uint8_t sessTarget = 25;

  // 15m bucket accumulators
  uint32_t curBucket15 = 0;
  uint32_t curBucketEpoch = 0;
  int64_t  curSumXT = 0; int curCntXT = 0;
  uint64_t curSumXH = 0; int curCntXH = 0;
  int64_t  curSumAcR = 0; int curCntAcR = 0;
  int64_t  curSumOut = 0; int curCntOut = 0;
  int64_t  curSumCoil = 0; int curCntCoil = 0;
  int64_t  curSumPipe = 0; int curCntPipe = 0;
  uint64_t curSumW = 0; uint32_t curSumHz = 0; int curCntPwr = 0;
  int curAcOnTicks = 0;
  uint8_t curTargetFlags = 31;
  int curBucketCount = 0;
  bool firstBootCommitted = false;

  // Daily running accumulators
  uint16_t accDayIdx = 0;
  int64_t  accDaySumT = 0; uint32_t accDayCntT = 0;
  uint64_t accDaySumH = 0; uint32_t accDayCntH = 0;
  float    accDayOnSec = 0.0f;
  float    accDayWh = 0.0f;
  uint64_t accDaySumOnW = 0; uint32_t accDayCntOnW = 0;
  unsigned long lastObsMs = 0;

  void resetBucketAccum(uint32_t b15) {
    curBucket15 = b15;
    curBucketEpoch = b15 * 900UL;
    curSumXT = 0; curCntXT = 0;
    curSumXH = 0; curCntXH = 0;
    curSumAcR = 0; curCntAcR = 0;
    curSumOut = 0; curCntOut = 0;
    curSumCoil = 0; curCntCoil = 0;
    curSumPipe = 0; curCntPipe = 0;
    curSumW = 0; curSumHz = 0; curCntPwr = 0;
    curAcOnTicks = 0;
    curTargetFlags = 31;
    curBucketCount = 0;
  }

  void load15mFromFs() {
    if (!LittleFS.exists("/hist15_v4.bin")) return;
    File f = LittleFS.open("/hist15_v4.bin", "r");
    if (!f) return;
    size_t nRecs = f.size() / sizeof(Hist15mRec);
    size_t skip = (nRecs > RAM_15M) ? (nRecs - RAM_15M) : 0;
    if (skip > 0) f.seek(skip * sizeof(Hist15mRec));
    Hist15mRec r;
    while (f.read((uint8_t*)&r, sizeof(r)) == sizeof(r)) {
      if (count15m > 0) {
        int prevIdx = (head15m - 1 + RAM_15M) % RAM_15M;
        if (ram15m[prevIdx].epoch == r.epoch) {
          ram15m[prevIdx] = r;
          continue;
        }
      }
      ram15m[head15m] = r;
      head15m = (head15m + 1) % RAM_15M;
      if (count15m < RAM_15M) count15m++;
    }
    f.close();
  }

  void loadFromFs() {
    load15mFromFs();
    if (LittleFS.exists("/histday_v4.bin")) {
      File f = LittleFS.open("/histday_v4.bin", "r");
      if (f) {
        size_t nRecs = f.size() / sizeof(HistDayRec);
        size_t skip = (nRecs > RAM_DAYS) ? (nRecs - RAM_DAYS) : 0;
        if (skip > 0) f.seek(skip * sizeof(HistDayRec));
        HistDayRec d;
        while (countDays < RAM_DAYS && f.read((uint8_t*)&d, sizeof(d)) == sizeof(d)) {
          ramDays[countDays++] = d;
        }
        f.close();
      }
    }
    if (LittleFS.exists("/sessions_v4.bin")) {
      File f = LittleFS.open("/sessions_v4.bin", "r");
      if (f) {
        AcSessionRec s;
        while (f.read((uint8_t*)&s, sizeof(s)) == sizeof(s)) {
          ramSessions[sessionHead] = s;
          sessionHead = (sessionHead + 1) % MAX_SESSIONS;
          if (sessionCount < MAX_SESSIONS) sessionCount++;
        }
        f.close();
      }
    }
  }

  void observeSession(uint32_t nowTs, bool isOn, uint8_t modeRaw, uint8_t targetT, uint16_t powerW, float deltaWh) {
    if (isOn) {
      if (!sessActive) {
        sessActive = true;
        sessStartEpoch = nowTs;
        sessWh = 0.0f;
        sessSumW = 0;
        sessCntW = 0;
      }
      sessMode = modeRaw;
      sessTarget = targetT;
      sessWh += deltaWh;
      sessSumW += powerW;
      sessCntW++;
    } else if (sessActive) {
      // AC just turned OFF -> record session if >= 60s
      if (nowTs > sessStartEpoch + 30) {
        AcSessionRec s;
        s.startEpoch = sessStartEpoch;
        s.endEpoch = nowTs;
        s.energyWh = (uint32_t)roundf(sessWh);
        s.modeRaw = sessMode;
        s.targetTemp = sessTarget;
        s.avgPowerW = (sessCntW > 0) ? (uint16_t)(sessSumW / sessCntW) : 0;
        ramSessions[sessionHead] = s;
        sessionHead = (sessionHead + 1) % MAX_SESSIONS;
        if (sessionCount < MAX_SESSIONS) sessionCount++;
        persistSessions();
      }
      sessActive = false;
    }
  }

  void persistSessions() {
    File f = LittleFS.open("/sessions_v4.bin", "w");
    if (!f) return;
    int start = (sessionCount < MAX_SESSIONS) ? 0 : sessionHead;
    for (int i = 0; i < sessionCount; i++) {
      int idx = (start + i) % MAX_SESSIONS;
      f.write((const uint8_t*)&ramSessions[idx], sizeof(AcSessionRec));
    }
    f.close();
  }

  void updateTodaySummary(uint16_t dayIdx, uint8_t localHr, float dtSec,
                          int16_t t100, uint16_t h100, bool acOn,
                          uint16_t powerW, float deltaWh, int pipeC, int16_t coil10) {
    HistDayRec* slot = nullptr;
    if (countDays > 0 && ramDays[countDays - 1].dayIndex == dayIdx) {
      slot = &ramDays[countDays - 1];
      if (accDayIdx != dayIdx) {
        // Restored after reboot on the same day -> seed accumulators from existing record!
        accDayIdx = dayIdx;
        accDayOnSec = slot->acOnMinutes * 60.0f;
        accDayWh = (float)slot->energyWh;
        accDaySumT = (slot->tAvgX100 != INT16_MIN) ? (int64_t)slot->tAvgX100 * 4 : 0;
        accDayCntT = (slot->tAvgX100 != INT16_MIN) ? 4 : 0;
        accDaySumH = (slot->hAvgX100 != 0xFFFF) ? (uint64_t)slot->hAvgX100 * 4 : 0;
        accDayCntH = (slot->hAvgX100 != 0xFFFF) ? 4 : 0;
        accDaySumOnW = (uint64_t)slot->avgOnWatts * 4;
        accDayCntOnW = (slot->avgOnWatts > 0) ? 4 : 0;
      }
    } else {
      accDayIdx = dayIdx;
      accDaySumT = 0; accDayCntT = 0;
      accDaySumH = 0; accDayCntH = 0;
      accDayOnSec = 0.0f; accDayWh = 0.0f;
      accDaySumOnW = 0; accDayCntOnW = 0;

      if (countDays >= RAM_DAYS) {
        memmove(&ramDays[0], &ramDays[1], sizeof(HistDayRec) * (RAM_DAYS - 1));
        countDays = RAM_DAYS - 1;
      }
      slot = &ramDays[countDays++];
      memset(slot, 0, sizeof(HistDayRec));
      slot->dayIndex = dayIdx;
      slot->tMinX100 = INT16_MIN;
      slot->tMaxX100 = INT16_MIN;
      slot->tAvgX100 = INT16_MIN;
      slot->hMinX100 = 0xFFFF;
      slot->hMaxX100 = 0xFFFF;
      slot->hAvgX100 = 0xFFFF;
      slot->peakPipeC = -999;
      slot->minCoilX10 = INT16_MIN;
    }

    if (t100 != INT16_MIN) {
      accDaySumT += t100; accDayCntT++;
      if (slot->tMinX100 == INT16_MIN || t100 < slot->tMinX100) slot->tMinX100 = t100;
      if (slot->tMaxX100 == INT16_MIN || t100 > slot->tMaxX100) slot->tMaxX100 = t100;
      slot->tAvgX100 = (int16_t)(accDaySumT / accDayCntT);
    }
    if (h100 != 0xFFFF) {
      accDaySumH += h100; accDayCntH++;
      if (slot->hMinX100 == 0xFFFF || h100 < slot->hMinX100) slot->hMinX100 = h100;
      if (slot->hMaxX100 == 0xFFFF || h100 > slot->hMaxX100) slot->hMaxX100 = h100;
      slot->hAvgX100 = (uint16_t)(accDaySumH / accDayCntH);
    }
    if (acOn) {
      accDayOnSec += dtSec;
      accDayWh += deltaWh;
      accDaySumOnW += powerW;
      accDayCntOnW++;
      slot->acOnMinutes = (uint16_t)(accDayOnSec / 60.0f);
      slot->energyWh = (uint32_t)roundf(accDayWh);
      slot->activeHoursMask |= (1UL << (localHr % 24));
      if (accDayCntOnW > 0) slot->avgOnWatts = (uint16_t)(accDaySumOnW / accDayCntOnW);
    }
    if (pipeC > -100 && pipeC > slot->peakPipeC) slot->peakPipeC = (int16_t)pipeC;
    if (coil10 != INT16_MIN && (slot->minCoilX10 == INT16_MIN || coil10 < slot->minCoilX10)) slot->minCoilX10 = coil10;
  }

  void commitCurrentBucket() {
    if (curBucketCount <= 0) return;
    Hist15mRec r;
    r.epoch = curBucketEpoch;
    r.xTempX100 = (curCntXT > 0) ? (int16_t)(curSumXT / curCntXT) : INT16_MIN;
    r.xHumX100 = (curCntXH > 0) ? (uint16_t)(curSumXH / curCntXH) : 0xFFFF;
    r.acRoomX10 = (curCntAcR > 0) ? (int16_t)(curSumAcR / curCntAcR) : INT16_MIN;
    r.outTempX10 = (curCntOut > 0) ? (int16_t)(curSumOut / curCntOut) : INT16_MIN;
    r.coilX10 = (curCntCoil > 0) ? (int16_t)(curSumCoil / curCntCoil) : INT16_MIN;
    r.pipeC = (curCntPipe > 0) ? (int16_t)(curSumPipe / curCntPipe) : INT16_MIN;
    r.avgPowerW = (curCntPwr > 0) ? (uint16_t)(curSumW / curCntPwr) : 0;
    r.avgFreqHz = (curCntPwr > 0) ? (uint8_t)(curSumHz / curCntPwr) : 0;
    r.targetAndFlags = curTargetFlags;

    // Update or append in RAM ring
    bool replaced = false;
    if (count15m > 0) {
      int lastIdx = (head15m - 1 + RAM_15M) % RAM_15M;
      if (ram15m[lastIdx].epoch == r.epoch) {
        ram15m[lastIdx] = r;
        replaced = true;
      }
    }
    if (!replaced) {
      ram15m[head15m] = r;
      head15m = (head15m + 1) % RAM_15M;
      if (count15m < RAM_15M) count15m++;
    }

    // Persist 15m file (trim when > MAX_15M + 32)
    File f = LittleFS.open("/hist15_v4.bin", "a");
    if (f) {
      f.write((const uint8_t*)&r, sizeof(r));
      size_t sz = f.size();
      f.close();
      if (sz > (MAX_15M + 32) * sizeof(Hist15mRec)) trim15mFile();
    }

    // Persist today's daily record
    if (countDays > 0) persistLatestDay(ramDays[countDays - 1]);
  }

  void trim15mFile() {
    File fin = LittleFS.open("/hist15_v4.bin", "r");
    if (!fin) return;
    size_t nRecs = fin.size() / sizeof(Hist15mRec);
    if (nRecs <= MAX_15M) { fin.close(); return; }
    fin.seek((nRecs - MAX_15M) * sizeof(Hist15mRec));
    File fout = LittleFS.open("/hist15_v4.tmp", "w");
    if (fout) {
      Hist15mRec r;
      while (fin.read((uint8_t*)&r, sizeof(r)) == sizeof(r)) fout.write((const uint8_t*)&r, sizeof(r));
      fout.close();
      fin.close();
      LittleFS.remove("/hist15_v4.bin");
      LittleFS.rename("/hist15_v4.tmp", "/hist15_v4.bin");
    } else {
      fin.close();
    }
  }

  void persistLatestDay(const HistDayRec& rec) {
    bool updated = false;
    if (LittleFS.exists("/histday_v4.bin")) {
      File f = LittleFS.open("/histday_v4.bin", "r+");
      if (f && f.size() >= sizeof(HistDayRec)) {
        f.seek(f.size() - sizeof(HistDayRec));
        HistDayRec last;
        if (f.read((uint8_t*)&last, sizeof(last)) == sizeof(last) && last.dayIndex == rec.dayIndex) {
          f.seek(f.size() - sizeof(HistDayRec));
          f.write((const uint8_t*)&rec, sizeof(rec));
          updated = true;
        }
      }
      if (f) f.close();
    }
    if (!updated) {
      File f = LittleFS.open("/histday_v4.bin", "a");
      if (f) {
        f.write((const uint8_t*)&rec, sizeof(rec));
        f.close();
      }
    }
  }

  static String fmtOptF(int32_t scaled, float div, int dec, int32_t invalidVal) {
    if (scaled == invalidVal) return "null";
    return String(scaled / div, dec);
  }

  String appendSessionsJson() const {
    String j = "[";
    int emitted = 0;
    // Include currently active ON session first!
    if (sessActive) {
      time_t nowUtc = time(nullptr);
      uint32_t nowTs = (nowUtc > 1700000000) ? (uint32_t)nowUtc : (1763900000UL + (millis() / 1000UL));
      uint32_t durMin = (nowTs > sessStartEpoch) ? ((nowTs - sessStartEpoch) / 60UL) : 0;
      float kwh = sessWh / 1000.0f;
      uint16_t avgW = (sessCntW > 0) ? (uint16_t)(sessSumW / sessCntW) : 0;
      j += "{\"active\":true,\"start\":\"" + epochToLocalTimeStr(sessStartEpoch, true) + "\",\"end\":\"กำลังเปิดอยู่\",";
      j += "\"dur_min\":" + String(durMin) + ",\"kwh\":" + String(kwh, 3) + ",\"cost\":" + String(kwh * tariffBahtPerKwh, 2) + ",";
      j += "\"avg_w\":" + String(avgW) + ",\"target\":" + String(sessTarget) + ",\"mode\":" + String(sessMode) + "}";
      emitted++;
    }
    for (int i = 0; i < sessionCount && emitted < 16; i++) {
      int idx = (sessionHead - 1 - i + MAX_SESSIONS) % MAX_SESSIONS;
      const AcSessionRec& s = ramSessions[idx];
      uint32_t durMin = (s.endEpoch > s.startEpoch) ? ((s.endEpoch - s.startEpoch) / 60UL) : 0;
      float kwh = s.energyWh / 1000.0f;
      if (emitted > 0) j += ",";
      j += "{\"active\":false,\"start\":\"" + epochToLocalTimeStr(s.startEpoch, true) + "\",\"end\":\"" + epochToLocalTimeStr(s.endEpoch, false) + "\",";
      j += "\"dur_min\":" + String(durMin) + ",\"kwh\":" + String(kwh, 3) + ",\"cost\":" + String(kwh * tariffBahtPerKwh, 2) + ",";
      j += "\"avg_w\":" + String(s.avgPowerW) + ",\"target\":" + String(s.targetTemp) + ",\"mode\":" + String(s.modeRaw) + "}";
      emitted++;
    }
    j += "]";
    return j;
  }

  String buildDayJson(int dayOffset, bool xOk, float xT, float xH,
                      bool acOn, float acR, int outT, float coilC, int pipeC,
                      uint16_t liveW, uint8_t liveHz, float liveTarget) const {
    time_t nowUtc = time(nullptr);
    uint32_t nowTs = (nowUtc > 1700000000) ? (uint32_t)nowUtc : (1763900000UL + (millis() / 1000UL));
    uint16_t todayIdx = epochToDayIndex(nowTs);
    uint16_t wantDay = (dayOffset > 0 && todayIdx >= dayOffset) ? (todayIdx - (uint16_t)dayOffset) : todayIdx;

    String j;
    j.reserve(4096);
    j = "{\"range\":\"day\",\"day_index\":" + String(wantDay) + ",\"date\":\"" + dayIndexToDateStr(wantDay) + "\",";
    j += "\"tariff\":" + String(tariffBahtPerKwh, 2) + ",\"points\":[";

    int emitted = 0;
    double sumT = 0, sumH = 0, sumW = 0;
    int cntT = 0, cntH = 0, cntW = 0, onSlots = 0;
    float minT = 999, maxT = -999;
    uint32_t hrMask = 0;

    auto emitRec = [&](const Hist15mRec& r, const char* lblOverride) {
      if (emitted > 0) j += ",";
      String lbl = lblOverride ? String(lblOverride) : epochToLocalTimeStr(r.epoch, false);
      uint8_t tRaw = r.targetAndFlags & 0x1F;
      bool isOn = (r.targetAndFlags & 0x20) != 0;
      j += "{\"lbl\":\"" + lbl + "\"";
      j += ",\"xt\":" + fmtOptF(r.xTempX100, 100.0f, 1, INT16_MIN);
      j += ",\"xh\":" + fmtOptF(r.xHumX100, 100.0f, 0, 0xFFFF);
      j += ",\"ar\":" + fmtOptF(r.acRoomX10, 10.0f, 1, INT16_MIN);
      j += ",\"ot\":" + fmtOptF(r.outTempX10, 10.0f, 1, INT16_MIN);
      j += ",\"cl\":" + fmtOptF(r.coilX10, 10.0f, 1, INT16_MIN);
      j += ",\"pp\":" + fmtOptF(r.pipeC, 1.0f, 0, INT16_MIN);
      j += ",\"w\":" + String(r.avgPowerW);
      j += ",\"hz\":" + String(r.avgFreqHz);
      j += ",\"tg\":" + (tRaw <= 14 ? String(16 + tRaw) : String("null"));
      j += ",\"on\":" + String(isOn ? 1 : 0) + "}";
      emitted++;

      float rt = (r.xTempX100 != INT16_MIN) ? (r.xTempX100 / 100.0f) : ((r.acRoomX10 != INT16_MIN) ? (r.acRoomX10 / 10.0f) : NAN);
      if (!isnan(rt)) { sumT += rt; cntT++; if (rt < minT) minT = rt; if (rt > maxT) maxT = rt; }
      if (r.xHumX100 != 0xFFFF) { sumH += r.xHumX100 / 100.0f; cntH++; }
      if (isOn) { sumW += r.avgPowerW; cntW++; onSlots++; hrMask |= (1UL << epochToLocalHour(r.epoch)); }
    };

    if (dayOffset == 0) {
      int start = (count15m < RAM_15M) ? 0 : head15m;
      for (int i = 0; i < count15m; i++) {
        const Hist15mRec& r = ram15m[(start + i) % RAM_15M];
        if (epochToDayIndex(r.epoch) == wantDay) emitRec(r, nullptr);
      }
      // Append live NOW point
      Hist15mRec nowR;
      nowR.epoch = nowTs;
      nowR.xTempX100 = (xOk && !isnan(xT)) ? (int16_t)roundf(xT * 100.0f) : INT16_MIN;
      nowR.xHumX100 = (xOk && !isnan(xH)) ? (uint16_t)roundf(xH * 100.0f) : 0xFFFF;
      nowR.acRoomX10 = !isnan(acR) ? (int16_t)roundf(acR * 10.0f) : INT16_MIN;
      nowR.outTempX10 = (outT > -100) ? (int16_t)(outT * 10) : INT16_MIN;
      nowR.coilX10 = !isnan(coilC) ? (int16_t)roundf(coilC * 10.0f) : INT16_MIN;
      nowR.pipeC = (pipeC > -100) ? (int16_t)pipeC : INT16_MIN;
      nowR.avgPowerW = acOn ? liveW : 0;
      nowR.avgFreqHz = acOn ? liveHz : 0;
      uint8_t tOff = acOn ? (uint8_t)constrain((int)roundf(liveTarget) - 16, 0, 14) : 31;
      nowR.targetAndFlags = (tOff & 0x1F) | (acOn ? 0x20 : 0);
      emitRec(nowR, "NOW");
    } else if (LittleFS.exists("/hist15_v4.bin")) {
      File f = LittleFS.open("/hist15_v4.bin", "r");
      if (f) {
        Hist15mRec r;
        while (f.read((uint8_t*)&r, sizeof(r)) == sizeof(r)) {
          if (epochToDayIndex(r.epoch) == wantDay) emitRec(r, nullptr);
        }
        f.close();
      }
    }

    // Lookup matching HistDayRec for accurate integrated Wh & ON minutes
    float totalKwh = (onSlots * 0.25f) * (cntW > 0 ? (sumW / cntW) : 0.0f) / 1000.0f;
    uint32_t totalOnMin = onSlots * 15;
    for (int i = countDays - 1; i >= 0; i--) {
      if (ramDays[i].dayIndex == wantDay) {
        totalKwh = ramDays[i].energyWh / 1000.0f;
        totalOnMin = ramDays[i].acOnMinutes;
        hrMask |= ramDays[i].activeHoursMask;
        break;
      }
    }

    j += "],\"summary\":{";
    j += "\"total_kwh\":" + String(totalKwh, 3);
    j += ",\"total_cost\":" + String(totalKwh * tariffBahtPerKwh, 2);
    j += ",\"avg_daily_kwh\":" + String(totalKwh, 3);
    j += ",\"total_on_min\":" + String(totalOnMin);
    j += ",\"avg_temp\":" + (cntT > 0 ? String(sumT / cntT, 1) : String("null"));
    j += ",\"min_temp\":" + (cntT > 0 ? String(minT, 1) : String("null"));
    j += ",\"max_temp\":" + (cntT > 0 ? String(maxT, 1) : String("null"));
    j += ",\"avg_hum\":" + (cntH > 0 ? String(sumH / cntH, 0) : String("null"));
    j += ",\"avg_watts\":" + String(cntW > 0 ? (int)(sumW / cntW) : 0);
    j += ",\"heatmap\":[";
    for (int h = 0; h < 24; h++) {
      if (h > 0) j += ",";
      j += String((hrMask >> h) & 1);
    }
    j += "]},\"sessions\":" + appendSessionsJson() + "}";
    return j;
  }

  String buildDaysJson(int maxDays) const {
    String rangeName = (maxDays <= 7) ? "week" : "month";
    String j;
    j.reserve(4096);
    j = "{\"range\":\"" + rangeName + "\",\"tariff\":" + String(tariffBahtPerKwh, 2) + ",\"points\":[";

    int start = (countDays > maxDays) ? (countDays - maxDays) : 0;
    int emitted = 0;
    double sumT = 0, sumH = 0, sumKwh = 0, sumW = 0;
    int cntT = 0, cntH = 0, cntW = 0;
    float minT = 999, maxT = -999;
    uint32_t totalOnMin = 0;
    int hrHeat[24] = {0};

    for (int i = start; i < countDays; i++) {
      const HistDayRec& d = ramDays[i];
      float kwh = d.energyWh / 1000.0f;
      float cost = kwh * tariffBahtPerKwh;
      String dt = dayIndexToDateStr(d.dayIndex);
      if (emitted > 0) j += ",";
      j += "{\"lbl\":\"" + dt.substring(5) + "\",\"date\":\"" + dt + "\",\"day_idx\":" + String(d.dayIndex);
      j += ",\"xt\":" + fmtOptF(d.tAvgX100, 100.0f, 1, INT16_MIN);
      j += ",\"tmin\":" + fmtOptF(d.tMinX100, 100.0f, 1, INT16_MIN);
      j += ",\"tmax\":" + fmtOptF(d.tMaxX100, 100.0f, 1, INT16_MIN);
      j += ",\"xh\":" + fmtOptF(d.hAvgX100, 100.0f, 0, 0xFFFF);
      j += ",\"w\":" + String(d.avgOnWatts);
      j += ",\"kwh\":" + String(kwh, 3);
      j += ",\"cost\":" + String(cost, 2);
      j += ",\"on_min\":" + String(d.acOnMinutes);
      j += ",\"pp\":" + (d.peakPipeC > -100 ? String(d.peakPipeC) : String("null"));
      j += ",\"cl\":" + fmtOptF(d.minCoilX10, 10.0f, 1, INT16_MIN) + "}";
      emitted++;

      if (d.tAvgX100 != INT16_MIN) { sumT += d.tAvgX100 / 100.0f; cntT++; }
      if (d.tMinX100 != INT16_MIN && (d.tMinX100 / 100.0f) < minT) minT = d.tMinX100 / 100.0f;
      if (d.tMaxX100 != INT16_MIN && (d.tMaxX100 / 100.0f) > maxT) maxT = d.tMaxX100 / 100.0f;
      if (d.hAvgX100 != 0xFFFF) { sumH += d.hAvgX100 / 100.0f; cntH++; }
      if (d.avgOnWatts > 0) { sumW += d.avgOnWatts; cntW++; }
      sumKwh += kwh;
      totalOnMin += d.acOnMinutes;
      for (int h = 0; h < 24; h++) {
        if ((d.activeHoursMask >> h) & 1) hrHeat[h]++;
      }
    }

    j += "],\"summary\":{";
    j += "\"total_kwh\":" + String(sumKwh, 3);
    j += ",\"total_cost\":" + String(sumKwh * tariffBahtPerKwh, 2);
    j += ",\"avg_daily_kwh\":" + String(emitted > 0 ? (sumKwh / emitted) : 0.0f, 3);
    j += ",\"total_on_min\":" + String(totalOnMin);
    j += ",\"avg_temp\":" + (cntT > 0 ? String(sumT / cntT, 1) : String("null"));
    j += ",\"min_temp\":" + (minT < 900 ? String(minT, 1) : String("null"));
    j += ",\"max_temp\":" + (maxT > -900 ? String(maxT, 1) : String("null"));
    j += ",\"avg_hum\":" + (cntH > 0 ? String(sumH / cntH, 0) : String("null"));
    j += ",\"avg_watts\":" + String(cntW > 0 ? (int)(sumW / cntW) : 0);
    j += ",\"heatmap\":[";
    for (int h = 0; h < 24; h++) {
      if (h > 0) j += ",";
      j += String(hrHeat[h]);
    }
    j += "]},\"sessions\":" + appendSessionsJson() + "}";
    return j;
  }

  String buildYearJson() const {
    // Aggregate by YYYY-MM from /histday_v4.bin (up to 12 months)
    struct MonthAgg {
      char ym[8];
      double sumT, sumH, sumKwh, sumW;
      int cntT, cntH, cntW, days;
      uint32_t onMin;
      float minT, maxT;
    } m[12];
    int mCnt = 0;
    int hrHeat[24] = {0};

    auto addDay = [&](const HistDayRec& d) {
      String dt = dayIndexToDateStr(d.dayIndex);
      String ym = dt.substring(0, 7);
      int idx = -1;
      for (int k = 0; k < mCnt; k++) {
        if (ym == m[k].ym) { idx = k; break; }
      }
      if (idx < 0) {
        if (mCnt >= 12) {
          memmove(&m[0], &m[1], sizeof(MonthAgg) * 11);
          mCnt = 11;
        }
        idx = mCnt++;
        strncpy(m[idx].ym, ym.c_str(), 7); m[idx].ym[7] = 0;
        m[idx].sumT = m[idx].sumH = m[idx].sumKwh = m[idx].sumW = 0;
        m[idx].cntT = m[idx].cntH = m[idx].cntW = m[idx].days = 0;
        m[idx].onMin = 0;
        m[idx].minT = 999; m[idx].maxT = -999;
      }
      m[idx].days++;
      if (d.tAvgX100 != INT16_MIN) { m[idx].sumT += d.tAvgX100 / 100.0f; m[idx].cntT++; }
      if (d.tMinX100 != INT16_MIN && (d.tMinX100 / 100.0f) < m[idx].minT) m[idx].minT = d.tMinX100 / 100.0f;
      if (d.tMaxX100 != INT16_MIN && (d.tMaxX100 / 100.0f) > m[idx].maxT) m[idx].maxT = d.tMaxX100 / 100.0f;
      if (d.hAvgX100 != 0xFFFF) { m[idx].sumH += d.hAvgX100 / 100.0f; m[idx].cntH++; }
      if (d.avgOnWatts > 0) { m[idx].sumW += d.avgOnWatts; m[idx].cntW++; }
      m[idx].sumKwh += d.energyWh / 1000.0f;
      m[idx].onMin += d.acOnMinutes;
      for (int h = 0; h < 24; h++) if ((d.activeHoursMask >> h) & 1) hrHeat[h]++;
    };

    if (LittleFS.exists("/histday_v4.bin")) {
      File f = LittleFS.open("/histday_v4.bin", "r");
      if (f) {
        HistDayRec d;
        while (f.read((uint8_t*)&d, sizeof(d)) == sizeof(d)) addDay(d);
        f.close();
      }
    } else {
      for (int i = 0; i < countDays; i++) addDay(ramDays[i]);
    }

    String j = "{\"range\":\"year\",\"tariff\":" + String(tariffBahtPerKwh, 2) + ",\"points\":[";
    double totKwh = 0, totT = 0, totH = 0, totW = 0;
    int cT = 0, cH = 0, cW = 0, totDays = 0;
    uint32_t totOn = 0;
    float gMin = 999, gMax = -999;
    for (int k = 0; k < mCnt; k++) {
      if (k > 0) j += ",";
      j += "{\"lbl\":\"" + String(m[k].ym) + "\"";
      j += ",\"xt\":" + (m[k].cntT > 0 ? String(m[k].sumT / m[k].cntT, 1) : String("null"));
      j += ",\"tmin\":" + (m[k].minT < 900 ? String(m[k].minT, 1) : String("null"));
      j += ",\"tmax\":" + (m[k].maxT > -900 ? String(m[k].maxT, 1) : String("null"));
      j += ",\"xh\":" + (m[k].cntH > 0 ? String(m[k].sumH / m[k].cntH, 0) : String("null"));
      j += ",\"w\":" + String(m[k].cntW > 0 ? (int)(m[k].sumW / m[k].cntW) : 0);
      j += ",\"kwh\":" + String(m[k].sumKwh, 2);
      j += ",\"cost\":" + String(m[k].sumKwh * tariffBahtPerKwh, 2);
      j += ",\"on_min\":" + String(m[k].onMin) + "}";
      totKwh += m[k].sumKwh; totOn += m[k].onMin; totDays += m[k].days;
      if (m[k].cntT > 0) { totT += m[k].sumT / m[k].cntT; cT++; }
      if (m[k].minT < gMin) gMin = m[k].minT;
      if (m[k].maxT > gMax) gMax = m[k].maxT;
      if (m[k].cntH > 0) { totH += m[k].sumH / m[k].cntH; cH++; }
      if (m[k].cntW > 0) { totW += m[k].sumW / m[k].cntW; cW++; }
    }
    j += "],\"summary\":{";
    j += "\"total_kwh\":" + String(totKwh, 2);
    j += ",\"total_cost\":" + String(totKwh * tariffBahtPerKwh, 2);
    j += ",\"avg_daily_kwh\":" + String(totDays > 0 ? (totKwh / totDays) : 0.0f, 3);
    j += ",\"total_on_min\":" + String(totOn);
    j += ",\"avg_temp\":" + (cT > 0 ? String(totT / cT, 1) : String("null"));
    j += ",\"min_temp\":" + (gMin < 900 ? String(gMin, 1) : String("null"));
    j += ",\"max_temp\":" + (gMax > -900 ? String(gMax, 1) : String("null"));
    j += ",\"avg_hum\":" + (cH > 0 ? String(totH / cH, 0) : String("null"));
    j += ",\"avg_watts\":" + String(cW > 0 ? (int)(totW / cW) : 0);
    j += ",\"heatmap\":[";
    for (int h = 0; h < 24; h++) { if (h > 0) j += ","; j += String(hrHeat[h]); }
    j += "]},\"sessions\":" + appendSessionsJson() + "}";
    return j;
  }
};
