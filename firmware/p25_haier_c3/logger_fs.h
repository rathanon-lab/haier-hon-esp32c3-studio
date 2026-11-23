#pragma once
#include <Arduino.h>
#include <LittleFS.h>
#include <time.h>

// ============================================================================
// Persistent Event Logger v4.0 for ESP32-C3 (Zero-Stall RAM Ring + Batch LittleFS)
// - Writes to RAM immediately (<0.05ms, zero HTTP/UART latency during clicks!)
// - Batch-flushes pending entries to LittleFS every 25s (or >=12 queued / force)
// - Auto-rotates at 64 KB so Flash never fills and SPI Flash wear is minimized
// ============================================================================

struct LogEntryItem {
  String ts;
  String cat; // "CMD", "AC", "CL", "RULE", "BLE", "SYS", "ERR"
  String msg;
};

class PersistentLogger {
public:
  static const int RAM_CAP = 80;
  static const int PENDING_CAP = 24;
  static const size_t ROTATE_BYTES = 64 * 1024;

  bool fsReady = false;
  uint32_t totalLogged = 0;

  void begin() {
    fsReady = LittleFS.begin(true);
    if (fsReady) {
      preloadTailFromFs();
      refreshFsStats();
    }
  }

  static String currentTimestamp() {
    time_t now = time(nullptr);
    if (now > 1700000000) {
      struct tm t;
      localtime_r(&now, &t);
      char buf[24];
      snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
               t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
               t.tm_hour, t.tm_min, t.tm_sec);
      return String(buf);
    }
    unsigned long s = millis() / 1000UL;
    char buf[24];
    snprintf(buf, sizeof(buf), "UP+%02lu:%02lu:%02lu", s / 3600UL, (s % 3600UL) / 60UL, s % 60UL);
    return String(buf);
  }

  static String jsonEsc(const String& s) {
    String out;
    out.reserve(s.length() + 4);
    for (size_t i = 0; i < s.length(); i++) {
      char c = s[i];
      if (c == '"' || c == '\\') out += '\\';
      if (c >= 32) out += c;
    }
    return out;
  }

  // O(1) RAM-only append -> never stalls HTTP response or UART!
  void add(const String& cat, const String& msg) {
    String ts = currentTimestamp();
    ramBuf[ramHead] = {ts, cat, msg};
    ramHead = (ramHead + 1) % RAM_CAP;
    if (ramCount < RAM_CAP) ramCount++;
    totalLogged++;

    if (!fsReady) return;
    if (pendingCount < PENDING_CAP) {
      pendingBuf[pendingCount++] = {ts, cat, msg};
    } else {
      // Shift oldest pending out if queue full
      for (int i = 1; i < PENDING_CAP; i++) pendingBuf[i - 1] = pendingBuf[i];
      pendingBuf[PENDING_CAP - 1] = {ts, cat, msg};
    }
  }

  void flushIfNeeded(bool force = false) {
    if (!fsReady || pendingCount == 0) return;
    unsigned long now = millis();
    if (!force && pendingCount < 12 && (now - lastFlushMs) < 25000UL) return;
    lastFlushMs = now;

    if (LittleFS.exists("/logs.jsonl")) {
      File check = LittleFS.open("/logs.jsonl", "r");
      size_t sz = check ? check.size() : 0;
      if (check) check.close();
      if (sz >= ROTATE_BYTES) {
        if (LittleFS.exists("/logs_prev.jsonl")) LittleFS.remove("/logs_prev.jsonl");
        LittleFS.rename("/logs.jsonl", "/logs_prev.jsonl");
      }
    }

    File f = LittleFS.open("/logs.jsonl", "a");
    if (f) {
      for (int i = 0; i < pendingCount; i++) {
        f.printf("{\"ts\":\"%s\",\"cat\":\"%s\",\"msg\":\"%s\"}\n",
                 pendingBuf[i].ts.c_str(),
                 jsonEsc(pendingBuf[i].cat).c_str(),
                 jsonEsc(pendingBuf[i].msg).c_str());
      }
      f.close();
    }
    pendingCount = 0;
    refreshFsStats();
  }

  void clear() {
    ramHead = 0;
    ramCount = 0;
    pendingCount = 0;
    if (fsReady) {
      LittleFS.remove("/logs.jsonl");
      LittleFS.remove("/logs_prev.jsonl");
      refreshFsStats();
    }
    add("SYS", "Logs cleared by user");
    flushIfNeeded(true);
  }

  void refreshFsStats() {
    if (!fsReady) return;
    cachedUsed = LittleFS.usedBytes();
    cachedTotal = LittleFS.totalBytes();
  }

  size_t fsUsedBytes() const { return cachedUsed; }
  size_t fsTotalBytes() const { return cachedTotal; }

  String toJson(const String& filterCat = "all", int limit = 60) const {
    String j;
    j.reserve(2048);
    j = "{\"fs_used\":" + String(cachedUsed) +
        ",\"fs_total\":" + String(cachedTotal) +
        ",\"ram_count\":" + String(ramCount) +
        ",\"pending_flush\":" + String(pendingCount) +
        ",\"entries\":[";
    int emitted = 0;
    for (int i = 0; i < ramCount && emitted < limit; i++) {
      int idx = (ramHead - 1 - i + RAM_CAP) % RAM_CAP;
      const LogEntryItem& e = ramBuf[idx];
      if (filterCat != "all" && filterCat.length() > 0 && e.cat != filterCat) continue;
      if (emitted > 0) j += ",";
      j += "{\"ts\":\"" + jsonEsc(e.ts) + "\",\"cat\":\"" + jsonEsc(e.cat) + "\",\"msg\":\"" + jsonEsc(e.msg) + "\"}";
      emitted++;
    }
    j += "]}";
    return j;
  }

  String readFullDownload() {
    flushIfNeeded(true);
    if (!fsReady) return "";
    String out;
    if (LittleFS.exists("/logs_prev.jsonl")) {
      File fp = LittleFS.open("/logs_prev.jsonl", "r");
      if (fp) {
        while (fp.available() && out.length() < 20000) out += (char)fp.read();
        fp.close();
      }
    }
    if (LittleFS.exists("/logs.jsonl")) {
      File fa = LittleFS.open("/logs.jsonl", "r");
      if (fa) {
        while (fa.available() && out.length() < 40000) out += (char)fa.read();
        fa.close();
      }
    }
    return out;
  }

private:
  LogEntryItem ramBuf[RAM_CAP];
  int ramHead = 0;
  int ramCount = 0;

  LogEntryItem pendingBuf[PENDING_CAP];
  int pendingCount = 0;
  unsigned long lastFlushMs = 0;
  size_t cachedUsed = 0;
  size_t cachedTotal = 0;

  void preloadTailFromFs() {
    if (!LittleFS.exists("/logs.jsonl")) return;
    File f = LittleFS.open("/logs.jsonl", "r");
    if (!f) return;
    size_t sz = f.size();
    if (sz > 8192) {
      f.seek(sz - 8192);
      f.readStringUntil('\n');
    }
    while (f.available()) {
      String ln = f.readStringUntil('\n');
      ln.trim();
      if (ln.length() < 10) continue;
      String ts = extractField(ln, "\"ts\":\"");
      String cat = extractField(ln, "\"cat\":\"");
      String msg = extractField(ln, "\"msg\":\"");
      if (cat.length() > 0) {
        ramBuf[ramHead] = {ts, cat, msg};
        ramHead = (ramHead + 1) % RAM_CAP;
        if (ramCount < RAM_CAP) ramCount++;
      }
    }
    f.close();
  }

  static String extractField(const String& line, const char* key) {
    int p = line.indexOf(key);
    if (p < 0) return "";
    p += strlen(key);
    int e = line.indexOf('"', p);
    if (e < 0) return "";
    return line.substring(p, e);
  }
};
