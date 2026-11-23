// ============================================================================
// P25 Standalone ESP32-C3 Haier AC Studio (v4.0)
// Target: Haier HSU-15VQRA03T (hOn E++2.18) + Xiaomi LYWSD03MMC (ATC_v59 / Stock)
// Features:
//   - Single-Session Exclusive Lease & Takeover (no conflicting web controllers)
//   - Adaptive Dynamic Polling & Eco-Idle Sleep (UART 1.8s / 15s / 60s, BLE 12.5% / 6.2%)
//   - 1-Request AC Control API (immediate full status response, zero polling storms)
//   - If-Else Autonomous Rule Engine (LittleFS /rules_v4.bin) + Direct Presets
//   - Multi-Sensor Time-Series & Real kWh/Cost Energy Meter (LittleFS /hist*_v4.bin)
//   - Zero-Stall RAM Ring + Batch LittleFS Logger (/logs.jsonl)
//   - Hardware Task Watchdog (30s) + Web OTA (/update) + ArduinoOTA + Fail-Safe AP
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPUpdateServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <esp_task_wdt.h>
#include <time.h>

#include "haier_protocol.h"
#include "xiaomi_ble.h"
#include "logger_fs.h"
#include "history_fs.h"
#include "rules_engine.h"
#include "web_ui.h"

static const char* FW_VERSION = "4.1.0";
static const char* AP_SSID    = "Haier-AC-P25-AP";
static const char* AP_PASS    = "haier-ap-setup";
static const char* OTA_PASS   = "CHANGE_ME_OTA_PASS";

// Core subsystems
WebServer server(80);
HTTPUpdateServer httpUpdater;
Preferences prefs;

HaierAcController ac;
XiaomiBleReader xiaomi;
PersistentLogger logger;
ClimateHistoryStore hist;
AutomationRulesEngine rules;

// NVS Config (defaults match live 192.168.1.25 hardware: RX=4, TX=3)
struct NodeConfig {
  String ssid1      = "YOUR_PRIMARY_WIFI_SSID";
  String pass1      = "YOUR_PRIMARY_WIFI_PASS";
  String ssid2      = "YOUR_BACKUP_WIFI_SSID";
  String pass2      = "YOUR_BACKUP_WIFI_PASS";
  bool   useStatic  = true;
  int    ipSuffix   = 25;
  int    rxPin      = 4;
  int    txPin      = 3;
  String bleMac     = "aa:bb:cc:dd:ee:ff";
  String clPost     = "restore_prev";
  float  tariff     = 4.69f;    // Effective THB/kWh (auto-synced from PEA config or custom)
  String peaType    = "1.1.2";  // "1.1.1" (<=150), "1.1.2" (>150), "1.2" (TOU), "custom"
  float  ftSatang   = 16.23f;   // PEA Ft in satang/kWh (default: Sep-Dec 2569 = 16.23 satang)
  bool   peaVat     = true;     // Include 7% VAT
  bool   peaSvc     = false;    // Include monthly service fee (8.19 / 24.62 THB)
  int    peaTier    = 1;        // 0=progressive from 0, 1=marginal 151-400 (4.2218), 2=marginal >400 (4.4217)
  float  customRate = 4.50f;    // Used when peaType == "custom"
  bool   stickyMute = false;
  bool   smartEco   = false;
  float  smartEcoTh = 27.0f;
} cfg;

// Single-Session Lease State
static const unsigned long SESSION_LEASE_MS = 25000UL;
String sessionOwnerId    = "";
String sessionOwnerLabel = "";
unsigned long sessionExpiresMs = 0;

// Adaptive Scheduler & Runtime State
unsigned long lastWebClientMs   = 0;
unsigned long realtimeExpiresMs = 0;
unsigned long lastAcPollMs      = 0;
unsigned long lastHistObsMs   = 0;
unsigned long lastRulesEvalMs = 0;
unsigned long lastWifiCheckMs = 0;
unsigned long wifiDownSinceMs = 0;
unsigned long lastProbeMs     = 0;
bool fastWakePoll = false;
bool apModeActive = false;
bool inBleYield   = false;
String currentPollMode = "ECO_IDLE";

// ============================================================================
// Helpers
// ============================================================================
static String jEsc(const String& s) {
  String out;
  out.reserve(s.length() + 4);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') out += '\\';
    if (c >= 32) out += c;
  }
  return out;
}

static void feedWdt() {
  esp_task_wdt_reset();
}

static float computeEffectivePeaRate() {
  float ftBaht = cfg.ftSatang / 100.0f;
  float vatMul = cfg.peaVat ? 1.07f : 1.0f;
  if (cfg.peaType == "custom") {
    return constrain(cfg.customRate, 0.5f, 25.0f);
  }
  if (cfg.peaType == "1.2") {
    // PEA 1.2.2 (<22kV TOU): Mon-Fri 09:00-22:00 On-Peak 5.7982, else Off-Peak 2.6369
    time_t nowUtc = time(nullptr);
    bool onPeak = false;
    if (nowUtc > 1700000000) {
      time_t loc = nowUtc + 7L * 3600L;
      struct tm t;
      gmtime_r(&loc, &t);
      bool weekday = (t.tm_wday >= 1 && t.tm_wday <= 5);
      onPeak = weekday && (t.tm_hour >= 9 && t.tm_hour < 22);
    }
    float base = onPeak ? 5.7982f : 2.6369f;
    return (base + ftBaht) * vatMul;
  }
  float base = 4.2218f;
  if (cfg.peaTier == 2) base = 4.4217f;
  else if (cfg.peaTier == 1) base = 4.2218f;
  else base = (cfg.peaType == "1.1.1") ? 3.6237f : 3.2484f;
  return (base + ftBaht) * vatMul;
}

static void loadConfig() {
  prefs.begin("p25haier", true);
  cfg.ssid1      = prefs.getString("ssid1", cfg.ssid1);
  cfg.pass1      = prefs.getString("pass1", cfg.pass1);
  cfg.ssid2      = prefs.getString("ssid2", cfg.ssid2);
  cfg.pass2      = prefs.getString("pass2", cfg.pass2);
  cfg.useStatic  = prefs.getBool("static", cfg.useStatic);
  cfg.ipSuffix   = constrain(prefs.getInt("suffix", cfg.ipSuffix), 2, 250);
  cfg.rxPin      = prefs.getInt("rx", cfg.rxPin);
  cfg.txPin      = prefs.getInt("tx", cfg.txPin);
  cfg.bleMac     = prefs.getString("mac", cfg.bleMac);
  cfg.clPost     = prefs.getString("clpost", cfg.clPost);
  cfg.peaType    = prefs.getString("peatype", cfg.peaType);
  cfg.ftSatang   = prefs.getFloat("ftsat", cfg.ftSatang);
  if (isnan(cfg.ftSatang) || cfg.ftSatang < -200.0f || cfg.ftSatang > 500.0f) cfg.ftSatang = 16.23f;
  cfg.peaVat     = prefs.getBool("peavat", cfg.peaVat);
  cfg.peaSvc     = prefs.getBool("peasvc", cfg.peaSvc);
  cfg.peaTier    = constrain(prefs.getInt("peatier", cfg.peaTier), 0, 2);
  cfg.customRate = prefs.getFloat("custrt", cfg.customRate);
  if (isnan(cfg.customRate) || cfg.customRate < 0.5f || cfg.customRate > 25.0f) cfg.customRate = 4.50f;
  cfg.stickyMute = prefs.getBool("mute", cfg.stickyMute);
  cfg.smartEco   = prefs.getBool("seco", cfg.smartEco);
  cfg.smartEcoTh = prefs.getFloat("secoth", cfg.smartEcoTh);
  prefs.end();

  cfg.tariff            = computeEffectivePeaRate();
  ac.cl.postAction      = cfg.clPost;
  ac.stickyBeeperMuted  = cfg.stickyMute;
  ac.smartEcoEnabled    = cfg.smartEco;
  ac.smartEcoThresholdC = cfg.smartEcoTh;
  hist.tariffBahtPerKwh = cfg.tariff;
}

static void saveConfig() {
  cfg.tariff = computeEffectivePeaRate();
  hist.tariffBahtPerKwh = cfg.tariff;
  prefs.begin("p25haier", false);
  prefs.putString("ssid1", cfg.ssid1);
  prefs.putString("pass1", cfg.pass1);
  prefs.putString("ssid2", cfg.ssid2);
  prefs.putString("pass2", cfg.pass2);
  prefs.putBool("static", cfg.useStatic);
  prefs.putInt("suffix", cfg.ipSuffix);
  prefs.putInt("rx", cfg.rxPin);
  prefs.putInt("tx", cfg.txPin);
  prefs.putString("mac", cfg.bleMac);
  prefs.putString("clpost", cfg.clPost);
  prefs.putString("peatype", cfg.peaType);
  prefs.putFloat("ftsat", cfg.ftSatang);
  prefs.putBool("peavat", cfg.peaVat);
  prefs.putBool("peasvc", cfg.peaSvc);
  prefs.putInt("peatier", cfg.peaTier);
  prefs.putFloat("custrt", cfg.customRate);
  prefs.putFloat("tariff", cfg.tariff);
  prefs.putBool("mute", cfg.stickyMute);
  prefs.putBool("seco", cfg.smartEco);
  prefs.putFloat("secoth", cfg.smartEcoTh);
  prefs.end();
}

// ============================================================================
// Single-Session Lease & Adaptive Wake
// ============================================================================
static void noteWebActivity() {
  unsigned long now = millis();
  bool wasIdle = (lastWebClientMs == 0) || ((now - lastWebClientMs) >= 45000UL);
  lastWebClientMs = now;
  if (wasIdle && (now - lastAcPollMs) > 2500UL) {
    fastWakePoll = true;
  }
}

static bool checkOrClaimSession(const String& cid, const String& dev, bool claimIfFree) {
  unsigned long now = millis();
  bool expired = (sessionOwnerId.length() == 0) || ((long)(now - sessionExpiresMs) >= 0);
  if (expired) {
    sessionOwnerId = "";
    sessionOwnerLabel = "";
  }
  if (cid.length() == 0) {
    return expired;
  }
  if (expired && claimIfFree) {
    sessionOwnerId = cid;
    sessionOwnerLabel = dev.length() > 0 ? dev : String("Web Client");
    sessionExpiresMs = now + SESSION_LEASE_MS;
    return true;
  }
  if (sessionOwnerId == cid) {
    if (dev.length() > 0) sessionOwnerLabel = dev;
    sessionExpiresMs = now + SESSION_LEASE_MS;
    return true;
  }
  return false;
}

static bool requireControlOwner() {
  noteWebActivity();
  String cid = server.hasArg("cid") ? server.arg("cid") : "";
  String dev = server.hasArg("dev") ? server.arg("dev") : "";
  if (checkOrClaimSession(cid, dev, true)) {
    return true;
  }
  String resp = "{\"ok\":false,\"locked\":true,\"locked_by\":\"" + jEsc(sessionOwnerLabel) + "\"}";
  server.send(409, "application/json", resp);
  return false;
}

// ============================================================================
// Wi-Fi + Fail-Safe SoftAP + NTP + mDNS + OTA
// ============================================================================
static bool connectStaNetwork(const String& ssid, const String& pass, uint32_t timeoutMs) {
  if (ssid.length() == 0) return false;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // Keep low HTTP latency on ESP32-C3 shared radio
  if (cfg.useStatic) {
    IPAddress localIp(192, 168, 1, cfg.ipSuffix);
    IPAddress gw(192, 168, 1, 1);
    IPAddress mask(255, 255, 255, 0);
    IPAddress dns1(8, 8, 8, 8);
    IPAddress dns2(1, 1, 1, 1);
    WiFi.config(localIp, gw, mask, dns1, dns2);
  }
  WiFi.begin(ssid.c_str(), pass.c_str());
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeoutMs) {
    delay(200);
    feedWdt();
  }
  return WiFi.status() == WL_CONNECTED;
}

static void startFallbackAp(bool apOnly) {
  WiFi.mode(apOnly ? WIFI_AP : WIFI_AP_STA);
  IPAddress apIp(192, 168, 4, 1);
  IPAddress apMask(255, 255, 255, 0);
  WiFi.softAPConfig(apIp, apIp, apMask);
  WiFi.softAP(AP_SSID, AP_PASS);
  apModeActive = true;
  logger.add("SYS", String("Fail-Safe AP active: ") + AP_SSID + " (192.168.4.1)");
}

static void setupWifi() {
  WiFi.setHostname("p25-haier");
  bool ok = connectStaNetwork(cfg.ssid1, cfg.pass1, 12000);
  if (!ok && cfg.ssid2.length() > 0) {
    ok = connectStaNetwork(cfg.ssid2, cfg.pass2, 10000);
  }
  if (ok) {
    apModeActive = false;
    logger.add("SYS", "Wi-Fi OK | " + WiFi.SSID() + " (" + WiFi.localIP().toString() + ")");
    configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
    MDNS.begin("p25-haier");
    MDNS.addService("http", "tcp", 80);
  } else {
    startFallbackAp(true);
  }
}

static void setupOta() {
  ArduinoOTA.setHostname("p25-haier");
  ArduinoOTA.setPassword(OTA_PASS);
  ArduinoOTA.onStart([]() {
    logger.add("SYS", "ArduinoOTA update starting...");
    logger.flushIfNeeded(true);
    hist.flushCurrentDay();
    feedWdt();
  });
  ArduinoOTA.onProgress([](unsigned int, unsigned int) {
    feedWdt();
  });
  ArduinoOTA.onEnd([]() {
    feedWdt();
  });
  ArduinoOTA.begin();
}

// ============================================================================
// Unified Compact Status JSON Builder (1-Request Architecture)
// ============================================================================
static String buildStatusJson(const String& cid, const String& dev, int okFlag = -1) {
  bool isOwner = checkOrClaimSession(cid, dev, true);
  unsigned long now = millis();
  long expSec = (sessionOwnerId.length() > 0 && (long)(sessionExpiresMs - now) > 0)
                  ? (long)((sessionExpiresMs - now) / 1000UL)
                  : 0;

  bool online = ac.isOnline();
  unsigned long clElapsed = ac.cl.active ? ((now - ac.cl.startedMs) / 1000UL) : 0;
  long timerRem = (ac.timerActive && (long)(ac.timerFireAtMs - now) > 0)
                    ? (long)((ac.timerFireAtMs - now) / 1000UL)
                    : 0;

  String j;
  j.reserve(2800);
  j = "{";
  if (okFlag >= 0) {
    j += "\"ok\":" + String(okFlag ? "true" : "false") + ",";
  }
  float liveEffRate = computeEffectivePeaRate();
  bool rtActive = (realtimeExpiresMs > 0) && ((long)(realtimeExpiresMs - now) > 0);
  j += "\"ver\":\"" + String(FW_VERSION) + "\",";
  j += "\"poll_mode\":\"" + currentPollMode + "\",";
  j += "\"realtime\":" + String(rtActive ? "true" : "false") + ",";
  j += "\"rules_active\":" + String(rules.activeRuleCount()) + ",";
  j += "\"rules_master\":" + String(rules.masterEnabled ? "true" : "false") + ",";
  j += "\"tariff\":" + String(liveEffRate, 4) + ",";
  j += "\"pea\":{\"type\":\"" + jEsc(cfg.peaType) + "\",\"ft\":" + String(cfg.ftSatang, 2) +
       ",\"vat\":" + String(cfg.peaVat ? "true" : "false") +
       ",\"svc\":" + String(cfg.peaSvc ? "true" : "false") +
       ",\"tier\":" + String(cfg.peaTier) +
       ",\"custom\":" + String(cfg.customRate, 2) +
       ",\"eff\":" + String(liveEffRate, 4) + "},";

  // Session ownership
  j += "\"session\":{";
  j += "\"is_owner\":" + String(isOwner ? "true" : "false") + ",";
  j += "\"owner_label\":\"" + jEsc(sessionOwnerLabel) + "\",";
  j += "\"expires_s\":" + String(expSec) + "},";

  // Network & System
  j += "\"net\":{";
  j += "\"ip\":\"" + (WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : WiFi.softAPIP().toString()) + "\",";
  j += "\"ssid\":\"" + jEsc(WiFi.status() == WL_CONNECTED ? WiFi.SSID() : String(AP_SSID)) + "\",";
  j += "\"rssi\":" + String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0) + ",";
  j += "\"ap\":" + String(apModeActive ? "true" : "false") + ",";
  j += "\"uptime_s\":" + String(now / 1000UL) + ",";
  j += "\"free_heap\":" + String(ESP.getFreeHeap()) + ",";
  j += "\"ntp_ok\":" + String(time(nullptr) > 1700000000 ? "true" : "false") + ",";
  j += "\"time\":\"" + logger.currentTimestamp() + "\"},";

  // AC State
  j += "\"ac\":{";
  j += "\"online\":" + String(online ? "true" : "false") + ",";
  j += "\"valid\":" + String(ac.state.valid ? "true" : "false") + ",";
  j += "\"sim\":" + String(ac.simMode ? "true" : "false") + ",";
  j += "\"rx_pin\":" + String(ac.rxPin) + ",\"tx_pin\":" + String(ac.txPin) + ",";
  j += "\"rx_ok\":" + String(ac.rxOkCount) + ",\"rx_rec_ff\":" + String(ac.rxRecoveredSingleFF) + ",";
  j += "\"rx_err\":" + String(ac.rxErrorCount) + ",\"tx_cnt\":" + String(ac.txCount) + ",";
  j += "\"dev_name\":\"" + jEsc(ac.devInfo.deviceName) + "\",";
  j += "\"proto_ver\":\"" + jEsc(ac.devInfo.protocolVer) + "\",";
  j += "\"sw_ver\":\"" + jEsc(ac.devInfo.softwareVer) + "\",";
  j += "\"hw_ver\":\"" + jEsc(ac.devInfo.hardwareVer) + "\",";
  j += "\"rtt_ms\":" + String(ac.lastRttMs) + ",";
  j += "\"rx_age_ms\":" + String(ac.lastRxMs > 0 ? (now - ac.lastRxMs) : 99999UL) + ",";
  j += "\"last_hex\":\"" + jEsc(ac.lastRxHex) + "\",";
  j += "\"ac_power\":" + String(ac.state.acPower ? "true" : "false") + ",";
  j += "\"ac_mode\":\"" + String(HaierAcController::modeName(ac.state.modeRaw)) + "\",";
  j += "\"mode_raw\":" + String(ac.state.modeRaw) + ",";
  j += "\"fan_mode\":\"" + String(HaierAcController::fanName(ac.state.fanRaw)) + "\",";
  j += "\"fan_mode_raw\":" + String(ac.state.fanRaw) + ",";
  j += "\"target_temp\":" + String((int)roundf(ac.state.targetTemp)) + ",";
  j += "\"vswing\":\"" + String(HaierAcController::vSwingName(ac.state.vSwingRaw)) + "\",";
  j += "\"vswing_raw\":" + String(ac.state.vSwingRaw) + ",";
  j += "\"hswing\":\"" + String(HaierAcController::hSwingName(ac.state.hSwingRaw)) + "\",";
  j += "\"hswing_raw\":" + String(ac.state.hSwingRaw) + ",";
  j += "\"eco_level\":" + String(ac.state.ecoLevel) + ",";
  j += "\"display_status\":" + String(ac.state.displayStatus ? "true" : "false") + ",";
  j += "\"health_mode\":" + String(ac.state.healthMode ? "true" : "false") + ",";
  j += "\"fast_mode\":" + String(ac.state.fastMode ? "true" : "false") + ",";
  j += "\"quiet_mode\":" + String(ac.state.quietMode ? "true" : "false") + ",";
  j += "\"sleep_mode\":" + String(ac.state.sleepMode ? "true" : "false") + ",";
  j += "\"lock_remote\":" + String(ac.state.lockRemote ? "true" : "false") + ",";
  j += "\"beeper_muted\":" + String(ac.stickyBeeperMuted ? "true" : "false") + ",";
  j += "\"self_cleaning\":" + String(ac.state.selfCleaning ? "true" : "false") + ",";
  j += "\"room_temp_c\":" + (!isnan(ac.state.roomTempC) ? String(ac.state.roomTempC, 1) : String("null")) + ",";
  j += "\"room_hum_pct\":" + (ac.state.roomHumPct >= 0 ? String(ac.state.roomHumPct) : String("null")) + ",";
  j += "\"outdoor_temp_c\":" + (ac.state.outdoorTempC > -100 ? String(ac.state.outdoorTempC) : String("null")) + ",";
  j += "\"error_code\":" + String(ac.state.errorCode) + ",";
  j += "\"op_source\":\"" + String(HaierAcController::opSourceName(ac.state.opSource)) + "\",";
  j += "\"alarm_count\":" + String(ac.alarmCount) + ",";
  j += "\"alarm_hex\":\"" + jEsc(ac.lastAlarmHex) + "\",";
  j += "\"comp_waiting\":" + String(ac.isCompWaitingDelay() ? "true" : "false") + ",";
  j += "\"smart_eco\":" + String(ac.smartEcoEnabled ? "true" : "false") + ",";
  j += "\"smart_eco_th\":" + String(ac.smartEcoThresholdC, 1) + ",";
  j += "\"timer_active\":" + String(ac.timerActive ? "true" : "false") + ",";
  j += "\"timer_on\":" + String(ac.timerTurnOn ? "true" : "false") + ",";
  j += "\"timer_rem_s\":" + String(timerRem) + ",";
  j += "\"timer_label\":\"" + jEsc(ac.timerLabel) + "\"},";

  // Inverter BigData (ALL decoded fields for Technician All-Sensors screen)
  j += "\"bigdata\":{";
  j += "\"valid\":" + String(ac.big.valid ? "true" : "false") + ",";
  j += "\"power_w\":" + String(ac.big.powerW) + ",";
  j += "\"indoor_coil_c\":" + (!isnan(ac.big.indoorCoilC) ? String(ac.big.indoorCoilC, 1) : String("null")) + ",";
  j += "\"outdoor_out_air_c\":" + (ac.big.outdoorOutAirC > -100 ? String(ac.big.outdoorOutAirC) : String("null")) + ",";
  j += "\"outdoor_coil_c\":" + (ac.big.outdoorCoilC > -100 ? String(ac.big.outdoorCoilC) : String("null")) + ",";
  j += "\"outdoor_in_air_c\":" + (ac.big.outdoorInAirC > -100 ? String(ac.big.outdoorInAirC) : String("null")) + ",";
  j += "\"outdoor_defrost_c\":" + (ac.big.outdoorDefrostC > -100 ? String(ac.big.outdoorDefrostC) : String("null")) + ",";
  j += "\"comp_freq_hz\":" + String(ac.big.compFreqHz) + ",";
  j += "\"comp_current_a\":" + String(ac.big.compCurrentA, 1) + ",";
  j += "\"comp_status\":" + String(ac.big.compStatus) + ",";
  j += "\"in_fan_status\":" + String(ac.big.inFanStatus) + ",";
  j += "\"out_fan_status\":" + String(ac.big.outFanStatus) + ",";
  j += "\"defrost_status\":" + String(ac.big.defrostStatus) + ",";
  j += "\"four_way_status\":" + String(ac.big.fourWayStatus) + ",";
  j += "\"eev_open\":" + String(ac.big.eevOpen) + "},";

  // Xiaomi BLE
  const XiaomiSensorData& x = xiaomi.data;
  unsigned long xAge = x.valid ? ((now - x.updatedMs) / 1000UL) : 9999UL;
  j += "\"xiaomi\":{";
  j += "\"ok\":" + String(x.valid ? "true" : "false") + ",";
  j += "\"temp_c\":" + (!isnan(x.tempC) ? String(x.tempC, 2) : String("null")) + ",";
  j += "\"humidity_pct\":" + (!isnan(x.humPct) ? String(x.humPct, 1) : String("null")) + ",";
  j += "\"batt_pct\":" + String(x.battPct) + ",";
  j += "\"batt_v\":" + (!isnan(x.battV) ? String(x.battV, 2) : String("null")) + ",";
  j += "\"rssi\":" + String(x.rssi) + ",";
  j += "\"age_s\":" + String(xAge) + ",";
  j += "\"mode\":\"" + jEsc(x.mode) + "\",";
  j += "\"mac\":\"" + jEsc(x.mac) + "\",";
  j += "\"adv_name\":\"" + jEsc(x.advName) + "\",";
  j += "\"passive_pkts\":" + String(x.passivePackets) + ",";
  j += "\"gatt_reads\":" + String(x.gattReads) + ",";
  j += "\"raw\":\"" + jEsc(x.rawHex) + "\",";
  j += "\"err\":\"" + jEsc(x.lastErr) + "\"},";

  // Self-Clean Tracker
  j += "\"cl\":{";
  j += "\"active\":" + String(ac.cl.active ? "true" : "false") + ",";
  j += "\"phase\":\"" + jEsc(ac.cl.phase) + "\",";
  j += "\"post_action\":\"" + jEsc(ac.cl.postAction) + "\",";
  j += "\"elapsed_s\":" + String(clElapsed) + ",";
  j += "\"min_coil_c\":" + (ac.cl.minCoilC < 500.0f ? String(ac.cl.minCoilC, 1) : String("null")) + ",";
  j += "\"peak_pipe_c\":" + (ac.cl.peakPipeC > -100 ? String(ac.cl.peakPipeC) : String("null")) + ",";
  j += "\"peak_power_w\":" + String(ac.cl.peakPowerW) + ",";
  j += "\"unacked\":" + String(ac.cl.unackedDone ? "true" : "false") + ",";
  j += "\"summary\":\"" + jEsc(ac.cl.lastSummary) + "\",";
  j += "\"seq\":" + String(ac.cl.completionSeq) + "},";

  // Config (omit passwords)
  j += "\"cfg\":{";
  j += "\"ssid1\":\"" + jEsc(cfg.ssid1) + "\",";
  j += "\"ssid2\":\"" + jEsc(cfg.ssid2) + "\",";
  j += "\"use_static\":" + String(cfg.useStatic ? "true" : "false") + ",";
  j += "\"ip_suffix\":" + String(cfg.ipSuffix) + ",";
  j += "\"tariff\":" + String(cfg.tariff, 2) + "}";

  j += "}";
  return j;
}

static void sendStatusResponse(int okFlag = -1) {
  String cid = server.hasArg("cid") ? server.arg("cid") : "";
  String dev = server.hasArg("dev") ? server.arg("dev") : "";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buildStatusJson(cid, dev, okFlag));
}

// ============================================================================
// HTTP Routes Registration
// ============================================================================
static void registerRoutes() {
  // Embedded gzipped SPA Web UI
  server.on("/", HTTP_GET, []() {
    noteWebActivity();
    server.sendHeader("Content-Encoding", "gzip");
    server.sendHeader("Cache-Control", "no-cache");
    server.send_P(200, "text/html; charset=utf-8", (const char*)P25_WEB_UI_GZ, P25_WEB_UI_GZ_LEN);
  });

  // Status & Session Takeover (supports ?rt=1 Real-Time 2Hz lease!)
  server.on("/api/status", HTTP_GET, []() {
    noteWebActivity();
    if (server.hasArg("rt")) {
      if (server.arg("rt").toInt() == 1) {
        realtimeExpiresMs = millis() + 12000UL;
      } else {
        realtimeExpiresMs = 0;
      }
    }
    sendStatusResponse(-1);
  });

  // PEA (กฟภ.) Residential Tariff Live Update (Zero Reboot!)
  server.on("/api/pea", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    if (server.hasArg("type")) {
      String tp = server.arg("type");
      if (tp == "1.1.1" || tp == "1.1.2" || tp == "1.2" || tp == "custom") cfg.peaType = tp;
    }
    if (server.hasArg("ft")) {
      float ft = server.arg("ft").toFloat();
      if (!isnan(ft) && ft >= -200.0f && ft <= 500.0f) cfg.ftSatang = ft;
    }
    if (server.hasArg("vat")) cfg.peaVat = (server.arg("vat").toInt() != 0);
    if (server.hasArg("svc")) cfg.peaSvc = (server.arg("svc").toInt() != 0);
    if (server.hasArg("tier")) cfg.peaTier = constrain(server.arg("tier").toInt(), 0, 2);
    if (server.hasArg("custom")) {
      float cr = server.arg("custom").toFloat();
      if (!isnan(cr) && cr >= 0.5f && cr <= 25.0f) cfg.customRate = cr;
    }
    saveConfig();
    logger.add("SYS", "PEA config -> " + cfg.peaType + " Ft=" + String(cfg.ftSatang, 2) + "st eff=" + String(cfg.tariff, 2));
    sendStatusResponse(1);
  });

  server.on("/api/session/takeover", HTTP_POST, []() {
    noteWebActivity();
    String cid = server.hasArg("cid") ? server.arg("cid") : "";
    String dev = server.hasArg("dev") ? server.arg("dev") : "Web Client";
    if (cid.length() > 0) {
      sessionOwnerId = cid;
      sessionOwnerLabel = dev;
      sessionExpiresMs = millis() + SESSION_LEASE_MS;
      logger.add("SYS", "Session takeover -> " + dev);
    }
    sendStatusResponse(1);
  });

  // AC Power (1-Request -> returns full status)
  server.on("/api/ac/power", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    bool on = server.hasArg("on") ? (server.arg("on").toInt() != 0) : !ac.state.acPower;
    bool ok = ac.setPower(on);
    logger.add("CMD", String("Power ") + (on ? "ON" : "OFF") + (ok ? " -> OK" : " -> ERR"));
    sendStatusResponse(ok ? 1 : 0);
  });

  // AC Full Control & Presets (1-Request -> returns full status)
  server.on("/api/ac/control", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    HaierControlState next = ac.state;
    String desc;

    if (server.hasArg("power")) {
      next.acPower = (server.arg("power").toInt() != 0);
      desc += "pwr=" + String(next.acPower ? 1 : 0) + " ";
    }
    if (server.hasArg("mode")) {
      next.modeRaw = (uint8_t)constrain(server.arg("mode").toInt(), 0, 6);
      next.acPower = true;
      desc += "mode=" + String(HaierAcController::modeName(next.modeRaw)) + " ";
    }
    if (server.hasArg("temp")) {
      float t = roundf(constrain(server.arg("temp").toFloat(), 16.0f, 30.0f));
      next.targetTemp = t;
      desc += "temp=" + String((int)t) + " ";
    }
    if (server.hasArg("fan")) {
      uint8_t f = (uint8_t)server.arg("fan").toInt();
      next.fanRaw = f;
      // Haier HSU-15VQRA03T rejects fan speed changes while TURBO or QUIET is active
      next.fastMode = false;
      next.quietMode = false;
      desc += "fan=" + String(HaierAcController::fanName(f)) + " ";
    }
    if (server.hasArg("perf")) {
      String pf = server.arg("perf");
      next.fastMode = (pf == "turbo");
      next.quietMode = (pf == "quiet");
      if (pf == "eco1") next.ecoLevel = 1;
      else if (pf == "eco2") next.ecoLevel = 2;
      else if (pf == "eco3") next.ecoLevel = 3;
      else next.ecoLevel = 0;
      desc += "perf=" + pf + " ";
    }
    if (server.hasArg("eco")) {
      uint8_t e = (uint8_t)constrain(server.arg("eco").toInt(), 0, 3);
      next.ecoLevel = e;
      if (e > 0) { next.fastMode = false; next.quietMode = false; }
      desc += "eco=L" + String(e) + " ";
    }
    if (server.hasArg("fast")) {
      next.fastMode = (server.arg("fast").toInt() != 0);
      if (next.fastMode) { next.quietMode = false; next.ecoLevel = 0; }
      desc += "turbo=" + String(next.fastMode ? 1 : 0) + " ";
    }
    if (server.hasArg("quiet")) {
      next.quietMode = (server.arg("quiet").toInt() != 0);
      if (next.quietMode) { next.fastMode = false; next.ecoLevel = 0; }
      desc += "quiet=" + String(next.quietMode ? 1 : 0) + " ";
    }
    if (server.hasArg("vswing")) {
      next.vSwingRaw = (uint8_t)server.arg("vswing").toInt();
      desc += "vsw=" + String(next.vSwingRaw) + " ";
    }
    if (server.hasArg("hswing")) {
      next.hSwingRaw = (uint8_t)server.arg("hswing").toInt();
      desc += "hsw=" + String(next.hSwingRaw) + " ";
    }
    if (server.hasArg("sleep")) {
      next.sleepMode = (server.arg("sleep").toInt() != 0);
      desc += "sleep=" + String(next.sleepMode ? 1 : 0) + " ";
    }
    if (server.hasArg("health")) {
      next.healthMode = (server.arg("health").toInt() != 0);
      desc += "uv=" + String(next.healthMode ? 1 : 0) + " ";
    }
    if (server.hasArg("display")) {
      next.displayStatus = (server.arg("display").toInt() != 0);
      desc += "led=" + String(next.displayStatus ? 1 : 0) + " ";
    }
    if (server.hasArg("lock")) {
      next.lockRemote = (server.arg("lock").toInt() != 0);
      desc += "lock=" + String(next.lockRemote ? 1 : 0) + " ";
    }
    if (server.hasArg("mute_beep")) {
      bool m = (server.arg("mute_beep").toInt() != 0);
      ac.stickyBeeperMuted = m;
      cfg.stickyMute = m;
      next.beeperMuted = m;
      saveConfig();
      desc += "mute=" + String(m ? 1 : 0) + " ";
    }

    bool ok = ac.sendControlState(next, "web_ctrl");
    // Optional off_min preset companion timer
    if (ok && server.hasArg("off_min")) {
      int om = server.arg("off_min").toInt();
      if (om > 0) ac.setCountdownTimer(false, (uint32_t)om * 60UL, "Preset Off " + String(om) + "m");
    }
    desc.trim();
    logger.add("CMD", "Ctrl: " + desc + (ok ? " -> OK" : " -> ERR"));
    sendStatusResponse(ok ? 1 : 0);
  });

  // Dry & Off 10m
  server.on("/api/ac/dry_off", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    bool ok = ac.startDryAndOff10m();
    logger.add("CMD", String("Dry & Off 10m -> ") + (ok ? "OK" : "ERR"));
    sendStatusResponse(ok ? 1 : 0);
  });

  // Check Alarms 0x73
  server.on("/api/ac/alarms", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    bool ok = ac.checkAlarms();
    sendStatusResponse(ok ? 1 : 0);
  });

  // Countdown Timer (TIME ON / TIME OFF / Cancel)
  server.on("/api/ac/timer", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    if (server.hasArg("cancel") && server.arg("cancel").toInt() == 1) {
      ac.cancelCountdownTimer();
    } else {
      bool turnOn = server.hasArg("on") && (server.arg("on").toInt() != 0);
      uint32_t sec = server.hasArg("sec") ? (uint32_t)max(60L, server.arg("sec").toInt()) : 3600UL;
      String lbl = (turnOn ? "เปิดแอร์ใน " : "ปิดแอร์ใน ") + String(sec / 60UL) + " น.";
      ac.setCountdownTimer(turnOn, sec, lbl);
    }
    sendStatusResponse(1);
  });

  // Self-Clean (CL) Start/Stop, Policy & Completion Ack
  server.on("/api/ac/cl", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    String action = server.hasArg("action") ? server.arg("action") : "start";
    String post   = server.hasArg("post") ? server.arg("post") : cfg.clPost;
    bool ok = false;
    if (action == "start") {
      cfg.clPost = post;
      saveConfig();
      ok = ac.startSelfClean(post);
    } else {
      ok = ac.stopSelfClean();
    }
    sendStatusResponse(ok ? 1 : 0);
  });

  server.on("/api/ac/cl_policy", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    String p = server.hasArg("post") ? server.arg("post") : "restore_prev";
    if (p == "restore_prev" || p == "power_off" || p == "dry_off_10m" || p == "stay_on") {
      cfg.clPost = p;
      ac.cl.postAction = p;
      saveConfig();
      logger.add("CL", "Post-CL policy -> " + p);
    }
    sendStatusResponse(1);
  });

  server.on("/api/ac/cl_ack", HTTP_POST, []() {
    noteWebActivity();
    ac.cl.unackedDone = false;
    sendStatusResponse(1);
  });

  // Legacy Smart-ECO quick toggle
  server.on("/api/ac/smart_eco", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    bool en = server.hasArg("enable") ? (server.arg("enable").toInt() != 0) : !ac.smartEcoEnabled;
    ac.smartEcoEnabled = en;
    cfg.smartEco = en;
    saveConfig();
    logger.add("RULE", String("Quick Auto-ECO -> ") + (en ? "ON" : "OFF"));
    sendStatusResponse(1);
  });

  // ==========================================================================
  // If-Else Automation Rules API
  // ==========================================================================
  server.on("/api/rules", HTTP_GET, []() {
    noteWebActivity();
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", rules.toJson());
  });

  server.on("/api/rules/save", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    int id         = server.hasArg("id") ? server.arg("id").toInt() : 0;
    bool en        = server.hasArg("en") ? (server.arg("en").toInt() != 0) : true;
    uint8_t cond   = (uint8_t)constrain(server.arg("cond").toInt(), 0, 4);
    bool onOnly    = server.hasArg("on_only") ? (server.arg("on_only").toInt() != 0) : true;
    int16_t th     = (int16_t)server.arg("th").toInt();
    uint8_t pwr    = (uint8_t)constrain(server.arg("pwr").toInt(), 0, 2);
    uint8_t mode   = (uint8_t)server.arg("mode").toInt();
    uint8_t temp   = (uint8_t)server.arg("temp").toInt();
    uint8_t fan    = (uint8_t)server.arg("fan").toInt();
    uint8_t perf   = (uint8_t)server.arg("perf").toInt();
    uint16_t cd    = (uint16_t)max(30L, server.hasArg("cd") ? server.arg("cd").toInt() : 180L);
    String name    = server.hasArg("name") ? server.arg("name") : "Auto Rule";
    bool ok = rules.upsertRule(id, en, cond, onOnly, th, pwr, mode, temp, fan, perf, cd, name);
    logger.add("RULE", (ok ? "Saved rule: " : "Rule full: ") + name);
    server.send(ok ? 200 : 400, "application/json", rules.toJson());
  });

  server.on("/api/rules/toggle", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    int id = server.arg("id").toInt();
    bool en = server.arg("en").toInt() != 0;
    rules.toggleRule(id, en);
    logger.add("RULE", "Rule #" + String(id) + (en ? " ENABLED" : " DISABLED"));
    server.send(200, "application/json", rules.toJson());
  });

  server.on("/api/rules/delete", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    int id = server.arg("id").toInt();
    rules.deleteRule(id);
    logger.add("RULE", "Deleted rule #" + String(id));
    server.send(200, "application/json", rules.toJson());
  });

  server.on("/api/rules/master", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    bool en = server.arg("en").toInt() != 0;
    rules.setMasterEnabled(en);
    logger.add("RULE", String("Rules Master -> ") + (en ? "ON" : "OFF"));
    server.send(200, "application/json", rules.toJson());
  });

  server.on("/api/rules/reset", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    rules.resetDefaults();
    logger.add("RULE", "Reset default rule templates");
    server.send(200, "application/json", rules.toJson());
  });

  // ==========================================================================
  // Multi-Overlay History, Energy Cost & Single-Day Deletion API
  // ==========================================================================
  server.on("/api/history", HTTP_GET, []() {
    noteWebActivity();
    String range = server.hasArg("range") ? server.arg("range") : "day";
    int dayOffset = server.hasArg("day_offset") ? constrain(server.arg("day_offset").toInt(), 0, 6) : 0;
    bool online = ac.isOnline();
    bool acOn = online && ac.state.acPower;
    String json = hist.buildHistoryJson(
      range, dayOffset,
      xiaomi.data.valid, xiaomi.data.tempC, xiaomi.data.humPct,
      acOn,
      online ? ac.state.roomTempC : NAN,
      online ? ac.state.outdoorTempC : -999,
      (online && ac.big.valid) ? ac.big.indoorCoilC : NAN,
      (online && ac.big.valid) ? ac.big.outdoorOutAirC : -999,
      (online && ac.big.valid) ? ac.big.powerW : 0,
      (online && ac.big.valid) ? ac.big.compFreqHz : 0,
      online ? ac.state.targetTemp : 25.0f
    );
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", json);
  });

  server.on("/api/history/delete_day", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    uint16_t dayIdx = (uint16_t)server.arg("day_idx").toInt();
    bool ok = hist.deleteDay(dayIdx);
    logger.add("SYS", "Deleted history day " + ClimateHistoryStore::dayIndexToDateStr(dayIdx));
    server.send(200, "application/json", String("{\"ok\":") + (ok ? "true" : "false") + "}");
  });

  server.on("/api/history/clear", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    hist.clearAll();
    logger.add("SYS", "Cleared all climate & energy history");
    server.send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/api/history.csv", HTTP_GET, []() {
    noteWebActivity();
    server.sendHeader("Content-Disposition", "attachment; filename=\"haier_ac_energy_history.csv\"");
    server.send(200, "text/csv; charset=utf-8", hist.buildCsv());
  });

  // ==========================================================================
  // Logs, BLE, Wi-Fi, Settings, Simulator & Reboot
  // ==========================================================================
  server.on("/api/logs", HTTP_GET, []() {
    noteWebActivity();
    String cat = server.hasArg("cat") ? server.arg("cat") : "all";
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", logger.toJson(cat, 60));
  });

  server.on("/api/logs/clear", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    logger.clear();
    server.send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/api/logs/download", HTTP_GET, []() {
    noteWebActivity();
    server.sendHeader("Content-Disposition", "attachment; filename=\"p25_haier_logs.jsonl\"");
    server.send(200, "application/x-ndjson", logger.readFullDownload());
  });

  server.on("/api/ble/scan", HTTP_GET, []() {
    noteWebActivity();
    server.send(200, "application/json", "{\"devices\":" + xiaomi.seenJson() + "}");
  });

  server.on("/api/ble/refresh", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    if (inBleYield) {
      server.send(429, "application/json", "{\"ok\":false,\"err\":\"busy\"}");
      return;
    }
    bool ok = xiaomi.forceGattRead();
    server.send(200, "application/json", String("{\"ok\":") + (ok ? "true" : "false") + ",\"err\":\"" + jEsc(xiaomi.data.lastErr) + "\"}");
  });

  server.on("/api/wifi/scan", HTTP_GET, []() {
    noteWebActivity();
    int n = WiFi.scanNetworks();
    String j = "{\"networks\":[";
    for (int i = 0; i < n && i < 15; i++) {
      if (i > 0) j += ",";
      j += "{\"ssid\":\"" + jEsc(WiFi.SSID(i)) + "\",\"rssi\":" + String(WiFi.RSSI(i)) + "}";
    }
    j += "]}";
    WiFi.scanDelete();
    server.send(200, "application/json", j);
  });

  server.on("/api/settings", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    bool needReboot = false;
    if (server.hasArg("ssid1") && server.arg("ssid1").length() > 0 && server.arg("ssid1") != cfg.ssid1) {
      cfg.ssid1 = server.arg("ssid1"); needReboot = true;
    }
    if (server.hasArg("pass1") && server.arg("pass1").length() > 0) {
      cfg.pass1 = server.arg("pass1"); needReboot = true;
    }
    if (server.hasArg("ssid2") && server.arg("ssid2") != cfg.ssid2) {
      cfg.ssid2 = server.arg("ssid2"); needReboot = true;
    }
    if (server.hasArg("pass2") && server.arg("pass2").length() > 0) {
      cfg.pass2 = server.arg("pass2"); needReboot = true;
    }
    if (server.hasArg("static")) {
      bool st = server.arg("static").toInt() != 0;
      if (st != cfg.useStatic) { cfg.useStatic = st; needReboot = true; }
    }
    if (server.hasArg("suffix")) {
      int sf = constrain(server.arg("suffix").toInt(), 2, 250);
      if (sf != cfg.ipSuffix) { cfg.ipSuffix = sf; needReboot = true; }
    }
    if (server.hasArg("rx") && server.hasArg("tx")) {
      int rx = server.arg("rx").toInt();
      int tx = server.arg("tx").toInt();
      if (rx != cfg.rxPin || tx != cfg.txPin) {
        cfg.rxPin = rx; cfg.txPin = tx;
        ac.begin(cfg.rxPin, cfg.txPin);
      }
    }
    if (server.hasArg("mac") && server.arg("mac").length() == 17) {
      cfg.bleMac = server.arg("mac");
      xiaomi.setTargetMac(cfg.bleMac);
    }
    if (server.hasArg("eco_th")) {
      cfg.smartEcoTh = constrain(server.arg("eco_th").toFloat(), 18.0f, 32.0f);
      ac.smartEcoThresholdC = cfg.smartEcoTh;
    }
    if (server.hasArg("tariff")) {
      float tf = server.arg("tariff").toFloat();
      if (tf >= 0.5f && tf <= 25.0f) {
        cfg.customRate = tf;
        cfg.tariff = computeEffectivePeaRate();
        hist.tariffBahtPerKwh = cfg.tariff;
      }
    }
    if (server.hasArg("pea_type")) {
      String tp = server.arg("pea_type");
      if (tp == "1.1.1" || tp == "1.1.2" || tp == "1.2" || tp == "custom") cfg.peaType = tp;
    }
    if (server.hasArg("ft")) {
      float ft = server.arg("ft").toFloat();
      if (!isnan(ft) && ft >= -200.0f && ft <= 500.0f) cfg.ftSatang = ft;
    }
    if (server.hasArg("pea_vat")) cfg.peaVat = (server.arg("pea_vat").toInt() != 0);
    if (server.hasArg("pea_svc")) cfg.peaSvc = (server.arg("pea_svc").toInt() != 0);
    if (server.hasArg("pea_tier")) cfg.peaTier = constrain(server.arg("pea_tier").toInt(), 0, 2);
    saveConfig();
    logger.add("SYS", needReboot ? "Settings saved -> rebooting" : "Settings updated live");
    logger.flushIfNeeded(true);
    hist.flushCurrentDay();
    server.send(200, "application/json", String("{\"ok\":true,\"reboot\":") + (needReboot ? "true" : "false") + "}");
    if (needReboot) {
      delay(400);
      ESP.restart();
    }
  });

  server.on("/api/wifi/force_ap", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    server.send(200, "application/json", "{\"ok\":true}");
    delay(300);
    startFallbackAp(false);
  });

  server.on("/api/debug/sim", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    bool en = server.hasArg("enable") && (server.arg("enable").toInt() != 0);
    ac.setSimMode(en);
    sendStatusResponse(1);
  });

  server.on("/api/reboot", HTTP_POST, []() {
    if (!requireControlOwner()) return;
    logger.add("SYS", "Manual reboot requested");
    logger.flushIfNeeded(true);
    hist.flushCurrentDay();
    server.send(200, "application/json", "{\"ok\":true}");
    delay(400);
    ESP.restart();
  });

  // Web OTA at /update (compatible with ota_flash.py and browser uploads)
  Update.onProgress([](size_t, size_t) {
    feedWdt();
  });
  httpUpdater.setup(&server, "/update");
}

// ============================================================================
// Setup & Adaptive Main Loop
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(150);

  // Configure 30s Hardware Task Watchdog for 24/7 rock-solid self-recovery
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t wdtCfg = {
    .timeout_ms = 30000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_reconfigure(&wdtCfg);
  esp_task_wdt_add(NULL);
#else
  esp_task_wdt_init(30, true);
  esp_task_wdt_add(NULL);
#endif

  setenv("TZ", "ICT-7", 1);
  tzset();

  logger.begin();
  loadConfig();
  hist.begin();
  rules.begin();

  ac.onLog = [](const String& cat, const String& msg) {
    logger.add(cat, msg);
  };
  ac.begin(cfg.rxPin, cfg.txPin);

  setupWifi();
  setupOta();

  // Keep HTTP responsive & WDT fed even if stock Xiaomi GATT fallback runs
  xiaomi.onYield = []() {
    inBleYield = true;
    server.handleClient();
    ArduinoOTA.handle();
    feedWdt();
    inBleYield = false;
  };
  xiaomi.begin(cfg.bleMac);

  registerRoutes();
  server.begin();

  // Initial AC probe & status poll
  ac.probeVersion();
  ac.pollStatus();
  lastAcPollMs = millis();

  logger.add("SYS", String("Boot v") + FW_VERSION + " | RX=" + cfg.rxPin + " TX=" + cfg.txPin +
                    " | IP=" + (WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "192.168.4.1"));
  logger.flushIfNeeded(true);
}

void loop() {
  feedWdt();
  server.handleClient();
  ArduinoOTA.handle();
  xiaomi.loop();

  unsigned long now = millis();

  // Determine Adaptive Poll Mode & Cadence (supports Technician Real-Time 2Hz Stream!)
  unsigned long pollInterval = 60000UL;
  bool rtMode = (realtimeExpiresMs > 0) && ((long)(realtimeExpiresMs - now) > 0);
  if (rtMode) {
    currentPollMode = "REALTIME";
    pollInterval = 450UL; // Max safe 9600bps UART cadence (~2 Hz)
    xiaomi.setEcoIdle(false);
    xiaomi.gattFallbackEnabled = false; // Zero GATT stalls during live technician telemetry!
  } else {
    xiaomi.gattFallbackEnabled = true;
    if (ac.cl.active || ac.state.selfCleaning) {
      currentPollMode = "CL_LOCK";
      pollInterval = 5000UL;
      xiaomi.setEcoIdle(false);
    } else if (lastWebClientMs > 0 && (now - lastWebClientMs) < 45000UL) {
      currentPollMode = "ACTIVE";
      pollInterval = 1800UL;
      xiaomi.setEcoIdle(false);
    } else if (ac.state.valid && ac.state.acPower) {
      currentPollMode = "ON_IDLE";
      pollInterval = 15000UL;
      xiaomi.setEcoIdle(false);
    } else {
      currentPollMode = "ECO_IDLE";
      pollInterval = 60000UL;
      xiaomi.setEcoIdle(true);
    }
  }

  // Adaptive UART Status Poll (or instant wake poll on browser return)
  if (fastWakePoll || (now - lastAcPollMs) >= pollInterval) {
    fastWakePoll = false;
    lastAcPollMs = now;
    ac.pollStatus();
    ac.processDeferredActions();

    if (!ac.devInfo.valid && ac.isOnline() && (now - lastProbeMs) > 20000UL) {
      lastProbeMs = now;
      ac.probeVersion();
    }
  } else if (ac.pendingPostClAction || ac.timerActive) {
    ac.processDeferredActions();
  }

  // Time-series & Energy Meter observation (10s active/ON, 30s Eco-Idle)
  unsigned long obsInterval = (currentPollMode == "ECO_IDLE") ? 30000UL : 10000UL;
  if ((now - lastHistObsMs) >= obsInterval) {
    lastHistObsMs = now;
    bool online = ac.isOnline();
    hist.observe(
      xiaomi.data.valid, xiaomi.data.tempC, xiaomi.data.humPct,
      online, ac.state.acPower, ac.state.modeRaw, ac.state.targetTemp,
      ac.state.fastMode, ac.state.ecoLevel,
      online ? ac.state.roomTempC : NAN,
      online ? ac.state.outdoorTempC : -999,
      (online && ac.big.valid) ? ac.big.indoorCoilC : NAN,
      (online && ac.big.valid) ? ac.big.outdoorOutAirC : -999,
      (online && ac.big.valid) ? ac.big.powerW : 0,
      (online && ac.big.valid) ? ac.big.compFreqHz : 0
    );
  }

  // Autonomous If-Else Automation Rules & Quick Smart-ECO (every 6s)
  if ((now - lastRulesEvalMs) >= 6000UL) {
    lastRulesEvalMs = now;
    if (ac.isOnline()) {
      rules.evaluate(ac, xiaomi.data.valid, xiaomi.data.tempC, xiaomi.data.humPct);
      ac.runSmartEcoCheck(xiaomi.data.valid ? xiaomi.data.tempC : NAN);
    }
  }

  // Deferred Batch LittleFS Log Flush (zero-stall!)
  logger.flushIfNeeded(false);

  // Wi-Fi Watchdog & Auto-Reconnect
  if ((now - lastWifiCheckMs) >= 15000UL) {
    lastWifiCheckMs = now;
    if (WiFi.status() != WL_CONNECTED) {
      if (wifiDownSinceMs == 0) wifiDownSinceMs = now;
      if (!apModeActive) {
        WiFi.reconnect();
        if ((now - wifiDownSinceMs) > 90000UL) {
          startFallbackAp(false);
        }
      }
    } else {
      wifiDownSinceMs = 0;
    }
  }

  delay(4);
}
