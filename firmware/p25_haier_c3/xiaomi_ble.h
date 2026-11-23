#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <functional>

// ============================================================================
// Hybrid Xiaomi LYWSD03MMC / ATC_v59 BLE Reader (NimBLE-Arduino 2.5.2)
//   Mode 1 (Preferred, Zero-Connection Passive Broadcast):
//     - BTHome v2   (0xFCD2, unencrypted 0x40 - DEFAULT of ATC_v59.bin!)
//     - pvvx Custom (0x181A, 15 bytes)
//     - ATC1441     (0x181A, 13 bytes)
//     - MiBeacon    (0xFE95, unencrypted Mi-Like)
//   Mode 2 (Auto-Fallback for Stock 2.1.1_0159):
//     - Only runs if NO passive broadcast packets have ever arrived
//     - Automatically disables permanently as soon as passive broadcast starts!
// ============================================================================

struct XiaomiSensorData {
  bool valid = false;
  float tempC = NAN;
  float humPct = NAN;
  float battV = NAN;
  int battPct = -1;
  int rssi = 0;
  String mode = "waiting"; // "bthome_v2", "pvvx_custom", "atc1441", "mi_like", "stock_gatt"
  String mac = "aa:bb:cc:dd:ee:ff";
  String advName = "";
  String rawHex = "";
  unsigned long updatedMs = 0;
  unsigned long lastPassiveMs = 0;
  uint32_t passivePackets = 0;
  uint32_t gattReads = 0;
  String lastErr = "";
};

struct BleDiscoveredItem {
  String mac;
  String name;
  int rssi;
  String svcInfo;
  bool hasEnvData;
  unsigned long seenMs;
};

class XiaomiBleReader : public NimBLEScanCallbacks {
public:
  XiaomiSensorData data;
  String targetMac = "aa:bb:cc:dd:ee:ff";
  bool gattFallbackEnabled = true;
  bool ecoIdleMode = false;
  std::function<void()> onYield = nullptr;

  static const int MAX_SEEN = 15;
  BleDiscoveredItem seen[MAX_SEEN];
  int seenCount = 0;

  void begin(const String& mac) {
    setTargetMac(mac);
    NimBLEDevice::init("P25-Haier-C3");
    NimBLEDevice::setPower(ESP_PWR_LVL_P6);

    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setScanCallbacks(this, true);  // wantDuplicates=true for live passive updates
    pScan->setActiveScan(false);          // PASSIVE scan! Receives ADV_NONCONN_IND (ATC_v59 BTHome) & minimal Wi-Fi conflict
    pScan->setInterval(320);              // 12.5% duty cycle (40/320) -> gives Wi-Fi 87.5% of shared 2.4GHz radio!
    pScan->setWindow(40);
    pScan->setMaxResults(0);
    startScan();
  }

  void setEcoIdle(bool idle) {
    if (ecoIdleMode == idle) return;
    ecoIdleMode = idle;
    NimBLEScan* pScan = NimBLEDevice::getScan();
    if (pScan->isScanning()) pScan->stop();
    pScan->setInterval(idle ? 640 : 320); // 6.2% in Eco-Idle vs 12.5% in Active
    pScan->setWindow(40);
    startScan();
  }

  void setTargetMac(const String& mac) {
    String m = mac;
    m.trim();
    m.toLowerCase();
    if (m.length() == 17) {
      targetMac = m;
      data.mac = m;
    }
  }

  void startScan() {
    NimBLEScan* pScan = NimBLEDevice::getScan();
    if (!pScan->isScanning()) {
      pScan->start(30000, false, true);
    }
  }

  void loop() {
    unsigned long now = millis();
    if (!NimBLEDevice::getScan()->isScanning()) {
      startScan();
    }

    // If passive broadcast has been seen recently (ATC_v59), NEVER block on GATT connect!
    bool hasPassive = (data.lastPassiveMs > 0) && ((now - data.lastPassiveMs) < 300000UL);
    if (gattFallbackEnabled && !hasPassive && targetMac.length() == 17) {
      unsigned long interval = ecoIdleMode ? 180000UL : (data.valid ? 120000UL : 30000UL);
      if ((now - lastGattAttemptMs) >= interval && now > 8000UL) {
        lastGattAttemptMs = now;
        readStockViaGatt();
      }
    }
  }

  bool forceGattRead() {
    lastGattAttemptMs = millis();
    return readStockViaGatt();
  }

  void onDiscovered(const NimBLEAdvertisedDevice* dev) override {
    handleAdv(dev);
  }

  void onResult(const NimBLEAdvertisedDevice* dev) override {
    handleAdv(dev);
  }

  void handleAdv(const NimBLEAdvertisedDevice* dev) {
    String mac = String(dev->getAddress().toString().c_str());
    mac.toLowerCase();
    int rssi = dev->getRSSI();
    String name = dev->haveName() ? String(dev->getName().c_str()) : "";

    bool decodedEnv = false;
    bool isTarget = (mac == targetMac);
    String svcSummary;

    uint8_t svcCount = dev->getServiceDataCount();
    for (uint8_t i = 0; i < svcCount; i++) {
      NimBLEUUID su = dev->getServiceDataUUID(i);
      std::string sd = dev->getServiceData(i);
      if (sd.empty()) continue;

      if (svcSummary.length() > 0) svcSummary += " ";
      svcSummary += String(su.toString().c_str()) + ":" + toHex((const uint8_t*)sd.data(), min(sd.size(), (size_t)12));

      if (su == NimBLEUUID((uint16_t)0xFCD2)) {
        if (isTarget && decodeBTHomeV2((const uint8_t*)sd.data(), sd.size(), rssi, name)) {
          decodedEnv = true;
        } else if (sd.size() >= 4 && ((uint8_t)sd[0] & 0xE0) == 0x40) {
          decodedEnv = true;
        }
      } else if (su == NimBLEUUID((uint16_t)0x181A)) {
        if (isTarget && decode0x181A((const uint8_t*)sd.data(), sd.size(), rssi, name)) {
          decodedEnv = true;
        } else if (sd.size() == 13 || sd.size() == 15) {
          decodedEnv = true;
        }
      } else if (su == NimBLEUUID((uint16_t)0xFE95)) {
        if (isTarget && decodeMiBeaconPlain((const uint8_t*)sd.data(), sd.size(), rssi, name)) {
          decodedEnv = true;
        } else {
          decodedEnv = true;
        }
      }
    }

    if (isTarget) {
      hasObservedTarget = true;
      observedTargetAddr = dev->getAddress();
      data.rssi = rssi;
      if (name.length() > 0) data.advName = name;
    }

    // Record target, env sensors, ATC/LYWSD names, or any strong nearby device
    if (isTarget || decodedEnv || name.indexOf("ATC") >= 0 || name.indexOf("LYWSD") >= 0 || svcCount > 0) {
      recordSeen(mac, name, rssi, svcSummary, decodedEnv);
    }
  }

  void onScanEnd(const NimBLEScanResults& results, int reason) override {
    (void)results; (void)reason;
  }

  String seenJson() const {
    String j = "[";
    for (int i = 0; i < seenCount; i++) {
      if (i > 0) j += ",";
      j += "{\"mac\":\"" + seen[i].mac + "\",";
      j += "\"name\":\"" + seen[i].name + "\",";
      j += "\"rssi\":" + String(seen[i].rssi) + ",";
      j += "\"svc\":\"" + seen[i].svcInfo + "\",";
      j += "\"env\":" + String(seen[i].hasEnvData ? "true" : "false") + ",";
      j += "\"age_s\":" + String((millis() - seen[i].seenMs) / 1000UL) + "}";
    }
    j += "]";
    return j;
  }

private:
  unsigned long lastGattAttemptMs = 0;
  bool hasObservedTarget = false;
  NimBLEAddress observedTargetAddr;

  static int voltageToPct(float v) {
    if (isnan(v)) return -1;
    int pct = (int)roundf((v - 2.1f) / (3.1f - 2.1f) * 100.0f);
    return constrain(pct, 0, 100);
  }

  static String toHex(const uint8_t* p, size_t n) {
    String out;
    char b[3];
    for (size_t i = 0; i < n && i < 24; i++) {
      snprintf(b, sizeof(b), "%02x", p[i]);
      out += b;
    }
    return out;
  }

  void recordSeen(const String& mac, const String& name, int rssi, const String& svc, bool hasEnv) {
    unsigned long now = millis();
    for (int i = 0; i < seenCount; i++) {
      if (seen[i].mac == mac) {
        if (name.length() > 0) seen[i].name = name;
        seen[i].rssi = rssi;
        if (svc.length() > 0) seen[i].svcInfo = svc;
        seen[i].hasEnvData = seen[i].hasEnvData || hasEnv;
        seen[i].seenMs = now;
        return;
      }
    }
    int idx = (seenCount < MAX_SEEN) ? seenCount++ : (MAX_SEEN - 1);
    seen[idx] = {mac, name, rssi, svc, hasEnv, now};
  }

  bool decode0x181A(const uint8_t* p, size_t n, int rssi, const String& name) {
    if (n == 15) {
      int16_t tRaw = (int16_t)(p[6] | (p[7] << 8));
      uint16_t hRaw = (uint16_t)(p[8] | (p[9] << 8));
      uint16_t mv = (uint16_t)(p[10] | (p[11] << 8));
      data.tempC = tRaw * 0.01f;
      data.humPct = hRaw * 0.01f;
      data.battV = mv * 0.001f;
      data.battPct = p[12];
      data.rssi = rssi;
      if (name.length() > 0) data.advName = name;
      data.mode = "pvvx_custom";
      data.rawHex = toHex(p, n);
      data.valid = true;
      data.updatedMs = millis();
      data.lastPassiveMs = data.updatedMs;
      data.passivePackets++;
      data.lastErr = "";
      return true;
    } else if (n == 13) {
      int16_t tRaw = (int16_t)((p[6] << 8) | p[7]);
      uint16_t mv = (uint16_t)((p[10] << 8) | p[11]);
      data.tempC = tRaw * 0.1f;
      data.humPct = (float)p[8];
      data.battPct = p[9];
      data.battV = mv * 0.001f;
      data.rssi = rssi;
      if (name.length() > 0) data.advName = name;
      data.mode = "atc1441";
      data.rawHex = toHex(p, n);
      data.valid = true;
      data.updatedMs = millis();
      data.lastPassiveMs = data.updatedMs;
      data.passivePackets++;
      data.lastErr = "";
      return true;
    }
    return false;
  }

  bool decodeBTHomeV2(const uint8_t* p, size_t n, int rssi, const String& name) {
    if (n < 4) return false;
    uint8_t info = p[0];
    if ((info & 0xE0) != 0x40) return false; // BTHome v2
    if (info & 0x01) {
      data.lastErr = "bthome_encrypted";
      return false;
    }

    float t = NAN, h = NAN, v = NAN;
    int bp = -1;
    size_t i = 1;
    while (i < n) {
      uint8_t objId = p[i++];
      if (objId == 0x00 && i + 1 <= n) { i += 1; }
      else if (objId == 0x01 && i + 1 <= n) { bp = p[i]; i += 1; }
      else if (objId == 0x02 && i + 2 <= n) { int16_t tv = (int16_t)(p[i] | (p[i + 1] << 8)); t = tv * 0.01f; i += 2; }
      else if (objId == 0x03 && i + 2 <= n) { uint16_t hv = (uint16_t)(p[i] | (p[i + 1] << 8)); h = hv * 0.01f; i += 2; }
      else if (objId == 0x0C && i + 2 <= n) { uint16_t mv = (uint16_t)(p[i] | (p[i + 1] << 8)); v = mv * 0.001f; i += 2; }
      else if (objId == 0x2E && i + 1 <= n) { h = (float)p[i]; i += 1; }
      else if (objId == 0x45 && i + 2 <= n) { int16_t tv = (int16_t)(p[i] | (p[i + 1] << 8)); t = tv * 0.1f; i += 2; }
      else break;
    }

    if (!isnan(t) || !isnan(h) || !isnan(v) || bp >= 0) {
      if (!isnan(t)) data.tempC = t;
      if (!isnan(h)) data.humPct = h;
      if (!isnan(v)) data.battV = v;
      if (bp >= 0) data.battPct = bp;
      else if (!isnan(v)) data.battPct = voltageToPct(v);
      data.rssi = rssi;
      if (name.length() > 0) data.advName = name;
      data.mode = "bthome_v2";
      data.rawHex = toHex(p, n);
      data.valid = !isnan(data.tempC);
      data.updatedMs = millis();
      data.lastPassiveMs = data.updatedMs;
      data.passivePackets++;
      data.lastErr = "";
      return true;
    }
    return false;
  }

  bool decodeMiBeaconPlain(const uint8_t* p, size_t n, int rssi, const String& name) {
    // Unencrypted MiBeacon v5 (e.g. pvvx Mi-Like mode): frameCtrl[0..1], devId[2..3], cnt[4], mac[5..10], objId[11..12], len[13], val...
    if (n < 15) return false;
    uint16_t fc = (uint16_t)(p[0] | (p[1] << 8));
    if (fc & 0x0008) return false; // Encrypted -> skip
    if (!(fc & 0x0040)) return false; // No object data
    size_t pos = 5;
    if (fc & 0x0010) pos += 6; // MAC present
    if (fc & 0x0020) pos += 1; // Capability present
    if (pos + 3 > n) return false;

    uint16_t objId = (uint16_t)(p[pos] | (p[pos + 1] << 8));
    uint8_t objLen = p[pos + 2];
    pos += 3;
    if (pos + objLen > n) return false;

    bool got = false;
    if (objId == 0x1004 && objLen == 2) {
      data.tempC = (int16_t)(p[pos] | (p[pos + 1] << 8)) * 0.1f;
      got = true;
    } else if (objId == 0x1006 && objLen == 2) {
      data.humPct = (uint16_t)(p[pos] | (p[pos + 1] << 8)) * 0.1f;
      got = true;
    } else if (objId == 0x100A && objLen == 1) {
      data.battPct = p[pos];
      got = true;
    } else if (objId == 0x100D && objLen == 4) {
      data.tempC = (int16_t)(p[pos] | (p[pos + 1] << 8)) * 0.1f;
      data.humPct = (uint16_t)(p[pos + 2] | (p[pos + 3] << 8)) * 0.1f;
      got = true;
    }

    if (got) {
      data.rssi = rssi;
      if (name.length() > 0) data.advName = name;
      data.mode = "mi_like";
      data.rawHex = toHex(p, n);
      data.valid = !isnan(data.tempC);
      data.updatedMs = millis();
      data.lastPassiveMs = data.updatedMs;
      data.passivePackets++;
      data.lastErr = "";
      return true;
    }
    return false;
  }

  bool readStockViaGatt() {
    NimBLEDevice::getScan()->stop();
    delay(40);

    NimBLEAddress addr = hasObservedTarget
                           ? observedTargetAddr
                           : NimBLEAddress(std::string(targetMac.c_str()), BLE_ADDR_PUBLIC);
    NimBLEClient* pClient = NimBLEDevice::getDisconnectedClient();
    if (!pClient) pClient = NimBLEDevice::createClient();
    if (!pClient) {
      data.lastErr = "gatt_no_client";
      startScan();
      return false;
    }

    pClient->setConnectTimeout(6500);
    if (onYield) onYield();
    if (!pClient->connect(addr, true)) {
      data.lastErr = "gatt_connect_timeout";
      NimBLEDevice::deleteClient(pClient);
      startScan();
      return false;
    }
    if (onYield) onYield();

    bool gotSample = false;
    int rssi = pClient->getRssi();

    NimBLERemoteService* pSvc = pClient->getService("ebe0ccb0-7a0a-4b0c-8a1a-6ff2997da3a6");
    if (pSvc) {
      NimBLERemoteCharacteristic* pChr = pSvc->getCharacteristic("ebe0ccc1-7a0a-4b0c-8a1a-6ff2997da3a6");
      if (pChr && pChr->canNotify()) {
        volatile bool notified = false;
        uint8_t sampleBuf[8] = {0};
        size_t sampleLen = 0;

        auto cb = [&](NimBLERemoteCharacteristic* chr, uint8_t* pData, size_t len, bool isNotify) {
          (void)chr; (void)isNotify;
          if (len >= 5 && !notified) {
            sampleLen = min(len, sizeof(sampleBuf));
            memcpy(sampleBuf, pData, sampleLen);
            notified = true;
          }
        };

        if (pChr->subscribe(true, cb, true)) {
          unsigned long t0 = millis();
          while (!notified && (millis() - t0) < 8500UL) {
            if (onYield) onYield();
            delay(15);
          }
          pChr->unsubscribe(false);

          if (notified && sampleLen >= 5) {
            int16_t rawT = (int16_t)(sampleBuf[0] | (sampleBuf[1] << 8));
            uint8_t rawH = sampleBuf[2];
            uint16_t rawMv = (uint16_t)(sampleBuf[3] | (sampleBuf[4] << 8));
            data.tempC = rawT * 0.01f;
            data.humPct = (float)rawH;
            data.battV = rawMv * 0.001f;
            data.battPct = voltageToPct(data.battV);
            data.rssi = rssi;
            data.mode = "stock_gatt";
            data.rawHex = toHex(sampleBuf, sampleLen);
            data.valid = true;
            data.updatedMs = millis();
            data.gattReads++;
            data.lastErr = "";
            gotSample = true;
          } else {
            data.lastErr = "gatt_notify_timeout";
          }
        } else {
          data.lastErr = "gatt_subscribe_fail";
        }
      } else {
        data.lastErr = "gatt_char_missing";
      }
    } else {
      data.lastErr = "gatt_svc_missing";
    }

    pClient->disconnect();
    delay(40);
    NimBLEDevice::deleteClient(pClient);
    startScan();
    return gotSample;
  }
};
