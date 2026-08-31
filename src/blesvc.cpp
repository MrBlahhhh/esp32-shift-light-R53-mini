#include "blesvc.h"
#include "proto.h"
#include "settings.h"
#include "canbus.h"
#include "shiftlight.h"
#include <Arduino.h>
#include <NimBLEDevice.h>

static NimBLECharacteristic* s_config    = nullptr;
static NimBLECharacteristic* s_telemetry = nullptr;
static NimBLECharacteristic* s_canframe  = nullptr;
static bool s_connected = false;

// Deferred work from BLE callbacks. NimBLE runs these on its own host task, and
// doing anything slow there — an NVS commit, a blocking LED flash — stalls the
// stack and gets the connection dropped. The callback records the intent; the
// main loop carries it out.
static volatile bool s_wantSave     = false;
static volatile bool s_wantIdentify = false;
static volatile bool s_wantReboot   = false;

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* /*s*/, NimBLEConnInfo& /*info*/) override {
    s_connected = true;
    Serial.println("BLE: connected");
  }
  void onDisconnect(NimBLEServer* /*s*/, NimBLEConnInfo& /*info*/, int /*reason*/) override {
    s_connected = false;
    // Streaming is per-connection state. Leaving it on would keep filling the
    // ring for a phone that has gone, so the next client inherits a backlog of
    // frames from someone else's session.
    canSetStream(SL_STREAM_OFF);
    Serial.println("BLE: disconnected, advertising again");
    NimBLEDevice::startAdvertising();
  }
};

class ConfigCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& /*info*/) override {
    NimBLEAttValue v = c->getValue();
    if (!settingsApply(v.data(), v.size())) {
      Serial.printf("BLE: config rejected (%u bytes)\n", (unsigned)v.size());
      // Overwrite the attribute with what is actually running. A rejected write
      // otherwise leaves the phone's value sitting in the characteristic, and
      // the next read hands it back as though it had been accepted.
      blePublishConfig();
      return;
    }
    Serial.println("BLE: config applied (not yet saved)");
  }
};

class CommandCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& /*info*/) override {
    NimBLEAttValue v = c->getValue();
    if (v.size() < 1) return;
    const uint8_t* p = v.data();

    switch (p[0]) {
      case SL_CMD_SAVE:
        s_wantSave = true;
        break;

      case SL_CMD_DEFAULTS:
        settingsDefaults();
        blePublishConfig();
        break;

      case SL_CMD_STREAM:
        if (v.size() >= 2) canSetStream(p[1]);
        break;

      case SL_CMD_FILTER: {
        uint32_t ids[SL_MAX_FILTER];
        size_t n = (v.size() - 1) / 4;
        if (n > SL_MAX_FILTER) n = SL_MAX_FILTER;
        for (size_t i = 0; i < n; i++) {
          memcpy(&ids[i], p + 1 + i * 4, 4);
        }
        canSetFilter(ids, n);
        break;
      }

      case SL_CMD_REBOOT:
        s_wantReboot = true;
        break;

      case SL_CMD_IDENTIFY:
        s_wantIdentify = true;
        break;

      default:
        break;
    }
  }
};

void blePublishConfig() {
  if (s_config) s_config->setValue((uint8_t*)&cfg, sizeof(cfg));
}

void bleBegin() {
  NimBLEDevice::init(SL_DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  // The frame stream is the only thing here that needs a big MTU; at the
  // default 23 a full batch would fragment into ten packets and arrive slower
  // than the bus produces it.
  NimBLEDevice::setMTU(247);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  NimBLEService* service = server->createService(SL_SERVICE_UUID);

  s_config = service->createCharacteristic(
      SL_CONFIG_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  s_config->setCallbacks(new ConfigCallbacks());
  blePublishConfig();

  s_telemetry = service->createCharacteristic(
      SL_TELEMETRY_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  s_canframe = service->createCharacteristic(
      SL_CANFRAME_UUID, NIMBLE_PROPERTY::NOTIFY);

  NimBLECharacteristic* cmd = service->createCharacteristic(
      SL_COMMAND_UUID, NIMBLE_PROPERTY::WRITE);
  cmd->setCallbacks(new CommandCallbacks());

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SL_SERVICE_UUID);
  adv->enableScanResponse(true);
  // Set explicitly. NimBLEDevice::init() names the GAP service but does not put
  // the name on air by itself, and a scan from a PC shows this board advertising
  // its UUID with an empty name unless this call is here. The app matches on the
  // service UUID for exactly that reason, but a nameless device is miserable to
  // find in nRF Connect when something needs debugging.
  adv->setName(SL_DEVICE_NAME);
  adv->start();

  Serial.printf("BLE: advertising as %s\n", SL_DEVICE_NAME);
}

bool bleConnected() { return s_connected; }

void blePoll(uint16_t rpm) {
  // Deferred callback work, on the main task where blocking is safe.
  if (s_wantSave) {
    s_wantSave = false;
    Serial.println(settingsSave() ? "settings: saved" : "settings: SAVE FAILED");
    blePublishConfig();
  }
  if (s_wantIdentify) {
    s_wantIdentify = false;
    shiftlightIdentify();
  }
  if (s_wantReboot) {
    s_wantReboot = false;
    Serial.println("rebooting on request");
    delay(100);
    ESP.restart();
  }

  if (!s_connected) return;

  static uint32_t lastTlm = 0;
  uint32_t now = millis();
  if (now - lastTlm >= (1000 / TELEMETRY_HZ)) {
    lastTlm = now;
    bool sim = cfg.flags & SL_FLAG_SIMULATE;
    TelemetryBlob t;
    t.rpm   = rpm;
    t.flags = 0;
    if (canUp())                    t.flags |= SL_TLM_CAN_UP;
    if (sim || canRpmFresh())       t.flags |= SL_TLM_RPM_FRESH;
    if (sim)                        t.flags |= SL_TLM_SIMULATING;
    if (settingsUnsaved()) t.flags |= SL_TLM_UNSAVED;
    t.ledLevel     = shiftlightLevel();
    t.framesPerSec = canFramesPerSec();
    t.rxMissed     = canRxMissed();
    t.uptimeSec    = now / 1000;
    s_telemetry->setValue((uint8_t*)&t, sizeof(t));
    s_telemetry->notify();
  }

  // One batch per loop, not a drain-until-empty. A flood on the bus would
  // otherwise keep this function running until the ring emptied, and the strip
  // would visibly stutter while the phone caught up.
  CanFrameRec recs[SL_FRAMES_PER_PKT];
  size_t n = canDrainFrames(recs, SL_FRAMES_PER_PKT);
  if (n > 0) {
    uint8_t pkt[1 + SL_FRAMES_PER_PKT * sizeof(CanFrameRec)];
    pkt[0] = (uint8_t)n;
    memcpy(pkt + 1, recs, n * sizeof(CanFrameRec));
    s_canframe->setValue(pkt, 1 + n * sizeof(CanFrameRec));
    s_canframe->notify();
  }
}
