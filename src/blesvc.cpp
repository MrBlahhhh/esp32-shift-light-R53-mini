#include "blesvc.h"
#include "proto.h"
#include "settings.h"
#include "canbus.h"
#include "shiftlight.h"
#include "vtpsvc.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include "vtp1_generated.h"   // VTP_SERVICE_UUID, for the scan response

static NimBLECharacteristic* s_config    = nullptr;
static NimBLECharacteristic* s_telemetry = nullptr;
static NimBLECharacteristic* s_canframe  = nullptr;
static bool s_connected = false;

// Negotiated ATT MTU for the current connection. 23 is the spec minimum and the
// value in force until the exchange completes, which happens shortly after
// connect -- so the first batch or two go out small and then the rest run full
// size. Cached rather than queried per loop because getPeerMTU wants a
// connection handle and this board only ever talks to one phone.
static volatile uint16_t s_mtu = 23;

// Deferred work from BLE callbacks. NimBLE runs these on its own host task, and
// doing anything slow there — an NVS commit, a blocking LED flash — stalls the
// stack and gets the connection dropped. The callback records the intent; the
// main loop carries it out.
static volatile bool s_wantSave     = false;
static volatile bool s_wantIdentify = false;
static volatile bool s_wantReboot   = false;

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
    s_connected = true;
    uint16_t h = info.getConnHandle();
    // Ask for a link both protocols want and neither can insist on. A flat
    // 15 ms interval, no peripheral latency, 4 s supervision timeout: the
    // interval is the unit of cost for every notification this board sends, and
    // the default timeout can run to 20 s, which is 20 s of neither connected
    // nor advertising after a dropout. VTP §2.3 asks for exactly this and then
    // says the central decides, so nothing downstream may assume it was granted.
    s->updateConnParams(h, 12, 12, 0, 400);
    // §2.1: the largest link-layer payload the controller will do. A 247-byte
    // MTU over the 27-octet default costs roughly three times the airtime per
    // byte delivered, at every other radio in the car's expense.
    s->setDataLen(h, 251);
    // §2.2: request 2M, keep 1M in the mask so a phone without it simply stays
    // where it is. Nothing in either protocol changes with the PHY.
    s->updatePhy(h, BLE_GAP_LE_PHY_1M_MASK | BLE_GAP_LE_PHY_2M_MASK,
                 BLE_GAP_LE_PHY_1M_MASK | BLE_GAP_LE_PHY_2M_MASK, 0);
    // Back to the floor: the previous connection's MTU says nothing about this
    // one, and carrying a phone's 247 over to a client that never exchanges
    // would size packets that client cannot receive.
    s_mtu = 23;
    // The other protocol on this server keeps its own per-connection state —
    // subscriptions, sequence numbers, a clock reading. Both are told about the
    // same link from here rather than each registering its own callbacks.
    vtpOnConnect();
    Serial.println("BLE: connected");
  }
  void onMTUChange(uint16_t mtu, NimBLEConnInfo& /*info*/) override {
    s_mtu = mtu;
    vtpSetMtu(mtu);
    Serial.printf("BLE: MTU %u\n", (unsigned)mtu);
  }
  void onDisconnect(NimBLEServer* /*s*/, NimBLEConnInfo& /*info*/, int /*reason*/) override {
    s_connected = false;
    // Streaming is per-connection state. Leaving it on would keep filling the
    // ring for a phone that has gone, so the next client inherits a backlog of
    // frames from someone else's session.
    canSetStream(SL_STREAM_OFF);
    vtpOnDisconnect();
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

  // The second protocol, on the same server and the same connection. It carries
  // the CAN bus and nothing else; every byte that makes this a shift light is
  // still on the service above.
  vtpBegin(server);

  // Two 128-bit service UUIDs do not fit in one advertisement — flags are 3
  // bytes and each UUID is 18 — so the primary packet keeps the shift light's,
  // unchanged, and the scan response carries VTP's alongside a shortened name.
  // The primary packet is what an offloaded scan filter sees on every chipset,
  // and it is byte for byte what it was before this protocol existed.
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SL_SERVICE_UUID);
  adv->enableScanResponse(true);

  // Set explicitly. NimBLEDevice::init() names the GAP service but does not put
  // the name on air by itself, and a scan from a PC shows this board advertising
  // its UUID with an empty name unless the name is placed somewhere. Built here
  // rather than through adv->setName(), which would put a complete name into
  // the scan response and leave no room for the UUID beside it.
  //
  // VTP §3.3 also asks for three bytes of Service Data. There is no room for it
  // — it would need 21 more — and the spec calls it advisory, requiring a client
  // to read the Info characteristic on every connection regardless.
  NimBLEAdvertisementData scanResp;
  bool srOk = scanResp.addServiceUUID(VTP_SERVICE_UUID);
  srOk = scanResp.setName(SL_ADV_SHORT_NAME, false) && srOk;   // 0x08, shortened
  // 18 bytes for the UUID and 11 for the name is 29 of 31, and each of these
  // refuses rather than truncates when it does not fit. Silence would mean a
  // board that scans as a shift light and is invisible to anything looking for
  // VTP, which is a long way from an obvious symptom.
  if (!srOk) Serial.println("BLE: scan response overflowed — VTP is not being advertised");
  adv->setScanResponseData(scanResp);
  adv->start();

  Serial.printf("BLE: advertising as %s (%s on air), shift light + VTP/1\n",
                SL_DEVICE_NAME, SL_ADV_SHORT_NAME);
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

  // Before the return below, and gated on its own state: VTP's control plane
  // has a response to send even on a connection where nothing here is due.
  vtpPoll();

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
  // Cap the batch to what this connection's MTU will carry. A notify is not
  // fragmented: anything past ATT_MTU-3 is dropped on the floor, and since the
  // count byte still claims the full batch the client discards the packet
  // whole. That is the whole failure -- no error, no short read, just a CAN
  // stream that silently never arrives.
  //
  // Android requests 247 and lands on 13, exactly what shipped before. iOS
  // never offers more than 185 and lands on 10, which is why this exists:
  // Web Bluetooth has no requestMtu, so it cannot be fixed from the app side.
  uint16_t mtu = s_mtu < 23 ? 23 : s_mtu;
  size_t maxRecs = (size_t)(mtu - 3 - 1) / sizeof(CanFrameRec);
  if (maxRecs < 1) maxRecs = 1;
  if (maxRecs > SL_FRAMES_PER_PKT) maxRecs = SL_FRAMES_PER_PKT;

  CanFrameRec recs[SL_FRAMES_PER_PKT];
  size_t n = canDrainFrames(recs, maxRecs);
  if (n > 0) {
    uint8_t pkt[1 + SL_FRAMES_PER_PKT * sizeof(CanFrameRec)];
    pkt[0] = (uint8_t)n;
    memcpy(pkt + 1, recs, n * sizeof(CanFrameRec));
    s_canframe->setValue(pkt, 1 + n * sizeof(CanFrameRec));
    s_canframe->notify();
  }
}
