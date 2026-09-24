#include "blesvc.h"
#include "proto.h"
#include "settings.h"
#include "canbus.h"
#include "shiftlight.h"
#include "vtpsvc.h"
#include "appverify.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <freertos/queue.h>
#include <string.h>
#include "vtp1_generated.h"   // VTP_SERVICE_UUID, for the scan response

static NimBLECharacteristic* s_config    = nullptr;
static NimBLECharacteristic* s_telemetry = nullptr;
static NimBLECharacteristic* s_canframe  = nullptr;
static volatile bool s_connected = false;

// Negotiated ATT MTU for the current connection. 23 is the spec minimum and the
// value in force until the exchange completes, which happens shortly after
// connect -- so the first batch or two go out small and then the rest run full
// size. Cached rather than queried per loop because getPeerMTU wants a
// connection handle and this board only ever talks to one phone.
static volatile uint16_t s_mtu = 23;

// Writes from the phone. NimBLE runs callbacks on its own host task, on the
// S3 the other core, while loop() renders from cfg and filters frames. So a
// callback only copies the write into this queue and blePoll() applies it on
// the loop, in the order it was sent. That also keeps an NVS commit or the
// identify flash off the host task, where it would stall the stack and drop
// the connection.
enum : uint8_t { WRITE_CONFIG, WRITE_COMMAND };
struct PhoneWrite {
  uint8_t target;                        // WRITE_CONFIG or WRITE_COMMAND
  uint8_t len;                           // clamped to sizeof(bytes)
  uint8_t bytes[1 + SL_MAX_FILTER * 4];  // the longest write either accepts
};
static QueueHandle_t s_phoneWrites = nullptr;

// Set on the host task when a config write from an unverified app is refused,
// cleared by blePoll() once it has put the running config back.
static volatile bool s_republishConfig = false;

static void queueWrite(uint8_t target, const uint8_t* bytes, size_t len) {
  if (!s_phoneWrites) return;
  PhoneWrite w = {};
  w.target = target;
  w.len    = len > sizeof(w.bytes) ? sizeof(w.bytes) : (uint8_t)len;
  memcpy(w.bytes, bytes, w.len);
  if (xQueueSend(s_phoneWrites, &w, 0) != pdTRUE) {
    Serial.println("BLE: write queue full, write dropped");
  }
}

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
    appVerifyOnConnect(h);
    Serial.println("BLE: connected");
  }
  void onMTUChange(uint16_t mtu, NimBLEConnInfo& /*info*/) override {
    s_mtu = mtu;
    vtpSetMtu(mtu);
    Serial.printf("BLE: MTU %u\n", (unsigned)mtu);
  }
  void onDisconnect(NimBLEServer* /*s*/, NimBLEConnInfo& info, int /*reason*/) override {
    s_connected = false;
    appVerifyOnDisconnect(info.getConnHandle());
    // Streaming is per-connection state; left on, the next client inherits a
    // backlog from someone else's session. Queued like the phone's own
    // stream-off so it lands after anything that phone wrote before it left.
    const uint8_t streamOff[] = { SL_CMD_STREAM, SL_STREAM_OFF };
    queueWrite(WRITE_COMMAND, streamOff, sizeof(streamOff));
    vtpOnDisconnect();
    Serial.println("BLE: disconnected, advertising again");
    NimBLEDevice::startAdvertising();
  }
};

// The challenge and the verified byte differ per connection and the attribute
// holds one value, so it is set for each reader. NimBLE calls onRead on the host
// task just before copying the value into the response (NimBLEServer.cpp,
// handleGattEvent), and every read is served from that one task, so two readers
// can't swap values in between.
class AuthCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    uint8_t value[SL_AUTH_CHALLENGE_LEN];
    size_t len = appVerifyReadValue(info.getConnHandle(), value);
    c->setValue(value, len);
  }
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    NimBLEAttValue v = c->getValue();
    appVerifyOnResponse(info.getConnHandle(), v.data(), v.size());
  }
};

// Settings writes from an app that has not verified on this connection are
// dropped here, before the queue. The write still succeeds at the ATT level:
// NimBLE-Arduino has no way for onWrite to return an error.
class ConfigCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    if (!appVerified(info.getConnHandle())) {
      Serial.println("BLE: config write refused: app not verified");
      // NimBLE stored the refused bytes as the attribute value before calling
      // this. Put back what is running, so a read shows nothing changed.
      s_republishConfig = true;
      return;
    }
    NimBLEAttValue v = c->getValue();
    queueWrite(WRITE_CONFIG, v.data(), v.size());
  }
};

class CommandCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    NimBLEAttValue v = c->getValue();
    if (!appVerified(info.getConnHandle())) {
      Serial.printf("BLE: command %d write refused: app not verified\n",
                    v.size() > 0 ? (int)v.data()[0] : -1);
      return;
    }
    queueWrite(WRITE_COMMAND, v.data(), v.size());
  }
};

// --- Phone writes, applied on the loop --------------------------------------

static void applyConfig(const PhoneWrite& w) {
  if (settingsApply(w.bytes, w.len)) {
    Serial.println("BLE: config applied (not yet saved)");
  } else {
    Serial.printf("BLE: config rejected (%u bytes)\n", (unsigned)w.len);
  }
  // Either way the attribute now holds what is running. A rejected write would
  // otherwise read back as though it had been accepted, and an accepted one
  // may have had its blink period raised to the minimum.
  blePublishConfig();
}

static void applyCommand(const PhoneWrite& w) {
  if (w.len < 1) return;
  const uint8_t* p = w.bytes;

  switch (p[0]) {
    case SL_CMD_SAVE:
      Serial.println(settingsSave() ? "settings: saved" : "settings: SAVE FAILED");
      blePublishConfig();
      break;

    case SL_CMD_DEFAULTS:
      settingsDefaults();
      blePublishConfig();
      break;

    case SL_CMD_STREAM:
      if (w.len >= 2) canSetStream(p[1]);
      break;

    case SL_CMD_FILTER: {
      uint32_t ids[SL_MAX_FILTER];
      size_t n = (w.len - 1) / 4;
      if (n > SL_MAX_FILTER) n = SL_MAX_FILTER;
      for (size_t i = 0; i < n; i++) {
        memcpy(&ids[i], p + 1 + i * 4, 4);
      }
      canSetFilter(ids, n);
      break;
    }

    case SL_CMD_REBOOT:
      Serial.println("rebooting on request");
      delay(100);
      ESP.restart();
      break;

    case SL_CMD_IDENTIFY:
      shiftlightIdentify();
      break;

    default:
      break;
  }
}

void blePublishConfig() {
  if (s_config) s_config->setValue((uint8_t*)&cfg, sizeof(cfg));
}

void bleBegin() {
  // Before init, so the queue exists before any callback can fire.
  s_phoneWrites = xQueueCreate(8, sizeof(PhoneWrite));

  NimBLEDevice::init(SL_DEVICE_NAME);
  // +9 dBm. NimBLE 2.x takes dBm, not an esp_power_level_t: the old
  // ESP_PWR_LVL_P9 is enum value 11, which it rounded up to +12 dBm.
  NimBLEDevice::setPower(9);
  // The frame stream is the only thing here that needs a big MTU; at the
  // default 23 a full batch would fragment into ten packets and arrive slower
  // than the bus produces it.
  NimBLEDevice::setMTU(247);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  NimBLEService* service = server->createService(SL_SERVICE_UUID);

  // Reads stay open: the blob is thresholds and colours, nothing secret, and an
  // app that has not verified can still show what the board is running.
  s_config = service->createCharacteristic(
      SL_CONFIG_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  s_config->setCallbacks(new ConfigCallbacks());
  blePublishConfig();

  s_telemetry = service->createCharacteristic(
      SL_TELEMETRY_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  s_canframe = service->createCharacteristic(
      SL_CANFRAME_UUID, NIMBLE_PROPERTY::NOTIFY);

  // Every command changes the board (save, defaults, reboot, identify, the frame
  // stream), so all of them need a verified app, the same as a config write.
  NimBLECharacteristic* cmd = service->createCharacteristic(
      SL_COMMAND_UUID, NIMBLE_PROPERTY::WRITE);
  cmd->setCallbacks(new CommandCallbacks());

  // Max length 16, so NimBLE refuses a longer write itself (Invalid Attribute
  // Value Length) rather than storing it.
  NimBLECharacteristic* auth = service->createCharacteristic(
      SL_AUTH_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE, SL_AUTH_CHALLENGE_LEN);
  auth->setCallbacks(new AuthCallbacks());

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
  PhoneWrite w;
  while (s_phoneWrites && xQueueReceive(s_phoneWrites, &w, 0) == pdTRUE) {
    if (w.target == WRITE_CONFIG) applyConfig(w);
    else                          applyCommand(w);
  }
  if (s_republishConfig) {
    s_republishConfig = false;
    blePublishConfig();
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
