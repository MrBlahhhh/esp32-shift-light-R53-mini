#include "vtpsvc.h"
#include "canbus.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <esp_timer.h>
#include <string.h>

#include "vtp1_encode.h"

// ---------------------------------------------------------------------------
// VTP/1, CAN role. Capabilities: can, control, masked_subscriptions.
//
// The board has no GNSS and no IMU, so those bits are clear — but §4.1's
// attribute table is fixed, so their characteristics exist anyway and simply
// never notify. That is not a formality: central stacks cache the attribute
// table across connections, and a table that changes shape between connections
// hands the next client a stale handle.
//
// Section numbers below are SPEC.md in github.com/Lapsmith-app/VTP.
// ---------------------------------------------------------------------------

// Control opcodes (§9). Not in vtp1_generated.h — the generated header carries
// record layouts, not the command table — so they are transcribed here.
#define VTP_OP_CAN_RESET          0x01
#define VTP_OP_CAN_SUBSCRIBE      0x02
#define VTP_OP_CAN_SUBSCRIBE_MASK 0x03
#define VTP_OP_CAN_UNSUBSCRIBE    0x04
#define VTP_OP_TIME_SYNC          0x30

// Bits 0..29: the twenty-nine arbitration bits and the standard/extended format
// bit. Everything above is how a frame was transmitted rather than which frame
// it is, and takes no part in matching or in identity (§9.1).
#define VTP_ID_BITS   0x3FFFFFFFu
#define VTP_EXT_BIT   (1u << 29)

#define VTP_SUB_SLOTS   8    // published as can_subscription_slots
#define VTP_SCHED_SLOTS 24   // the §6.8 bound on (subscription, identifier) state
#define VTP_MAX_BATCH   16   // 15 fits a 247-byte MTU; one spare
#define VTP_FLUSH_MS    10   // §6.1 recommends once per connection interval

// What this device says it can forward (§4). It is a description, not a budget
// it polices: §9.3 forbids refusing a subscription on rate grounds, so the
// honest mechanism for more load than this is shedding, which is reported.
//
// Where the figure comes from: one batch every VTP_FLUSH_MS, and a batch at the
// 247-byte MTU Android negotiates holds fifteen eight-byte frames. That is 1500
// a second before the radio is the limit rather than this code; 1000 is that
// with the headroom a connection interval the central chose, not this board,
// can take away at any moment.
#define VTP_MAX_FRAMES_PER_S 1000

static NimBLECharacteristic* s_info    = nullptr;
static NimBLECharacteristic* s_can     = nullptr;
static NimBLECharacteristic* s_control = nullptr;

static bool     s_connected      = false;
static bool     s_canSubscribed  = false;  // CCCD on the CAN stream
static bool     s_ctlIndications = false;  // CCCD on Control, indications enabled
static uint16_t s_mtu            = 23;
static uint16_t s_canSeq         = 0;
static uint16_t s_shed           = 0;      // discarded for want of schedule state
static uint32_t s_lastFlushMs    = 0;
static bool     s_mtuWarned      = false;

static uint64_t nowUs() { return (uint64_t)esp_timer_get_time(); }

static uint16_t satAdd(uint16_t a, uint64_t b) {
  uint64_t sum = (uint64_t)a + b;
  return sum > 0xFFFF ? 0xFFFF : (uint16_t)sum;
}

// --- Subscriptions (§9.1) ---------------------------------------------------

struct Sub {
  bool     used;
  uint32_t id;     // masked to bits 0..29, which is the whole of its identity
  uint32_t mask;
  uint8_t  mode;
  uint16_t arg;
  uint32_t order;  // install order, for §9.2's tie-break
};
static Sub      s_subs[VTP_SUB_SLOTS];
static uint32_t s_order = 0;

// Per (subscription, identifier) scheduling state. Not per subscription — a
// shared interval lets whichever identifier arrives first consume it, and a
// client subscribed to a group as a group then hears one signal out of it and
// sees a quiet bus rather than a bug. Not per identifier either, or a second
// subscription covering one id of a masked group destroys the first's schedule.
struct Sched {
  bool     used;
  uint8_t  sub;
  uint32_t key;     // id with the format bit in place, as matching sees it
  uint64_t lastUs;
  bool     armed;   // the first matching frame is still owed (§6.8)
};
static Sched s_sched[VTP_SCHED_SLOTS];

// §9.2: the most specific match, and among equals the one installed earliest.
// A frame is forwarded at most once however many subscriptions cover it —
// duplicates on one bus-arrival timestamp are indistinguishable from a fault.
static int governing(uint32_t key) {
  int      best = -1;
  int      bestBits = -1;
  uint32_t bestOrder = 0;
  for (int i = 0; i < VTP_SUB_SLOTS; i++) {
    if (!s_subs[i].used) continue;
    if (((key ^ s_subs[i].id) & s_subs[i].mask) != 0) continue;
    int bits = __builtin_popcount(s_subs[i].mask);
    if (bits > bestBits || (bits == bestBits && s_subs[i].order < bestOrder)) {
      best = i;
      bestBits = bits;
      bestOrder = s_subs[i].order;
    }
  }
  return best;
}

static Sched* schedFind(int sub, uint32_t key) {
  for (int i = 0; i < VTP_SCHED_SLOTS; i++) {
    if (s_sched[i].used && s_sched[i].sub == sub && s_sched[i].key == key) return &s_sched[i];
  }
  return nullptr;
}

static void schedClearForSub(int sub) {
  for (int i = 0; i < VTP_SCHED_SLOTS; i++) {
    if (s_sched[i].used && s_sched[i].sub == sub) s_sched[i].used = false;
  }
}

// §6.8: a displaced subscription keeps its schedule, so entries outlive
// governance — but state nothing is currently using must be reclaimed before a
// frame whose governing subscription has no state is shed. Reclaiming costs one
// early frame if governance comes back; shedding costs the client the stream it
// asked for.
static Sched* schedAlloc(int sub, uint32_t key) {
  Sched* e = nullptr;
  for (int i = 0; i < VTP_SCHED_SLOTS && !e; i++) {
    if (!s_sched[i].used) e = &s_sched[i];
  }
  for (int i = 0; i < VTP_SCHED_SLOTS && !e; i++) {
    if (governing(s_sched[i].key) != s_sched[i].sub) e = &s_sched[i];
  }
  if (!e) return nullptr;
  e->used   = true;
  e->sub    = (uint8_t)sub;
  e->key    = key;
  e->lastUs = 0;
  e->armed  = true;
  return e;
}

bool vtpAdmit(uint32_t id, bool extended, uint64_t tsUs) {
  // Nowhere to send it is not the same as nobody wanting it, but the effect on
  // the client is: with no link and no CCCD there is no stream for a frame to
  // be accepted into, so none is, and `dropped` stays honestly at zero.
  if (!s_connected || !s_canSubscribed) return false;

  uint32_t key = (id & 0x1FFFFFFFu) | (extended ? VTP_EXT_BIT : 0u);
  int si = governing(key);
  if (si < 0) return false;                      // matched nothing: never accepted (§6.3)

  const Sub& s = s_subs[si];
  if (s.mode == VTP_SUB_MODE_EVERY_FRAME) return true;
  if (s.arg == 0) return true;                   // periodic, no limit

  Sched* e = schedFind(si, key);
  if (!e) {
    e = schedAlloc(si, key);
    if (!e) {                                    // §6.8: shed rather than forward unscheduled
      s_shed = satAdd(s_shed, 1);
      return false;
    }
  }
  if (e->armed) {                                // the first matching frame, in every mode
    e->armed  = false;
    e->lastUs = tsUs;
    return true;
  }
  if (tsUs - e->lastUs < (uint64_t)s.arg * 1000ull) return false;  // mode did not select it
  e->lastUs = tsUs;
  return true;
}

static uint8_t canSubscribe(uint32_t id, uint32_t mask, uint8_t mode, uint16_t arg) {
  if (mode != VTP_SUB_MODE_EVERY_FRAME && mode != VTP_SUB_MODE_PERIODIC) {
    return VTP_STATUS_BAD_PARAMS;                // 2 and 3 are unassigned, not defaults
  }
  id   &= VTP_ID_BITS;
  mask &= VTP_ID_BITS;

  for (int i = 0; i < VTP_SUB_SLOTS; i++) {
    if (!s_subs[i].used || s_subs[i].id != id || s_subs[i].mask != mask) continue;
    // A re-install that changes nothing changes nothing (§6.8): a client
    // retrying a request whose response was lost must not be paid for it with a
    // frame inside the interval it asked for. It consumes no slot either, so
    // reprogramming on every connection cannot exhaust the table (§9.1).
    if (s_subs[i].mode != mode || s_subs[i].arg != arg) {
      s_subs[i].mode = mode;
      s_subs[i].arg  = arg;
      schedClearForSub(i);                       // a new instruction re-arms the first frame
    }
    return VTP_STATUS_OK;
  }

  for (int i = 0; i < VTP_SUB_SLOTS; i++) {
    if (s_subs[i].used) continue;
    s_subs[i].used  = true;
    s_subs[i].id    = id;
    s_subs[i].mask  = mask;
    s_subs[i].mode  = mode;
    s_subs[i].arg   = arg;
    s_subs[i].order = ++s_order;
    return VTP_STATUS_OK;
  }
  return VTP_STATUS_TABLE_FULL;
}

static uint8_t canUnsubscribe(uint32_t id, uint32_t mask) {
  id   &= VTP_ID_BITS;
  mask &= VTP_ID_BITS;
  for (int i = 0; i < VTP_SUB_SLOTS; i++) {
    if (!s_subs[i].used || s_subs[i].id != id || s_subs[i].mask != mask) continue;
    s_subs[i].used = false;
    schedClearForSub(i);
    return VTP_STATUS_OK;
  }
  return VTP_STATUS_UNKNOWN_SUBSCRIPTION;
}

static void canReset() {
  memset(s_subs, 0, sizeof(s_subs));
  memset(s_sched, 0, sizeof(s_sched));
  s_order = 0;
  canVtpReset();
}

// --- Control (§9) -----------------------------------------------------------
//
// A request is applied on the main loop, never in the BLE callback: applying it
// there would run a table edit on NimBLE's host task while the CAN path reads
// the same tables from loop(), and everything here is unsynchronised precisely
// because it all runs in one place.

struct CtlReq {
  bool     used;
  uint8_t  op, tag;
  uint8_t  p[16];
  uint8_t  plen;
  uint64_t rxUs;    // §9.5: taken when the write arrived, not when it is answered
};
static CtlReq s_req;

// Two slots. One holds the response being sent; the second is the room §9 says
// a device must have to hold a response composed while an earlier indication is
// still unconfirmed — which is exactly the window a conforming client's next
// request arrives in. Past that there is no room, and §9 says to discard the
// request unanswered rather than apply one that cannot be answered.
#define CTL_RESP_SLOTS 2
struct CtlResp {
  uint8_t buf[VTP_CONTROL_RESPONSE_SIZE + VTP_TIME_SYNC_SIZE];
  uint8_t len;
};
static CtlResp s_resp[CTL_RESP_SLOTS];
static uint8_t s_respHead  = 0;   // a queue, not two slots: a `busy` refusal
static uint8_t s_respCount = 0;   // must not overtake the answer it refers to
static bool    s_indicateOutstanding = false;

static bool respRoom() { return s_respCount < CTL_RESP_SLOTS; }

// A response is owed from the moment its request is accepted until the device
// has sent it (§9) — the send, not the confirmation, because that is where the
// client's own boundary falls.
static bool respOwed() {
  return s_req.used || s_respCount > 0;
}

static void respPush(uint8_t op, uint8_t tag, uint8_t status,
                     const uint8_t* detail, size_t detailLen) {
  if (!respRoom()) return;
  CtlResp& slot = s_resp[(s_respHead + s_respCount) % CTL_RESP_SLOTS];
  vtp_control_response_t r = {};
  r.opcode = op;
  r.tag    = tag;
  r.status = status;
  // §9: detail is present if and only if status is ok. The encoder enforces it;
  // this is not the place to second-guess a caller that got it wrong.
  int n = vtp_encode_control_response(&r, slot.buf, sizeof(slot.buf));
  if (n < 0) return;
  if (status == VTP_STATUS_OK && detail && detailLen) {
    if ((size_t)n + detailLen > sizeof(slot.buf)) return;
    memcpy(slot.buf + n, detail, detailLen);
    n += (int)detailLen;
  }
  slot.len = (uint8_t)n;
  s_respCount++;
}

static void ctlApply() {
  const CtlReq r = s_req;
  s_req.used = false;

  switch (r.op) {
    case VTP_OP_CAN_RESET:
      if (r.plen != 0) { respPush(r.op, r.tag, VTP_STATUS_BAD_PARAMS, nullptr, 0); break; }
      canReset();
      respPush(r.op, r.tag, VTP_STATUS_OK, nullptr, 0);
      break;

    case VTP_OP_CAN_SUBSCRIBE: {
      if (r.plen != 7) { respPush(r.op, r.tag, VTP_STATUS_BAD_PARAMS, nullptr, 0); break; }
      uint32_t id  = (uint32_t)r.p[0] | ((uint32_t)r.p[1] << 8) |
                     ((uint32_t)r.p[2] << 16) | ((uint32_t)r.p[3] << 24);
      uint8_t  mode = r.p[4];
      uint16_t arg  = (uint16_t)r.p[5] | ((uint16_t)r.p[6] << 8);
      // §9.1: exactly CAN_SUBSCRIBE_MASK with a full mask.
      respPush(r.op, r.tag, canSubscribe(id, VTP_ID_BITS, mode, arg), nullptr, 0);
      break;
    }

    case VTP_OP_CAN_SUBSCRIBE_MASK: {
      if (r.plen != 11) { respPush(r.op, r.tag, VTP_STATUS_BAD_PARAMS, nullptr, 0); break; }
      uint32_t id   = (uint32_t)r.p[0] | ((uint32_t)r.p[1] << 8) |
                      ((uint32_t)r.p[2] << 16) | ((uint32_t)r.p[3] << 24);
      uint32_t mask = (uint32_t)r.p[4] | ((uint32_t)r.p[5] << 8) |
                      ((uint32_t)r.p[6] << 16) | ((uint32_t)r.p[7] << 24);
      uint8_t  mode = r.p[8];
      uint16_t arg  = (uint16_t)r.p[9] | ((uint16_t)r.p[10] << 8);
      respPush(r.op, r.tag, canSubscribe(id, mask, mode, arg), nullptr, 0);
      break;
    }

    case VTP_OP_CAN_UNSUBSCRIBE: {
      if (r.plen != 8) { respPush(r.op, r.tag, VTP_STATUS_BAD_PARAMS, nullptr, 0); break; }
      uint32_t id   = (uint32_t)r.p[0] | ((uint32_t)r.p[1] << 8) |
                      ((uint32_t)r.p[2] << 16) | ((uint32_t)r.p[3] << 24);
      uint32_t mask = (uint32_t)r.p[4] | ((uint32_t)r.p[5] << 8) |
                      ((uint32_t)r.p[6] << 16) | ((uint32_t)r.p[7] << 24);
      respPush(r.op, r.tag, canUnsubscribe(id, mask), nullptr, 0);
      break;
    }

    case VTP_OP_TIME_SYNC: {
      if (r.plen != 0) { respPush(r.op, r.tag, VTP_STATUS_BAD_PARAMS, nullptr, 0); break; }
      vtp_time_sync_t t = {};
      t.t_device_rx = r.rxUs;
      t.t_device_tx = nowUs();
      uint8_t detail[VTP_TIME_SYNC_SIZE];
      int n = vtp_encode_time_sync(&t, detail, sizeof(detail));
      if (n < 0) break;
      respPush(r.op, r.tag, VTP_STATUS_OK, detail, (size_t)n);
      break;
    }

    default:
      // §9: availability is decided before parameters, so an opcode this
      // device does not own is refused without its arguments being read.
      respPush(r.op, r.tag, VTP_STATUS_UNSUPPORTED_OPCODE, nullptr, 0);
      break;
  }
}

class ControlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& /*info*/) override {
    uint64_t rxUs = nowUs();   // §9.5 wants the arrival, not the composition
    NimBLEAttValue v = c->getValue();
    if (v.size() < 2) return;  // not a request; there is no tag to answer with

    // §9.4: deliverability is decided before dispatch. With indications off
    // there is nowhere for the answer to go, so the request must not take
    // effect and must not be counted as received.
    if (!s_ctlIndications) return;

    if (respOwed()) {
      // The client has broken the one-outstanding rule. `busy` says nothing
      // about the request itself, and the request is not applied. With no room
      // to hold the refusal either, §9 says to discard it rather than apply
      // something that cannot be answered — which respPush does by itself.
      respPush(v.data()[0], v.data()[1], VTP_STATUS_BUSY, nullptr, 0);
      return;
    }

    size_t plen = v.size() - 2;
    if (plen > sizeof(s_req.p)) {
      respPush(v.data()[0], v.data()[1], VTP_STATUS_BAD_PARAMS, nullptr, 0);
      return;
    }
    s_req.op   = v.data()[0];
    s_req.tag  = v.data()[1];
    s_req.plen = (uint8_t)plen;
    if (plen) memcpy(s_req.p, v.data() + 2, plen);
    s_req.rxUs = rxUs;
    s_req.used = true;
  }

  void onSubscribe(NimBLECharacteristic* /*c*/, NimBLEConnInfo& /*info*/,
                   uint16_t subValue) override {
    s_ctlIndications = (subValue & 0x0002) != 0;
  }

  void onStatus(NimBLECharacteristic* /*c*/, NimBLEConnInfo& /*info*/, int /*code*/) override {
    // Called once an indication is resolved — confirmed, or failed. Either way
    // the link is free to carry the next one.
    s_indicateOutstanding = false;
  }
};

class CanStreamCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic* /*c*/, NimBLEConnInfo& /*info*/,
                   uint16_t subValue) override {
    bool on = (subValue & 0x0001) != 0;
    if (on && !s_canSubscribed) canVtpReset();   // no backlog from before the client asked
    s_canSubscribed = on;
  }
};

// The inert streams. §4.1 requires a device to accept a CCCD write on a stream
// whose capability bit is clear and then simply never notify, rather than
// refusing it — so these need no callbacks at all and are here to be named.

// --- The CAN stream (§6) ----------------------------------------------------

static void canFlush() {
  if (!s_connected || !s_canSubscribed || !s_can) return;

  uint32_t nowMs = millis();
  if (nowMs - s_lastFlushMs < VTP_FLUSH_MS) return;
  s_lastFlushMs = nowMs;

  // §2: the client owes this device an MTU of at least 100. Below the size of a
  // header plus one full record there is no batch to send at all, and silence
  // would be indistinguishable from a quiet bus.
  static uint8_t pkt[VTP_CAN_HEADER_SIZE + VTP_MAX_BATCH * (VTP_CAN_RECORD_SIZE + 8)];

  size_t budget = (s_mtu > 3 ? (size_t)s_mtu - 3 : 20);
  if (budget > sizeof(pkt)) budget = sizeof(pkt);   // a batch never outgrows its buffer
  if (budget < VTP_CAN_HEADER_SIZE + VTP_CAN_RECORD_SIZE + 8) {
    if (!s_mtuWarned) {
      s_mtuWarned = true;
      Serial.printf("VTP: MTU %u is below the 100 the spec requires — no CAN batches\n",
                    (unsigned)s_mtu);
    }
    return;
  }

  static CapFrame       cap[VTP_MAX_BATCH];
  static vtp_can_frame_t fr[VTP_MAX_BATCH];
  size_t   n = 0;
  size_t   used = VTP_CAN_HEADER_SIZE;
  uint64_t tBase = 0;

  CapFrame c;
  while (n < VTP_MAX_BATCH && canVtpPeek(&c)) {
    if (n == 0) tBase = c.tsUs;
    uint64_t dt = (c.tsUs - tBase) / 10;
    if (dt > 0xFFFF) break;                       // §6.1's window; the rest rides the next batch
    size_t need = VTP_CAN_RECORD_SIZE + c.len;
    if (used + need > budget) break;
    canVtpPop();

    cap[n] = c;
    memset(&fr[n], 0, sizeof(fr[n]));
    fr[n].dt       = (uint16_t)dt;
    fr[n].id       = c.id & 0x1FFFFFFFu;
    fr[n].extended = c.ext ? 1 : 0;
    fr[n].len      = c.len;
    fr[n].payload  = cap[n].data;
    fr[n].t_device = c.tsUs;
    used += need;
    n++;
  }

  // §6.2: count must not be zero. A quiet bus is reported by sending nothing —
  // there is no empty-batch heartbeat, and `dropped` rides the next batch that
  // has content.
  if (n == 0) return;

  uint16_t dropped = satAdd(canVtpTakeDropped(), s_shed);
  s_shed = 0;

  vtp_can_header_t h = {};
  h.seq      = s_canSeq;
  h.dropped  = dropped;
  h.t_base   = tBase;
  h.count    = (uint8_t)n;
  h.flags    = dropped ? VTP_CAN_FLAGS_SHEDDING : 0;
  h.reserved = 0;

  int len = vtp_encode_can_batch(&h, fr, pkt, sizeof(pkt));
  if (len < 0) {
    // The encoder refused a batch its own decoder would reject. Dropping it is
    // the only honest outcome — those frames were accepted and are now gone.
    Serial.println("VTP: encoder refused a CAN batch — dropped");
    s_shed = satAdd(s_shed, n + dropped);
    return;
  }

  if (!s_can->notify(pkt, (size_t)len)) {
    // Not sent, so `seq` does not advance: §8.2's gap means the transport lost
    // what the device sent, and a number burned here would report a loss that
    // never happened while hiding one that did.
    s_shed = satAdd(s_shed, n + dropped);
    return;
  }
  s_canSeq++;
}

// --- Link lifecycle ---------------------------------------------------------

void vtpOnConnect() {
  s_connected      = true;
  s_canSubscribed  = false;
  s_ctlIndications = false;
  s_mtu            = 23;
  s_mtuWarned      = false;
  // §8.2: the first notification after a connection carries seq 0, so a client
  // never has to tell a reconnection from a wrap and the protocol needs no
  // session id.
  s_canSeq = 0;
  s_shed   = 0;
  s_indicateOutstanding = false;
  s_req.used   = false;
  s_respHead   = 0;
  s_respCount  = 0;
  canReset();
}

void vtpOnDisconnect() {
  s_connected      = false;
  s_canSubscribed  = false;
  s_ctlIndications = false;
  // §9.1: subscriptions do not survive disconnection, so a client always finds
  // a known state and never inherits one it did not install.
  canReset();
}

void vtpSetMtu(uint16_t mtu) {
  s_mtu = mtu;
  s_mtuWarned = false;
}

void vtpPoll() {
  if (!s_connected) return;

  // One response out per pass, and only when the link is not already carrying
  // an indication. A response composed behind an unconfirmed one waits for that
  // confirmation rather than being refused.
  if (s_respCount && !s_indicateOutstanding && s_control) {
    const CtlResp& front = s_resp[s_respHead];
    if (s_control->indicate(front.buf, front.len)) {
      s_indicateOutstanding = true;
      s_respHead = (s_respHead + 1) % CTL_RESP_SLOTS;
      s_respCount--;
    }
  }

  if (s_req.used && respRoom()) ctlApply();

  canFlush();
}

// --- Bring-up ---------------------------------------------------------------

void vtpBegin(NimBLEServer* server) {
  NimBLEService* svc = server->createService(VTP_SERVICE_UUID);

  s_info = svc->createCharacteristic(VTP_CHAR_INFO_UUID, NIMBLE_PROPERTY::READ);

  vtp_info_t info = {};
  info.protocol_major = 1;
  info.protocol_minor = 0;
  info.capabilities   = VTP_CAPABILITIES_CAN | VTP_CAPABILITIES_CONTROL |
                        VTP_CAPABILITIES_MASKED_SUBSCRIPTIONS;
  // §4.1: a capacity behind a cleared bit is a capability the device does not
  // have, so the GPS and IMU rates stay at zero rather than at something
  // plausible.
  info.gps_rate_hz            = 0;
  info.gps_max_rate_hz        = 0;
  info.can_subscription_slots = VTP_SUB_SLOTS;
  info.can_max_frames_per_s   = VTP_MAX_FRAMES_PER_S;
  info.imu_rate_hz            = 0;
  info.imu_max_rate_hz        = 0;
  info.obd_poll_slots         = 0;
  // The clock is esp_timer's, which counts from boot and knows nothing about
  // the link, so it survives a reconnection. Nothing disciplines it.
  info.clock_flags  = VTP_CLOCK_FLAGS_SURVIVES_RECONNECT;
  info.reserved_22  = 0;

  uint8_t enc[VTP_INFO_SIZE];
  int n = vtp_encode_info(&info, enc, sizeof(enc));
  if (n == VTP_INFO_SIZE) s_info->setValue(enc, (size_t)n);
  else Serial.println("VTP: info record refused by the encoder — service is broken");

  // The two inert streams. Present because §4.1's attribute table is fixed, and
  // subscribable because §4.1 says a device must accept a CCCD write on an
  // inert stream and then never notify.
  svc->createCharacteristic(VTP_CHAR_GPS_UUID, NIMBLE_PROPERTY::NOTIFY);
  svc->createCharacteristic(VTP_CHAR_IMU_UUID, NIMBLE_PROPERTY::NOTIFY);

  s_can = svc->createCharacteristic(VTP_CHAR_CAN_UUID, NIMBLE_PROPERTY::NOTIFY);
  s_can->setCallbacks(new CanStreamCallbacks());

  s_control = svc->createCharacteristic(
      VTP_CHAR_CONTROL_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
  s_control->setCallbacks(new ControlCallbacks());

  // Inert, and inert here has to mean an ATT error rather than a shrug: §4.1
  // requires a write to monitor_values to be rejected while the write property
  // is still declared. WRITE_AUTHOR declares the property and refuses every
  // write with "insufficient authorization", which is precisely that.
  svc->createCharacteristic(VTP_CHAR_MONITOR_VALUES_UUID,
                            NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_AUTHOR);

  // The one exception. A Write Command carries no response of any kind, so an
  // inert aiding characteristic can only discard silently — §14 puts every
  // refusal a client needs on Control instead.
  svc->createCharacteristic(VTP_CHAR_AIDING_UUID, NIMBLE_PROPERTY::WRITE_NR);

  Serial.println("VTP/1: service up — can, control, masked_subscriptions");
}
