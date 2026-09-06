#include "canbus.h"
#include "settings.h"
#include "vtpsvc.h"
#include <Arduino.h>
#include <driver/twai.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <string.h>

// Overridable from platformio.ini so a rewire is a build flag, not a patch.
#ifndef CAN_TX_GPIO
#define CAN_TX_GPIO 6
#endif
#ifndef CAN_RX_GPIO
#define CAN_RX_GPIO 7
#endif
#define TWAI_TX_PIN ((gpio_num_t)CAN_TX_GPIO)
#define TWAI_RX_PIN ((gpio_num_t)CAN_RX_GPIO)

static bool     s_up            = false;
static uint16_t s_rpm           = 0;
static uint32_t s_lastRpmMs     = 0;
static uint32_t s_frameCount    = 0;
static uint16_t s_framesPerSec  = 0;
static uint32_t s_rateWindowMs  = 0;
static uint16_t s_rxMissed      = 0;

static uint8_t  s_stream        = SL_STREAM_OFF;
static uint32_t s_filter[SL_MAX_FILTER];
static size_t   s_filterCount   = 0;

// Frame ring. Sized to hold roughly a second of a busy bus so a phone that
// wakes up late gets recent history rather than a gap, and deliberately
// overwrites oldest-first: on a logger that cannot keep up, the newest frames
// are the ones worth having.
//
// The cursors count frames rather than index slots — s_head is every frame ever
// pushed, and a reader is behind by (s_head - tail). A cursor that has fallen
// more than RING_SIZE behind has been overwritten, and it finds that out by
// arithmetic at read time instead of the writer having to know who is reading.
// That is what makes two independent readers cheap: the writer does not touch
// either cursor.
//
// Everything here runs on the main loop task — canPoll() writes, the two BLE
// services read, all from loop() — so none of it is synchronised and none of it
// needs to be. Moving any of it to a task of its own is not a small change.
#define RING_SIZE 128
static CapFrame s_ring[RING_SIZE];
static uint64_t s_head = 0, s_tailLegacy = 0, s_tailVtp = 0;
static uint16_t s_vtpDropped = 0;

static uint16_t satAdd(uint16_t a, uint64_t b) {
  uint64_t sum = (uint64_t)a + b;
  return sum > 0xFFFF ? 0xFFFF : (uint16_t)sum;
}

// A cursor that has been overwritten is moved to the oldest frame still held.
// For VTP those lost frames are the definition of §8.3's `dropped`: the device
// accepted them — a subscription asked for them and its mode selected them —
// and then discarded them because it could not keep up.
static void catchUp(uint64_t& tail, uint16_t* dropped) {
  if (s_head - tail <= RING_SIZE) return;
  uint64_t lost = (s_head - RING_SIZE) - tail;
  tail = s_head - RING_SIZE;
  if (dropped) *dropped = satAdd(*dropped, lost);
}

static void ringPush(const twai_message_t& m, uint64_t tsUs, uint8_t want) {
  CapFrame& r = s_ring[s_head % RING_SIZE];
  r.tsUs = tsUs;
  r.id   = m.identifier;
  r.ext  = m.extd ? 1 : 0;
  r.len  = m.data_length_code > 8 ? 8 : m.data_length_code;
  r.want = want;
  memset(r.data, 0, sizeof(r.data));
  memcpy(r.data, m.data, r.len);
  s_head++;
}

static bool wanted(uint32_t id) {
  if (s_stream == SL_STREAM_OFF) return false;
  if (s_stream == SL_STREAM_ALL) return true;
  for (size_t i = 0; i < s_filterCount; i++) if (s_filter[i] == id) return true;
  return false;
}

// Both protocols get a say on every frame, and both are asked exactly once.
// vtpAdmit() is not a predicate — it advances per-identifier schedule state — so
// calling it twice, or skipping it because the old protocol already wants the
// frame, would quietly corrupt a periodic subscription's timing.
static uint8_t consumers(const twai_message_t& m, uint64_t tsUs) {
  uint8_t want = 0;
  if (wanted(m.identifier)) want |= CAN_WANT_LEGACY;
  if (vtpAdmit(m.identifier, m.extd != 0, tsUs)) want |= CAN_WANT_VTP;
  return want;
}

void canBegin() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN,
                                                        TWAI_MODE_LISTEN_ONLY);
  // The default of 5 is far too shallow once the phone is streaming every id:
  // the rx queue has to survive a loop iteration that spent its time in FastLED
  // or a BLE notify, and an overrun there shows up as missing frames in the log
  // with nothing to explain them.
  g.rx_queue_len = 64;

  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) {
    Serial.println("CAN: driver install failed");
    s_up = false;
    return;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("CAN: start failed");
    s_up = false;
    return;
  }
  Serial.printf("CAN: listen-only, 500 kbit, TX=GPIO%d RX=GPIO%d\n", CAN_TX_GPIO, CAN_RX_GPIO);
  s_up = true;
  s_rateWindowMs = millis();
}

void canPoll() {
  uint32_t now = millis();

  twai_status_info_t st;
  if (twai_get_status_info(&st) == ESP_OK) {
    // Listen-only cannot reach bus-off — it never transmits an error frame — so
    // the only recovery that matters here is a driver that stopped.
    if (st.state == TWAI_STATE_STOPPED) {
      s_up = false;
      twai_start();
    } else if (st.state == TWAI_STATE_RUNNING) {
      s_up = true;
    }
    if (st.rx_missed_count > s_rxMissed) {
      s_rxMissed = st.rx_missed_count > 0xFFFF ? 0xFFFF : (uint16_t)st.rx_missed_count;
    }
  }

  // Bounded per call. Draining without a cap lets a busy bus starve the LED
  // update and the BLE stack, which is the one failure that is visible from the
  // driver's seat.
  for (int i = 0; i < 24; i++) {
    twai_message_t m;
    if (twai_receive(&m, 0) != ESP_OK) break;
    s_frameCount++;

    // Read the clock per frame, not per call: at 24 frames an iteration a
    // shared stamp would file a whole batch under one instant and flatten the
    // spacing VTP exists to carry.
    //
    // This is when the driver handed the frame over, which is later than the
    // end-of-frame VTP §6.7 asks for by however long it sat in the rx queue.
    // The gap is small and it is not zero; it is stated here rather than
    // claimed away, because a client aligning below a millisecond is entitled
    // to know that this device measures arrival at the software boundary.
    uint64_t tsUs = (uint64_t)esp_timer_get_time();

    if (m.identifier == cfg.canRpmId && m.data_length_code >= 4) {
      uint16_t raw = (uint16_t)((m.data[3] << 8) | m.data[2]);
      s_rpm = (uint16_t)((raw * 10UL) / cfg.rpmScaleX10);
      s_lastRpmMs = now;
    }

    uint8_t want = consumers(m, tsUs);
    if (want) ringPush(m, tsUs, want);
  }

  if (now - s_rateWindowMs >= 1000) {
    s_framesPerSec = (uint16_t)(s_frameCount > 0xFFFF ? 0xFFFF : s_frameCount);
    s_frameCount = 0;
    s_rateWindowMs = now;
  }
}

uint16_t canRpm() {
  // A stale reading is reported as no reading, never as a convincing zero: the
  // strip must go dark when the bus goes quiet, not sit at whatever RPM the
  // engine happened to be doing when the wire fell off.
  return canRpmFresh() ? s_rpm : 0;
}
bool canRpmFresh()        { return s_lastRpmMs != 0 && (millis() - s_lastRpmMs) < RPM_STALE_MS; }
bool canUp()              { return s_up; }
uint16_t canFramesPerSec(){ return s_framesPerSec; }
uint16_t canRxMissed()    { return s_rxMissed; }

size_t canDrainFrames(CanFrameRec* out, size_t max) {
  // No drop count on this side: the …0004 stream has never had a field to
  // report one in, and inventing a behaviour change here would be a protocol
  // change to the thing every shipped app already speaks.
  catchUp(s_tailLegacy, nullptr);
  size_t n = 0;
  while (n < max && s_tailLegacy != s_head) {
    const CapFrame& c = s_ring[s_tailLegacy % RING_SIZE];
    s_tailLegacy++;
    if (!(c.want & CAN_WANT_LEGACY)) continue;   // captured for VTP, not for this
    CanFrameRec& r = out[n++];
    r.tsMs     = (uint32_t)(c.tsUs / 1000);
    r.id       = c.id;
    r.dlcFlags = (uint8_t)(c.len & 0x0F);
    if (c.ext) r.dlcFlags |= SL_FRAME_EXT;
    memcpy(r.data, c.data, sizeof(r.data));
  }
  return n;
}

// Skips whatever this cursor was not meant to see, and reports the overrun on
// the way past.
static bool vtpSeek() {
  catchUp(s_tailVtp, &s_vtpDropped);
  while (s_tailVtp != s_head && !(s_ring[s_tailVtp % RING_SIZE].want & CAN_WANT_VTP)) {
    s_tailVtp++;
  }
  return s_tailVtp != s_head;
}

bool canVtpPeek(CapFrame* out) {
  if (!vtpSeek()) return false;
  *out = s_ring[s_tailVtp % RING_SIZE];
  return true;
}

void canVtpPop() {
  if (s_tailVtp != s_head) s_tailVtp++;
}

uint16_t canVtpTakeDropped() {
  // Fold in the overrun before answering, or a batch sent on a quiet moment
  // reports zero while frames are sitting overwritten behind the cursor.
  catchUp(s_tailVtp, &s_vtpDropped);
  uint16_t d = s_vtpDropped;
  s_vtpDropped = 0;
  return d;
}

void canVtpReset() {
  s_tailVtp = s_head;
  s_vtpDropped = 0;
}

void canInjectSimulated(uint16_t rpm) {
  uint32_t now = millis();
  // 0x316 arrives at roughly 100 Hz on the car. Pacing the synthetic copy the
  // same way keeps frames/s and the app's rate column honest — a simulated bus
  // that ran at loop speed would report thousands of frames a second and look
  // nothing like the thing it stands in for.
  static uint32_t last = 0;
  if (now - last < 10) return;
  last = now;

  twai_message_t m = {};
  m.identifier = cfg.canRpmId;
  m.data_length_code = 8;
  // Inverse of the decode in canPoll(), so the app recovers exactly the RPM the
  // strip is running on rather than something a rounding step away from it.
  uint16_t raw = (uint16_t)(((uint32_t)rpm * cfg.rpmScaleX10) / 10);
  m.data[2] = (uint8_t)(raw & 0xFF);
  m.data[3] = (uint8_t)(raw >> 8);

  s_frameCount++;
  uint64_t tsUs = (uint64_t)esp_timer_get_time();
  uint8_t want = consumers(m, tsUs);
  if (want) ringPush(m, tsUs, want);
}

void canSetStream(uint8_t mode) {
  s_stream = mode;
  if (mode == SL_STREAM_OFF) s_tailLegacy = s_head;  // drop the backlog, don't ship it late
}

void canSetFilter(const uint32_t* ids, size_t count) {
  if (count > SL_MAX_FILTER) count = SL_MAX_FILTER;
  memcpy(s_filter, ids, count * sizeof(uint32_t));
  s_filterCount = count;
}
