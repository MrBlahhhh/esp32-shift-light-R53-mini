#include "canbus.h"
#include "settings.h"
#include <Arduino.h>
#include <driver/twai.h>
#include <driver/gpio.h>
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
#define RING_SIZE 128
static CanFrameRec s_ring[RING_SIZE];
static volatile size_t s_head = 0, s_tail = 0;

static void ringPush(const twai_message_t& m, uint32_t now) {
  CanFrameRec r;
  r.tsMs     = now;
  r.id       = m.identifier;
  r.dlcFlags = (uint8_t)(m.data_length_code & 0x0F);
  if (m.extd) r.dlcFlags |= SL_FRAME_EXT;
  memset(r.data, 0, sizeof(r.data));
  uint8_t n = m.data_length_code > 8 ? 8 : m.data_length_code;
  memcpy(r.data, m.data, n);

  size_t next = (s_head + 1) % RING_SIZE;
  if (next == s_tail) s_tail = (s_tail + 1) % RING_SIZE;  // drop oldest
  s_ring[s_head] = r;
  s_head = next;
}

static bool wanted(uint32_t id) {
  if (s_stream == SL_STREAM_OFF) return false;
  if (s_stream == SL_STREAM_ALL) return true;
  for (size_t i = 0; i < s_filterCount; i++) if (s_filter[i] == id) return true;
  return false;
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

    if (m.identifier == cfg.canRpmId && m.data_length_code >= 4) {
      uint16_t raw = (uint16_t)((m.data[3] << 8) | m.data[2]);
      s_rpm = (uint16_t)((raw * 10UL) / cfg.rpmScaleX10);
      s_lastRpmMs = now;
    }
    if (wanted(m.identifier)) ringPush(m, now);
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
  size_t n = 0;
  while (n < max && s_tail != s_head) {
    out[n++] = s_ring[s_tail];
    s_tail = (s_tail + 1) % RING_SIZE;
  }
  return n;
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
  if (wanted(m.identifier)) ringPush(m, now);
}

void canSetStream(uint8_t mode) {
  s_stream = mode;
  if (mode == SL_STREAM_OFF) s_tail = s_head;  // drop the backlog, don't ship it late
}

void canSetFilter(const uint32_t* ids, size_t count) {
  if (count > SL_MAX_FILTER) count = SL_MAX_FILTER;
  memcpy(s_filter, ids, count * sizeof(uint32_t));
  s_filterCount = count;
}
