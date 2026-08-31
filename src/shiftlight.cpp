#include "shiftlight.h"
#include "settings.h"
#include <Arduino.h>
#include <FastLED.h>

#ifndef LED_GPIO
#define LED_GPIO 5
#endif
#ifndef STATUS_LED_MODE
#define STATUS_LED_MODE 0
#endif
#ifndef STATUS_LED_ACTIVE_LOW
#define STATUS_LED_ACTIVE_LOW 0
#endif
// The S3-Zero's onboard pixel is RGB-ordered, unlike the GRB strip on LED_GPIO.
// Get this wrong and red and green swap while blue looks perfect, because blue
// is the last byte either way — which is a genuinely confusing thing to debug.
#ifndef STATUS_LED_ORDER
#define STATUS_LED_ORDER RGB
#endif

static CRGB    s_leds[SL_MAX_LEDS];
static uint8_t s_level = 0;

#if STATUS_LED_MODE == 2
static CRGB s_status[1];
#endif

void shiftlightBegin() {
  // The controller is created for the full buffer, not cfg.numLeds. FastLED
  // cannot be re-added at runtime, so a strip length arriving from the phone
  // would otherwise need a reboot to take effect; the tail is simply held black
  // and clocked out to LEDs that are not fitted, which costs nothing.
  FastLED.addLeds<WS2812B, LED_GPIO, GRB>(s_leds, SL_MAX_LEDS);
  fill_solid(s_leds, SL_MAX_LEDS, CRGB::Black);
  FastLED.setBrightness(cfg.brightness);
  FastLED.show();
}

static inline CRGB blend8(const uint8_t a[3], const uint8_t b[3], uint8_t t) {
  return CRGB(a[0] + (((int)b[0] - a[0]) * t) / 255,
              a[1] + (((int)b[1] - a[1]) * t) / 255,
              a[2] + (((int)b[2] - a[2]) * t) / 255);
}

void shiftlightRender(uint16_t rpm) {
  uint8_t n = cfg.numLeds > SL_MAX_LEDS ? SL_MAX_LEDS : cfg.numLeds;
  bool mirrored = cfg.flags & SL_FLAG_MIRRORED;

  // In mirrored mode a "slot" is a pair lit from both ends inward, so an odd
  // strip length leaves the middle LED permanently dark rather than lighting
  // half a pair.
  uint8_t slots = mirrored ? (n / 2) : n;

  fill_solid(s_leds, SL_MAX_LEDS, CRGB::Black);

  if (!(cfg.flags & SL_FLAG_ENABLED) || slots == 0 || rpm < cfg.rpmStart) {
    s_level = 0;
    FastLED.setBrightness(cfg.brightness);
    FastLED.show();
    return;
  }

  CRGB color;
  uint8_t level;

  if (rpm >= cfg.rpmBlink) {
    // Derived from the clock rather than a toggled flag: a missed render — and
    // at 20 Hz alongside CAN and BLE there will be missed renders — then shows
    // up as one skipped frame instead of inverting the blink from there on.
    bool on = ((millis() / (cfg.blinkPeriodMs / 2)) & 1) == 0;
    color = on ? CRGB(cfg.colorBlink[0], cfg.colorBlink[1], cfg.colorBlink[2])
               : CRGB::Black;
    level = slots;
  } else {
    uint16_t span = cfg.rpmRedline - cfg.rpmStart;   // valid() guarantees > 0
    uint32_t up   = rpm - cfg.rpmStart;
    level = 1 + (uint8_t)((up * (slots - 1)) / span);
    if (level > slots) level = slots;

    if (rpm < cfg.rpmMid) {
      color = CRGB(cfg.colorLow[0], cfg.colorLow[1], cfg.colorLow[2]);
    } else {
      uint16_t cspan = cfg.rpmRedline - cfg.rpmMid;  // valid() guarantees > 0
      uint32_t t = ((uint32_t)(rpm - cfg.rpmMid) * 255) / cspan;
      color = blend8(cfg.colorMid, cfg.colorHigh, t > 255 ? 255 : (uint8_t)t);
    }
  }

  for (uint8_t i = 0; i < level; i++) {
    if (mirrored) {
      s_leds[i]         = color;
      s_leds[n - 1 - i] = color;
    } else {
      s_leds[i] = color;
    }
  }

  s_level = level;
  FastLED.setBrightness(cfg.brightness);
  FastLED.show();
}

uint8_t shiftlightLevel() { return s_level; }

void shiftlightIdentify() {
  uint8_t n = cfg.numLeds > SL_MAX_LEDS ? SL_MAX_LEDS : cfg.numLeds;
  for (int flash = 0; flash < 3; flash++) {
    fill_solid(s_leds, n, CRGB::White);
    FastLED.show();
    delay(120);
    fill_solid(s_leds, SL_MAX_LEDS, CRGB::Black);
    FastLED.show();
    delay(120);
  }
}

// --- Status indicator -------------------------------------------------------
// Steady = CAN up. 1 Hz blink = CAN down, which is the fault worth seeing from
// the driver's seat because it is indistinguishable from "engine off" until you
// look. BLE state is shown only on the addressable variant, where it can have a
// colour of its own instead of competing for the same blink pattern.

void statusBegin() {
#if STATUS_LED_MODE == 1
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, STATUS_LED_ACTIVE_LOW ? HIGH : LOW);
#elif STATUS_LED_MODE == 2
  FastLED.addLeds<WS2812B, STATUS_LED_PIN, STATUS_LED_ORDER>(s_status, 1);
  s_status[0] = CRGB::Black;
#endif
}

void statusUpdate(bool canOk, bool bleConnected) {
#if STATUS_LED_MODE == 1
  (void)bleConnected;
  bool on = canOk ? false : (((millis() / 500) & 1) == 0);
  digitalWrite(STATUS_LED_PIN, (on != (bool)STATUS_LED_ACTIVE_LOW) ? HIGH : LOW);
#elif STATUS_LED_MODE == 2
  // Near-full channel values on purpose. FastLED's master brightness is global
  // and set from cfg.brightness for the strip, so anything subtle here gets
  // scaled down with it and disappears at the low brightness the strip wants.
  if (!canOk) {
    s_status[0] = (((millis() / 500) & 1) == 0) ? CRGB(255, 0, 0) : CRGB::Black;
  } else {
    s_status[0] = bleConnected ? CRGB(0, 0, 200) : CRGB(0, 140, 0);
  }
#else
  (void)canOk; (void)bleConnected;
#endif
}

// --- Simulator --------------------------------------------------------------

uint16_t simulatedRpm() {
  const uint32_t period = 10000;
  uint32_t t = millis() % period;
  return (uint16_t)(1000 + (t * 8000UL) / period);   // 1000 -> 9000 ramp
}
