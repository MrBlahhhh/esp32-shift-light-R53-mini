#include "shiftlight.h"
#include "settings.h"
#include <Arduino.h>
#include <FastLED.h>

// From platformio.ini only. No fallback: on the carrier GPIO5 is the CAN
// transceiver's TXD, and a guessed pin could clock LED data onto the car's bus.
#ifndef LED_GPIO
#error "LED_GPIO is not defined; set it in platformio.ini"
#endif
#ifndef STATUS_LED_MODE
#define STATUS_LED_MODE 0
#endif
#ifndef STATUS_LED_ACTIVE_LOW
#define STATUS_LED_ACTIVE_LOW 0
#endif
// The S3-Zero's onboard pixel is RGB-ordered, unlike the GRB strip on LED_GPIO.
// Get this wrong and red and green swap while blue looks right, because blue is
// the last byte either way.
#ifndef STATUS_LED_ORDER
#define STATUS_LED_ORDER RGB
#endif

// Each threshold switches on exactly where it is set and back off only this far
// below it, so a few rpm of jitter in 0x316 cannot flicker an LED step, the
// colour change at rpmMid or the blink.
#define SHIFT_HYSTERESIS_RPM 75
#define RENDER_MS (1000 / LED_HZ)

static CRGB     s_leds[SL_MAX_LEDS];
static uint8_t  s_level        = 0;
static uint16_t s_heldRpm      = 0;   // what the strip is drawn from
static uint32_t s_blinkRenders = 0;   // renders since the blink started

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

// Follows the engine up at once and down only once it is SHIFT_HYSTERESIS_RPM
// below, which puts the dead band under every threshold at once.
static uint16_t holdRpm(uint16_t engineRpm) {
  bool noReading = engineRpm == 0;   // canRpm() once stale: go dark now, not 75 rpm later
  if (noReading || engineRpm >= s_heldRpm) {
    s_heldRpm = engineRpm;
  } else if (s_heldRpm - engineRpm > SHIFT_HYSTERESIS_RPM) {
    s_heldRpm = engineRpm + SHIFT_HYSTERESIS_RPM;
  }
  return s_heldRpm;
}

void shiftlightRender(uint16_t engineRpm) {
  uint16_t rpm = holdRpm(engineRpm);
  uint8_t n = cfg.numLeds > SL_MAX_LEDS ? SL_MAX_LEDS : cfg.numLeds;
  bool mirrored = cfg.flags & SL_FLAG_MIRRORED;

  // In mirrored mode a "slot" is a pair lit from both ends inward, so an odd
  // strip length leaves the middle LED permanently dark rather than lighting
  // half a pair.
  uint8_t slots = mirrored ? (n / 2) : n;

  fill_solid(s_leds, SL_MAX_LEDS, CRGB::Black);

  if (!(cfg.flags & SL_FLAG_ENABLED) || slots == 0 || rpm < cfg.rpmStart) {
    s_level = 0;
    s_blinkRenders = 0;
    FastLED.setBrightness(cfg.brightness);
    FastLED.show();
    return;
  }

  CRGB color;
  uint8_t level;

  // The divisions below are guarded even though valid() rejects zero spans and
  // periods: an integer divide by zero is a panic on the S3.
  if (rpm >= cfg.rpmBlink) {
    // Counted in renders, not read off millis(): the strip is only redrawn
    // every RENDER_MS, and a clock phase sampled that coarsely aliases into an
    // uneven blink. The period rounds to whole renders; each blink starts lit.
    uint32_t rendersPerHalf = (cfg.blinkPeriodMs / 2 + RENDER_MS / 2) / RENDER_MS;
    if (rendersPerHalf == 0) rendersPerHalf = 1;
    bool on = (s_blinkRenders / rendersPerHalf) % 2 == 0;
    s_blinkRenders++;
    color = on ? CRGB(cfg.colorBlink[0], cfg.colorBlink[1], cfg.colorBlink[2])
               : CRGB::Black;
    level = slots;
  } else {
    s_blinkRenders = 0;

    if (rpm >= cfg.rpmRedline || cfg.rpmRedline <= cfg.rpmStart) {
      level = slots;
    } else {
      uint32_t span = cfg.rpmRedline - cfg.rpmStart;
      uint32_t up   = rpm - cfg.rpmStart;
      level = (uint8_t)(1 + (up * (slots - 1)) / span);
    }

    if (rpm < cfg.rpmMid) {
      color = CRGB(cfg.colorLow[0], cfg.colorLow[1], cfg.colorLow[2]);
    } else {
      uint8_t t = 255;
      if (rpm < cfg.rpmRedline) {   // so rpmRedline > rpmMid and the span is not zero
        t = (uint8_t)(((uint32_t)(rpm - cfg.rpmMid) * 255) / (cfg.rpmRedline - cfg.rpmMid));
      }
      color = blend8(cfg.colorMid, cfg.colorHigh, t);
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
// Steady = CAN up, meaning a frame heard in the last CAN_SILENT_MS. 1 Hz blink =
// CAN down, which from the driver's seat looks like "engine off" until you
// look. Dark = no power or no firmware running. BLE state is shown only on the
// addressable variant, where it can have a colour of its own.

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
  bool on = canOk || ((millis() / 500) & 1) == 0;   // lit while up, so dark means dead
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
