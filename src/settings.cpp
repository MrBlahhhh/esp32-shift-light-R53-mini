#include "settings.h"
#include <Arduino.h>
#include <Preferences.h>
#include <string.h>

ConfigBlob cfg;

static Preferences prefs;
static ConfigBlob saved;   // last thing successfully written to NVS

// Defaults reproduce the strip's proven behaviour exactly: dark below 3500,
// green to 6000, green fading to red by 7100, then the whole strip blinking.
// Anyone flashing this over the old firmware should see no change until they
// open the app.
static ConfigBlob defaultConfig() {
  ConfigBlob d;
  memset(&d, 0, sizeof(d));
  d.version       = PROTO_VERSION;
  d.numLeds       = 8;
  d.brightness    = 75;
  d.flags         = SL_FLAG_ENABLED | SL_FLAG_MIRRORED;
  d.rpmStart      = 3500;
  d.rpmMid        = 6000;
  d.rpmRedline    = 7100;
  d.rpmBlink      = 7100;
  d.blinkPeriodMs = 200;
  d.colorLow[0]   = 0;   d.colorLow[1]   = 255; d.colorLow[2]   = 0;
  d.colorMid[0]   = 0;   d.colorMid[1]   = 255; d.colorMid[2]   = 0;
  d.colorHigh[0]  = 255; d.colorHigh[1]  = 0;   d.colorHigh[2]  = 0;
  d.colorBlink[0] = 255; d.colorBlink[1] = 0;   d.colorBlink[2] = 0;
  d.canRpmId      = 0x316;
  d.rpmScaleX10   = 64;   // raw / 6.4
  return d;
}

// Built whole and then assigned, so the render never sees a half-reset config.
void settingsDefaults() {
  cfg = defaultConfig();
}

// Rejects rather than clamps. A blob that fails these tests did not come from a
// version of the app that agrees with this firmware, and quietly repairing it
// would hide the mismatch until the strip did something surprising at 7000 rpm.
static bool valid(const ConfigBlob& c) {
  if (c.version != PROTO_VERSION)              return false;
  if (c.numLeds == 0 || c.numLeds > SL_MAX_LEDS) return false;
  if (c.rpmScaleX10 == 0)                      return false;
  // Thresholds must be monotonic, else the colour ramp divides by a negative
  // span and the bar fills backwards.
  if (!(c.rpmStart < c.rpmMid && c.rpmMid < c.rpmRedline)) return false;
  if (c.rpmBlink < c.rpmMid)                   return false;
  if (c.blinkPeriodMs < SL_BLINK_PERIOD_MIN_MS || c.blinkPeriodMs > 5000) return false;
  return true;
}

// The one repair. Firmware before SL_BLINK_PERIOD_MIN_MS accepted 40 ms, saved
// configs may hold it and the shipped apps still offer it, so 40..99 runs at
// the minimum instead of being rejected.
static void raiseBlinkPeriod(ConfigBlob& c) {
  if (c.blinkPeriodMs >= 40 && c.blinkPeriodMs < SL_BLINK_PERIOD_MIN_MS) {
    c.blinkPeriodMs = SL_BLINK_PERIOD_MIN_MS;
  }
}

// SL_FLAG_SIMULATE is runtime only: never written to NVS, and never the reason
// the live config counts as unsaved.
static ConfigBlob withoutSimulate(const ConfigBlob& c) {
  ConfigBlob persisted = c;
  persisted.flags &= ~SL_FLAG_SIMULATE;
  return persisted;
}

bool settingsApply(const uint8_t* data, size_t len) {
  if (len != sizeof(ConfigBlob)) return false;
  ConfigBlob incoming;
  memcpy(&incoming, data, sizeof(incoming));
  raiseBlinkPeriod(incoming);
  if (!valid(incoming)) return false;
  cfg = incoming;
  return true;
}

bool settingsUnsaved() {
  ConfigBlob live = withoutSimulate(cfg);
  return memcmp(&live, &saved, sizeof(live)) != 0;
}

bool settingsSave() {
  ConfigBlob persisted = withoutSimulate(cfg);
  prefs.begin("shiftlight", false);
  size_t n = prefs.putBytes("cfg", &persisted, sizeof(persisted));
  prefs.end();
  if (n != sizeof(persisted)) return false;
  saved = persisted;
  return true;
}

void settingsBegin() {
  settingsDefaults();
  prefs.begin("shiftlight", true);
  ConfigBlob stored = {};
  size_t n = prefs.getBytes("cfg", &stored, sizeof(stored));
  prefs.end();

  // Older firmware could save the simulate flag; a board must never boot into
  // the sweep off the back of it.
  stored = withoutSimulate(stored);
  raiseBlinkPeriod(stored);

  // A stored blob from an older PROTO_VERSION fails valid() and is discarded in
  // favour of defaults. That is deliberate: this is a shift light, and losing
  // four colour choices costs less than rendering a struct read at the wrong
  // offsets.
  if (n == sizeof(stored) && valid(stored)) {
    cfg = stored;
    saved = cfg;
    Serial.println("settings: loaded from NVS");
  } else {
    // Nothing usable in flash. Leave `saved` zeroed so settingsUnsaved() reports
    // true and the app offers a Save — defaults are running but are not yet
    // committed, and saying otherwise loses the user's first edit on the next
    // power cycle.
    memset(&saved, 0, sizeof(saved));
    Serial.printf("settings: defaults (stored %u bytes)\n", (unsigned)n);
  }
}
