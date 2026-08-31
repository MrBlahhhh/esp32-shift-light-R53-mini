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
void settingsDefaults() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.version       = PROTO_VERSION;
  cfg.numLeds       = 8;
  cfg.brightness    = 75;
  cfg.flags         = SL_FLAG_ENABLED | SL_FLAG_MIRRORED;
  cfg.rpmStart      = 3500;
  cfg.rpmMid        = 6000;
  cfg.rpmRedline    = 7100;
  cfg.rpmBlink      = 7100;
  cfg.blinkPeriodMs = 200;
  cfg.colorLow[0]   = 0;   cfg.colorLow[1]   = 255; cfg.colorLow[2]   = 0;
  cfg.colorMid[0]   = 0;   cfg.colorMid[1]   = 255; cfg.colorMid[2]   = 0;
  cfg.colorHigh[0]  = 255; cfg.colorHigh[1]  = 0;   cfg.colorHigh[2]  = 0;
  cfg.colorBlink[0] = 255; cfg.colorBlink[1] = 0;   cfg.colorBlink[2] = 0;
  cfg.canRpmId      = 0x316;
  cfg.rpmScaleX10   = 64;   // raw / 6.4
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
  if (c.blinkPeriodMs < 40 || c.blinkPeriodMs > 5000) return false;
  return true;
}

bool settingsApply(const uint8_t* data, size_t len) {
  if (len != sizeof(ConfigBlob)) return false;
  ConfigBlob incoming;
  memcpy(&incoming, data, sizeof(incoming));
  if (!valid(incoming)) return false;
  cfg = incoming;
  return true;
}

bool settingsUnsaved() {
  return memcmp(&cfg, &saved, sizeof(cfg)) != 0;
}

bool settingsSave() {
  prefs.begin("shiftlight", false);
  size_t n = prefs.putBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
  if (n != sizeof(cfg)) return false;
  saved = cfg;
  return true;
}

void settingsBegin() {
  settingsDefaults();
  prefs.begin("shiftlight", true);
  ConfigBlob stored;
  size_t n = prefs.getBytes("cfg", &stored, sizeof(stored));
  prefs.end();

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
