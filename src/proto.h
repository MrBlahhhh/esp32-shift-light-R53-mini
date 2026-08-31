#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
// BLE wire format. Everything here is little-endian and byte-packed, and the
// Android app mirrors it field for field — change a struct in this file and you
// have changed the app's parser too.
//
// Every layout is fixed-size and carries no strings. A phone that reads a
// config blob of an unexpected length must reject it rather than parse what it
// can: a short read here means a firmware/app version mismatch, and guessing at
// the missing tail is how you end up writing a redline into a colour field.
// ---------------------------------------------------------------------------

#define PROTO_VERSION 1

#define SL_SERVICE_UUID   "6d5f0001-9c2b-4a7e-b8d3-5a1f2c4e8b70"
#define SL_CONFIG_UUID    "6d5f0002-9c2b-4a7e-b8d3-5a1f2c4e8b70"  // read/write
#define SL_TELEMETRY_UUID "6d5f0003-9c2b-4a7e-b8d3-5a1f2c4e8b70"  // notify
#define SL_CANFRAME_UUID  "6d5f0004-9c2b-4a7e-b8d3-5a1f2c4e8b70"  // notify
#define SL_COMMAND_UUID   "6d5f0005-9c2b-4a7e-b8d3-5a1f2c4e8b70"  // write

#define SL_DEVICE_NAME    "R53-ShiftLight"

// --- Config -----------------------------------------------------------------
// Written whole, never field-by-field: a partial write would leave the strip
// running a mix of old and new thresholds, which looks like a firmware bug and
// is not one.

struct __attribute__((packed)) ConfigBlob {
  uint8_t  version;        // == PROTO_VERSION, else the blob is rejected
  uint8_t  numLeds;        // logical strip length, 1..SL_MAX_LEDS
  uint8_t  brightness;     // FastLED master brightness, 0..255
  uint8_t  flags;          // SL_FLAG_*
  uint16_t rpmStart;       // first LED lights here
  uint16_t rpmMid;         // colour starts crossing from mid toward high
  uint16_t rpmRedline;     // bar full, colour fully at high
  uint16_t rpmBlink;       // whole strip blinks at and above this
  uint16_t blinkPeriodMs;  // full on+off cycle, so 200 is the old 5 Hz blink
  uint8_t  colorLow[3];    // R,G,B below rpmMid
  uint8_t  colorMid[3];    // R,G,B at rpmMid
  uint8_t  colorHigh[3];   // R,G,B at rpmRedline
  uint8_t  colorBlink[3];  // R,G,B while blinking
  uint32_t canRpmId;       // CAN id carrying RPM (0x316 on the R53)
  uint16_t rpmScaleX10;    // raw / (rpmScaleX10 / 10). 64 => raw / 6.4
};
static_assert(sizeof(ConfigBlob) == 32, "ConfigBlob must stay 32 bytes");

#define SL_FLAG_ENABLED   0x01  // strip off entirely when clear
#define SL_FLAG_MIRRORED  0x02  // fill in pairs from both ends inward
#define SL_FLAG_SIMULATE  0x04  // sweep RPM instead of reading CAN

// --- Telemetry --------------------------------------------------------------
// Pushed at a fixed rate whether or not anything changed. A display that stops
// updating is then unambiguously a dropped link, not a quiet bus.

struct __attribute__((packed)) TelemetryBlob {
  uint16_t rpm;
  uint8_t  flags;         // SL_TLM_*
  uint8_t  ledLevel;      // lit LEDs (or pairs when mirrored), for the app's mimic
  uint16_t framesPerSec;  // whole-bus rate, not just the RPM id
  uint16_t rxMissed;      // driver-side overruns since boot, saturating
  uint32_t uptimeSec;
};
static_assert(sizeof(TelemetryBlob) == 12, "TelemetryBlob must stay 12 bytes");

#define SL_TLM_CAN_UP     0x01  // TWAI running, not bus-off or stopped
#define SL_TLM_RPM_FRESH  0x02  // an RPM frame arrived inside RPM_STALE_MS
#define SL_TLM_SIMULATING 0x04  // rpm is synthetic — never log this as real
#define SL_TLM_UNSAVED    0x08  // live config differs from what is in NVS

// --- CAN frames -------------------------------------------------------------
// Batched, because one notify per frame cannot keep up: a quiet R53 bus is
// already ~150 frames/s and BLE will not carry that as 150 separate packets.

struct __attribute__((packed)) CanFrameRec {
  uint32_t tsMs;      // millis() at capture; wraps every ~49 days
  uint32_t id;
  uint8_t  dlcFlags;  // low nibble = dlc, 0x80 = extended id
  uint8_t  data[8];
};
static_assert(sizeof(CanFrameRec) == 17, "CanFrameRec must stay 17 bytes");

#define SL_FRAME_EXT      0x80
#define SL_FRAMES_PER_PKT 13   // 1 + 13*17 = 222 bytes, inside a 244-byte MTU

// --- Commands ---------------------------------------------------------------

#define SL_CMD_SAVE        1  // commit live config to NVS
#define SL_CMD_DEFAULTS    2  // reset live config only — the app must still save
#define SL_CMD_STREAM      3  // payload[1]: 0 off, 1 all ids, 2 filtered
#define SL_CMD_FILTER      4  // payload[1..]: up to SL_MAX_FILTER u32 ids
#define SL_CMD_REBOOT      5
#define SL_CMD_IDENTIFY    6  // flash the strip so you can tell two boards apart

#define SL_STREAM_OFF      0
#define SL_STREAM_ALL      1
#define SL_STREAM_FILTERED 2

#define SL_MAX_LEDS   32
#define SL_MAX_FILTER 16
