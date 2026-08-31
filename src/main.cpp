// R53 shift light — CAN-driven WS2812B strip with BLE configuration.
//
// One ESP32-C3 in the footwell reads RPM off the MINI's CAN bus, drives an
// eight-LED strip, and serves a BLE service the Android app uses to set the
// thresholds, colours and brightness, and to watch the bus.
//
// The strip works with no phone connected and no app running. Everything the
// app does is configuration and observation; nothing on the BLE side is in the
// path between CAN and the LEDs.
//
// Wiring and pin choices live in platformio.ini. The BLE wire format lives in
// proto.h and is mirrored field-for-field by the app.

#include <Arduino.h>
#include "proto.h"
#include "settings.h"
#include "canbus.h"
#include "shiftlight.h"
#include "blesvc.h"

static uint32_t s_lastRender = 0;
static uint32_t s_lastPrint  = 0;

void setup() {
  Serial.begin(115200);
  delay(200);   // let USB CDC enumerate, or the boot lines go nowhere

  Serial.println();
  Serial.printf("R53 shift light — proto v%d\n", PROTO_VERSION);

  settingsBegin();

#ifdef SIMULATE_RPM
  // The build flag wins at power-on so a board on the bench sweeps with no
  // phone nearby. The app can still clear the flag afterwards.
  cfg.flags |= SL_FLAG_SIMULATE;
  Serial.println("SIMULATE_RPM compiled in — ignoring CAN for RPM");
#endif

  // Order matters. Each of these claims an RMT channel, and a controller that
  // fails to get one goes silently dead — no error, no log line, just a strip
  // that never lights. The shift light is the point of the board, so it asks
  // first and the status LED takes what is left.
  shiftlightBegin();
  statusBegin();

  // Brought up even when simulating: the app's bus view is still worth having
  // on a bench board, and a simulated RPM is flagged as such in telemetry so a
  // log can never mistake it for a real reading.
  canBegin();

  bleBegin();
}

void loop() {
  uint32_t now = millis();

  canPoll();

  bool simulating = cfg.flags & SL_FLAG_SIMULATE;
  uint16_t rpm = simulating ? simulatedRpm() : canRpm();

  // Feed the synthetic frame into the same ring the real bus fills, so the
  // phone's CAN view shows 0x316 carrying exactly this RPM. It stays flagged as
  // simulated in telemetry — a log must never be able to mistake it for the car.
  if (simulating) canInjectSimulated(rpm);

  if (now - s_lastRender >= (1000 / LED_HZ)) {
    s_lastRender = now;
    shiftlightRender(rpm);
    statusUpdate(canUp(), bleConnected());
  }

  blePoll(rpm);

  if (now - s_lastPrint >= 1000) {
    s_lastPrint = now;
    Serial.printf("rpm %4u%s  leds %u/%u  can %s %u f/s missed %u  ble %s%s\n",
                  rpm, simulating ? " (sim)" : "",
                  shiftlightLevel(), cfg.numLeds,
                  canUp() ? "up" : "DOWN", canFramesPerSec(), canRxMissed(),
                  bleConnected() ? "connected" : "advertising",
                  settingsUnsaved() ? "  [unsaved]" : "");
  }

  // Yield, or the idle task never runs and the watchdog eventually says so.
  vTaskDelay(1 / portTICK_PERIOD_MS);
}
