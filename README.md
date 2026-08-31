# R53 shift light — CAN, WS2812B, and a phone to set it up

An ESP32-S3 in the footwell of a first-generation MINI Cooper S. It reads RPM off
the car's CAN bus, drives an eight-LED strip on the A-pillar, and serves a BLE
service an Android app uses to set the thresholds, colours and brightness, and to
watch the bus.

The strip works with no phone connected and no app running. Everything the app
does is configuration and observation — nothing on the BLE side sits between CAN
and the LEDs.

> **Not AWS Route 53.** "R53" is the chassis code for the 2002–2006 supercharged
> MINI Cooper S.

## Hardware

| Part | Qty | Notes |
|---|---:|---|
| Waveshare ESP32-S3-Zero | 1 | ESP32-S3FH4R2 — 4 MB flash, 2 MB quad PSRAM |
| SN65HVD230 CAN breakout | 1 | blue screw-terminal type |
| 8 × WS2812B strip | 1 | the shift light itself |

### Pin map

| GPIO | Goes to |
|---|---|
| `4` | WS2812B data in |
| `5` | SN65HVD230 `TXD` |
| `6` | SN65HVD230 `RXD` |
| `21` | onboard WS2812 — CAN/BLE status |

`TXD` and `RXD` go **straight across, not crossed**. On the transceiver module
`TXD` is an input the micro drives and `RXD` is an output. Swapping them is the
classic reason a freshly built CAN node hears nothing.

GPIO19 and GPIO20 are the native USB pair on this board and are left alone.

The onboard LED on the S3-Zero is an **addressable WS2812**, not a plain one, so
it is driven as a one-pixel strip (`STATUS_LED_MODE=2`). Driving it with
`digitalWrite`, as the older firmware in this family did, leaves it dark or stuck
on whatever colour the first stray pulse happened to clock in.

Status colours: green = CAN up, blue = CAN up and a phone connected, blinking red
= CAN down.

The node is **listen-only**. It is configured `TWAI_MODE_LISTEN_ONLY` and never
drives a dominant bit, so it cannot acknowledge or disturb the car's bus.

## Firmware

PlatformIO, one env:

```sh
pio run -e esp32-s3-zero -t upload
pio device monitor
```

There is no stock `esp32-s3-fh4r2` board in the platform — the older repos in
this family pointed at a community board JSON. `esp32-s3-devkitc1-n4r2` is the
stock definition for the same silicon (N4 = 4 MB flash, R2 = 2 MB quad PSRAM),
which is what the chip reports.

Every pin and toggle is a build flag in [`platformio.ini`](platformio.ini) —
rewiring is an edit there, never to the source. Uncomment `-DSIMULATE_RPM` to
boot straight into an RPM sweep and ignore CAN, which is how you exercise a
strip on the bench with no car attached.

`min_spiffs` is not optional: NimBLE plus the Arduino core plus FastLED does not
fit the default 1.3 MB app partition, and the overflow surfaces as a link error
with no obvious connection to Bluetooth.

### RPM

RPM comes off CAN id `0x316`, bytes 2–3 little-endian, divided by 6.4. Both the
id and the divisor are in the config blob, so a different car is a setting
rather than a rebuild.

A stale reading is reported as *no* reading. If RPM frames stop for two seconds
the strip goes dark rather than holding whatever the engine was doing when the
wire fell off.

## The app

Kotlin and Compose. The app is not part of this repository and is not open
source.

It does three things:

- **Configure.** Thresholds, four colours, brightness, LED count, fill
  direction. A mimic of the strip updates from live RPM, so thresholds get
  dialled in against a real engine instead of guessed at.
- **Watch.** RPM, bus state, frame rate, and whether the board has unsaved
  changes.
- **Log.** Every CAN id on the bus with its rate and latest payload. Streaming
  is off until you switch it on — a phone that is not looking should not cost
  the board airtime it needs for telemetry.

### Apply and Save are different

**Apply** writes the config to RAM. The strip changes immediately and the change
is gone at the next power cycle. That is the mode for dialling thresholds in
with the engine running: dragging a slider does not burn a flash write per
frame.

**Save** commits what the board is running to NVS, where it survives power-off.

The board reports which state it is in. Whenever the live config differs from
what is in flash it sets `SL_TLM_UNSAVED` and the app says so. That includes
first boot on a blank board, where the defaults are running but are not yet
committed — telling you otherwise would lose your first edit.

## The wire format

[`src/proto.h`](src/proto.h) is the contract, and the app mirrors it field for
field. Change one without the other and the strip renders a colour channel as a
redline.

The header is published here in full so anything else — a script, another app,
nRF Connect — can talk to the board.

Everything is fixed-size and little-endian. A blob of unexpected length is
rejected at both ends rather than parsed as far as it goes — a short read means
the app and firmware disagree about the protocol, and guessing at the tail is
how a bad threshold reaches the LEDs.

| Characteristic | Dir | Payload |
|---|---|---|
| `…0002` | read/write | `ConfigBlob`, 32 bytes |
| `…0003` | notify | `TelemetryBlob`, 12 bytes, 10 Hz |
| `…0004` | notify | count byte + up to 13 × `CanFrameRec` |
| `…0005` | write | one opcode byte, plus arguments |

Both ends pin these byte offsets in tests. They are the only thing standing
between a one-byte layout drift and a shift light that looks fine and is wrong.

## Related

- [esp32-canbus-SN65HVD230-v2](https://github.com/MrBlahhhh/esp32-canbus-SN65HVD230-v2) — the CAN shift light this grew out of
