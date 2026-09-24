# R53 shift light — CAN, WS2812B, and a phone to set it up

An ESP32 behind the dash of a first-generation MINI Cooper S. It reads RPM off
the car's CAN bus, drives an eight-LED strip on the steering column shroud, and serves a BLE
service an Android app uses to set the thresholds, colours and brightness, and to
watch the bus. It also serves [VTP/1](#vtp1) on the same connection, so a logger
that speaks no shift light at all can still take the CAN bus off it.

The strip works with no phone connected and no app running. Everything the app
does is configuration and observation — nothing on the BLE side sits between CAN
and the LEDs.

> **Not AWS Route 53.** "R53" is the chassis code for the 2002–2006 supercharged
> MINI Cooper S.

## Hardware

Two builds, one firmware. The pin map below is identical on both.

The **prototype** is loose parts, and it is the one still in the car:

| Part | Qty | Notes |
|---|---:|---|
| Waveshare ESP32-S3-Zero | 1 | ESP32-S3FH4R2 — 4 MB flash, 2 MB quad PSRAM |
| SN65HVD230 CAN breakout | 1 | blue screw-terminal type |
| 8 × WS2812B strip | 1 | the shift light itself |

There is also a **54 × 58 mm carrier board** that replaces the dev board, the CAN
breakout, the buck module and the jumper-wire harness between them, with an
**ESP32-C3 SuperMini** soldered flat in the middle and a D-SUN MP1584 buck on
four through-holes at the top. **Its design files are not in this repo.**

| Part | Qty | Notes |
|---|---:|---|
| Carrier board (rev B) | 1 | 12 V in, CAN, strip out |
| ESP32-C3 SuperMini | 1 | castellated module, USB-C at the top when face up |
| D-SUN MP1584EN | 1 | solder-on buck, trim to 5.0 V before housing |
| 8 × WS2812B strip | 1 | same strip |

### Pin map

Strip and CAN pins are identical on both boards.

| GPIO | Goes to |
|---|---|
| `4` | WS2812B data in |
| `5` | SN65HVD230 `TXD` |
| `6` | SN65HVD230 `RXD` |

Status LED differs by module:

| Board | GPIO | LED type |
|---|---|---|
| Carrier (C3 SuperMini) | `8` | plain blue LED on the module, active low |
| Prototype (S3-Zero) | `21` | addressable WS2812 on the module |

`TXD` and `RXD` go **straight across, not crossed**. On the transceiver `TXD`
is an input the micro drives and `RXD` is an output. Swapping them is the
classic reason a freshly built CAN node hears nothing.

On the **C3 SuperMini**, component side up with USB-C at the top: the right edge
(top → bottom) is 5V, GND, 3V3, IO4, IO3, IO2, IO1, IO0 and the left edge is
IO5, IO6, IO7, IO8, IO9, IO10, IO20, IO21. IO4 is the strip, IO5 CAN TX, IO6
CAN RX. IO8 and IO9 (BOOT) stay on the module.

On the **S3-Zero**, GPIO19 and GPIO20 are the native USB pair and are left
alone. The onboard LED is a WS2812, not a plain one, so it is driven as a
one-pixel strip (`STATUS_LED_MODE=2`). Driving it with `digitalWrite` leaves it
dark or stuck on whatever colour the first stray pulse happened to clock in.

The status LED shows whether CAN is up, and up means a frame heard in the last
500 ms. The TWAI driver says it's running on a dead or unplugged bus, so its
state isn't the test. On the S3-Zero: green = CAN up, blue = CAN up and a phone
connected, blinking red = CAN down. On the C3's plain LED: steady = CAN up,
1 Hz blink = CAN down, dark = no power or no firmware running. The app's "CAN
down" uses the same 500 ms test.

The node is **listen-only**, and that takes two things. `TWAI_MODE_LISTEN_ONLY`
stops the ACK. But on the ESP32, S2, S3 and C3 the controller still sends
dominant error frames in that mode unless the IDF was built with
`CONFIG_TWAI_ERRATA_FIX_LISTEN_ONLY_DOM`, which holds it error-passive. With
both, it never drives a dominant bit and can't acknowledge or disturb the car's
bus. `canbus.cpp` refuses to compile without the option, which is why the
platform is pinned (see [Firmware](#firmware)).

## Firmware

PlatformIO, two envs — one per module:

```sh
# Carrier board (ESP32-C3 SuperMini)
pio run -e esp32-c3 -t upload

# Prototype (Waveshare ESP32-S3-Zero, still in the car)
pio run -e esp32-s3-zero -t upload

pio device monitor
```

There is no stock `esp32-s3-fh4r2` board in the platform — the older repos in
this family pointed at a community board JSON. `esp32-s3-devkitc1-n4r2` is the
stock definition for the same silicon (N4 = 4 MB flash, R2 = 2 MB quad PSRAM),
which is what the chip reports. The C3 env uses `esp32-c3-devkitm-1`, which
matches the SuperMini's pinout close enough.

The platform is pinned to pioarduino
[55.03.37](https://github.com/pioarduino/platform-espressif32/releases/tag/55.03.37)
(Arduino core 3.3.7, IDF 5.5.2), whose prebuilt sdkconfig has the listen-only
errata fix on. Don't put back a bare `platform = espressif32`: that resolves to
whichever espressif32 platform is newest on the machine, and a clean machine
gets the official 6.x line on IDF 4.4. Moving to a newer pioarduino release is
safe for the bus as long as `canbus.cpp` still compiles, since it checks for
the option.

Every pin and toggle is a build flag in [`platformio.ini`](platformio.ini), so
rewiring is an edit there, never to the source. There are no fallback pins: a
build missing `LED_GPIO`, `CAN_TX_GPIO` or `CAN_RX_GPIO` stops with an error,
because on the carrier GPIO5 is the transceiver's `TXD` and a guessed default
could clock LED data onto the car's bus. Uncomment `-DSIMULATE_RPM` to boot
straight into an RPM sweep and ignore CAN, which is how you exercise a strip on
the bench with no car attached.

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

Every threshold has 75 rpm of hysteresis. An LED step, the colour change at
`rpmMid` and the blink each switch on exactly at their set point and back off
only 75 rpm below it, so jitter on `0x316` can't flicker the strip at a
boundary.

The blink is counted in renders (20 Hz), not read off the clock, which would
alias against the render. The shortest period is 100 ms, one render on and one
off, and any period is shown rounded to a multiple of 100 ms. A config asking
for 40 to 99 ms, which older firmware and the current apps allow, runs at
100 ms instead of being rejected, and a saved one is raised the same way at
boot. `SL_BLINK_PERIOD_MIN_MS` in `proto.h` is the number for the apps to match.

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

Simulate is the exception. The flag is live only: Save never writes it and it
never makes the config count as unsaved, so a board can't end up booting into
the sweep because someone saved while testing. While simulating, the frame
stream carries the synthetic `0x316` in place of the car's.

## Pairing

Changing anything on the board needs a phone paired with its PIN, **530053**.
Every board has the same one. It keeps random phones in a car park from moving
your redline or rebooting the board. It doesn't keep out anyone who has the app
or knows the PIN, and isn't meant to.

What needs the PIN is every write on the shift light service: the config, and
every command (save, defaults, reboot, identify, and the frame stream on/off
and filter). What stays open:

- Reading the config and telemetry, and the telemetry notifications. A phone
  that hasn't paired can connect and watch the RPM, but can't change anything.
- The whole [VTP/1](#vtp1) service. A logger that only speaks VTP is never
  asked to pair, and still gets the bus.

It's LE Secure Connections with a fixed passkey. `bleBegin()` turns on bonding,
MITM protection and SC, declares the board DisplayOnly so the phone asks for a
passkey, and answers every passkey request with the PIN
(`setSecurityAuth(true, true, true)`, `setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY)`,
`setSecurityPasskey(SHIFTLIGHT_PAIRING_PIN)`). The config and command
characteristics are `WRITE_ENC | WRITE_AUTHEN`: the link has to be encrypted
with a key that came from the PIN, so Just Works from a central with no keyboard
is refused as well. The board never starts pairing itself.

- **Android app:** pairs as soon as it connects and types the PIN in for you.
  Some phones flash the system PIN box for a moment anyway.
- **Web app (Chrome, Bluefy on iPhone):** the first Apply, Save or command
  makes the phone ask for a PIN. Type 530053. The app shows it on the Shift
  light tab, and again if pairing fails.
- **nRF Connect or a script:** bond, and enter 530053 when asked.

Up to eight phones are remembered, in NVS, across power cycles. A ninth pushes
out the one paired longest ago. The boot log says how many are stored:

```
BLE: settings need pairing, PIN 530053; 2 of 8 phones paired (type 'forget' to clear them)
```

**Changing the PIN.** Uncomment `-DSHIFTLIGHT_PAIRING_PIN=` in
[`platformio.ini`](platformio.ini) and give it six digits. The apps carry the
same number, `Proto.PAIRING_PIN` in the Android app and `PAIRING_PIN` in the web
app's `src/lib/proto.ts`, so change both with it. Android can't type a PIN it
doesn't know, and the web app would show the wrong one.

**Forgetting phones.** Open a serial monitor at 115200 (`pio device monitor`),
type `forget` and press Enter. Every stored pairing goes, and a paired phone
that's connected is dropped. Erasing the flash (`pio run -e esp32-c3 -t erase`)
does the same and also wipes the saved settings. Either way the phone still
holds its half of the old pairing, so remove the board from the phone's
Bluetooth settings before connecting again, or the reconnect fails.

**Flashing this over older firmware.** Older builds never paired, so there is
nothing to clear. Each phone pairs once, the first time the Android app connects
or the first time the web app changes something.

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
| `…0002` | read, write (PIN) | `ConfigBlob`, 32 bytes |
| `…0003` | notify | `TelemetryBlob`, 12 bytes, 10 Hz |
| `…0004` | notify | count byte + up to 13 × `CanFrameRec` |
| `…0005` | write (PIN) | one opcode byte, plus arguments |

"(PIN)" means the write needs a link paired with the PIN; see
[Pairing](#pairing). Without one the board answers Insufficient Authentication
(Insufficient Encryption from a phone it has a key for that hasn't encrypted yet).

Both ends pin these byte offsets in tests. They are the only thing standing
between a one-byte layout drift and a shift light that looks fine and is wrong.

## VTP/1

The board serves a second GATT service:
[VTP/1](https://github.com/Lapsmith-app/VTP), the open protocol for carrying
CAN, GNSS and inertial data off a hand-built logger. Same connection, same
board, nothing removed. This one carries the bus and only the bus — every byte
that makes the thing a shift light is still on the service above, and an app
that has never heard of VTP works exactly as it did.

Capabilities are `can`, `control` and `masked_subscriptions`. There is no GNSS
and no IMU here, so those bits are clear and their characteristics are *inert*
rather than absent: §4.1 fixes the attribute table, because a central caches it
across connections and a table that changes shape between them hands the next
client a stale handle.

| | |
|---|---|
| Service | `56545001-5f05-5b56-af87-dcab2baf2522` |
| Subscription slots | 8, each `(id, mask)` over bits 0–29 |
| Schedule slots | 24 `(subscription, identifier)` pairs |
| Declared rate | 1000 frames/s |
| Batch | one every 10 ms, sized to the negotiated MTU |
| Clock | `esp_timer`, microseconds, monotonic from boot |

Nothing streams until a client subscribes. `CAN_RESET` then `CAN_SUBSCRIBE` per
identifier is the whole setup, and both stream and subscriptions die with the
connection — a client always finds a known table and never inherits one it did
not install.

Two things are worth knowing before trusting a timestamp from this board. The
arrival time is taken when the TWAI driver hands the frame over, not at
end-of-frame on the wire, so it is later than §6.7 asks for by however long the
frame sat in the rx queue. And the encoder is not written here: `lib/vtp1` is
the reference encoder from the VTP repo, vendored unmodified, so a batch on the
wire is byte for byte what the conformance corpus says a batch is.

VTP only ever carries the real bus. VTP/1 has no way to flag a frame as
synthetic, so the simulated `0x316` never reaches it, and a VTP client sees the
car's own `0x316` even while the strip is sweeping.

### Checking it

The VTP repository ships a conformance harness that connects to a device and
tests it against the specification — the control plane, `seq` starting at zero
per connection, a frame matching two subscriptions being forwarded once, a
re-install costing the client nothing:

```sh
cd ../VTP && uv run vtp1-harness
```

That, and not a successful build, is what says this firmware is conformant. It
is also the first time VTP/1 has run on a microcontroller at all, so a failure
is as likely to be a finding about the harness as about this board.

### What it costs the shift light

Nothing on the CAN path: the frame ring grew a second read cursor, so the two
protocols each see every frame they subscribed to and neither consumes the
other's.

One thing did change on air. Two 128-bit service UUIDs will not fit in one
advertisement — flags are 3 bytes and a UUID is 18 — so the primary packet
keeps the shift light's UUID, byte for byte as before, and the scan response
carries VTP's next to a **shortened** name, `R53-Shift`. The full
`R53-ShiftLight` is still the GAP name once connected. Anything matching on the
service UUID is untouched, which is every scan filter; anything matching the
advertised name for an exact string is not.

## Two generations of board

Boards flashed before VTP was added are in cars and in customers' hands. They
serve the one service, advertise the complete name, and have no VTP
characteristics at all. One app talks to both generations, because nothing in
the old protocol moved: same service UUID, same four characteristics, the config
blob still 32 bytes and telemetry still 12, same opcodes.

**`PROTO_VERSION` stays 1.** `settingsApply()` rejects a config blob whose
version byte does not match, so bumping it makes every board already out there
refuse every write the app sends. It is not a build number and must not be used
as one.

Three consequences worth having written down:

- **Match on the service UUID, not the name.** That is the path both generations
  share. The name fallback in the app now matches the older boards and not the
  newer ones, which is the opposite way round to how it reads.
- **There is no OTA.** A shipped board reaches VTP over USB-C and
  `pio run -e esp32-c3 -t upload`, or not at all.
- **A board cannot say what firmware it runs.** There is no build number in the
  old protocol and no room to add one without changing a blob the app pins byte
  offsets against. Whether the VTP service is present in the GATT table is the
  only thing that distinguishes the two.

Pairing didn't move anything either. The PIN lives in the security flags on two
characteristics, which a client never sees in the GATT table, so boards from
before it look the same and take writes from anyone. Both apps cope: the web
app's writes just succeed, and the Android app's bond request at connect goes
nowhere on an old board (it never bonds) without the app calling that a failure.

## Related

- [esp32-canbus-SN65HVD230-v2](https://github.com/MrBlahhhh/esp32-canbus-SN65HVD230-v2) — the CAN shift light this grew out of
- [esp32-autosport](https://github.com/MrBlahhhh/esp32-autosport) — the PCB generation pipeline
