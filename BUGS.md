# Bugs & TODO — R53 MINI shift light

## Desk read 2026-09-29 — firmware and web flasher - FIXED the same night, unverified on a board

Reviewed checkout `5574d94` (both ESP32-C3 and ESP32-S3-Zero targets).
Documentation only: no firmware, flasher, test or configuration fixes applied.
This repository had no bugs file; entries use the severity, unchecked finding
IDs, evidence and proposed-direction format of the R53 logger review.

### HIGH

- [x] *(fixed 2026-09-29: remote frames never decode as RPM; the capture keeps `rtr` with length 0 and VTP sends the flag with no payload; the app's own stream, whose record has no remote flag, does not carry them at all.)* **C1 — Remote-request CAN frames are decoded as RPM and forwarded as
  ordinary data frames.** `src/canbus.cpp:174-178` checks only the identifier
  and requested DLC before reading bytes 2–3. It never checks `m.rtr`.
  `ringPush` (`:80-88`) also copies DLC bytes and discards the RTR flag;
  `CapFrame` (`src/canbus.h:26-33`) has nowhere to retain it. The VTP batcher
  zeroes the output record and sets only `extended` and `len`
  (`src/vtpsvc.cpp:520-525`), so the reference encoder receives a normal data
  frame, not a remote request.

  A remote frame requests data; it does not supply RPM bytes. Receiving an
  RTR for the configured RPM ID with DLC >= 4 can refresh the RPM cache from
  bytes that are not an engine reading. Both stream formats also present
  that request as data. The VTP reference contract at `6a67515`, SPEC sections
  6.4–6.5, requires RTR to be represented with its flag and zero payload length.

  **Checked:** compiled the unchanged `canbus.cpp` with a fake TWAI receiver.
  An RTR `0x316`, requested DLC 8, with test buffer bytes 2–3 set to `00 A0`,
  produced RPM **6400**, `fresh=1`, and a legacy record with length 8 and flags
  `0x08`. The test buffer deliberately exposes the invalid read; it does not
  establish what bytes a real driver supplies for an RTR. The VTP flag loss
  was traced through the capture struct and encoder input. No claim that an
  R53 normally emits this RTR, and no on-car reproduction.

  **Fix direction, not applied:** exclude RTR from RPM decoding; preserve
  remote-frame identity through capture and VTP encoding, with zero payload.
  Decide explicitly how the legacy format handles RTR rather than inventing
  data for it. Test data and remote frames with the same ID.

### MEDIUM

- [x] *(fixed 2026-09-29: VTP's losses are counted in `ringPush` as each record is overwritten, and only when that record wanted VTP and VTP had not read it. `catchUp` now only moves a cursor.)* **V1 — VTP reports losses for frames only the legacy stream requested.**
  `src/canbus.cpp:73-77, 232-255`: both consumers share the ring, but when the
  VTP cursor falls behind, `catchUp` adds every overwritten ring position to
  its drop count before looking at the per-record `CAN_WANT_VTP` bit. Legacy
  traffic and simulated RPM records can therefore inflate VTP loss reports.
  `src/vtpsvc.cpp:536-544` puts that count and the shedding flag on the next
  real VTP batch. This contradicts SPEC sections 6.3 and 8.3 at the vendored
  reference commit: a frame not accepted by VTP must not count as its loss.

  **Reproduce:** enable the legacy all-frame stream and a narrower VTP
  subscription, then stall delivery long enough for the shared 128-slot ring
  to wrap. The Identify command below is one available source of a stall.
  The next VTP notification can claim more lost subscribed frames than were
  actually overwritten.

  **Checked:** compiled the unchanged ring implementation. Starting with reset
  cursors, 130 legacy-only records reported **2 VTP drops**, expected **0**.
  One VTP record followed by 129 legacy-only records reported **2**, expected
  **1**. These are deterministic ring probes, not a radio-throughput test.
  **Fix direction:** account for overwritten records per consumer while their
  ownership is still available; retain saturation and reset semantics.

- [x] *(fixed 2026-09-29: Identify is a timed animation inside the render; nothing blocks the loop.)* **B1 — Identify blocks CAN capture and BLE servicing for at least
  720 ms.** `src/blesvc.cpp:200-202, 296-309` executes
  `shiftlightIdentify()` inside the main-loop write drain.
  `src/shiftlight.cpp:142-151` performs six blocking 120 ms delays. During
  those delays, the loop cannot call `canPoll`, service VTP, or update normal
  telemetry (`src/main.cpp:59-75`). Moving Identify off the BLE host task
  avoided blocking that task but moved the stall onto the capture task.

  **Reproduce:** stream CAN while a verified app requests Identify. The TWAI
  queue is only 64 frames (`src/canbus.cpp:117`); at 100 frames/s, approximately
  72 frames arrive during the explicit delays alone, so even an initially
  empty queue cannot hold them all. Other bus traffic increases the loss.
  The later dequeue also gives retained frames delayed capture timestamps.

  **Checked:** compiled the unchanged LED implementation with a fake clock
  and FastLED calls; one Identify advanced the caller's clock by exactly
  **720 ms**. Actual LED output and CAN arrivals were not run on hardware;
  queue overflow follows from the stated arrival rate and queue capacity.
  **Fix direction:** advance Identify as a timed animation while the main
  loop continues polling CAN and servicing both protocols.

- [x] *(fixed 2026-09-29: a disconnect bumps a session counter instead of queueing stream-off; `blePoll` turns the stream and filter off on seeing it, and drops stream and filter commands queued under an earlier session. Config and saves from that session still apply.)* **B2 — A full phone-write queue can discard disconnect cleanup, leaving
  legacy streaming enabled for the next connection.**
  `src/blesvc.cpp:90-98` queues stream-off through the same eight-entry queue
  used for app writes. `queueWrite` (`:44-52`) drops it when full; there is
  no reserved lifecycle slot, retry or separate pending-disconnect state.
  `onConnect` (`:56-83`) does not reset the legacy stream. Previously queued
  stream-on commands can also remain ahead of the lost cleanup.

  **Reproduce:** with legacy streaming enabled, let a blocking operation such
  as Identify hold the loop, fill the eight pending-write slots, then
  disconnect before the loop drains them. After the writes drain, the board
  still captures legacy stream records. A subsequent client enabling frame
  notifications can receive records without issuing its own stream-on command.
  This breaks the documented per-connection stream lifetime; it does not
  bypass verification for configuration writes.

  **Checked:** compiled the source's unchanged `queueWrite`, `onDisconnect`
  and `applyCommand` functions against an eight-slot queue stub and the real
  CAN stream state. After filling the queue, disconnecting and draining it,
  the result was **connected=0, stream=1**, where off is **0**. This exercises
  queue behavior, not BLE callback timing on a device.
  **Fix direction:** make disconnect cleanup non-droppable and apply it in
  session order; prevent stale queued commands from reopening a retired
  connection's stream.

- [x] *(fixed 2026-09-29 in `web-flasher/flasher.js`: an unknown flash size is a refusal before erase, for hand and automatic picks alike. Live once the Pages site is republished.)* **F1 — Manual board selection permits erasing and flashing when flash
  capacity could not be detected.** `web-flasher/flasher.js:156-163` converts
  an unknown flash size into `0`. `checkChipAgainstPick` (`:486-500`) rejects
  insufficient capacity only when that value is truthy, and a manual pick
  skips the automatic board match. `startFlash` can consequently reach
  `eraseFlash` and `writeFlash` without establishing that the selected 4 MB
  layout fits. The publishing checks verify the image against its target
  partition size, not the connected chip's capacity.

  **Reproduce:** use “I know what this is”, choose the matching chip family,
  select Fresh install, and have flash-size detection return unknown. The
  chip-family check passes and the capacity check is skipped. An eventual
  verification error would occur after the destructive erase/write.

  **Checked:** executed the actual flasher functions in a Node VM, with DOM,
  downloads and serial hardware mocked. A manually selected C3 reporting
  `flashMB=0` reached **one erase and one write** and, with mocked matching
  checksums, reported success. No serial port was opened or board written.
  **Fix direction:** treat unknown capacity as an unresolved prerequisite and
  stop before erase/write; retain the chip-family and partition checks.

### LOW

- [x] *(fixed 2026-09-29: a config or defaults change that moves the RPM id or scale drops the cached reading.)* **S1 — Changing the RPM source keeps the old source's reading marked
  fresh for up to two seconds.** `src/settings.cpp:73-80` replaces the config
  without invalidating the decoded RPM cache. `src/canbus.cpp:174-178, 195-201`
  retains the old RPM and timestamp until a matching new frame arrives or the
  original two-second timeout expires. The strip and telemetry can therefore
  show a valid-looking reading for an ID that has supplied no data. Changing
  the scale also temporarily leaves a value decoded with the old scale.

  **Checked:** compiled the unchanged settings and CAN implementations with
  fake Preferences/TWAI. After a real-data `0x316` sample decoded to 6400 RPM,
  a valid `settingsApply` changing the ID to `0x500` succeeded. With no
  `0x500` frame, RPM remained **6400, fresh=1** through 1999 ms after the old
  sample, becoming **0, fresh=0** at 2000 ms.
  **Fix direction:** invalidate the cached reading when the source ID or
  decoder scale changes; unrelated colour/brightness edits should preserve it.

### Validation and limits

- `pio run -e esp32-c3 -e esp32-s3-zero`: both environments succeeded using
  the installed dependencies/build cache. No upload target was invoked.
- Temporary host harnesses outside the repository exercised the unchanged
  CAN/settings/LED sources and extracted BLE queue/lifecycle functions with
  hardware stubs. These demonstrate the logic paths, not ESP32 timing.
- The web-flasher probe ran the source functions with mocked hardware. Its
  `Uint8Array` input and chip-feature calls were also checked against the
  pinned esptool-js 0.7.0 implementation; those API uses were not filed as bugs.
- `test/` contains only the PlatformIO placeholder README; there was no
  project test suite to report as passing. No tests were added or changed.
- VTP checks used the local reference repository at `6a67515`, matching
  `lib/vtp1/README.md`. The on-device conformance harness was not run.
- No device, car, NVS, firmware release, Pages site or source code was changed.
  The pre-existing `.vscode/extensions.json` edit was left alone.
