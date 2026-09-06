#pragma once
#include <stdint.h>
#include <stddef.h>
#include "proto.h"

// TWAI in listen-only mode. The R53's bus is a running car's bus and this board
// has no business acknowledging frames on it, so the controller is configured
// NO_ACK: it hears everything and never drives the dominant bit.

void canBegin();
void canPoll();          // drain the rx queue; call every loop
uint16_t canRpm();       // decoded RPM, 0 once stale
bool canRpmFresh();
bool canUp();
uint16_t canFramesPerSec();
uint16_t canRxMissed();

// One frame as the ring holds it, which is not what either protocol puts on the
// wire. Two of them read this ring — the shift light's own frame stream and the
// VTP/1 one — and they disagree about almost everything except the bytes: the
// old protocol wants a millisecond stamp and a packed dlc/flags byte, VTP wants
// microseconds and the format bit kept separate from the identifier. Storing the
// capture rather than either wire format is what lets both be served from one
// copy of each frame.
struct CapFrame {
  uint64_t tsUs;     // device clock at capture — see the note in canPoll()
  uint32_t id;       // arbitration id only; no format or RTR bits packed in
  uint8_t  ext;      // 29-bit identifier
  uint8_t  len;      // 0..8
  uint8_t  want;     // CAN_WANT_* — who asked for this frame
  uint8_t  data[8];
};

#define CAN_WANT_LEGACY 0x01
#define CAN_WANT_VTP    0x02

// Ring of captured frames for the phone. Two independent read cursors, because
// the two protocols are subscribed to different things and neither may consume
// the other's frames. Each returns how many records it copied.
size_t canDrainFrames(CanFrameRec* out, size_t max);   // …0004 stream cursor
void canSetStream(uint8_t mode);
void canSetFilter(const uint32_t* ids, size_t count);

// VTP cursor. Peek/pop rather than a drain, because a VTP batch is bounded by
// bytes and by the 655.35 ms dt window rather than by a record count, and only
// the caller knows how much room is left in the notification it is building.
bool     canVtpPeek(CapFrame* out);   // false once the cursor is caught up
void     canVtpPop();
uint16_t canVtpTakeDropped();         // accepted-then-discarded since last call, saturating
void     canVtpReset();               // drop the backlog and the drop count

// Synthesise the RPM frame while simulating, so the phone's bus view and its
// RPM read-out agree with the strip instead of showing a dead bus.
void canInjectSimulated(uint16_t rpm);

#define RPM_STALE_MS 2000
