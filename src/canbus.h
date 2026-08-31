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

// Ring of captured frames for the phone. Returns how many records were copied.
size_t canDrainFrames(CanFrameRec* out, size_t max);
void canSetStream(uint8_t mode);
void canSetFilter(const uint32_t* ids, size_t count);

// Synthesise the RPM frame while simulating, so the phone's bus view and its
// RPM read-out agree with the strip instead of showing a dead bus.
void canInjectSimulated(uint16_t rpm);

#define RPM_STALE_MS 2000
