#pragma once
#include <stdint.h>

// VTP/1 — the second BLE protocol this board speaks, alongside the shift
// light's own service in proto.h. It carries the CAN bus and nothing else: no
// configuration, no telemetry, no thresholds. Everything that makes this board
// a shift light stays on the old service, where every app already shipped
// expects to find it.
//
// The contract is https://github.com/Lapsmith-app/VTP, SPEC.md, and the wire
// bytes come from the reference encoder in lib/vtp1 rather than from anything
// written here. Section numbers in vtpsvc.cpp refer to that spec.

class NimBLEServer;

// Builds the service on the server blesvc.cpp already created. Called from
// bleBegin(), before advertising starts.
void vtpBegin(NimBLEServer* server);

// Sends whatever the CAN batcher has, and works the control queue. Called every
// loop, from blePoll().
void vtpPoll();

// Link events, forwarded by blesvc.cpp because that is where the server
// callbacks live. A VTP subscription table lives and dies with one connection
// (§9.1) and so does the notification sequence (§8.2).
void vtpOnConnect();
void vtpOnDisconnect();
void vtpSetMtu(uint16_t mtu);

// Called by canbus.cpp for every frame off the bus, exactly once, at capture.
// True when a subscription asked for this frame and its mode selected it —
// which is what §8.3 means by "accepted", and therefore what makes a later
// discard something the device has to report rather than hide.
//
// This advances per-identifier scheduling state, so it is not a predicate to
// be called twice or skipped.
bool vtpAdmit(uint32_t id, bool extended, uint64_t tsUs);
