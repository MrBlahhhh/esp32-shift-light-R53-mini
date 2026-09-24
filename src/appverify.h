#pragma once
#include <stddef.h>
#include <stdint.h>

// Per-connection app verification: the challenge, the response check, and which
// connections may change settings. The protocol is in proto.h.
//
// Called only from NimBLE callbacks (connect, disconnect, and the auth, config
// and command characteristics), which all run on the NimBLE host task, so the
// state needs no lock. loop() never asks: only verified writes reach its queue.
//
// Test vector for the response, also asserted by both apps' unit tests:
//   key       00 01 02 .. 1f (32 bytes)
//   challenge a0 a1 a2 .. af (16 bytes)
//   response  8af7a67b26ef306f603cf56693c0988b
// (the first 16 bytes of HMAC-SHA256(key, "SLv1" || challenge)).

void appVerifyOnConnect(uint16_t connHandle);
void appVerifyOnDisconnect(uint16_t connHandle);

// What a read of the auth characteristic returns to this connection: its
// challenge, or one SL_AUTH_VERIFIED byte once it has sent the right response.
// `out` holds at least SL_AUTH_CHALLENGE_LEN bytes. Returns the length, 0 for
// an unknown handle.
size_t appVerifyReadValue(uint16_t connHandle, uint8_t* out);

// A write to the auth characteristic.
void appVerifyOnResponse(uint16_t connHandle, const uint8_t* response, size_t len);

bool appVerified(uint16_t connHandle);
