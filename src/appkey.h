#pragma once
#include <stdint.h>

// The key the apps prove they hold before this board takes a config or command
// write from them (see "App verification" in proto.h and the README). This
// board, the twins board and every one of their apps carry the same 32 bytes;
// a board and an app with different keys can watch each other but never change
// anything. As hex, which is how the apps spell it:
// 4d3b5b150b25196e4f30983221788b003d3cbdd8931669d547f841e71492543c
//
// Check value: with this key and the challenge 00 01 02 .. 0f, the right answer
// is a0a7568feacefff5e0908131bf23cca3. The apps' unit tests assert the same
// value, so a key changed in one place and not the others fails a test.
//
// It ships inside every app build, so anyone who digs it out can change
// settings. It keeps out random phones and generic BLE apps, nothing more.
static const uint8_t SHIFTLIGHT_APP_KEY[32] = {
  0x4d, 0x3b, 0x5b, 0x15, 0x0b, 0x25, 0x19, 0x6e,
  0x4f, 0x30, 0x98, 0x32, 0x21, 0x78, 0x8b, 0x00,
  0x3d, 0x3c, 0xbd, 0xd8, 0x93, 0x16, 0x69, 0xd5,
  0x47, 0xf8, 0x41, 0xe7, 0x14, 0x92, 0x54, 0x3c,
};
