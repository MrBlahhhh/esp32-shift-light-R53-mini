#pragma once
#include <stdint.h>

void shiftlightBegin();
void shiftlightRender(uint16_t rpm);   // compute + push pixels; call at LED_HZ
void shiftlightIdentify();             // brief flash, to tell two boards apart
uint8_t shiftlightLevel();             // lit LEDs (or pairs), for telemetry

void statusBegin();
void statusUpdate(bool canOk, bool bleConnected);

uint16_t simulatedRpm();               // 1000..9000 sweep on a 10 s cycle

#define LED_HZ 20
