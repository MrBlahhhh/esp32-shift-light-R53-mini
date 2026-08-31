#pragma once
#include <stdint.h>

void bleBegin();
// `rpm` is the value actually driving the strip — simulated or off the bus.
// Telemetry must report that, not canRpm(), or the app shows 0 while the
// strip sweeps.
void blePoll(uint16_t rpm);
bool bleConnected();
void blePublishConfig(); // refresh the config characteristic after a local change

#define TELEMETRY_HZ 10
