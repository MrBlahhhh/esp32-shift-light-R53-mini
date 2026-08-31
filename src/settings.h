#pragma once
#include <stddef.h>
#include "proto.h"

// Live config plus its NVS backing. The live copy is what the strip renders
// from; NVS is only touched on an explicit SL_CMD_SAVE, so a phone can drag a
// threshold slider around at speed without writing flash on every frame.

void settingsBegin();          // load from NVS, or install defaults on first boot
void settingsDefaults();       // reset the live copy only
bool settingsApply(const uint8_t* data, size_t len);  // validate then adopt
bool settingsSave();           // commit live copy to NVS
bool settingsUnsaved();        // live copy differs from what NVS holds

extern ConfigBlob cfg;
