#pragma once
#include <stddef.h>
#include "proto.h"

// Live config plus its NVS backing. The live copy is what the strip renders
// from; NVS is only touched on an explicit SL_CMD_SAVE, so a phone can drag a
// threshold slider around at speed without writing flash on every frame.
//
// cfg is read by the render and the CAN decode on the loop task, so every
// function here that writes it must be called from loop(), never from a BLE
// callback. The simulate flag lives in cfg but is never saved.

void settingsBegin();          // load from NVS, or install defaults on first boot
void settingsDefaults();       // reset the live copy only
bool settingsApply(const uint8_t* data, size_t len);  // validate then adopt
bool settingsSave();           // commit live copy, minus the simulate flag, to NVS
bool settingsUnsaved();        // live copy differs from what NVS holds, simulate aside

extern ConfigBlob cfg;
