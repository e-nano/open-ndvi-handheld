#pragma once

#include <Arduino.h>

bool portalBegin(const char *deviceName, const char *firmwareVersion);
void portalEnd();
void portalLoop();
bool portalActive();
const String &portalSsid();
const String &portalPassword();
const String &portalMessage();
