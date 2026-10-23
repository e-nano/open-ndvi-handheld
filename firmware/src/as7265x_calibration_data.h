#pragma once

#include <Arduino.h>

static constexpr size_t AS7265X_CHANNEL_COUNT = 18;
// Retained solely to read existing schema-5 blobs without rewriting the partition.
static constexpr size_t AS7265X_CAL_POINT_COUNT = 6;
