// Minimal Arduino stand-in for compiling firmware sources on a PC (unit tests only).
#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>
#include <algorithm>
#ifndef PI
#define PI 3.14159265358979323846f
#endif
namespace fake { inline uint32_t us = 0; }
inline uint32_t micros() { return fake::us; }
inline uint32_t millis() { return fake::us / 1000; }
