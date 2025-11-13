#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "v5.h"
#include "v5_vcs.h"

using namespace vex;

/**
 * Convenience helper that mirrors the classic VEX competition macros.
 * Use `waitUntil(condition);` to poll a condition every 5 ms.
 */
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, vex::msec);                                                        \
  } while (!(condition))

/**
 * Convenience helper for simple counted loops in examples.
 */
#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)

// Project specific headers
#include "robot-config.h"
#include "utils.h"
#include "pid.h"
#include "motor-control.h"