// Extended Arduino stub for fire control native tests.
// Adds HIGH/LOW/INPUT/OUTPUT constants that the base HMTL_Test stub omits.
// Must come before the base stub to avoid pragma-once short-circuit.
#pragma once

// Pull in the base stub (absolute path so the pragma once resolves correctly)
#include "/Users/amp/Dropbox/Arduino/HMTL_Ecosystem/HMTL/platformio/HMTL_Test/stubs/Arduino.h"

#ifdef __cplusplus

#ifndef HIGH
  #define HIGH 1
#endif
#ifndef LOW
  #define LOW  0
#endif
#ifndef INPUT
  #define INPUT       0
  #define OUTPUT      1
  #define INPUT_PULLUP 2
#endif

#endif /* __cplusplus */
