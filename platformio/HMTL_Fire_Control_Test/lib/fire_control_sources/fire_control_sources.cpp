/*
 * Pulls real fire control firmware into every test binary.
 *
 * test_support.cpp provides all global instances and stubs for hardware
 * functions (sendHMTL*, mode functions, LCD, MPR121, digitalRead, etc.).
 *
 * Fire_Control_Connect.cpp is NOT included here — sendHMTL* functions are
 * stubbed in test_support.cpp so tests can capture and assert on them.
 * modes.cpp is NOT included — those functions are stubbed in test_support.cpp.
 */

#include "../../stubs/test_support.cpp"
#include "../../../../HMTL_Fire_Control_Wickerman/Fire_Control_Sensors.cpp"
