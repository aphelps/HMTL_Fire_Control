/*
 * Pulls real fire control firmware into every test binary.
 *
 * test_support.cpp provides all global instances and stubs for hardware
 * functions (sendHMTL*, mode functions, LCD, MPR121, digitalRead, etc.).
 *
 * Fire_Control_Connect.cpp is NOT included here — sendHMTL* functions are
 * stubbed in test_support.cpp so tests can capture and assert on them.
 * modes.cpp is NOT included — those functions are stubbed in test_support.cpp.
 *
 * fc_mcp_switches.cpp is included ONLY under FC_SWITCHES_MCP23017, because that
 * is the only build in which Fire_Control_Sensors.cpp references it. It must
 * stay conditional: test_mcp_switches.cpp #includes the same .cpp directly into
 * its own translation unit, so pulling it in unconditionally would give every
 * binary two copies. env:native_mcp is filtered to test_switch_health for the
 * same reason.
 */

#include "../../stubs/test_support.cpp"
#ifdef FC_SWITCHES_MCP23017
#include "../../../../HMTL_Fire_Control_Wickerman/fc_mcp_switches.cpp"
#endif
#include "../../../../HMTL_Fire_Control_Wickerman/Fire_Control_Sensors.cpp"
