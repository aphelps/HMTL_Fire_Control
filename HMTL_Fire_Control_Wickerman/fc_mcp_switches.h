/*******************************************************************************
 * Author: Adam Phelps
 * License: Create Commons Attribution-Non-Commercial
 * Copyright: 2026
 *
 * Ignition-switch input via an MCP23017 expander (PA0-PA3, optocoupler
 * collectors, active-low with the chip's internal pull-ups).
 ******************************************************************************/

#ifndef FC_MCP_SWITCHES_H
#define FC_MCP_SWITCHES_H

#include <stdint.h>

/* Configure PA as inputs with pull-ups on the switch bits; false if the
 * expander did not ack. */
bool fc_mcp_switches_init();

/* One-byte atomic read of all switch bits into *bits (1 = open, 0 = closed).
 * false on any bus failure — the caller must then treat every switch as OPEN,
 * never last-known-state. */
bool fc_mcp_switches_read(uint8_t *bits);

/* Cumulative failed transactions since boot */
uint32_t fc_mcp_switch_errors();

#endif // FC_MCP_SWITCHES_H
