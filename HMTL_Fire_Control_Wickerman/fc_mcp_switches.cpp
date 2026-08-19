/*******************************************************************************
 * Author: Adam Phelps
 * License: Create Commons Attribution-Non-Commercial
 * Copyright: 2026
 *
 * Register-level Wire transactions rather than a driver library: every call
 * surfaces failure, because a NACKed or short read must report the switches
 * open rather than whatever the chip last said.  (Bus recovery — clocking a
 * stuck slave free — is not implemented here yet; a persistent failure reads
 * as all-open, which cannot arm.)
 ******************************************************************************/

#include <Arduino.h>
#include <Wire.h>

#include "fc_mcp_switches.h"

#define FC_MCP_ADDR  0x26     /* A0 jumpered; 0x27 is the LCD backpack */
#define MCP_IODIRA   0x00
#define MCP_GPPUA    0x0C
#define MCP_GPIOA    0x12

static uint32_t error_count = 0;

static bool write_reg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(FC_MCP_ADDR);
  Wire.write(reg);
  Wire.write(val);
  if (Wire.endTransmission() != 0) {
    error_count++;
    return false;
  }
  return true;
}

bool fc_mcp_switches_init() {
  if (!write_reg(MCP_IODIRA, 0xFF)) return false;  /* all PA inputs (POR default, made explicit) */
  if (!write_reg(MCP_GPPUA, 0x0F)) return false;   /* pull-ups on the four switch bits */
  return true;
}

bool fc_mcp_switches_read(uint8_t *bits) {
  Wire.beginTransmission(FC_MCP_ADDR);
  Wire.write(MCP_GPIOA);
  if (Wire.endTransmission() != 0) {
    error_count++;
    return false;
  }
  if (Wire.requestFrom((uint8_t)FC_MCP_ADDR, (uint8_t)1) != 1) {
    error_count++;
    return false;
  }
  int value = Wire.read();
  if (value < 0) {
    error_count++;
    return false;
  }
  *bits = (uint8_t)value;
  return true;
}

uint32_t fc_mcp_switch_errors() {
  return error_count;
}
