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

/*
 * The expander's I2C address is set by the A0/A1/A2 jumpers, so it is a
 * property of the BOARD, not of the firmware.  Overridable rather than
 * hardcoded: a differently-jumpered board otherwise needs a source edit, and
 * getting it wrong fails in a way that is easy to misread -- every read NACKs,
 * which is indistinguishable from a dead bus.  0x27 is the LCD backpack, so
 * 0x26 is the default here.
 */
#ifndef FC_MCP_ADDR
  #define FC_MCP_ADDR  0x26
#endif
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
