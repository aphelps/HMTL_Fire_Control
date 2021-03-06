/*******************************************************************************
 * Author: Adam Phelps
 * License: Create Commons Attribution-Non-Commercial
 * Copyright: 2014
 *
 * Code for communicating with remote modules
 ******************************************************************************/

#include <Arduino.h>
#include "EEPROM.h"
#include <RS485_non_blocking.h>
#include <SoftwareSerial.h>
#include "SPI.h"
#include "Wire.h"
#include "FastLED.h"


#define DEBUG_LEVEL DEBUG_MID
#include "Debug.h"

#include "GeneralUtils.h"
#include "EEPromUtils.h"
#include "HMTLTypes.h"
#include <HMTLProtocol.h>
#include "HMTLMessaging.h"
#include "HMTLPrograms.h"

#include "PixelUtil.h"
#include "RS485Utils.h"
#include "XBeeSocket.h"
#include "MPR121.h"

#include "HMTL_Fire_Control.h"


byte rs485_data_buffer[RS485_BUFFER_TOTAL(SEND_BUFFER_SIZE)];

byte *send_buffer; // Pointer to use for start of send data

void initialize_connect() {
  /* Setup the RS485 connection */
  if (!rs485.initialized()) {
    DEBUG_ERR("RS485 was not initialized, check config");
    DEBUG_ERR_STATE(DEBUG_ERR_UNINIT);
  }

  rs485.setup();
  send_buffer = rs485.initBuffer(rs485_data_buffer, SEND_BUFFER_SIZE);

  DEBUG2_VALUE("Initialized RS485. address=", my_address);
  DEBUG2_VALUELN(" bufsize=", SEND_BUFFER_SIZE);
}

void sendHMTLValue(uint16_t address, uint8_t output, int value) {
  hmtl_send_value(&rs485, send_buffer, SEND_BUFFER_SIZE,
		  address, output, value);
}

void sendHMTLTimedChange(uint16_t address, uint8_t output,
			 uint32_t change_period,
			 uint32_t start_color,
			 uint32_t stop_color) {
  hmtl_send_timed_change(&rs485, send_buffer, SEND_BUFFER_SIZE,
			 address, output,
			 change_period,
			 start_color,
			 stop_color);
}
