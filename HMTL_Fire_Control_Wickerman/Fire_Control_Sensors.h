/*******************************************************************************
 * Author: Adam Phelps
 * License: Create Commons Attribution-Non-Commercial
 * Copyright: 2016
 ******************************************************************************/

#ifndef FIRE_CONTROL_SENSORS_H
#define FIRE_CONTROL_SENSORS_H

#include "Arduino.h"

#define DISPLAY_CAP_SENSORS       0
#define DISPLAY_ADJUST_BPM1_1     1
#define DISPLAY_ADJUST_BPM1_2     2
#define DISPLAY_ADJUST_BPM2_1     3
#define DISPLAY_ADJUST_BPM2_2     4
#define DISPLAY_ADJUST_BPM3_1     5
#define DISPLAY_ADJUST_BPM3_2     6
#define DISPLAY_ADJUST_BPM4_1     7
#define DISPLAY_ADJUST_BPM4_2     8
#define DISPLAY_ADJUST_BRIGHTNESS 9
#define DISPLAY_LED_MODE          10
#define DISPLAY_ADDRESS_MODE      11
#define DISPLAY_MAX              (11 + 1)

extern uint8_t display_mode;
#define NUM_DISPLAY_MODES DISPLAY_MAX

#define LED_MODE_ON    0
#define LED_MODE_BLINK 1
#define LED_MODE_MAX (1 + 1)
extern uint8_t led_mode;
extern uint8_t led_mode_value;

byte sensor_to_led(byte sensor);

/*
 * Read-only views of the physical switch state for status reporting.  The
 * arming predicate is the same condition handle_sensors() gates ignition on;
 * both read the interlock-qualified states, not raw pin levels.
 */
#define FC_NUM_SWITCHES 4
bool fc_is_armed();
bool fc_switch_state(uint8_t sw);
void fc_reset_switch_interlock();

#endif
