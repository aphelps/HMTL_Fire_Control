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

/*
 * True when the last sensor_switches() call actually read the switches.
 *
 * Without this a remote consumer cannot tell "all switches open" from "the bus
 * is dead": the fail-safe reports every switch OPEN on a failed read, which is
 * indistinguishable on the wire from a genuinely disarmed panel.  The
 * cumulative error counter is not a substitute -- a monotonic count carries no
 * information on a single poll, only across two.
 */
bool fc_switches_read_ok();
void fc_reset_switch_interlock();

/*
 * RAW switch state -- the debounced physical reading, WITHOUT the arming
 * interlock applied.
 *
 * fc_switch_state() reports the interlock-qualified state (raw && seen-open),
 * which is the right thing for ignition: a switch held closed since boot must
 * not count as armed.  It is the WRONG thing for anything asking "is this
 * switch physically active?", because a stuck-closed switch reports false
 * there.  An OTA guard built on fc_switch_state() would therefore treat a
 * switch closed across a reboot as inactive and permit the upload.  Anything
 * that must refuse on activity reads this instead.
 *
 * Raw is a superset of qualified (qualified == raw && seen_open), so "no raw
 * switch active" implies "not armed" -- the guard never needs both.
 */
bool fc_switch_raw(uint8_t sw);
bool fc_any_switch_raw_active();

/*
 * Positive evidence that the switch bank was actually read.
 *
 * False until sensor_switches() has completed at least one successful read,
 * and false again if a read ever fails.  Callers that must FAIL CLOSED need
 * this: the ignition fail-safe drives unreadable switches to OPEN (correct --
 * unreadable means not-armed), which any "is anything active?" test would read
 * as "nothing active".  Without this flag such a test permits on fault.
 *
 * On the GPIO switch path a digitalRead cannot fail, so this becomes "we have
 * sampled the switches at least once" -- still the property a cross-core
 * caller needs, since it distinguishes a running core 1 from one that has not
 * reached its first sensor_switches() (or has stopped reaching it).  When the
 * MCP23017 expander path lands it only has to make read_switch_bank() return
 * false on an I2C error; the meaning strengthens with no caller change and no
 * change to the failure handling, which is already written.
 *
 * The invariant that makes this safe is that sensor_switches() CLEARS the flag
 * on entry and sets it only after a complete read.  A flag that is merely set
 * at the end of a successful read is stale-true for the whole of a failed one.
 *
 * THIS ACCESSOR IS THE SINGLE SOURCE OF TRUTH FOR SWITCH-READ HEALTH.  Every
 * consumer -- the OTA guard, the cross-core status snapshot, the /status
 * response, anything added later -- must read it rather than keep its own
 * notion of health or, worse, infer health from the switch VALUES.  The values
 * cannot carry that information: a failed read reports every switch open, which
 * is indistinguishable from a genuinely idle panel.  That is the whole reason
 * this flag exists, and a second flag maintained alongside it would be worse
 * than none, because the two can disagree and nothing would say which is right.
 */
bool fc_switches_read_ok();

/*
 * How long the switch bank must be UNREADABLE before that counts as a fault.
 *
 * Public because it is part of this module's contract: the OTA guard's
 * behaviour depends on it, and a test carrying its own copy of the value could
 * drift from the implementation silently.  Rationale in Fire_Control_Sensors.cpp.
 */
#define SWITCH_READ_FAIL_DEBOUNCE_MS 1000

/*
 * Drive outputs to their safe state.  Every one of these CANCELS the remote
 * program before setting the output off: the igniter and pilot are driven with
 * sendBurst(), i.e. a 30-second TIMED_CHANGE program running on the remote
 * module, and a bare value-0 does not stop it -- the program simply re-asserts
 * the output for the remainder of its duration.  Cancel-then-off is the only
 * sequence that actually closes a remote output.
 *
 * Safe to call unconditionally and repeatedly; they are plain sends with no
 * edge-trigger or state of their own.  They must be called from the core that
 * owns RS485 (core 1 / loop()), never from the API task -- see
 * HMTL_Fire_Control_API.h.
 */
void fc_poofers_safe();
void fc_igniter_safe();
void fc_pilot_safe();
void fc_all_outputs_safe();

#endif
