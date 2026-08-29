/*******************************************************************************
 * Author: Adam Phelps
 * License: Create Commons Attribution-Non-Commercial
 * Copyright: 2014
 ******************************************************************************/

#ifdef DEBUG_LEVEL_SENSORS
  #define DEBUG_LEVEL DEBUG_LEVEL_SENSORS
#endif
#ifndef DEBUG_LEVEL
  #define DEBUG_LEVEL DEBUG_HIGH
#endif
#include <Debug.h>

#include <Arduino.h>
#include <Wire.h>
#include "MPR121.h"

#include "HMTLTypes.h"
#include "HMTLMessaging.h"
#include "HMTLPoofer.h"

#include "HMTL_Fire_Control.h"
#include "modes.h"
#include "Fire_Control_Sensors.h"
#ifdef FC_SWITCHES_MCP23017
#include "fc_mcp_switches.h"
#endif

bool data_changed = true;

/* Poofer addresses to allow for overrides */
uint16_t poofer1_address = POOFER1_ADDRESS;
uint16_t poofer2_address = POOFER2_ADDRESS;
uint16_t lights_address = LIGHTS_ADDRESS;

/******* Switches *************************************************************/

#define NUM_SWITCHES FC_NUM_SWITCHES
bool switch_states[NUM_SWITCHES] = { false, false, false, false };
bool switch_changed[NUM_SWITCHES] = { false, false, false, false };

/*
 * Health of the most recent switch read.
 *
 * Set false as the FIRST statement of sensor_switches() and true only once a
 * read has actually completed, so every early return leaves it false by
 * construction rather than by remembering to clear it.  An earlier version of
 * this flag was set at the top and cleared on the failure path, which left it
 * stale-TRUE on the I2C early return -- the exact bug this ordering prevents.
 * If a merge ever puts the `!mcp_ok` early return above the false-assignment,
 * that bug is back.
 */

bool fc_is_armed() {
  return switch_states[POOFER_ENABLE_SWITCH] && switch_states[POOFER_PILOT_SWITCH];
}

bool fc_switch_state(uint8_t sw) {
  return (sw < NUM_SWITCHES) ? switch_states[sw] : false;
}

/*
 * Raw (pre-interlock) switch state, and whether the bank has actually been
 * read.  See the commentary on fc_switch_raw()/fc_switches_read_ok() in
 * Fire_Control_Sensors.h for why a caller that must fail closed needs these
 * rather than switch_states[].
 */
static bool switch_raw[NUM_SWITCHES] = { false, false, false, false };
static bool switches_read_ok = false;

#ifdef FC_SWITCHES_TEST_FAULT_INJECTION
/*
 * Native-test hook, compiled out of every firmware build.  See
 * read_switch_bank() for why it is here rather than in a test double.
 */
static bool s_switch_bank_read_fails = false;
extern "C" void fc_test_set_switch_bank_read_fails(bool fails) {
  s_switch_bank_read_fails = fails;
}
#endif

bool fc_switch_raw(uint8_t sw) {
  return (sw < NUM_SWITCHES) ? switch_raw[sw] : false;
}

bool fc_any_switch_raw_active() {
  for (uint8_t i = 0; i < NUM_SWITCHES; i++) {
    if (switch_raw[i]) return true;
  }
  return false;
}

bool fc_switches_read_ok() {
  return switches_read_ok;
}


/*
 * Arming interlock: a switch only counts as CLOSED after it has been observed
 * OPEN since boot.  Anything that holds a switch line low from power-on (a
 * shorted harness, a miswired pin, a radio sharing the pin) would otherwise
 * make the controller boot believing the ignition switches are closed and arm
 * with no operator action.  The cost is one open/close cycle for a switch left
 * on across a reboot -- the correct arming semantic anyway: outputs default
 * safe until an operator does something.
 */
static bool switch_seen_open[NUM_SWITCHES] = { false, false, false, false };

/*
 * The latch is TIME-QUALIFIED, not single-sample: a driven or noisy signal
 * sharing the switch line would otherwise satisfy "seen open" with one pulse
 * and then read as an armed, closed switch.  A switch must read open continuously for
 * SWITCH_OPEN_LATCH_MS before closed counts; operators flip switches on human
 * timescales, so a 1s qualification is invisible to them.
 */
#define SWITCH_OPEN_LATCH_MS 1000

/*
 * Debounce for an UNREADABLE switch bank -- see SWITCH_READ_FAIL_DEBOUNCE_MS
 * in the header.
 *
 * Measured on the bench 2026-08-21, on the first build to put the MCP23017 on
 * this hardware: the bus glitches at random -- roughly one failed transaction
 * in five hundred, recovering on the very next read.  Tripping the fail-safe on
 * a single failure forced every switch OPEN several times a minute during
 * normal operation, and could refuse an OTA at random, because the guard reads
 * switches_read_ok.  (The GPIO path here cannot fail, so this only bites once
 * the expander branch lands -- but the handling belongs with the failure seam,
 * which is here.)
 *
 * Milliseconds do not matter for switches: operators flip them on human
 * timescales, and SWITCH_OPEN_LATCH_MS already spends a full second qualifying
 * one.  So a sub-second read outage is not evidence of anything.
 *
 * The cost, stated rather than buried: for up to a second after the bank dies,
 * the switch values are held at their last good reading, so a disarm inside
 * that window is noticed up to a second late -- the same order as the
 * qualification delay that already exists.
 */
static bool          switch_read_failing = false;
static unsigned long switch_read_fail_since = 0;
static unsigned long switch_open_since[NUM_SWITCHES] = { 0, 0, 0, 0 };
#ifndef FC_SWITCHES_MCP23017
const uint8_t switch_pins[NUM_SWITCHES] = {
  SWITCH_PIN_1, SWITCH_PIN_2, SWITCH_PIN_3, SWITCH_PIN_4 };
#endif

/* Force every switch back through the seen-open qualification */
void fc_reset_switch_interlock() {
  for (uint8_t i = 0; i < NUM_SWITCHES; i++) {
    switch_seen_open[i] = false;
    switch_open_since[i] = 0;
  }
}

void initialize_switches(void) {
#ifdef FC_SWITCHES_MCP23017
  /* A failed init is survivable: every read will fail too, and a failed read
   * reports all switches open, so the system runs but cannot arm. */
  if (!fc_mcp_switches_init()) {
    DEBUG_ERR("MCP23017 init failed - switches read as open");
  }
#else
  for (uint8_t i = 0; i < NUM_SWITCHES; i++) {
    /*
     * INPUT_PULLUP, not INPUT: switches are active-low (closed = LOW), so the
     * pulled-up open state is the safe one, and a floating pin reads a steady
     * OPEN instead of oscillating -- without it, one noise "open" would let
     * the seen-open interlock count noise as real closes.  The interlock
     * handles stuck-low lines; the pullup handles floating ones.
     */
    pinMode(switch_pins[i], INPUT_PULLUP);
  }
#endif

  calculate_pulse();
}

/*
 * Sample the physical switch bank into raw[], active-high (true == closed).
 *
 * Returns false if the bank COULD NOT BE READ.  The direct-GPIO path here
 * cannot fail; the MCP23017/I2C path on the expander branch can, and this is
 * the single seam it slots into, so the failure handling in sensor_switches()
 * below is written once and the platform difference stays confined to this
 * function.
 */
static bool read_switch_bank(bool *raw) {
#ifdef FC_SWITCHES_TEST_FAULT_INJECTION
  /*
   * Test-only, and never defined by any firmware environment (see
   * platformio/HMTL_Fire_Control_Test/platformio.ini).  It exists because the
   * single most dangerous behaviour in this file -- what a FAILED switch read
   * does to fc_switches_read_ok(), and therefore to the OTA guard -- has no
   * failure mode to stage on the GPIO path.  Without a way to inject one, the
   * permit-on-fault regression could only be caught on hardware that does not
   * exist on this branch yet.
   */
  if (s_switch_bank_read_fails) return false;
#endif
#ifdef FC_SWITCHES_MCP23017
  /*
   * The expander read is the failure this seam exists for.  A bus error returns
   * false here and the caller does the rest -- reports every switch OPEN, clears
   * switch_raw[] so no refusal can quote a stale sample, and leaves
   * switches_read_ok false so the OTA guard refuses.
   */
  uint8_t mcp_bits = 0xFF;
  if (!fc_mcp_switches_read(&mcp_bits)) {
    return false;
  }
  for (uint8_t i = 0; i < NUM_SWITCHES; i++) {
    raw[i] = ((mcp_bits & (1 << i)) == 0);  /* opto conducts -> line low -> closed */
  }
#else
  for (uint8_t i = 0; i < NUM_SWITCHES; i++) {
    raw[i] = (digitalRead(switch_pins[i]) == LOW);
  }
#endif
  return true;
}

void sensor_switches(void) {
  /*
   * Cleared on ENTRY, not merely set at the end.
   *
   * This flag is the OTA guard's positive evidence of health, and the guard's
   * fail-safe direction is the opposite of ignition's: a failed read forces
   * every switch to report OPEN, which an "is anything active?" test reads as
   * "nothing active" and would PERMIT on.  Only a completed read may leave
   * this true.  Setting it at the end alone is not enough -- any early return
   * from a failure path would then leave a STALE TRUE from the last good read,
   * which is precisely the silent permit the flag exists to refuse.
   */
  switches_read_ok = false;

  bool raw[NUM_SWITCHES];
  if (!read_switch_bank(raw)) {
    unsigned long now = millis();
    if (!switch_read_failing) {
      switch_read_failing = true;
      switch_read_fail_since = now;
    }
    /*
     * The clock alone already requires MULTIPLE failures, which is why there
     * is no separate count.
     *
     * switch_read_fail_since is stamped on the FIRST failure of a run, so at
     * that moment the elapsed time is zero.  Crossing the threshold therefore
     * always takes at least one LATER failed read in the same unbroken run: a
     * lone glitch can never trip it, and any trip means the bank was unreadable
     * across a span of at least SWITCH_READ_FAIL_DEBOUNCE_MS.
     *
     * An explicit `count >= 2` was tried and removed -- it is provably inert
     * for the reason above, and a condition that cannot change an outcome is
     * how an inert guard gets mistaken for a working one.  A count only starts
     * to matter at 3 or more, which would begin DELAYING a genuine fault on a
     * slow loop: the wrong direction for a fail-safe.
     */
    if ((unsigned long)(now - switch_read_fail_since) < SWITCH_READ_FAIL_DEBOUNCE_MS) {
      /*
       * Too brief to mean anything.  This call simply observed nothing:
       * switch_states[], switch_raw[] and the seen-open timers are all left as
       * the last good read left them.
       *
       * switch_changed[] IS cleared, because no edge was observed either and a
       * consumer must not re-act on the previous call's edge.
       *
       * switches_read_ok is restored to true: its contract is that the switch
       * data can be trusted, and data under a second old can be, for a signal
       * that moves on human timescales.  Left false, every glitch would refuse
       * an OTA at random -- the noise this debounce exists to remove.
       */
      for (uint8_t i = 0; i < NUM_SWITCHES; i++) {
        switch_changed[i] = false;
      }
      switches_read_ok = true;
      return;
    }
    /*
     * Sustained: the bank genuinely cannot be read.  Report every switch OPEN
     * and never
     * last-known-state -- a dead bus observes nothing, so it must neither arm
     * nor accrue open-time toward arming.  switch_raw[] is cleared along with
     * switch_states[]: it is what the OTA guard reads, and leaving the last
     * good sample there would report a stale "all clear" from a bank nobody
     * can see.  switches_read_ok stays false, which is what actually refuses
     * the upload; the cleared raw values only make sure a refusal reason can
     * never quote values that came from a failed read.
     */
    for (uint8_t i = 0; i < NUM_SWITCHES; i++) {
      switch_raw[i] = false;
      switch_open_since[i] = 0;
      if (switch_states[i]) {
        switch_changed[i] = true;
        data_changed = true;
        switch_states[i] = false;
      } else {
        switch_changed[i] = false;
      }
    }
    /* One line, not one per switch: on AVR every distinct literal is flash
     * this firmware does not have (both AVR envs sit above 98%). */
    DEBUG1_PRINTLN("Switch read FAILED; all open");
    return;
  }

  switch_read_failing = false;

  for (uint8_t i = 0; i < NUM_SWITCHES; i++) {
    /* Record the physical reading before the arming interlock is applied --
     * fc_switch_raw()'s contract. */
    switch_raw[i] = raw[i];
    if (!raw[i]) {
      if (!switch_seen_open[i]) {
        unsigned long now = millis();
        if (switch_open_since[i] == 0) {
          switch_open_since[i] = now;
        } else if (now - switch_open_since[i] >= SWITCH_OPEN_LATCH_MS) {
          switch_seen_open[i] = true;
          DEBUG3_VALUELN("Switch armed (seen open) ", i);
        }
      }
    } else {
      /* A closed reading restarts the qualification window */
      switch_open_since[i] = 0;
    }
    /* Closed only counts once the switch has proven it can stay open */
    bool value = raw[i] && switch_seen_open[i];
    if (value != switch_states[i]) {
      switch_changed[i] = true;
      data_changed = true;
      switch_states[i] = value;
      if (value) {
        DEBUG3_VALUELN("Switch on ", i);
      } else {
        DEBUG3_VALUELN("Switch off ", i);
      }
    } else {
      switch_changed[i] = false;
    }
  }

  /*
   * A complete read finished.  Reached only by falling off the end of the loop
   * above, so the flag can never be true without the bank having been sampled
   * this call -- see the clear on entry.
   */
  switches_read_ok = true;
}

/******* Capacitive Sensors ***************************************************/

void sensor_cap(void) 
{
  if (touch_sensor.readTouchInputs()) {
    DEBUG_COMMAND(DEBUG_TRACE,
                  DEBUG5_PRINT("Cap:");
                  for (uint8_t i = 0; i < MPR121::MAX_SENSORS; i++) {
                    DEBUG5_VALUE(" ", touch_sensor.touched(i));
                  }
                  DEBUG5_VALUELN(" ms:", millis());
                  );
    data_changed = true;
  }
}

/******* Handle Sensors *******************************************************/

uint8_t display_mode = 0;

uint16_t pulse_bpm_1 = 120;
uint16_t pulse_length_1 = 25;
uint16_t pulse_delay_1;

uint16_t pulse_bpm_2 = 140;
uint16_t pulse_length_2 = 25;
uint16_t pulse_delay_2;

uint16_t pulse_bpm_3 = 175;
uint16_t pulse_length_3 = 25;
uint16_t pulse_delay_3;

uint16_t pulse_bpm_4 = 200;
uint16_t pulse_length_4 = 25;
uint16_t pulse_delay_4;

uint16_t minimum_burst = 30;
uint16_t short_burst = 50; /* Short burst length in milliseconds */
uint16_t long_burst = 100; /* Long burst length in milliseconds */
uint16_t full_burst = 75; /* Large busrt length in milliseconds */

/* Exterior lights */
bool lights_on = false;
uint8_t led_mode = LED_MODE_ON;
uint8_t led_mode_value = 50;
uint8_t brightness = 96;

void calculate_pulse() {
  pulse_delay_1 = ((uint16_t)1000 * (uint16_t)60 / pulse_bpm_1) - pulse_length_1;
  pulse_delay_2 = ((uint16_t)1000 * (uint16_t)60 / pulse_bpm_2) - pulse_length_2;
  pulse_delay_3 = ((uint16_t)1000 * (uint16_t)60 / pulse_bpm_3) - pulse_length_3;
  pulse_delay_4 = ((uint16_t)1000 * (uint16_t)60 / pulse_bpm_4) - pulse_length_4;
}

void sendOn(uint16_t address, uint8_t output) {
  sendHMTLValue(address, output, 255);
}

void sendOff(uint16_t address, uint8_t output) {
  sendHMTLValue(address, output, 0);
}

void sendBurst(uint16_t address, uint8_t output, uint32_t duration) {
  sendHMTLTimedChange(address,
                      output, duration, 0xFFFFFFFF, 0);
}

void sendCancel(uint16_t address, uint8_t output) {
  sendHMTLCancel(address, output);
}

void sendCancelAndOff(uint16_t address, uint8_t output) {
  sendCancel(address, output);
  sendOff(address, output);
}

/*
 * Safe-state drives.
 *
 * These are the ONE implementation of "close these outputs".  The poofer
 * sequence used to exist as two hand-maintained copies (the poofer-disable
 * branch of handle_poof_enable() and the programs-off branch of
 * handle_single_quint()), which is exactly the kind of duplication that lets a
 * newly added output be closed on one path and left running on the other.
 *
 * Every drive CANCELS before it sets the output off.  sendBurst() runs a
 * TIMED_CHANGE program on the remote module -- 30 seconds for the igniter and
 * pilot -- and a bare value-0 does not stop that program; it re-asserts the
 * output for the rest of its duration.  The igniter and pilot off-edges used to
 * send a bare sendOff() and so did not actually close a burst that was still
 * running.  Cancel-then-off does.
 *
 * All of these send RS485 traffic, so they belong to the core that owns the bus
 * (core 1 / loop()).  The API task must never call them directly; it asks core
 * 1 for a safe state and waits for the acknowledgement.
 */
void fc_poofers_safe() {
#if CONTROL_MODE == CONTROL_SINGLE_QUINT
  sendCancelAndOff(poofer1_address, POOFER1_LARGE);
  sendCancelAndOff(poofer2_address, POOFER2_POOF1);
  sendCancelAndOff(poofer2_address, POOFER2_POOF2);
  sendCancelAndOff(poofer2_address, POOFER2_POOF3);
  sendCancelAndOff(poofer2_address, POOFER2_POOF4);
#else
  sendCancelAndOff(poofer1_address, POOFER1_POOF1);
  sendCancelAndOff(poofer1_address, POOFER1_POOF2);
#if CONTROL_MODE == CONTROL_DOUBLE_DOUBLE
  sendCancelAndOff(poofer2_address, POOFER2_POOF1);
  sendCancelAndOff(poofer2_address, POOFER2_POOF2);
#endif
#endif
}

void fc_igniter_safe() {
  sendCancelAndOff(poofer1_address, POOFER1_IGNITER);
#if CONTROL_MODE == CONTROL_DOUBLE_DOUBLE
  sendCancelAndOff(poofer2_address, POOFER2_IGNITER);
#endif
}

void fc_pilot_safe() {
  sendCancelAndOff(poofer1_address, POOFER1_PILOT);
#if CONTROL_MODE == CONTROL_DOUBLE_DOUBLE
  sendCancelAndOff(poofer2_address, POOFER2_PILOT);
#endif
}

/*
 * Everything off.  Ordering is deliberate: accumulators first (the outputs that
 * actually produce flame), then the igniter, then the pilot last -- so a drive
 * that is interrupted part-way has still closed the largest hazards.
 */
void fc_all_outputs_safe() {
  fc_poofers_safe();
  fc_igniter_safe();
  fc_pilot_safe();
}

void sendPulse(uint16_t address, uint8_t output,
               uint16_t onperiod, uint16_t offperiod) {
  sendHMTLBlink(address, output, onperiod, 0xFFFFFFFF, offperiod, 0);
}


void sendLEDMode() {
  if (lights_on) {
    switch (led_mode) {
      case LED_MODE_ON: {
        sendHMTLValue(lights_address, HMTL_ALL_OUTPUTS, brightness);
        break;
      }
      case LED_MODE_BLINK: {
        sendPulse(lights_address, HMTL_ALL_OUTPUTS,led_mode_value, led_mode_value);
        break;
      }
    }
  } else {
    sendCancelAndOff(lights_address, HMTL_ALL_OUTPUTS);
  }
}

void resetLights() {
  sendCancel(lights_address, HMTL_ALL_OUTPUTS);
  sendLEDMode();
}

/*
 * Check a sensor to see if a BPM pulse should be triggered
 */
void checkPulse(uint8_t sensor, uint16_t address, uint8_t output,
                uint16_t onperiod, uint16_t offperiod) {
  if (touch_sensor.changed(sensor)) {
    if (touch_sensor.touched(sensor)) {
      sendPulse(address, output, onperiod, offperiod);
      //sendPulse(lights_address, HMTL_ALL_OUTPUTS,  onperiod, offperiod);
    } else if (touch_sensor.changed(sensor)) {
      sendCancelAndOff(address, output);
      //resetLights();
    }
  }
}


/* Convert between a sensor number and the LED associated with it */
uint8_t sensor_to_led(uint8_t sensor) {
  uint8_t led = 0;

#if OBJECT_TYPE == OBJECT_TYPE_TOUCH_CONTROLLER
  /*
   * Sensor  LED
   * 11       0
   * 10       1
   *  9       2
   *  8       3
   *  7       4
   *  6       5
   *  5       11
   *  4       10
   *  3       9
   *  2       8
   *  1       7
   *  0       6
   */
  if (sensor > 5) {
    led = (uint8_t)11 - sensor;
  } else {
    led = sensor + (uint8_t)6;
  }
#else
  led = sensor;
#endif

  return led;
}


/*
 * Everything to do with external illumination and related switches
 */
void handle_lights() {
#if LIGHTS_ON_SWITCH != -1
  if (switch_changed[LIGHTS_ON_SWITCH]) {
    if (switch_states[LIGHTS_ON_SWITCH]) {
      DEBUG3_PRINTLN("LIGHTS ON");
      lights_on = true;
      sendLEDMode();
    } else {
      DEBUG3_PRINTLN("LIGHTS OFF");
      lights_on = false;
      sendLEDMode();
    }
  }
#else
//  /* Default to exterior lights enabled */
//  DEBUG3_PRINTLN("LIGHTS ON");
//  lights_on = true;
//  sendLEDMode();
#endif
}


/*
 * Everything to do with the hot-surface igniter and related switches
 */
void handle_ignition() {
  /* Igniter switches */
  static unsigned long last_on = 0;
  if (switch_states[POOFER_IGNITER_SWITCH]) {
    if (millis() - last_on > 15 * 1000) {
      if (switch_changed[POOFER_IGNITER_SWITCH]) {
        DEBUG2_PRINTLN("IGNITE ON");
      }
      sendBurst(poofer1_address, POOFER1_IGNITER, 30 * 1000);
#if CONTROL_MODE == CONTROL_DOUBLE_DOUBLE
      sendBurst(poofer2_address, POOFER2_IGNITER, 30 * 1000);
#endif
      last_on = millis();
    }
  } else if (switch_changed[POOFER_IGNITER_SWITCH]) {
    DEBUG2_PRINTLN("IGNITE OFF");
    fc_igniter_safe();
    last_on = 0;
  }
}

/*
 * Everything to do with the pilot light and related switches
 */
void handle_pilot() {
  /* Pilot Switch */
  if (switch_states[POOFER_PILOT_SWITCH]) {
    static unsigned long last_on = 0;
    if (millis() - last_on > 15 * 1000) {
      if (switch_changed[POOFER_PILOT_SWITCH]) {
        DEBUG1_PRINTLN("PILOT ON");
      }
      sendBurst(poofer1_address, POOFER1_PILOT, 30 * 1000);
#if CONTROL_MODE == CONTROL_DOUBLE_DOUBLE
      sendBurst(poofer2_address, POOFER2_PILOT, 30 * 1000);
#endif
      last_on = millis();
    }
  } else if (switch_changed[POOFER_PILOT_SWITCH]) {
    DEBUG1_PRINTLN("PILOT OFF");
    fc_pilot_safe();
  }
}

/*
 * Enable and disable the poofers
 */
void handle_poof_enable() {
  /*  Poofer Enable Switch */
  if (switch_changed[POOFER_ENABLE_SWITCH]) {
    if (switch_states[POOFER_ENABLE_SWITCH]) {
      DEBUG1_PRINTLN("POOFERS ENABLED");

      /* Set lights for poofing mode */
      setBlink(pixel_color(255,0,0));
    } else {
      /* Cancel all poofing programs and ensure all poofers are disabled */
      DEBUG1_PRINTLN("POOFERS DISABLED");

      fc_poofers_safe();

      /* Set lights for non-poof mode */
      setSparkle();
    }
  }
}

/*
 * Handle adjusting settings via the LCD
 */
void handle_settings() {

#if OBJECT_TYPE == OBJECT_TYPE_TOUCH_CONTROLLER
  static unsigned long settings_touch_ms = 0;
  static bool settings_mode = false;

  /* Check if entering or exiting settings mode */
  if (!switch_states[POOFER_ENABLE_SWITCH] && // Only do settings when poofing is off
      touch_sensor.touched(SENSOR_MENU_ENABLE_1) &&
      touch_sensor.touched(SENSOR_MENU_ENABLE_2)) {

    if (settings_touch_ms == 0) {
      settings_touch_ms = timesync.ms();
    }

    if (timesync.ms() - settings_touch_ms > 2000) {
      /* All settings enable buttons have been held, switch modes */
      settings_mode = !settings_mode;
      if (settings_mode) {
        setBlink(pixel_color(0, 255, 0));
      } else {
        setSparkle();
      }

      settings_touch_ms = 0;
    }
  } else {
    settings_touch_ms = 0;
  }

  if (!settings_mode) {
    return;
  }
#elif OBJECT_TYPE == OBJECT_TYPE_FIRE_CONTROLLER
  /* The wooden fire controller can alays adjust settings */
#endif

  /* Change display mode */
  if (touch_sensor.changed(SENSOR_DISPLAY_MODE) &&
      touch_sensor.touched(SENSOR_DISPLAY_MODE)) {
    lcd.clear();
    display_mode = (display_mode + 1) % NUM_DISPLAY_MODES;
  }

  /*
   * Display adjustments
   */
  if (display_mode == DISPLAY_ADJUST_BPM1_1) {
    if (touch_sensor.changed(SENSOR_LCD_UP)) {
      if (touch_sensor.touched(SENSOR_LCD_UP)) {
        pulse_bpm_1++;
        calculate_pulse();
      }
    }

    if (touch_sensor.changed(SENSOR_LCD_DOWN)) {
      if (touch_sensor.touched(SENSOR_LCD_DOWN)) {
        pulse_bpm_1--;
        calculate_pulse();
      }
    }
  }

  if (display_mode == DISPLAY_ADJUST_BPM1_2) {
    if (touch_sensor.changed(SENSOR_LCD_UP)) {
      if (touch_sensor.touched(SENSOR_LCD_UP)) {
        pulse_length_1++;
        calculate_pulse();
      }
    }

    if (touch_sensor.changed(SENSOR_LCD_DOWN)) {
      if (touch_sensor.touched(SENSOR_LCD_DOWN)) {
        pulse_length_1--;
        calculate_pulse();
      }
    }
  }

  if (display_mode == DISPLAY_ADJUST_BPM2_1) {
    if (touch_sensor.changed(SENSOR_LCD_UP)) {
      if (touch_sensor.touched(SENSOR_LCD_UP)) {
        pulse_bpm_2++;
        calculate_pulse();
      }
    }

    if (touch_sensor.changed(SENSOR_LCD_DOWN)) {
      if (touch_sensor.touched(SENSOR_LCD_DOWN)) {
        pulse_bpm_2--;
        calculate_pulse();
      }
    }
  }

  if (display_mode == DISPLAY_ADJUST_BPM2_2) {
    if (touch_sensor.changed(SENSOR_LCD_UP)) {
      if (touch_sensor.touched(SENSOR_LCD_UP)) {
        pulse_length_2++;
        calculate_pulse();
      }
    }

    if (touch_sensor.changed(SENSOR_LCD_DOWN)) {
      if (touch_sensor.touched(SENSOR_LCD_DOWN)) {
        pulse_length_2--;
        calculate_pulse();
      }
    }
  }

  if (display_mode == DISPLAY_ADJUST_BPM3_1) {
    if (touch_sensor.changed(SENSOR_LCD_UP)) {
      if (touch_sensor.touched(SENSOR_LCD_UP)) {
        pulse_bpm_3++;
        calculate_pulse();
      }
    }

    if (touch_sensor.changed(SENSOR_LCD_DOWN)) {
      if (touch_sensor.touched(SENSOR_LCD_DOWN)) {
        pulse_bpm_3--;
        calculate_pulse();
      }
    }
  }

  if (display_mode == DISPLAY_ADJUST_BPM3_2) {
    if (touch_sensor.changed(SENSOR_LCD_UP)) {
      if (touch_sensor.touched(SENSOR_LCD_UP)) {
        pulse_length_3++;
        calculate_pulse();
      }
    }

    if (touch_sensor.changed(SENSOR_LCD_DOWN)) {
      if (touch_sensor.touched(SENSOR_LCD_DOWN)) {
        pulse_length_3--;
        calculate_pulse();
      }
    }
  }

  if (display_mode == DISPLAY_ADJUST_BPM4_1) {
    if (touch_sensor.changed(SENSOR_LCD_UP)) {
      if (touch_sensor.touched(SENSOR_LCD_UP)) {
        pulse_bpm_4++;
        calculate_pulse();
      }
    }

    if (touch_sensor.changed(SENSOR_LCD_DOWN)) {
      if (touch_sensor.touched(SENSOR_LCD_DOWN)) {
        pulse_bpm_4--;
        calculate_pulse();
      }
    }
  }

  if (display_mode == DISPLAY_ADJUST_BPM4_2) {
    if (touch_sensor.changed(SENSOR_LCD_UP)) {
      if (touch_sensor.touched(SENSOR_LCD_UP)) {
        pulse_length_4++;
        calculate_pulse();
      }
    }

    if (touch_sensor.changed(SENSOR_LCD_DOWN)) {
      if (touch_sensor.touched(SENSOR_LCD_DOWN)) {
        pulse_length_4--;
        calculate_pulse();
      }
    }
  }

  if (display_mode == DISPLAY_ADJUST_BRIGHTNESS) {
    if (touch_sensor.changed(SENSOR_LCD_UP)) {
      if (touch_sensor.touched(SENSOR_LCD_UP)) {
        brightness++;
        sendLEDMode();
      }
    }

    if (touch_sensor.changed(SENSOR_LCD_DOWN)) {
      if (touch_sensor.touched(SENSOR_LCD_DOWN)) {
        brightness--;
        sendLEDMode();
      }
    }
  }

  if (display_mode == DISPLAY_LED_MODE) {
    if (touch_sensor.changed(SENSOR_LCD_UP)) {
      if (touch_sensor.touched(SENSOR_LCD_UP)) {
        led_mode = (led_mode + 1) % LED_MODE_MAX;
        sendCancel(lights_address, HMTL_ALL_OUTPUTS);
        sendLEDMode();
      }
    }

    if (touch_sensor.changed(SENSOR_LCD_DOWN)) {
      if (touch_sensor.touched(SENSOR_LCD_DOWN)) {
        led_mode_value = (led_mode_value + 1) % 100;
        sendLEDMode();
      }
    }
  }

  if (display_mode == DISPLAY_ADDRESS_MODE) {
    if (touch_sensor.changed(SENSOR_LCD_UP)) {
      if (touch_sensor.touched(SENSOR_LCD_UP)) {
        poofer1_address++;
        if (poofer1_address > 72) {
          poofer1_address = 64;
        }
      }
    }

    if (touch_sensor.changed(SENSOR_LCD_DOWN)) {
      if (touch_sensor.touched(SENSOR_LCD_DOWN)) {
        lights_address++;
        if (lights_address > 72) {
          lights_address = 64;
        }
      }
    }
  }
}

#if (CONTROL_MODE == CONTROL_SINGLE_QUINT)

/*
 * Long-combo gestures (direct mode only): both pads of a pair held for
 * LONG_COMBO_MS fire a one-shot action, then latch until both release.
 *
 * Pads 0+1 -> one COMBO_LARGE_BURST_MS burst on the large poofer (71).
 * Pads 2+3 -> one sweep of poofer outputs 1-4 on 72, driven from here as
 * individually self-expiring TIMED_CHANGE bursts (deliberately NOT the
 * module-side SEQUENCE program: that loops until a cancel frame arrives,
 * and a lost cancel must never leave poofers cycling).  A lost frame here
 * skips one step; nothing sticks on.
 *
 * Hold times are tracked locally: the MPR121 is initialized with touch-time
 * tracking disabled (HMTLTypes.cpp passes times=false), so touchTime() is 0.
 */
#ifndef LONG_COMBO_MS
  #define LONG_COMBO_MS        500  /* hold to qualify a combo */
#endif
#ifndef COMBO_LARGE_BURST_MS
  #define COMBO_LARGE_BURST_MS 500  /* pads 0+1: large-poofer burst */
#endif
#define COMBO_SEQ_STEPS 4
#define NUM_COMBO_PADS  4  /* pads 0-3; == COMBO_SEQ_STEPS only by coincidence */
#ifndef COMBO_SEQ_STEP_MS
  #define COMBO_SEQ_STEP_MS    120  /* pads 2+3: start-to-start per output */
#endif
#ifndef COMBO_SEQ_BURST_MS
  #define COMBO_SEQ_BURST_MS   100  /* pads 2+3: on-time per output */
#endif

static unsigned long combo_touch_start[NUM_COMBO_PADS] = {0, 0, 0, 0};
static boolean combo01_latched = false;
static boolean combo23_latched = false;
static uint8_t combo_seq_step = COMBO_SEQ_STEPS;  /* >= COMBO_SEQ_STEPS: idle */
static unsigned long combo_seq_next_ms = 0;

/* Abort any in-flight poofer sweep (disarm, program-mode entry) */
void combo_sequence_abort() {
  combo_seq_step = COMBO_SEQ_STEPS;
}

/* Full combo teardown: sweep aborted AND hold/latch state cleared.  Called on
 * disarm and program-mode entry so held-pad time never accumulates while the
 * combo handler is not watching — stale hold times would otherwise fire a
 * combo INSTANTLY on the transition back, bypassing the 500ms qualification. */
void combo_reset() {
  for (uint8_t i = 0; i < NUM_COMBO_PADS; i++) combo_touch_start[i] = 0;
  combo01_latched = false;
  combo23_latched = false;
  combo_sequence_abort();
}

static void update_combo_hold(uint8_t pad) {
  if (pad >= NUM_COMBO_PADS) return;
  if (touch_sensor.touched(pad)) {
    if (combo_touch_start[pad] == 0) combo_touch_start[pad] = millis();
  } else {
    combo_touch_start[pad] = 0;
  }
}

static boolean combo_held(uint8_t a, uint8_t b) {
  unsigned long now = millis();
  return combo_touch_start[a] != 0 && combo_touch_start[b] != 0 &&
         (now - combo_touch_start[a]) >= LONG_COMBO_MS &&
         (now - combo_touch_start[b]) >= LONG_COMBO_MS;
}

/* Step the pads-2+3 sweep; called every pass while armed in direct mode */
void run_combo_sequence() {
  if (combo_seq_step >= COMBO_SEQ_STEPS) return;
  if (!fc_is_armed()) {
    /* Disarmed mid-sweep: the remaining steps must never fire */
    combo_sequence_abort();
    return;
  }
  /* The sweep runs only while BOTH pads stay held: releasing either stops
   * it at the next step boundary (already-sent bursts self-expire) */
  if (!touch_sensor.touched(POOFER3_QUICK_SENSOR) ||
      !touch_sensor.touched(POOFER4_QUICK_SENSOR)) {
    combo_sequence_abort();
    return;
  }
  unsigned long now = millis();
  if ((long)(now - combo_seq_next_ms) < 0) return;
  static const uint8_t seq_outputs[COMBO_SEQ_STEPS] =
      {POOFER2_POOF1, POOFER2_POOF2, POOFER2_POOF3, POOFER2_POOF4};
  DEBUG4_VALUELN("Combo seq step:", combo_seq_step);
  sendBurst(poofer2_address, seq_outputs[combo_seq_step], COMBO_SEQ_BURST_MS);
  combo_seq_step++;
  if (combo_seq_step >= COMBO_SEQ_STEPS) {
    combo_seq_step = 0;  /* wrap: loop for as long as the pads stay held */
  }
  combo_seq_next_ms = now + COMBO_SEQ_STEP_MS;
}

void handle_long_combos() {
  /* A wedged I2C bus leaves touched() frozen at its last value; combos are
   * level-triggered, so stale touched-state must read as all-released or a
   * dead sensor could sustain the sweep with nobody at the panel. */
  if (!touch_sensor.readOk()) {
    combo_reset();
    return;
  }

  update_combo_hold(POOFER1_QUICK_SENSOR);
  update_combo_hold(POOFER2_QUICK_SENSOR);
  update_combo_hold(POOFER3_QUICK_SENSOR);
  update_combo_hold(POOFER4_QUICK_SENSOR);

  if (combo_held(POOFER1_QUICK_SENSOR, POOFER2_QUICK_SENSOR)) {
    if (!combo01_latched) {
      combo01_latched = true;
      DEBUG4_PRINTLN("Combo 0+1: large poofer");
      sendBurst(poofer1_address, POOFER1_LARGE, COMBO_LARGE_BURST_MS);
    }
  } else if (!touch_sensor.touched(POOFER1_QUICK_SENSOR) &&
             !touch_sensor.touched(POOFER2_QUICK_SENSOR)) {
    combo01_latched = false;
  }

  if (combo_held(POOFER3_QUICK_SENSOR, POOFER4_QUICK_SENSOR)) {
    if (!combo23_latched) {
      combo23_latched = true;
      DEBUG4_PRINTLN("Combo 2+3: poofer sweep");
      combo_seq_step = 0;
      combo_seq_next_ms = millis();
    }
  } else if (!touch_sensor.touched(POOFER3_QUICK_SENSOR) &&
             !touch_sensor.touched(POOFER4_QUICK_SENSOR)) {
    combo23_latched = false;
  }

  run_combo_sequence();
}

void handle_single_quint() {
  if (switch_states[PROGRAM_MODE_SWITCH]) {
    /* Capacitive touch runs programs */

    if (switch_changed[PROGRAM_MODE_SWITCH]) {
      DEBUG2_PRINTLN("Programs on");
      setBlink(pixel_color(0, 0, 255));
      combo_reset();
    }

    checkPulse(POOFER1_QUICK_SENSOR,poofer2_address,POOFER2_POOF1,
               pulse_length_1, pulse_delay_1);
    checkPulse(POOFER2_QUICK_SENSOR,poofer2_address,POOFER2_POOF2,
               pulse_length_1, pulse_delay_1);

    checkPulse(POOFER3_QUICK_SENSOR,poofer2_address,POOFER2_POOF3,
               pulse_length_2, pulse_delay_2);
    checkPulse(POOFER4_QUICK_SENSOR,poofer2_address,POOFER2_POOF4,
               pulse_length_2, pulse_delay_2);

    checkPulse(POOFER1_LONG_SENSOR,poofer2_address,POOFER2_POOF1,
               pulse_length_3, pulse_delay_3);
    checkPulse(POOFER2_LONG_SENSOR,poofer2_address,POOFER2_POOF2,
               pulse_length_3, pulse_delay_3);

    checkPulse(POOFER3_LONG_SENSOR,poofer2_address,POOFER2_POOF3,
               pulse_length_4, pulse_delay_4);
    checkPulse(POOFER4_LONG_SENSOR,poofer2_address,POOFER2_POOF4,
               pulse_length_4, pulse_delay_4);

    checkPulse(POOFER5_QUICK_SENSOR,poofer2_address,POOFER2_POOF1,
               pulse_length_1, pulse_delay_1);
    checkPulse(POOFER5_QUICK_SENSOR,poofer2_address,POOFER2_POOF2,
               pulse_delay_1, pulse_length_1);

    checkPulse(POOFER5_LONG_SENSOR,poofer2_address,POOFER2_POOF1,
               pulse_length_4, pulse_delay_4);
    checkPulse(POOFER5_LONG_SENSOR,poofer2_address,POOFER2_POOF2,
               pulse_delay_4, pulse_length_4);

#if POOFER_PROGRAM_1_SENSOR >= 0
    checkPulse(POOFER_PROGRAM_1_SENSOR,poofer2_address,POOFER2_POOF1,
               pulse_length_1, pulse_delay_1);
    checkPulse(POOFER_PROGRAM_1_SENSOR,poofer2_address,POOFER2_POOF2,
               pulse_length_1, pulse_delay_1);
    checkPulse(POOFER_PROGRAM_1_SENSOR,poofer2_address,POOFER2_POOF3,
               pulse_length_1, pulse_delay_1);
    checkPulse(POOFER_PROGRAM_1_SENSOR,poofer2_address,POOFER2_POOF4,
               pulse_length_1, pulse_delay_1);
#endif

#if POOFER_PROGRAM_2_SENSOR >= 0
    checkPulse(POOFER_PROGRAM_2_SENSOR,poofer2_address,POOFER2_POOF1,
               pulse_length_3, pulse_delay_3);
    checkPulse(POOFER_PROGRAM_2_SENSOR,poofer2_address,POOFER2_POOF2,
               pulse_delay_3, pulse_length_3);
    checkPulse(POOFER_PROGRAM_2_SENSOR,poofer2_address,POOFER2_POOF3,
               pulse_length_3, pulse_delay_3);
    checkPulse(POOFER_PROGRAM_2_SENSOR,poofer2_address,POOFER2_POOF4,
               pulse_delay_3, pulse_length_3);
#endif
  } else {
    /* Capacitive touch controls directly */

    if (switch_changed[PROGRAM_MODE_SWITCH]) {
      DEBUG3_PRINTLN("Programs off");

      fc_poofers_safe();

      setBlink(pixel_color(255,0,0));
    }

    handle_long_combos();

    if (touch_sensor.changed(POOFER1_QUICK_SENSOR) &&
        touch_sensor.touched(POOFER1_QUICK_SENSOR)) {
      sendBurst(poofer2_address, POOFER2_POOF1, short_burst);
    }

    if (touch_sensor.changed(POOFER2_QUICK_SENSOR) &&
        touch_sensor.touched(POOFER2_QUICK_SENSOR)) {
      sendBurst(poofer2_address, POOFER2_POOF2, short_burst);
    }

    if (touch_sensor.changed(POOFER3_QUICK_SENSOR) &&
        touch_sensor.touched(POOFER3_QUICK_SENSOR)) {
      sendBurst(poofer2_address, POOFER2_POOF3, short_burst);
    }

    if (touch_sensor.changed(POOFER4_QUICK_SENSOR) &&
        touch_sensor.touched(POOFER4_QUICK_SENSOR)) {
      sendBurst(poofer2_address, POOFER2_POOF4, short_burst);
    }

    if (touch_sensor.changed(POOFER5_QUICK_SENSOR) &&
        touch_sensor.touched(POOFER5_QUICK_SENSOR)) {
      sendBurst(poofer1_address, POOFER1_LARGE, short_burst);
    }

    if (touch_sensor.changed(POOFER1_LONG_SENSOR) &&
        touch_sensor.touched(POOFER1_LONG_SENSOR)) {
      sendBurst(poofer2_address, POOFER2_POOF1, long_burst);
    }

    if (touch_sensor.changed(POOFER2_LONG_SENSOR) &&
        touch_sensor.touched(POOFER2_LONG_SENSOR)) {
      sendBurst(poofer2_address, POOFER2_POOF2, long_burst);
    }

    if (touch_sensor.changed(POOFER3_LONG_SENSOR) &&
        touch_sensor.touched(POOFER3_LONG_SENSOR)) {
      sendBurst(poofer2_address, POOFER2_POOF3, long_burst);
    }

    if (touch_sensor.changed(POOFER4_LONG_SENSOR) &&
        touch_sensor.touched(POOFER4_LONG_SENSOR)) {
      sendBurst(poofer2_address, POOFER2_POOF4, long_burst);
    }

    if (touch_sensor.changed(POOFER5_LONG_SENSOR) &&
        touch_sensor.touched(POOFER5_LONG_SENSOR)) {
      sendBurst(poofer1_address, POOFER1_LARGE, long_burst);
    }

#if POOFER_PROGRAM_1_SENSOR >= 0
    if (touch_sensor.changed(POOFER_PROGRAM_1_SENSOR) &&
        touch_sensor.touched(POOFER_PROGRAM_1_SENSOR)) {
      /* All on quick burst */
      sendBurst(poofer2_address, POOFER2_POOF1, minimum_burst);
      sendBurst(poofer2_address, POOFER2_POOF2, minimum_burst);
      sendBurst(poofer2_address, POOFER2_POOF3, minimum_burst);
      sendBurst(poofer2_address, POOFER2_POOF4, minimum_burst);
//      sendBurst(poofer1_address, POOFER1_LARGE, minimum_burst);
    }
#endif

#if POOFER_PROGRAM_2_SENSOR >= 0
    if (touch_sensor.changed(POOFER_PROGRAM_2_SENSOR) &&
        touch_sensor.touched(POOFER_PROGRAM_2_SENSOR)) {
      /* All on large burst */
      sendBurst(poofer2_address, POOFER2_POOF1, full_burst);
      sendBurst(poofer2_address, POOFER2_POOF2, full_burst);
      sendBurst(poofer2_address, POOFER2_POOF3, full_burst);
      sendBurst(poofer2_address, POOFER2_POOF4, full_burst);
//      sendBurst(poofer1_address, POOFER1_LARGE, full_burst);
    }
#endif
  }
}
#endif

void handle_sensors() {

  /* Handlers for external devices */
  handle_lights();
  handle_ignition();
  handle_pilot();
  handle_poof_enable();

  if (switch_states[POOFER_ENABLE_SWITCH] &&
      switch_states[POOFER_PILOT_SWITCH]) {
    /* Poofers are enabled and the pilot is open */


#if (CONTROL_MODE == CONTROL_SINGLE_QUINT)
    handle_single_quint();
    return;
#else

    /*
     * Main control box sensors
     */

#if (CONTROL_MODE == CONTROL_SINGLE_DOUBLE) || (CONTROL_MODE == CONTROL_SINGLE_QUAD) || (CONTROL_MODE == CONTROL_DOUBLE_DOUBLE)
    /* Brief burst */
    if (touch_sensor.changed(POOFER1_POOF1_QUICK_SENSOR) &&
        touch_sensor.touched(POOFER1_POOF1_QUICK_SENSOR)) {
      sendBurst(poofer1_address, POOFER1_POOF1, 50);
    }

    if (touch_sensor.changed(POOFER1_POOF2_QUICK_SENSOR) &&
        touch_sensor.touched(POOFER1_POOF2_QUICK_SENSOR)) {
      sendBurst(poofer1_address, POOFER1_POOF2, 50);
    }
#endif

#if OBJECT_TYPE == OBJECT_TYPE_TOUCH_CONTROLLER
    if (touch_sensor.changed(POOFER2_POOF1_QUICK_SENSOR) &&
        touch_sensor.touched(POOFER2_POOF1_QUICK_SENSOR)) {
      sendBurst(poofer2_address, POOFER2_POOF1, 50);
    }

    if (touch_sensor.changed(POOFER2_POOF2_QUICK_SENSOR) &&
        touch_sensor.touched(POOFER2_POOF2_QUICK_SENSOR)) {
      sendBurst(poofer2_address, POOFER2_POOF2, 50);
    }
#endif

#if (CONTROL_MODE == CONTROL_SINGLE_QUAD)
    /*
     * For four cylinders as used as Wickerman 2018, where there is a single
     * pilot and ignitor, but four separate accumulators.
     * XXX: This currently just assumes that one module is wired as with
     * CONTROL_SINGLE_DOUBLE and the second module has the poofers in the same
     * controls as CONTROL_DOUBLE_DOUBLE
     */

    if (touch_sensor.changed(POOFER1_POOF1_LONG_SENSOR) &&
        touch_sensor.touched(POOFER1_POOF1_LONG_SENSOR)) {
      sendBurst(poofer2_address, POOFER2_POOF1, 50);
    }

    if (touch_sensor.changed(POOFER1_POOF2_LONG_SENSOR) &&
        touch_sensor.touched(POOFER1_POOF2_LONG_SENSOR)) {
      sendBurst(poofer2_address, POOFER2_POOF2, 50);
    }
#elif 0
    /* On for length of touch */
    if (touch_sensor.touched(POOFER1_POOF1_LONG_SENSOR)) {
      sendBurst(poofer1_address, POOFER1_POOF1, 250);
    } else if (touch_sensor.changed(POOFER1_POOF1_LONG_SENSOR)) {
      sendCancelAndOff(poofer1_address, POOFER1_POOF1);
    }

    if (touch_sensor.touched(POOFER2_POOF2_LONG_SENSOR)) {
      sendBurst(poofer1_address, POOFER1_POOF2, 250);
    } else if (touch_sensor.changed(POOFER2_POOF2_LONG_SENSOR)) {
      sendCancelAndOff(poofer1_address, POOFER1_POOF2);
    }
#elif 0
    /* Pulse the poofers */
    if (touch_sensor.changed(POOFER1_POOF1_LONG_SENSOR)) {
      if (touch_sensor.touched(POOFER1_POOF1_LONG_SENSOR)) {
        sendPulse(poofer1_address, POOFER1_POOF1,
                /*on period*/ pulse_length_1, /*off period*/ pulse_delay_1);
        sendPulse(lights_address, HMTL_ALL_OUTPUTS,
                /*on period*/ pulse_length_1, /*off period*/ pulse_delay_1);
      } else if (touch_sensor.changed(POOFER1_POOF1_LONG_SENSOR)) {
        sendCancelAndOff(poofer1_address, POOFER1_POOF1);

        sendCancel(lights_address, HMTL_ALL_OUTPUTS);
        sendLEDMode();
      }
    }

    if (touch_sensor.changed(POOFER2_POOF2_LONG_SENSOR)) {
      if (touch_sensor.touched(POOFER2_POOF2_LONG_SENSOR)) {
        sendPulse(poofer1_address, POOFER1_POOF2,
                /*on period*/ pulse_length_2, /*off period*/ pulse_delay_2);
        sendPulse(lights_address, HMTL_ALL_OUTPUTS,
                /*on period*/ pulse_length_2, /*off period*/ pulse_delay_2);
      } else if (touch_sensor.changed(POOFER2_POOF2_LONG_SENSOR)) {
        sendCancelAndOff(poofer1_address, POOFER1_POOF2);

        sendCancel(lights_address, HMTL_ALL_OUTPUTS);
        sendLEDMode();
      }
    }

#else
    /* On for length of touch up to maximum value */
    #define MAXIMUM_BURST (10 * 1000)

    static unsigned long poofer1_poof1_on_ms = 0;
    static unsigned long poofer1_poof2_on_ms = 0;
    static unsigned long poofer2_poof1_on_ms = 0;
    static unsigned long poofer2_poof2_on_ms = 0;

    if (touch_sensor.touched(POOFER1_POOF1_LONG_SENSOR)) {
      if (poofer1_poof1_on_ms == 0) {
        poofer1_poof1_on_ms = millis();
      }
      if (poofer1_poof1_on_ms - millis() <= MAXIMUM_BURST) {
        sendBurst(poofer1_address, POOFER1_POOF1, 250);
      }
    } else if (touch_sensor.changed(POOFER1_POOF1_LONG_SENSOR)) {
      sendCancelAndOff(poofer1_address, POOFER1_POOF1);
      poofer1_poof1_on_ms = 0;
    }

    if (touch_sensor.touched(POOFER1_POOF2_LONG_SENSOR)) {
      if (poofer1_poof2_on_ms == 0) {
        poofer1_poof2_on_ms = millis();
      }
      if (poofer1_poof2_on_ms - millis() <= MAXIMUM_BURST) {
        sendBurst(poofer1_address, POOFER1_POOF2, 250);
      }
    } else if (touch_sensor.changed(POOFER1_POOF2_LONG_SENSOR)) {
      sendCancelAndOff(poofer1_address, POOFER1_POOF2);
      poofer1_poof2_on_ms = 0;
    }

#if (OBJECT_TYPE == OBJECT_TYPE_TOUCH_CONTROLLER) && (CONTROL_MODE == CONTROL_DOUBLE_DOUBLE)
    if (touch_sensor.touched(POOFER2_POOF1_LONG_SENSOR)) {
      if (poofer2_poof1_on_ms == 0) {
        poofer2_poof1_on_ms = millis();
      }
      if (poofer2_poof1_on_ms - millis() <= MAXIMUM_BURST) {
        sendBurst(poofer2_address, POOFER2_POOF1, 250);
      }
    } else if (touch_sensor.changed(POOFER2_POOF1_LONG_SENSOR)) {
      sendCancelAndOff(poofer2_address, POOFER2_POOF1);
      poofer2_poof1_on_ms = 0;
    }

    if (touch_sensor.touched(POOFER2_POOF2_LONG_SENSOR)) {
      if (poofer2_poof2_on_ms == 0) {
        poofer2_poof2_on_ms = millis();
      }
      if (poofer2_poof2_on_ms - millis() <= MAXIMUM_BURST) {
        sendBurst(poofer2_address, POOFER2_POOF2, 250);
      }
    } else if (touch_sensor.changed(POOFER2_POOF2_LONG_SENSOR)) {
      sendCancelAndOff(poofer2_address, POOFER2_POOF2);
      poofer2_poof2_on_ms = 0;
    }
#endif // OBJECT_TYPE == OBJECT_TYPE_TOUCH_CONTROLLER

#endif

#if OBJECT_TYPE == OBJECT_TYPE_FIRE_CONTROLLER
    /*
     * External Sensors
     */

    /* Pulse the poofers */
    if (touch_sensor.changed(SENSOR_EXTERNAL_1)) {
      if (touch_sensor.touched(SENSOR_EXTERNAL_1)) {
        sendPulse(poofer1_address, POOFER1_POOF1,
                /*on period*/ pulse_length_1, /*off period*/ pulse_delay_1);
        sendPulse(lights_address, HMTL_ALL_OUTPUTS,
                /*on period*/ pulse_length_1, /*off period*/ pulse_delay_1);
      } else if (touch_sensor.changed(SENSOR_EXTERNAL_1)) {
        sendCancelAndOff(poofer1_address, POOFER1_POOF1);

        resetLights();
      }
    }

    if (touch_sensor.changed(SENSOR_EXTERNAL_4)) {
      if (touch_sensor.touched(SENSOR_EXTERNAL_4)) {
        sendPulse(poofer1_address, POOFER1_POOF2,
                /*on period*/ pulse_length_2, /*off period*/ pulse_delay_2);
        sendPulse(lights_address, HMTL_ALL_OUTPUTS,
                /*on period*/ pulse_length_2, /*off period*/ pulse_delay_2);
      } else if (touch_sensor.changed(SENSOR_EXTERNAL_4)) {
        sendCancelAndOff(poofer1_address, POOFER1_POOF2);

        resetLights();
      }
    }

    /* Minimal burst */
    if (touch_sensor.changed(SENSOR_EXTERNAL_2) &&
        touch_sensor.touched(SENSOR_EXTERNAL_2)) {
      sendBurst(poofer1_address, POOFER1_POOF1, 25);
    }

    if (touch_sensor.changed(SENSOR_EXTERNAL_3) &&
        touch_sensor.touched(SENSOR_EXTERNAL_3)) {
      sendBurst(poofer1_address, POOFER1_POOF2, 25);
    }

#endif

#if OBJECT_TYPE == OBJECT_TYPE_TOUCH_CONTROLLER
    /* Pulse the poofers */
    checkPulse(POOFER1_MODE1_SENSOR, poofer1_address, POOFER1_POOF1,
            /*on period*/ pulse_length_1, /*off period*/ pulse_delay_1);
    checkPulse(POOFER1_MODE2_SENSOR, poofer1_address, POOFER1_POOF2,
            /*on period*/ pulse_length_2, /*off period*/ pulse_delay_2);

    checkPulse(POOFER2_MODE1_SENSOR, poofer2_address, POOFER2_POOF1,
            /*on period*/ pulse_length_3, /*off period*/ pulse_delay_3);
    checkPulse(POOFER2_MODE2_SENSOR, poofer2_address, POOFER2_POOF2,
            /*on period*/ pulse_length_4, /*off period*/ pulse_delay_4);

#endif

#endif
  } else {
#if (CONTROL_MODE == CONTROL_SINGLE_QUINT)
    /* Disarmed: abort any in-flight sweep AND clear hold/latch state, so
     * pads held across a disarm cannot fire the instant of re-arm */
    combo_reset();
#endif
  }
  // END: Poofer controls

  /* Check for settings adjustments */
  handle_settings();
}

void initialize_display() {
#ifdef ESP32
  lcd.init();
  lcd.backlight();
#else
  lcd.begin(16, 2);
  lcd.setBacklight(HIGH);
#endif

  lcd.setCursor(0, 0);
  lcd.print("Initializing");
}


void update_lcd() {
  switch (display_mode) {
    case DISPLAY_CAP_SENSORS: {
      /* Display the value of sensors and switches */
      if (data_changed) {
        lcd.setCursor(0, 0);
        lcd.print("C:");
        for (uint8_t i = 0; i < MPR121::MAX_SENSORS; i++) {
          lcd.print(touch_sensor.touched(i));
        }
        lcd.print("    ");

        lcd.setCursor(0, 1);
        lcd.print("S:");
        for (uint8_t i = 0; i < NUM_SWITCHES; i++) {
          lcd.print(switch_states[i]);
        }
        lcd.print("      ");
    
        data_changed = false;
      }
      break;
    }

    case DISPLAY_ADJUST_BPM1_1:
    case DISPLAY_ADJUST_BPM1_2:
    {
      lcd.setCursor(0, 0);
      lcd.print("BPM1:");
      lcd.print(pulse_bpm_1);
      lcd.print("    ");

      lcd.setCursor(0, 1);
      lcd.print("Len:");
      lcd.print(pulse_length_1);
      lcd.print(" D:");
      lcd.print(pulse_delay_1);
      lcd.print("    ");
      break;
    }

    case DISPLAY_ADJUST_BPM2_1:
    case DISPLAY_ADJUST_BPM2_2:
    {
      lcd.setCursor(0, 0);
      lcd.print("BPM2:");
      lcd.print(pulse_bpm_2);
      lcd.print("    ");

      lcd.setCursor(0, 1);
      lcd.print("Len:");
      lcd.print(pulse_length_2);
      lcd.print(" D:");
      lcd.print(pulse_delay_2);
      lcd.print("    ");
      break;
    }

    case DISPLAY_ADJUST_BPM3_1:
    case DISPLAY_ADJUST_BPM3_2:
    {
      lcd.setCursor(0, 0);
      lcd.print("BPM3:");
      lcd.print(pulse_bpm_3);
      lcd.print("    ");

      lcd.setCursor(0, 1);
      lcd.print("Len:");
      lcd.print(pulse_length_3);
      lcd.print(" D:");
      lcd.print(pulse_delay_3);
      lcd.print("    ");
      break;
    }


    case DISPLAY_ADJUST_BPM4_1:
    case DISPLAY_ADJUST_BPM4_2:
    {
      lcd.setCursor(0, 0);
      lcd.print("BPM4:");
      lcd.print(pulse_bpm_4);
      lcd.print("    ");

      lcd.setCursor(0, 1);
      lcd.print("Len:");
      lcd.print(pulse_length_4);
      lcd.print(" D:");
      lcd.print(pulse_delay_4);
      lcd.print("    ");
      break;
    }

    case DISPLAY_ADJUST_BRIGHTNESS: {
      lcd.setCursor(0, 0);
      lcd.print("BRIGHTNESS:");
      lcd.print(brightness);
      lcd.print("       ");
      break;
    }

    case DISPLAY_LED_MODE: {
      lcd.setCursor(0, 0);
      lcd.print("LEDs:");
      switch (led_mode) {
        case LED_MODE_ON: {
          lcd.print("ON");
          break;
        }
        case LED_MODE_BLINK: {
          lcd.print("BLINK");
          break;
        }
      }
      lcd.print("       ");

      lcd.setCursor(0, 1);
      lcd.print("VALUE:");
      lcd.print(led_mode_value);
      break;
    }

    case DISPLAY_ADDRESS_MODE: {
      lcd.setCursor(0, 0);
      lcd.print("POOF1_ADDR:");
      lcd.print(poofer1_address);

      lcd.setCursor(0, 1);
      lcd.print("POOF2_ADDR:");
      lcd.print(lights_address);
      break;
    }

  }
}