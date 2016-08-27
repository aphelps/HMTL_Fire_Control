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
#include <NewPing.h>
#include <Wire.h>
#include "MPR121.h"

#include "HMTLTypes.h"
#include "HMTLMessaging.h"
#include "HMTLPoofer.h"

#include "HMTL_Fire_Control.h"
#include "modes.h"
#include "Fire_Control_Sensors.h"

boolean data_changed = true;

uint16_t poofer_address = POOFER1_ADDRESS;
uint16_t lights_address = LIGHTS_ADDRESS;

/******* Switches *************************************************************/

#define NUM_SWITCHES 4
boolean switch_states[NUM_SWITCHES] = { false, false, false, false };
boolean switch_changed[NUM_SWITCHES] = { false, false, false, false };
const byte switch_pins[NUM_SWITCHES] = {
  SWITCH_PIN_1, SWITCH_PIN_2, SWITCH_PIN_3, SWITCH_PIN_4 };

void initialize_switches(void) {
  for (byte i = 0; i < NUM_SWITCHES; i++) {
    pinMode(switch_pins[i], INPUT);
  }

  calculate_pulse();
}

void sensor_switches(void) {
  for (byte i = 0; i < NUM_SWITCHES; i++) {
    boolean value = (digitalRead(switch_pins[i]) == LOW);
    if (value != switch_states[i]) {
      switch_changed[i] = true;
      data_changed = true;
      switch_states[i] = value;
      if (value) {
        DEBUG5_VALUELN("Switch is on: ", i);
      } else {
        DEBUG5_VALUELN("Switch is off: ", i);
      }
    } else {
      switch_changed[i] = false;
    }
  }
}

/******* Capacitive Sensors ***************************************************/

void sensor_cap(void) 
{
  if (touch_sensor.readTouchInputs()) {
    DEBUG_COMMAND(DEBUG_TRACE,
                  DEBUG5_PRINT("Cap:");
                  for (byte i = 0; i < MPR121::MAX_SENSORS; i++) {
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

uint16_t pulse_bpm_2 = 240;
uint16_t pulse_length_2 = 25;
uint16_t pulse_delay_2;

uint8_t brightness = 96;

boolean lights_on = false;
uint8_t led_mode = LED_MODE_ON;
uint8_t led_mode_value = 50;

void calculate_pulse() {
  pulse_delay_1 = ((uint16_t)1000 * (uint16_t)60 / pulse_bpm_1) - pulse_length_1;
  pulse_delay_2 = ((uint16_t)1000 * (uint16_t)60 / pulse_bpm_2) - pulse_length_2;
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
    sendCancel(lights_address, HMTL_ALL_OUTPUTS);
    sendOff(lights_address, HMTL_ALL_OUTPUTS);
  }
}


/* Convert between a sensor number and the LED associated with it */
byte sensor_to_led(byte sensor) {
  byte led;

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
    led = (byte)11 - sensor;
  } else {
    led = sensor + (byte)6;
  }
#endif

  return led;
}

void handle_sensors(void) {
  static unsigned long last_send = millis();

  /* Goblin Lights */
  if (switch_changed[LIGHTS_ON_SWITCH]) {
    if (switch_states[LIGHTS_ON_SWITCH]) {
      DEBUG1_PRINTLN("LIGHTS ON");
      lights_on = true;
      sendLEDMode();
    } else {
      DEBUG1_PRINTLN("LIGHTS OFF");
      lights_on = false;
      sendLEDMode();
    }
  }

  /* Igniter switches */
  if (switch_states[POOFER1_IGNITER_SWITCH]) {
    static unsigned long last_on = 0;
    if (millis() - last_on > 15 * 1000) {
      DEBUG1_PRINTLN("IGNITE ON");
      sendBurst(POOFER1_ADDRESS, POOFER1_IGNITER, 30 * 1000);
      last_on = millis();
    }
  } else if (switch_changed[POOFER1_IGNITER_SWITCH]) {
    DEBUG1_PRINTLN("IGNITE OFF");
    sendOff(POOFER1_ADDRESS, POOFER1_IGNITER);
  }

  /* Pilot Switch */
  if (switch_states[POOFER1_PILOT_SWITCH]) {
    static unsigned long last_on = 0;
    if (millis() - last_on > 15 * 1000) {
      DEBUG1_PRINTLN("PILOT ON");
      sendBurst(POOFER1_ADDRESS, POOFER1_PILOT, 30 * 1000);
      last_on = millis();
    }
  } else if (switch_changed[POOFER1_PILOT_SWITCH]) {
    DEBUG1_PRINTLN("PILOT OFF");
    sendOff(POOFER1_ADDRESS, POOFER1_PILOT);
  }

  /*  Poofer Enable Switch */
  if (switch_changed[POOFER1_ENABLE_SWITCH]) {
    if (switch_states[POOFER1_ENABLE_SWITCH]) {
      DEBUG1_PRINTLN("POOFERS ENABLED");
      setBlink();
    } else {
      DEBUG1_PRINTLN("POOFERS DISABLED");
      sendCancel(poofer_address, POOFER1_POOF1);
      sendCancel(poofer_address, POOFER1_POOF2);

      sendOff(poofer_address, POOFER1_POOF1);
      sendOff(poofer_address, POOFER1_POOF2);
      setSparkle();
    }
  }

#if OBJECT_TYPE == OBJECT_TYPE_TOUCH_CONTROLLER
  return;
#endif

  if (switch_states[POOFER1_ENABLE_SWITCH] &&
      switch_states[POOFER1_PILOT_SWITCH]) {
    /* Poofers are enabled and the pilot is open */

    /*
     * Main control box sensors
     */

    /* Brief burst */
    if (touch_sensor.changed(POOFER1_QUICK_POOF_SENSOR) &&
        touch_sensor.touched(POOFER1_QUICK_POOF_SENSOR)) {
      sendBurst(poofer_address, POOFER1_POOF1, 50);
    }

    if (touch_sensor.changed(POOFER2_QUICK_POOF_SENSOR) &&
        touch_sensor.touched(POOFER2_QUICK_POOF_SENSOR)) {
      sendBurst(poofer_address, POOFER1_POOF2, 50);
    }

#if 0
    /* On for length of touch */
    if (touch_sensor.touched(POOFER1_LONG_POOF_SENSOR)) {
      sendBurst(poofer_address, POOFER1_POOF1, 250);
    } else if (touch_sensor.changed(POOFER1_LONG_POOF_SENSOR)) {
      sendOff(poofer_address, POOFER1_POOF1);
      sendCancel(poofer_address, POOFER1_POOF1);
    }

    if (touch_sensor.touched(POOFER2_LONG_POOF_SENSOR)) {
      sendBurst(poofer_address, POOFER1_POOF2, 250);
    } else if (touch_sensor.changed(POOFER2_LONG_POOF_SENSOR)) {
      sendOff(poofer_address, POOFER1_POOF2);
      sendCancel(poofer_address, POOFER1_POOF2);
    }
#elif 1
    /* Pulse the poofers */
    if (touch_sensor.changed(POOFER1_LONG_POOF_SENSOR)) {
      if (touch_sensor.touched(POOFER1_LONG_POOF_SENSOR)) {
        sendPulse(poofer_address, POOFER1_POOF1,
                /*on period*/ pulse_length_1, /*off period*/ pulse_delay_1);
        sendPulse(lights_address, HMTL_ALL_OUTPUTS,
                /*on period*/ pulse_length_1, /*off period*/ pulse_delay_1);
      } else if (touch_sensor.changed(POOFER1_LONG_POOF_SENSOR)) {
        sendCancel(poofer_address, POOFER1_POOF1);
        sendOff(poofer_address, POOFER1_POOF1);

        sendCancel(lights_address, HMTL_ALL_OUTPUTS);
        sendLEDMode();
      }
    }

    if (touch_sensor.changed(POOFER2_LONG_POOF_SENSOR)) {
      if (touch_sensor.touched(POOFER2_LONG_POOF_SENSOR)) {
        sendPulse(poofer_address, POOFER1_POOF2,
                /*on period*/ pulse_length_2, /*off period*/ pulse_delay_2);
        sendPulse(lights_address, HMTL_ALL_OUTPUTS,
                /*on period*/ pulse_length_2, /*off period*/ pulse_delay_2);
      } else if (touch_sensor.changed(POOFER2_LONG_POOF_SENSOR)) {
        sendCancel(poofer_address, POOFER1_POOF2);
        sendOff(poofer_address, POOFER1_POOF2);

        sendCancel(lights_address, HMTL_ALL_OUTPUTS);
        sendLEDMode();
      }
    }

#else
    /* On for length of touch up to maximum value */
    static unsigned long poof1_on_ms = 0;
    static unsigned long poof2_on_ms = 0;

    if (touch_sensor.touched(POOFER1_LONG_POOF_SENSOR)) {
      if (poof1_on_ms == 0) {
        poof1_on_ms = millis();
      }
      if (poof1_on_ms - millis() <= 2 * 1000) {
        sendBurst(poofer_address, POOFER1_POOF1, 250);
      }
    } else if (touch_sensor.changed(POOFER1_LONG_POOF_SENSOR)) {
      sendOff(poofer_address, POOFER1_POOF1);
      sendCancel(poofer_address, POOFER1_POOF1);
      poof1_on_ms = 0;
    }

    if (touch_sensor.touched(POOFER2_LONG_POOF_SENSOR)) {
      if (poof2_on_ms == 0) {
        poof2_on_ms = millis();
      }
      if (poof2_on_ms - millis() <= 2 * 1000) {
        sendBurst(poofer_address, POOFER1_POOF2, 250);
      }
    } else if (touch_sensor.changed(POOFER2_LONG_POOF_SENSOR)) {
      sendOff(poofer_address, POOFER1_POOF2);
      sendCancel(poofer_address, POOFER1_POOF2);
      poof2_on_ms = 0;
    }
#endif

#ifdef FIRE_CONTROLLER
    /*
     * External Sensors
     */

    /* Pulse the poofers */
    if (touch_sensor.changed(SENSOR_EXTERNAL_1)) {
      if (touch_sensor.touched(SENSOR_EXTERNAL_1)) {
        sendPulse(poofer_address, POOFER1_POOF1,
                /*on period*/ pulse_length_1, /*off period*/ pulse_delay_1);
        sendPulse(lights_address, HMTL_ALL_OUTPUTS,
                /*on period*/ pulse_length_1, /*off period*/ pulse_delay_1);
      } else if (touch_sensor.changed(SENSOR_EXTERNAL_1)) {
        sendCancel(poofer_address, POOFER1_POOF1);
        sendOff(poofer_address, POOFER1_POOF1);

        sendCancel(lights_address, HMTL_ALL_OUTPUTS);
        sendLEDMode();
      }
    }

    if (touch_sensor.changed(SENSOR_EXTERNAL_4)) {
      if (touch_sensor.touched(SENSOR_EXTERNAL_4)) {
        sendPulse(poofer_address, POOFER1_POOF2,
                /*on period*/ pulse_length_2, /*off period*/ pulse_delay_2);
        sendPulse(lights_address, HMTL_ALL_OUTPUTS,
                /*on period*/ pulse_length_2, /*off period*/ pulse_delay_2);
      } else if (touch_sensor.changed(SENSOR_EXTERNAL_4)) {
        sendCancel(poofer_address, POOFER1_POOF2);
        sendOff(poofer_address, POOFER1_POOF2);

        sendCancel(lights_address, HMTL_ALL_OUTPUTS);
        sendLEDMode();
      }
    }

    /* Minimal burst */
    if (touch_sensor.changed(SENSOR_EXTERNAL_2) &&
        touch_sensor.touched(SENSOR_EXTERNAL_2)) {
      sendBurst(poofer_address, POOFER1_POOF1, 25);
    }

    if (touch_sensor.changed(SENSOR_EXTERNAL_3) &&
        touch_sensor.touched(SENSOR_EXTERNAL_3)) {
      sendBurst(poofer_address, POOFER1_POOF2, 25);
    }

#endif
  } // END: Poofer controls

  /* Change display mode */
  if (touch_sensor.changed(SENSOR_LCD_LEFT) &&
      touch_sensor.touched(SENSOR_LCD_LEFT)) {
    lcd.clear();
    display_mode = (display_mode + 1) % NUM_DISPLAY_MODES;
  }

  /*
   * Display adjustments
   */
  if (display_mode == DISPLAY_ADJUST_LEFT1) {
    if (touch_sensor.changed(SENSOR_LCD_UP)) {
      if (touch_sensor.touched(SENSOR_LCD_UP)) {
        DEBUG1_PRINTLN("LEFT UP");
        pulse_bpm_1++;
        calculate_pulse();
      }
    }

    if (touch_sensor.changed(SENSOR_LCD_DOWN)) {
      if (touch_sensor.touched(SENSOR_LCD_DOWN)) {
        DEBUG1_PRINTLN("LEFT DOWN");
        pulse_bpm_1--;
        calculate_pulse();
      }
    }
  }

  if (display_mode == DISPLAY_ADJUST_LEFT2) {
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

  if (display_mode == DISPLAY_ADJUST_RIGHT1) {
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

  if (display_mode == DISPLAY_ADJUST_RIGHT2) {
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
        poofer_address++;
        if (poofer_address > 72) {
          poofer_address = 64;
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

void initialize_display() {
  lcd.begin(16, 2);

  lcd.setCursor(0, 0);
  lcd.print("Initializing");
  lcd.setBacklight(HIGH);
}


void update_lcd() {
  uint32_t now = millis();
  static uint32_t last_update = 0;

  switch (display_mode) {
    case 0: {
      if (data_changed) {
        lcd.setCursor(0, 0);
        lcd.print("C:");
        for (byte i = 0; i < MPR121::MAX_SENSORS; i++) {
          lcd.print(touch_sensor.touched(i));
        }
        lcd.print("    ");

        lcd.setCursor(0, 1);
        lcd.print("S:");
        for (byte i = 0; i < NUM_SWITCHES; i++) {
          lcd.print(switch_states[i]);
        }
        lcd.print("      ");
    
        data_changed = false;
      }
      break;
    }
    case DISPLAY_ADJUST_LEFT1:
    case DISPLAY_ADJUST_LEFT2:
    {
      lcd.setCursor(0, 0);
      lcd.print("LEFT BPM:");
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

    case DISPLAY_ADJUST_RIGHT1:
    case DISPLAY_ADJUST_RIGHT2:
    {
      lcd.setCursor(0, 0);
      lcd.print("RIGHT BPM:");
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
      lcd.print("FIRE_ADDR:");
      lcd.print(poofer_address);

      lcd.setCursor(0, 1);
      lcd.print("LIGHT_ADDR:");
      lcd.print(lights_address);
      break;
    }

  }
}

void update_poofers() {
}
