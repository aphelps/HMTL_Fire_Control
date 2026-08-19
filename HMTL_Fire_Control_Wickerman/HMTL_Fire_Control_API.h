/*******************************************************************************
 * Author: Adam Phelps
 * License: Create Commons Attribution-Non-Commercial
 * Copyright: 2026
 *
 * WiFi + REST status API for the ESP32 fire controller.
 *
 * The web server runs in a FreeRTOS task pinned to the other core, so HTTP —
 * including a stalled or adversarial client — can never delay the flame path
 * running in loop().  loop() publishes a status snapshot; the server task only
 * ever reads that snapshot, and never touches ignition state.
 ******************************************************************************/

#ifndef HMTL_FIRE_CONTROL_API_H
#define HMTL_FIRE_CONTROL_API_H

#ifdef ESP32

/* Start the WiFi/API task (call once, after initialize_switches()) */
void fc_api_setup();

/* Publish the loop-side status snapshot (call every loop()) */
void fc_api_publish();

#else

/* AVR builds carry no WiFi; the hooks compile away */
static inline void fc_api_setup() {}
static inline void fc_api_publish() {}

#endif // ESP32

#endif // HMTL_FIRE_CONTROL_API_H
