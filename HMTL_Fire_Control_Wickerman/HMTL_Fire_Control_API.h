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
 *
 * That last clause survives the addition of OTA, which needs outputs driven
 * safe before it flashes.  The server task does NOT send those RS485 messages:
 * it raises a request and blocks on an acknowledgement, and loop() — the core
 * that owns the bus — performs the drive in fc_api_service().  The rule is
 * unchanged: nothing on the server task ever touches ignition state directly.
 ******************************************************************************/

#ifndef HMTL_FIRE_CONTROL_API_H
#define HMTL_FIRE_CONTROL_API_H

#ifdef ESP32

#include <stdint.h>

/* Start the WiFi/API task (call once, after initialize_switches()) */
void fc_api_setup();

/* Publish the loop-side status snapshot (call every loop()) */
void fc_api_publish();

/*
 * Service cross-core requests raised by the API task (call every loop(), on
 * core 1).  Currently that is the OTA safe-state request: when one is
 * outstanding this calls fc_all_outputs_safe() and acknowledges it.  A no-op
 * when nothing has asked, so it costs one relaxed read per iteration.
 */
void fc_api_service();

/*
 * Ask core 1 to drive every output safe and wait for it to confirm it did.
 * Called from the API task (core 0) only.  Returns false if the acknowledgement
 * does not arrive within timeout_ms, which means core 1 is not running its
 * loop; the caller must then ABORT whatever it was about to do.
 */
bool fc_request_safe_state(uint32_t timeout_ms);

/*
 * Age in milliseconds of the published status snapshot, and whether one has
 * ever been published.  This is how a cross-core reader distinguishes "the
 * controller is disarmed" from "core 1 has stopped telling us anything" — two
 * states that are indistinguishable from the snapshot's contents alone, and
 * whose conflation is exactly how a safety guard ends up permitting on fault.
 */
bool fc_status_snapshot_valid();
uint32_t fc_status_snapshot_age_ms();
bool fc_status_is_fresh(uint32_t max_age_ms);

#else

/* AVR builds carry no WiFi; the hooks compile away */
static inline void fc_api_setup() {}
static inline void fc_api_publish() {}
static inline void fc_api_service() {}

#endif // ESP32

#endif // HMTL_FIRE_CONTROL_API_H
