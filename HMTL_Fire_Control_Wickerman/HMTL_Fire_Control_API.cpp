/*******************************************************************************
 * Author: Adam Phelps
 * License: Create Commons Attribution-Non-Commercial
 * Copyright: 2026
 *
 * WiFi bring-up and REST status API, isolated on the non-loop() core.
 ******************************************************************************/

#ifdef ESP32

#ifdef DEBUG_LEVEL_API
  #define DEBUG_LEVEL DEBUG_LEVEL_API
#endif
#ifndef DEBUG_LEVEL
  #define DEBUG_LEVEL 2
#endif
#include <Arduino.h>
#include <Debug.h>
#include <WiFi.h>
#include <WiFiBase.h>
#include <Preferences.h>

#include "HMTL_Fire_Control.h"
#include "Fire_Control_Sensors.h"
#ifdef FC_SWITCHES_MCP23017
#include "fc_mcp_switches.h"
#endif
#include "HMTL_Fire_Control_API.h"
#include "fc_ota_guard.h"
#include "fc_safe_state.h"

#ifdef FC_OTA_ENABLE
#include <Update.h>
#include <esp_timer.h>
#endif

/*
 * Station credentials may come from an untracked header (preferred — never in
 * the binary of a shared build) or from -DFC_WIFI_SSID/-DFC_WIFI_PASS build
 * flags.  With neither, the controller runs AP-only until a network is
 * configured at runtime via the authenticated /network endpoint.
 *
 * The fallback AP's own identity is set the same two ways:
 *   -DFC_WIFI_AP_SSID  the entire AP name, replacing FIRE-CONTROL-<mac>
 *   -DFC_WIFI_AP_PASS  the entire AP/API password, replacing the derived one
 * Setting both is how you build an image whose AP credentials are known before
 * the board is ever powered on — see docs/fire-controller-esp32/API.md §3.
 */
#if __has_include("wifi_credentials.h")
#include "wifi_credentials.h"
#endif

/*
 * Snapshot shared between cores.  loop() (core 1) writes it via
 * fc_api_publish(); the server task (core 0) only reads.  Both sides copy the
 * whole struct under the spinlock so a reader can never observe a torn write.
 */
struct FcStatusSnapshot {
  bool armed;
  bool switches[FC_NUM_SWITCHES];
  /*
   * RAW switch states, before the arming interlock.  Carried alongside the
   * qualified ones because a guard asking "is anything physically active?"
   * must not use the qualified view: a switch closed since boot reads as
   * inactive there.  See fc_ota_guard.h.
   */
  bool switches_raw[FC_NUM_SWITCHES];
  /* The last switch read succeeded — positive evidence, not an assumption. */
  bool switches_read_ok;
  /* Cumulative failed expander transactions since boot.  Not a substitute for
   * switches_read_ok: a monotonic count says nothing on a single poll. */
  uint32_t switch_errors;
  uint32_t uptime_ms;
  /*
   * millis() at publish time.
   *
   * Deliberately a separate field from uptime_ms even though the two are equal
   * today.  uptime_ms is a value this device SERVES over HTTP and someone may
   * reasonably reformat it (seconds, a wall clock, a string); published_ms is a
   * safety input whose only job is to answer "how long since core 1 last spoke".
   * Tying a staleness interlock to a field that exists for a JSON response is
   * how that interlock silently stops working.
   */
  uint32_t published_ms;
  /* False until the first publish; distinguishes "no news" from "all clear". */
  bool valid;
};
static FcStatusSnapshot fc_snapshot;
static portMUX_TYPE fc_snapshot_mux = portMUX_INITIALIZER_UNLOCKED;

/*
 * Core 0 -> core 1 safe-state handshake.
 *
 * The counter logic itself lives in fc_safe_state.cpp, which knows nothing
 * about the ESP32: this file supplies only the platform services below.  That
 * split exists so the handshake is testable on the host -- it is what turns
 * "core 0 asked" into "core 1 actually ran fc_all_outputs_safe()", the OTA
 * guard sees only its boolean result, and an ack that satisfied the wrong
 * request would hand the guard a confident `true` for a drive that never
 * happened.  See test_safe_state.
 */
static portMUX_TYPE fc_safe_mux = portMUX_INITIALIZER_UNLOCKED;

static void fc_safe_lock(void *)   { portENTER_CRITICAL(&fc_safe_mux); }
static void fc_safe_unlock(void *) { portEXIT_CRITICAL(&fc_safe_mux); }
static uint32_t fc_safe_now(void *) { return (uint32_t)millis(); }
static void fc_safe_wait(void *, uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

static const fc_safe_state_ops_t fc_safe_ops = {
  fc_safe_lock, fc_safe_unlock, fc_safe_now, fc_safe_wait, nullptr
};
static fc_safe_state_t fc_safe_state = { 0, 0, &fc_safe_ops };

/* useStored=false: the constructor runs at static-init, before the WiFi driver
 * exists, so its WiFi.SSID() probe cannot be trusted. The task adds the
 * empty-SSID sentinel explicitly instead — that is what retries the SDK-stored
 * network (the one a runtime /network join persisted) at every boot. */
static WiFiBase wfb(false);

/* An ESP32 softAP SSID may be a full 32 bytes: WiFiAP.cpp's wifi_softap_config()
 * copies up to 32 into wifi_config.ap.ssid, so size for that rather than the 24
 * the MAC-derived default happened to need — FC_WIFI_AP_SSID exists so an
 * operator can name a board something meaningful, and 32 is the real ceiling.
 * The password stays 24 (8-23 usable), the same range /appass accepts, so a
 * build-flag password and a runtime-set one are interchangeable. */
static char fc_ap_ssid[33];
static char fc_ap_pass[24];

/*
 * Over-long build-flag values are a BUILD error, not a runtime truncation.
 * These flags exist so the credentials are known in advance; silently shipping
 * a truncated name or password would produce exactly the failure they are meant
 * to prevent — an operator holding credentials the board does not answer to,
 * discoverable only by reading serial off a board that may already be installed.
 * Failing the build puts the error in front of whoever typed the flag.
 * (The core is worse than truncating: wifi_softap_config() copies 32 bytes but
 * sets ssid_len = strlen(ssid), so a 40-char SSID yields an invalid ssid_len.)
 */
#ifdef FC_WIFI_AP_SSID
static_assert(sizeof(FC_WIFI_AP_SSID) <= sizeof(fc_ap_ssid),
              "FC_WIFI_AP_SSID is longer than 32 characters "
              "(the ESP32 softAP SSID limit)");
static_assert(sizeof(FC_WIFI_AP_SSID) > 1, "FC_WIFI_AP_SSID is empty");
#endif
#ifdef FC_WIFI_AP_PASS
static_assert(sizeof(FC_WIFI_AP_PASS) >= 9,
              "FC_WIFI_AP_PASS is shorter than WPA2's 8-character minimum");
static_assert(sizeof(FC_WIFI_AP_PASS) <= sizeof(fc_ap_pass),
              "FC_WIFI_AP_PASS is longer than 23 characters "
              "(the range /appass also accepts)");
#endif

/*
 * OTA is compiled in only with -DFC_OTA_ENABLE, and then a password is
 * MANDATORY.  This is the FC_WIFI_AP_* rule applied to a far sharper edge: an
 * unauthenticated firmware upload on a device that can light a fire is not a
 * weaker default, it is a remote code execution path into an ignition
 * controller.  There is deliberately no default, no derived-from-MAC fallback
 * and no "unset means disabled" — an unset password FAILS THE BUILD, in front
 * of whoever configured it, rather than silently shipping an image that either
 * cannot be updated or can be updated by anyone.
 */
#ifdef FC_OTA_ENABLE
  #ifndef FC_OTA_PASS
    #error "FC_OTA_ENABLE requires FC_OTA_PASS. Build with -DFC_OTA_PASS='\"<password>\"' or define it in the untracked wifi_credentials.h. OTA on an ignition controller is never unauthenticated."
    /* The build has already failed.  This placeholder exists only so the real
     * message above is not buried under a cascade of "FC_OTA_PASS was not
     * declared" errors from every later use of it — an error you have to scroll
     * to find is an error people learn to ignore.  It can never reach a binary:
     * #error is fatal to the build that produced it. */
    #define FC_OTA_PASS "unset-build-failed"
  #endif
static_assert(sizeof(FC_OTA_PASS) >= 9,
              "FC_OTA_PASS is shorter than 8 characters");
static_assert(sizeof(FC_OTA_PASS) > 1, "FC_OTA_PASS is empty");
#endif

/*
 * Publish loop()'s view of the controller for the API to serve.
 *
 * Called from loop() on core 1 every iteration: reads the armed state, each
 * switch state and uptime into a local struct, then copies that struct into
 * fc_snapshot under the spinlock.  The server task on core 0 (fc_handle_status)
 * copies it back out the same way, so a request is always answered from one
 * self-consistent set of values rather than fields sampled mid-update.
 *
 * The critical section deliberately covers only the struct copy — the sensor
 * reads happen outside it, so neither core is ever blocked for longer than a
 * few-word memcpy, and no HTTP work can delay the flame path.
 */
void fc_api_publish() {
  FcStatusSnapshot s;
  s.armed = fc_is_armed();
  for (uint8_t i = 0; i < FC_NUM_SWITCHES; i++) {
    s.switches[i] = fc_switch_state(i);
    s.switches_raw[i] = fc_switch_raw(i);
  }
  s.switches_read_ok = fc_switches_read_ok();
#ifdef FC_SWITCHES_MCP23017
  s.switch_errors = fc_mcp_switch_errors();
#else
  s.switch_errors = 0;
#endif
  s.uptime_ms = millis();
  s.published_ms = s.uptime_ms;
  s.valid = true;

  portENTER_CRITICAL(&fc_snapshot_mux);
  fc_snapshot = s;
  portEXIT_CRITICAL(&fc_snapshot_mux);
}

/* Copy the snapshot out; the only sanctioned cross-core read. */
static FcStatusSnapshot fc_snapshot_read() {
  FcStatusSnapshot s;
  portENTER_CRITICAL(&fc_snapshot_mux);
  s = fc_snapshot;
  portEXIT_CRITICAL(&fc_snapshot_mux);
  return s;
}

bool fc_status_snapshot_valid() {
  return fc_snapshot_read().valid;
}

uint32_t fc_status_snapshot_age_ms() {
  FcStatusSnapshot s = fc_snapshot_read();
  if (!s.valid) {
    /* Never published: report the largest possible age rather than 0, so a
     * caller that forgets to test validity still fails closed. */
    return 0xFFFFFFFFUL;
  }
  /* Unsigned subtraction, so the 49.7-day millis() wrap costs nothing. */
  return (uint32_t)(millis() - s.published_ms);
}

bool fc_status_is_fresh(uint32_t max_age_ms) {
  return fc_status_snapshot_age_ms() <= max_age_ms;
}

/* The drive itself: RS485 sends, so core 1 only, and never under the lock. */
static void fc_safe_drive(void *) {
  DEBUG1_PRINTLN("API: safe-state requested; driving outputs safe");
  fc_all_outputs_safe();
}

/* Core 1: run any outstanding safe-state drive and acknowledge it. */
void fc_api_service() {
  fc_safe_state_service(&fc_safe_state, fc_safe_drive, nullptr);
}

/*
 * Core 0: ask for a safe state and wait for core 1 to confirm it.
 *
 * Returns false on timeout, which the caller must treat as a hard refusal —
 * core 1 not answering means either it is stopped or it is busy in a way we
 * cannot account for, and neither is a state to start rewriting flash in.
 * Nothing is latched: the next request starts clean, so a refused upload is
 * simply retried.
 */
bool fc_request_safe_state(uint32_t timeout_ms) {
  bool acked = fc_safe_state_request(&fc_safe_state, timeout_ms);
  if (!acked) {
    DEBUG1_PRINTLN("API: safe-state request TIMED OUT");
  }
  return acked;
}

static void fc_handle_status() {
  FcStatusSnapshot s = fc_snapshot_read();

  String response = "{\"armed\":";
  response += s.armed ? "true" : "false";
  response += ",\"switches\":[";
  for (uint8_t i = 0; i < FC_NUM_SWITCHES; i++) {
    if (i > 0) response += ",";
    response += s.switches[i] ? "true" : "false";
  }
  response += "],\"switches_raw\":[";
  for (uint8_t i = 0; i < FC_NUM_SWITCHES; i++) {
    if (i > 0) response += ",";
    response += s.switches_raw[i] ? "true" : "false";
  }
  /* Reported so an operator can see WHICH switch is blocking an upload without
   * having to attempt one, and see it for a switch that has been closed since
   * boot (which reads false in "switches" above). */
  response += "],\"switches_read_ok\":";
  response += s.switches_read_ok ? "true" : "false";
  response += ",\"switch_errors\":";
  response += s.switch_errors;
  response += ",\"uptime_ms\":";
  response += s.uptime_ms;
  response += ",\"wifi\":{\"connected\":";
  response += (WiFi.status() == WL_CONNECTED) ? "true" : "false";
  response += ",\"ssid\":\"";
  response += WiFi.SSID();
  response += "\",\"ip\":\"";
  response += WiFi.localIP().toString();
  response += "\",\"rssi\":";
  response += WiFi.RSSI();
  response += "}}";

  wfb.getServer()->send(200, "application/json", response);
}

/* Store a new AP/API password (>=8 chars, WPA2 floor). Persisted to NVS and
 * applied at the next boot so the live AP association is not yanked out from
 * under the client mid-request. */
static void fc_handle_appass() {
  if (!wfb.authorizeConfigRequest()) {
    return;
  }
  String pass = wfb.getServer()->arg("pass");
  if (pass.length() < 8 || pass.length() >= sizeof(fc_ap_pass)) {
    wfb.getServer()->send(400, "application/json",
                          "{\"error\":\"pass must be 8-23 chars\"}");
    return;
  }
  Preferences prefs;
  prefs.begin("fc_api", false);
  size_t wrote = prefs.putString("ap_pass", pass);
  prefs.end();
  if (wrote == 0) {
    wfb.getServer()->send(500, "application/json",
                          "{\"error\":\"NVS write failed\"}");
    return;
  }
  wfb.getServer()->send(200, "application/json",
                        "{\"stored\":true,\"applies\":\"next boot\"}");
}

#ifdef FC_OTA_ENABLE

/*
 * HTTP firmware upload.
 *
 * Transport is HTTP rather than espota because macOS Sequoia blocks espota's
 * UDP; see docs/fire-controller-esp32/OTA.md.  That makes this endpoint a
 * guarded surface WE own rather than a UI borrowed from WLED, so every guard
 * applies to it and a refusal must be a visible HTTP error — never a silent
 * timeout that an operator reads as a flaky network.
 *
 * We are deliberately stricter than WLED, which gates OTA on connectivity, two
 * enable flags and a PIN, and has no concept of output state at all.  Here:
 *   - ANY active switch refuses, not merely "armed";
 *   - refusal requires positive evidence of health, so a failed switch read or
 *     a stalled control loop refuses rather than permitting;
 *   - outputs are driven safe, by core 1, and confirmed, BEFORE flashing;
 *   - the guard is re-evaluated on every chunk, so a switch thrown mid-upload
 *     aborts the flash instead of being discovered after the reboot.
 */

/* Refusal state for the request in flight.  The upload handler runs before the
 * response handler, so a refusal is recorded here and sent from there — the
 * body is drained either way rather than half-closing the connection under a
 * client that is still POSTing. */
static int    fc_ota_status = 0;      /* 0 = nothing recorded yet */
static String fc_ota_message;
static bool   fc_ota_writing = false; /* Update.begin() succeeded */

/*
 * Idle bound on the upload (FC_OTA_UPLOAD_IDLE_TIMEOUT_MS).
 *
 * The stall happens BETWEEN chunks, inside the library's read loop, where no
 * handler of ours runs -- so a deadline checked in the upload handler can only
 * fire once data arrives, which is exactly when it is not needed.  Only socket
 * state or a reset can end that spin.
 *
 * Closing the socket from the callback DOES NOT WORK, and the reason is worth
 * recording so it is not attempted again:
 *
 *   - WiFiClient::stop() (WiFiClient.cpp:214) is only
 *     `clientSocketHandle = NULL; _rxBuffer = NULL; _connected = false;`
 *     It never close()s.  The fd dies with the LAST shared_ptr reference
 *     (~WiFiClientSocketHandle), and WebServer::client() hands out a copy, so
 *     stopping our copy takes the refcount 2 -> 1 and changes nothing the
 *     server can see.
 *   - connected() short-circuits on the server copy's own _connected, then
 *     recv(fd, &dummy, 0, MSG_DONTWAIT) on its own still-valid fd.  A silent
 *     but connected peer yields EWOULDBLOCK -> still true.
 *   - shutdown() is not a reliable substitute either: recv() with a zero length
 *     returns 0 WITHOUT setting errno, so connected() switches on a stale errno.
 *   - close()ing the fd behind the library's back does break the loop
 *     (lwip_ioctl(FIONREAD) fails -> _rxBuffer->failed() -> available() calls
 *     stop() on the server's own copy), but the handle's destructor then closes
 *     the same fd number a second time, which is a live hazard if it has been
 *     reused in between.
 *
 * So the bound recovers the DEVICE rather than the connection: abort the update
 * and reset.  That is well-defined here specifically because reaching this point
 * required the guards to pass AND core 1 to acknowledge safe state, so outputs
 * are already closed, and the boot-time safe drive closes them again on the way
 * up.  A stalled flash is a failed flash; rebooting out of it loses nothing.
 *
 * Armed only once Update.begin() has succeeded -- NOT on the refusal path.  A
 * refused upload drains its body through the same unbounded wait, but the system
 * may be mid-show with core 1 driving poofers (a refusal means a switch is
 * active), and a reset there would interrupt the show to fix a wedged API.  Core
 * 1 is unaffected by the wedge, so that trade is the wrong way round.  That case
 * is the same defect as every other endpoint on this server and belongs with it
 * -- see the stalled-HTTP-client task, which this does NOT close.
 *
 * Re-armed on every chunk, so it measures silence rather than total duration.
 */
static esp_timer_handle_t fc_ota_idle_timer = nullptr;

static void fc_ota_idle_cb(void *) {
  /* Runs on the esp_timer task while the API task is stuck in the read loop. */
  DEBUG1_PRINTLN("OTA: no data for the idle timeout -- aborting and resetting");
  Update.abort();
  ESP.restart();
}

static void fc_ota_idle_arm() {
  if (fc_ota_idle_timer == nullptr) {
    const esp_timer_create_args_t args = {
      .callback = &fc_ota_idle_cb,
      .arg = nullptr,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "fc_ota_idle",
      .skip_unhandled_events = true,
    };
    if (esp_timer_create(&args, &fc_ota_idle_timer) != ESP_OK) {
      fc_ota_idle_timer = nullptr;
      DEBUG1_PRINTLN("OTA: could not create the idle timer");
      return;
    }
  }
  esp_timer_stop(fc_ota_idle_timer);   /* not-running is not an error here */
  esp_timer_start_once(fc_ota_idle_timer,
                       (uint64_t)FC_OTA_UPLOAD_IDLE_TIMEOUT_MS * 1000ULL);
}

static void fc_ota_idle_disarm() {
  if (fc_ota_idle_timer != nullptr) {
    esp_timer_stop(fc_ota_idle_timer);
  }
}

static void fc_ota_refuse(int status, const String &reason) {
  /* First refusal wins: it is the one that describes the actual cause. */
  if (fc_ota_status == 0 || fc_ota_status == 200) {
    fc_ota_status = status;
    fc_ota_message = reason;
  }
  if (fc_ota_writing) {
    Update.abort();
    fc_ota_writing = false;
  }
  DEBUG1_VALUE("OTA refused ", status);
  DEBUG1_VALUELN(": ", reason);
}

/* Fill the guard's input from the cross-core snapshot.  Nothing here reads the
 * sensor globals directly — they have no synchronisation and belong to core 1. */
static fc_ota_guard_input_t fc_ota_guard_input() {
  FcStatusSnapshot s = fc_snapshot_read();
  fc_ota_guard_input_t in;
  for (uint8_t i = 0; i < FC_NUM_SWITCHES; i++) {
    /* RAW, not the interlock-qualified state — see fc_ota_guard.h. */
    in.switch_active[i] = s.switches_raw[i];
  }
  in.switch_read_ok  = s.switches_read_ok;
  in.snapshot_valid  = s.valid;
  in.snapshot_age_ms = s.valid ? (uint32_t)(millis() - s.published_ms)
                               : 0xFFFFFFFFUL;
  return in;
}

static String fc_ota_reason(fc_ota_verdict_t verdict, int8_t blocking_switch) {
  String reason = fc_ota_verdict_str(verdict);
  if (verdict == FC_OTA_REFUSE_SWITCH_ACTIVE && blocking_switch >= 0) {
    reason += " (switch ";
    reason += blocking_switch;
    reason += ")";
  }
  return reason;
}

/* Evaluate the controller-state guard; returns true when it permits. */
static bool fc_ota_check_state() {
  int8_t blocking = -1;
  fc_ota_verdict_t verdict =
      fc_ota_evaluate(fc_ota_guard_input(), FC_OTA_MAX_SNAPSHOT_AGE_MS,
                      &blocking);
  if (verdict != FC_OTA_PERMIT) {
    fc_ota_refuse(409, fc_ota_reason(verdict, blocking));
    return false;
  }
  return true;
}

static void fc_handle_update_upload() {
  WebServer *server = wfb.getServer();

  /*
   * This callback is registered as the UPLOAD handler, but the library also
   * invokes it as the RAW handler: FunctionRequestHandler::canRaw() returns
   * true whenever an upload fn is registered and the method is not GET
   * (detail/RequestHandlersImpl.h:39-44), and raw() calls the same _ufn()
   * (:62-67).  _parseRequest routes there for any POST that is NOT multipart
   * (Parsing.cpp:176, `if (!isForm && ... canRaw(...))`).
   *
   * On that path _currentUpload is null, and WebServer::upload() is a bare
   * `return *_currentUpload` with no check (WebServer.h:113) -- so the line
   * below used to panic with LoadProhibited at EXCVADDR 0x00000000.  Verified
   * on hardware 2026-08-20: a single `curl -X POST http://<ip>/update` with no
   * body rebooted the controller, UNAUTHENTICATED -- the crash happens before
   * the auth check inside the handler can run.
   *
   * There is no public way to ask whether _currentUpload is set (it is
   * protected, and WiFiBase owns the server), so the discriminator is the
   * request itself: only a multipart POST is a real upload.
   */
  if (!server->hasHeader(F("Content-Type")) ||
      server->header(F("Content-Type")).indexOf(F("multipart/form-data")) < 0) {
    DEBUG1_PRINTLN("OTA: non-multipart POST to /update, ignoring");
    return;
  }

  HTTPUpload &upload = server->upload();

  switch (upload.status) {
    case UPLOAD_FILE_START: {
      fc_ota_status = 0;
      fc_ota_message = "";
      fc_ota_writing = false;

      /* Its own credential, not the AP/API password: the AP password is handed
       * out to anyone who needs to read /status, and reading status must not
       * confer the ability to replace the firmware. */
      if (!server->authenticate("ota", FC_OTA_PASS)) {
        fc_ota_refuse(401, "authentication required");
        return;
      }

      if (!fc_ota_check_state()) {
        return;
      }

      /*
       * Ask core 1 to close every output and WAIT for it to confirm it did.
       * The API task must not send RS485 itself (HMTL_Fire_Control_API.h), and
       * "assume it worked" would defeat the point of asking.
       */
      bool acked = fc_request_safe_state(FC_OTA_SAFE_STATE_TIMEOUT_MS);

      /*
       * Re-evaluate against a FRESH snapshot, now including the ack.  The
       * handshake takes real time, so this also catches a switch thrown while
       * we were waiting — and it is what turns a missing ack into a refusal
       * rather than something noticed too late.
       */
      int8_t blocking = -1;
      fc_ota_verdict_t verdict =
          fc_ota_evaluate_begin(fc_ota_guard_input(),
                                FC_OTA_MAX_SNAPSHOT_AGE_MS, acked, &blocking);
      if (verdict != FC_OTA_PERMIT) {
        fc_ota_refuse(409, fc_ota_reason(verdict, blocking));
        return;
      }

      DEBUG1_VALUELN("OTA: accepting image ", upload.filename);
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        fc_ota_refuse(500, "could not start the update");
        return;
      }
      fc_ota_writing = true;
      /* Only now: the guards passed and core 1 acked safe state, which is what
       * makes a reset out of a stalled flash a safe recovery. */
      fc_ota_idle_arm();
      break;
    }

    case UPLOAD_FILE_WRITE: {
      if (!fc_ota_writing) {
        /* Already refused; let the body drain so the client gets our status.
         * No timer on this path -- see the note above fc_ota_idle_cb. */
        return;
      }
      fc_ota_idle_arm();   /* progress */
      /* An upload runs for seconds.  Re-checking per chunk is what makes "a
       * switch blocks OTA" true for the whole flash rather than only for the
       * instant it started. */
      if (!fc_ota_check_state()) {
        return;
      }
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        fc_ota_refuse(500, "flash write failed");
      }
      break;
    }

    case UPLOAD_FILE_END: {
      /* Before the early return: a refused upload reaches here too, and leaving
       * the timer armed past the request is the failure mode this whole block
       * exists to avoid. */
      fc_ota_idle_disarm();
      if (!fc_ota_writing) {
        return;
      }
      if (!Update.end(true)) {
        fc_ota_idle_disarm();
        fc_ota_refuse(500, "image incomplete or invalid");
        return;
      }
      fc_ota_writing = false;
      fc_ota_idle_disarm();
      fc_ota_status = 200;
      fc_ota_message = "update accepted";
      DEBUG1_PRINTLN("OTA: image accepted");
      break;
    }

    case UPLOAD_FILE_ABORTED:
    default: {
      fc_ota_idle_disarm();
      /* No distinct status for a timeout abort: _parseFormUploadAborted()
       * (Parsing.cpp:549) returns false, so handleClient() never reaches
       * _handleRequest() and NOTHING recorded here is sent -- the pre-existing
       * 400 has the same fate. The operator's signal is the serial line in
       * fc_ota_idle_cb plus the reset itself, not an HTTP status. */
      fc_ota_refuse(400, "upload aborted");
      break;
    }
  }
}

static void fc_handle_update_done() {
  WebServer *server = wfb.getServer();

  /* Catch-all disarm. The response handler runs for every outcome, including
   * ones that never reach UPLOAD_FILE_END or _ABORTED. A timer left armed past
   * the request would fire later and stop whatever client happened to be
   * connected then -- a bug strictly worse than the one being fixed. */
  fc_ota_idle_disarm();
  int status = fc_ota_status ? fc_ota_status : 500;
  String message = fc_ota_message.length() ? fc_ota_message
                                           : String("no image received");

  String body = "{\"ok\":";
  body += (status == 200) ? "true" : "false";
  body += ",\"status\":\"";
  body += message;
  body += "\"}";

  server->sendHeader("Connection", "close");
  server->send(status, "application/json", body);

  if (status == 200) {
    /* Let the response reach the client before the reboot takes the socket. */
    DEBUG1_PRINTLN("OTA: rebooting into the new image");
    delay(200);
    ESP.restart();
  }
}

#endif // FC_OTA_ENABLE

/*
 * The server task owns all blocking WiFi work: startup() may wait a full
 * connect timeout per known network and handleClient() is unbounded against
 * an adversarial client — both acceptable here because a stall parks this
 * task, never loop().
 */
static void fc_api_task(void *arg) {
  /* Retry the SDK-stored network first: a runtime /network join persists there
   * and must survive a reboot. */
  wfb.addKnownNetwork("", "");
  wfb.configureAccessPoint(fc_ap_ssid, fc_ap_pass);
  /* The WiFiManager portal blocks indefinitely inside startup(); the simple
   * AP plus the authenticated /network endpoint is the runtime-config path. */
  wfb.useConfigPortal(false);
  wfb.setAuthPassword(fc_ap_pass);
  wfb.setConnectTimeoutMs(8000);
#if defined(FC_WIFI_SSID) && defined(FC_WIFI_PASS)
  wfb.addKnownNetwork(FC_WIFI_SSID, FC_WIFI_PASS);
#endif

  while (!wfb.startup()) {
    DEBUG1_PRINTLN("API: wifi startup failed, retrying");
    vTaskDelay(pdMS_TO_TICKS(30 * 1000));
  }

  WebServer *server = wfb.getServer();
  if (server) {
    /* The server's idle path costs a delay(1) per check by default; this task
     * paces itself with vTaskDelay instead. */
    server->enableDelay(false);
    wfb.addRESTEndpoint("/status", fc_handle_status,
                        "\"description\":\"controller status\"");
    wfb.addRESTEndpoint("/appass", fc_handle_appass,
                        "\"description\":\"set the AP/API password (applies next boot)\",\"args\":[\"pass\"]");
#ifdef FC_OTA_ENABLE
    /* Registered with server->on() rather than addRESTEndpoint(): a firmware
     * upload needs the two-handler form (response handler + upload handler),
     * which the single-handler REST helper cannot express. */
    /* header() only returns headers that were explicitly collected, and the
     * upload handler's multipart guard depends on Content-Type being one of
     * them -- without this the guard rejects every upload, including valid
     * ones. */
    static const char *fc_ota_headers[] = { "Content-Type" };
    server->collectHeaders(fc_ota_headers, 1);

    server->on("/update", HTTP_POST, fc_handle_update_done,
               fc_handle_update_upload);
    DEBUG1_PRINTLN("API: OTA enabled at POST /update");
#endif
  }

  DEBUG1_VALUE("API: up, AP=", fc_ap_ssid);
  DEBUG1_VALUELN(" IP=", WiFi.localIP().toString());

  uint32_t last_status = 0;
  for (;;) {
    wfb.checkServer();

#ifdef FC_OTA_ENABLE
    /*
     * The whole upload -- headers, body, every chunk -- happens inside ONE
     * handleClient() call, so once checkServer() returns the request is over
     * and the idle timer must not outlive it.
     *
     * This is a sweep rather than another disarm call site because the library
     * has exit paths that deliver NEITHER UPLOAD_FILE_ABORTED nor the response
     * handler -- Parsing.cpp:419 returns false on a post-arg overflow, for one --
     * and enumerating them is a losing game.  Anchoring the timer's lifetime to
     * "inside a checkServer() call" bounds it no matter which path the library
     * took.
     *
     * It matters more now than it looks: the callback resets the device, so a
     * timer left armed past its request is a SPURIOUS REBOOT of a fire
     * controller five minutes later, not a harmless no-op.
     */
    fc_ota_idle_disarm();
#endif

    if (millis() - last_status > 10 * 1000) {
      last_status = millis();
      DEBUG3_VALUE("API: wifi connected:", (WiFi.status() == WL_CONNECTED));
      DEBUG3_VALUE(" ssid:", WiFi.SSID());
      DEBUG3_VALUE(" ip:", WiFi.localIP().toString());
      DEBUG3_VALUELN(" rssi:", WiFi.RSSI());
    }
    /* Yield so the core 0 idle task feeds its watchdog */
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

/* The raw MAC is broadcast in every softAP beacon (the BSSID is derived from
 * it), so a password computable from the MAC alone would be readable off the
 * air.  Salting keeps that from being trivially derivable from the beacon: the
 * bar becomes "has the firmware image", and an operator-set password (NVS or
 * FC_WIFI_AP_PASS) removes it entirely.
 *
 * The salt is a FIXED constant on purpose.  It used to be "fc-ap-" __DATE__,
 * which meant the derived default changed whenever the image was rebuilt on a
 * different day: an operator who wrote the default down could not get onto the
 * AP after a reflash without serial access, which is exactly what you lack when
 * you need the AP.  A fixed salt makes the default stable per device and
 * forever — write it down once — and costs nothing in secrecy, because the salt
 * was always in the image either way.  Do not reintroduce __DATE__ (it also
 * made builds non-reproducible); bump the version suffix only if every device's
 * default deliberately needs to change. */
static uint32_t fc_default_pass_hash(uint64_t mac) {
  const char salt[] = "fc-ap-v1";
  uint32_t h = 2166136261UL;
  for (const char *c = salt; *c; c++) { h = (h ^ (uint8_t)*c) * 16777619UL; }
  for (int i = 0; i < 8; i++) { h = (h ^ (uint8_t)(mac >> (i * 8))) * 16777619UL; }
  return h;
}

/* The derived default: stable per device, printed at boot because it is the one
 * value nobody chose and so the only one the operator cannot look up. */
static void fc_set_derived_pass(uint64_t mac) {
  snprintf(fc_ap_pass, sizeof(fc_ap_pass), "fire-%08lx",
           (unsigned long)fc_default_pass_hash(mac));
}

void fc_api_setup() {
  uint64_t mac = ESP.getEfuseMac();

  /* AP name: the whole thing from FC_WIFI_AP_SSID when the build sets it,
   * otherwise the MAC-derived name, which stays the default so an unflagged
   * image is still unique on a bench with several boards on it. */
#ifdef FC_WIFI_AP_SSID
  strlcpy(fc_ap_ssid, FC_WIFI_AP_SSID, sizeof(fc_ap_ssid));
#else
  snprintf(fc_ap_ssid, sizeof(fc_ap_ssid), "FIRE-CONTROL-%04X",
           (unsigned)((mac >> 32) ^ (mac & 0xFFFF)) & 0xFFFF);
#endif

  /* Password precedence: runtime-set (NVS) > build flag > salted per-device
   * default. */
  bool derived_pass = false;
  Preferences prefs;
  prefs.begin("fc_api", true);
  String stored = prefs.getString("ap_pass", "");
  prefs.end();
  if (stored.length() >= 8) {
    strlcpy(fc_ap_pass, stored.c_str(), sizeof(fc_ap_pass));
  } else {
#ifdef FC_WIFI_AP_PASS
    strlcpy(fc_ap_pass, FC_WIFI_AP_PASS, sizeof(fc_ap_pass));
#else
    fc_set_derived_pass(mac);
    derived_pass = true;
#endif
  }
  /* A sub-8-char value would be refused by configureAccessPoint and leave the
   * controller with no AP and no server; fall back to the derived default
   * rather than running WiFi-dead until a reflash.  FC_WIFI_AP_PASS can no
   * longer land here (static_assert above), so in practice this catches a
   * short NVS value written by some other tool. */
  if (strlen(fc_ap_pass) < 8) {
    fc_set_derived_pass(mac);
    derived_pass = true;
    DEBUG1_PRINTLN("API: AP pass too short; using the derived default");
  }

  /* The AP name is ALWAYS printed: it is not a secret, and an operator who
   * cannot see the board still has to know what to look for on the air.  The
   * password is printed only when it is the derived default — a password that
   * was configured (build flag or /appass) is one the operator already has, so
   * echoing it on a production image would leak it for nothing.  The
   * <configured> marker still tells them which regime the board is in. */
  DEBUG1_VALUE("API: AP ssid=", fc_ap_ssid);
  if (derived_pass) {
    DEBUG1_VALUELN(" default pass=", fc_ap_pass);
  } else {
    DEBUG1_PRINTLN(" pass=<configured>");
  }

  fc_api_publish();

  xTaskCreatePinnedToCore(fc_api_task, "fc_api", 8192, nullptr,
                          1, nullptr, 0);
}

#endif // ESP32
