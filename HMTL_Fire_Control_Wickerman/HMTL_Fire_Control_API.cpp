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
  bool switches_read_ok;
  uint32_t switch_errors;
  uint32_t uptime_ms;
};
static FcStatusSnapshot fc_snapshot;
static portMUX_TYPE fc_snapshot_mux = portMUX_INITIALIZER_UNLOCKED;

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
  }
  s.switches_read_ok = fc_switches_read_ok();
#ifdef FC_SWITCHES_MCP23017
  s.switch_errors = fc_mcp_switch_errors();
#else
  s.switch_errors = 0;
#endif
  s.uptime_ms = millis();

  portENTER_CRITICAL(&fc_snapshot_mux);
  fc_snapshot = s;
  portEXIT_CRITICAL(&fc_snapshot_mux);
}

static void fc_handle_status() {
  FcStatusSnapshot s;
  portENTER_CRITICAL(&fc_snapshot_mux);
  s = fc_snapshot;
  portEXIT_CRITICAL(&fc_snapshot_mux);

  String response = "{\"armed\":";
  response += s.armed ? "true" : "false";
  response += ",\"switches\":[";
  for (uint8_t i = 0; i < FC_NUM_SWITCHES; i++) {
    if (i > 0) response += ",";
    response += s.switches[i] ? "true" : "false";
  }
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
  }

  DEBUG1_VALUE("API: up, AP=", fc_ap_ssid);
  DEBUG1_VALUELN(" IP=", WiFi.localIP().toString());

  uint32_t last_status = 0;
  for (;;) {
    wfb.checkServer();
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
