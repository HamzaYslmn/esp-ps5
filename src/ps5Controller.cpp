/* ps5Controller.cpp - Arduino-friendly DualSense (PS5) controller for ESP32.
 *
 * What this file does:
 *   - Owns the ps5_t/ps5_event_t/ps5_cmd_t state for the sketch.
 *   - Brings up Bluedroid + GAP + L2CAP HID transport via the helpers in
 *     bluedroid/bluedroid.cpp.
 *   - Routes connect/disconnect and per-packet input events to the sketch's
 *     attach*() callbacks.
 *   - Handles auto-pair (scan + first-match connect) and auto-reconnect.
 *
 * What it does NOT do:
 *   - Bluedroid plumbing (L2CAP / GAP / SPP). See bluedroid/bluedroid.cpp.
 *   - Build or parse any DualSense bytes. See ps5_bytes.cpp.
 */

#include "ps5Controller.h"

#include <esp_bt.h>
#include <esp_bt_defs.h>
#include <esp_bt_device.h>
#include <esp_bt_main.h>
#include <esp_gap_bt_api.h>
#include <esp_log.h>

#define ps5_TAG "ps5"

/* ============================================================================
 * MARK: ARDUINO CLASS
 * ==========================================================================*/

#define ESP_BD_ADDR_HEX_PTR(addr) \
  (uint8_t*)addr + 0, (uint8_t*)addr + 1, (uint8_t*)addr + 2, \
  (uint8_t*)addr + 3, (uint8_t*)addr + 4, (uint8_t*)addr + 5

/* Built in ps5_bytes.cpp; we expose a C++ caller here. */
extern "C" void ps5Cmd(ps5_cmd_t cmd);
extern "C" void ps5Enable(void);

/* ============================================================ callbacks
 *
 * We support exactly one ps5Controller instance at a time (the global `ps5`
 * unless NO_GLOBAL_INSTANCES is defined). The C-side parser hands every
 * packet to ps5PacketEvent() and the L2CAP layer hands connect/disconnect
 * to ps5ConnectEvent(). We forward both to the active instance. */

static ps5Controller* g_active = nullptr;
static bool           g_isActive = false;   /* true after first input packet */

extern "C" void ps5PacketEvent(ps5_t s, ps5_event_t e) {
  /* First packet doubles as "truly connected" — fire onConnect once. */
  if (!g_isActive) {
    g_isActive = true;
    if (g_active) ps5Controller::_connection_callback(g_active, 1);
  }
  if (g_active) ps5Controller::_event_callback(g_active, s, e);
}

extern "C" void ps5ConnectEvent(uint8_t isConnected) {
  if (isConnected) {
    ps5Enable();   /* tell controller to start streaming the BT 0x31 input */
  } else {
    g_isActive = false;
    if (g_active) ps5Controller::_connection_callback(g_active, 0);
  }
}

/* ============================================================ ps5Controller */

// MARK: ctor
ps5Controller::ps5Controller() {}

// MARK: begin() - bring up BT Classic + Bluedroid + register HID L2CAP listeners.
//   No args:  scan up to 30 s, connect to the FIRST DualSense detected (early-exit).
//   uint8_t:  same, custom timeout.
//   const char*: connect to that specific MAC.
// Either way, auto-reconnect runs every 5 s while disconnected (see isConnected()).
bool ps5Controller::begin() { return begin((uint8_t)30); }

bool ps5Controller::begin(uint8_t timeoutSecs) {
  g_active = this;

  if (!btStarted() && !btStart())                                         { log_e("btStart failed");           return false; }
  if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED &&
      esp_bluedroid_init() != ESP_OK)                                     { log_e("bluedroid_init failed");    return false; }
  if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED &&
      esp_bluedroid_enable() != ESP_OK)                                   { log_e("bluedroid_enable failed");  return false; }

  sppInit();
  ps5_l2cap_init_services();

  /* MARK: auto-pair - if no MAC stored, scan & connect to first DualSense seen. */
  if (!ps5_l2cap_has_target()) {
    log_i("ps5.begin(): scanning up to %us for a DualSense...", timeoutSecs);
    if (!autoPair(timeoutSecs)) {
      log_e("ps5.begin(): no DualSense found within %us. Will keep retrying via isConnected().", timeoutSecs);
    }
  } else {
    /* Re-kick reconnect on every begin() in case the radio was previously down. */
    ps5_l2cap_reconnect();
  }
  return true;
}

// MARK: begin(mac) - parse MAC, kick outbound connect, then begin().
bool ps5Controller::begin(const char* mac) {
  esp_bd_addr_t addr;
  if (sscanf(mac, ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX_PTR(addr)) != ESP_BD_ADDR_LEN) {
    log_e("Could not convert %s to a MAC address", mac);
    return false;
  }
  ps5_l2cap_connect(addr);
  return begin((uint8_t)30);
}

// MARK: end() - placeholder (no clean teardown yet).
void ps5Controller::end() {}

/* ============================================================ scanDevices */

namespace {
  ps5Controller::scan_cb_t gScanCb = nullptr;

  void onDiscovery(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t* p) {
    if (event != ESP_BT_GAP_DISC_RES_EVT || !gScanCb) return;

    char   name[32] = {0};
    int8_t rssi     = 0;
    for (int i = 0; i < p->disc_res.num_prop; i++) {
      esp_bt_gap_dev_prop_t* prop = &p->disc_res.prop[i];
      if (prop->type == ESP_BT_GAP_DEV_PROP_BDNAME && prop->len > 0) {
        size_t n = (size_t)prop->len < sizeof(name) - 1 ? (size_t)prop->len : sizeof(name) - 1;
        memcpy(name, prop->val, n);
      } else if (prop->type == ESP_BT_GAP_DEV_PROP_EIR) {
        uint8_t blen = 0;
        uint8_t* bn = esp_bt_gap_resolve_eir_data((uint8_t*)prop->val,
                       ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &blen);
        if (!bn) bn = esp_bt_gap_resolve_eir_data((uint8_t*)prop->val,
                       ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &blen);
        if (bn && blen) {
          size_t n = blen < sizeof(name) - 1 ? blen : sizeof(name) - 1;
          memcpy(name, bn, n);
        }
      } else if (prop->type == ESP_BT_GAP_DEV_PROP_RSSI) {
        rssi = *(int8_t*)prop->val;
      }
    }
    gScanCb(p->disc_res.bda, name, rssi);
  }
}

bool ps5Controller::scanDevices(uint8_t secs, scan_cb_t cb) {
  if (!cb) return false;
  if (!btStarted() && !btStart()) { log_e("btStart failed"); return false; }
  if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED &&
      esp_bluedroid_init() != ESP_OK) return false;
  if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED &&
      esp_bluedroid_enable() != ESP_OK) return false;

  gScanCb = cb;
  esp_bt_gap_register_callback(onDiscovery);

  uint8_t units = (uint8_t)((secs * 100 + 127) / 128);  /* 1.28-s units */
  if (units < 1) units = 1;
  esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, units, 0);

  uint32_t deadline = millis() + units * 1280UL + 1500UL;
  while (millis() < deadline) vTaskDelay(pdMS_TO_TICKS(50));
  esp_bt_gap_cancel_discovery();
  gScanCb = nullptr;
  return true;
}

// MARK: isConnected - true after first input packet; auto-retry every 5 s while down.
bool ps5Controller::isConnected() {
  if (g_isActive) return true;
  static unsigned long tryAt = 0;
  if (millis() - tryAt > 5000UL) {
    tryAt = millis();
    ps5_l2cap_reconnect();
  }
  return false;
}

/* ============================================================ auto-pair */

// MARK: autoPair - scan up to `timeoutSecs` and connect to the FIRST DualSense
// or Wireless Controller detected. Bails out as soon as one is seen; doesn't
// wait the full timeout. Used internally by begin().
namespace {
  esp_bd_addr_t gApMac    = {0};
  bool          gApFound  = false;

  void onAutoPair(const uint8_t mac[6], const char* name, int8_t rssi) {
    if (gApFound) return;
    if (!name || !*name) return;
    if (!strstr(name, "DualSense") && !strstr(name, "Wireless Controller")) return;
    memcpy(gApMac, mac, 6);
    gApFound = true;
    log_i("auto-pair: %02x:%02x:%02x:%02x:%02x:%02x rssi=%d (%s)",
          mac[0],mac[1],mac[2],mac[3],mac[4],mac[5], rssi, name);
  }
}

bool ps5Controller::autoPair(uint8_t timeoutSecs) {
  if (timeoutSecs < 2) timeoutSecs = 2;
  gApFound = false;

  if (!btStarted() && !btStart())                                       return false;
  if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED &&
      esp_bluedroid_init() != ESP_OK)                                   return false;
  if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED &&
      esp_bluedroid_enable() != ESP_OK)                                 return false;

  gScanCb = onAutoPair;
  esp_bt_gap_register_callback(onDiscovery);

  uint8_t units = (uint8_t)((timeoutSecs * 100 + 127) / 128);  /* 1.28-s units */
  if (units < 1) units = 1;
  esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, units, 0);

  /* Early-exit: stop polling the moment a DualSense shows up. */
  uint32_t deadline = millis() + (uint32_t)timeoutSecs * 1000UL;
  while (!gApFound && millis() < deadline) vTaskDelay(pdMS_TO_TICKS(100));
  esp_bt_gap_cancel_discovery();
  gScanCb = nullptr;

  if (!gApFound) return false;
  ps5_l2cap_connect(gApMac);
  return true;
}

/* ============================================================ output helpers */

// MARK: setLed - mutate local lightbar state. Sent on next sendToController().
void ps5Controller::setLed(uint8_t r, uint8_t g, uint8_t b) {
  output.r = r; output.g = g; output.b = b;
}

// MARK: setRumble - small=high-freq motor, large=low-freq motor (0..255 each).
void ps5Controller::setRumble(uint8_t small, uint8_t large) {
  output.smallRumble = small;
  output.largeRumble = large;
}

// MARK: setPlayerLeds - 5-bit mask, bit0=far-left ... bit4=far-right.
void ps5Controller::setPlayerLeds(uint8_t bitmask) {
  output.playerLeds = bitmask & 0x1F;
}

// MARK: setMuteLed - 0=off, 1=solid, 2=pulse.
void ps5Controller::setMuteLed(uint8_t mode) {
  output.muteLed = mode;
}

// MARK: sendToController - flush `output` as a real DualSense BT 0x31 frame.
void ps5Controller::sendToController() { ps5Cmd(output); }

/* ============================================================ sketch hooks */

void ps5Controller::attach(callback_t cb)            { _callback_event      = cb; }
void ps5Controller::attachOnConnect(callback_t cb)   { _callback_connect    = cb; }
void ps5Controller::attachOnDisconnect(callback_t cb){ _callback_disconnect = cb; }

// MARK: _event_callback - static trampoline; copy state into instance and fire user callback.
void ps5Controller::_event_callback(void* obj, ps5_t d, ps5_event_t e) {
  ps5Controller* self = (ps5Controller*)obj;
  self->data  = d;
  self->event = e;

  /* MARK: flat-api refresh - keep ps5.lx / ps5.cross / ps5.gyroX in sync. */
  self->lx = d.analog.stick.lx;   self->ly = d.analog.stick.ly;
  self->rx = d.analog.stick.rx;   self->ry = d.analog.stick.ry;
  self->l2 = d.analog.button.l2;  self->r2 = d.analog.button.r2;

  self->up    = d.button.up;      self->down  = d.button.down;
  self->left  = d.button.left;    self->right = d.button.right;
  self->cross = d.button.cross;   self->circle   = d.button.circle;
  self->square= d.button.square;  self->triangle = d.button.triangle;
  self->l1 = d.button.l1;  self->r1 = d.button.r1;
  self->l3 = d.button.l3;  self->r3 = d.button.r3;
  self->share = d.button.share;  self->options = d.button.options;
  self->ps_btn = d.button.ps;    self->touchpad = d.button.touchpad;
  self->mute   = d.button.mute;

  self->gyroX = d.sensor.gyro.x;   self->gyroY = d.sensor.gyro.y;   self->gyroZ = d.sensor.gyro.z;
  self->accelX= d.sensor.accel.x;  self->accelY= d.sensor.accel.y;  self->accelZ= d.sensor.accel.z;
  self->sensorTime = d.sensor.timestamp;

  self->battery      = d.status.battery;
  self->charging     = d.status.charging;
  self->fullyCharged = d.status.fully_charged;
  self->headphones   = d.status.headphones;
  self->micJack      = d.status.mic;

  if (self->_callback_event) self->_callback_event();
}

// MARK: _connection_callback - static trampoline. Runs on the Bluedroid thread
// for disconnects, and on the L2CAP RX thread (via ps5PacketEvent) for the
// first input packet. Must NOT block: the BT stack drives both paths.
void ps5Controller::_connection_callback(void* obj, uint8_t isConnected) {
  ps5Controller* self = (ps5Controller*)obj;
  if (isConnected) {
    if (self->_callback_connect) self->_callback_connect();
  } else {
    if (self->_callback_disconnect) self->_callback_disconnect();
  }
}

#if !defined(NO_GLOBAL_INSTANCES)
ps5Controller ps5;
#endif
