/* ps5Controller.cpp - Arduino-friendly DualSense (PS5) controller for ESP32.
 *
 * What this file does:
 *   - Owns the ps5_t/ps5_event_t/ps5_cmd_t state for the sketch.
 *   - Brings up Bluedroid + GAP + L2CAP HID transport (was ps5_spp.c + ps5_l2cap.c).
 *   - Routes connect/disconnect and per-packet input events to the sketch's
 *     attach*() callbacks.
 *
 * What it does NOT do:
 *   - Build or parse any DualSense bytes. That all lives in ps5_bytes.cpp.
 *
 * Bluedroid internal symbols (L2CA_*, BTM_*, osi_*, BT_HDR, BD_ADDR, ...) are
 * declared by the headers vendored under src/bluedroid/. Implementations live
 * inside the precompiled Bluedroid blob shipped with Arduino-ESP32. See
 * src/bluedroid/README.md for how/when to refresh those headers.
 */

#include "ps5Controller.h"

#include <esp_bt.h>
#include <esp_bt_defs.h>
#include <esp_bt_device.h>
#include <esp_bt_main.h>
#include <esp_gap_bt_api.h>
#include <esp_spp_api.h>
#include <esp_log.h>

#include "bluedroid/bluedroid.h"

#define ps5_TAG "ps5"

/* ============================================================================
 * MARK: TRANSPORT - L2CAP HID (was ps5_l2cap.c)
 *
 *   - Registers HID control (PSM 0x11) + interrupt (PSM 0x13) listeners.
 *   - Handles the L2CAP config handshake (controller -> us is fully connected
 *     after we get the *second* config-confirm, which is on the interrupt CID).
 *   - Sends fully-formed HID frames built by ps5_bytes.cpp.
 * The functions exposed back to ps5_bytes.cpp / the class are declared with
 * C linkage in ps5Controller.h's extern "C" block.
 * ==========================================================================*/

extern "C" {

/* L2CAP application-info struct - one set of callbacks shared by both PSMs. */
static void ps5_l2cap_connect_ind_cback (BD_ADDR bd_addr, uint16_t cid, uint16_t psm, uint8_t id);
static void ps5_l2cap_connect_cfm_cback (uint16_t cid, uint16_t result);
static void ps5_l2cap_config_ind_cback  (uint16_t cid, tL2CAP_CFG_INFO* p_cfg);
static void ps5_l2cap_config_cfm_cback  (uint16_t cid, tL2CAP_CFG_INFO* p_cfg);
static void ps5_l2cap_disconnect_ind_cback(uint16_t cid, bool ack_needed);
static void ps5_l2cap_disconnect_cfm_cback(uint16_t cid, uint16_t result);
static void ps5_l2cap_data_ind_cback    (uint16_t cid, BT_HDR* p_msg);
static void ps5_l2cap_congest_cback     (uint16_t cid, bool congested);

static const tL2CAP_APPL_INFO dyn_info = {
    ps5_l2cap_connect_ind_cback,
    ps5_l2cap_connect_cfm_cback,
    NULL,
    ps5_l2cap_config_ind_cback,
    ps5_l2cap_config_cfm_cback,
    ps5_l2cap_disconnect_ind_cback,
    ps5_l2cap_disconnect_cfm_cback,
    NULL,
    ps5_l2cap_data_ind_cback,
    ps5_l2cap_congest_cback,
    NULL
};

static tL2CAP_CFG_INFO ps5_cfg_info;
static bool       is_connected            = false;
static BD_ADDR    g_bd_addr               = {0};
static uint16_t   l2cap_control_channel   = 0;
static uint16_t   l2cap_interrupt_channel = 0;

// MARK: l2cap_init_service - register one PSM with L2CAP + Security Manager.
static void ps5_l2cap_init_service(const char* name, uint16_t psm, uint8_t security_id) {
    if (!L2CA_Register(psm, (tL2CAP_APPL_INFO*)&dyn_info)) {
        ESP_LOGE(ps5_TAG, "L2CA_Register %s failed", name); return;
    }
    if (!BTM_SetSecurityLevel(false, name, security_id, 0, psm, 0, 0)) {
        ESP_LOGE(ps5_TAG, "BTM_SetSecurityLevel %s failed", name); return;
    }
    ESP_LOGI(ps5_TAG, "Service %s up", name);
}

// MARK: l2cap_init_services - bring up HID control + interrupt PSMs.
void ps5_l2cap_init_services(void) {
    ps5_l2cap_init_service("ps5-HIDC", BT_PSM_HIDC, BTM_SEC_SERVICE_FIRST_EMPTY);
    ps5_l2cap_init_service("ps5-HIDI", BT_PSM_HIDI, BTM_SEC_SERVICE_FIRST_EMPTY + 1);
}

void ps5_l2cap_deinit_services(void) {
    L2CA_Deregister(BT_PSM_HIDC);
    L2CA_Deregister(BT_PSM_HIDI);
}

// MARK: l2cap_reconnect - retry the outbound HID-control L2CAP connect.
long ps5_l2cap_reconnect(void) {
    long ret = L2CA_CONNECT_REQ(BT_PSM_HIDC, g_bd_addr, NULL, NULL);
    ESP_LOGE(ps5_TAG, "L2CA_CONNECT_REQ ret=%ld", ret);
    if (ret == 0) return -1;
    l2cap_control_channel = (uint16_t)ret;
    return ret;
}

// MARK: l2cap_connect - remember target MAC and fire the first outbound CONNECT_REQ.
long ps5_l2cap_connect(BD_ADDR addr) {
    memmove(g_bd_addr, addr, sizeof(BD_ADDR));
    return ps5_l2cap_reconnect();
}

// MARK: l2cap_send - copy bytes into a Bluedroid BT_HDR and write on the given CID.
static void ps5_l2cap_send_on(uint16_t cid, hid_cmd_t* hid_cmd, uint8_t len) {
    if (cid == 0) { ESP_LOGE(ps5_TAG, "send: cid=0"); return; }
    BT_HDR* p_buf = (BT_HDR*)osi_malloc(BT_DEFAULT_BUFFER_SIZE);
    if (!p_buf) { ESP_LOGE(ps5_TAG, "send: osi_malloc failed"); return; }
    p_buf->length = len;
    p_buf->offset = L2CAP_MIN_OFFSET;
    memcpy((uint8_t*)(p_buf + 1) + p_buf->offset, hid_cmd->data, len);
    uint8_t r = L2CA_DataWrite(cid, p_buf);
    if      (r == L2CAP_DW_SUCCESS)   ESP_LOGD(ps5_TAG, "tx cid=0x%02x ok (%uB)", cid, len);
    else if (r == L2CAP_DW_CONGESTED) ESP_LOGW(ps5_TAG, "tx cid=0x%02x congested",  cid);
    else                              ESP_LOGE(ps5_TAG, "tx cid=0x%02x failed (%u)", cid, r);
}
void ps5_l2cap_send_hid          (hid_cmd_t* c, uint8_t len) { ps5_l2cap_send_on(l2cap_control_channel,   c, len); }
void ps5_l2cap_send_hid_interrupt(hid_cmd_t* c, uint8_t len) { ps5_l2cap_send_on(l2cap_interrupt_channel, c, len); }

/* ---- L2CAP callbacks ---- */

// MARK: connect_ind - inbound L2CAP connect from the controller; ack + start config.
static void ps5_l2cap_connect_ind_cback(BD_ADDR bd_addr, uint16_t cid, uint16_t psm, uint8_t id) {
    L2CA_CONNECT_RSP(bd_addr, id, cid, L2CAP_CONN_PENDING, L2CAP_CONN_PENDING, NULL, NULL);
    L2CA_CONNECT_RSP(bd_addr, id, cid, L2CAP_CONN_OK,      L2CAP_CONN_OK,      NULL, NULL);
    L2CA_CONFIG_REQ(cid, &ps5_cfg_info);
    if      (psm == BT_PSM_HIDC) l2cap_control_channel   = cid;
    else if (psm == BT_PSM_HIDI) l2cap_interrupt_channel = cid;
}

static void ps5_l2cap_connect_cfm_cback(uint16_t cid, uint16_t result) {
    ESP_LOGI(ps5_TAG, "connect_cfm cid=0x%02x result=%u", cid, result);
}

// MARK: config_ind - accept the controller's config request as-is.
static void ps5_l2cap_config_ind_cback(uint16_t cid, tL2CAP_CFG_INFO* p_cfg) {
    p_cfg->result = L2CAP_CFG_OK;
    L2CA_ConfigRsp(cid, p_cfg);
}

// MARK: config_cfm - second config-confirm = controller fully connected.
static void ps5_l2cap_config_cfm_cback(uint16_t cid, tL2CAP_CFG_INFO* p_cfg) {
    (void)p_cfg;
    bool prev = is_connected;
    is_connected = (cid == l2cap_interrupt_channel);
    if (prev != is_connected) ps5ConnectEvent(is_connected ? 1 : 0);
}

static void ps5_l2cap_disconnect_ind_cback(uint16_t cid, bool ack_needed) {
    is_connected = false;
    if (ack_needed) L2CA_DisconnectRsp(cid);
    ps5ConnectEvent(0);
}

static void ps5_l2cap_disconnect_cfm_cback(uint16_t cid, uint16_t result) {
    ESP_LOGI(ps5_TAG, "disconnect_cfm cid=0x%02x result=%u", cid, result);
}

// MARK: data_ind - inbound HID input report; hand to parsePacket().
static void ps5_l2cap_data_ind_cback(uint16_t cid, BT_HDR* p_buf) {
    (void)cid;
    if (p_buf->length > 2) {
        /* Real L2CAP payload starts at p_buf->offset; first byte is the
         * HIDP transaction header (0xA1 = DATA|INPUT) which parsePacket()
         * will skip itself. */
        parsePacket(p_buf->data + p_buf->offset);
    }
    osi_free(p_buf);
}

static void ps5_l2cap_congest_cback(uint16_t cid, bool congested) {
    ESP_LOGI(ps5_TAG, "congest cid=0x%02x %d", cid, congested);
}

/* ============================================================================
 * MARK: GAP/SPP - device-name + connectable scan-mode bring-up (was ps5_spp.c)
 * ==========================================================================*/

static void sppCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
    (void)param;
    if (event == ESP_SPP_INIT_EVT) {
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    }
}

void sppInit(void) {
    esp_err_t ret;
    if ((ret = esp_spp_register_callback(sppCallback)) != ESP_OK) {
        ESP_LOGE(ps5_TAG, "spp register failed: %s", esp_err_to_name(ret)); return;
    }
    if ((ret = esp_spp_init(ESP_SPP_MODE_CB)) != ESP_OK) {
        ESP_LOGE(ps5_TAG, "spp init failed: %s", esp_err_to_name(ret)); return;
    }
}

} /* extern "C" */

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
bool ps5Controller::begin() {
  g_active = this;

  if (!btStarted() && !btStart())                                         { log_e("btStart failed");           return false; }
  if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED &&
      esp_bluedroid_init() != ESP_OK)                                     { log_e("bluedroid_init failed");    return false; }
  if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED &&
      esp_bluedroid_enable() != ESP_OK)                                   { log_e("bluedroid_enable failed");  return false; }

  sppInit();
  ps5_l2cap_init_services();
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
  return begin();
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
