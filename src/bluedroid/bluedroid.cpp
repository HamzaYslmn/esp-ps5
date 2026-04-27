/* bluedroid.cpp - Bluedroid transport glue (L2CAP HID + GAP bring-up).
 *
 * This file is intentionally NOT about the DualSense protocol. Everything
 * here is generic Bluedroid plumbing: opening L2CAP listeners on the two
 * HID PSMs (control 0x11 + interrupt 0x13), shuttling bytes through, and
 * setting connectable scan mode. The PS5-specific code (parser, builder,
 * Arduino class) lives in ../ps5Controller.cpp and ../ps5_bytes.cpp.
 *
 * Why this is split out:
 *   - It uses Bluedroid-internal symbols (L2CA_*, BTM_*, osi_*, BT_HDR,
 *     BD_ADDR, ...). Those are declared by the hand-curated headers in
 *     this folder and resolved at link time against the precompiled
 *     Bluedroid blob shipped with Arduino-ESP32.
 *   - Keeping it isolated means a future port to NimBLE / a different
 *     stack only has to rewrite ONE file.
 *
 * The two functional groups in this file:
 *   1. L2CAP transport - register HIDC/HIDI PSMs, run the connect/config
 *      handshake, send/receive HID frames.
 *   2. GAP bring-up - device-name + connectable scan mode (handled by the
 *      Arduino-side ensureServices(); no SPP profile needed for L2CAP HID).
 *
 * The handful of C-linkage entry points used by ps5Controller.cpp are
 * declared in ../ps5Controller.h's extern "C" block.
 */

#include "../ps5Controller.h"
#include "bluedroid.h"

#include <esp_bt.h>
#include <esp_bt_defs.h>
#include <esp_bt_main.h>
#include <esp_gap_bt_api.h>
#include <esp_bt_main.h>
#include <esp_log.h>

#define ps5_TAG "ps5"

extern "C" {

/* ============================================================================
 * MARK: TRANSPORT - L2CAP HID
 *
 *   - Registers HID control (PSM 0x11) + interrupt (PSM 0x13) listeners.
 *   - Runs the L2CAP config handshake (controller -> us is fully connected
 *     after we get the *second* config-confirm, on the interrupt CID).
 *   - Sends fully-formed HID frames built by ps5_bytes.cpp.
 * ==========================================================================*/

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
static volatile bool is_connected            = false;
static BD_ADDR    g_bd_addr               = {0};
static uint16_t   l2cap_control_channel   = 0;
static uint16_t   l2cap_interrupt_channel = 0;

// MARK: l2cap_has_target - true once a MAC has been latched (via connect()).
bool ps5_l2cap_is_active(void) { return is_connected; }

// MARK: l2cap_has_any_cid - true while EITHER channel still holds a CID.
// Used by isConnected()'s reconnect guard so we never fire CONNECT_REQ on a
// half-alive link (one channel torn down, the other still configured).
bool ps5_l2cap_has_any_cid(void) {
    return (l2cap_control_channel != 0) || (l2cap_interrupt_channel != 0);
}

bool ps5_l2cap_has_target(void) {
    for (int i = 0; i < 6; i++) if (g_bd_addr[i]) return true;
    return false;
}

// MARK: l2cap_get_target - copy the latched MAC out (zeros if none).
void ps5_l2cap_get_target(uint8_t out[6]) {
    memcpy(out, g_bd_addr, 6);
}

// MARK: l2cap_clear_target - forget the saved MAC (used by ps5.forget()).
void ps5_l2cap_clear_target(void) {
    memset(g_bd_addr, 0, sizeof(g_bd_addr));
}

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

// MARK: config_cfm - both channels must configure for the connection to be live.
// Track each channel's configured state independently; only fire the up-edge
// once both are ready, regardless of which order the stack confirms them in.
extern void ps5_scan_cache_release(void);
static bool       l2cap_ctrl_configured = false;
static bool       l2cap_int_configured  = false;
static void ps5_l2cap_config_cfm_cback(uint16_t cid, tL2CAP_CFG_INFO* p_cfg) {
    (void)p_cfg;
    if      (cid == l2cap_control_channel)   l2cap_ctrl_configured = true;
    else if (cid == l2cap_interrupt_channel) l2cap_int_configured  = true;
    bool prev = is_connected;
    is_connected = l2cap_ctrl_configured && l2cap_int_configured;
    if (prev != is_connected) {
        if (is_connected) ps5_scan_cache_release();
        ps5ConnectEvent(is_connected ? 1 : 0);
    }
}

static void ps5_l2cap_disconnect_ind_cback(uint16_t cid, bool ack_needed) {
    /* Bluedroid fires disconnect_ind PER CHANNEL. A momentary blip on one
     * channel does NOT mean the other is dead - if we wipe both CIDs every
     * time, then on the partial reconnect that follows, only the affected
     * channel re-handshakes; the other's CID stays at 0 and send() silently
     * drops every frame even though RX still works (Bluedroid routes RX by
     * its own internal lookup). That's the "DOWN -> UP -> no TX" bug.
     * So: only clear the channel that actually disconnected, and only fire
     * ps5ConnectEvent(0) on the up->down edge. */
    bool prev_up = is_connected;
    if (cid == l2cap_control_channel)   { l2cap_control_channel   = 0; l2cap_ctrl_configured = false; }
    if (cid == l2cap_interrupt_channel) { l2cap_interrupt_channel = 0; l2cap_int_configured  = false; }
    is_connected = l2cap_ctrl_configured && l2cap_int_configured;
    if (ack_needed) L2CA_DisconnectRsp(cid);
    if (prev_up && !is_connected) ps5ConnectEvent(0);
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

} /* extern "C" */
