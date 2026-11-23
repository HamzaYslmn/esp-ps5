/* bluedroid.cpp - L2CAP transport for HID control (PSM 0x11) + interrupt (0x13).
 * No DualSense knowledge here: bytes in -> ps5_on_input(), bytes out <- ps5_l2cap_send(). */
#include "../ps5Controller.h"
#include "bluedroid.h"
#include <esp_log.h>

#define TAG "ps5"

static tL2CAP_CFG_INFO g_cfg;
static BD_ADDR  g_target;
static bool     g_hasTarget = false;
static volatile uint16_t g_ctrl = 0, g_intr = 0;      /* channel ids, 0 = closed */
static volatile bool g_ctrlCfg = false, g_intrCfg = false, g_up = false;

/* Recompute link state after any per-channel change; report only real edges.
 * Bluedroid reports each channel separately, so never wipe the other one. */
/* Page scan: 11.25 ms window every 40 ms (interlaced) so the pad's PS-press page lands fast.
 * Linked: non-connectable, the radio is all for the link. */
static void connectable(bool on) { BTM_SetConnectability(on, 0x12, 0x40); }

static void update() {
  bool up = g_ctrlCfg && g_intrCfg;
  if (up == g_up) return;
  g_up = up;
  connectable(!up);
  ps5_on_link(up);
}

static void onConnectInd(BD_ADDR bd, uint16_t cid, uint16_t psm, uint8_t id) {
  L2CA_ErtmConnectRsp(bd, id, cid, L2CAP_CONN_PENDING, L2CAP_CONN_PENDING, NULL);
  L2CA_ErtmConnectRsp(bd, id, cid, L2CAP_CONN_OK, L2CAP_CONN_OK, NULL);
  L2CA_ConfigReq(cid, &g_cfg);
  if (psm == BT_PSM_HIDC) g_ctrl = cid; else g_intr = cid;
}

static void onConnectCfm(uint16_t cid, uint16_t result) {
  if (result != L2CAP_CONN_OK && cid == g_ctrl) g_ctrl = 0;   /* failed outbound: allow retry */
}

static void onConfigInd(uint16_t cid, tL2CAP_CFG_INFO* cfg) {
  cfg->result = L2CAP_CFG_OK;
  L2CA_ConfigRsp(cid, cfg);
}

static void onConfigCfm(uint16_t cid, tL2CAP_CFG_INFO*) {
  if (cid == g_ctrl) g_ctrlCfg = true;
  if (cid == g_intr) g_intrCfg = true;
  update();
}

static void onDisconnectInd(uint16_t cid, bool ack) {
  if (cid == g_ctrl) { g_ctrl = 0; g_ctrlCfg = false; }
  if (cid == g_intr) { g_intr = 0; g_intrCfg = false; }
  if (ack) L2CA_DisconnectRsp(cid);
  update();
}

static void onDisconnectCfm(uint16_t, uint16_t) {}

static void onData(uint16_t, BT_HDR* b) {
  ps5_on_input(b->data + b->offset, b->len);
  osi_free_func(b);
}

static tL2CAP_APPL_INFO g_appl = {
  onConnectInd, onConnectCfm, NULL, onConfigInd, onConfigCfm,
  onDisconnectInd, onDisconnectCfm, NULL, onData, NULL, NULL, NULL
};

void ps5_l2cap_init() {
  BTM_SetPageScanType(1);                     /* interlaced */
  connectable(true);
  static const struct { const char* name; uint16_t psm; } svc[] = {
    {"ps5-HIDC", BT_PSM_HIDC}, {"ps5-HIDI", BT_PSM_HIDI}};
  for (int i = 0; i < 2; i++) {
    if (!L2CA_Register(svc[i].psm, &g_appl) ||
        !BTM_SetSecurityLevel(false, svc[i].name, BTM_SEC_SERVICE_FIRST_EMPTY + i, 0, svc[i].psm, 0, 0))
      ESP_LOGE(TAG, "register %s failed", svc[i].name);
  }
}

void ps5_l2cap_connect(const uint8_t* mac) {
  if (mac) { memcpy(g_target, mac, 6); g_hasTarget = true; }
  if (!g_hasTarget) return;
  g_ctrl = L2CA_ErtmConnectReq(BT_PSM_HIDC, g_target, NULL);
  ESP_LOGI(TAG, "connect %02x:%02x:%02x:%02x:%02x:%02x cid=%u",
           g_target[0], g_target[1], g_target[2], g_target[3], g_target[4], g_target[5], g_ctrl);
}

bool ps5_l2cap_up()         { return g_up; }
bool ps5_l2cap_idle()       { return !g_ctrl && !g_intr; }
bool ps5_l2cap_target(uint8_t* out) { if (out && g_hasTarget) memcpy(out, g_target, 6); return g_hasTarget; }
void ps5_l2cap_forget()     { g_hasTarget = false; }

void ps5_l2cap_send(bool control, const uint8_t* d, uint8_t len) {
  /* loop() (send) and the esp_timer task (kick, sound) both write: serialize them. */
  static StaticSemaphore_t mbuf;
  static SemaphoreHandle_t mtx = xSemaphoreCreateMutexStatic(&mbuf);
  uint16_t cid = control ? g_ctrl : g_intr;
  if (!cid) return;
  /* Header + L2CAP/HCI headroom + payload (~100-160 B, not a 4 KB default buffer). */
  BT_HDR* b = (BT_HDR*)osi_malloc_func(sizeof(BT_HDR) + L2CAP_MIN_OFFSET + len);
  if (!b) return;
  b->len = len;
  b->offset = L2CAP_MIN_OFFSET;
  memcpy(b->data + b->offset, d, len);
  xSemaphoreTake(mtx, portMAX_DELAY);
  uint8_t r = L2CA_DataWrite(cid, b);
  xSemaphoreGive(mtx);
  if (r == L2CAP_DW_FAILED) ESP_LOGW(TAG, "tx failed cid=%u", cid);
}
