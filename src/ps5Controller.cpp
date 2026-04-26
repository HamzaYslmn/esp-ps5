/* ps5Controller.cpp - Arduino-friendly DualSense (PS5) controller for ESP32.
 *
 * Owns:
 *   - The global `ps5` instance + its user callbacks.
 *   - Bluedroid bring-up (BT controller, Bluedroid, GAP, SPP, L2CAP).
 *   - Scan-and-pair (linked-list dedupe, freed on connect).
 *   - Auto-reconnect (driven from isConnected()).
 *
 * Does NOT touch any DualSense bytes - see ps5_bytes.cpp.
 */

#include "ps5Controller.h"

#include <esp_bt.h>
#include <esp_bt_defs.h>
#include <esp_bt_main.h>
#include <esp_gap_bt_api.h>
#include <esp_log.h>
#include <nvs.h>
#include <nvs_flash.h>

#define ps5_TAG "ps5"
#define ps5_NVS_NS  "ps5"
#define ps5_NVS_KEY "mac"

#define ESP_BD_ADDR_HEX_PTR(a) \
  (uint8_t*)(a)+0,(uint8_t*)(a)+1,(uint8_t*)(a)+2, \
  (uint8_t*)(a)+3,(uint8_t*)(a)+4,(uint8_t*)(a)+5

/* ============================================================ link state */

static bool g_active = false;   /* true once first input packet has arrived */

/* ============================================================ NVS MAC store
 *
 * The DualSense pairs to whatever BT MAC last spoke "console" to it. Once a
 * given controller has bonded with this ESP32, its address never changes
 * (unless the user re-runs sixaxispairer against a different host). So we
 * stash the MAC in NVS the first time a link goes fully active, and on the
 * next boot ps5.begin() can skip the scan entirely and fire a direct
 * L2CAP connect to the known MAC. */

static bool nvsEnsure() {
  static bool done = false;
  if (done) return true;
  esp_err_t e = nvs_flash_init();
  if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    e = nvs_flash_init();
  }
  done = (e == ESP_OK);
  return done;
}

static bool macLoad(uint8_t out[6]) {
  if (!nvsEnsure()) return false;
  nvs_handle_t h;
  if (nvs_open(ps5_NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
  size_t sz = 6;
  esp_err_t e = nvs_get_blob(h, ps5_NVS_KEY, out, &sz);
  nvs_close(h);
  return (e == ESP_OK && sz == 6);
}

static void macSave(const uint8_t mac[6]) {
  if (!nvsEnsure()) return;
  nvs_handle_t h;
  if (nvs_open(ps5_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
  nvs_set_blob(h, ps5_NVS_KEY, mac, 6);
  nvs_commit(h);
  nvs_close(h);
}

static void macErase() {
  if (!nvsEnsure()) return;
  nvs_handle_t h;
  if (nvs_open(ps5_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
  nvs_erase_key(h, ps5_NVS_KEY);
  nvs_commit(h);
  nvs_close(h);
}

/* MARK: ps5ConnectEvent - C-side dispatcher.
 *   1 = link up   -> kick the SET_FEATURE handshake so the controller starts
 *                    streaming the BT 0x31 input report. (We don't fire the
 *                    user onConnect here - we wait for the first real packet
 *                    in parsePacket() so it's a true "alive" signal.)
 *   0 = link down -> drop the active flag and notify the user. */
extern "C" void ps5ConnectEvent(uint8_t up) {
  if (up) {
    ps5Enable();
  } else {
    g_active = false;
    ps5._fireConnState(false);
  }
}

/* Called from parsePacket() once per input report. First call doubles as the
 * real "controller is alive" moment - that's when we fire onConnect AND
 * persist the MAC to NVS so the next boot can skip the scan. */
extern "C" void ps5_mark_alive(void) {
  if (!g_active) {
    g_active = true;
    uint8_t mac[6];
    ps5_l2cap_get_target(mac);
    macSave(mac);
    ps5._fireConnState(true);
  }
  ps5._fireInput();
}

/* ============================================================ bring-up */

static bool ensureBT() {
  if (!btStarted() && !btStart())                                         { log_e("btStart failed"); return false; }
  if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED &&
      esp_bluedroid_init() != ESP_OK)                                     { log_e("bluedroid_init"); return false; }
  if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED &&
      esp_bluedroid_enable() != ESP_OK)                                   { log_e("bluedroid_enable"); return false; }
  return true;
}

static void ensureServices() {
  static bool up = false;
  if (up) return;
  sppInit();
  ps5_l2cap_init_services();
  up = true;
}

/* ============================================================ scan dedupe.
 *
 * During an inquiry every nearby device emits multiple ESP_BT_GAP_DISC_RES_EVT
 * (one per advertising packet seen). We only want to surface a unique device
 * to the user / auto-pair filter once.
 *
 * A singly-linked list grows during the scan (one node = 6 B MAC + 1 next
 * pointer = ~10 B). When a connection is established (or the next scan
 * begins), the list is freed. RAM usage is therefore proportional to the
 * actual count of distinct nearby BT devices, not a fixed cap. */

namespace {
  struct Seen { uint8_t mac[6]; Seen* next; };
  Seen* g_seen = nullptr;

  bool seenAddOrSkip(const uint8_t mac[6]) {
    for (Seen* n = g_seen; n; n = n->next)
      if (memcmp(n->mac, mac, 6) == 0) return true;
    Seen* n = (Seen*)malloc(sizeof(Seen));
    if (!n) return true;   /* OOM: pretend already seen so we don't re-fire */
    memcpy(n->mac, mac, 6); n->next = g_seen; g_seen = n;
    return false;
  }

  void seenClear() {
    while (g_seen) { Seen* n = g_seen->next; free(g_seen); g_seen = n; }
  }
}

/* Public so bluedroid.cpp can drop the cache the moment L2CAP comes up. */
extern "C" void ps5_scan_cache_release(void) { seenClear(); }

/* ============================================================ scan core */

namespace {
  ps5Controller::scan_cb_t gScanCb = nullptr;

  /* GAP discovery callback - resolves name/RSSI, dedupes, calls user cb. */
  void onDiscovery(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t* p) {
    if (event != ESP_BT_GAP_DISC_RES_EVT || !gScanCb) return;
    if (seenAddOrSkip(p->disc_res.bda)) return;

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
    log_i("scan: %02x:%02x:%02x:%02x:%02x:%02x  rssi=%d  name=\"%s\"",
          p->disc_res.bda[0], p->disc_res.bda[1], p->disc_res.bda[2],
          p->disc_res.bda[3], p->disc_res.bda[4], p->disc_res.bda[5],
          rssi, name[0] ? name : "?");
    gScanCb(p->disc_res.bda, name, rssi);
  }

  /* Run a BT inquiry. Calls cb per UNIQUE device. Optional early-exit hook
   * lets autoPair stop the moment a DualSense shows up. */
  bool runScan(uint8_t secs, ps5Controller::scan_cb_t cb, volatile bool* earlyExit) {
    if (!cb) return false;
    if (!ensureBT()) return false;
    seenClear();
    gScanCb = cb;
    esp_bt_gap_register_callback(onDiscovery);

    uint8_t units = (uint8_t)((secs * 100 + 127) / 128);   /* 1.28-s units */
    if (units < 1) units = 1;
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, units, 0);

    uint32_t deadline = millis() + (uint32_t)secs * 1000UL + 1500UL;
    while (millis() < deadline) {
      if (earlyExit && *earlyExit) break;
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    esp_bt_gap_cancel_discovery();
    gScanCb = nullptr;
    return true;
  }
}

bool ps5Controller::scanDevices(uint8_t secs, scan_cb_t cb) {
  bool ok = runScan(secs, cb, nullptr);
  seenClear();   /* user-driven scan: drop the dedupe list before returning */
  return ok;
}

/* ============================================================ auto-pair */

namespace {
  esp_bd_addr_t  gApMac    = {0};
  volatile bool  gApFound  = false;

  void onAutoPair(const uint8_t mac[6], const char* name, int8_t rssi) {
    if (gApFound || !name || !*name) return;
    if (!strstr(name, "DualSense") && !strstr(name, "Wireless Controller")) return;
    memcpy(gApMac, mac, 6);
    gApFound = true;
    log_i("auto-pair: %02x:%02x:%02x:%02x:%02x:%02x rssi=%d (%s)",
          mac[0],mac[1],mac[2],mac[3],mac[4],mac[5], rssi, name);
  }

  bool autoPair(uint8_t timeoutSecs) {
    if (timeoutSecs < 2) timeoutSecs = 2;
    gApFound = false;
    if (!runScan(timeoutSecs, onAutoPair, &gApFound)) return false;
    if (!gApFound) return false;
    ps5_l2cap_connect(gApMac);
    /* Don't free the seen list here - bluedroid will call
     * ps5_scan_cache_release() once L2CAP actually configures, so a failed
     * connect attempt can still skip already-seen MACs on the next retry. */
    return true;
  }
}

/* ============================================================ begin / connect */

bool ps5Controller::begin(uint8_t timeoutSecs) {
  if (!ensureBT()) return false;
  ensureServices();

  /* Fast path: a previously-paired controller's MAC is in NVS. Try it first
   * and give it a few seconds to come up before falling back to a full scan. */
  uint8_t saved[6];
  if (!ps5_l2cap_has_target() && macLoad(saved)) {
    log_i("ps5.begin(): trying saved MAC %02x:%02x:%02x:%02x:%02x:%02x",
          saved[0],saved[1],saved[2],saved[3],saved[4],saved[5]);
    ps5_l2cap_connect(saved);
    uint32_t deadline = millis() + 4000UL;
    while (millis() < deadline) {
      if (g_active) return true;
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    log_w("ps5.begin(): saved MAC didn't respond, falling back to scan");
  }

  if (!ps5_l2cap_has_target()) {
    log_i("ps5.begin(): scanning up to %us for a DualSense...", timeoutSecs);
    if (!autoPair(timeoutSecs))
      log_e("ps5.begin(): no DualSense found within %us. Will keep retrying via isConnected().", timeoutSecs);
  } else {
    ps5_l2cap_reconnect();
  }
  return true;
}

void ps5Controller::forget() {
  macErase();
  ps5_l2cap_clear_target();
}

bool ps5Controller::begin(const char* mac) {
  esp_bd_addr_t addr;
  if (sscanf(mac, ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX_PTR(addr)) != ESP_BD_ADDR_LEN) {
    log_e("Could not convert %s to a MAC address", mac); return false;
  }
  if (!ensureBT()) return false;
  ensureServices();
  ps5_l2cap_connect(addr);

  uint32_t deadline = millis() + 10000UL;
  while (millis() < deadline) {
    if (g_active) return true;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  return false;
}

bool ps5Controller::isConnected() {
  if (g_active) return true;
  static uint32_t tryAt = 0;
  if (millis() - tryAt > 5000UL) { tryAt = millis(); ps5_l2cap_reconnect(); }
  return false;
}

/* ============================================================ output (fluent) */

ps5Controller& ps5Controller::lightbar(uint8_t r, uint8_t g, uint8_t b) {
  output.r = r; output.g = g; output.b = b; return *this;
}
ps5Controller& ps5Controller::rumble(uint8_t small, uint8_t large) {
  output.smallRumble = small; output.largeRumble = large; return *this;
}
ps5Controller& ps5Controller::playerLed(uint8_t index, uint8_t value) {
  if (index < 1 || index > 5) return *this;
  uint8_t bit = (uint8_t)(1u << (index - 1));
  if (value == 0) {
    output.playerLeds &= (uint8_t)~bit;       // off; don't touch brightness
  } else {
    output.playerLeds |= bit;
    // API value 1=dim, 2=mid, 3+=bright. Wire is inverted: 0=bright,1=mid,2=dim.
    uint8_t wire = (value >= 3) ? 0 : (value == 2 ? 1 : 2);
    output.ledBrightness = wire;
  }
  return *this;
}
ps5Controller& ps5Controller::muteLed(uint8_t mode) {
  output.muteLed = mode; return *this;
}
ps5Controller& ps5Controller::send() {
  /* Don't write to L2CAP unless the controller is fully alive. Sending during
   * the half-connected window (or after a remote-initiated tear-down that we
   * haven't observed yet) crashes the BT stack on a stale CID. */
  if (g_active) ps5BuildAndSend();
  return *this;
}

/* ---- edge detection ------------------------------------------------------- */
bool* ps5Controller::_edgePrev(const bool& field) {
  const bool* addr = &field;
  /* find existing slot */
  for (auto& s : _edges) if (s.p == addr) return &s.prev;
  /* allocate first empty slot */
  for (auto& s : _edges) if (!s.p) { s.p = addr; s.prev = false; return &s.prev; }
  /* full: recycle slot 0 (rare; bump array size in .h if it matters) */
  _edges[0].p = addr; _edges[0].prev = false;
  return &_edges[0].prev;
}
bool ps5Controller::pressed(const bool& field) {
  bool* prev = _edgePrev(field);
  bool edge = (field && !*prev);
  *prev = field;
  return edge;
}
bool ps5Controller::released(const bool& field) {
  bool* prev = _edgePrev(field);
  bool edge = (!field && *prev);
  *prev = field;
  return edge;
}

/* ============================================================ adaptive triggers
 *
 * Public API takes percent (0..100). Hardware uses quantised zones:
 *   position : 0..9   (10 trigger zones)
 *   strength : 0..8   (9 force levels; 0 = effect off)
 *
 *   trigger block is: [mode][p0..p9].
 *
 *   Off       (0x05) : all params 0.
 *   Feedback  (0x21) : "rigid wall" past `position`. Active across zones
 *                       [position..9]. p1..p2 = uint16 LE active-zone bitmask;
 *                       p3..p6 = uint32 LE force-zone packed 3-bit values
 *                       (((strength-1)&7) << (3*i)) per active zone.
 *   Weapon    (0x25) : "trigger break" between start..end (start in 2..7,
 *                       end in start+1..8). p1..p2 = (1<<start)|(1<<end);
 *                       p3 = strength-1.
 *   Vibration (0x26) : pulses past `position`. Same active/strength packing
 *                       as Feedback but the strength bytes encode amplitude.
 *                       p9 = frequency in Hz (raw byte).
 *
 */

namespace {
inline uint8_t pctToPos     (uint8_t pct) { if (pct > 100) pct = 100; return (uint8_t)((pct * 9 + 50) / 100); }
inline uint8_t pctToStrength(uint8_t pct) { if (pct > 100) pct = 100; return (uint8_t)((pct * 8 + 50) / 100); }

inline void buildOff(uint8_t* m, uint8_t* p) {
  *m = 0x05; memset(p, 0, 10);
}
inline void buildFeedback(uint8_t* m, uint8_t* p, uint8_t startPct, uint8_t strengthPct) {
  uint8_t pos      = pctToPos(startPct);
  uint8_t strength = pctToStrength(strengthPct);   /* 0..8 */
  if (strength == 0) { buildOff(m, p); return; }
  *m = 0x21;
  memset(p, 0, 10);
  uint16_t active = 0;
  uint32_t zones  = 0;
  uint8_t  v      = (uint8_t)((strength - 1) & 0x07);
  for (int i = pos; i < 10; i++) {
    active |= (uint16_t)(1u << i);
    zones  |= (uint32_t)v << (3 * i);
  }
  p[0] = (uint8_t)(active     );
  p[1] = (uint8_t)(active >> 8);
  p[2] = (uint8_t)(zones      );
  p[3] = (uint8_t)(zones >>  8);
  p[4] = (uint8_t)(zones >> 16);
  p[5] = (uint8_t)(zones >> 24);
}
inline void buildWeapon(uint8_t* m, uint8_t* p, uint8_t startPct, uint8_t endPct, uint8_t strengthPct) {
  uint8_t a        = pctToPos(startPct);
  uint8_t b        = pctToPos(endPct);
  uint8_t strength = pctToStrength(strengthPct);
  if (a < 2) a = 2;
  if (a > 7) a = 7;
  if (b <= a) b = (uint8_t)(a + 1);
  if (b > 8)  b = 8;
  if (strength == 0) { buildOff(m, p); return; }
  *m = 0x25;
  memset(p, 0, 10);
  uint16_t mask = (uint16_t)((1u << a) | (1u << b));
  p[0] = (uint8_t)(mask     );
  p[1] = (uint8_t)(mask >> 8);
  p[2] = (uint8_t)(strength - 1);
}
inline void buildVibration(uint8_t* m, uint8_t* p, uint8_t startPct, uint8_t amplitudePct, uint8_t freqHz) {
  uint8_t pos       = pctToPos(startPct);
  uint8_t amplitude = pctToStrength(amplitudePct);
  if (amplitude == 0 || freqHz == 0) { buildOff(m, p); return; }
  *m = 0x26;
  memset(p, 0, 10);
  uint16_t active = 0;
  uint32_t zones  = 0;
  uint8_t  v      = (uint8_t)((amplitude - 1) & 0x07);
  for (int i = pos; i < 10; i++) {
    active |= (uint16_t)(1u << i);
    zones  |= (uint32_t)v << (3 * i);
  }
  p[0] = (uint8_t)(active     );
  p[1] = (uint8_t)(active >> 8);
  p[2] = (uint8_t)(zones      );
  p[3] = (uint8_t)(zones >>  8);
  p[4] = (uint8_t)(zones >> 16);
  p[5] = (uint8_t)(zones >> 24);
  p[8] = freqHz;             /* param byte index 8 == p9 in Nielk1's gist */
}

/* Bow (0x22): Weapon-style break with extra snap-back force.
 *   p1..p2 = (1<<startPos) | (1<<endPos), startPos 0..8, endPos start+1..8.
 *   p3..p4 = ((strength-1)&7) | ((snap-1)&7)<<3   (uint16 LE).
 *   strength,snap clamp 1..8; any zero / endPos==0 -> Off. */
inline void buildBow(uint8_t* m, uint8_t* p, uint8_t startPct, uint8_t endPct, uint8_t strengthPct, uint8_t snapPct) {
  uint8_t a    = pctToPos(startPct);
  uint8_t b    = pctToPos(endPct);
  uint8_t str  = pctToStrength(strengthPct);
  uint8_t snap = pctToStrength(snapPct);
  if (a > 8) a = 8;
  if (b > 8) b = 8;
  if (b <= a) b = (uint8_t)((a < 8) ? a + 1 : 8);
  if (str == 0 || snap == 0) { buildOff(m, p); return; }
  *m = 0x22;
  memset(p, 0, 10);
  uint16_t mask = (uint16_t)((1u << a) | (1u << b));
  uint16_t pair = (uint16_t)(((str - 1) & 0x07) | (((snap - 1) & 0x07) << 3));
  p[0] = (uint8_t)(mask     );
  p[1] = (uint8_t)(mask >> 8);
  p[2] = (uint8_t)(pair     );
  p[3] = (uint8_t)(pair >> 8);
}

/* Galloping (0x23): rhythmic pulse pattern between two zones.
 *   foot1/foot2 are positions inside the cycle, foot1<foot2.
 *   p1..p2 = (1<<startPos) | (1<<endPos).
 *   p3 = (foot2&7) | (foot1&7)<<3.   p4 = freqHz.
 *   freqHz==0 -> Off. */
inline void buildGalloping(uint8_t* m, uint8_t* p, uint8_t startPct, uint8_t endPct, uint8_t foot1Pct, uint8_t foot2Pct, uint8_t freqHz) {
  uint8_t a  = pctToPos(startPct);
  uint8_t b  = pctToPos(endPct);
  /* foot positions: foot1 0..6, foot2 foot1+1..7. Reuse pctToPos then clamp. */
  uint8_t f1 = pctToPos(foot1Pct);
  uint8_t f2 = pctToPos(foot2Pct);
  if (a > 8) a = 8;
  if (b > 8) b = 8;
  if (b <= a) b = (uint8_t)((a < 8) ? a + 1 : 8);
  if (f1 > 6) f1 = 6;
  if (f2 > 7) f2 = 7;
  if (f2 <= f1) f2 = (uint8_t)((f1 < 7) ? f1 + 1 : 7);
  if (freqHz == 0) { buildOff(m, p); return; }
  *m = 0x23;
  memset(p, 0, 10);
  uint16_t mask = (uint16_t)((1u << a) | (1u << b));
  p[0] = (uint8_t)(mask     );
  p[1] = (uint8_t)(mask >> 8);
  p[2] = (uint8_t)((f2 & 0x07) | ((f1 & 0x07) << 3));
  p[3] = freqHz;
}

/* Machine (0x27): bounded vibration oscillating between two amplitudes.
 *   p1..p2 = (1<<startPos) | (1<<endPos).
 *   p3 = (ampA&7) | (ampB&7)<<3   (NOTE: Nielk1 uses raw ampA/ampB 0..7, not -1).
 *   p4 = freqHz.   p5 = period (tenths of a second between A and B).
 *   freqHz==0 -> Off. */
inline void buildMachine(uint8_t* m, uint8_t* p, uint8_t startPct, uint8_t endPct, uint8_t ampAPct, uint8_t ampBPct, uint8_t freqHz, uint8_t periodTenths) {
  uint8_t a    = pctToPos(startPct);
  uint8_t b    = pctToPos(endPct);
  /* ampA/ampB encode raw 0..7 (no -1). 100% -> 7. */
  uint8_t ampA = (uint8_t)((ampAPct > 100 ? 100 : ampAPct) * 7 / 100);
  uint8_t ampB = (uint8_t)((ampBPct > 100 ? 100 : ampBPct) * 7 / 100);
  if (a > 8) a = 8;
  if (b <= a) b = (uint8_t)(a + 1);
  if (b > 9)  b = 9;
  if (freqHz == 0) { buildOff(m, p); return; }
  *m = 0x27;
  memset(p, 0, 10);
  uint16_t mask = (uint16_t)((1u << a) | (1u << b));
  p[0] = (uint8_t)(mask     );
  p[1] = (uint8_t)(mask >> 8);
  p[2] = (uint8_t)((ampA & 0x07) | ((ampB & 0x07) << 3));
  p[3] = freqHz;
  p[4] = periodTenths;
}
} // namespace

ps5Controller& ps5Controller::l2Off()                                                    { buildOff      (&output.leftTriggerMode, output.leftTriggerParam); return *this; }
ps5Controller& ps5Controller::l2Rigid  (uint8_t s, uint8_t f)                            { buildFeedback (&output.leftTriggerMode, output.leftTriggerParam, s, f); return *this; }
ps5Controller& ps5Controller::l2Trigger(uint8_t s, uint8_t e, uint8_t f)                 { buildWeapon   (&output.leftTriggerMode, output.leftTriggerParam, s, e, f); return *this; }
ps5Controller& ps5Controller::l2Pulse  (uint8_t s, uint8_t f, uint8_t hz)                { buildVibration(&output.leftTriggerMode, output.leftTriggerParam, s, f, hz); return *this; }
ps5Controller& ps5Controller::l2Bow      (uint8_t s, uint8_t e, uint8_t f, uint8_t sn)                                          { buildBow      (&output.leftTriggerMode,  output.leftTriggerParam,  s, e, f, sn); return *this; }
ps5Controller& ps5Controller::l2Galloping(uint8_t s, uint8_t e, uint8_t f1, uint8_t f2, uint8_t hz)                             { buildGalloping(&output.leftTriggerMode,  output.leftTriggerParam,  s, e, f1, f2, hz); return *this; }
ps5Controller& ps5Controller::l2Machine  (uint8_t s, uint8_t e, uint8_t aA, uint8_t aB, uint8_t hz, uint8_t per)                { buildMachine  (&output.leftTriggerMode,  output.leftTriggerParam,  s, e, aA, aB, hz, per); return *this; }
ps5Controller& ps5Controller::r2Off()                                                    { buildOff      (&output.rightTriggerMode, output.rightTriggerParam); return *this; }
ps5Controller& ps5Controller::r2Rigid  (uint8_t s, uint8_t f)                            { buildFeedback (&output.rightTriggerMode, output.rightTriggerParam, s, f); return *this; }
ps5Controller& ps5Controller::r2Trigger(uint8_t s, uint8_t e, uint8_t f)                 { buildWeapon   (&output.rightTriggerMode, output.rightTriggerParam, s, e, f); return *this; }
ps5Controller& ps5Controller::r2Pulse  (uint8_t s, uint8_t f, uint8_t hz)                { buildVibration(&output.rightTriggerMode, output.rightTriggerParam, s, f, hz); return *this; }
ps5Controller& ps5Controller::r2Bow      (uint8_t s, uint8_t e, uint8_t f, uint8_t sn)                                          { buildBow      (&output.rightTriggerMode, output.rightTriggerParam, s, e, f, sn); return *this; }
ps5Controller& ps5Controller::r2Galloping(uint8_t s, uint8_t e, uint8_t f1, uint8_t f2, uint8_t hz)                             { buildGalloping(&output.rightTriggerMode, output.rightTriggerParam, s, e, f1, f2, hz); return *this; }
ps5Controller& ps5Controller::r2Machine  (uint8_t s, uint8_t e, uint8_t aA, uint8_t aB, uint8_t hz, uint8_t per)                { buildMachine  (&output.rightTriggerMode, output.rightTriggerParam, s, e, aA, aB, hz, per); return *this; }

#if !defined(NO_GLOBAL_INSTANCES)
ps5Controller ps5;
#endif
