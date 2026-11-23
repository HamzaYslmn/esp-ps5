/* ps5Controller.cpp - BT bring-up, async scan/connect/reconnect, output scheduling, sound.
 * All DualSense byte work lives in ps5_bytes.cpp; L2CAP in bluedroid/.
 *
 * Threading: frames are sent from the Bluetooth task (link/input callbacks), never from
 * loop(): send() only marks output dirty. The esp_timer tick only scans/pages/kicks while
 * no input is flowing. Nothing here blocks. */
#include "ps5Controller.h"

#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_gap_bt_api.h>
#include <esp_timer.h>

/* Core >= 3.3.9 / 4.x free Classic-BT RAM in initArduino() unless a library
 * includes this header. Older cores keep it by default. */
#if __has_include("esp32-hal-alloc-bt-classic-mem.h")
#include "esp32-hal-alloc-bt-classic-mem.h"
#endif

static volatile bool g_active   = false;   /* input reports flowing */
static volatile bool g_scanning = false;
static volatile bool g_found    = false;
static volatile bool g_dirty    = false;   /* send() requested a frame */
static volatile bool g_playing  = false;
static volatile uint32_t g_lastInput = 0;  /* millis() of the last report */
static bool g_ledsReady = false;           /* pad's connect animation done (BT task only) */
static bool g_released  = false;           /* LEDs handed to firmware (BT task only) */
static esp_bd_addr_t g_foundMac;

/* ============================================================ sound (BT task, every 10.667 ms) */

static const ps5Controller::Note* g_notes;
static size_t   g_left;
static ps5Controller::Note g_one;            /* storage for tone() */
static uint32_t g_phase, g_step, g_noteLeft;  /* DDS phase, phase step, samples left in note */
static int8_t   g_amp;

static int8_t wave(uint32_t ph, int a) { return (ph & 0x80000000u) ? a : -a; }
static int8_t clip(int v) { return constrain(v, -127, 127); }

/* 32 stereo samples at 3 kHz per 0x32 report. Haptics mode disables firmware rumble,
 * so rumble is mixed in as low square waves (left 40 Hz, right 120 Hz). */
static void soundStep() {
  static int64_t next = 0;
  static uint32_t rl, rr;
  if (!g_playing) return;
  int64_t now = esp_timer_get_time();
  if (now < next) return;
  next = now - next > 50000 ? now + 10667 : next + 10667;   /* keep cadence, no burst after gaps */
  int8_t s[64] = {};
  int lA = ps5.output.largeRumble >> 1, rA = ps5.output.smallRumble >> 1;
  for (int i = 0; i < 64; i += 2) {
    s[i]     = wave(rl += 57266230u, lA);      /* 2^32 * 40 / 3000 */
    s[i + 1] = wave(rr += 171798692u, rA);     /* 2^32 * 120 / 3000 */
    while (!g_noteLeft && g_left) {
      g_step = (uint32_t)(((uint64_t)g_notes->hz << 32) / 3000);
      g_noteLeft = g_notes->ms * 3u;           /* 3 samples per ms */
      g_notes++; g_left--;
    }
    if (!g_noteLeft) { g_playing = false; break; }   /* rest of the frame stays silent */
    int8_t m = g_step ? wave(g_phase, g_amp) : 0;
    s[i] = clip(s[i] + m); s[i + 1] = clip(s[i + 1] + m);
    g_phase += g_step;
    g_noteLeft--;
  }
  ps5_send_haptics(s);
}

/* ============================================================ link events (BT task) */

void ps5_on_link(bool up) {
  g_ledsReady = g_released = false;
  if (up) {                                   /* first input report flips g_active */
    esp_bt_gap_cancel_discovery();            /* inbound reconnect may land mid-scan */
    ps5_send_output(false, false);            /* an output report starts the 0x31 stream */
    return;
  }
  g_playing = false;
  bool was = g_active;
  g_active = false;
  if (was && ps5.onDisconnect) ps5.onDisconnect();
}

void ps5_on_input(const uint8_t* p, uint16_t len) {
  /* First report: no edges, the user is usually still holding the pair gesture. */
  if (!ps5_parse(p, len, g_active)) return;
  uint32_t now = millis();
  g_lastInput = now;
  if (!g_active) { g_active = true; if (ps5.onConnect) ps5.onConnect(); }

  /* LEDs, SDL order: wait out the pad's connect animation (sensor clock >= 10200000,
   * ~3.4 s after power-on), send ONE reset-only frame, then colours. Same reset when
   * handing LEDs to / back from the firmware (releaseLeds). */
  if (!g_ledsReady && ps5.sensorTime >= 10200000) {
    ps5_send_reset(); g_ledsReady = true; g_dirty = true;
  }
  if (g_ledsReady && g_released != ps5.output.releaseLeds) {
    g_released = ps5.output.releaseLeds; ps5_send_reset(); g_dirty = true;
  }

  static uint32_t lastTx = 0;                 /* send() is rate-limited here to 10 ms */
  if (g_dirty && now - lastTx >= 10) {
    g_dirty = false; lastTx = now;
    ps5_send_output(g_ledsReady, g_ledsReady && !g_released);
  }
  soundStep();
  if (ps5.onPacket) ps5.onPacket();
}

/* ============================================================ async scan */

static bool isPad(const esp_bt_gap_cb_param_t* p) {
  for (int i = 0; i < p->disc_res.num_prop; i++) {
    const esp_bt_gap_dev_prop_t& pr = p->disc_res.prop[i];
    uint8_t* s = nullptr; uint8_t n = 0;
    if (pr.type == ESP_BT_GAP_DEV_PROP_BDNAME) { s = (uint8_t*)pr.val; n = pr.len; }
    else if (pr.type == ESP_BT_GAP_DEV_PROP_EIR) {
      s = esp_bt_gap_resolve_eir_data((uint8_t*)pr.val, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &n);
      if (!s) s = esp_bt_gap_resolve_eir_data((uint8_t*)pr.val, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &n);
    }
    if (!s || !n) continue;
    char name[32] = {};
    memcpy(name, s, n < sizeof(name) ? n : sizeof(name) - 1);
    if (strstr(name, "DualSense") || strstr(name, "Wireless Controller")) return true;
  }
  return false;
}

static void onGap(esp_bt_gap_cb_event_t e, esp_bt_gap_cb_param_t* p) {
  if (e == ESP_BT_GAP_DISC_RES_EVT && !g_found && isPad(p)) {
    memcpy(g_foundMac, p->disc_res.bda, 6);
    g_found = true;
    esp_bt_gap_cancel_discovery();            /* connect once inquiry has stopped */
  } else if (e == ESP_BT_GAP_DISC_STATE_CHANGED_EVT &&
             p->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
    g_scanning = false;
    if (g_found) { g_found = false; if (ps5_l2cap_idle()) ps5_l2cap_connect(g_foundMac); }
  }
}

static void startScan() {
  if (g_scanning) return;
  /* 5 s bursts: an inquiry starves page scan, which inbound reconnects need. */
  g_scanning = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 4, 0) == ESP_OK;
}

/* ============================================================ background tick (esp_timer, 400 ms) */

static void tick(void*) {
  static uint8_t n = 0;
  if (g_active) {                             /* stalled stream (no report for 1 s): kick */
    if (millis() - g_lastInput > 1000) ps5_send_output(false, false);
    return;
  }
  if (ps5_l2cap_up()) { ps5_send_output(false, false); return; }   /* up, no input yet: kick */
  if (g_scanning || !ps5_l2cap_idle() || ++n < 12) return;          /* retry every ~5 s */
  n = 0;
  /* Alternate: page the known pad / inquire (finds a pad in pairing mode, e.g. re-pair). */
  static bool page = false;
  if (ps5_l2cap_target(nullptr) && (page = !page)) ps5_l2cap_connect(nullptr);
  else                                             startScan();
}

static bool bringUp() {
  static bool done = false;
  if (done) return true;
  if (!btStarted() && !btStart()) { log_e("btStart failed"); return false; }
  if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED && esp_bluedroid_init() != ESP_OK) return false;
  if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED && esp_bluedroid_enable() != ESP_OK) return false;
  esp_bt_gap_register_callback(onGap);
  esp_bt_gap_set_page_timeout(0x0800);        /* 1.28 s: paging a sleeping pad can't hog the radio */
  ps5_l2cap_init();                           /* also sets fast page scan */
  const esp_timer_create_args_t args = { .callback = tick, .name = "ps5" };
  esp_timer_handle_t t;
  if (esp_timer_create(&args, &t) != ESP_OK || esp_timer_start_periodic(t, 400000) != ESP_OK) return false;
  return done = true;
}

/* ============================================================ public API */

bool ps5Controller::begin() {
  if (!bringUp()) return false;
  /* Already paired (link key in Bluedroid NVS)? Page it directly, no inquiry. */
  int n = 1; esp_bd_addr_t bonded;
  if (!ps5_l2cap_target(nullptr) && esp_bt_gap_get_bond_device_list(&n, &bonded) == ESP_OK && n > 0)
    ps5_l2cap_connect(bonded);
  else if (ps5_l2cap_target(nullptr)) ps5_l2cap_connect(nullptr);
  else startScan();
  return true;
}

bool ps5Controller::begin(const char* mac) {
  uint8_t a[6];
  if (sscanf(mac, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5]) != 6) {
    log_e("bad MAC %s", mac); return false;
  }
  if (!bringUp()) return false;
  ps5_l2cap_connect(a);
  return true;
}

bool ps5Controller::isConnected() const { return g_active; }
void ps5Controller::forget() {                /* unpair: drop target + link key */
  uint8_t mac[6];
  if (ps5_l2cap_target(mac)) esp_bt_gap_remove_bond_device(mac);
  ps5_l2cap_forget();
}

ps5Controller& ps5Controller::lightbar(uint8_t r, uint8_t g, uint8_t b) { output.r = r; output.g = g; output.b = b; return *this; }
ps5Controller& ps5Controller::rumble(uint8_t s, uint8_t l) { output.smallRumble = s; output.largeRumble = l; return *this; }
ps5Controller& ps5Controller::muteLed(uint8_t m)     { output.muteLed = m; return *this; }
ps5Controller& ps5Controller::micMute(bool on)       { output.micMute = on; return *this; }
ps5Controller& ps5Controller::releaseLeds(bool on)   { output.releaseLeds = on; return *this; }
ps5Controller& ps5Controller::send()                 { g_dirty = true; return *this; }

void ps5Controller::play(const Note* notes, size_t count, uint8_t volume) {
  g_playing = false;
  if (!notes || !count) return;
  g_notes = notes; g_left = count; g_noteLeft = 0; g_phase = 0;
  g_amp = min<uint8_t>(volume, 100) * 127 / 100;
  g_playing = true;                           /* BT task picks it up on the next report */
}
void ps5Controller::tone(uint16_t hz, uint16_t ms, uint8_t volume) { g_playing = false; g_one = {hz, ms}; play(&g_one, 1, volume); }
void ps5Controller::stopSound()       { g_playing = false; }
bool ps5Controller::isPlaying() const { return g_playing; }

ps5Controller& ps5Controller::playerLed(uint8_t index, uint8_t value) {
  if (index < 1 || index > 5) return *this;
  uint8_t bit = 1u << (index - 1);
  if (!value) { output.playerLeds &= ~bit; return *this; }
  output.playerLeds |= bit;
  output.ledBrightness = value >= 3 ? 0 : value == 2 ? 1 : 2;   /* wire is inverted */
  return *this;
}

/* ============================================================ adaptive triggers
 * Block = [mode][p0..p9]. Zones: position 0..9, strength 1..8 (0 = off).
 *   0x05 Off, 0x21 Feedback, 0x25 Weapon, 0x26 Vibration,
 *   0x22 Bow, 0x23 Galloping, 0x27 Machine (Nielk1 spec, see AGENTS.md). */
namespace {
uint8_t pos(uint8_t pct) { return (min<uint8_t>(pct, 100) * 9 + 50) / 100; }
uint8_t str(uint8_t pct) { return (min<uint8_t>(pct, 100) * 8 + 50) / 100; }

uint8_t* mode(uint8_t* t, uint8_t m) { memset(t, 0, 11); t[0] = m; return t + 1; }
void off(uint8_t* t) { mode(t, 0x05); }
void mask(uint8_t* p, uint16_t m) { p[0] = m; p[1] = m >> 8; }

/* active zones [from..9] + 3-bit force per zone (Feedback / Vibration) */
void tail(uint8_t* p, uint8_t from, uint8_t s) {
  uint16_t act = 0; uint32_t f = 0;
  for (int i = from; i < 10; i++) { act |= 1u << i; f |= (uint32_t)((s - 1) & 7) << (3 * i); }
  mask(p, act);
  memcpy(p + 2, &f, 4);                       /* LE */
}
/* Weapon/Machine: a in [lo..hi-1], b in [a+1..hi] */
uint16_t pairStrict(uint8_t a, uint8_t b, uint8_t lo, uint8_t hi) {
  a = constrain(a, lo, hi - 1); b = constrain(b, a + 1, hi);
  return (1u << a) | (1u << b);
}
/* Bow/Galloping: same, but a may equal hi (collapses to one bit, matches old wire output) */
uint16_t pairLoose(uint8_t a, uint8_t b, uint8_t hi) {
  a = min(a, hi); b = min(b, hi);
  if (b <= a) b = a < hi ? a + 1 : hi;
  return (1u << a) | (1u << b);
}

void feedback(uint8_t* t, uint8_t s0, uint8_t sp) {
  uint8_t s = str(sp);
  if (!s) return off(t);
  tail(mode(t, 0x21), pos(s0), s);
}
void weapon(uint8_t* t, uint8_t s0, uint8_t e0, uint8_t sp) {
  uint8_t s = str(sp);
  if (!s) return off(t);
  uint8_t* p = mode(t, 0x25);
  mask(p, pairStrict(pos(s0), pos(e0), 2, 8));
  p[2] = s - 1;
}
void vibration(uint8_t* t, uint8_t s0, uint8_t ap, uint8_t hz) {
  uint8_t a = str(ap);
  if (!a || !hz) return off(t);
  uint8_t* p = mode(t, 0x26);
  tail(p, pos(s0), a);
  p[8] = hz;
}
void bow(uint8_t* t, uint8_t s0, uint8_t e0, uint8_t sp, uint8_t np) {
  uint8_t s = str(sp), n = str(np);
  if (!s || !n) return off(t);
  uint8_t* p = mode(t, 0x22);
  mask(p, pairLoose(pos(s0), pos(e0), 8));
  p[2] = ((s - 1) & 7) | (((n - 1) & 7) << 3);
}
void galloping(uint8_t* t, uint8_t s0, uint8_t e0, uint8_t f1p, uint8_t f2p, uint8_t hz) {
  if (!hz) return off(t);
  uint8_t f1 = min<uint8_t>(pos(f1p), 6), f2 = min<uint8_t>(pos(f2p), 7);
  if (f2 <= f1) f2 = f1 + 1;
  uint8_t* p = mode(t, 0x23);
  mask(p, pairLoose(pos(s0), pos(e0), 8));
  p[2] = (f2 & 7) | ((f1 & 7) << 3);
  p[3] = hz;
}
void machine(uint8_t* t, uint8_t s0, uint8_t e0, uint8_t ap, uint8_t bp, uint8_t hz, uint8_t per) {
  if (!hz) return off(t);
  uint8_t a = min<uint8_t>(ap, 100) * 7 / 100, b = min<uint8_t>(bp, 100) * 7 / 100;
  uint8_t* p = mode(t, 0x27);
  mask(p, pairStrict(pos(s0), pos(e0), 0, 9));
  p[2] = a | (b << 3);
  p[3] = hz;
  p[4] = per;
}
}  // namespace

#define L output.leftTrigger
#define R output.rightTrigger
ps5Controller& ps5Controller::l2Off()                                                                     { off(L); return *this; }
ps5Controller& ps5Controller::l2Rigid    (uint8_t s, uint8_t f)                                           { feedback(L, s, f); return *this; }
ps5Controller& ps5Controller::l2Trigger  (uint8_t s, uint8_t e, uint8_t f)                                { weapon(L, s, e, f); return *this; }
ps5Controller& ps5Controller::l2Pulse    (uint8_t s, uint8_t f, uint8_t hz)                               { vibration(L, s, f, hz); return *this; }
ps5Controller& ps5Controller::l2Bow      (uint8_t s, uint8_t e, uint8_t f, uint8_t n)                     { bow(L, s, e, f, n); return *this; }
ps5Controller& ps5Controller::l2Galloping(uint8_t s, uint8_t e, uint8_t a, uint8_t b, uint8_t hz)         { galloping(L, s, e, a, b, hz); return *this; }
ps5Controller& ps5Controller::l2Machine  (uint8_t s, uint8_t e, uint8_t a, uint8_t b, uint8_t hz, uint8_t p) { machine(L, s, e, a, b, hz, p); return *this; }
ps5Controller& ps5Controller::r2Off()                                                                     { off(R); return *this; }
ps5Controller& ps5Controller::r2Rigid    (uint8_t s, uint8_t f)                                           { feedback(R, s, f); return *this; }
ps5Controller& ps5Controller::r2Trigger  (uint8_t s, uint8_t e, uint8_t f)                                { weapon(R, s, e, f); return *this; }
ps5Controller& ps5Controller::r2Pulse    (uint8_t s, uint8_t f, uint8_t hz)                               { vibration(R, s, f, hz); return *this; }
ps5Controller& ps5Controller::r2Bow      (uint8_t s, uint8_t e, uint8_t f, uint8_t n)                     { bow(R, s, e, f, n); return *this; }
ps5Controller& ps5Controller::r2Galloping(uint8_t s, uint8_t e, uint8_t a, uint8_t b, uint8_t hz)         { galloping(R, s, e, a, b, hz); return *this; }
ps5Controller& ps5Controller::r2Machine  (uint8_t s, uint8_t e, uint8_t a, uint8_t b, uint8_t hz, uint8_t p) { machine(R, s, e, a, b, hz, p); return *this; }
#undef L
#undef R

#ifndef NO_GLOBAL_INSTANCES
ps5Controller ps5;
#endif
