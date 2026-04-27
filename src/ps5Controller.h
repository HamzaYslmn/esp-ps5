/* ps5Controller.h - DualSense (PS5) controller for ESP32, Arduino style.
 *
 * Hello world:
 *   ps5.begin();                                      // connect
 *   while (!ps5.isConnected()) delay(10);
 *   if (ps5.cross) ps5.lightbar(255,0,0).rumble(0,200).send();
 */

#ifndef ps5Controller_h
#define ps5Controller_h

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
#include "Arduino.h"

class ps5Controller {
 public:
  typedef void (*callback_t)();
  typedef void (*scan_cb_t)(const uint8_t mac[6], const char* name, int8_t rssi);

  /* ============================================================ INPUT.
   * Read these like normal variables; they refresh automatically. */

  /* Sticks: -128..+127, centered at 0. Y is flipped (push UP = negative). */
  int8_t   lx = 0, ly = 0, rx = 0, ry = 0;

  /* Analog triggers: 0 = released, 255 = fully pressed. */
  /* Analog triggers: 0 = released, 255 = fully pressed. */
  uint8_t  l2 = 0, r2 = 0;

  /* Easy percent helpers. Sticks: -100..+100 (0 centered, up = negative for Y).
   * Triggers: 0..100. Use these if you don't want to deal with raw -128/+127 or 0/255. */
  inline int8_t  lxPct() const { return (int8_t)((int)lx * 100 / 127); }
  inline int8_t  lyPct() const { return (int8_t)((int)ly * 100 / 127); }
  inline int8_t  rxPct() const { return (int8_t)((int)rx * 100 / 127); }
  inline int8_t  ryPct() const { return (int8_t)((int)ry * 100 / 127); }
  inline uint8_t l2Pct() const { return (uint8_t)((int)l2 * 100 / 255); }
  inline uint8_t r2Pct() const { return (uint8_t)((int)r2 * 100 / 255); }

  /* Buttons. Each one is a tiny struct that:
   *   - Acts like a bool          : `if (ps5.square) ...`           true while held.
   *   - Has `.pressed`  edge flag : `if (ps5.square.pressed)  ...`  fires ONCE per press.
   *   - Has `.released` edge flag : `if (ps5.square.released) ...`  fires ONCE per release.
   * Edge flags are LATCHED until you read them, so you never miss an event no
   * matter how slow your loop runs. */
  struct EdgeFlag {
    mutable bool v = false;
    /* read = consume: returns the flag and clears it in one shot. */
    operator bool() const { bool r = v; v = false; return r; }
  };
  struct Button {
    bool     cur = false;
    EdgeFlag pressed;
    EdgeFlag released;
    operator bool() const { return cur; }   /* `if (ps5.square)` still works */
  };

  Button   l1, r1, l3, r3;                  /* shoulders + stick clicks */
  Button   up, down, left, right;           /* D-pad */
  Button   cross, circle, square, triangle;
  Button   share, options, ps_btn, touchpad, mute;

  /* Motion sensors (raw). gyro / 1024 = deg/sec.  accel / 8192 = g.
   * Marked volatile because parsePacket() writes from the Bluedroid task while
   * loop() reads from the Arduino task; without volatile the compiler could
   * cache a stale value in a register across reads. Each individual access is
   * still subject to a one-sample tear if it lands mid-write - read three
   * times and discard if a difference is observed if you need bit-perfect
   * snapshots, otherwise just live with the rare 1-frame jitter. */
  volatile int16_t  gyroX = 0, gyroY = 0, gyroZ = 0;
  volatile int16_t  accelX = 0, accelY = 0, accelZ = 0;
  volatile uint32_t sensorTime = 0;

  /* Status. battery: 0..100 (percent). */
  uint8_t  battery = 0;
  bool     charging = false, fullyCharged = false, headphones = false, micJack = false;
  /* True when the controller's charging logic reports a fault (over-/under-
   * voltage, temperature out of range, charge fault). Combine with `charging`:
   * if both false -> discharging on battery; if `chargingError` true the
   * controller will NOT charge until the user unplugs/cools/replugs. */
  bool     chargingError = false;
  /* True when the user has pressed the mute button on the controller and the
   * controller's firmware has marked the mic as muted (persists until the
   * user taps it again). Independent of `output.micMute` (which we send to
   * the controller); they may briefly disagree until the next packet round-trip. */
  bool     micMuted = false;

  /* Raw last input packet (78 B). Pointer to the wire buffer; valid only just
   * after a packet arrives. Useful when reverse-engineering new fields
   * (mic / speaker / future firmware bytes). volatile because parsePacket
   * (Bluedroid task) writes the pointer while loop() reads it. */
  const uint8_t* volatile latestPacket = nullptr;

  /* Touchpad: 2 fingers max. Surface is 1920 x 1080. x/y are volatile for the
   * same cross-task reason as the motion fields above. */
  struct Touch {
    bool              active;   /* finger currently on the pad? */
    uint8_t           id;       /* changes each time a finger lifts + touches again */
    volatile uint16_t x, y;     /* pixel position */
  } touch[2] = {};
  bool     TouchActive(int i) { return touch[i & 1].active; }
  uint8_t  TouchId    (int i) { return touch[i & 1].id; }
  uint16_t TouchX     (int i) { return touch[i & 1].x; }
  uint16_t TouchY     (int i) { return touch[i & 1].y; }

  /* ============================================================ OUTPUT.
   * What we WANT the controller to do next. Use the helpers below, then send().
   * (You can also poke fields directly, e.g. ps5.output.r = 200;) */
  struct Out {
    uint8_t smallRumble = 0, largeRumble = 0;   /* small=sharp buzz, large=deep rumble. 0..255 */
    uint8_t r = 0, g = 0, b = 0;                /* lightbar color, 0..255 each */
    uint8_t playerLeds = 0;                     /* 5-bit mask, bit 0 = far-left LED */
    uint8_t ledBrightness = 1;                  /* wire value, 0=bright,1=mid,2=dim. Set indirectly via playerLed(). */
    uint8_t muteLed = 0;                        /* 0=off, 1=on, 2=pulse */
    bool    micMute = true;                     /* default ON: mic hardware OFF + power-save bit set. Call micMute(false) to capture audio (needs A2DP/HFP — not currently wired). */
    bool    releaseLeds = false;                /* one-shot: hand lightbar/player-LED control back to firmware on next send(); auto-cleared. */
    uint8_t leftTriggerMode = 0,  leftTriggerParam[10] = {0};   /* set via l2*() helpers */
    uint8_t rightTriggerMode = 0, rightTriggerParam[10] = {0};  /* set via r2*() helpers */
  } output;

  /* ============================================================ API. */

  /* Connect. First call scans + pairs; later calls fast-reconnect (saved MAC). */
  bool begin(uint8_t timeoutSecs = 30);

  /* Connect to a specific MAC, e.g. "AA:BB:CC:DD:EE:FF". Skips scanning. */
  bool begin(const char* mac);

  /* True while the controller is talking to us. Call this every loop. */
  bool isConnected();

  /* Erase the saved MAC, so the next begin() scans for a new controller. */
  void forget();

  /* Scan for `secs` seconds. cb(mac, name, rssi) fires once per device found. */
  bool scanDevices(uint8_t secs, scan_cb_t cb);

  /* ---- Output helpers. Chainable. Nothing sent until you call send(). ---- */

  /* Lightbar RGB color, 0..255 each. (0,0,0) = off.
   * Lightbar = the colored strip around the touchpad. (Player LEDs are the
   * row of 5 white LEDs below it — use playerLed() for those.) */
  ps5Controller& lightbar     (uint8_t r, uint8_t g, uint8_t b);

  /* Rumble motors, 0..255. small = sharp buzz, large = deep rumble. */
  ps5Controller& rumble       (uint8_t small, uint8_t large);

  /* Light a player LED. index = 1..5 (1 = far-left, 5 = far-right).
   * value sets BOTH on/off and brightness:
   *   0          = off
   *   1          = on, dim
   *   2          = on, medium
   *   3 or more  = on, bright (255 also works)
   * Chain freely: ps5.playerLed(1, 3).playerLed(3, 3).playerLed(5, 3).send(); */
  ps5Controller& playerLed    (uint8_t index, uint8_t value);

  /* Mic-mute LED.  0 = off,  1 = on,  2 = pulse. */
  ps5Controller& muteLed      (uint8_t mode);

  /* Electrically mute / un-mute the microphone hardware (separate from the
   * LED above — this gates the actual audio capture). */
  ps5Controller& micMute      (bool on);

  /* One-shot: tell the controller to take its lightbar / player-LED state
   * back into firmware control (default boot behaviour). Cleared after send(). */
  ps5Controller& releaseLeds  ();

  /* Send everything you set above. Call at most once per 10 ms. */
  ps5Controller& send();

  /* ---- Adaptive triggers (L2 / R2). ----
   * All percents are 0..100. freqHz is real Hz (try 5..30). Pick ONE mode per
   * trigger; L2 and R2 are independent so you can mix two:
   *   ps5.l2Trigger(20,80,100).r2Pulse(30,100,15).send();
   *
   * Basic:
   *   l2Off()                                  - no force.
   *   l2Rigid(start, strength)                 - stiff wall past `start`.
   *   l2Trigger(start, end, strength)          - gun-trigger squeeze + click.
   *   l2Pulse(start, strength, freqHz)         - vibrating buzz past `start`.
   *
   * Combo (firmware presets - effectively two effects at once):
   *   l2Bow(start, end, strength, snap)        - squeeze + snap-back at the end.
   *   l2Galloping(start, end, foot1, foot2, freqHz) - horse-gallop rhythm.
   *   l2Machine(start, end, ampA, ampB, freqHz, periodTenths)
   *      Buzz inside [start..end]; strength swaps between ampA and ampB every
   *      `periodTenths` (in 0.1 s units, so 5 = 0.5 s).
   *
   * r2*() are the same for the right trigger. */
  ps5Controller& l2Off();
  ps5Controller& l2Rigid    (uint8_t start, uint8_t strength);
  ps5Controller& l2Trigger  (uint8_t start, uint8_t end, uint8_t strength);
  ps5Controller& l2Pulse    (uint8_t start, uint8_t strength, uint8_t freqHz);
  ps5Controller& l2Bow      (uint8_t start, uint8_t end, uint8_t strength, uint8_t snap);
  ps5Controller& l2Galloping(uint8_t start, uint8_t end, uint8_t foot1, uint8_t foot2, uint8_t freqHz);
  ps5Controller& l2Machine  (uint8_t start, uint8_t end, uint8_t ampA, uint8_t ampB, uint8_t freqHz, uint8_t periodTenths);
  ps5Controller& r2Off();
  ps5Controller& r2Rigid    (uint8_t start, uint8_t strength);
  ps5Controller& r2Trigger  (uint8_t start, uint8_t end, uint8_t strength);
  ps5Controller& r2Pulse    (uint8_t start, uint8_t strength, uint8_t freqHz);
  ps5Controller& r2Bow      (uint8_t start, uint8_t end, uint8_t strength, uint8_t snap);
  ps5Controller& r2Galloping(uint8_t start, uint8_t end, uint8_t foot1, uint8_t foot2, uint8_t freqHz);
  ps5Controller& r2Machine  (uint8_t start, uint8_t end, uint8_t ampA, uint8_t ampB, uint8_t freqHz, uint8_t periodTenths);

  /* ---- Callbacks (optional). Pass a `void myFn()` function. ---- */
  void attach            (callback_t cb) { _onPacket     = cb; }  /* every packet (~250 Hz) */
  void attachOnConnect   (callback_t cb) { _onConnect    = cb; }  /* once, when controller wakes up */
  void attachOnDisconnect(callback_t cb) { _onDisconnect = cb; }  /* when controller drops */

  /* Internal - called by the C-side dispatchers in ps5_bytes.cpp / bluedroid.cpp. */
  void _fireInput()             { if (_onPacket)     _onPacket(); }
  void _fireConnState(bool up)  { if (up) { if (_onConnect) _onConnect(); }
                                  else    { if (_onDisconnect) _onDisconnect(); } }

 private:
  callback_t _onPacket = nullptr, _onConnect = nullptr, _onDisconnect = nullptr;
};

#ifndef NO_GLOBAL_INSTANCES
extern ps5Controller ps5;
#endif

#endif /* __cplusplus */

/* ============================================================ INTERNAL C API.
 * Sketches don't need any of this. Used by the layers to talk to each other. */
#ifdef __cplusplus
extern "C" {
#endif

/* Outbound HID-over-L2CAP buffer.
 *   data[0] = HID transaction header (0xA2 OUTPUT / 0x53 SET_FEATURE).
 *   data[1..] = HID report id then payload. */
#define ps5_SEND_BUFFER_SIZE 80
typedef struct { uint8_t data[ps5_SEND_BUFFER_SIZE]; uint8_t length; } hid_cmd_t;

/* ps5_bytes.cpp */
void parsePacket(uint8_t* p);            /* writes flat fields on global ps5, fires _fireInput */
void ps5BuildAndSend(void);              /* reads ps5.output, builds frame, sends on interrupt PSM */
void ps5Enable(void);                    /* SET_FEATURE 0xF4 handshake on control PSM (legacy one-shot) */

/* ps5Controller.cpp */
void ps5ConnectEvent(uint8_t isConnected);   /* called by L2CAP on link up/down */

/* bluedroid/bluedroid.cpp */
void  ps5_l2cap_init_services(void);
long  ps5_l2cap_connect(uint8_t addr[6]);
long  ps5_l2cap_reconnect(void);
bool  ps5_l2cap_has_target(void);
bool  ps5_l2cap_is_active(void);   /* both HID L2CAP channels configured */
bool  ps5_l2cap_has_any_cid(void); /* either control or interrupt CID still non-zero */
void  ps5_l2cap_get_target(uint8_t out[6]);
void  ps5_l2cap_clear_target(void);
void  ps5_l2cap_send_hid          (hid_cmd_t* c, uint8_t len);  /* control PSM 0x11   */
void  ps5_l2cap_send_hid_interrupt(hid_cmd_t* c, uint8_t len);  /* interrupt PSM 0x13 */

#ifdef __cplusplus
}
#endif

#endif
