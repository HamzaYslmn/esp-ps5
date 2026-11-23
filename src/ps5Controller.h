/* ps5Controller.h - DualSense (PS5) controller for ESP32 over Bluetooth Classic.
 *
 *   ps5.begin();                                   // returns at once; connects in the background
 *   if (ps5.isConnected() && ps5.cross) ps5.lightbar(255,0,0).rumble(0,200).send();
 *
 * Never blocks. Input fields are written by the Bluetooth task; frames are sent from it too.
 */
#ifndef ps5Controller_h
#define ps5Controller_h

#include "Arduino.h"

class ps5Controller {
 public:
  typedef void (*callback_t)();

  /* ============================================================ INPUT (auto-refreshed) */

  int8_t  lx = 0, ly = 0, rx = 0, ry = 0;   /* sticks -128..127, 0 = center, push UP = negative */
  uint8_t deadzone = 10;                    /* % of stick travel that reads 0. Set any value; 0 = raw */
  uint8_t l2 = 0, r2 = 0;                   /* analog triggers 0..255 */

  int8_t  lxPct() const { return lx * 100 / 127; }   /* -100..100 */
  int8_t  lyPct() const { return ly * 100 / 127; }
  int8_t  rxPct() const { return rx * 100 / 127; }
  int8_t  ryPct() const { return ry * 100 / 127; }
  uint8_t l2Pct() const { return l2 * 100 / 255; }   /* 0..100 */
  uint8_t r2Pct() const { return r2 * 100 / 255; }

  /* `if (ps5.square)` = held. `.pressed` / `.released` = edge, latched until read. */
  struct EdgeFlag {
    mutable bool v = false;
    operator bool() const { return __atomic_exchange_n(&v, false, __ATOMIC_ACQ_REL); }   /* read = consume */
  };
  struct Button {
    bool cur = false;
    EdgeFlag pressed, released;
    operator bool() const { return cur; }
  };
  Button l1, r1, l3, r3;
  Button up, down, left, right;              /* diagonals: ps5.up && ps5.right */
  Button cross, circle, square, triangle;
  Button share, options, ps_btn, touchpad, mute;

  /* Raw motion. gyro / 1024 = deg/s, accel / 8192 = g. Written from the BT task. */
  volatile int16_t  gyroX = 0, gyroY = 0, gyroZ = 0;
  volatile int16_t  accelX = 0, accelY = 0, accelZ = 0;
  volatile uint32_t sensorTime = 0;          /* 0.33 us per LSB */

  /* Touchpad, 2 fingers, 1920 x 1080. */
  struct Touch { bool active; uint8_t id; volatile uint16_t x, y; } touch[2] = {};

  uint8_t battery = 0;                       /* 0..100 % */
  bool    charging = false, fullyCharged = false, chargingError = false;
  bool    headphones = false, micJack = false;
  bool    micMuted = false;                  /* firmware-persisted mute state */

  uint8_t latestPacket[78] = {};             /* copy of the last raw 0x31 input report */

  /* ============================================================ OUTPUT (staged; call send()) */
  struct Out {
    uint8_t smallRumble = 0, largeRumble = 0;   /* 0..255 */
    uint8_t r = 0, g = 0, b = 0;                /* lightbar */
    uint8_t playerLeds = 0;                     /* 5-bit mask, bit 0 = far-left */
    uint8_t ledBrightness = 1;                  /* wire: 0=bright 1=mid 2=dim (use playerLed()) */
    uint8_t muteLed = 0;                        /* 0=off 1=on 2=pulse */
    bool    micMute = true;                     /* mic hardware off by default */
    bool    releaseLeds = false;                /* true: lightbar + player LEDs run by the pad firmware */
    uint8_t leftTrigger[11] = {0x05}, rightTrigger[11] = {0x05};   /* [mode, p0..p9]; 0x05 = off */
  } output;

  /* ============================================================ API */

  bool begin();                        /* start BT; connect to the paired pad or scan. Returns at once. */
  bool begin(const char* mac);         /* connect to "AA:BB:CC:DD:EE:FF". Returns at once. */
  bool isConnected() const;            /* true while input reports are flowing */
  void forget();                       /* drop the target MAC; next retry rescans */

  /* Chainable output setters. send() queues one frame; call it as often as you like
   * (sent from the BT task, at most every 10 ms). LEDs apply once the pad's own
   * ~3 s connect animation has finished. */
  ps5Controller& lightbar   (uint8_t r, uint8_t g, uint8_t b);
  ps5Controller& rumble     (uint8_t small, uint8_t large);
  ps5Controller& playerLed  (uint8_t index /*1..5*/, uint8_t value /*0 off,1 dim,2 mid,3+ bright*/);
  ps5Controller& muteLed    (uint8_t mode);
  ps5Controller& micMute    (bool on);
  ps5Controller& releaseLeds(bool on);   /* give LEDs to the pad firmware / take them back */
  ps5Controller& send();

  /* Adaptive triggers. Percent args 0..100, freqHz in Hz. One mode per trigger;
   * L2 and R2 are independent. Combo modes (Bow/Galloping/Machine) are firmware presets. */
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

  /* Sound through the HD-haptic actuators (square wave, 3 kHz sample rate, notes up to
   * ~1.5 kHz). Non-blocking: plays in the background. hz = 0 is a rest.
   * The notes array must stay alive while playing (use static const). */
  struct Note { uint16_t hz, ms; };
  void tone(uint16_t hz, uint16_t ms, uint8_t volume = 60);
  void play(const Note* notes, size_t count, uint8_t volume = 60);
  void stopSound();
  bool isPlaying() const;

  /* Callbacks run on the Bluetooth task: keep them short. */
  void attach            (callback_t cb) { onPacket     = cb; }   /* every report (~250 Hz) */
  void attachOnConnect   (callback_t cb) { onConnect    = cb; }   /* first report after connect */
  void attachOnDisconnect(callback_t cb) { onDisconnect = cb; }

  callback_t onPacket = nullptr, onConnect = nullptr, onDisconnect = nullptr;
};

#ifndef NO_GLOBAL_INSTANCES
extern ps5Controller ps5;
#endif

/* ---- internal glue between the three .cpp files (not for sketches) ---- */
bool ps5_parse(const uint8_t* p, uint16_t len, bool edges);   /* ps5_bytes.cpp */
void ps5_send_output(bool ready, bool leds);
void ps5_send_reset();
void ps5_send_haptics(const int8_t* samples64);
void ps5_on_link(bool up);                                    /* ps5Controller.cpp */
void ps5_on_input(const uint8_t* p, uint16_t len);
void ps5_l2cap_init();                                        /* bluedroid/bluedroid.cpp */
void ps5_l2cap_connect(const uint8_t* mac);                   /* nullptr = last target */
bool ps5_l2cap_up();                                          /* both HID channels configured */
bool ps5_l2cap_idle();                                        /* no channel open at all */
bool ps5_l2cap_target(uint8_t* out);                          /* copies MAC if set */
void ps5_l2cap_forget();
void ps5_l2cap_send(bool control, const uint8_t* d, uint8_t len);

#endif
