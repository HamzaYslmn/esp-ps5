/* ps5Controller.h - Arduino-friendly DualSense (PS5) controller for ESP32.
 *
 * Layered overview of the library:
 *
 *   ps5Controller.cpp   <- Arduino class, GAP/Bluedroid bring-up, scanDevices,
 *                          callbacks dispatch, reconnect loop. Holds the
 *                          ps5_t/ps5_event_t/ps5_cmd_t state for the sketch.
 *   ps5_bytes.cpp       <- ALL byte/bit work. Builds the wire-correct
 *                          DualSense BT 0x31 OUTPUT report (rumble, lightbar,
 *                          player LEDs) with trailing CRC32, and parses the
 *                          incoming BT 0x31 INPUT report (sticks, buttons,
 *                          triggers, status). Every offset is documented.
 *   ps5_l2cap.c         <- L2CAP transport (HID control PSM 0x11 + interrupt
 *                          PSM 0x13). Vendored Bluedroid plumbing.
 *   ps5_spp.c           <- Bluedroid bring-up (device name, scan mode).
 *   stack/ + osi/       <- Vendored ESP-IDF Bluedroid headers.
 *
 * If you only want to read the protocol, open ps5_bytes.cpp.
 */

#ifndef ps5Controller_h
#define ps5Controller_h

#ifdef __cplusplus
#include "Arduino.h"
#endif
#include <stdint.h>
#include <stdbool.h>

/* ============================================================ DATA TYPES */

typedef struct {
  int8_t lx, ly;   /* left  stick, signed -128..127 (centered at 0) */
  int8_t rx, ry;   /* right stick, signed -128..127 (centered at 0) */
} ps5_analog_stick_t;

typedef struct {
  uint8_t l2;      /* L2 trigger pressure 0..255 */
  uint8_t r2;      /* R2 trigger pressure 0..255 */
} ps5_analog_button_t;

typedef struct {
  ps5_analog_stick_t  stick;
  ps5_analog_button_t button;
} ps5_analog_t;

typedef struct {
  uint8_t right    : 1;
  uint8_t down     : 1;
  uint8_t up       : 1;
  uint8_t left     : 1;

  uint8_t square   : 1;
  uint8_t cross    : 1;
  uint8_t circle   : 1;
  uint8_t triangle : 1;

  uint8_t upright   : 1;
  uint8_t downright : 1;
  uint8_t upleft    : 1;
  uint8_t downleft  : 1;

  uint8_t l1 : 1;
  uint8_t r1 : 1;
  uint8_t l2 : 1;
  uint8_t r2 : 1;

  uint8_t share   : 1;   /* "Create" button on DualSense */
  uint8_t options : 1;
  uint8_t l3      : 1;
  uint8_t r3      : 1;

  uint8_t ps       : 1;
  uint8_t touchpad : 1;
  uint8_t mute     : 1;  /* mic-mute capacitive button */
} ps5_button_t;

typedef struct {
  uint8_t battery       : 4;  /* 0..10 (10% steps) */
  uint8_t charging      : 1;
  uint8_t fully_charged : 1;
  uint8_t headphones    : 1;
  uint8_t mic           : 1;
} ps5_status_t;

/* Motion sensors. Raw int16 LSB; the DualSense reports gyro in deg/s
 * scaled by ~0x10000/2000 and accel in g scaled by 0x10000/8 (per kernel),
 * but we expose the raw counts so the sketch can pick its own units. */
typedef struct {
  int16_t x, y, z;
} ps5_imu_t;

/* One touchpad contact. The DualSense surface is 1920 x 1080. */
typedef struct {
  uint8_t  active;   /* 1 = finger present, 0 = lifted */
  uint8_t  id;       /* contact id 0..127, increments per fresh touch */
  uint16_t x;        /* 0..1919 */
  uint16_t y;        /* 0..1079 */
} ps5_touch_t;

typedef struct {
  ps5_imu_t   gyro;
  ps5_imu_t   accel;
  uint32_t    timestamp;
  ps5_touch_t touch[2];
} ps5_sensor_t;

/* Output to controller. Caller fills these and calls sendToController(). */
typedef struct {
  uint8_t smallRumble;   /* high-frequency motor (right), 0..255 */
  uint8_t largeRumble;   /* low-frequency  motor (left),  0..255 */
  uint8_t r, g, b;       /* lightbar RGB, 0..255 each */
  uint8_t playerLeds;    /* 5 player LEDs: bit0=far-left ... bit4=far-right */
  uint8_t muteLed;       /* 0 = off, 1 = solid on, 2 = pulsing */
} ps5_cmd_t;

typedef struct {
  ps5_button_t button_down;   /* edge: not-pressed -> pressed THIS packet */
  ps5_button_t button_up;     /* edge: pressed     -> released THIS packet */
  ps5_analog_t analog_move;   /* signed delta vs previous packet (sticks/triggers) */
} ps5_event_t;

typedef struct {
  ps5_analog_t  analog;
  ps5_button_t  button;
  ps5_status_t  status;
  ps5_sensor_t  sensor;
  uint8_t      *latestPacket;
} ps5_t;

/* ============================================================ ARDUINO API */

#ifdef __cplusplus
class ps5Controller {
 public:
  typedef void (*callback_t)();
  typedef void (*scan_cb_t)(const uint8_t mac[6], const char* name, int8_t rssi);

  ps5_t       data;
  ps5_event_t event;
  ps5_cmd_t   output;

  ps5Controller();

  bool begin();
  bool begin(const char* mac);

  /* MARK: scanDevices - one-shot BT Classic inquiry. cb fires per discovered
   * device with (mac, name, rssi). Caller picks one and feeds it to begin(). */
  bool scanDevices(uint8_t secs, scan_cb_t cb);

  void end();
  bool isConnected();

  /* Set local state. Nothing is sent until sendToController(). */
  void setLed(uint8_t r, uint8_t g, uint8_t b);
  void setRumble(uint8_t small, uint8_t large);
  void setPlayerLeds(uint8_t bitmask);   /* 5 bits, bit0..bit4 */
  void setMuteLed(uint8_t mode);         /* 0=off, 1=on, 2=pulse */
  void sendToController();

  /* Sketch hooks. */
  void attach(callback_t cb);
  void attachOnConnect(callback_t cb);
  void attachOnDisconnect(callback_t cb);

  uint8_t* LatestPacket() { return data.latestPacket; }

  /* Convenience accessors used by sketches. */
  bool Right()    { return data.button.right; }
  bool Down()     { return data.button.down; }
  bool Up()       { return data.button.up; }
  bool Left()     { return data.button.left; }
  bool Square()   { return data.button.square; }
  bool Cross()    { return data.button.cross; }
  bool Circle()   { return data.button.circle; }
  bool Triangle() { return data.button.triangle; }
  bool UpRight()   { return data.button.upright; }
  bool DownRight() { return data.button.downright; }
  bool UpLeft()    { return data.button.upleft; }
  bool DownLeft()  { return data.button.downleft; }
  bool L1() { return data.button.l1; }
  bool R1() { return data.button.r1; }
  bool L2() { return data.button.l2; }
  bool R2() { return data.button.r2; }
  bool Share()    { return data.button.share; }
  bool Options()  { return data.button.options; }
  bool L3()       { return data.button.l3; }
  bool R3()       { return data.button.r3; }
  bool PSButton() { return data.button.ps; }
  bool Touchpad() { return data.button.touchpad; }
  bool Mute()     { return data.button.mute; }

  uint8_t L2Value() { return data.analog.button.l2; }
  uint8_t R2Value() { return data.analog.button.r2; }

  int8_t LStickX() { return data.analog.stick.lx; }
  int8_t LStickY() { return data.analog.stick.ly; }
  int8_t RStickX() { return data.analog.stick.rx; }
  int8_t RStickY() { return data.analog.stick.ry; }

  uint8_t Battery()      { return data.status.battery; }
  bool    Charging()     { return data.status.charging; }
  bool    FullyCharged() { return data.status.fully_charged; }
  bool    Headphones()   { return data.status.headphones; }
  bool    MicJack()      { return data.status.mic; }

  /* Motion + touchpad accessors. */
  int16_t GyroX()  { return data.sensor.gyro.x; }
  int16_t GyroY()  { return data.sensor.gyro.y; }
  int16_t GyroZ()  { return data.sensor.gyro.z; }
  int16_t AccelX() { return data.sensor.accel.x; }
  int16_t AccelY() { return data.sensor.accel.y; }
  int16_t AccelZ() { return data.sensor.accel.z; }
  uint32_t SensorTimestamp() { return data.sensor.timestamp; }

  /* idx = 0 (first finger) or 1 (second finger). */
  bool     TouchActive(int idx) { return data.sensor.touch[idx & 1].active; }
  uint8_t  TouchId    (int idx) { return data.sensor.touch[idx & 1].id; }
  uint16_t TouchX     (int idx) { return data.sensor.touch[idx & 1].x; }
  uint16_t TouchY     (int idx) { return data.sensor.touch[idx & 1].y; }

  /* Internal: invoked by the C-side dispatchers. Public so extern "C" glue
   * can call them; sketches should not. */
  static void _event_callback(void* obj, ps5_t d, ps5_event_t e);
  static void _connection_callback(void* obj, uint8_t isConnected);

 private:
  callback_t _callback_event      = nullptr;
  callback_t _callback_connect    = nullptr;
  callback_t _callback_disconnect = nullptr;
};

#ifndef NO_GLOBAL_INSTANCES
extern ps5Controller ps5;
#endif

#endif /* __cplusplus */

/* ============================================================ INTERNAL C API
 *
 * These symbols are exported by ps5_bytes.cpp and ps5Controller.cpp with
 * C linkage so the C-only files (ps5_l2cap.c, ps5_spp.c) can call them.
 * Sketches don't need to touch any of this.
 */
#ifdef __cplusplus
extern "C" {
#endif

/* hid_cmd_t: outbound HID-over-L2CAP buffer.
 * data[0] = HID transaction header byte (0xA2 for OUTPUT, 0x53 for SET_FEATURE).
 * data[1..] = HID report id then payload.
 * length    = number of valid bytes in data[].
 */
#define ps5_SEND_BUFFER_SIZE 80   /* DualSense BT OUTPUT = 0xA2 + 78 bytes + (4-byte CRC is part of 78) */
typedef struct {
  uint8_t data[ps5_SEND_BUFFER_SIZE];
  uint8_t length;
} hid_cmd_t;

/* Implemented in ps5_bytes.cpp */
void     parsePacket(uint8_t* packet);
uint32_t ps5_crc32(uint32_t seed, const uint8_t* buf, uint16_t len);

/* Implemented in ps5Controller.cpp */
void ps5ConnectEvent(uint8_t isConnected);
void ps5PacketEvent(ps5_t ps5, ps5_event_t event);

/* Implemented in ps5_spp.c / ps5_l2cap.c */
void  sppInit(void);
void  ps5_l2cap_init_services(void);
void  ps5_l2cap_deinit_services(void);
long  ps5_l2cap_connect(uint8_t addr[6]);
long  ps5_l2cap_reconnect(void);
void  ps5_l2cap_send_hid          (hid_cmd_t* cmd, uint8_t len); /* control PSM 0x11   */
void  ps5_l2cap_send_hid_interrupt(hid_cmd_t* cmd, uint8_t len); /* interrupt PSM 0x13 */

#ifdef __cplusplus
}
#endif

#endif
