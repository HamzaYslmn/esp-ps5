/* ps5_bytes.cpp - DualSense (PS5) Bluetooth wire-protocol parser & builder.
 *
 * THIS FILE IS THE PROTOCOL DOCUMENTATION.
 * If you change anything here, double-check against:
 *   - Linux kernel: drivers/hid/hid-playstation.c
 *     (struct dualsense_input_report, dualsense_output_report_common,
 *      dualsense_output_report_bt, plus DS_OUTPUT_VALID_FLAG* / DS_BUTTONS*)
 *   - https://controllers.fandom.com/wiki/Sony_DualSense
 *
 * ----------------------------------------------------------------------------
 * INPUT report layout, BT mode (controller -> ESP32), 78 bytes total
 * ----------------------------------------------------------------------------
 *
 *   wire byte | meaning
 *   ----------|---------------------------------------------------------------
 *      0      | report id 0x31 (BT input report)
 *      1      | reserved tag byte (varies, ignore)
 *
 *   The remaining 76 bytes are the "common" input struct. Offsets below are
 *   ABSOLUTE wire offsets (so add 0 to read directly from the L2CAP buffer):
 *
 *      2      | LX  - left  stick X, 0..255, 128 = center, 0 = left
 *      3      | LY  - left  stick Y, 0..255, 128 = center, 0 = up    (Y inverted)
 *      4      | RX  - right stick X
 *      5      | RY  - right stick Y
 *      6      | L2  - left  trigger pressure 0..255
 *      7      | R2  - right trigger pressure 0..255
 *      8      | sequence/seq_number
 *      9      | buttons[0]:
 *             |   bits 0..3 = HAT switch (D-pad), 0=N,1=NE,2=E,3=SE,4=S,5=SW,
 *             |                                  6=W,7=NW,8=neutral
 *             |   bit 4 = SQUARE
 *             |   bit 5 = CROSS
 *             |   bit 6 = CIRCLE
 *             |   bit 7 = TRIANGLE
 *     10      | buttons[1]:
 *             |   bit 0 = L1, bit 1 = R1, bit 2 = L2 (digital), bit 3 = R2 (digital)
 *             |   bit 4 = CREATE/SHARE, bit 5 = OPTIONS, bit 6 = L3, bit 7 = R3
 *     11      | buttons[2]:
 *             |   bit 0 = PS HOME, bit 1 = TOUCHPAD click, bit 2 = MIC MUTE
 *     12      | buttons[3] (reserved)
 *  13..16     | reserved
 *  17..22     | gyroscope  (le16 each, RAW int16, uncalibrated)  -- parsed
 *             |   17..18 = gyro[0] = PITCH  (nose up/down)  -> .gyro.x
 *             |   19..20 = gyro[1] = YAW    (left/right turn) -> .gyro.y
 *             |   21..22 = gyro[2] = ROLL   (tilt L/R)       -> .gyro.z
 *             |   Kernel scale: raw / 1024  =>  deg/s
 *             |   Note: even at rest each axis has a bias of a few hundred
 *             |   LSBs. Linux subtracts a per-unit factory bias from feature
 *             |   report 0x05; we expose the raw value, sketch must offset.
 *  23..28     | accelerometer (le16 each, RAW int16, uncalibrated) -- parsed
 *             |   23..24 = accel[0] = X  -> .accel.x
 *             |   25..26 = accel[1] = Y  -> .accel.y
 *             |   27..28 = accel[2] = Z  -> .accel.z
 *             |   Kernel scale: raw / 8192  =>  g  (so resting flat,
 *             |   accel.z ~= +8192 = 1g down through controller's Z).
 *             |   Sign convention is unmodified vs Linux (no negation).
 *  29..32     | sensor timestamp (le32, units of 0.33 us)  -- parsed
 *     33      | reserved
 *  34..41     | touchpad point 0 + point 1 (4 bytes each) -- parsed
 *  42..53     | reserved
 *     54      | status[0]:
 *             |   bits 0..3 = battery capacity (0..10 = 0%..100%)
 *             |   bits 4..7 = charging status
 *             |               0 = discharging, 1 = charging,
 *             |               2 = full,        0xA/0xB = error,
 *             |               0xF = charge fault
 *     55      | status[1]:
 *             |   bit 0 = HP detect, bit 1 = MIC detect, bit 2 = MIC muted
 *     56      | status[2] (reserved)
 *  57..73     | reserved + crc32 trailer (last 4 bytes are CRC32 of report)
 *
 * ----------------------------------------------------------------------------
 * OUTPUT report layout, BT mode (ESP32 -> controller), 79 bytes ON THE WIRE
 * ----------------------------------------------------------------------------
 *
 *   wire byte | meaning
 *   ----------|---------------------------------------------------------------
 *      0      | 0xA2 - HID transaction header (DATA | OUTPUT)
 *      1      | 0x31 - report id (BT output)
 *      2      | seq_tag - high nibble = sequence number (0..15, increments
 *             |           per report), low nibble = 0
 *      3      | tag = 0x10 - DS_OUTPUT_TAG (mandatory; controller drops the
 *             |              report if this is wrong)
 *
 *      Bytes 4..50 are `dualsense_output_report_common` (47 bytes):
 *
 *      4      | valid_flag0 - says which fields below to honour:
 *             |   bit 0 = COMPATIBLE_VIBRATION  (use motor_left/motor_right)
 *             |   bit 1 = HAPTICS_SELECT        (select classic rumble path)
 *             |   bit 5 = SPEAKER_VOLUME_ENABLE
 *             |   bit 6 = MIC_VOLUME_ENABLE
 *             |   bit 7 = AUDIO_CONTROL_ENABLE
 *             | For rumble we need BOTH bit0 and bit1 set together.
 *      5      | valid_flag1:
 *             |   bit 0 = MIC_MUTE_LED_CONTROL_ENABLE
 *             |   bit 1 = POWER_SAVE_CONTROL_ENABLE
 *             |   bit 2 = LIGHTBAR_CONTROL_ENABLE  (use lightbar_red/g/b)
 *             |   bit 3 = RELEASE_LEDS
 *             |   bit 4 = PLAYER_INDICATOR_CONTROL_ENABLE  (use player_leds)
 *             |   bit 7 = AUDIO_CONTROL2_ENABLE
 *      6      | motor_right - high-frequency rumble motor 0..255
 *      7      | motor_left  - low-frequency  rumble motor 0..255
 *      8      | headphone_volume 0..0x7F
 *      9      | speaker_volume   0..0xFF
 *     10      | mic_volume       0..0x40
 *     11      | audio_control - bits 4..5 = output path select
 *     12      | mute_button_led 0=off, 1=solid, 2=pulse
 *     13      | power_save_control - bit 4 = mic mute
 *  14..40     | reserved2[27]
 *     41      | audio_control2 - bits 0..2 = SP preamp gain
 *     42      | valid_flag2:
 *             |   bit 1 = LIGHTBAR_SETUP_CONTROL_ENABLE  (apply lightbar_setup)
 *             |   bit 2 = COMPATIBLE_VIBRATION2 (alt rumble path for v2 fw)
 *  43..44     | reserved3[2]
 *     45      | lightbar_setup - bit 1 = LIGHT_OUT (cancels startup blue fade
 *             |                  so user RGB is honoured immediately)
 *     46      | led_brightness  - 0..2 (player LED brightness)
 *     47      | player_leds:
 *             |   bit 0 = far-left LED
 *             |   bit 1
 *             |   bit 2 = center LED
 *             |   bit 3
 *             |   bit 4 = far-right LED
 *             |   bit 5 = "off" indicator (set to fade animation off)
 *     48      | lightbar_red   0..255
 *     49      | lightbar_green 0..255
 *     50      | lightbar_blue  0..255
 *  51..74     | reserved (zero)
 *  75..78     | CRC32 LE - reflected CRC32 (poly 0xEDB88320) of bytes [0..74]
 *             |            seeded with 0xFFFFFFFF, ones-complemented at the end
 *             |            (i.e. standard zlib/Ethernet/Linux crc32_le).
 *
 * Total wire bytes = 1 (0xA2) + 1 (id) + 1 (seq_tag) + 1 (tag) + 47 (common)
 *                  + 24 (reserved) + 4 (crc32) = 79.
 *
 * The wire 0xA2 prefix IS part of the CRC input -- it's the BT-HID transaction
 * byte the controller signs along with the rest of the payload.
 * ============================================================================
 */

#include "ps5Controller.h"
#include <string.h>

/* ============================================================ wire offsets */

/* OUTPUT (sent on HID interrupt channel): absolute wire offsets. */
enum {
  WO_HID_HDR        = 0,   /* 0xA2 */
  WO_REPORT_ID      = 1,   /* 0x31 */
  WO_SEQ_TAG        = 2,
  WO_TAG            = 3,   /* 0x10 */

  /* dualsense_output_report_common starts at 4 */
  WO_VALID_FLAG0    = 4,
  WO_VALID_FLAG1    = 5,
  WO_MOTOR_RIGHT    = 6,
  WO_MOTOR_LEFT     = 7,
  WO_MUTE_LED       = 12,
  WO_VALID_FLAG2    = 42,
  WO_LIGHTBAR_SETUP = 45,
  WO_LED_BRIGHTNESS = 46,
  WO_PLAYER_LEDS    = 47,
  WO_LIGHTBAR_R     = 48,
  WO_LIGHTBAR_G     = 49,
  WO_LIGHTBAR_B     = 50,

  WO_CRC32          = 75,  /* CRC covers bytes [0..74] */
  WO_TOTAL          = 79
};

/* OUTPUT valid_flag bits (Linux kernel naming) */
#define VF0_COMPATIBLE_VIBRATION   0x01
#define VF0_HAPTICS_SELECT         0x02
#define VF1_MIC_MUTE_LED_ENABLE    0x01
#define VF1_LIGHTBAR_ENABLE        0x04
#define VF1_PLAYER_LED_ENABLE      0x10
#define VF2_LIGHTBAR_SETUP_ENABLE  0x02
#define LIGHTBAR_SETUP_LIGHT_OUT   0x02   /* cancel startup fade */

/* INPUT (received on HID interrupt channel): absolute wire offsets. */
enum {
  WI_REPORT_ID    = 0,    /* 0x31 */
  WI_TAG          = 1,    /* reserved */
  WI_LX           = 2,
  WI_LY           = 3,
  WI_RX           = 4,
  WI_RY           = 5,
  WI_L2_TRIGGER   = 6,
  WI_R2_TRIGGER   = 7,
  WI_SEQ_NUMBER   = 8,
  WI_BTN0         = 9,    /* hat + face buttons */
  WI_BTN1         = 10,   /* L1/R1/L2/R2/Share/Options/L3/R3 */
  WI_BTN2         = 11,   /* PS / touchpad / mute */
  WI_GYRO_X       = 17,   /* int16 LE x3 */
  WI_ACCEL_X      = 23,   /* int16 LE x3 */
  WI_TIMESTAMP    = 29,   /* uint32 LE */
  WI_TOUCH0       = 34,   /* 4 bytes per contact */
  WI_TOUCH1       = 38,
  WI_STATUS0      = 54,   /* battery + charging */
  WI_STATUS1      = 55    /* HP/MIC detect, MIC mute */
};

/* INPUT bits */
#define BTN0_HAT_MASK     0x0F
#define BTN0_SQUARE       0x10
#define BTN0_CROSS        0x20
#define BTN0_CIRCLE       0x40
#define BTN0_TRIANGLE     0x80
#define BTN1_L1           0x01
#define BTN1_R1           0x02
#define BTN1_L2           0x04
#define BTN1_R2           0x08
#define BTN1_CREATE       0x10   /* "Share" */
#define BTN1_OPTIONS      0x20
#define BTN1_L3           0x40
#define BTN1_R3           0x80
#define BTN2_PS_HOME      0x01
#define BTN2_TOUCHPAD     0x02
#define BTN2_MIC_MUTE     0x04
#define STATUS0_BATTERY   0x0F
#define STATUS0_CHARGING  0xF0
#define STATUS1_HP        0x01
#define STATUS1_MIC       0x02
#define STATUS1_MIC_MUTE  0x04

/* CRC32 seeds used by the DualSense (kept here for reference; we feed the
 * 0xA2 byte as the first byte of `buf` so we don't apply the seed manually). */

/* ============================================================ CRC32 */

/* Reflected CRC-32 (poly 0xEDB88320, init 0xFFFFFFFF, xorout 0xFFFFFFFF).
 * Same algorithm as zlib / Ethernet / Linux's crc32_le. */
static uint32_t crc32_table[256];
static bool     crc32_table_ready = false;

static void crc32_init_table() {
  for (uint32_t i = 0; i < 256; i++) {
    uint32_t c = i;
    for (int j = 0; j < 8; j++)
      c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    crc32_table[i] = c;
  }
  crc32_table_ready = true;
}

extern "C" uint32_t ps5_crc32(uint32_t seed, const uint8_t* buf, uint16_t len) {
  if (!crc32_table_ready) crc32_init_table();
  uint32_t c = seed ^ 0xFFFFFFFFu;
  for (uint16_t i = 0; i < len; i++)
    c = crc32_table[(c ^ buf[i]) & 0xFF] ^ (c >> 8);
  return c ^ 0xFFFFFFFFu;
}

/* ============================================================ output report */

static uint8_t out_seq = 0;  /* 4-bit sequence number; increments per report */

/* Build and send the DualSense BT 0x31 OUTPUT report on the HID interrupt
 * channel. Sketch supplies state via the `cmd` parameter; we wrap it in the
 * full wire layout, append CRC32, hand it to L2CAP. */
extern "C" void ps5Cmd(ps5_cmd_t cmd) {
  hid_cmd_t out = {};

  /* HID-over-L2CAP framing */
  out.data[WO_HID_HDR]   = 0xA2;
  out.data[WO_REPORT_ID] = 0x31;
  out.data[WO_SEQ_TAG]   = (uint8_t)((out_seq & 0x0F) << 4);
  out.data[WO_TAG]       = 0x10;
  out_seq = (uint8_t)((out_seq + 1) & 0x0F);

  /* Tell controller which fields we are setting */
  out.data[WO_VALID_FLAG0] = VF0_COMPATIBLE_VIBRATION | VF0_HAPTICS_SELECT;
  out.data[WO_VALID_FLAG1] = VF1_LIGHTBAR_ENABLE
                           | VF1_PLAYER_LED_ENABLE
                           | VF1_MIC_MUTE_LED_ENABLE;
  out.data[WO_VALID_FLAG2] = VF2_LIGHTBAR_SETUP_ENABLE;

  /* Rumble */
  out.data[WO_MOTOR_RIGHT] = cmd.smallRumble;   /* high-freq */
  out.data[WO_MOTOR_LEFT]  = cmd.largeRumble;   /* low-freq  */

  /* Mic-mute LED (0=off, 1=solid, 2=pulse) */
  out.data[WO_MUTE_LED] = cmd.muteLed;

  /* Lightbar */
  out.data[WO_LIGHTBAR_SETUP] = LIGHTBAR_SETUP_LIGHT_OUT;
  out.data[WO_LIGHTBAR_R]     = cmd.r;
  out.data[WO_LIGHTBAR_G]     = cmd.g;
  out.data[WO_LIGHTBAR_B]     = cmd.b;

  /* Player LEDs */
  out.data[WO_LED_BRIGHTNESS] = 1;                   /* mid brightness */
  out.data[WO_PLAYER_LEDS]    = cmd.playerLeds & 0x1F;

  /* Trailing CRC32 (covers bytes [0..74], i.e. everything before the CRC) */
  uint32_t crc = ps5_crc32(0, out.data, WO_CRC32);
  out.data[WO_CRC32 + 0] = (uint8_t)(crc      );
  out.data[WO_CRC32 + 1] = (uint8_t)(crc >>  8);
  out.data[WO_CRC32 + 2] = (uint8_t)(crc >> 16);
  out.data[WO_CRC32 + 3] = (uint8_t)(crc >> 24);

  out.length = WO_TOTAL;
  ps5_l2cap_send_hid_interrupt(&out, WO_TOTAL);
}

/* Enable handshake: SET_REPORT(FEATURE, 0xF4) with payload {0x43, 0x02} on the
 * HID *control* channel. This flips the controller from short USB-style
 * report 0x01 to full BT 0x31 input streaming. Must be sent right after
 * the L2CAP interrupt channel comes up. */
extern "C" void ps5Enable(void) {
  hid_cmd_t cmd = {};
  cmd.data[0] = 0x53;          /* SET_REPORT | type FEATURE */
  cmd.data[1] = 0xF4;          /* feature report id */
  cmd.data[2] = 0x43;          /* magic byte 1 */
  cmd.data[3] = 0x02;          /* magic byte 2 */
  cmd.length  = 4;
  ps5_l2cap_send_hid(&cmd, 4);
}

/* ============================================================ input parser */

/* Hat switch (D-pad) decode table. Order matches kernel ps_gamepad_hat_mapping. */
struct HatBits { uint8_t up, right, down, left, ne, se, sw, nw; };
static const HatBits HAT_DECODE[9] = {
  /* 0=N      */ {1,0,0,0, 0,0,0,0},
  /* 1=NE     */ {0,0,0,0, 1,0,0,0},
  /* 2=E      */ {0,1,0,0, 0,0,0,0},
  /* 3=SE     */ {0,0,0,0, 0,1,0,0},
  /* 4=S      */ {0,0,1,0, 0,0,0,0},
  /* 5=SW     */ {0,0,0,0, 0,0,1,0},
  /* 6=W      */ {0,0,0,1, 0,0,0,0},
  /* 7=NW     */ {0,0,0,0, 0,0,0,1},
  /* 8=center */ {0,0,0,0, 0,0,0,0},
};

static ps5_t prev_state = {};

static ps5_event_t computeEdges(const ps5_t& prev, const ps5_t& cur) {
  ps5_event_t ev = {};
  #define EDGE(field) do { \
    ev.button_down.field = !prev.button.field &&  cur.button.field; \
    ev.button_up.field   =  prev.button.field && !cur.button.field; \
  } while (0)

  EDGE(up); EDGE(right); EDGE(down); EDGE(left);
  EDGE(upright); EDGE(downright); EDGE(upleft); EDGE(downleft);
  EDGE(square); EDGE(cross); EDGE(circle); EDGE(triangle);
  EDGE(l1); EDGE(r1); EDGE(l2); EDGE(r2);
  EDGE(share); EDGE(options); EDGE(l3); EDGE(r3);
  EDGE(ps); EDGE(touchpad); EDGE(mute);
  #undef EDGE

  ev.analog_move.stick.lx = (int8_t)(cur.analog.stick.lx - prev.analog.stick.lx);
  ev.analog_move.stick.ly = (int8_t)(cur.analog.stick.ly - prev.analog.stick.ly);
  ev.analog_move.stick.rx = (int8_t)(cur.analog.stick.rx - prev.analog.stick.rx);
  ev.analog_move.stick.ry = (int8_t)(cur.analog.stick.ry - prev.analog.stick.ry);
  ev.analog_move.button.l2 = (uint8_t)(cur.analog.button.l2 - prev.analog.button.l2);
  ev.analog_move.button.r2 = (uint8_t)(cur.analog.button.r2 - prev.analog.button.r2);
  return ev;
}

extern "C" void parsePacket(uint8_t* p) {
  /* Strip the HIDP DATA|INPUT transaction header (0xA1) if present, so the
   * rest of this function can use absolute wire offsets where p[0] is the
   * report ID 0x31. */
  if (p[0] == 0xA1) p++;

  ps5_t s = {};

  /* Sticks: convert 0..255 (128 = center) to signed int8.
   * Y axes are inverted on PS controllers (0 = up). */
  s.analog.stick.lx = (int8_t)((int)p[WI_LX] - 128);
  s.analog.stick.ly = (int8_t)(127 - (int)p[WI_LY]);
  s.analog.stick.rx = (int8_t)((int)p[WI_RX] - 128);
  s.analog.stick.ry = (int8_t)(127 - (int)p[WI_RY]);
  s.analog.button.l2 = p[WI_L2_TRIGGER];
  s.analog.button.r2 = p[WI_R2_TRIGGER];

  uint8_t b0 = p[WI_BTN0];
  uint8_t b1 = p[WI_BTN1];
  uint8_t b2 = p[WI_BTN2];

  /* D-pad */
  uint8_t hat = b0 & BTN0_HAT_MASK;
  if (hat > 8) hat = 8;
  const HatBits& h = HAT_DECODE[hat];
  s.button.up = h.up; s.button.right = h.right;
  s.button.down = h.down; s.button.left = h.left;
  s.button.upright = h.ne; s.button.downright = h.se;
  s.button.downleft = h.sw; s.button.upleft = h.nw;

  /* Face buttons */
  s.button.square   = (b0 & BTN0_SQUARE)   ? 1 : 0;
  s.button.cross    = (b0 & BTN0_CROSS)    ? 1 : 0;
  s.button.circle   = (b0 & BTN0_CIRCLE)   ? 1 : 0;
  s.button.triangle = (b0 & BTN0_TRIANGLE) ? 1 : 0;

  /* Shoulder + meta */
  s.button.l1      = (b1 & BTN1_L1)      ? 1 : 0;
  s.button.r1      = (b1 & BTN1_R1)      ? 1 : 0;
  s.button.l2      = (b1 & BTN1_L2)      ? 1 : 0;
  s.button.r2      = (b1 & BTN1_R2)      ? 1 : 0;
  s.button.share   = (b1 & BTN1_CREATE)  ? 1 : 0;
  s.button.options = (b1 & BTN1_OPTIONS) ? 1 : 0;
  s.button.l3      = (b1 & BTN1_L3)      ? 1 : 0;
  s.button.r3      = (b1 & BTN1_R3)      ? 1 : 0;

  s.button.ps       = (b2 & BTN2_PS_HOME)  ? 1 : 0;
  s.button.touchpad = (b2 & BTN2_TOUCHPAD) ? 1 : 0;
  s.button.mute     = (b2 & BTN2_MIC_MUTE) ? 1 : 0;

  /* Motion sensors: gyro + accel as int16 little-endian.
   * Verified byte-for-byte against Linux kernel
   *   struct dualsense_input_report { ...; __le16 gyro[3]; __le16 accel[3]; ... }
   * Values are RAW (uncalibrated). Kernel scale: gyro / 1024 deg/s,
   * accel / 8192 g. Gyro semantics: x=PITCH, y=YAW, z=ROLL. */
  s.sensor.gyro.x  = (int16_t)(p[WI_GYRO_X + 0] | (p[WI_GYRO_X + 1] << 8));  /* pitch */
  s.sensor.gyro.y  = (int16_t)(p[WI_GYRO_X + 2] | (p[WI_GYRO_X + 3] << 8));  /* yaw   */
  s.sensor.gyro.z  = (int16_t)(p[WI_GYRO_X + 4] | (p[WI_GYRO_X + 5] << 8));  /* roll  */
  s.sensor.accel.x = (int16_t)(p[WI_ACCEL_X + 0] | (p[WI_ACCEL_X + 1] << 8));
  s.sensor.accel.y = (int16_t)(p[WI_ACCEL_X + 2] | (p[WI_ACCEL_X + 3] << 8));
  s.sensor.accel.z = (int16_t)(p[WI_ACCEL_X + 4] | (p[WI_ACCEL_X + 5] << 8));
  s.sensor.timestamp = (uint32_t)p[WI_TIMESTAMP + 0]
                     | ((uint32_t)p[WI_TIMESTAMP + 1] << 8)
                     | ((uint32_t)p[WI_TIMESTAMP + 2] << 16)
                     | ((uint32_t)p[WI_TIMESTAMP + 3] << 24);

  /* Touchpad: two contact slots, 4 bytes each.
   *   byte 0: bit 7 = inactive (1 = lifted), bits 0..6 = contact id
   *   byte 1: x[7:0]
   *   byte 2: y[3:0] | x[11:8]<<4 (low nibble = x_high, high nibble = y_low)
   *   byte 3: y[11:4]                                                       */
  for (int i = 0; i < 2; i++) {
    const uint8_t* tp = p + (i == 0 ? WI_TOUCH0 : WI_TOUCH1);
    s.sensor.touch[i].active = (tp[0] & 0x80) ? 0 : 1;
    s.sensor.touch[i].id     = tp[0] & 0x7F;
    s.sensor.touch[i].x      = (uint16_t)tp[1] | ((uint16_t)(tp[2] & 0x0F) << 8);
    s.sensor.touch[i].y      = ((uint16_t)tp[2] >> 4) | ((uint16_t)tp[3] << 4);
  }

  /* Status: battery + charging */
  uint8_t st0 = p[WI_STATUS0];
  uint8_t st1 = p[WI_STATUS1];
  uint8_t batt = st0 & STATUS0_BATTERY;
  uint8_t chg  = (st0 & STATUS0_CHARGING) >> 4;
  if (batt > 10) batt = 10;
  s.status.battery       = batt;
  s.status.charging      = (chg == 1);
  s.status.fully_charged = (chg == 2);
  s.status.headphones    = (st1 & STATUS1_HP)  ? 1 : 0;
  s.status.mic           = (st1 & STATUS1_MIC) ? 1 : 0;

  s.latestPacket = p;

  ps5_event_t ev = computeEdges(prev_state, s);
  prev_state = s;
  ps5PacketEvent(s, ev);
}
