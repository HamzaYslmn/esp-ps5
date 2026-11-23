/* ps5_bytes.cpp - DualSense BT wire format: 0x31 input parser + 0x31 output builder.
 * Offset tables + valid-flag bits are documented in AGENTS.md; cross-check against
 * Linux drivers/hid/hid-playstation.c before changing anything here. */
#include "ps5Controller.h"
#include <esp_rom_crc.h>

/* ============================================================ output */

enum {                           /* wire offsets, byte 0 = 0xA2 HID header (CRC'd) */
  O_VF0 = 4, O_VF1, O_MOTOR_R, O_MOTOR_L,
  O_MUTE_LED = 12, O_POWER_SAVE, O_R2 = 14, O_L2 = 25,
  O_VF2 = 42, O_BRIGHTNESS = 46, O_PLAYER, O_RGB,
  O_LEN = 79
};

/* 0xA2-prefixed frame: CRC32 (zlib, from ROM) of all but the last 4 bytes, stored LE in them. */
static void sendCrc(uint8_t* d, uint8_t len) {
  uint32_t crc = esp_rom_crc32_le(0, d, len - 4);
  memcpy(d + len - 4, &crc, 4);
  ps5_l2cap_send(false, d, len);
}

/* SDL-style 0x31 frame. Seq nibble stays 0 (SDL never sets it).
 *   ready: pad's connect animation is over -> triggers, mute LED, mic power-save, brightness.
 *          Before that only rumble goes out (anything else during the animation glitches it).
 *   leds:  lightbar + player LEDs (false while handed to the firmware). */
void ps5_send_output(bool ready, bool leds) {
  const ps5Controller::Out& o = ps5.output;
  bool snd = ps5.isPlaying();                  /* melody owns the actuators via 0x32 */
  uint8_t mr = snd ? 0 : o.smallRumble, ml = snd ? 0 : o.largeRumble;
  uint8_t d[O_LEN] = {0xA2, 0x31, 0x00, 0x10};
  /* Rumble emulation (0x01) + "disable audio haptics" (0x02) only while motors run (SDL). */
  d[O_VF0] = (mr || ml) ? 0x03 : 0x00;
  d[O_MOTOR_R] = mr;
  d[O_MOTOR_L] = ml;
  if (ready) {
    d[O_VF0] |= 0x0C;                          /* R2 + L2 adaptive triggers */
    d[O_VF1]  = 0x01;                          /* mute LED */
    d[O_VF2]  = 0x01;                          /* player-LED brightness */
    if (o.micMute) { d[O_VF1] |= 0x02; d[O_POWER_SAVE] = 0x10; }   /* power-save: mic off */
    d[O_MUTE_LED]   = o.muteLed;
    memcpy(d + O_R2, o.rightTrigger, 11);
    memcpy(d + O_L2, o.leftTrigger, 11);
    d[O_BRIGHTNESS] = min<uint8_t>(o.ledBrightness, 2);
  }
  if (leds) {
    d[O_VF1]       |= 0x14;                    /* lightbar + player LEDs */
    d[O_PLAYER]     = (o.playerLeds & 0x1F) | 0x20;   /* 0x20: change instantly, no fade */
    d[O_RGB]        = o.r;
    d[O_RGB + 1]    = o.g;
    d[O_RGB + 2]    = o.b;
  }
  sendCrc(d, O_LEN);
}

/* "Reset LED state" alone (valid_flag1 bit 3, nothing else): hands LEDs between firmware
 * and host. SDL sends it once after the connect animation, then the colour frame. */
void ps5_send_reset() {
  uint8_t d[O_LEN] = {0xA2, 0x31, 0x00, 0x10, 0x00, 0x08};
  sendCrc(d, O_LEN);
}

/* Report 0x32 HD haptics (SAxense layout, MPL-2.0 reference): sub-packet 0x11 config,
 * sub-packet 0x12 = 64 int8 bytes = 32 stereo L/R samples at 3 kHz. Send every 10.667 ms. */
void ps5_send_haptics(const int8_t* s) {
  static uint8_t n = 0;
  uint8_t d[143] = {0xA2, 0x32, 0x00, 0x91, 0x07, 0xFE, 0, 0, 0, 0, 0xFF, n++, 0x92, 0x40};
  memcpy(d + 14, s, 64);
  sendCrc(d, sizeof(d));
}

/* ============================================================ input */

static inline int16_t le16(const uint8_t* p) { return (int16_t)(p[0] | p[1] << 8); }
static int8_t stick(uint8_t raw) {            /* 128 = centre; inside the deadzone reads 0 */
  int v = raw - 128;
  return abs(v) * 100 < ps5.deadzone * 128 ? 0 : v;
}

static void setBtn(ps5Controller::Button& b, bool v, bool edges) {
  if (!edges) b.pressed.v = b.released.v = false;   /* first report: drop stale edges */
  else if (v != b.cur) (v ? b.pressed : b.released).v = true;
  b.cur = v;
}

/* Returns false (and touches nothing) unless p is a full 0x31 report. */
bool ps5_parse(const uint8_t* p, uint16_t len, bool edges) {
  if (len && p[0] == 0xA1) { p++; len--; }     /* HIDP DATA|INPUT header */
  if (len < sizeof(ps5.latestPacket) || p[0] != 0x31) return false;   /* ignore short 0x01 reports */
  memcpy(ps5.latestPacket, p, sizeof(ps5.latestPacket));

  ps5.lx = stick(p[2]); ps5.ly = stick(p[3]);
  ps5.rx = stick(p[4]); ps5.ry = stick(p[5]);
  ps5.l2 = p[6];       ps5.r2 = p[7];

  /* hat 0..7 = N, NE, E, SE, S, SW, W, NW; 8 = neutral. bits: 1 up, 2 right, 4 down, 8 left */
  static const uint8_t HAT[9] = {1, 3, 2, 6, 4, 12, 8, 9, 0};
  uint8_t b0 = p[9], b1 = p[10], b2 = p[11], h = HAT[min(b0 & 0x0F, 8)];
  setBtn(ps5.up,       h & 1,     edges);
  setBtn(ps5.right,    h & 2,     edges);
  setBtn(ps5.down,     h & 4,     edges);
  setBtn(ps5.left,     h & 8,     edges);
  setBtn(ps5.square,   b0 & 0x10, edges);
  setBtn(ps5.cross,    b0 & 0x20, edges);
  setBtn(ps5.circle,   b0 & 0x40, edges);
  setBtn(ps5.triangle, b0 & 0x80, edges);
  setBtn(ps5.l1,       b1 & 0x01, edges);
  setBtn(ps5.r1,       b1 & 0x02, edges);
  setBtn(ps5.share,    b1 & 0x10, edges);
  setBtn(ps5.options,  b1 & 0x20, edges);
  setBtn(ps5.l3,       b1 & 0x40, edges);
  setBtn(ps5.r3,       b1 & 0x80, edges);
  setBtn(ps5.ps_btn,   b2 & 0x01, edges);
  setBtn(ps5.touchpad, b2 & 0x02, edges);
  setBtn(ps5.mute,     b2 & 0x04, edges);

  ps5.gyroX  = le16(p + 17); ps5.gyroY  = le16(p + 19); ps5.gyroZ  = le16(p + 21);
  ps5.accelX = le16(p + 23); ps5.accelY = le16(p + 25); ps5.accelZ = le16(p + 27);
  uint32_t ts; memcpy(&ts, p + 29, 4); ps5.sensorTime = ts;

  for (int i = 0; i < 2; i++) {                /* [id|inactive<<7][x lo][x hi | y lo<<4][y hi] */
    const uint8_t* t = p + 34 + 4 * i;
    ps5.touch[i].active = !(t[0] & 0x80);
    ps5.touch[i].id     = t[0] & 0x7F;
    ps5.touch[i].x      = t[1] | (t[2] & 0x0F) << 8;
    ps5.touch[i].y      = t[2] >> 4 | t[3] << 4;
  }

  uint8_t chg = p[54] >> 4;
  ps5.battery       = min(p[54] & 0x0F, 10) * 10;
  ps5.charging      = chg == 1;
  ps5.fullyCharged  = chg == 2;
  ps5.chargingError = chg == 0xA || chg == 0xB || chg == 0xF;
  ps5.headphones    = p[55] & 0x01;
  ps5.micJack       = p[55] & 0x02;
  ps5.micMuted      = p[55] & 0x04;
  return true;
}
