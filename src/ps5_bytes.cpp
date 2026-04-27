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
 *
 * ----------------------------------------------------------------------------
 * UNUSED / FUTURE-WORK BITS (verified vs Linux hid-playstation.c, 2026)
 * ----------------------------------------------------------------------------
 *
 * Cross-checked against drivers/hid/hid-playstation.c in linux master
 * (Sony Interactive Entertainment, GPL-2.0). All identifiers below match
 * the kernel's DS_OUTPUT_VALID_FLAG* / DS_OUTPUT_* / DS_STATUS* names.
 *
 * == Audio (mic + speaker) - protocol-supported, transport-blocked ==
 *
 * The OUTPUT report already carries:
 *   - byte  8: headphone_volume (0..0x7F)
 *   - byte  9: speaker_volume   (0..0xFF, DualSense uses 0x3D..0x64)
 *   - byte 10: mic_volume       (0..0x40)
 *   - byte 11: audio_control - bits 4..5 = output path select:
 *               0 = HP=L|R, SP=mute  (headphones plugged in)
 *               1 = HP=L|L, SP=mute
 *               2 = HP=L|L, SP=R
 *               3 = HP=mute, SP=R    (no headphones, route to internal SP)
 *   - byte 41: audio_control2 - bits 0..2 = SP preamp gain (0=+0dB, 1=+6dB,
 *               2=+12dB; kernel uses +6 dB when only SP is active).
 * Required valid_flag bits to make any of this take effect:
 *   VF0 bit 5 = SPEAKER_VOLUME_ENABLE
 *   VF0 bit 6 = MIC_VOLUME_ENABLE
 *   VF0 bit 7 = AUDIO_CONTROL_ENABLE   (path select + mute toggles)
 *   VF1 bit 7 = AUDIO_CONTROL2_ENABLE  (SP preamp gain)
 *
 * BUT: actually streaming audio (capturing from the mic, playing through
 * the speaker) needs a separate Bluetooth audio profile - HFP/HSP for the
 * mic, A2DP for the speaker. The Linux kernel explicitly notes
 * "Bluetooth audio is currently not supported" for DualSense; on ESP32 it
 * would mean enabling the BT classic audio stack alongside L2CAP HID
 * (Bluedroid supports both, but co-existence is heavy on RAM/flash).
 * Conclusion: volume / routing / mute *control* is a small future feature;
 * actual mic capture is a much larger project.
 *
 * == Compatible vibration v2 ==
 *
 * VF2 bit 2 = COMPATIBLE_VIBRATION2. Firmware feature-version >= 2.21
 * (DualSense Edge unconditionally) prefers the v2 rumble path over the v1
 * COMPATIBLE_VIBRATION (VF0 bit 0) we currently use. v1 still works on
 * new firmwares but Sony recommends v2. Reading the firmware version
 * needs feature report 0x20 (DS_FEATURE_REPORT_FIRMWARE_INFO, 64 B).
 *
 * == Feature reports we don't read (would need a USB connection or a
 *    GET_REPORT round-trip on the control PSM) ==
 *
 * Feature 0x05 (41 B): factory motion calibration -- per-axis bias +
 *   sensitivity numerator/denominator. Without this, raw gyro / accel are
 *   off by hundreds of LSBs; sketch has to compute its own offset. The
 *   kernel reads this once at probe and applies a linear correction.
 * Feature 0x09 (20 B): pairing info -- includes the controller's BT MAC
 *   (we already get that from the BT link itself, so unnecessary).
 * Feature 0x20 (64 B): firmware/hardware version + update_version, used
 *   to gate vibration v2 (see above).
 *
 * == Other reserved input bytes ==
 *
 * Input bytes 12 (buttons[3]), 13..16, 33, 42..53, 56, 57..73 are zero on
 * every firmware seen so far. If new firmware ever lights one of these
 * up, `ps5.latestPacket` is the place to inspect it from a sketch.
 *
 * ----------------------------------------------------------------------------
 * ============================================================================
 */
#include "ps5Controller.h"
#include <string.h>

extern "C" void ps5_mark_alive(void);   /* defined in ps5Controller.cpp */

/* ============================================================ wire offsets */

enum { /* OUTPUT */
  WO_HID_HDR        = 0,   /* 0xA2 */
  WO_REPORT_ID      = 1,   /* 0x31 */
  WO_SEQ_TAG        = 2,
  WO_TAG            = 3,   /* 0x10 */
  WO_VALID_FLAG0    = 4,
  WO_VALID_FLAG1    = 5,
  WO_MOTOR_RIGHT    = 6,
  WO_MOTOR_LEFT     = 7,
  WO_MUTE_LED       = 12,
  WO_POWER_SAVE     = 13,  /* bit 4 = DS_OUTPUT_POWER_SAVE_CONTROL_MIC_MUTE */
  WO_R2_TRIG_MODE   = 14,  /* +10 param bytes */
  WO_R2_TRIG_PARAM  = 15,
  WO_L2_TRIG_MODE   = 25,  /* +10 param bytes */
  WO_L2_TRIG_PARAM  = 26,
  WO_VALID_FLAG2    = 42,
  WO_LIGHTBAR_SETUP = 45,
  WO_LED_BRIGHTNESS = 46,
  WO_PLAYER_LEDS    = 47,
  WO_LIGHTBAR_R     = 48,
  WO_LIGHTBAR_G     = 49,
  WO_LIGHTBAR_B     = 50,
  WO_CRC32          = 75,  /* CRC covers [0..74] */
  WO_TOTAL          = 79
};

/* OUTPUT valid_flag bits (Linux kernel naming) */
#define VF0_COMPATIBLE_VIBRATION   0x01
#define VF0_HAPTICS_SELECT         0x02
#define VF0_R2_TRIGGER_ENABLE      0x04
#define VF0_L2_TRIGGER_ENABLE      0x08
#define VF1_MIC_MUTE_LED_ENABLE    0x01
#define VF1_POWER_SAVE_ENABLE      0x02   /* DS_OUTPUT_VALID_FLAG1_POWER_SAVE_CONTROL_ENABLE */
#define VF1_LIGHTBAR_ENABLE        0x04
#define VF1_RELEASE_LEDS           0x08   /* DS_OUTPUT_VALID_FLAG1_RELEASE_LEDS (one-shot) */
#define VF1_PLAYER_LED_ENABLE      0x10
#define VF2_LIGHT_BRIGHTNESS_ENABLE 0x01  /* gates byte 46 (player LED brightness) */
#define VF2_LIGHTBAR_SETUP_ENABLE  0x02  /* gates byte 45 (lightbar fade-in cancel) */
#define LIGHTBAR_SETUP_LIGHT_OUT   0x02

enum { /* INPUT */
  WI_LX = 2, WI_LY, WI_RX, WI_RY,
  WI_L2_TRIGGER = 6, WI_R2_TRIGGER,
  WI_BTN0 = 9, WI_BTN1, WI_BTN2,
  WI_GYRO_X    = 17,
  WI_ACCEL_X   = 23,
  WI_TIMESTAMP = 29,
  WI_TOUCH0    = 34, WI_TOUCH1 = 38,
  WI_STATUS0   = 54, WI_STATUS1 = 55
};

#define BTN0_HAT_MASK   0x0F
#define BTN0_SQUARE     0x10
#define BTN0_CROSS      0x20
#define BTN0_CIRCLE     0x40
#define BTN0_TRIANGLE   0x80
#define BTN1_L1         0x01
#define BTN1_R1         0x02
#define BTN1_L2         0x04
#define BTN1_R2         0x08
#define BTN1_CREATE     0x10
#define BTN1_OPTIONS    0x20
#define BTN1_L3         0x40
#define BTN1_R3         0x80
#define BTN2_PS_HOME    0x01
#define BTN2_TOUCHPAD   0x02
#define BTN2_MIC_MUTE   0x04
#define STATUS0_BATTERY 0x0F
#define STATUS0_CHARGING 0xF0
#define STATUS1_HP      0x01
#define STATUS1_MIC     0x02
#define STATUS1_MIC_MUTE 0x04   /* DS_STATUS1_MIC_MUTE: persisted mute toggle */

/* ============================================================ CRC32 (zlib) */

/* Compile-time CRC32 table (zlib polynomial 0xEDB88320, reflected). Lives in
 * flash via constexpr+const, so it costs 0 B RAM (vs 1 KB for the runtime
 * version we used to lazy-init on first send). */
static constexpr uint32_t crc32_byte(uint32_t c) {
  return (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
}
static constexpr uint32_t crc32_step(uint32_t c, int n) {
  return n == 0 ? c : crc32_step(crc32_byte(c), n - 1);
}
#define CRC32_E(i) crc32_step((uint32_t)(i), 8)
#define CRC32_R4(i) CRC32_E(i),     CRC32_E(i+1),   CRC32_E(i+2),   CRC32_E(i+3)
#define CRC32_R16(i) CRC32_R4(i),   CRC32_R4(i+4),  CRC32_R4(i+8),  CRC32_R4(i+12)
#define CRC32_R64(i) CRC32_R16(i),  CRC32_R16(i+16),CRC32_R16(i+32),CRC32_R16(i+48)
static const uint32_t crc32_table[256] = {
  CRC32_R64(0), CRC32_R64(64), CRC32_R64(128), CRC32_R64(192)
};
#undef CRC32_E
#undef CRC32_R4
#undef CRC32_R16
#undef CRC32_R64

static uint32_t crc32_le(const uint8_t* buf, uint16_t len) {
  uint32_t c = 0xFFFFFFFFu;
  for (uint16_t i = 0; i < len; i++)
    c = crc32_table[(c ^ buf[i]) & 0xFF] ^ (c >> 8);
  return c ^ 0xFFFFFFFFu;
}

/* ============================================================ output (build) */

static uint8_t out_seq = 0;   /* 4-bit sequence */

extern "C" void ps5BuildAndSend(void) {
  hid_cmd_t out = {};
  uint8_t* d = out.data;

  /* HID-over-L2CAP framing */
  d[WO_HID_HDR]   = 0xA2;
  d[WO_REPORT_ID] = 0x31;
  d[WO_SEQ_TAG]   = (uint8_t)((out_seq & 0x0F) << 4);
  d[WO_TAG]       = 0x10;
  out_seq = (uint8_t)((out_seq + 1) & 0x0F);

  d[WO_VALID_FLAG0] = VF0_COMPATIBLE_VIBRATION | VF0_HAPTICS_SELECT
                    | VF0_R2_TRIGGER_ENABLE   | VF0_L2_TRIGGER_ENABLE;
  d[WO_VALID_FLAG1] = VF1_LIGHTBAR_ENABLE | VF1_PLAYER_LED_ENABLE | VF1_MIC_MUTE_LED_ENABLE;
  d[WO_VALID_FLAG2] = VF2_LIGHTBAR_SETUP_ENABLE | VF2_LIGHT_BRIGHTNESS_ENABLE;

  const ps5Controller::Out& o = ps5.output;
  /* Optional output paths the user may have opted into. POWER_SAVE drives the
   * actual mic-mute hardware bit (independent of the muteLed visual). */
  if (o.micMute)     { d[WO_VALID_FLAG1] |= VF1_POWER_SAVE_ENABLE; d[WO_POWER_SAVE] |= 0x10; }
  if (o.releaseLeds) { d[WO_VALID_FLAG1] |= VF1_RELEASE_LEDS; ps5.output.releaseLeds = false; }
  d[WO_MOTOR_RIGHT]    = o.smallRumble;
  d[WO_MOTOR_LEFT]     = o.largeRumble;
  d[WO_MUTE_LED]       = o.muteLed;
  d[WO_LIGHTBAR_SETUP] = LIGHTBAR_SETUP_LIGHT_OUT;
  d[WO_LIGHTBAR_R]     = o.r;
  d[WO_LIGHTBAR_G]     = o.g;
  d[WO_LIGHTBAR_B]     = o.b;
  d[WO_LED_BRIGHTNESS] = (o.ledBrightness > 2) ? 2 : o.ledBrightness;
  d[WO_PLAYER_LEDS]    = o.playerLeds & 0x1F;

  d[WO_R2_TRIG_MODE] = o.rightTriggerMode;
  memcpy(&d[WO_R2_TRIG_PARAM], o.rightTriggerParam, 10);
  d[WO_L2_TRIG_MODE] = o.leftTriggerMode;
  memcpy(&d[WO_L2_TRIG_PARAM], o.leftTriggerParam,  10);

  uint32_t crc = crc32_le(d, WO_CRC32);
  d[WO_CRC32 + 0] = (uint8_t)(crc      );
  d[WO_CRC32 + 1] = (uint8_t)(crc >>  8);
  d[WO_CRC32 + 2] = (uint8_t)(crc >> 16);
  d[WO_CRC32 + 3] = (uint8_t)(crc >> 24);

  out.length = WO_TOTAL;
  ps5_l2cap_send_hid_interrupt(&out, WO_TOTAL);
}

/* SET_REPORT(FEATURE, 0xF4) {0x43, 0x02} on the HID *control* channel.
 * Flips the controller from short USB-style report 0x01 to full BT 0x31. */
extern "C" void ps5Enable(void) {
  hid_cmd_t cmd = {};
  cmd.data[0] = 0x53;   /* SET_REPORT | type FEATURE */
  cmd.data[1] = 0xF4;
  cmd.data[2] = 0x43;
  cmd.data[3] = 0x02;
  cmd.length  = 4;
  ps5_l2cap_send_hid(&cmd, 4);
}

/* ============================================================ input parser */

/* HAT (D-pad) decode. Order matches kernel ps_gamepad_hat_mapping.
 *  index = N, NE, E, SE, S, SW, W, NW, neutral. We expose the four cardinals
 *  plus the four diagonals so a sketch can do `ps5.up && ps5.right` for NE. */
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

extern "C" void parsePacket(uint8_t* p) {
  /* Strip the HIDP DATA|INPUT transaction header (0xA1) if present. */
  if (p[0] == 0xA1) p++;

  /* Sticks: 0..255 (128 = center) -> signed int8 by raw-minus-128. The DualSense
   * wire already has up=low-raw and down=high-raw, so the same formula on X and
   * Y gives the documented convention: push UP = negative, push DOWN = positive,
   * push RIGHT = positive, push LEFT = negative. */
  ps5.lx = (int8_t)((int)p[WI_LX] - 128);
  ps5.ly = (int8_t)((int)p[WI_LY] - 128);
  ps5.rx = (int8_t)((int)p[WI_RX] - 128);
  ps5.ry = (int8_t)((int)p[WI_RY] - 128);
  ps5.l2 = p[WI_L2_TRIGGER];
  ps5.r2 = p[WI_R2_TRIGGER];

  uint8_t b0 = p[WI_BTN0], b1 = p[WI_BTN1], b2 = p[WI_BTN2];

  /* Set a Button: update `cur` and latch the .pressed/.released edge flag. */
  auto setBtn = [](ps5Controller::Button& b, bool v) {
    if (v && !b.cur)      b.pressed.v  = true;
    else if (!v && b.cur) b.released.v = true;
    b.cur = v;
  };

  /* D-pad - we expose the four cardinals; diagonals are derivable as e.g.
   * (ps5.up && ps5.right). Neutral = all four false. */
  uint8_t hat = b0 & BTN0_HAT_MASK;
  if (hat > 8) hat = 8;
  const HatBits& h = HAT_DECODE[hat];
  setBtn(ps5.up,    h.up   || h.ne || h.nw);
  setBtn(ps5.down,  h.down || h.se || h.sw);
  setBtn(ps5.right, h.right|| h.ne || h.se);
  setBtn(ps5.left,  h.left || h.nw || h.sw);

  setBtn(ps5.square,   (b0 & BTN0_SQUARE));
  setBtn(ps5.cross,    (b0 & BTN0_CROSS));
  setBtn(ps5.circle,   (b0 & BTN0_CIRCLE));
  setBtn(ps5.triangle, (b0 & BTN0_TRIANGLE));

  setBtn(ps5.l1,      (b1 & BTN1_L1));
  setBtn(ps5.r1,      (b1 & BTN1_R1));
  /* Digital L2/R2 booleans are exposed via the analog l2/r2 fields:
   * "pressed if > 0". Bits BTN1_L2/R2 carry the same info, so we drop
   * them rather than expose two duplicated representations. */
  setBtn(ps5.share,   (b1 & BTN1_CREATE));
  setBtn(ps5.options, (b1 & BTN1_OPTIONS));
  setBtn(ps5.l3,      (b1 & BTN1_L3));
  setBtn(ps5.r3,      (b1 & BTN1_R3));
  setBtn(ps5.ps_btn,  (b2 & BTN2_PS_HOME));
  setBtn(ps5.touchpad,(b2 & BTN2_TOUCHPAD));
  setBtn(ps5.mute,    (b2 & BTN2_MIC_MUTE));

  /* Motion (raw int16 LE). Kernel scale: gyro/1024 deg/s, accel/8192 g. */
  ps5.gyroX  = (int16_t)(p[WI_GYRO_X + 0] | (p[WI_GYRO_X + 1] << 8));
  ps5.gyroY  = (int16_t)(p[WI_GYRO_X + 2] | (p[WI_GYRO_X + 3] << 8));
  ps5.gyroZ  = (int16_t)(p[WI_GYRO_X + 4] | (p[WI_GYRO_X + 5] << 8));
  ps5.accelX = (int16_t)(p[WI_ACCEL_X + 0] | (p[WI_ACCEL_X + 1] << 8));
  ps5.accelY = (int16_t)(p[WI_ACCEL_X + 2] | (p[WI_ACCEL_X + 3] << 8));
  ps5.accelZ = (int16_t)(p[WI_ACCEL_X + 4] | (p[WI_ACCEL_X + 5] << 8));
  ps5.sensorTime = (uint32_t)p[WI_TIMESTAMP + 0]
                 | ((uint32_t)p[WI_TIMESTAMP + 1] << 8)
                 | ((uint32_t)p[WI_TIMESTAMP + 2] << 16)
                 | ((uint32_t)p[WI_TIMESTAMP + 3] << 24);

  /* Touchpad: 4 bytes per contact.
   *   byte 0: bit7 inactive (1=lifted), bits 0..6 = id
   *   byte 1: x[7:0]
   *   byte 2: low nibble = x[11:8], high nibble = y[3:0]
   *   byte 3: y[11:4] */
  for (int i = 0; i < 2; i++) {
    const uint8_t* tp = p + (i == 0 ? WI_TOUCH0 : WI_TOUCH1);
    ps5.touch[i].active = !(tp[0] & 0x80);
    ps5.touch[i].id     = tp[0] & 0x7F;
    ps5.touch[i].x      = (uint16_t)tp[1] | ((uint16_t)(tp[2] & 0x0F) << 8);
    ps5.touch[i].y      = ((uint16_t)tp[2] >> 4) | ((uint16_t)tp[3] << 4);
  }

  /* Status */
  uint8_t st0 = p[WI_STATUS0], st1 = p[WI_STATUS1];
  uint8_t batt = st0 & STATUS0_BATTERY;
  uint8_t chg  = (st0 & STATUS0_CHARGING) >> 4;
  if (batt > 10) batt = 10;
  ps5.battery      = (uint8_t)(batt * 10);     /* 0..10 raw -> 0..100 %% */
  ps5.charging      = (chg == 1);
  ps5.fullyCharged  = (chg == 2);
  ps5.chargingError = (chg == 0xA) || (chg == 0xB) || (chg == 0xF);
  ps5.headphones    = (st1 & STATUS1_HP);
  ps5.micJack       = (st1 & STATUS1_MIC);
  ps5.micMuted      = (st1 & STATUS1_MIC_MUTE);

  ps5.latestPacket = p;          /* expose the raw 78-B wire buffer for debug */

  ps5_mark_alive();
}
