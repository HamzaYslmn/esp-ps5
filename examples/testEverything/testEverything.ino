// testEverything.ino - PS5 DualSense full-feature test sketch.
//
// Wire-up: Bluetooth Classic on an ESP32-WROOM-32. Pick the
//   "Huge APP" partition scheme so Bluedroid fits.
//
// Mappings:
//   LEFT  stick  -> lightbar (Wikipedia RGB wheel: up=red, BR=green, BL=blue)
//   RIGHT stick  -> rumble (RX+ right motor, RX- left motor, |RY| both)
//   Mute button  -> mute LED mirrors button state
//   Player LEDs  -> march left -> right every 600 ms
//
// The serial monitor prints a tidy 1 Hz snapshot of every control,
// laid out so left-side and right-side controls sit side by side.

#include <ps5Controller.h>

static const uint32_t kSendMs   = 30;
static const uint32_t kPlayerMs = 600;
static const uint32_t kReportMs = 1000;
static const int      kStickDeadzone = 16;   // LSB; ~12 % of full scale

// MARK: hsvToRgb - 6-sector HSV->RGB. h in [0..360), s,v in [0..1].
static void hsvToRgb(float h, float s, float v,
                     uint8_t& r, uint8_t& g, uint8_t& b) {
  if (s <= 0.0f) { r = g = b = (uint8_t)(v * 255.0f); return; }
  while (h <    0.0f) h += 360.0f;
  while (h >= 360.0f) h -= 360.0f;
  float hh = h / 60.0f;
  int   i  = (int)hh;
  float f  = hh - (float)i;
  float p = v * (1 - s),  q = v * (1 - s * f),  t = v * (1 - s * (1 - f));
  float rf, gf, bf;
  switch (i) {
    case 0: rf=v; gf=t; bf=p; break;
    case 1: rf=q; gf=v; bf=p; break;
    case 2: rf=p; gf=v; bf=t; break;
    case 3: rf=p; gf=q; bf=v; break;
    case 4: rf=t; gf=p; bf=v; break;
    default:rf=v; gf=p; bf=q; break;
  }
  r = (uint8_t)(rf * 255.0f);
  g = (uint8_t)(gf * 255.0f);
  b = (uint8_t)(bf * 255.0f);
}

// MARK: stickHueSat - LX/LY -> Wikipedia RGB color wheel (up=red).
static void stickToLightbar(int8_t lx, int8_t ly, uint8_t& r, uint8_t& g, uint8_t& b) {
  float fx = (float)lx / 127.0f, fy = (float)ly / 127.0f;
  float mag = sqrtf(fx*fx + fy*fy); if (mag > 1.0f) mag = 1.0f;
  float hue = atan2f(fx, fy) * 180.0f / (float)M_PI;
  if (hue < 0.0f) hue += 360.0f;
  const float kDead = (float)kStickDeadzone / 127.0f;
  float sat = (mag <= kDead) ? 0.0f : (mag - kDead) / (1.0f - kDead);
  hsvToRgb(hue, sat, 1.0f, r, g, b);
}

// MARK: stickToRumble - additive: RX+ right motor, RX- left, |RY| both.
static int rescaleAxis(int v) {
  int a = (v >= 0) ? v : -v; if (a > 127) a = 127;
  return (a > kStickDeadzone) ? ((a - kStickDeadzone) * 127 / (127 - kStickDeadzone)) : 0;
}
static void stickToRumble(int8_t rx, int8_t ry, uint8_t& small_, uint8_t& large_) {
  int absX = rescaleAxis(rx);
  int absY = rescaleAxis(ry);
  int rightX = (rx > 0) ? absX : 0;
  int leftX  = (rx < 0) ? absX : 0;
  int sR = (rightX + absY) * 2; if (sR > 255) sR = 255;
  int lR = (leftX  + absY) * 2; if (lR > 255) lR = 255;
  small_ = (uint8_t)sR;  large_ = (uint8_t)lR;
}

// MARK: snapshot - one group per line, label-prefixed for easy reading.
static void snapshot(uint8_t r, uint8_t g, uint8_t b,
                     uint8_t small_, uint8_t large_, uint8_t playerBit) {
  Serial.println(F("------------------------------------------------"));
  Serial.printf("LS:    x=%+4d y=%+4d  click=%d\n", ps5.lx, ps5.ly, ps5.l3);
  Serial.printf("RS:    x=%+4d y=%+4d  click=%d\n", ps5.rx, ps5.ry, ps5.r3);
  Serial.printf("TRIG:  L1=%d L2val=%3u   R1=%d R2val=%3u\n",
                ps5.l1, ps5.l2, ps5.r1, ps5.r2);
  Serial.printf("DPAD:  U=%d D=%d L=%d R=%d\n",
                ps5.up, ps5.down, ps5.left, ps5.right);
  Serial.printf("FACE:  Tri=%d Cir=%d Cro=%d Sq=%d\n",
                ps5.triangle, ps5.circle, ps5.cross, ps5.square);
  Serial.printf("META:  Share=%d Opt=%d PS=%d Touch=%d Mute=%d\n",
                ps5.share, ps5.options, ps5.ps_btn, ps5.touchpad, ps5.mute);
  Serial.printf("GYRO:  x=%+6d y=%+6d z=%+6d\n",
                ps5.gyroX, ps5.gyroY, ps5.gyroZ);
  Serial.printf("ACCEL: x=%+6d y=%+6d z=%+6d\n",
                ps5.accelX, ps5.accelY, ps5.accelZ);
  for (int i = 0; i < 2; i++) {
    if (ps5.TouchActive(i))
      Serial.printf("TOUCH%d: x=%4u y=%4u id=%u\n",
                    i, ps5.TouchX(i), ps5.TouchY(i), ps5.TouchId(i));
    else
      Serial.printf("TOUCH%d: idle\n", i);
  }
  Serial.printf("BAT:   %u0%% %s  HP=%d MIC=%d\n",
                ps5.battery, ps5.charging ? "CHG" : "DSCH",
                ps5.headphones, ps5.micJack);
  Serial.printf("OUT:   rgb=(%3u,%3u,%3u)  rumble=(%3u,%3u)  player=bit%u  muteLED=%d\n",
                r, g, b, small_, large_, playerBit, ps5.mute ? 1 : 0);
}

void setup() {
  Serial.begin(115200);
  vTaskDelay(pdMS_TO_TICKS(500));
  Serial.println(F("\n[BOOT] esp-ps5 testEverything"));
  Serial.println(F("[BOOT] ps5.begin(20): scanning up to 20s, connects on first DualSense seen."));
  ps5.begin(20);   // auto-scan + pair (early-exit) + auto-reconnect
}

void loop() {
  static uint8_t  rOut = 255, gOut = 255, bOut = 255;
  static uint8_t  smallR = 0, largeR = 0, playerBit = 0;
  static int      lastConn = -1;
  static uint32_t tSend = 0, tPlayer = 0, tReport = 0;

  int conn = ps5.isConnected() ? 1 : 0;
  if (conn != lastConn) {
    Serial.printf("[STATUS] connection %s\n", conn ? "UP" : "DOWN");
    lastConn = conn;
  }
  if (!conn) { vTaskDelay(pdMS_TO_TICKS(100)); return; }

  uint32_t now = millis();
  if (now - tPlayer >= kPlayerMs) { tPlayer = now; playerBit = (playerBit + 1) % 5; }

  if (now - tSend >= kSendMs) {
    tSend = now;
    stickToLightbar(ps5.lx, ps5.ly, rOut, gOut, bOut);
    stickToRumble  (ps5.rx, ps5.ry, smallR, largeR);
    // MARK: chained output - all four set* in one fluent call.
    ps5.led(rOut, gOut, bOut)
       .rumble(smallR, largeR)
       .playerLeds(1u << playerBit)
       .muteLed(ps5.mute ? 1 : 0)
       .send();
  }

  if (now - tReport >= kReportMs) {
    tReport = now;
    snapshot(rOut, gOut, bOut, smallR, largeR, playerBit);
  }
}
