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

// MARK: pickedMac - first DualSense / Wireless Controller seen during scan.
static char gPickedMac[18] = {0};

static void onScan(const uint8_t mac[6], const char* name, int8_t rssi) {
  Serial.printf("  %02x:%02x:%02x:%02x:%02x:%02x  rssi=%4d  %s\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                rssi, (name && name[0]) ? name : "(unknown)");
  if (gPickedMac[0]) return;
  if (!name) return;
  if (!strstr(name, "DualSense") && !strstr(name, "Wireless Controller")) return;
  snprintf(gPickedMac, sizeof(gPickedMac), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

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
  Serial.printf("LS:    x=%+4d y=%+4d  click=%d\n",
                ps5.LStickX(), ps5.LStickY(), ps5.L3());
  Serial.printf("RS:    x=%+4d y=%+4d  click=%d\n",
                ps5.RStickX(), ps5.RStickY(), ps5.R3());
  Serial.printf("TRIG:  L1=%d L2=%d (val=%3u)   R1=%d R2=%d (val=%3u)\n",
                ps5.L1(), ps5.L2(), ps5.L2Value(),
                ps5.R1(), ps5.R2(), ps5.R2Value());
  Serial.printf("DPAD:  U=%d D=%d L=%d R=%d\n",
                ps5.Up(), ps5.Down(), ps5.Left(), ps5.Right());
  Serial.printf("FACE:  Tri=%d Cir=%d Cro=%d Sq=%d\n",
                ps5.Triangle(), ps5.Circle(), ps5.Cross(), ps5.Square());
  Serial.printf("META:  Share=%d Opt=%d PS=%d Touch=%d Mute=%d\n",
                ps5.Share(), ps5.Options(), ps5.PSButton(), ps5.Touchpad(), ps5.Mute());
  Serial.printf("GYRO:  x=%+6d y=%+6d z=%+6d\n",
                ps5.GyroX(), ps5.GyroY(), ps5.GyroZ());
  Serial.printf("ACCEL: x=%+6d y=%+6d z=%+6d\n",
                ps5.AccelX(), ps5.AccelY(), ps5.AccelZ());
  for (int i = 0; i < 2; i++) {
    if (ps5.TouchActive(i))
      Serial.printf("TOUCH%d: x=%4u y=%4u id=%u\n",
                    i, ps5.TouchX(i), ps5.TouchY(i), ps5.TouchId(i));
    else
      Serial.printf("TOUCH%d: idle\n", i);
  }
  Serial.printf("BAT:   %u0%% %s  HP=%d MIC=%d\n",
                ps5.Battery(), ps5.Charging() ? "CHG" : "DSCH",
                ps5.Headphones(), ps5.MicJack());
  Serial.printf("OUT:   rgb=(%3u,%3u,%3u)  rumble=(%3u,%3u)  player=bit%u  muteLED=%d\n",
                r, g, b, small_, large_, playerBit, ps5.Mute() ? 1 : 0);
}

void setup() {
  Serial.begin(115200);
  vTaskDelay(pdMS_TO_TICKS(500));
  Serial.println(F("\n[BOOT] esp-ps5 testEverything"));
  Serial.println(F("[BOOT] Scanning Bluetooth Classic for 5 s..."));
  ps5.scanDevices(5, onScan);
  if (!gPickedMac[0]) {
    Serial.println(F("[BOOT] No DualSense found. Hold PS+Create and reset."));
    return;
  }
  Serial.printf("[BOOT] Connecting to %s\n", gPickedMac);
  ps5.begin(gPickedMac);
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
    stickToLightbar(ps5.LStickX(), ps5.LStickY(), rOut, gOut, bOut);
    stickToRumble  (ps5.RStickX(), ps5.RStickY(), smallR, largeR);
    ps5.setLed(rOut, gOut, bOut);
    ps5.setRumble(smallR, largeR);
    ps5.setPlayerLeds(1u << playerBit);
    ps5.setMuteLed(ps5.Mute() ? 1 : 0);
    ps5.sendToController();
  }

  if (now - tReport >= kReportMs) {
    tReport = now;
    snapshot(rOut, gOut, bOut, smallR, largeR, playerBit);
  }
}
