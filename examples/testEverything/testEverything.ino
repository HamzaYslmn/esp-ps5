// testEverything.ino - PS5 DualSense full-feature test sketch.
//
// Wire-up: Bluetooth Classic on an ESP32-WROOM-32. Pick the
//   "Huge APP" partition scheme so Bluedroid fits.
//
// Mappings:
//   LEFT  stick  -> lightbar (Wikipedia RGB wheel: up=red, BR=green, BL=blue)
//   RIGHT stick  -> rumble (RX+ right motor, RX- left motor, |RY| both)
//   MUTE button  -> short click cycles mute LED (off / solid / pulse)
//                   long press (>= 600 ms) toggles real mic hardware mute
//   SHARE button -> on/off toggle: release LEDs to firmware, press again to take back
//   Player LEDs  -> march left -> right every 600 ms
//
// The serial monitor prints a tidy 1 Hz snapshot of every control,
// laid out so left-side and right-side controls sit side by side.

#include <ps5Controller.h>

static const uint32_t kSendMs   = 30;
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

// MARK: stickHueSat - LX/LY -> Wikipedia RGB color wheel (up=red). Writes ps5.output.{r,g,b}.
static void stickToLightbar(int8_t lx, int8_t ly) {
  float fx = (float)lx / 127.0f, fy = (float)ly / 127.0f;
  float mag = sqrtf(fx*fx + fy*fy); if (mag > 1.0f) mag = 1.0f;
  /* ly is up=negative, so flip into screen coords (up=positive) for atan2. */
  float hue = atan2f(fx, -fy) * 180.0f / (float)M_PI;
  if (hue < 0.0f) hue += 360.0f;
  const float kDead = (float)kStickDeadzone / 127.0f;
  float sat = (mag <= kDead) ? 0.0f : (mag - kDead) / (1.0f - kDead);
  hsvToRgb(hue, sat, 1.0f, ps5.output.r, ps5.output.g, ps5.output.b);
}

// MARK: stickToRumble - additive: RX+ right motor, RX- left, |RY| both. Writes ps5.output.{smallRumble,largeRumble}.
static int rescaleAxis(int v) {
  int a = (v >= 0) ? v : -v; if (a > 127) a = 127;
  return (a > kStickDeadzone) ? ((a - kStickDeadzone) * 127 / (127 - kStickDeadzone)) : 0;
}
static void stickToRumble(int8_t rx, int8_t ry) {
  int absX = rescaleAxis(rx);
  int absY = rescaleAxis(ry);
  int rightX = (rx > 0) ? absX : 0;
  int leftX  = (rx < 0) ? absX : 0;
  int sR = (rightX + absY) * 2; if (sR > 255) sR = 255;
  int lR = (leftX  + absY) * 2; if (lR > 255) lR = 255;
  ps5.output.smallRumble = (uint8_t)sR;
  ps5.output.largeRumble = (uint8_t)lR;
}

// CROSS-button on/off toggle for releaseLeds(). Defined here so snapshot()
// (above) and handleLedsRelease() (below) can both see it.
static bool ledsReleased = false;

// MARK: snapshot - compact, list-friendly view for the Arduino IDE serial
// monitor. Only "interesting" stuff is printed: pressed buttons are listed
// by name, idle ones are skipped, motion/touch only show when active.
static void appendBtn(String& s, bool on, const char* name) {
  if (on) { if (s.length()) s += ' '; s += name; }
}

static void snapshot() {
  Serial.println();
  Serial.printf("PS5  link=%s  bat=%u%% %s%s  hp=%d  mic=%d/%d\n",
                ps5.isConnected() ? "UP" : "DOWN",
                ps5.battery,
                ps5.charging ? "CHG" : "BAT",
                ps5.chargingError ? "!" : "",
                ps5.headphones, ps5.micJack, ps5.micMuted);

  Serial.printf("Sticks  L=(%+4d,%+4d) R=(%+4d,%+4d)   Triggers  L2=%3u R2=%3u\n",
                ps5.lx, ps5.ly, ps5.rx, ps5.ry, ps5.l2, ps5.r2);

  String btns;
  appendBtn(btns, ps5.up,        "Up");
  appendBtn(btns, ps5.down,      "Down");
  appendBtn(btns, ps5.left,      "Left");
  appendBtn(btns, ps5.right,     "Right");
  appendBtn(btns, ps5.triangle,  "Tri");
  appendBtn(btns, ps5.circle,    "Cir");
  appendBtn(btns, ps5.cross,     "Cro");
  appendBtn(btns, ps5.square,    "Sqr");
  appendBtn(btns, ps5.l1,        "L1");
  appendBtn(btns, ps5.r1,        "R1");
  appendBtn(btns, ps5.l3,        "L3");
  appendBtn(btns, ps5.r3,        "R3");
  appendBtn(btns, ps5.share,     "Share");
  appendBtn(btns, ps5.options,   "Options");
  appendBtn(btns, ps5.ps_btn,    "PS");
  appendBtn(btns, ps5.touchpad,  "Touchpad");
  appendBtn(btns, ps5.mute,      "Mute");
  Serial.printf("Pressed  %s\n", btns.length() ? btns.c_str() : "(none)");

  for (int i = 0; i < 2; i++)
    if (ps5.TouchActive(i))
      Serial.printf("Touch%d   x=%u y=%u id=%u\n",
                    i, ps5.TouchX(i), ps5.TouchY(i), ps5.TouchId(i));

  static const char* mlName[3] = { "off", "solid", "pulse" };
  uint8_t pm = ps5.output.playerLeds & 0x1F;
  Serial.printf("Output   lightbar=#%02X%02X%02X  rumble=%u/%u  player=%c%c%c%c%c  muteLED=%s  micHW=%s  release=%s\n",
                ps5.output.r, ps5.output.g, ps5.output.b,
                ps5.output.smallRumble, ps5.output.largeRumble,
                (pm>>0)&1?'o':'.', (pm>>1)&1?'o':'.', (pm>>2)&1?'o':'.',
                (pm>>3)&1?'o':'.', (pm>>4)&1?'o':'.',
                mlName[ps5.output.muteLed > 2 ? 0 : ps5.output.muteLed],
                ps5.output.micMute ? "on" : "off",
                ledsReleased       ? "on" : "off");
}

// ============================================================================
// MARK: shared loop state - sketch-only flags. Output state lives in ps5.output.
// ============================================================================
static uint8_t  briApi = 3;                // 1=dim, 2=mid, 3=bright. Shared between handleBrightness + tickPlayerLed.
static int      lastConn = -1;
static uint32_t tSend = 0, tReport = 0;

// Edge detection is built into each Button: ps5.<button>.pressed / .released.

// MARK: handleConnection - log up/down transitions, return true if connected.
// Library auto-drains all button edges on the first input packet, so the
// sketch only needs to reset its own toggle state (ledsReleased).
static bool handleConnection() {
  int conn = ps5.isConnected() ? 1 : 0;
  if (conn != lastConn) {
    Serial.printf("[STATUS] connection %s\n", conn ? "UP" : "DOWN");
    if (conn) ledsReleased = false;
    lastConn = conn;
  }
  if (!conn) { vTaskDelay(pdMS_TO_TICKS(100)); return false; }
  return true;
}

// MARK: tickPlayerLed - TRIANGLE advances the player-LED bitmask 0..31
// (5 bits, one per LED). 0 = all off, 1 = only far-left, 31 = all five.
// Re-applies via playerLed(idx, val) so the current brightness sticks.
static void tickPlayerLed() {
  if (!ps5.triangle.pressed) return;
  uint8_t mask = (uint8_t)((ps5.output.playerLeds + 1) & 0x1F);
  for (uint8_t i = 1; i <= 5; i++) {
    bool on = (mask >> (i - 1)) & 1;
    ps5.playerLed(i, on ? briApi : 0);
  }
  Serial.printf("[LEDS] mask=%u (0b%u%u%u%u%u)\n", mask,
                (mask>>4)&1,(mask>>3)&1,(mask>>2)&1,
                (mask>>1)&1, mask&1);
}

// MARK: handleAdaptiveTriggers - L1 = prev, R1 = next. One step per press.
// 0=Off, 1=Rigid, 2=Trigger, 3=Pulse, 4=Bow, 5=Galloping, 6=Machine.
// Modes 4..6 are firmware-built combos (Bow=Trigger+snap, Machine=Trigger range+Pulse).
// All parameters are percent (0..100), freq is Hz.
static void handleAdaptiveTriggers() {
  static const char* trigName[7] = { "Off", "Rigid", "Trigger", "Pulse", "Bow", "Galloping", "Machine" };
  static int8_t trigMode    = 0;
  bool          trigChanged = false;
  if (ps5.r1.pressed) { trigMode = (int8_t)((trigMode + 1) % 7);       trigChanged = true; }
  if (ps5.l1.pressed) { trigMode = (int8_t)((trigMode + 7 - 1) % 7);   trigChanged = true; }
  if (!trigChanged) return;
  switch (trigMode) {
    case 0: ps5.l2Off()                              .r2Off();                                break;
    //                  startPct, strengthPct
    case 1: ps5.l2Rigid    (10, 50)                  .r2Rigid    (50, 100);                   break;  // wall begins / wall force
    //                    startPct, endPct, strengthPct
    case 2: ps5.l2Trigger  (90, 100, 100)            .r2Trigger  (50, 60, 100);               break;  // squeeze range / max force
    //                  startPct, strengthPct, freqHz
    case 3: ps5.l2Pulse    (20, 100, 5)              .r2Pulse    (20, 100, 10);               break;  // buzz begins / how strong / how often (Hz)
    //                startPct, endPct, strengthPct, snapPct
    case 4: ps5.l2Bow      (20, 70, 100, 100)        .r2Bow      (20, 70, 100, 100);          break;  // squeeze range / squeeze force / snap force
    //                      startPct, endPct, foot1Pct, foot2Pct, freqHz
    case 5: ps5.l2Galloping(20, 80, 30, 60, 8)       .r2Galloping(20, 80, 30, 60, 8);         break;  // zone range / 2 beat positions / gallop Hz
    //                    startPct, endPct, ampAPct, ampBPct, freqHz, periodTenths
    case 6: ps5.l2Machine  (20, 90, 30, 100, 12, 5)  .r2Machine  (20, 90, 30, 100, 12, 5);    break;  // buzz zone / soft buzz 30% / loud buzz 100% / 12 Hz / swap every 0.5 s
  }
  Serial.printf("[TRIG] %d/6  %s\n", trigMode, trigName[trigMode]);
}

// MARK: handleBrightness - SQUARE cycles player-LED brightness BRIGHT -> MID -> DIM.
// Re-applies the current LED mask with the new brightness via playerLed(idx, val).
static void handleBrightness() {
  if (!ps5.square.pressed) return;
  briApi = (briApi == 3) ? 2 : (briApi == 2 ? 1 : 3);   // 3 -> 2 -> 1 -> 3
  uint8_t mask = ps5.output.playerLeds;
  for (uint8_t i = 1; i <= 5; i++) {
    bool on = (mask >> (i - 1)) & 1;
    ps5.playerLed(i, on ? briApi : 0);      // last non-zero call sets global brightness
  }
  static const char* briName[4] = { "OFF", "DIM", "MID", "BRIGHT" };
  Serial.printf("[BRI ] %s\n", briName[briApi]);
}

// MARK: handleForget - OPTIONS clears the in-RAM target so the next
// begin() rescans from scratch. (Bluedroid still keeps the link key in
// its own NVS, so a paired pad stays paired across reboots regardless.)
static void handleForget() {
  if (!ps5.options.pressed) return;
  ps5.forget();
  Serial.println(F("[FGT ] target cleared - next begin() will rescan"));
}

// MARK: handleMute - short click cycles mute LED off/solid/pulse,
// long press (>= 600 ms) toggles the real mic hardware mute. The two
// are independent on the wire: muteLed is just a visual, micMute()
// gates the actual microphone capture via the power-save register.
static void handleMute() {
  static uint32_t pressedAt = 0;
  if (ps5.mute.pressed)  pressedAt = millis();
  if (!ps5.mute.released) return;
  uint32_t held = millis() - pressedAt;
  if (held >= 600) {
    ps5.micMute(!ps5.output.micMute);
    Serial.printf("[MUTE] hw mic = %s (held %lu ms)\n",
                  ps5.output.micMute ? "MUTED" : "LIVE", (unsigned long)held);
  } else {
    static const char* mn[3] = { "OFF", "SOLID", "PULSE" };
    uint8_t next = (uint8_t)((ps5.output.muteLed + 1) % 3);
    ps5.muteLed(next);
    Serial.printf("[MUTE] led = %s\n", mn[next]);
  }
}

// MARK: handleLedsRelease - CROSS button is an on/off toggle: first press
// hands lightbar + player LEDs back to firmware (boot animation returns);
// second press lets the sketch take control again (next pushOutputs sets
// lightbar from the L-stick and player LEDs from tickPlayerLed).
// IMPORTANT: while `ledsReleased` is true we MUST stop sending output
// reports entirely. The firmware reads RELEASE_LEDS as "host is ceding
// LED control" - if we keep streaming output frames the link drops.
// pushOutputs() honours this by early-returning while the toggle is on.
static void handleLedsRelease() {
  if (!ps5.cross.pressed) return;
  ledsReleased = !ledsReleased;
  if (ledsReleased) {
    ps5.releaseLeds().send();                         // fire ONCE, then go silent
    Serial.println(F("[CROS] LEDs -> firmware"));
  } else {
    Serial.println(F("[CROS] LEDs -> sketch"));
  }
}

// MARK: handleRawDump - TOUCHPAD click prints the latest 78-byte BT 0x31
// input report as hex. Handy when reverse-engineering new fields
// (mic / speaker / future firmware bytes).
static void handleRawDump() {
  if (!ps5.touchpad.pressed || !ps5.latestPacket) return;
  Serial.print(F("[RAW ] "));
  for (int i = 0; i < 78; i++) Serial.printf("%02x ", ps5.latestPacket[i]);
  Serial.println();
}

// MARK: pushOutputs - throttled (kSendMs) chained send: lightbar + rumble.
// muteLed / micMute are toggled by handleMute() and persist in ps5.output
// between sends, so we don't re-write them here. Skipped entirely while
// the user has handed LEDs back to firmware via CROSS - any output frame
// in that state confuses the firmware and tears the link down.
static void pushOutputs(uint32_t now) {
  if (now - tSend < kSendMs) return;
  tSend = now;
  if (ledsReleased) return;                                // GO SILENT - do not send anything
  stickToLightbar(ps5.lx, ps5.ly);
  stickToRumble  (ps5.rx, ps5.ry);
  ps5.send();
}

// MARK: pushReport - 1 Hz tidy serial dump of all input + output state.
static void pushReport(uint32_t now) {
  if (now - tReport < kReportMs) return;
  tReport = now;
  snapshot();
}

void setup() {
  Serial.begin(115200);
  vTaskDelay(pdMS_TO_TICKS(500));
  Serial.println(F("\n[BOOT] esp-ps5 testEverything"));
  Serial.println(F("[BOOT] ps5.begin(20): tries saved MAC from NVS first, else scans up to 20s."));
  Serial.println(F("[BOOT] Controls:  L1/R1 = prev/next adaptive-trigger mode,  SQUARE = cycle player-LED brightness."));
  Serial.println(F("[BOOT]            TRIANGLE = step player-LED bitmask 0..31,  TOUCHPAD click = dump raw bytes,  OPTIONS = forget saved MAC."));
  Serial.println(F("[BOOT]            MUTE click = cycle LED off/solid/pulse, MUTE long >=600ms = toggle hw mic mute.  SHARE = release LEDs on/off toggle."));
  ps5.begin(20);   // tries saved MAC -> scan -> pair (early-exit) + auto-reconnect
}

void loop() {
  if (!handleConnection()) return;

  uint32_t now = millis();
  tickPlayerLed();
  handleAdaptiveTriggers();
  handleBrightness();
  handleForget();
  handleMute();
  handleLedsRelease();
  handleRawDump();
  pushOutputs(now);
  pushReport(now);
}
