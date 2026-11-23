// testEverything.ino - every esp-ps5 feature on one sketch.
//
//   Left stick  -> lightbar colour wheel (white at centre)
//   Right stick -> rumble
//   L1 / R1     -> adaptive trigger mode  TRIANGLE    -> next player-LED pattern
//   SQUARE      -> player-LED brightness  CIRCLE      -> melody (haptic actuators)
//   MUTE        -> mute LED off/on/pulse  OPTIONS     -> forget (unpair) controller
//   CROSS (X)   -> give LEDs to the pad firmware / take them back
//   CREATE      -> power the controller off
//   TOUCHPAD    -> dump the raw input report
//
// loop() never blocks: work is paced with millis(), the library runs on the BT task.
//
// First time: hold PS + Create on the controller until the lightbar flashes.

#include <ps5Controller.h>

static const ps5Controller::Note kTune[] = {
  {523, 120}, {659, 120}, {784, 120}, {1047, 240}, {0, 60}, {784, 120}, {1047, 360},
};

void setup() {
  Serial.begin(115200);
  ps5.attachOnConnect([] { Serial.println("connected"); });
  ps5.attachOnDisconnect([] { Serial.println("disconnected"); });
  ps5.begin();
}

static void triggers() {
  static int mode = 0;
  if (ps5.r1.pressed) mode = (mode + 1) % 7;
  else if (ps5.l1.pressed) mode = (mode + 6) % 7;
  else return;
  switch (mode) {   // percent args, Hz where noted
    case 0: ps5.l2Off().r2Off(); break;
    case 1: ps5.l2Rigid(10, 50).r2Rigid(50, 100); break;
    case 2: ps5.l2Trigger(60, 90, 100).r2Trigger(50, 60, 100); break;
    case 3: ps5.l2Pulse(20, 100, 5).r2Pulse(20, 100, 10); break;
    case 4: ps5.l2Bow(20, 70, 100, 100).r2Bow(20, 70, 100, 100); break;
    case 5: ps5.l2Galloping(20, 80, 30, 60, 8).r2Galloping(20, 80, 30, 60, 8); break;
    case 6: ps5.l2Machine(20, 90, 30, 100, 12, 5).r2Machine(20, 90, 30, 100, 12, 5); break;
  }
  Serial.printf("trigger mode %d\n", mode);
}

static void buttons() {
  static uint8_t leds = 0, bright = 3;
  if (ps5.triangle.pressed) leds = (leds + 1) & 0x1F;
  if (ps5.square.pressed)   bright = bright % 3 + 1;                 // 1 dim, 2 mid, 3 bright
  for (int i = 1; i <= 5; i++) ps5.playerLed(i, (leds >> (i - 1)) & 1 ? bright : 0);

  if (ps5.mute.pressed)     ps5.muteLed((ps5.output.muteLed + 1) % 3);
  if (ps5.circle.pressed)   ps5.play(kTune, sizeof(kTune) / sizeof(kTune[0]));
  if (ps5.options.pressed)  ps5.forget();
  if (ps5.share.pressed)    ps5.powerOff();
  if (ps5.cross.pressed) {
    ps5.releaseLeds(!ps5.output.releaseLeds);
    Serial.println(ps5.output.releaseLeds ? "LEDs -> firmware" : "LEDs -> sketch");
  }
  if (ps5.touchpad.pressed) {
    for (uint8_t b : ps5.latestPacket) Serial.printf("%02x ", b);
    Serial.println();
  }
}

static uint8_t mag(int8_t v) { return abs(v) * 255 / 128; }   // stick -> 0..255

// Colour wheel: centre = white, angle = hue (up red, R/G/B 120 deg apart), push = saturation.
static void stickColour(int8_t x, int8_t y) {
  float h = atan2f(x, -y) * 180 / M_PI, sat = min(1.0f, hypotf(x, y) / 127);
  auto ch = [&](float n) {                          // standard HSV->RGB per channel, v = 1
    float k = fmodf(n + h / 60 + 6, 6);
    return (uint8_t)(255 * (1 - sat * max(0.0f, min(min(k, 4 - k), 1.0f))));
  };
  ps5.lightbar(ch(5), ch(3), ch(1));
}

static void report() {
  Serial.printf("L(%4d,%4d) R(%4d,%4d) L2 %3u R2 %3u  bat %u%%%s  fw %x.%02x\n",
                ps5.lx, ps5.ly, ps5.rx, ps5.ry, ps5.l2, ps5.r2, ps5.battery,
                ps5.charging ? " chg" : "", ps5.firmware >> 8, ps5.firmware & 0xFF);
  Serial.printf("  gyro %7.1f %7.1f %7.1f deg/s  accel %5.2f %5.2f %5.2f g  touch %s\n",
                ps5.gx, ps5.gy, ps5.gz, ps5.ax, ps5.ay, ps5.az, ps5.touch[0].active ? "yes" : "no");
}

void loop() {
  if (!ps5.isConnected()) return;
  triggers();
  buttons();

  static uint32_t tSend = 0, tReport = 0;
  uint32_t now = millis();
  if (now - tSend >= 20) {                   // 50 Hz output is plenty
    tSend = now;
    stickColour(ps5.lx, ps5.ly);
    ps5.rumble(mag(ps5.rx), mag(ps5.ry)).send();
  }
  if (now - tReport >= 1000) { tReport = now; report(); }
}
