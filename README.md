# esp-ps5

Use a PS5 DualSense controller with an ESP32 over Bluetooth Classic.

```cpp
#include <ps5Controller.h>

void setup() { Serial.begin(115200); ps5.begin(); }   // returns at once; connects in the background

void loop() {
  if (!ps5.isConnected()) return;
  if (ps5.cross.pressed) Serial.println("X!");
  ps5.lightbar(ps5.l2, 0, ps5.r2).rumble(0, ps5.l1 ? 200 : 0).send();
  delay(20);                                // send() at most every ~10 ms
}
```

<img width="540" height="403" alt="demo" src="https://github.com/user-attachments/assets/af5aa939-e6fa-472d-adec-05e6ba25314e" />

## Setup

- Arduino-ESP32 **3.3.x or 4.x**, board *ESP32 Dev Module* (needs Classic BT: ESP32, not S3/C3). Huge APP partition recommended.
- Pair once: hold **PS + Create** until the lightbar flashes, then boot the ESP32. The link key is kept across reboots; reconnects happen automatically every ~5 s.
- `ps5.begin("AA:BB:CC:DD:EE:FF")` skips the scan. `ps5.forget()` makes the next retry rescan.
- Full demo: *File → Examples → esp-ps5 → testEverything*.

## Input

```cpp
ps5.lx, ly, rx, ry          // -128..127, 0 = center, push UP = negative   (lxPct() ... = -100..100)
ps5.l2, r2                  // 0..255                                      (l2Pct() ... = 0..100)
ps5.cross                   // held?   also: circle square triangle up down left right
ps5.cross.pressed           // edge, latched until read. also .released
                            // l1 r1 l3 r3 share options ps_btn touchpad mute
ps5.touch[0].active/.x/.y   // 2 fingers, 1920 x 1080
ps5.gyroX..Z, accelX..Z     // raw; /1024 = deg/s, /8192 = g
ps5.battery                 // 0..100. also charging fullyCharged chargingError headphones micJack micMuted
ps5.latestPacket            // copy of the last raw 78-byte report
```

## Output (chain, then `send()`)

```cpp
ps5.lightbar(r, g, b)       // 0..255
   .rumble(small, large)    // 0..255
   .playerLed(1..5, v)      // 0 off, 1 dim, 2 mid, 3 bright (brightness is shared)
   .muteLed(m)              // 0 off, 1 on, 2 pulse
   .micMute(true)           // mic hardware off (default)
   .releaseLeds()           // one-shot: give LEDs back to firmware, then stop sending
   .send();
```

Adaptive triggers (percent args, `hz` in Hz, one effect per trigger, `r2*` mirrors `l2*`):

```cpp
l2Off()  l2Rigid(start, strength)  l2Trigger(start, end, strength)  l2Pulse(start, strength, hz)
l2Bow(start, end, strength, snap)  l2Galloping(start, end, foot1, foot2, hz)
l2Machine(start, end, ampA, ampB, hz, periodTenths)
```

## Callbacks (optional, run on the BT task, keep them short)

```cpp
ps5.attach(fn);              // every report, ~250 Hz
ps5.attachOnConnect(fn);     // first report after connecting
ps5.attachOnDisconnect(fn);
```

Protocol notes: [AGENTS.md](AGENTS.md). License: LGPL-3.0. DualSense / PS5 are trademarks of Sony Interactive Entertainment; this project is unofficial.
