# esp-ps5

**Talk to a PS5 DualSense controller from your ESP32 over Bluetooth.**

Read the buttons. Light up the bar. Make it rumble. That's it.

```cpp
#include <ps5Controller.h>

void setup() {
  Serial.begin(115200);
  ps5.begin(20);              // wait up to 20 seconds for a controller
}

void loop() {
  if (!ps5.isConnected()) return;

  if (ps5.cross.pressed)      Serial.println("X!");
  if (ps5.l1)                 ps5.lightbar(255, 0, 0).rumble(180, 0).send();
  else                        ps5.lightbar(0, 0, 0).rumble(0, 0).send();

  delay(20);                  // don't .send() faster than ~10 ms
}
```

That's a full sketch. No callbacks, no event objects — the buttons are just fields on a global `ps5` object. Read them like normal variables.

> Tags: **PS5** · **DualSense** · **ESP32** · **Bluetooth Classic** · **Gamepad** · **Sony** · **HID**

---

## Preview

<img width="540" height="403" alt="ezgif com-speed" src="https://github.com/user-attachments/assets/af5aa939-e6fa-472d-adec-05e6ba25314e" />

---

## Install

1. Drop this folder into `~/Documents/Arduino/libraries/esp-ps5`.
2. *Tools → Board → ESP32 Dev Module*.
3. *File → Examples → esp-ps5 → testEverything* and upload.

**Tested on:** ESP32-WROOM-32 / Arduino-ESP32 3.3.6. We compiled with the *Huge APP* partition (Bluedroid is large) — that's not mandatory, just what we used.

---

## Pair the controller

1. Hold **PS + Create** for ~3 seconds. The lightbar will pulse white — it's now broadcasting.
2. Power up your ESP32 with `ps5.begin(20)` in `setup()`.
3. The library finds the controller (~1–3 s) and connects.

Pairing is remembered across reboots, so you only do this once per controller. If the controller goes to sleep or wanders out of range, the library quietly reconnects in the background.

---

## What you can read (input)

Every field below lives on the global `ps5` object and refreshes ~250 times per second.

### Sticks

```cpp
ps5.lx, ps5.ly      // left stick   -128 .. +127  (push UP = negative, 0 = center)
ps5.rx, ps5.ry      // right stick  same scale
ps5.lxPct(), ...    // same thing in -100..+100 if you prefer
```

### Triggers

```cpp
ps5.l2, ps5.r2      // 0 = released, 255 = fully pressed
ps5.l2Pct(), ...    // 0..100 if you prefer
```

### Buttons

Every button works **three** ways:

```cpp
if (ps5.square)             // true while held
if (ps5.square.pressed)     // fires ONCE when you push it down
if (ps5.square.released)    // fires ONCE when you let it go
```

`.pressed` and `.released` are **latched** — they stay true until you read them, so a slow loop never misses a click.

The buttons:

| Group | Names |
|---|---|
| D-pad | `up`  `down`  `left`  `right` |
| Face | `cross`  `circle`  `square`  `triangle` |
| Shoulders | `l1`  `r1` |
| Stick clicks | `l3`  `r3` |
| System | `share`  `options`  `ps_btn`  `touchpad`  `mute` |

Diagonal D-pad? Just AND them: `if (ps5.up && ps5.right)`.

### Touchpad

Two fingers max. Surface is 1920 × 1080.

```cpp
ps5.TouchActive(0)   // is finger 0 down?
ps5.TouchX(0)        // 0..1919
ps5.TouchY(0)        // 0..1079
ps5.TouchId(0)       // counts up each new touch
```

### Battery & jacks

```cpp
ps5.battery          // 0..100  (percent)
ps5.charging         // true = USB plugged in
ps5.fullyCharged     // true = 100 %
ps5.chargingError    // true = controller refuses to charge (over-/under-volt, temp, fault)
ps5.headphones       // true = something in the 3.5 mm jack
ps5.micJack          // true = mic detected
ps5.micMuted         // true = user has tapped the mute button (firmware-persisted)
```

### Raw packet (debug / reverse-engineering)

```cpp
ps5.latestPacket     // const uint8_t* to the last 78-byte BT 0x31 input report
                     // valid right after a packet arrives. handy when you're
                     // reverse-engineering new fields (mic, speaker, etc).
```

### Motion (gyro / accelerometer)

```cpp
ps5.gyroX, gyroY, gyroZ      // raw int16. divide by 1024 for deg/sec
ps5.accelX, accelY, accelZ   // raw int16. divide by 8192 for g (gravity included)
ps5.sensorTime               // microsecond-ish timestamp
```

> Lying flat, one accel axis reads ~8192 (that's gravity, 1 g). To detect motion, threshold `|magnitude − 8192|`.

---

## What you can do (output)

Every output is a chainable setter. **Stage** what you want, then **send** it:

```cpp
ps5.lightbar(0, 255, 0)       // RGB lightbar
   .rumble(255, 100)          // small motor (sharp), large motor (deep). 0..255
   .playerLed(3, 3)           // light LED #3 at brightness 3
   .muteLed(2)                // 0=off, 1=on, 2=pulse
   .send();                   // push the frame. don't go faster than ~10 ms.
```

| Method | What it does |
|---|---|
| `.lightbar(r, g, b)` | RGB strip around the touchpad. 0..255 each. |
| `.rumble(small, large)` | small = sharp buzz, large = deep rumble. 0..255 each. |
| `.playerLed(idx, val)` | LED 1..5 (1 = far-left). `val`: 0=off, 1=dim, 2=mid, 3+=bright. |
| `.muteLed(mode)` | Mic-mute LED visual. 0=off, 1=on, 2=pulse. |
| `.micMute(on)` | Electrically mute / un-mute the mic hardware (separate from the LED above). **Default: ON** (mic off + power-save bit enabled). Call `ps5.micMute(false)` if you ever wire up audio capture. |
| `.releaseLeds()` | One-shot: hand lightbar + player LEDs back to firmware control (boot animation returns). |
| `.send()` | Push everything. Call at most every ~10 ms. |

> **Player-LED brightness is global.** The last non-zero `val` you pass to `playerLed()` before `.send()` sets the brightness for every lit LED in that frame.

### Adaptive triggers — making L2 / R2 feel like things

The DualSense triggers can pretend to be a wall, a gun, a vibrating pad, etc. Each trigger plays **one** effect at a time. L2 and R2 are independent — you can give them different effects.

```cpp
ps5.l2Trigger(20, 80, 100)    // gun-trigger squeeze on left
   .r2Pulse(30, 100, 15)      // buzzing right trigger
   .send();
```

Numbers below are all **percent (0..100)** unless noted. `freqHz` is real Hz (try 5–30).

#### Basic effects

| Method | Feels like | What the args mean |
|---|---|---|
| `.l2Off()` | Free trigger | — |
| `.l2Rigid(start, strength)` | A wall past `start` | how far in / how stiff |
| `.l2Trigger(start, end, strength)` | Gun trigger that breaks | range / how heavy |
| `.l2Pulse(start, strength, freqHz)` | Vibrating past `start` | how far in / how strong / how fast |

#### Combo effects (firmware presets)

| Method | Feels like | What the args mean |
|---|---|---|
| `.l2Bow(start, end, strength, snap)` | Drawing a bow + snap-back | range / squeeze / snap |
| `.l2Galloping(start, end, foot1, foot2, freqHz)` | Horse gallop two-beat | range / two beat positions / speed |
| `.l2Machine(start, end, ampA, ampB, freqHz, periodTenths)` | Buzzer that swaps strength | range / two strengths / speed / swap period in 0.1 s |

The same methods exist as `.r2*()` for the right trigger.

> **Want pulse + trigger together on the same trigger?** You can't — only one effect per trigger. But `l2Machine` (buzz inside a range) is the closest fused version. Or just put pulse on L2 and trigger on R2.

---

## Connection helpers

| Call | What it does |
|---|---|
| `ps5.begin(20)` | Bring up Bluetooth, scan up to 20 s, auto-connect. |
| `ps5.begin("AA:BB:CC:DD:EE:FF")` | Connect to a known MAC, skip the scan. |
| `ps5.isConnected()` | True while packets are flowing. Auto-reconnects if not. Call every loop. |
| `ps5.forget()` | Drop the latched controller; next `begin()` rescans. |
| `ps5.scanDevices(secs, cb)` | Manual scan; calls `cb(mac, name, rssi)` per device. Doesn't connect. |

---

## Tips

- Call `ps5.isConnected()` every loop — it's also the auto-reconnect heartbeat.
- Don't `send()` faster than every ~10 ms or the BT queue will jam.
- Don't block `loop()` for long; the radio runs on FreeRTOS tasks underneath.
- The accelerometer always reads ~1 g of gravity even at rest. That's normal.

---

## Wire format (for the curious)

DualSense talks **HID over L2CAP** on PSMs **0x11 (control)** + **0x13 (interrupt)**. The library:

1. After both channels are up, sends a magic feature-set on PSM 0x11 to flip the controller into the full BT 0x31 report mode.
2. Reads input report `0x31` (78 bytes) on PSM 0x13.
3. Writes output report `0x31` (79 bytes incl. `0xA2` HID header + CRC32) on PSM 0x13.

### Input report — 78 bytes

Aligned with Linux's `drivers/hid/hid-playstation.c`.

| Byte | Field |
|---:|---|
| 0 | report id `0x31` |
| 1 | reserved tag |
| 2 / 3 | LX / LY (0..255, 128 = center, Y inverted) |
| 4 / 5 | RX / RY |
| 6 / 7 | L2 / R2 trigger pressure |
| 8 | seq number |
| 9 | low nibble = D-pad hat, high nibble = ◯ △ ✕ □ |
| 10 | L1, R1, L2, R2, Create, Options, L3, R3 |
| 11 | bit 0 = PS, bit 1 = Touchpad, bit 2 = Mic-Mute |
| 17..22 | gyro x, y, z (le16) |
| 23..28 | accel x, y, z (le16) |
| 29..32 | sensor timestamp (le32, 0.33 µs/LSB) |
| 34..41 | touchpad: 2 contacts × 4 bytes |
| 54 | low nibble = battery 0..10, high nibble = charging state |
| 55 | HP detect, mic detect, mic-mute |
| 74..77 | CRC32 LE (not verified on input) |

### Output report — 79 bytes

| Byte | Field |
|---:|---|
| 0 | `0xA2` HID DATA \| OUTPUT header (covered by CRC) |
| 1 | report id `0x31` |
| 2 | seq tag (high nibble = sequence 0..15) |
| 3 | tag `0x10` (required DualSense BT marker) |
| 4 | valid_flag0 — vibration + haptics + L2/R2 adaptive enables |
| 5 | valid_flag1 — mic-mute LED + lightbar + player LED enables |
| 6 / 7 | motor_right / motor_left rumble |
| 12 | mute LED |
| 14..24 | R2 adaptive trigger (1 mode + 10 params) |
| 25..35 | L2 adaptive trigger (1 mode + 10 params) |
| 42 | valid_flag2 — brightness + lightbar setup enables |
| 45 | lightbar setup byte |
| 46 | player LED brightness (0=bright, 1=mid, 2=dim) |
| 47 | player LED bitmask |
| 48..50 | lightbar R, G, B |
| 75..78 | CRC32 LE of bytes 0..74 (poly `0xEDB88320`) |

---

## Files

```
src/
  ps5Controller.h     public Arduino API
  ps5Controller.cpp   begin/scan/auto-reconnect + fluent setters
  ps5_bytes.cpp       protocol parser (input) + frame builder (output)
  bluedroid/          L2CAP transport glue + minimal vendored headers
examples/testEverything/   full-feature demo sketch
```

---

## License & credits

**License:** LGPL-3.0 — see [LICENSE](LICENSE).

**Author:** [hamzayslmn](https://github.com/hamzayslmn)

DualSense, PlayStation, and PS5 are trademarks of Sony Interactive Entertainment. This is an independent, unofficial implementation, not affiliated with or endorsed by Sony.
