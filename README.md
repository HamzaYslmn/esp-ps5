# esp-ps5

**Pair a Sony DualSense (PS5) controller to an ESP32 over Bluetooth Classic.**
Simple, robust, dependency-free. One header, one global object — that's it.

```cpp
#include <ps5Controller.h>

void setup() {
  Serial.begin(115200);
  ps5.begin();              // auto-scan, connect to the first DualSense seen
}

void loop() {
  if (!ps5.isConnected()) return;
  if (ps5.cross) ps5.rumble(255, 0).lightbar(255, 0, 0).send();
  int speed = ps5.ly;       // -128..+127, up = negative
  int trig  = ps5.r2;       //   0..255
  delay(50);
}
```

> Tags: **PS5** · **DualSense** · **ESP32** · **ESP** · **Bluetooth Classic** · **Gamepad** · **Sony** · **HID**

---

## Contents

1. [Tested setups](#tested-setups)
2. [Install](#install)
3. [Pair the controller](#pair-the-controller)
4. [API reference](#api-reference)
   - [Connection](#connection)
   - [Sticks & triggers](#sticks--triggers)
   - [Buttons](#buttons)
   - [Motion sensors](#motion-sensors)
   - [Touchpad](#touchpad)
   - [Battery & jacks](#battery--jacks)
   - [Outputs (chainable)](#outputs-chainable)
   - [Adaptive triggers](#adaptive-triggers)
5. [Things to watch for](#things-to-watch-for)
6. [Wire format reference](#wire-format-reference)
7. [Files](#files)
8. [License & credits](#license--credits)

---

## Tested setups

| Item | Version |
|---|---|
| Arduino-ESP32 core | **3.3.6** (= ESP-IDF 5.5.2) |
| Boards | **ESP32-WROOM-32D**, **ESP32-WROOM-32U** |
| Partition scheme | **Huge APP** (mandatory — Bluedroid is large) |
| Controller firmware | DualSense BT 0x31 protocol (any current FW) |

**Verified working on hardware:** pairing · auto-reconnect · all buttons · both sticks · both triggers (digital + analog) · D-pad · touchpad button · mic-mute capacitive button · both rumble motors · RGB lightbar · all 5 player LEDs · mic-mute LED · gyroscope · accelerometer · both touchpad fingers · battery percentage · charging state · headphone-jack detect · mic-jack detect.

---

## Install

1. Clone or download into `~/Documents/Arduino/libraries/esp-ps5`.
2. *Tools → Board → ESP32 Arduino → ESP32 Dev Module*.
3. *Tools → Partition Scheme → **Huge APP*** ← **mandatory.**
4. *File → Examples → esp-ps5 → testEverything*.

---

## Pair the controller

1. Hold **PS + Create** for 3 seconds — the lightbar pulses white.
2. Call `ps5.begin()` in `setup()`.
3. The library scans Bluetooth Classic and **connects the moment** any DualSense / Wireless Controller is seen — no need to know the MAC.

| Call | Behaviour |
|---|---|
| `ps5.begin()` | Auto-scan, default timeout **30 s**, connect on first match. |
| `ps5.begin(20)` | Same, but only scan up to **20 s**. |
| `ps5.begin("AA:BB:CC:DD:EE:FF")` | Connect to a specific MAC (skip scan). |

**Auto-reconnect** is built in: if the controller goes out of range or sleeps, the library quietly retries every 5 s in the background. Your sketch keeps running.

**Persistent re-pairing**: the first time a controller successfully connects, its MAC is saved to NVS. On the next boot, `ps5.begin()` skips the BT inquiry and fast-connects directly (~1 s). Falls back to a full scan if the saved controller is unreachable. Call `ps5.forget()` to wipe the saved MAC — useful when switching to a different controller.

---

## API reference

### Connection

| Member | Type | What it does |
|---|---|---|
| `ps5.begin([timeout])` | `bool` | Auto-scan + pair. |
| `ps5.begin(mac)` | `bool` | Pair a specific MAC. |
| `ps5.isConnected()` | `bool` | Live link state. Auto-reconnect runs while `false`. |
| `ps5.forget()` | — | Erase the saved MAC from NVS so the next `begin()` re-scans. |
| `ps5.attach(cb)` | — | `void cb()` fires on every input packet (~250 Hz). |
| `ps5.attachOnConnect(cb)` | — | Fires once when the link comes up. |
| `ps5.attachOnDisconnect(cb)` | — | Fires once when the link drops. |
| `ps5.scanDevices(secs, cb)` | `bool` | Manual inquiry; `cb(mac, name, rssi)` per device. |

> **Inputs are plain fields** — refreshed automatically every packet. Just read them like variables, no parentheses.

### Sticks & triggers

| Field | Type | Range | Notes |
|---|---|---|---|
| `ps5.lx` `ps5.ly` | `int8` | `-128 … +127` | Left stick. **Y is inverted** (up = negative). 0 = centered. |
| `ps5.rx` `ps5.ry` | `int8` | `-128 … +127` | Right stick, same convention. |
| `ps5.l2` `ps5.r2` | `uint8` | `0 … 255` | Analog trigger. Doubles as "pressed" if `> 0`. |

Don't want to deal with raw `-128..+127` / `0..255`? Use the percent helpers:

| Helper | Range |
|---|---|
| `ps5.lxPct()` `ps5.lyPct()` `ps5.rxPct()` `ps5.ryPct()` | `-100 … +100` |
| `ps5.l2Pct()` `ps5.r2Pct()` | `0 … 100` |

### Buttons

All booleans. `true` while held.

| Group | Fields |
|---|---|
| **D-pad** | `ps5.up`  `ps5.down`  `ps5.left`  `ps5.right` |
| **Face** | `ps5.cross`  `ps5.circle`  `ps5.square`  `ps5.triangle` |
| **Shoulders** | `ps5.l1`  `ps5.r1` |
| **Stick clicks** | `ps5.l3`  `ps5.r3` |
| **System** | `ps5.share`  `ps5.options`  `ps5.ps_btn`  `ps5.touchpad`  `ps5.mute` |

> **Edge detection.** Use `ps5.pressed(ps5.square)` for a one-shot rising edge (fires once when `false → true`) and `ps5.released(ps5.square)` for the falling edge. Works on any button bool, e.g. `if (ps5.pressed(ps5.l1)) { ... }`. The library tracks previous state internally — no shadow variables in your sketch.

### Motion sensors

| Field | Type | Conversion |
|---|---|---|
| `ps5.gyroX` `ps5.gyroY` `ps5.gyroZ` | `int16` raw | `÷ 1024` → deg/s |
| `ps5.accelX` `ps5.accelY` `ps5.accelZ` | `int16` raw | `÷ 8192` → g (**includes gravity**) |
| `ps5.sensorTime` | `uint32` | `× 0.33 µs` per LSB |

### Touchpad

The touchpad surface is **1920 × 1080**. Two simultaneous fingers, indexed `0` or `1`.

| Method | Returns |
|---|---|
| `ps5.TouchActive(i)` | `bool` — finger present? |
| `ps5.TouchX(i)` | `0 … 1919` |
| `ps5.TouchY(i)` | `0 … 1079` |
| `ps5.TouchId(i)` | `0 … 127` — increments per fresh touch |

### Battery & jacks

| Field | Type | Meaning |
|---|---|---|
| `ps5.battery` | `uint8` | `0 … 100` (percent) |
| `ps5.charging` | `bool` | USB power detected |
| `ps5.fullyCharged` | `bool` | Battery at 100 % |
| `ps5.headphones` | `bool` | 3.5 mm jack inserted |
| `ps5.micJack` | `bool` | External mic detected |

### Outputs (chainable)

Each setter returns `*this`, so you can chain everything into one statement and finish with `.send()`.

```cpp
ps5.lightbar(255, 0, 0)         // RGB lightbar around touchpad, 0..255 each
   .rumble(255, 0)              // small (sharp buzz), large (deep rumble), 0..255
   .playerLed(3, 3)             // turn on LED 3 at brightness 3 (1=dim, 2=mid, 3+=bright). 0 = off
   .muteLed(2)                  // 0 = off, 1 = solid, 2 = pulse
   .l2Trigger(20, 80, 100)      // see Adaptive triggers below
   .send();
```

| Method | Effect |
|---|---|
| `.lightbar(r, g, b)` | RGB lightbar (the colored strip around the touchpad) |
| `.rumble(small, large)` | small = sharp buzz, large = deep rumble |
| `.playerLed(index, value)` | Light one of the 5 white LEDs below the touchpad. `index` 1..5 (1 = far-left). `value`: `0` = off, `1` = dim, `2` = mid, `3` (or any ≥3) = bright. Chain freely: `ps5.playerLed(1,3).playerLed(3,3).playerLed(5,3).send();`. **Note:** the controller has only one global brightness register, so the *last non-zero `value`* before `send()` sets brightness for every lit LED. Pass `value=0` to turn a LED off without changing the others' brightness. For a raw 5-bit mask: `ps5.output.playerLeds = 0b10101; ps5.send();` |
| `.muteLed(mode)` | 0 = off, 1 = solid, 2 = pulse |
| `.send()` | Push everything to the controller (call at most once per 10 ms) |

### Adaptive triggers

The DualSense triggers have little motors inside. Each trigger (L2 / R2) plays **one** effect at a time. L2 and R2 are independent — you can run different effects on each.

All position / strength values are **percent (0..100)**. `freqHz` is real Hz (try 5..30).

```cpp
ps5.l2Trigger(20, 80, 100)      // gun-trigger squeeze + click on left
   .r2Pulse(30, 100, 15)        // buzzing right trigger
   .send();
```

#### Basic effects

| Method | Feels like | Args |
|---|---|---|
| `.l2Off()` / `.r2Off()` | Normal trigger, no force | — |
| `.l2Rigid(start, strength)` | Stiff wall past `start` | start %, strength % |
| `.l2Trigger(start, end, strength)` | Gun-trigger squeeze with click at the end | start %, end %, strength % |
| `.l2Pulse(start, strength, freqHz)` | Buzzing past `start` | start %, strength %, freq Hz |

#### Combo effects (firmware presets — two effects fused into one)

| Method | Feels like | Args |
|---|---|---|
| `.l2Bow(start, end, strength, snap)` | Squeeze + snap-back at the end | start %, end %, strength %, snap % |
| `.l2Galloping(start, end, foot1, foot2, freqHz)` | Horse-gallop two-beat rhythm | start %, end %, foot1 %, foot2 %, freq Hz |
| `.l2Machine(start, end, ampA, ampB, freqHz, periodTenths)` | Buzz that swaps strength every `periodTenths × 0.1 s` | start %, end %, ampA %, ampB %, freq Hz, period in tenths |

`r2*` versions are identical for the right trigger.

> Want two basic effects on the **same** trigger (e.g. "trigger + pulse")? You can't — only one mode block per trigger. Use `l2Machine` (buzz inside a range) for the closest equivalent, or run different effects on L2 vs R2.

---

## Things to watch for

### Hardware quirks
- **Accelerometer always reads ~1 g at rest.** It measures *proper acceleration* — gravity is part of it. Lay flat → one axis ≈ ±8192, others ≈ 0. Tilt → that 1 g redistributes. `√(x² + y² + z²) ≈ 8192` whenever the controller is still. To detect motion, threshold `|magnitude − 8192|`.
- **Gyro at rest jitters ±20 LSB** ≈ ±0.02 deg/s. That's sensor noise around per-unit bias. To track orientation, integrate with bias subtracted using `sensorTime`; to detect motion, threshold the magnitude.

### Sketch tips
- **Don't block in `loop()`.** All radio work runs on FreeRTOS tasks — use `delay()` or `vTaskDelay()`, never a busy-loop.
- **Reconnect is automatic.** No code needed. `attachOnDisconnect` fires when the link drops.
- **Use the Huge APP partition.** With the default 1.2 MB partition, Bluedroid won't fit and the sketch won't even link.

---

## Wire format reference

The DualSense talks **HID over L2CAP** on two PSMs: **0x11 (control)** + **0x13 (interrupt)**. The library:

1. After connect, sends a feature-set on PSM 0x11 to flip the controller out of "USB-style minimal report" into the full BT report. *Without this you'd only get sticks + face buttons.*
2. Reads input report **`0x31`** (78 bytes) on PSM 0x13.
3. Writes output report **`0x31`** (79 bytes incl. 0xA2 HID header + CRC32) on PSM 0x13.

### Input report — 78 bytes

Byte-for-byte aligned with Linux kernel `drivers/hid/hid-playstation.c` `struct dualsense_input_report`.

| Byte | Field |
|---:|---|
| 0 | report id `0x31` |
| 1 | reserved tag |
| 2 / 3 | LX / LY (left stick, 0..255, 128 = center, Y inverted) |
| 4 / 5 | RX / RY (right stick) |
| 6 / 7 | L2 / R2 trigger pressure 0..255 |
| 8 | seq number |
| 9 | buttons[0]: bits 0..3 = D-pad hat; bit 4 = □; bit 5 = ✕; bit 6 = ◯; bit 7 = △ |
| 10 | buttons[1]: L1, R1, L2, R2, Share/Create, Options, L3, R3 |
| 11 | buttons[2]: bit 0 = PS, bit 1 = Touchpad, bit 2 = Mic-Mute |
| 12 | reserved |
| 13..16 | reserved |
| 17..22 | gyroscope x, y, z (le16, raw int16, `÷ 1024` = deg/s) |
| 23..28 | accelerometer x, y, z (le16, raw int16, `÷ 8192` = g) |
| 29..32 | sensor timestamp (le32, 0.33 µs / LSB) |
| 33 | reserved |
| 34..41 | touchpad: 2 contacts × 4 bytes — `[id+active, x_lo, x_hi+y_lo, y_hi]` |
| 42..53 | reserved |
| 54 | status[0]: low nibble = battery 0..10; high nibble = charging state |
| 55 | status[1]: HP detect, Mic detect, Mic-mute |
| 56 | status[2] reserved |
| 57..73 | reserved |
| 74..77 | CRC32 LE of bytes 0..73 |

### Output report — 79 bytes

| Byte | Field |
|---:|---|
| 0 | `0xA2` — HID transaction header (DATA \| OUTPUT), part of CRC |
| 1 | report id `0x31` |
| 2 | seq_tag — high nibble = sequence 0..15, low nibble = 0 |
| 3 | tag = `0x10` (controller drops the report if missing) |
| 4 | valid_flag0 — bit 0 = COMPATIBLE_VIBRATION, bit 1 = HAPTICS_SELECT |
| 5 | valid_flag1 — bit 0 = MIC_MUTE_LED, bit 2 = LIGHTBAR_ENABLE, bit 4 = PLAYER_INDICATOR |
| 6 | motor_right (small/high-freq rumble) |
| 7 | motor_left  (large/low-freq  rumble) |
| 8..11 | audio (HP vol, speaker vol, mic vol, audio_control) — unused |
| 12 | mute_button_led (0=off, 1=solid, 2=pulse) |
| 13..40 | audio reserved |
| 41 | audio_control2 |
| 42 | valid_flag2 — bit 1 = LIGHTBAR_SETUP, bit 2 = COMPATIBLE_VIBRATION2 |
| 43..44 | reserved |
| 45 | lightbar_setup — bit 1 = LIGHT_OUT (cancels startup blue fade) |
| 46 | player LED brightness 0..2 |
| 47 | player_leds bitmask (bit 0 = far-left … bit 4 = far-right) |
| 48..50 | lightbar R, G, B |
| 51..74 | reserved (zero) |
| 75..78 | CRC32 LE of bytes 0..74 (poly `0xEDB88320`, init `0xFFFFFFFF`, xorout `0xFFFFFFFF`) |

---

## Files

```
src/
  ps5Controller.h        public Arduino API
  ps5Controller.cpp      Arduino class, GAP setup, auto-pair, auto-reconnect
  ps5_bytes.cpp          protocol parser + builder (the byte spec)
  bluedroid/
    bluedroid.h          minimal hand-curated L2CAP / BTM / OSI headers
    bluedroid.cpp        L2CAP transport + GAP/SPP bring-up (stack glue)
examples/testEverything/   full-feature sketch + 1 Hz serial snapshot
```

---

## License & credits

**License:** LGPL-3.0 — see [`LICENSE`](LICENSE).

**Author:** [hamzayslmn](https://github.com/hamzayslmn)

DualSense, PlayStation, and PS5 are trademarks of Sony Interactive Entertainment. This library is an independent, unofficial implementation and is not affiliated with or endorsed by Sony.
