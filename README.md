# esp-ps5

**Use a Sony DualSense (PS5) controller from an ESP32 over Bluetooth Classic.**
One header, one global object, no callbacks, no boilerplate — just read fields and chain output setters.

```cpp
#include <ps5Controller.h>

void setup() {
  Serial.begin(115200);
  ps5.begin(20);   // scan up to 20 s, auto-connect to the first DualSense seen
}

void loop() {
  if (!ps5.isConnected()) return;            // auto-reconnects in the background

  if (ps5.pressed(ps5.cross)) Serial.println("X pressed!");
  if (ps5.l1)                 ps5.lightbar(255, 0, 0).rumble(180, 0).send();
  else                        ps5.lightbar(0, 0, 0).rumble(0, 0).send();

  delay(20);   // don't call .send() faster than ~10 ms
}
```

> Tags: **PS5** · **DualSense** · **ESP32** · **Bluetooth Classic** · **Gamepad** · **Sony** · **HID**

---

## Contents

1. [Tested setups](#tested-setups)
2. [Install](#install)
3. [Pair the controller](#pair-the-controller)
4. [API reference](#api-reference)
   - [Connection](#connection)
   - [Sticks & triggers](#sticks--triggers)
   - [Buttons](#buttons)
   - [Edge detection](#edge-detection)
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
| Controller firmware | DualSense BT 0x31 (any current FW) |

**Verified on hardware:** pairing · auto-reconnect · all buttons · both sticks · both triggers (digital + analog) · D-pad · touchpad button · mic-mute capacitive button · both rumble motors · RGB lightbar · all 5 player LEDs · mic-mute LED · gyroscope · accelerometer · both touchpad fingers · battery percentage · charging state · headphone-jack detect · mic-jack detect · all 7 adaptive trigger modes.

---

## Install

1. Clone or download into `~/Documents/Arduino/libraries/esp-ps5`.
2. *Tools → Board → ESP32 Arduino → ESP32 Dev Module*.
3. *Tools → Partition Scheme → **Huge APP*** ← **preffered**
4. *File → Examples → esp-ps5 → testEverything*.

---

**Every boot:**

1. Hold **PS + Create** for ~3 s — the lightbar pulses white (controller is now advertising).
2. Call `ps5.begin(timeoutSecs)` in `setup()`.
3. The library scans Bluetooth Classic and **early-exits the moment** any device named *DualSense* or *Wireless Controller* is seen (~1–3 s typical), then connects.

| Call | Behaviour |
|---|---|
| `ps5.begin(20)` | Scan up to **20 s**, connect to first DualSense found. |
| `ps5.begin("AA:BB:CC:DD:EE:FF")` | Connect to a specific MAC (skip the inquiry). |
| `ps5.forget()` | Drop the latched controller MAC; the next `begin()` re-scans. |

> **Reconnect is automatic.** If the controller goes out of range, sleeps, or browses away, the library quietly fires a fresh L2CAP connect every 5 s in the background. Your `loop()` just keeps polling `ps5.isConnected()`.

> **Pairing keys persist across reboots.** Bluedroid stores the link key in its own NVS area, so once paired, you don't need to re-pair on every power-up — just hold PS+Create to wake the controller.

---

## API reference

> **No callbacks. No event objects. Just fields.** Inputs are plain public fields on the global `ps5` instance, refreshed every input packet (~250 Hz). Read them like ordinary variables.

### Connection

| Member | Returns | What it does |
|---|---|---|
| `ps5.begin(timeoutSecs)` | `bool` | Bring up the BT stack, scan up to `timeoutSecs`, auto-connect to the first DualSense seen. Returns `true` once stack is up; the actual connection is async — poll `isConnected()`. |
| `ps5.begin("AA:BB:..")` | `bool` | Connect to a known MAC. Blocks up to 10 s for the first input packet. |
| `ps5.isConnected()` | `bool` | True while input packets are flowing. Doubles as the auto-reconnect heartbeat. |
| `ps5.forget()` | — | Drop the latched controller MAC so the next `begin()` re-scans from scratch. |
| `ps5.scanDevices(secs, cb)` | `bool` | Manual BT inquiry; `cb(mac, name, rssi)` per unique device. Does NOT auto-connect — use `begin()` for that. |

### Sticks & triggers

| Field | Type | Range | Notes |
|---|---|---|---|
| `ps5.lx` `ps5.ly` | `int8` | `-128 … +127` | Left stick. **Y is inverted** (push UP = negative). 0 = centered. |
| `ps5.rx` `ps5.ry` | `int8` | `-128 … +127` | Right stick, same convention. |
| `ps5.l2` `ps5.r2` | `uint8` | `0 … 255` | Analog triggers. Doubles as "pressed" if `> 0`. |

Prefer percent units? Use the helpers:

| Helper | Range |
|---|---|
| `ps5.lxPct() lyPct() rxPct() ryPct()` | `-100 … +100` |
| `ps5.l2Pct() r2Pct()` | `0 … 100` |

### Buttons

All booleans. `true` while held.

| Group | Fields |
|---|---|
| **D-pad** | `ps5.up`  `ps5.down`  `ps5.left`  `ps5.right` |
| **Face** | `ps5.cross`  `ps5.circle`  `ps5.square`  `ps5.triangle` |
| **Shoulders** | `ps5.l1`  `ps5.r1` |
| **Stick clicks** | `ps5.l3`  `ps5.r3` |
| **System** | `ps5.share`  `ps5.options`  `ps5.ps_btn`  `ps5.touchpad`  `ps5.mute` |

Diagonals come naturally from logical AND, e.g. `ps5.up && ps5.right` for NE.

### Edge detection

For **one-shot** events instead of "true while held", wrap any bool in `pressed()` / `released()`:

```cpp
if (ps5.pressed(ps5.square))  Serial.println("square clicked");
if (ps5.released(ps5.l1))     Serial.println("L1 let go");
```

The library tracks previous state internally for up to 24 fields — no `wasX` shadow flags in your sketch.

### Motion sensors

| Field | Type | Conversion |
|---|---|---|
| `ps5.gyroX gyroY gyroZ` | `int16` raw | `÷ 1024` → deg/s |
| `ps5.accelX accelY accelZ` | `int16` raw | `÷ 8192` → g (**includes gravity**) |
| `ps5.sensorTime` | `uint32` | `× 0.33 µs` per LSB |

### Touchpad

Surface is **1920 × 1080**. Two simultaneous fingers, indexed `0` or `1`.

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

Each setter returns `*this`. Stage everything you want, then push it all in one frame with `.send()`:

```cpp
ps5.lightbar(255, 0, 0)     // RGB lightbar (touchpad strip), 0..255 each
   .rumble(255, 0)          // small (sharp buzz), large (deep rumble), 0..255
   .playerLed(3, 3)         // player LED #3 at brightness 3 (1=dim, 2=mid, 3+=bright). 0=off
   .muteLed(2)              // 0=off, 1=solid, 2=pulse
   .l2Trigger(20, 80, 100)  // see Adaptive triggers
   .send();
```

| Method | Effect |
|---|---|
| `.lightbar(r, g, b)` | RGB lightbar |
| `.rumble(small, large)` | small = high-freq motor (right grip), large = low-freq motor (left grip) |
| `.playerLed(index, value)` | LED #1..5 (1 = far-left). `value`: 0=off, 1=dim, 2=mid, 3+=bright. **One global brightness register**, so the *last non-zero `value` before `send()`* sets brightness for every lit LED. |
| `.muteLed(mode)` | 0=off, 1=solid, 2=pulse |
| `.send()` | Push the staged frame. **Don't call faster than every ~10 ms** or the L2CAP TX queue congests. |

For a raw 5-bit player-LED mask: `ps5.output.playerLeds = 0b10101; ps5.send();`

### Adaptive triggers

Each trigger (L2 / R2) plays **one** effect at a time. L2 and R2 are independent — you can mix different effects on each. All position / strength values are **percent (0..100)**. `freqHz` is real Hz (try 5–30).

```cpp
ps5.l2Trigger(20, 80, 100)   // gun-trigger squeeze + click on left
   .r2Pulse(30, 100, 15)     // buzzing right trigger
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

> Want two basic effects on the **same** trigger (e.g. trigger + pulse)? You can't — only one mode block per trigger per frame. Use `l2Machine` (buzz inside a range) for the closest equivalent, or run different effects on L2 vs R2.

---

## Things to watch for

### Hardware quirks

- **Accelerometer reads ~1 g at rest.** It measures *proper acceleration* — gravity is part of it. Lay flat → one axis ≈ ±8192, others ≈ 0. `√(x² + y² + z²) ≈ 8192` whenever still. To detect motion, threshold `|magnitude − 8192|`.
- **Gyro at rest jitters ±20 LSB** ≈ ±0.02 deg/s. Sensor noise around per-unit bias. Integrate with bias subtracted (using `sensorTime`) to track orientation; threshold the magnitude to detect motion.

### Sketch tips

- **Don't block in `loop()`.** All radio work runs on FreeRTOS tasks. Use `delay()` / `vTaskDelay()`, not busy-loops.
- **Reconnect is automatic** — `isConnected()` does it for you. Just call it every loop and skip work when it returns `false`.
- **Don't `send()` faster than ~10 ms apart**, or the L2CAP TX queue congests.

---

## Wire format reference

The DualSense talks **HID over L2CAP** on two PSMs: **0x11 (control)** + **0x13 (interrupt)**. The library:

1. After both channels are configured, sends a feature-set on PSM 0x11 to flip the controller out of "USB-style minimal report" into the full BT 0x31 report. *Without this you'd only get sticks + face buttons.*
2. Reads input report **`0x31`** (78 bytes) on PSM 0x13.
3. Writes output report **`0x31`** (79 bytes incl. 0xA2 HID header + CRC32) on PSM 0x13.

### Input report — 78 bytes

Byte-for-byte aligned with Linux kernel `drivers/hid/hid-playstation.c`.

| Byte | Field |
|---:|---|
| 0 | report id `0x31` |
| 1 | reserved tag |
| 2 / 3 | LX / LY (left stick, 0..255, 128 = center, Y inverted) |
| 4 / 5 | RX / RY (right stick) |
| 6 / 7 | L2 / R2 trigger pressure 0..255 |
| 8 | seq number |
| 9 | low nibble = D-pad hat, high nibble = ◯ △ ✕ □ |
| 10 | L1, R1, L2, R2, Create, Options, L3, R3 |
| 11 | bit 0 = PS, bit 1 = Touchpad, bit 2 = Mic-Mute |
| 17..22 | gyro x, y, z (le16, ÷ 1024 → deg/s) |
| 23..28 | accel x, y, z (le16, ÷ 8192 → g) |
| 29..32 | sensor timestamp (le32, 0.33 µs / LSB) |
| 34..41 | touchpad: 2 contacts × 4 bytes |
| 54 | low nibble = battery 0..10, high nibble = charging state |
| 55 | HP detect, mic detect, mic-mute |
| 74..77 | CRC32 LE (not currently verified on input) |

### Output report — 79 bytes

| Byte | Field |
|---:|---|
| 0 | `0xA2` HID DATA \| OUTPUT header (covered by CRC) |
| 1 | report id `0x31` |
| 2 | seq tag (high nibble = sequence 0..15) |
| 3 | tag `0x10` (DualSense BT marker — required) |
| 4 | valid_flag0 — vibration + haptics + L2/R2 adaptive enables |
| 5 | valid_flag1 — mic-mute LED, lightbar, player LED enables |
| 6 / 7 | motor_right / motor_left rumble |
| 12 | mute LED (0=off, 1=solid, 2=pulse) |
| 14..24 | R2 adaptive trigger (1 mode byte + 10 params) |
| 25..35 | L2 adaptive trigger (1 mode byte + 10 params) |
| 42 | valid_flag2 — brightness + lightbar setup enables |
| 45 | lightbar setup byte (0x02 disables startup blue fade) |
| 46 | player LED brightness (0=bright, 1=mid, 2=dim) |
| 47 | player LED bitmask (bit 0 = far-left … bit 4 = far-right) |
| 48..50 | lightbar R, G, B |
| 75..78 | CRC32 LE of bytes 0..74 (poly `0xEDB88320`, init/xorout `0xFFFFFFFF`) |

---

## Files

```
src/
  ps5Controller.h        public Arduino API
  ps5Controller.cpp      Arduino class + GAP/SPP setup + auto-pair + auto-reconnect
  ps5_bytes.cpp          protocol parser (input) + builder (output)
  bluedroid/
    bluedroid.h          minimal hand-curated L2CAP / BTM / OSI headers
    bluedroid.cpp        L2CAP transport glue
examples/testEverything/   full-feature sketch + 1 Hz serial snapshot
```

---

## License & credits

**License:** LGPL-3.0 — see [`LICENSE`](LICENSE).

**Author:** [hamzayslmn](https://github.com/hamzayslmn)

DualSense, PlayStation, and PS5 are trademarks of Sony Interactive Entertainment. This library is an independent, unofficial implementation and is not affiliated with or endorsed by Sony.
