# esp-ps5

**Pair a Sony DualSense (PS5) controller to an ESP32 over Bluetooth Classic.**
Simple, robust, dependency-free. One header, one global object — that's it.

```cpp
#include <ps5Controller.h>

void setup() {
  Serial.begin(115200);
  ps5.begin("AA:BB:CC:DD:EE:FF");   // your DualSense MAC
}

void loop() {
  if (!ps5.isConnected()) return;
  if (ps5.Cross())  ps5.setRumble(255, 0);
  if (ps5.Circle()) ps5.setLed(255, 0, 0);
  ps5.sendToController();
  delay(50);
}
```

Tags: **PS5** · **DualSense** · **ESP32** · **ESP** · **Bluetooth Classic** · **Gamepad** · **Sony** · **HID**

---

## Tested

| Item | Version |
|---|---|
| Arduino-ESP32 core | **3.3.6** (= ESP-IDF 5.5.2) |
| Boards | **ESP32-WROOM-32D**, **ESP32-WROOM-32U** |
| Partition scheme | **Huge APP** (Bluedroid won't fit otherwise) |
| Controller firmware | DualSense BT 0x31 protocol (any current FW) |

Everything below is verified working on hardware: pairing, reconnect, all buttons, both sticks, both triggers (digital + analog), D-pad, touchpad button, mic-mute capacitive button, both rumble motors, RGB lightbar, all 5 player LEDs, mic-mute LED, gyroscope, accelerometer, both touchpad fingers, battery percentage, charging state, headphone-jack detect, mic-jack detect.

---

## Install

1. Clone or download this folder into `~/Documents/Arduino/libraries/esp-ps5`.
2. Arduino IDE → *Tools → Board → ESP32 Arduino → ESP32 Dev Module*.
3. *Tools → Partition Scheme → **Huge APP*** (mandatory — Bluedroid is large).
4. *File → Examples → esp-ps5 → testEverything*.

## Pair the controller

1. Hold **PS + Create** on the DualSense for 3 seconds — the lightbar pulses white.
2. On a phone or PC, scan and write down the MAC address (looks like `AA:BB:CC:DD:EE:FF`).
3. Pass that string to `ps5.begin("AA:BB:CC:DD:EE:FF")`.

Or call `ps5.scanDevices(secs, cb)` with a 5–10 second window and pick yours from the callback.

---

## API in one screen

```cpp
ps5.begin();                    // reconnect to last paired controller
ps5.begin("AA:BB:CC:DD:EE:FF"); // pair a new one
ps5.isConnected();
ps5.attachOnConnect(cb);        // void cb()
ps5.attachOnDisconnect(cb);
ps5.attach(cb);                 // fires on every input packet (~250 Hz)

// Buttons (digital, 0/1)
ps5.Up()  ps5.Down()  ps5.Left()  ps5.Right()
ps5.UpRight()  ps5.UpLeft()  ps5.DownRight()  ps5.DownLeft()
ps5.Cross()  ps5.Circle()  ps5.Square()  ps5.Triangle()
ps5.L1()  ps5.R1()  ps5.L2()  ps5.R2()  ps5.L3()  ps5.R3()
ps5.Share()  ps5.Options()  ps5.PSButton()  ps5.Touchpad()  ps5.Mute()

// Analog
ps5.LStickX()   ps5.LStickY()    // int8, -128..+127, centered at 0
ps5.RStickX()   ps5.RStickY()
ps5.L2Value()   ps5.R2Value()    // uint8, 0..255

// Motion + touchpad
ps5.GyroX/Y/Z()                  // int16 raw, ~1024 LSB / deg/s
ps5.AccelX/Y/Z()                 // int16 raw, ~8192 LSB / g  (includes gravity!)
ps5.SensorTimestamp()            // uint32, 0.33 us / LSB
ps5.TouchActive(0|1)  TouchX(i)  TouchY(i)  TouchId(i)

// Status
ps5.Battery()                    // 0..10  (* 10% steps)
ps5.Charging()  ps5.FullyCharged()
ps5.Headphones()  ps5.MicJack()

// Output  (call sendToController() once you've staged everything)
ps5.setLed(r, g, b);             // 0..255 each — full RGB lightbar
ps5.setRumble(small, large);     // small=high-freq, large=low-freq, 0..255
ps5.setPlayerLeds(0b00100);      // 5 bits, bit0=far-left LED .. bit4=far-right
ps5.setMuteLed(0|1|2);           // 0=off, 1=solid, 2=pulse
ps5.sendToController();
```

Edges are also computed for you: `ps5.event.button_down.cross` is `true` only on the packet where Cross transitioned from released → pressed. Same with `button_up.cross` for the release edge.

---

## What every byte means

The DualSense talks **HID over L2CAP** on two PSMs: **0x11 (control)** and **0x13 (interrupt)**. The library opens both and:

1. After connect, sends a feature-set on PSM 0x11 to flip the controller out of "USB-style minimal report" into the full BT report (this is required — without it you only get sticks + face buttons).
2. Reads input report **`0x31`** (78 bytes) on PSM 0x13.
3. Writes output report **`0x31`** (79 bytes incl. 0xA2 HID header + CRC32) back on PSM 0x13.

### Input report (controller → ESP32, 78 bytes)

Byte-for-byte aligned with Linux kernel `drivers/hid/hid-playstation.c` `struct dualsense_input_report`.

| wire byte | meaning |
|---|---|
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
| 17..22 | gyroscope x, y, z (le16 each, RAW int16, kernel scale `÷ 1024 = deg/s`) |
| 23..28 | accelerometer x, y, z (le16 each, RAW int16, kernel scale `÷ 8192 = g`) |
| 29..32 | sensor timestamp (le32, 0.33 µs / LSB) |
| 33 | reserved |
| 34..41 | touchpad: 2 contacts × 4 bytes — `[id+active, x_lo, x_hi+y_lo, y_hi]` |
| 42..53 | reserved |
| 54 | status[0]: low nibble = battery 0..10; high nibble = charging state |
| 55 | status[1]: HP detect, Mic detect, Mic-mute |
| 56 | status[2] reserved |
| 57..73 | reserved |
| 74..77 | CRC32 LE of bytes 0..73 |

### Output report (ESP32 → controller, 79 bytes)

| wire byte | meaning |
|---|---|
| 0 | `0xA2` — HID transaction header (DATA \| OUTPUT). Part of CRC. |
| 1 | report id `0x31` |
| 2 | seq_tag — high nibble = sequence number 0..15, low nibble = 0 |
| 3 | tag = `0x10` — magic (controller drops the report if missing) |
| 4 | valid_flag0 — bit 0 = COMPATIBLE_VIBRATION, bit 1 = HAPTICS_SELECT |
| 5 | valid_flag1 — bit 2 = LIGHTBAR_ENABLE, bit 4 = PLAYER_INDICATOR_ENABLE, bit 0 = MIC_MUTE_LED_ENABLE |
| 6 | motor_right (small/high-freq rumble, 0..255) |
| 7 | motor_left  (large/low-freq  rumble, 0..255) |
| 8..11 | audio (headphone vol, speaker vol, mic vol, audio_control) — unused |
| 12 | mute_button_led (0=off, 1=solid, 2=pulse) |
| 13..40 | audio reserved |
| 41 | audio_control2 |
| 42 | valid_flag2 — bit 1 = LIGHTBAR_SETUP, bit 2 = COMPATIBLE_VIBRATION2 |
| 43..44 | reserved |
| 45 | lightbar_setup — bit 1 = LIGHT_OUT (cancels the startup blue fade so user RGB is honoured immediately) |
| 46 | player LED brightness 0..2 |
| 47 | player_leds bitmask (bit 0 = far-left ... bit 4 = far-right) |
| 48..50 | lightbar R, G, B (0..255 each) |
| 51..74 | reserved (zero) |
| 75..78 | CRC32 LE of bytes 0..74 (poly 0xEDB88320, init 0xFFFFFFFF, xorout 0xFFFFFFFF) |

---

## Things to watch for

- **Accelerometer always reads ~1g at rest.** It measures *proper acceleration* — gravity is part of it. Lay the controller flat → one axis sits near ±8192, the other two near 0. Tilt the controller → that 1g vector redistributes across axes. Magnitude `√(x² + y² + z²) ≈ 8192` whenever the controller isn't moving. If you want "is it moving?", check `|magnitude − 8192| > threshold`.
- **Gyro at rest jitters by ±20 LSB.** That's ±0.02 deg/s — sensor noise around the unit's individual bias. To track orientation, integrate (with bias subtracted, using the packet timestamp); to detect motion, threshold the magnitude.
- **Reconnect.** If the controller wanders out of range, the library will retry on the next `begin()` call automatically. The `attachOnDisconnect` callback fires when this happens.
- **Don't block in your loop.** All radio work is on FreeRTOS tasks. Use `vTaskDelay` (or short `delay`) — never busy-loop.
- **Huge APP partition is mandatory.** With the default 1.2 MB partition, Bluedroid won't fit and the sketch won't even link.

## Files

```
src/
  ps5Controller.h    public Arduino API
  ps5Controller.cpp  Bluedroid bring-up, GAP, reconnect, callbacks
  ps5_bytes.cpp      protocol parser + builder (this is THE byte spec)
  bluedroid/         minimal hand-curated headers for L2CAP / BTM / OSI
examples/testEverything/   full-feature sketch + 1 Hz serial snapshot
```

## License

LGPL-3.0. See `LICENSE`.

## Credits

- Sony DualSense controller — Sony Interactive Entertainment.
- Wire-format reference: [Linux kernel `drivers/hid/hid-playstation.c`](https://github.com/torvalds/linux/blob/master/drivers/hid/hid-playstation.c).
- Original esp32 PS3 port that inspired the L2CAP/BTM glue.
