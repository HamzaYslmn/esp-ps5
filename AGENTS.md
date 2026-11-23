# esp-ps5 — agent notes

ESP32 as a Bluetooth Classic HID host for a PS5 DualSense (raw Bluedroid L2CAP).
Protocol cross-checked against Linux `hid-playstation.c`, SDL `SDL_hidapi_ps5.c`, Bluepad32, SAxense.

## Files

```
src/ps5Controller.h      public API + internal glue declarations
src/ps5Controller.cpp    bring-up, async scan/connect, output scheduling, LED gate, sound, trigger encoders
src/ps5_bytes.cpp        all byte work: 0x31 parse, 0x31 / reset / 0x32 builders, feature reports, ROM CRC32
src/bluedroid/           L2CAP transport (PSM 0x11 + 0x13), page scan, hand-declared Bluedroid internals
```

## Flow (nothing blocks; frames go out from the Bluetooth task)

1. `begin()`: BT + Bluedroid up, GAP callback, fast interlaced page scan (11.25 ms / 40 ms),
   page timeout 1.28 s, 400 ms `esp_timer` tick. Bonded pad in Bluedroid NVS → page it; else 5 s inquiry.
2. Tick, while not connected: every ~5 s alternate paging the known pad / inquiry (inquiry finds a pad
   in pairing mode). Inbound reconnect (pad's PS press pages us) needs page scan, so inquiries stay short.
3. Both channels configured → non-connectable, `ps5_on_link(true)` → one output frame (starts 0x31 input)
   + GET feature 0x20 (firmware); its reply triggers GET 0x05 (calibration).
4. Every input report (`ps5_on_input`, BT task): parse → LED gate → send pending output (≤ every 10 ms)
   → haptics frame if a melody is due. `send()` only sets a dirty flag.
5. LED gate (SDL): pad plays its connect animation until its sensor clock ≥ 10200000 (0.33 µs ticks,
   ~3.4 s after pad power-on). Before that, frames carry rumble only. Then one reset-only frame
   (VF1 bit 3), latch ready, then full frames. Same reset when `releaseLeds` toggles.
6. Tick while connected: no report for 1 s → kick an output frame.

## OUTPUT 0x31 (79 bytes incl. 0xA2 header, interrupt PSM)

| Offset | Field |
|---|---|
| 0 | `0xA2` HIDP DATA\|OUTPUT (covered by CRC) |
| 1 | `0x31` |
| 2 | seq tag, kept 0 (SDL) |
| 3 | `0x10` |
| 4 | valid_flag0: 0x01 rumble emulation + 0x02 disable audio haptics (only while motors ≠ 0), 0x04 R2, 0x08 L2 |
| 5 | valid_flag1: 0x01 mute LED, 0x02 power save, 0x04 lightbar, 0x08 reset LEDs, 0x10 player LEDs |
| 6 / 7 | motor right (small) / left (large) |
| 12 | mute LED 0 off, 1 solid, 2 pulse |
| 13 | power save: bit 4 = mic hardware mute |
| 14..24 / 25..35 | R2 / L2 trigger block: mode + 10 params |
| 42 | valid_flag2: 0x01 player-LED brightness (0x02 lightbar setup: unused, LIGHT_OUT = 3 s fade) |
| 46 | brightness 0 bright, 1 mid, 2 dim |
| 47 | player LEDs (5 bits) \| 0x20 instant change |
| 48..50 | lightbar R, G, B |
| 75..78 | CRC32 LE over 0..74 (zlib; `esp_rom_crc32_le(0, …)`) |

Reset-only frame: header + `0x10`, VF0 0, VF1 `0x08`, rest 0.

## HAPTICS 0x32 (143 bytes incl. 0xA2, SAxense layout)

`A2 32 00 91 07 FE 00 00 00 00 FF <ctr> 92 40` + 64 × int8 (32 stereo L/R samples, 3 kHz) + zeros + CRC32
over 0..138 at 139..142. Every 10.667 ms. Plays only while VF0 0x02 is clear (no firmware rumble).
Speaker = report 0x36 with 200-byte Opus frames (48 kHz) — not implemented (needs libopus).

## Feature reports (control channel)

GET: `43 <id>` (no CRC). Reply: `A3 <id> …` + CRC32 (not checked); offsets below count `<id>` as 0.
- `0x20` firmware (64 B): `ps5.firmware` = le16 at 44 (SDL). Firmware ≥ 2.24 offers "improved" rumble
  (VF2 0x04, VF0 0x02): tested softer than VF0 0x01, so not used.
- `0x05` calibration (41 B), le16: gyro bias pitch/yaw/roll 1,3,5; plus/minus pitch 7/9, yaw 11/13,
  roll 15/17; speed plus/minus 19/21; accel plus/minus x 23/25, y 27/29, z 31/33.
  gyro deg/s = (raw − bias) · (speed+ + speed−) / (plus − minus); accel g = (raw − (plus − range/2)) · 2 / range,
  range = plus − minus. Rejected (nominal 1/16 deg/s, 1/8192 g kept) if |bias| > 1024 or scale off > 50 % (SDL).
- SET `0x08` power off: `53 08 02` + zeros, 49 B, CRC32 of bytes 0..44 at 45..48 (DS5Dongle).

## Adaptive trigger modes (Nielk1; deprecated 0x01/0x02/0x06 not used)

| Mode | Setter | Params |
|---|---|---|
| 0x05 Off | `l2Off()` | 0 |
| 0x21 Feedback | `l2Rigid(start, str)` | p0..1 zone mask [pos..9], p2..5 3-bit force per zone `(str-1)&7` |
| 0x25 Weapon | `l2Trigger(start, end, str)` | p0..1 `(1<<a)\|(1<<b)` a∈[2..7] b∈[a+1..8], p2 `str-1` |
| 0x26 Vibration | `l2Pulse(start, amp, hz)` | as Feedback, p8 Hz |
| 0x22 Bow | `l2Bow(start, end, str, snap)` | mask, p2 `(str-1)\|(snap-1)<<3` |
| 0x23 Galloping | `l2Galloping(start, end, f1, f2, hz)` | mask, p2 `f2\|f1<<3` (f1≤6, f2>f1), p3 Hz |
| 0x27 Machine | `l2Machine(start, end, a, b, hz, per)` | mask, p2 `a\|b<<3` (raw 0..7), p3 Hz, p4 period 0.1 s |

Percent → zone `(pct*9+50)/100`, strength `(pct*8+50)/100` (0 → Off).

## INPUT 0x31 (78 bytes after the 0xA1 header)

| Offset | Field |
|---|---|
| 0 | `0x31` (other reports, e.g. short 0x01, are ignored) |
| 2..5 | LX LY RX RY, 128 centre → `raw-128`, deadzone applied |
| 6 / 7 | L2 / R2 |
| 9 | hat low nibble (0..7 N..NW, 8 neutral), □ ✕ ○ △ high nibble |
| 10 | L1 R1 L2 R2 Create Options L3 R3 |
| 11 | PS, touchpad, mute |
| 17..22 / 23..28 | gyro / accel xyz le16 (raw gyro ≈ 16 LSB per deg/s, accel 8192 per g) |
| 29..32 | sensor clock le32, 0.33 µs (LED gate; wraps ~24 min) |
| 34..41 | touch ×2: `[id\|inactive<<7][x lo][x hi\|y lo<<4][y hi]` |
| 54 | battery 0..10 + charging nibble (1 chg, 2 full, A/B/F error) |
| 55 | headphones, mic jack, mic muted |

## Don't regress

- Core ≥ 3.3.9 / 4.x: include `esp32-hal-alloc-bt-classic-mem.h` or `btStart()` fails (issue #2).
- IDF 6.x `tL2CAP_APPL_INFO` may have one more trailing pointer: keep the null slot.
- Bluedroid reports disconnects per channel: clear only that CID; link-down only on the up→down edge.
- No CONNECT_REQ unless both CIDs are 0; failed outbound connect clears the control CID.
- LEDs: no writes during the connect animation, reset-only frame first, never LIGHT_OUT every frame,
  never RELEASE (reset) in every frame (the pad drops the link).
- TX buffer is `sizeof(BT_HDR) + 13 + len`, not Bluedroid's 4 KB default. `latestPacket` is a copy.

## Rough edges

Input CRC not verified. No `end()`. Bond list: first bonded device is assumed to be the pad.
