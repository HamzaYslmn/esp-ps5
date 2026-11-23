# esp-ps5 — How the system works

Arduino library: an ESP32 acts as a Bluetooth Classic HID host for a PS5
DualSense. Input lands in flat public fields on the global `ps5`; output is
staged in `ps5.output` and sent with `ps5.send()`. Protocol verified against
Linux `drivers/hid/hid-playstation.c`.

## File layout

```
src/ps5Controller.h      public API + the internal glue declarations
src/ps5Controller.cpp    BT bring-up, async scan/connect/reconnect (GAP cb + esp_timer),
                         output setters, adaptive-trigger encoders
src/ps5_bytes.cpp        ALL byte work: 0x31 input parser, 0x31 output builder (ROM CRC32)
src/bluedroid/           L2CAP transport (PSM 0x11 + 0x13) + hand-declared Bluedroid internals
```

## Connection flow (fully async; nothing blocks loop())

1. `begin(secs)` brings up BT controller + Bluedroid, registers the GAP callback,
   sets connectable mode, registers HID PSMs, starts a 400 ms `esp_timer`, then
   starts a GAP inquiry (or connects to the latched MAC) and returns (~0.5 s).
2. GAP `DISC_RES` with name "DualSense" / "Wireless Controller" → latch MAC,
   cancel inquiry. On `DISC_STATE_CHANGED: STOPPED` → `ps5_l2cap_connect(mac)`.
3. Both L2CAP channels configured → `ps5_on_link(true)` → `ps5_send_enable()`
   (SET_REPORT feature 0xF4 on control).
4. Timer tick (esp_timer task), only while no input yet:
   - channels up → send a full output report every 400 ms (Bluepad32: an output
     report enables input 0x31; fixes the "channels up but silent" stall);
   - link fully idle → every ~5 s reconnect to the latched MAC, else rescan.
5. First valid 0x31 report → `g_active = true`, `onConnect`; every report → `onPacket`.
   The first report is parsed without edges (user is still holding PS+Create).
6. `isConnected()` just returns `g_active`.

`begin("AA:BB:..")` latches the MAC and connects; also returns immediately.

## DualSense Bluetooth wire format

### OUTPUT report (host → controller, 79 bytes, interrupt PSM 0x13)

| Offset | Field |
|---|---|
| 0 | `0xA2` HID DATA\|OUTPUT header (covered by CRC) |
| 1 | `0x31` report ID |
| 2 | seq tag (high nibble = sequence) |
| 3 | `0x10` DualSense BT tag (mandatory) |
| 4 | valid_flag0 = `0x0F`: compatible vibration, haptics select, R2, L2 trigger |
| 5 | valid_flag1 = `0x15`: mute LED, lightbar, player LEDs (+`0x02` power save, +`0x08` release LEDs) |
| 6 / 7 | motor_right (small) / motor_left (large) |
| 12 | mute LED (0 off, 1 solid, 2 pulse) |
| 13 | power save: bit 4 = mic hardware mute |
| 14..24 | R2 trigger block: mode + 10 params |
| 25..35 | L2 trigger block: mode + 10 params |
| 42 | valid_flag2 = `0x03`: LED brightness, lightbar setup |
| 45 | lightbar setup = `0x02` LIGHT_OUT (cancels the boot blue fade) |
| 46 | player LED brightness (0 bright, 1 mid, 2 dim) |
| 47 | player LED mask (5 bits) |
| 48..50 | lightbar R, G, B |
| 75..78 | CRC32 LE over bytes 0..74 (zlib; `esp_rom_crc32_le(0, ...)`) |

Don't `send()` faster than ~10 ms. `releaseLeds()` hands LEDs to firmware: send it
once, then stop sending output reports (continued output after RELEASE_LEDS makes
the firmware drop the link — the kernel defines the bit but never sets it).

### Adaptive trigger modes (Nielk1 spec; deprecated 0x01/0x02/0x06 are NOT used)

| Mode | Setter | Params |
|---|---|---|
| `0x05` Off | `l2Off()` | all 0 |
| `0x21` Feedback | `l2Rigid(start, str)` | p0..1 zone mask [pos..9], p2..5 3-bit force per zone `(str-1)&7` |
| `0x25` Weapon | `l2Trigger(start, end, str)` | p0..1 `(1<<a)\|(1<<b)` a∈[2..7] b∈[a+1..8], p2 `str-1` |
| `0x26` Vibration | `l2Pulse(start, amp, hz)` | as Feedback, p8 = Hz |
| `0x22` Bow | `l2Bow(start, end, str, snap)` | p0..1 mask, p2 `(str-1)\|(snap-1)<<3` |
| `0x23` Galloping | `l2Galloping(start, end, f1, f2, hz)` | p0..1 mask, p2 `f2\|f1<<3` (f1≤6, f2>f1), p3 Hz |
| `0x27` Machine | `l2Machine(start, end, ampA, ampB, hz, per)` | p0..1 mask, p2 `a\|b<<3` (raw 0..7), p3 Hz, p4 period (0.1 s) |

Percent → zone `pos = (pct*9+50)/100`, strength `(pct*8+50)/100` (0 → Off).
One block per trigger per frame; L2 and R2 are independent.

### INPUT report (controller → host, 78 bytes after the `0xA1` header)

| Offset | Field |
|---|---|
| 0 | `0x31` (anything else, e.g. the short `0x01` report, is ignored) |
| 2..5 | LX, LY, RX, RY (128 center; Y up = low) → `raw - 128` |
| 6 / 7 | L2 / R2 analog |
| 9 | low nibble hat (0..7 N..NW, 8 neutral), high nibble □ ✕ ○ △ |
| 10 | L1 R1 L2 R2 Create Options L3 R3 |
| 11 | PS, touchpad, mute |
| 17..22 / 23..28 | gyro / accel xyz, le16 |
| 29..32 | sensor timestamp le32 (0.33 µs) |
| 34..41 | 2 touch contacts: `[id\|inactive<<7][x lo][x hi\|y lo<<4][y hi]` |
| 54 | battery 0..10 (low) + charging nibble (1 chg, 2 full, A/B/F error) |
| 55 | headphones, mic jack, mic muted |
| 74..77 | CRC32 (not verified) |

Unused but known: audio volume/routing bytes 8..11 + 41 (needs a BT audio profile
to be useful), vibration v2 (VF2 bit 2, fw ≥ 2.21), feature reports 0x05
(motion calibration), 0x09 (pairing), 0x20 (firmware). Inspect new firmware
bytes via `ps5.latestPacket`.

## Pairing

The DualSense pairs with PS + Create. Bluedroid stores the link key in its own
NVS, so pairing survives reboots. The target MAC is RAM-only: after a reboot
`begin()` rescans; within a boot it reconnects to the latched MAC. `forget()`
drops it so the next retry rescans.

## Design decisions (don't regress)

- **Core ≥ 3.3.9 / 4.x**: include `esp32-hal-alloc-bt-classic-mem.h` or
  `initArduino()` frees Classic-BT RAM and `btStart()` fails.
- **IDF 6.x** `tL2CAP_APPL_INFO` may have an extra trailing pointer; our struct
  carries a null slot for it.
- **Per-channel disconnect**: Bluedroid reports each channel separately; clear
  only that channel's CID, fire link-down only on the up→down edge (else TX freezes
  after a one-channel blip).
- **No CONNECT_REQ on a half-open link**: retry only when both CIDs are 0. A failed
  outbound connect (`connect_cfm` ≠ OK) clears the control CID so retries continue.
- **Ignore non-0x31 reports**, otherwise `g_active` latches on the short 0x01
  report and the enable kick stops.
- **TX buffer** is `sizeof(BT_HDR) + 13 + len` (~100 B), not Bluedroid's 4 KB default.
- **latestPacket is a copy**: Bluedroid frees the RX buffer after the callback.
- Mic hardware is muted by default (`output.micMute = true`).

## Measured (ESP32-WROOM-32, core 3.3.12, 2026-11-23, v1.3.4 → v1.4.0)

| | before | after |
|---|---|---|
| `begin()` | 21,958 ms (no pad) / 5,838 ms (pad) | 455–474 ms |
| worst `loop()` stall, no pad | 5,500,000 µs | 4 µs |
| free heap, connected | 135,008 | 139,360 |
| min free heap, connected | 130,204 | 137,956 |
| `send()` | ~298 µs | ~283 µs (noise level) |
| flash (testEverything) | 1,057,204 | 1,055,512 |
| static RAM | 41,108 | 41,236 (+78 B latestPacket copy) |

## Known rough edges

- Input CRC32 is not verified.
- No `end()` / Bluedroid teardown.
- MAC spoofing: call `esp_base_mac_addr_set` before `begin()`.
