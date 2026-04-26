# esp-ps5 — How the system works

An Arduino / ESP-IDF library that lets an ESP32 act as a Bluetooth Classic
host for a PS5 DualSense controller. The ESP32 pretends to be a PS5
console, the controller pairs with it, and the library exposes
buttons / sticks / triggers / status to the user sketch and accepts
LED / rumble / player-LED / mute-LED / adaptive-trigger output back to
the controller.

The protocol layer is a verified port of Sony's DualSense BT report
format (cross-checked against Linux's `drivers/hid/hid-playstation.c`).

## File layout

```
   Sketch (.ino)
        |
        v
   ps5Controller.{h,cpp}      Arduino class: scan/pair/auto-reconnect,
                              fluent output API, user callbacks. Owns the
                              global `ps5` instance with all input as flat
                              public fields.
        |
        v
   ps5_bytes.cpp              ALL byte/bit work. Builds the wire-correct
                              BT 0x31 OUTPUT report (rumble, lightbar, LEDs,
                              adaptive triggers) with trailing CRC32, and
                              parses the BT 0x31 INPUT report straight into
                              the global `ps5` flat fields.
        |
        v
   bluedroid/bluedroid.cpp    L2CAP transport (HID PSMs 0x11 + 0x13) +
                              GAP/SPP bring-up. Vendored Bluedroid plumbing.
   bluedroid/                 Vendored ESP-IDF Bluedroid internal headers.
                              (Implementations live inside the Arduino-ESP32
                              core's Bluedroid blob; only the headers are
                              vendored because ESP-IDF doesn't expose them.)
```

## How a connection happens

1. Sketch calls `ps5.begin()` (or `ps5.begin(secs)` / `ps5.begin("AA:BB:..")`).
2. `begin()` brings up the BT controller, Bluedroid, GAP, SPP, and registers
   HID **control** + **interrupt** PSMs (0x11 + 0x13) as L2CAP listeners.
3. With no MAC stored yet, `begin()` runs an auto-pair scan: every unique
   nearby device is added to a **growing linked list** (one node per MAC,
   ~10 B). Scan early-exits the moment a device named "DualSense" or
   "Wireless Controller" appears. Its MAC is fed to `ps5_l2cap_connect()`,
   which fires an outbound `L2CA_CONNECT_REQ` on PSM 0x11.
4. Both L2CAP channels configure successfully → on the up-edge, bluedroid
   calls `ps5_scan_cache_release()` (frees the linked list, RAM goes back
   to baseline) and `ps5ConnectEvent(true)` fires.
5. `ps5ConnectEvent` calls `ps5Enable()`, which sends the magic feature
   report `{0x53, 0xF4, 0x43, 0x02}` on the **control** channel — this
   tells the DualSense to start streaming BT 0x31 input reports.
6. First input report arrives at `ps5_l2cap_data_ind_cback` →
   `parsePacket()` writes every field directly onto `ps5` (sticks,
   buttons, gyro, accel, touchpad, status), then calls `ps5_mark_alive()`.
7. `ps5_mark_alive()` flips `g_active` true on its first call, fires the
   user's `attachOnConnect` callback (real "alive" moment), and on every
   subsequent call fires `attach`.

`isConnected()` auto-retries: when disconnected, it calls
`ps5_l2cap_reconnect()` at most every 5 seconds.

## DualSense Bluetooth wire format (verified vs Linux kernel)

### OUTPUT report (host -> controller, 79 bytes total)

Sent on the HID **interrupt** channel (PSM 0x13). Layout:

| Offset | Field                                                    |
|--------|----------------------------------------------------------|
|   0    | `0xA2` HID DATA\|OUTPUT header (covered by CRC)          |
|   1    | `0x31` report ID                                         |
|   2    | seq tag (high 4 bits = sequence, increments each frame)  |
|   3    | `0x10` (DualSense BT tag)                                |
|   4    | valid_flag0                                              |
|   5    | valid_flag1                                              |
|   6    | motor_right (small / high-frequency rumble)              |
|   7    | motor_left  (large / low-frequency rumble)               |
|  12    | mute LED (0=off, 1=on, 2=pulse)                          |
|  14    | R2 trigger mode + 10 param bytes [15..24]                |
|  25    | L2 trigger mode + 10 param bytes [26..35]                |
|  42    | valid_flag2                                              |
|  45    | lightbar setup byte                                      |
|  46    | LED brightness                                           |
|  47    | player LED bitmask (5 bits, bit0..bit4)                  |
|  48-50 | lightbar R / G / B                                       |
|  75-78 | CRC32 little-endian over bytes [0..74]                   |

Valid-flag bits that we set:

- `valid_flag0 |= 0x01` (compatible vibration)  ← rumble takes effect
- `valid_flag0 |= 0x02` (haptics select)
- `valid_flag0 |= 0x04` (R2 adaptive trigger enable)
- `valid_flag0 |= 0x08` (L2 adaptive trigger enable)
- `valid_flag1 |= 0x01` (mic-mute LED enable)
- `valid_flag1 |= 0x04` (lightbar enable)       ← RGB takes effect
- `valid_flag1 |= 0x10` (player LED enable)     ← player LEDs take effect
- `valid_flag2 |= 0x01` (light brightness enable) ← byte 46 (player LED brightness) takes effect
- `valid_flag2 |= 0x02` (lightbar setup enable)
- `lightbar_setup = 0x02` (LIGHT_OUT — disables the startup blue fade)

Adaptive trigger modes used by the fluent API (per Nielk1's verified spec):
- `0x05` Off / Reset — all 10 param bytes 0.
- `0x21` Feedback (Rigid wall) — params: `activeZones` u16 LE at p1..p2 +
  `forceZones` u32 LE at p3..p6 (3 bits per zone, value `(strength-1)&7`).
  Setter: `l2Rigid(startPct, strengthPct)`. `pos = pctToPos(start)` selects
  zones [pos..9]; strength 0 collapses to mode 0x05.
- `0x25` Weapon (Trigger break) — params: p1..p2 = `(1<<startZone)|(1<<endZone)`,
  p3 = `strength-1`. startZone clamped 2..7, endZone clamped start+1..8.
  Setter: `l2Trigger(startPct, endPct, strengthPct)`.
- `0x26` Vibration (Pulse) — same active+amplitude packing as Feedback,
  plus p9 = frequency (raw Hz). Setter: `l2Pulse(startPct, ampPct, freqHz)`.
  freqHz==0 or amplitude==0 collapses to mode 0x05.
- `0x22` Bow (unofficial combo, Weapon + snap-back) — p1..p2 =
  `(1<<startZone)|(1<<endZone)`; p3..p4 u16 LE = `((str-1)&7) | ((snap-1)&7)<<3`.
  Setter: `l2Bow(startPct, endPct, strengthPct, snapPct)`.
- `0x23` Galloping (unofficial, rhythmic pulses bounded by zones) — p1..p2 =
  zone bitmask; p3 = `(foot2&7) | (foot1&7)<<3` (foot1 0..6, foot2 first+1..7);
  p4 = freqHz. Setter: `l2Galloping(start, end, foot1, foot2, freqHz)`.
- `0x27` Machine (unofficial combo, **Trigger range + Pulse oscillating between
  two amplitudes**) — p1..p2 = zone bitmask; p3 = `(ampA&7) | (ampB&7)<<3`
  (ampA/ampB are RAW 0..7, NOT `-1`); p4 = freqHz; p5 = period in tenths of a
  second between A↔B oscillation. Setter:
  `l2Machine(start, end, ampAPct, ampBPct, freqHz, periodTenths)`.

L2 and R2 triggers each have only ONE 11-byte mode block per frame, so the
basic modes (Rigid / Trigger / Pulse) cannot be stacked on the same trigger.
The combo modes above (Bow / Galloping / Machine) are the firmware's built-in
"two effects at once" — Machine in particular gives you Trigger-range + Pulse
in a single mode. L2 and R2 ARE independent: `ps5.l2Trigger(...).r2Pulse(...)`
runs both simultaneously.

The deprecated "Simple_Feedback" (0x01) / "Simple_Weapon" (0x02) /
"Simple_Vibration" (0x06) modes from the early reverse-engineering era are
NOT used — the kernel comment in Nielk1's gist explicitly says "do not use".

CRC32: zlib polynomial `0xEDB88320`, init `0xFFFFFFFF`, xorout
`0xFFFFFFFF`, computed over the 75 wire bytes including byte 0
(`0xA2`). LE-encoded into bytes 75..78.

Do **not** call `ps5.send()` faster than every ~10 ms or the L2CAP TX
queue congests.

### INPUT report (controller -> host, 78 bytes)

Parsed by `parsePacket()` in `ps5_bytes.cpp`. Wire offsets:

| Offset | Field                                                    |
|--------|----------------------------------------------------------|
|   0    | `0x31` report ID                                         |
|   1    | reserved (BT seq)                                        |
|   2-5  | LX, LY, RX, RY (uint8, centered at 128, Y inverted)      |
|   6-7  | L2 / R2 analog triggers (0..255)                         |
|   8    | report counter                                           |
|   9    | low nibble = D-pad direction (0..7 + 8=neutral) +        |
|        | high nibble = face buttons (square/cross/circle/triangle)|
|  10    | L1 / R1 / L2 / R2 / Create / Options / L3 / R3           |
|  11    | PS / Touchpad / Mute                                     |
| 17-22  | gyro X/Y/Z (le16 each, raw int16)                        |
| 23-28  | accel X/Y/Z (le16 each, raw int16)                       |
| 29-32  | sensor timestamp (le32, 0.33 us per LSB)                 |
| 34-41  | touchpad: two 4-byte contacts                            |
|  54    | status byte 0: battery (0..10) + charging nibble         |
|  55    | status byte 1: headphones (bit 0) + mic plug (bit 1)     |
|  74-77 | CRC32 (we don't currently verify it on input)            |

D-pad decoding uses a 9-entry `HAT_DECODE` table. `parsePacket` exposes
the four cardinals (`up`, `down`, `left`, `right`); diagonals come
naturally from logical AND, e.g. NE = `ps5.up && ps5.right`.

## Sending output

The fluent setters (`lightbar`, `rumble`, `playerLed`,
`muteLed`, `l2*`/`r2*`) just **mutate the public `ps5.output` struct**.
Nothing is transmitted until the sketch calls `ps5.send()`, which calls
`ps5BuildAndSend()` in `ps5_bytes.cpp`. That builds the full 79-byte
frame, sets all valid flags listed above, increments the sequence tag,
computes the CRC32, and forwards to `ps5_l2cap_send_hid_interrupt()`.

## User callbacks

- `ps5.attach(cb)` — fires every input packet (~250 Hz).
- `ps5.attachOnConnect(cb)` — fires on first input packet (real "alive" moment).
- `ps5.attachOnDisconnect(cb)` — fires on L2CAP disconnect.

## Pairing model

The DualSense pairs to whatever Bluetooth address it last saw a console
respond from. The intended flow is:

1. Use `sixaxispairer` (or equivalent) over USB to write the ESP32's BT
   MAC into the controller.
2. Hold PS + Create on the controller until the lightbar pulses.
3. The ESP32 either accepts the inbound connection, or initiates the
   outbound connect via `ps5.begin()` (auto-scan) or `ps5.begin("<mac>")`.

The ESP32 keeps its factory BT MAC; the MAC passed to `begin()` is the
*controller's* address used to connect *to*.

### NVS persistence (auto-reconnect without re-pairing)

The first time a controller goes fully alive (first input packet arrives),
`ps5_mark_alive()` reads the latched target MAC out of bluedroid via
`ps5_l2cap_get_target()` and writes it to NVS under namespace `"ps5"`,
key `"mac"` (6-byte blob).

On the next boot, `ps5.begin(timeoutSecs)` reads that MAC back and fires a
direct outbound L2CAP connect, waits up to 4 s for the controller to come
alive, and only falls back to a full BT inquiry if the saved MAC doesn't
respond. Result: a previously-paired DualSense reconnects in ~1 s without
re-pairing.

Call `ps5.forget()` to wipe the NVS entry and clear bluedroid's latched
MAC — useful when switching to a different controller.

## Memory / CPU notes

- Scan dedupe uses a singly-linked list (~10 B/node). Grows during the
  inquiry, freed automatically on the L2CAP up-edge or at the end of a
  user-driven `scanDevices()` call. There is no fixed cap.
- `parsePacket` writes straight into the public `ps5` flat fields — no
  intermediate `ps5_t`/`ps5_event_t` copy.
- Output frame is built into a stack-allocated `hid_cmd_t` (80 B) and
  copied once into the L2CAP TX buffer. No persistent output cache.

## Build configuration

- `Kconfig`: only `IDF_COMPATIBILITY_STABLE` / `_MASTER` are kept.
- `component.mk` makes this usable as an ESP-IDF component too.
- Bluetooth controller mode must include Classic BT (BTDM or
  BR/EDR-only); BLE-only builds will fail the `#error` check.

## Known rough edges

- We don't validate the incoming CRC32; bad packets would still parse.
- No clean Bluedroid teardown (there's no `end()` method anymore).
- If you need MAC spoofing, do it before `begin()` via
  `esp_base_mac_addr_set`.

## Fixed (2026-04-26)

- **Wire format**: previous library was sending a wrong-sized frame on
  the wrong PSM with stale PS3-style offsets, so LED/rumble/player LEDs
  all silently no-op'd. Now uses the verified DualSense BT 0x31 layout
  (79 bytes, tag = 0x10, all required valid_flag bits, CRC32 over byte 0
  inclusive, sent on the interrupt channel).
- **Input parser**: was reading PS3 offsets (11+) instead of DualSense
  offsets (2+); sticks and buttons report correctly now.
- **API simplification (KISS / DRY)**: dropped the `ps5_t` / `ps5_event_t`
  / `ps5_cmd_t` nested structs, the duplicated `set*` / `Cross()` /
  `LStickX()` accessors, the `connected()` alias, the public `autoPair()`
  / `end()` / `LatestPacket()` symbols, and the fixed-cap `gSeen[16]`
  scan dedupe. Input is now plain public fields on the global `ps5`;
  scan dedupe is a linked list freed on connect.
- **Adaptive triggers**: `l2*()` / `r2*()` chainable setters with
  percent-unit arguments; modes Off / Rigid / Trigger / Pulse.

## Fixed (2026-04-27)

- **Adaptive trigger byte encoding**: previous build used the deprecated
  "Simple_*" modes 0x01 / 0x02 / 0x06 with flat (pos, strength, freq)
  bytes \u2014 those modes are documented as "do not use" by the kernel /
  Nielk1 and the controller silently rejects them on most firmwares.
  Now uses the official zone-bitmask format: Feedback `0x21`, Weapon
  `0x25`, Vibration `0x26`, with active-zone u16 LE + 3-bit-packed force
  u32 LE in p1..p6 (Vibration adds raw-Hz frequency at p9). Rigid /
  Pulse with strength 0 collapse cleanly to mode 0x05.
- **NVS auto-reconnect**: ESP32 now persists the controller's BT MAC to
  NVS (`ps5`/`mac`) the first time a packet arrives, so subsequent
  `begin()` calls skip the BT inquiry and fast-connect directly. Falls
  back to a full scan if the saved controller is unreachable.
  `ps5.forget()` wipes the entry.

## Fixed (2026-04-28)

- **Multi-effect adaptive triggers**: added `l2Bow` / `l2Galloping` / `l2Machine`
  (and `r2*` mirrors) for the firmware's built-in combo modes. Machine
  (`0x27`) is what people typically mean by "Trigger + Pulse together" — it
  bounds a vibration between start/end zones AND oscillates the amplitude
  between two values at a configurable period. Bow (`0x22`) is Weapon with
  an extra snap-back pull. Galloping (`0x23`) is rhythmic bounded pulses.
  Per-trigger you still get only one mode at a time (one 11-byte block on
  the wire); these are the firmware's pre-baked combinations.

## Fixed (2026-04-29)

- **Unified player-LED API**: replaced the old `playerLeds(mask)` +
  `ledBrightness(level)` pair (and the `PS5_LED_HIGH/MEDIUM/LOW` macros)
  with a single fluent setter `ps5.playerLed(idx, value)`. `value=0` clears
  bit `idx` of the player-LED mask without touching brightness; non-zero
  sets the bit AND writes the brightness register, where 1=dim, 2=mid,
  3+=bright (mapped internally to wire 2/1/0). The required valid-flag bit
  for brightness (`valid_flag2 bit 0`, undocumented in the kernel because
  it never writes byte 46) is OR'd in alongside the lightbar-setup bit so
  the controller actually applies the level. Previously brightness silently
  no-op'd.
- **`led(r,g,b)` renamed to `lightbar(r,g,b)`** — disambiguates from the
  player LEDs.
- **Edge detection in the library**: `ps5.pressed(field)` and
  `ps5.released(field)` track per-bool previous state inside the library
  (24-slot LRU table keyed by the field's address), so sketches no longer
  need their own `wasX` shadow flags.
- **Battery scale**: `ps5.battery` is now 0..100 percent (was 0..10 raw
  hardware units). Conversion happens in `parsePacket`: `batt * 10`.
- **Stick / trigger percent helpers**: added inline `lxPct()/lyPct()/
  rxPct()/ryPct()/l2Pct()/r2Pct()` on the global `ps5` for sketches that
  prefer -100..+100 sticks / 0..100 triggers over raw 0..255 bytes.

## Fixed (2026-04-30)

- **Y-stick sign convention**: `parsePacket` was computing `ly = 127 - raw`,
  which mapped raw center 128 → -1 (off-by-one) AND made push-UP positive,
  contradicting the public docs that promise "push UP = negative". The
  DualSense wire already has UP at low raw and DOWN at high raw, so Y now
  uses the same `raw - 128` formula as X. Result: center → 0, push-UP →
  −128, push-DOWN → +127 — matches the documented convention and X axis.
- **Bow / Galloping zone clamping**: previous order
  (`if(b<=a) b=a+1; if(b>8) b=8;`) collapsed both endpoints to the same
  bit when `start=100%` (a=8 → b=9 → b=8), producing an invalid 1-bit
  zone mask. Reordered to clamp absolute bounds first, then enforce
  `b > a` only when there's room. Galloping had the same shape and was
  fixed identically.
- **L2CAP up-edge race**: `ps5_l2cap_config_cfm_cback` was setting
  `is_connected = (cid == interrupt_channel)`, so if the stack ever
  re-confirmed the control channel after the interrupt channel (the
  AGENTS.md spec requires both channels to be configured), it spuriously
  flipped to disconnected. Now tracks each channel independently and the
  up-edge fires only when both are configured. Disconnect callback also
  zeros both CIDs and the configured flags so reconnects start clean.
- **Edge-slot table**: bumped from 16 to 24 entries to comfortably hold
  all 17 button bools plus user-added ones.
- **CRC32 table moved to flash**: previously a 1 KB `static uint32_t
  crc32_table[256]` lazy-initialised on first `send()`. Now built at
  compile time with a recursive `constexpr` step + macro fan-out, so the
  table lives in `.rodata` (flash) and costs **0 bytes RAM**. First-send
  latency also drops since there's no init loop. Net win: −1024 B RAM.
- **Header comment**: a `/* ... */` block in `ps5Controller.h` had a
  missing `*/` and silently swallowed the `playerLed()` declaration,
  giving sketches a `'class ps5Controller' has no member named 'playerLed'`
  link error. Fixed by closing the comment.
