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

Note: pairing keys (link keys) are stored by Bluedroid itself in its own
NVS area, so a paired DualSense stays paired across reboots without us
needing to track its MAC. On every boot, `begin(timeoutSecs)` runs a
fresh BT inquiry; the moment any device named "DualSense" or
"Wireless Controller" appears, the scan early-exits and the L2CAP
connect fires. `ps5.forget()` clears the in-RAM target so the next
`begin()` re-scans from scratch.

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
  `ps5.forget()` wipes the entry. *(Removed 2026-05-01 — see below.)*

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

## Fixed (2026-05-01)

- **TX freezes after a brief link blip ("DOWN -> UP -> no TX")**:
  `disconnect_ind_cback` was wiping BOTH channels' state on every callback,
  but Bluedroid fires that callback PER CHANNEL. A blip on just one channel
  would also wipe the still-alive channel's CID. On the partial reconnect
  that followed, only the affected channel re-handshaked; the other's CID
  stayed at 0. RX kept working (Bluedroid routes inbound by its own internal
  lookup, `data_ind_cback` doesn't consult our stored CID), but `send()`
  silently dropped every frame against the cid==0 guard.
  **Fix**: only clear the CID + configured flag for the channel that was
  actually torn down; only fire `ps5ConnectEvent(0)` on the up->down edge.
  No mutex/locking needed - the partial-clear alone closes the bug.
- **Auto-reconnect "keeps disconnecting/reconnecting" loop**: `isConnected()`
  was firing `ps5_l2cap_reconnect()` (== outbound `L2CA_CONNECT_REQ` on HIDC)
  every 5 s whenever `g_active==false`, even when the L2CAP channels were
  already up but the first input packet hadn't landed yet (the gap between
  `ps5ConnectEvent(1)` and `ps5_mark_alive()`), or when only one channel had
  blipped and the other was still alive. The duplicate outbound connect
  confuses Bluedroid into tearing the existing link down -> repeats forever.
  **Fix**: exposed `ps5_l2cap_is_active()` (returns `is_connected`, i.e. both
  channels configured) and gated the 5-s reconnect on `!ps5_l2cap_is_active()`,
  so the retry only fires when the link is genuinely down.
- **`begin(saved-MAC)` fallback racing the active link**: was waiting on
  `g_active` (first input packet) with a 4-s deadline before falling back to a
  BT inquiry scan. If channels came up but the first packet was slow, the
  fallback would start `esp_bt_gap_start_discovery` while a connection was
  in flight, disrupting it. **Fix**: wait on `ps5_l2cap_is_active()` instead
  (channels-up is the right "saved MAC responded" signal) and bumped the
  deadline to 8 s for a known-paired controller to power-on + handshake.
- **NVS removed (KISS)**: the MAC-persistence layer (`nvsEnsure` / `macSave`
  / `macLoad` / `macErase` + the `begin()` saved-MAC fast path) was deleted.
  Bluedroid keeps the link key in its own NVS area, so a paired DualSense
  stays paired across reboots regardless. Every `begin(timeoutSecs)` now
  just runs a fresh inquiry that early-exits on the first DualSense match
  (~1–3 s typical). Saves ~50 lines + the `nvs_flash` dependency.
  `ps5.forget()` simplified to just clearing bluedroid's in-RAM target.

## Fixed (2026-05-02)

- **Chained edge API (`ps5.square.pressed` / `ps5.square.released`)**: every
  button is now a tiny `Button { bool cur; EdgeFlag pressed; EdgeFlag
  released; }` with `operator bool()` returning `cur`, so `if (ps5.square)`
  still works. `EdgeFlag::operator bool() const` is consume-on-read (sets
  itself false), so edges stay LATCHED until consumed — slow loops never
  miss a press. `parsePacket` updates each button via a local `setBtn(b, v)`
  lambda that sets `pressed.v=true` on rising edge and `released.v=true` on
  falling edge. Replaces the old `ps5.pressed(field)` / `ps5.released(field)`
  methods + the 24-slot `EdgeSlot _edges[24]` LRU table (~200 B RAM saved,
  zero address-keyed lookup overhead per call). Net win: cleaner API, less
  RAM, no missed edges.

## Fixed (2026-05-03)

- **Cross-task volatiles**: `g_active` (in `ps5Controller.cpp`) and `is_connected`
  (in `bluedroid.cpp`) are written from the Bluedroid task and read from the
  Arduino loop() task. They were plain `bool`, so the compiler was free to
  cache stale values in a register across the read inside `send()` /
  `isConnected()` — meaning the loop could keep skipping `send()` for many
  iterations after the controller had already gone live, or vice versa. Both
  are now `volatile bool`. Two-line fix, closes a latent race.
- **Removed `latestPacket`**: ~~the public `const uint8_t* latestPacket`
  pointer~~ — **REVERTED**: kept after all (now `const uint8_t* volatile`)
  because future protocol work (mic / speaker / new firmware bytes) needs
  raw access for reverse-engineering. Volatile guards against torn pointer
  reads from the loop() task.
- **Dropped SPP profile init**: `sppInit()` registered an SPP callback whose
  *only* job was to run `esp_bt_gap_set_scan_mode(CONNECTABLE,
  NON_DISCOVERABLE)` from inside `ESP_SPP_INIT_EVT`. SPP is not required for
  L2CAP HID — replaced with a direct `esp_bt_gap_set_scan_mode()` call in
  `ensureServices()`. Removes `sppCallback`, `sppInit`, and the
  `<esp_spp_api.h>` include (~25 lines).
- **Adaptive trigger DRY**: the seven mode builders (`buildOff` /
  `buildFeedback` / `buildWeapon` / `buildVibration` / `buildBow` /
  `buildGalloping` / `buildMachine`) repeated the same patterns: memset+mode
  set, two-zone bitmask packing, and tail-mask + 3-bit force packing.
  Extracted three shared helpers — `setOff()`, `writeMask()`, `packTail()`,
  plus `pairMaskStrict()` (Weapon/Machine: a in [lo..hi-1]) and
  `pairMaskLoose()` (Bow/Galloping: a in [lo..hi], allows degenerate
  collapse). The clamp split intentionally preserves byte-for-byte wire
  parity with the previous code, including the pre-existing 1-bit-mask
  edge case for Bow/Galloping at start=100%. Net: ~−45 LOC, identical
  wire output.
- **`isConnected()` also rescans when no target MAC**: previously the 5 s
  retry inside `isConnected()` only fired `ps5_l2cap_reconnect()` (outbound
  CONNECT_REQ to a known MAC). If `begin()`'s initial scan didn't see a
  controller (pad off, out of range, just paired), the loop would never
  re-scan and the only way to recover was to call `begin()` again. Now
  `isConnected()` checks `ps5_l2cap_has_target()` and runs `autoPair(4)`
  when no MAC is latched, so the sketch can sit in `if (!ps5.isConnected())
  return;` and recover from cold-start no-pad situations on its own. Single
  task — the retry still runs on the existing user-loop call, no
  `xTaskCreate` overhead.
- **Future-work protocol map**: added a "UNUSED / FUTURE-WORK BITS" section
  to `ps5_bytes.cpp` documenting every protocol feature we don't currently
  use, cross-checked against `drivers/hid/hid-playstation.c` (Linux master,
  2026). Covers: audio (mic capture + speaker volume + output path + preamp
  gain) — protocol-supported but needs a separate BT audio profile
  (HFP/A2DP) which Linux itself doesn't ship for DualSense; real mic mute
  via power-save register; vibration v2 (firmware >= 2.21); RELEASE_LEDS;
  charging error nibbles (0xA/0xB/0xF); persisted mic-mute status bit;
  mute LED pulse mode; feature reports 0x05 (motion calibration), 0x09
  (pairing info), 0x20 (firmware version); reserved input bytes (12,
  13..16, 33, 42..53, 56, 57..73) — inspect via `ps5.latestPacket` if new
  firmware ever lights any of them up.

## Fixed (2026-05-04)

- **Power-save mic mute**: new `ps5.micMute(bool)` fluent setter, plus
  `ps5.output.micMute` field. On `send()` we set `VF1 bit 1`
  (POWER_SAVE_CONTROL_ENABLE) and `byte 13 bit 4`
  (DS_OUTPUT_POWER_SAVE_CONTROL_MIC_MUTE) — this gates the actual mic
  hardware (audio capture), independent of the existing `muteLed()` visual.
  Linux kernel pairs these two together inside the mic-mute work handler.
- **Release LEDs**: new one-shot `ps5.releaseLeds()` setter. Sets `VF1
  bit 3` on the next frame and auto-clears the request bit, handing
  lightbar + player-LED control back to the controller's firmware
  (default boot animation returns until the sketch sets a colour again).
- **Charging error nibble**: new `ps5.chargingError` input bool. True when
  the charging nibble (input byte 54 bits 4..7) is `0xA` (voltage/temp out
  of range), `0xB` (temperature error), or `0xF` (charge fault). The pad
  will not accept charge until the user unplugs / cools / replugs.
  `charging` and `fullyCharged` already exist for the normal nibbles.
- **Persisted mic-mute status**: new `ps5.micMuted` input bool, set from
  `DS_STATUS1_MIC_MUTE` (input byte 55 bit 2). Tracks the controller's
  *firmware-persisted* mute state (toggled by the user tapping the mute
  button), separate from `ps5.mute` which is the raw button edge.
- **First-packet stall fix**: `isConnected()` now re-fires `ps5Enable()`
  (the SET_FEATURE 0xF4/0x43/0x02 handshake) every 400 ms while the
  L2CAP channels are up but no input report has arrived yet. The
  DualSense occasionally drops the very first enable packet right after
  channel setup, leaving the link in a permanent "channels up but
  silent" stall. Cheap (4 bytes on the control PSM) and stops as soon
  as the first packet flips `g_active`.
- **Mic hardware default = OFF**: `Out::micMute` now defaults to `true`,
  so on boot every send() carries the power-save bit (VF1 0x02 + byte 13
  bit 4) and the mic is electrically muted. ESP32 controller projects
  almost never need DualSense mic capture (BT audio profile isn't
  wired anyway), and the saved current pays for itself instantly. Any
  sketch that *does* want the mic live calls `ps5.micMute(false)`.
- **Example sketch**: `testEverything.ino` MUTE button handler now
  distinguishes short click (cycles `muteLed` 0→1→2 = off/solid/pulse)
  from long press ≥600 ms (toggles real `micMute()` hw mute). SHARE
  button is an on/off toggle that fires `releaseLeds()` and keeps
  re-arming that bit every frame so the firmware retains lightbar +
  player LEDs (second press flips back, sketch resumes writing them).
  The 1 Hz snapshot prints `chargingError`, `micMuted`, and the new
  `output.micMute` field. Removed the old "mute LED mirrors button"
  one-liner that was stomping the user's pulse selection every frame.

## Fixed (2026-05-06)

- **First-pair stall**: `isConnected()` no longer re-kicks the legacy 0xF4
  SET_FEATURE during the channels-up-but-silent window. It now sends a full
  79-byte output report on the interrupt PSM every 400 ms, which is what
  Bluepad32 (`uni_hid_parser_ds5.c`, `ds5_send_enable_lightbar_report`)
  documents as the production trigger: *"sending an output report enables
  input report 0x31"*. Same channel/format the input will come back on, so
  no encryption/timing race on initial pair.
- **Partial-tear reconnect race**: the 5-s `CONNECT_REQ` retry now also
  requires `!ps5_l2cap_has_any_cid()` (both control + interrupt CIDs zero),
  not just `!is_active`. Prevents firing CONNECT_REQ on a half-alive link
  where one channel blipped but the other is still configured. New helper
  `ps5_l2cap_has_any_cid()` exposed from bluedroid.cpp.
- **FUTURE-WORK trim** in `ps5_bytes.cpp`: removed subsections that landed
  in v1.3.1 (real mic mute, release LEDs, charging-error nibbles,
  persisted mic-mute, mute-LED pulse note). Kept audio (transport-blocked),
  vibration v2, feature reports 0x05/0x09/0x20, reserved input bytes.

## Fixed (2026-05-05)

- **Release-LEDs link drop ("when LEDs handed back to firmware, connection
  drops")**: `testEverything.ino`'s `pushOutputs()` was re-firing
  `ps5.releaseLeds().send()` every 10 ms throttle tick while the toggle was
  on, so the controller saw a continuous stream of output frames *with* the
  RELEASE_LEDS valid-flag bit set. Cross-checked against Linux
  `drivers/hid/hid-playstation.c` (mainline 2026): the kernel **defines**
  `DS_OUTPUT_VALID_FLAG1_RELEASE_LEDS = BIT(3)` but **never sets it
  anywhere** — strong implication that firmware reads it as "host is ceding
  control, stop sending output reports". Continuing to send output frames
  in that state is contradictory, and after a couple of seconds the firmware
  drops the L2CAP link as a reset. Fix is sketch-side: send `releaseLeds()`
  ONCE on the toggle press, then `pushOutputs()` early-returns (sends
  nothing) until the user toggles releaseLeds back off. Library-side
  `releaseLeds` semantics already auto-clear after one frame, so no library
  change was needed.
- **CROSS rebind for releaseLeds**: per user request the release-LEDs
  toggle moved from SHARE to CROSS (X face button). Updated handler name
  reads `ps5.cross.pressed`, log strings now `[CROS]` instead of `[SHR ]`.
- **Initial-pairing first-packet stall (still investigating)**: the
  research pass found the kernel does NOT use the `0x53 0xF4 0x43 0x02`
  magic packet to start full-report mode — it relies on issuing
  `GET_REPORT(FEATURE, 0x05)` (calibration) on the control PSM, and the
  controller's response flips it from minimal report `0x01` to streaming
  the full `0x31` reports. We kept the existing `ps5Enable()` for v1.3.1
  (the `0xF4` SET_FEATURE works ~95% of the time and changing it could
  regress the working path); flagged for a follow-up where we'd send the
  GET_REPORT(0x05) handshake instead and gracefully ignore the calibration
  blob it returns.
