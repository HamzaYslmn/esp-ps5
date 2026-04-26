# esp-ps5 — How the system works

An Arduino / ESP-IDF library that lets an ESP32 act as a Bluetooth Classic
host for a PS5 DualSense controller. The ESP32 pretends to be a PS5
console, the controller pairs with it, and the library exposes
buttons / sticks / triggers / status to the user sketch and accepts
LED / rumble / player-LED / mute-LED output back to the controller.

The protocol layer is a verified port of Sony's DualSense BT report
format (cross-checked against Linux's `drivers/hid/hid-playstation.c`).

## File layout (now 2 + 2 files)

```
   Sketch (.ino)
         |
         v
   ps5Controller.{h,cpp}    Arduino class + Bluedroid bring-up + scanDevices.
         |
         v
   ps5_bytes.cpp            ALL byte/bit work. Builds the wire-correct BT
                            0x31 OUTPUT report and parses the BT 0x31 INPUT
                            report. Every offset is documented at the top.
         |
         v
   ps5_l2cap.c              L2CAP transport (HID control PSM 0x11 + interrupt
                            PSM 0x13). Uses vendored Bluedroid headers.
   ps5_spp.c                Bluedroid GAP bring-up (device name, scan mode).
         |
         v
   stack/ + osi/            Vendored ESP-IDF Bluedroid internal headers.
                            (Implementations live in the Arduino-ESP32 core's
                            Bluedroid blob; only the headers are vendored
                            because ESP-IDF doesn't expose them publicly.)
```

The previous `ps5.{c,h}` + `ps5_int.h` + `ps5_parser.c` split is gone —
all of that work is now consolidated into `ps5_bytes.cpp` (protocol) and
`ps5Controller.cpp` (transport glue + Arduino class).

## How a connection happens

1. Sketch calls `ps5.begin("aa:bb:cc:..")` (or scans first via
   `ps5.scanDevices(secs, cb)` and feeds the picked MAC to `begin()`).
2. `ps5_l2cap_connect()` stores the controller MAC and immediately fires
   an outbound `L2CA_CONNECT_REQ` on the HID **control** PSM (0x11).
3. `begin()` then starts the BT controller, brings up Bluedroid, and
   registers HID **control** + **interrupt** PSMs (0x11 + 0x13) as
   listeners so the controller can also reconnect inbound.
4. Both L2CAP channels configure successfully → `is_connected` flips
   true → `ps5ConnectEvent(true)` fires.
5. `ps5ConnectEvent` calls `ps5Enable()`, which sends the magic feature
   report `{0x53, 0xF4, 0x43, 0x02}` on the **control** channel — this
   tells the DualSense to start streaming input reports.
6. First input report arrives at `ps5_l2cap_data_ind_cback` →
   `parsePacket()` decodes it into `ps5_t` and computes button-edge
   events into `ps5_event_t`, then calls `ps5PacketEvent`.
7. `ps5PacketEvent` treats the very first packet as the real "connected"
   moment and fires the user's `attachOnConnect` callback. Subsequent
   packets fire `attach`.

`isConnected()` auto-retries: when disconnected, it calls
`ps5_l2cap_reconnect()` at most every 5 seconds.

## DualSense Bluetooth wire format (verified vs Linux kernel)

### OUTPUT report (host -> controller, 79 bytes total)

Sent on the HID **interrupt** channel (PSM 0x13). Layout:

| Offset | Field                                                    |
|--------|----------------------------------------------------------|
|   0    | `0xA2` HID DATA\|OUTPUT header (covered by CRC)          |
|   1    | `0x31` report ID                                         |
|   2    | seq tag (low 4 bits = sequence, increments each frame)   |
|   3    | `0x10` (DualSense BT tag)                                |
|   4    | valid_flag0                                              |
|   5    | valid_flag1                                              |
|   6    | motor_right (small / high-frequency rumble)              |
|   7    | motor_left  (large / low-frequency rumble)               |
|  12    | mute LED (0=off, 1=on, 2=pulse)                          |
|  42    | valid_flag2                                              |
|  45    | lightbar setup byte                                      |
|  46    | LED brightness                                           |
|  47    | player LED bitmask (5 bits, bit0..bit4)                  |
|  48-50 | lightbar R / G / B                                       |
|  75-78 | CRC32 little-endian over bytes [0..74]                   |

Valid-flag bits that we set:

- `valid_flag0 |= 0x01` (compatible vibration)  ← rumble takes effect
- `valid_flag0 |= 0x02` (haptics select)
- `valid_flag1 |= 0x01` (mic-mute LED enable)
- `valid_flag1 |= 0x04` (lightbar enable)       ← RGB takes effect
- `valid_flag1 |= 0x10` (player LED enable)     ← player LEDs take effect
- `valid_flag2 |= 0x02` (lightbar setup enable)
- `lightbar_setup = 0x02` (LIGHT_OUT — disables the startup blue fade
  so user RGB takes effect immediately)

CRC32: zlib polynomial `0xEDB88320`, init `0xFFFFFFFF`, xorout
`0xFFFFFFFF`, computed over the 75 wire bytes including byte 0
(`0xA2`). LE-encoded into bytes 75..78.

Do **not** call `sendToController()` faster than every ~10 ms or the
L2CAP TX queue congests.

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
|  11    | PS / Touchpad / Mute / mic-mute toggle                   |
|  54    | status byte 0: battery level (low nibble) +              |
|        |                charging flag (bit 4) + full-charge (bit 5)|
|  55    | status byte 1: headphones (bit 0) + mic plug (bit 1)     |
|  74-77 | CRC32 (we don't currently verify it on input)            |

D-pad is decoded via a 9-entry `HAT_DECODE` table; eight diagonals plus
the neutral (`0x08`) value collapse to the four cardinal flags + four
diagonal flags in `ps5_button_t`.

## Sending output

`setLed`, `setRumble`, `setPlayerLeds`, `setMuteLed` all just **mutate
the local `output`** (`ps5_cmd_t`) on the C++ object. Nothing is
transmitted until the sketch calls `sendToController()`, which calls
`ps5SetOutput()` -> `ps5Cmd()` in `ps5_bytes.cpp`. `ps5Cmd()` builds
the full 79-byte frame, sets all valid flags listed above, increments
the sequence tag, computes the CRC32, and forwards to
`ps5_l2cap_send_hid_interrupt()`.

## User callbacks

C++ (preferred):
- `ps5.attach(cb)` — fires every input packet
- `ps5.attachOnConnect(cb)` — fires on first input packet (real "alive" moment)
- `ps5.attachOnDisconnect(cb)` — fires on L2CAP disconnect

The C-side dispatchers `ps5PacketEvent` / `ps5ConnectEvent` route into
`ps5Controller::_event_callback` / `_connection_callback`.

## Pairing model

The DualSense pairs to whatever Bluetooth address it last saw a console
respond from. The intended flow (same as esp32-ps3) is:

1. Use `sixaxispairer` (or equivalent) over USB to write the ESP32's BT
   MAC into the controller.
2. Hold PS + Create on the controller until the lightbar pulses.
3. The ESP32 either accepts the inbound connection, or initiates the
   outbound connect via `ps5.begin("<mac>")`.

The ESP32 keeps its factory BT MAC; the MAC passed to `begin()` is the
*controller's* address used to connect *to*.

## Build configuration

- `Kconfig`: only `IDF_COMPATIBILITY_STABLE` / `_MASTER` are kept now.
  The old per-revision `_21165ED` / `_D9CE0BB` / `_21AF1D7` snapshots
  were dropped — they referenced ESP-IDF revisions from 2018-2019 and
  hadn't been needed for years.
- `component.mk` makes this usable as an ESP-IDF component too.
- Bluetooth controller mode must include Classic BT (BTDM or
  BR/EDR-only); BLE-only builds will fail the `#error` check.

## Known rough edges

- Sensor parsing (gyro / accel / touchpad XY) is not wired up yet —
  the input report carries it (offsets 16+), but `parsePacket` doesn't
  expose it on `ps5_t`.
- We don't validate the incoming CRC32; bad packets would still parse.
- `ps5Controller::end()` is a stub — no Bluedroid teardown.
- `ps5SetBluetoothMacAddress` no longer exists (it was unused). If you
  need MAC spoofing, do it before `begin()` via `esp_base_mac_addr_set`.

## Fixed (2026-04-26)

- **Wire format**: was sending a wrong-sized frame on the wrong PSM
  with stale PS3-style offsets, so LED/rumble/player LEDs all silently
  no-op'd. Now uses the verified DualSense BT 0x31 layout (79 bytes,
  tag = 0x10, all required valid_flag bits, CRC32 over byte 0
  inclusive, sent on the interrupt channel).
- **Input parser**: was reading PS3 offsets (11+) instead of DualSense
  offsets (2+); sticks and buttons report correctly now.
- **2-file rewrite**: previous 5-file split (`ps5.{c,h}` +
  `ps5_int.h` + `ps5_parser.c` + `ps5Controller.{h,cpp}`) collapsed
  to two files with a clear protocol/transport split.
- **API additions**: `setPlayerLeds(bitmask)` and `setMuteLed(mode)`
  replace the old `setFlashRate` (which was a no-op anyway).
  `Mute()` accessor returns the mic-mute capacitive button state.
