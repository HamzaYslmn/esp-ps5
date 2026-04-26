# `src/bluedroid/` — minimal Bluedroid internal API shim

Just one file: `bluedroid.h`.

## What it does

It hand-declares the ~50 Bluedroid internal symbols (`L2CA_*`, `BTM_*`,
`osi_*`, `BT_HDR`, `BD_ADDR`, ...) that `src/ps5Controller.cpp` calls
directly to talk to the precompiled Bluedroid blob shipped with
Arduino-ESP32.

Why a hand-curated shim instead of vendoring the upstream ESP-IDF
headers verbatim? The upstream headers transitively
`#include "bt_common.h"` → `"bt_user_config.h"` → `"sdkconfig.h"` ...,
none of which are on a user library's include path in Arduino.
Vendoring the full tree means dragging in dozens of files. The shim is
~150 lines and self-contained.

## Verified against

ESP-IDF **v5.5.2** (which matches Arduino-ESP32 **v3.3.6**). The
function signatures and constant values in `bluedroid.h` were copied
from upstream at that tag.

Upstream source paths (for cross-checking when something breaks):

| Symbols | Upstream file in espressif/esp-idf |
|---|---|
| `BD_ADDR`, `BT_HDR`, `BT_PSM_HIDC/HIDI`, `BT_DEFAULT_BUFFER_SIZE` | `components/bt/host/bluedroid/stack/include/stack/bt_types.h` |
| `tL2CAP_APPL_INFO`, `tL2CAP_CFG_INFO`, `L2CA_Register`, `L2CA_Deregister`, `L2CA_ErtmConnectReq/Rsp`, `L2CA_ConfigReq/Rsp`, `L2CA_DisconnectReq/Rsp`, `L2CA_DataWrite`, `L2CAP_MIN_OFFSET` | `components/bt/host/bluedroid/stack/include/stack/l2c_api.h` |
| `L2CAP_CONN_OK/PENDING`, `L2CAP_CFG_OK`, `L2CAP_DW_SUCCESS/CONGESTED` | `components/bt/host/bluedroid/stack/include/stack/l2cdefs.h` |
| `BTM_SetSecurityLevel`, `BTM_SEC_SERVICE_FIRST_EMPTY` | `components/bt/host/bluedroid/stack/include/stack/btm_api.h` |
| `osi_malloc`, `osi_free` (and the `_func` variants) | `components/bt/common/osi/include/osi/allocator.h` |

## When to update `bluedroid.h`

Only when a future Arduino-ESP32 / ESP-IDF release renames or changes
the signature of one of these symbols. Espressif keeps these internal
APIs fairly stable across minor versions.

## How to update

1. Find your Arduino-ESP32 core's ESP-IDF version (release notes on
   github.com/espressif/arduino-esp32).
2. Browse the upstream files listed in the table above at that tag,
   e.g. `https://github.com/espressif/esp-idf/blob/v5.5.2/components/bt/host/bluedroid/stack/include/stack/l2c_api.h`.
3. Patch `bluedroid.h` only where the symbol you use has changed.
4. Recompile and flash.

## Why not use ESP-IDF's public `esp_hidh` API?

`esp_hidh` (HID Host) would be a cleaner replacement, but it requires
`CONFIG_BT_HID_HOST_ENABLED=y` in the BT stack's sdkconfig. Arduino-ESP32
ships with that disabled, so `esp_hidh` symbols aren't linked into the
default Arduino-ESP32 BT blob. That's why we go below the HID layer and
talk to L2CAP directly via this shim.
