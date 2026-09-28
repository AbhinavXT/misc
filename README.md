# updater — Multi-protocol Fail-Safe Bootloader for Kavach ATP

`updater` is the bootloader that runs on every controller card of a Kavach ATP unit (VCC, Input, Output and Analog cards) on the TI AM263x (AM2634) Cortex-R5F. It runs at every reset, ahead of the application. It does three jobs:

1. **Receives** a new firmware image (`.appimage`) over UDP/Ethernet, CAN-FD or UART.
2. **Verifies** it block by block (CRC-32) and as a whole (CRC-32 + SHA-256).
3. **Commits** it to a dual-bank QSPI flash and boots it, rolling back if the new image fails to confirm itself.

If nothing arrives within 5 s, it boots the existing application. A node never waits forever in the bootloader, and a failed update never leaves it unbootable.

This code accompanies the paper *"A Multi-protocol Fail-Safe Bootloader Framework for Remote Firmware Updates in Automatic Train Protection Systems"* (CRL Technical Symposium 2026). See [Implementation status](#implementation-status) for where the code and the paper currently differ.

---

## Contents

- [How it works](#how-it-works)
- [Repository layout](#repository-layout)
- [Building](#building)
- [Flashing an image](#flashing-an-image)
- [Wire protocol](#wire-protocol)
- [Flash layout and boot metadata](#flash-layout-and-boot-metadata)
- [Boot decision and rollback](#boot-decision-and-rollback)
- [Application integration](#application-integration)
- [Configuration reference](#configuration-reference)
- [Implementation status](#implementation-status)
- [Troubleshooting](#troubleshooting)

---

## How it works

```
                     UDP :50001                      CAN-FD (MCAN)
 ┌──────────────┐   all images    ┌──────────────┐  peripheral images  ┌──────────────┐
 │  Host PC     │ ──────────────▶ │  VCC         │ ──────────────────▶ │ Input card   │
 │  udp_file    │ ◀────────────── │  (master)    │                     │ Output card  │
 └──────────────┘  STATUS replies │              │ ─ ─ ─ ─ ─ ─ ─ ─ ─ ▶ │ Analog card  │
                                  └──────────────┘   UART (fallback)   └──────────────┘
                                   own dual-bank                        own dual-bank
```

1. The host sends a **META** packet naming the target card, the image size, the whole-image CRC-32 and SHA-256.
2. The host streams **DATA** blocks. The VCC checks each block's CRC-32, stores it in RAM and sets its bit in a coverage bitmap.
3. The host sends **POLL**. The VCC replies with **STATUS**: `INCOMPLETE` plus the bitmap, and the host resends only the missing blocks. This repeats until `COMPLETE`.
4. When coverage completes, the VCC runs the whole-image CRC-32 and SHA-256 once and caches the result.
5. The VCC replies `COMPLETE` (3×) **before** it acts, so the host always learns of success.
6. It then routes by `card_type`:
   - **VCC image:** writes it to its own inactive flash bank and reboots.
   - **Peripheral image:** re-sends it over CAN-FD with the same META/DATA format. The peripheral verifies and commits it to its own inactive bank.

Everything runs in a single polled super-loop (`main.c`). Interrupts only enqueue received bytes/frames. Long operations such as flash writes hold the loop until they finish.

---

## Repository layout

```
updater/
├── main.c                          Super-loop: poll links, commit, boot watchdog
├── example.syscfg                  SysConfig: pins, UART, MCAN, QSPI, Enet, bootloader
├── makefile_ccs_bootimage_gen      Post-build: .out → .rprc → .appimage
├── source/
│   ├── inc/
│   │   ├── common_include.h        ★ Card selection (#define VCC / INPUT / ...)
│   │   ├── const.h                 Card types, controller IDs, CAN message IDs
│   │   └── crc32.h, utility.h
│   ├── src/                        CRC-32 (0x04C11DB7), helpers
│   └── custom_drivers/
│       ├── FIRMWARE_UPDATER/       ★ Transfer core: META/DATA/POLL/STATUS, bitmap, verify
│       ├── BOOTLOADER/             ★ Boot decision, attempt counter, rollback, ConfirmBoot
│       ├── QSPI/                   ★ Dual-bank write, metadata (backup-first)
│       ├── ETHERNET/               lwIP init, UDP socket, reply path to host
│       ├── MCAN/                   CAN-FD driver + RX matrix, image forwarding
│       ├── UART/                   UART driver + firmware framing transport
│       ├── SHA256/                 SHA-256 on the DTHE hardware accelerator
│       ├── GPIO/                   Card slot / CPU ID detection
│       └── syscfg/                 Generated SysConfig output, linker.cmd
└── targetConfigs/                  CCS debug target config
```

★ marks the files you'll touch most.

---

## Building

### Prerequisites

- Code Composer Studio (CCS) with the TI Arm Clang compiler
- TI **MCU+ SDK for AM263x** (`COM_TI_MCU_PLUS_SDK_AM263X_INSTALL_DIR` must be set in CCS)
- SysConfig (bundled with CCS)

### 1. Select the card type

One source tree builds all four card variants. Pick the target in `source/inc/common_include.h` by leaving exactly one line uncommented:

```c
#define VCC                   1
//#define INPUT                 2
//#define OUTPUT                3
//#define ANALOG                4
```

`VCC` enables lwIP, the UDP receiver and CAN forwarding. Any other choice builds a peripheral that listens on CAN-FD/UART with 47 B blocks.

### 2. Build

1. Import the `updater` project into CCS (**File → Import → CCS Projects**).
2. Select the **Release** configuration.
3. Build. The post-build step (`makefile_ccs_bootimage_gen`) produces `updater.out`, `updater.rprc` and an `.appimage`.

### 3. Program the bootloader

Program the bootloader to the start of QSPI flash (`0x000000`) with the standard MCU+ SDK flashing flow (UART uniflash or CCS). This happens once per board at production. The updater **never rewrites its own region**.

On first boot, both metadata slots are empty. The updater then defaults to **Bank A, confirmed**, and treats whatever is in Bank A as the trusted base image.

---

## Flashing an image

Application images are sent from the host with the PC tool `udp_file` (`pc_flah_code/udp_file_v4.cpp` / `v5`), which is maintained separately and not in this repository.

| Setting | Value |
|---|---|
| VCC IP address | `192.168.25.168` (netmask `255.255.0.0`, gateway `192.168.25.1`) |
| UDP port | `50001` |
| Block size (host → VCC) | `1450` bytes; must match `FW_BLOCK_SIZE` |
| Max image size | `800 KB` (`MAX_IMAGE_SIZE`) |
| Card type in META | `1` VCC · `2` Input · `3` Output · `4` Analog |

The VCC sends STATUS replies to whichever address and port the last packet came from.

A typical session:

1. Power-cycle or reset the unit. Every card sits in the updater for 5 s.
2. Start the host tool within that window, pointing at the image and card type.
3. The host announces, streams and repairs until it receives `COMPLETE`.
4. The target commits the image and reboots into a **trial boot**.
5. The new application must call `Bootloader_ConfirmBoot()` (see [Application integration](#application-integration)). Otherwise it rolls back after 3 resets.

For a peripheral image, the VCC tells the target card "update incoming" as soon as it reads the META. The card then widens its wait window from 5 s to 60 s, so it doesn't boot away while the VCC is still receiving.

---

## Wire protocol

All links carry the same four messages. Every message starts with a one-byte type. All fields are packed and little-endian.

| Type | Value | Direction | Size | Fields after the type byte |
|---|---|---|---|---|
| `META` | `0x00` | host → target | 42 B | `card_type` (1), `totalSize` (4), `image_crc` (4), `image_sha256` (32) |
| `DATA` | `0x01` | host → target | 13 B + payload | `offset` (4), `size` (4), `chunk_crc` (4), then payload |
| `POLL` | `0x02` | host → target | 5 B | `image_crc` (4): session id |
| `STATUS` | `0x03` | target → host | 16 B (+ bitmap) | `status` (1), `image_crc` (4), `total_blocks` (4), `blocks_received` (4), `bitmap_bytes` (2), then bitmap if `INCOMPLETE` |

### STATUS outcomes

| Code | Name | Meaning | Host action |
|---|---|---|---|
| `0x00` | `COMPLETE` | Every block received; CRC-32 and SHA-256 passed | Stop. The target now commits. |
| `0x01` | `INCOMPLETE` | Blocks missing; bitmap attached (bit *i* set = block *i* OK) | Resend blocks with clear bits, poll again |
| `0x02` | `NEED_META` | No session, or POLL's `image_crc` doesn't match | Resend META, restart |
| `0x03` | `IMAGE_FAIL` | Every block present, but whole-image check failed | Restart the full transfer |

### Rules the sender must follow

- `offset` must be block-aligned (`offset % block_size == 0`). Every block is full size except the last.
- `chunk_crc` is CRC-32 over the payload bytes only.
- `image_crc` from META is the session id. A POLL with any other value gets `NEED_META`.
- Duplicate blocks are harmless: they overwrite the same slot and don't advance coverage.

### Per-link framing

| Link | Block size | Framing |
|---|---|---|
| UDP/Ethernet | 1450 B | One message per datagram (fits the 1472 B UDP payload) |
| CAN-FD | 47 B | 64 B frames: 4 B link assembly + 13 B header + 47 B payload (`DATA_FRAME_PAYLOAD_SIZE = 60`) |
| UART | 47 B | `AA AA` · len (2) · payload · CRC-32 (4) · `BB BB`, at 115200 baud |

---

## Flash layout and boot metadata

8 MB QSPI NOR flash (`qspi_user.h`):

| Offset | Size | Region |
|---|---|---|
| `0x000000` | 2 MB reserved | Bootloader / updater (never self-modified) |
| `0x200000` | 1 MB | **Bank A**: image slot |
| `0x300000` | 1 MB | **Bank B**: image slot |
| `0x400000` | 2 MB | unused |
| `0x600000` | 1 erase block | Metadata, **primary** |
| `0x610000` | 1 erase block | Metadata, **backup** |

`BootMetadata` is stored identically in both slots:

| Field | Purpose |
|---|---|
| `magic` | `0xA55AB00B`, which neither erased (`0xFF`) nor zeroed flash can match |
| `active_bank` | `BANK_A` (0) or `BANK_B` (1) |
| `bank_x_offset/size/crc` | Location, length and CRC-32 of each bank's image |
| `confirmed` | Set by the application once it has proven itself |
| `attempt_count` | Trial boots so far without confirmation |
| `crc_self` | CRC over the record, which detects a torn write |

### Power-safe commit order (`QSPI_Write_Full_Image`)

1. If the new image CRC equals the active bank's CRC, skip it as a duplicate.
2. Erase the **inactive** bank. The running image is never touched.
3. Write the image, then read it back and compare byte for byte.
4. Build new metadata: `active_bank` = new bank, `confirmed = 0`, `attempt_count = 0`.
5. Write the **backup** metadata slot first, then the **primary**.

If power fails at any step, one valid image and one valid metadata copy survive. On read, the updater takes the primary if its `crc_self` is valid. Otherwise it takes the backup and repairs the primary from it. If both are bad, it defaults to Bank A, confirmed.

---

## Boot decision and rollback

`Boot_Switch()` in `bootloader_user.c` runs whenever the updater decides to leave:

```
read metadata ─▶ active bank CRC valid?
                   │ no ─▶ other bank valid? ─▶ boot it  │  else stay in updater
                   │ yes
                   ▼
                confirmed? ── yes ─▶ boot (no flash write)
                   │ no
                   ▼
          attempt_count ≥ 3? ── no ─▶ attempt_count++, save, trial boot
                   │ yes
                   ▼
          roll back: active_bank = other bank, confirmed = 1, attempts = 0, boot it
```

A confirmed image boots with **zero flash writes**, so steady-state operation causes no flash wear. Each trial boot costs at most one metadata write, and there are at most 3 per update.

---

## Application integration

Every application image deployed through this updater **must** confirm itself, or it rolls back after 3 resets.

```c
#include "bootloader_user.h"

/* Call once the application has proven it is healthy:
 * e.g. joined the CAN bus, passed self-test, ran N seconds stably. */
if (Bootloader_ConfirmBoot() != SystemP_SUCCESS)
{
    /* Metadata write failed. Retry later; until it succeeds,
     * every reset counts as another trial boot. */
}
```

Guidelines:

- **Don't call it at the top of `main()`.** An image that confirms and then crashes seconds later can never roll back.
- It's **idempotent**: calling it when already confirmed does nothing and writes nothing.
- Arm the **hardware watchdog** before confirming. The attempt counter only advances on a reset, so an image that hangs without resetting is never counted.

---

## Configuration reference

| Constant | Default | File | Meaning |
|---|---|---|---|
| `MAX_IMAGE_SIZE` | 800 KB | `firmware_update_user.h` | RAM staging buffer size and maximum image size |
| `FW_BLOCK_SIZE` | 1450 | `firmware_update_user.h` | UDP block size; must match host |
| `IOA_FW_BLOCK_SIZE` | 47 | `firmware_update_user.h` | CAN-FD / UART block size |
| `FW_COMPLETE_REPEATS` | 3 | `firmware_update_user.h` | Copies of the final COMPLETE reply |
| `FW_WATCHDOG_DEFAULT_MS` | 5000 | `firmware_update_user.h` | Idle time before booting the app |
| `FW_WATCHDOG_PENDING_MS` | 60000 | `firmware_update_user.h` | Idle time after an "update incoming" announce |
| `FW_ANNOUNCE_REPEATS` | 4 | `firmware_update_user.h` | Copies of the announce sent to a peripheral |
| `CAN_TX_BATCH_SIZE` | 2 | `firmware_update_user.h` | CAN frames per TX batch |
| `BOOT_ATTEMPT_THRESHOLD` | 3 | `qspi_user.h` | Trial boots before rollback |
| `METADATA_MAGIC` | `0xA55AB00B` | `qspi_user.h` | Valid-record marker |
| `UART_FW_TX_BATCH_DELAY_MS` | 10 | `uart_fw_transport.h` | Pacing between UART frames |
| IP / port | `192.168.25.168:50001` | `enet_user.c` | VCC network address |

Changing a block size or the wire structs requires the same change in the host tool.

---

## Implementation status

This snapshot implements most of the design in the paper, but not all of it. Keep this table current.

| Feature | Paper | This code |
|---|---|---|
| Coverage bitmap + POLL/STATUS repair on **UDP** (host → VCC) | ✅ | ✅ Implemented |
| Poll-gated finalisation (COMPLETE sent before commit), 3× repeat | ✅ | ✅ on the VCC |
| Poll/repair on **CAN-FD** (VCC → peripheral) | ✅ | ⚠️ One-shot push. The peripheral verifies and commits, but there is no POLL/STATUS repair loop yet. |
| Poll/repair on **UART** | ✅ | ⚠️ Framing layer and RX path exist, one-shot push only |
| Automatic **UART fallback** when CAN-FD fails | ✅ | ❌ The call in `Check_And_Write_Firmware()` is commented out |
| "Update incoming" announce, 5 s / 60 s windows | ✅ | ✅ |
| Dual-bank commit, backup-first metadata | ✅ | ✅ |
| Boot-attempt counter + rollback, `Bootloader_ConfirmBoot()` | ✅ | ✅ Single `confirmed` / `attempt_count` per record (the paper's algorithm tracks confirmation per bank) |
| Whole-image SHA-256 check | ✅ | ✅ on the DTHE accelerator |
| **Digital signature**: host signs SHA-256, public key in OTP | ✅ | ❌ Not implemented. META carries the plain SHA-256 digest (32 B), not a signature. |
| Forwarding from any VCC controller | — | ⚠️ Only `VCC_1_CONTROLLER_1_ID` forwards peripheral images |

Next planned work: CAN-FD and UART repair loops (chunked STATUS replies), UART fallback, and signature verification (the META format will grow to carry the full signature).

---

## Troubleshooting

All logs go to the debug UART (`UART0`) and are prefixed by module.

| Symptom | Likely cause / fix |
|---|---|
| Host keeps getting `NEED_META` | META was lost, or the host's POLL uses a different `image_crc`. Resend META. |
| `DATA: unaligned offset` in the log | Host block size ≠ `FW_BLOCK_SIZE`. Match them. |
| `DATA: would overflow buffer` | Image is larger than `MAX_IMAGE_SIZE` (800 KB). |
| Card boots its old image before the transfer starts | Host started after the 5 s window. Reset the unit and start the host immediately. |
| `IMAGE_FAIL` on every attempt | The CRC-32 or SHA-256 in META doesn't match the image file. Rebuild META from the exact file being sent. |
| `[QSPI] Flashing Same image again` | The image CRC equals the running image. Nothing is written. This is expected. |
| New image boots, then the old one returns after a few resets | The application never called `Bootloader_ConfirmBoot()`, so rollback after 3 attempts is working as designed. |
| `[META] Both slots corrupt - defaulting to Bank A` | Normal on the first boot after factory programming. Otherwise, investigate flash integrity. |
