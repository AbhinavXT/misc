# Dual-Bank Firmware Update System

## TI AM2634 — Design Document

**Project:** Multi-protocol OTA Bootloader
**Target:** TI AM2634 (AM263x family), bare-metal, SIL-4 safety context
**Protocols:** Ethernet/UDP (primary), CAN, UART (planned)

---

## 1. The Core Problem This Solves

A firmware update is inherently dangerous. You are overwriting the software that runs the system while the system is running. If anything goes wrong mid-update — a power cut, a corrupted packet, a flash write error — you could be left with a partially written image that cannot boot. In a safety-critical embedded system, a bricked device is not just an inconvenience. It is a safety hazard.

The dual-bank system solves this by making the update process atomic from the perspective of the bootloader. The bootloader only ever boots from one complete, verified image. It never sees a partially-written image. The moment it switches its view from the old image to the new one is a single metadata write — and even that write is protected against power cuts.

---

## 2. Flash Layout

The 8 MB flash chip is partitioned as follows:

```
0x000000  ┌────────────────────────────┐
          │  SBL / Updater Code        │  2 MB reserved
0x200000  ├────────────────────────────┤
          │  Bank A  (1 MB)            │  ← original image, always the safe fallback
0x300000  ├────────────────────────────┤
          │  Bank B  (1 MB)            │  ← new inactive bank
0x400000  ├────────────────────────────┤
          │  (unused / reserved)       │  2 MB
0x600000  ├────────────────────────────┤
          │  Metadata Primary          │  1 erase block (~64 KB)
0x610000  ├────────────────────────────┤
          │  Metadata Backup           │  1 erase block (~64 KB)
0x620000  ├────────────────────────────┤
          │  (remaining flash)         │
0x800000  └────────────────────────────┘
```

Bank A lives at `0x200000`, which is the same address the original single-bank system used. This means the first factory-programmed image works without any migration step — Bank A is simply where it always was.

---

## 3. The Metadata Struct

The metadata is the single source of truth for the bootloader. It tells the bootloader which bank is the authoritative running image, and stores enough information about both banks to validate either one from flash before booting.

```c
typedef struct __attribute__((packed)) {
    uint32_t magic;          // 0xA55AB00B — sanity sentinel
    uint8_t  active_bank;    // BANK_A (0x00) or BANK_B (0x01)

    uint32_t bank_a_offset;  // always 0x200000
    uint32_t bank_a_size;    // bytes; 0 means no image present
    uint32_t bank_a_crc;     // CRC32 of the Bank A image

    uint32_t bank_b_offset;  // always 0x300000
    uint32_t bank_b_size;    // bytes; 0 means no image present
    uint32_t bank_b_crc;     // CRC32 of the Bank B image

    uint32_t crc_self;       // CRC32 of all fields above this one
} BootMetadata;
```

### Why `magic`?

Erased NOR flash reads as `0xFFFFFFFF`. A freshly erased metadata block will never accidentally match `0xA55AB00B`. This gives the bootloader a fast, cheap way to detect an uninitialised or completely corrupted metadata block before even computing a CRC.

### Why `crc_self`?

A power cut mid-write can leave a metadata block where some fields were written and others were not. For example, `active_bank` might have been written to `BANK_B` but `bank_b_size` was not yet written, leaving it as `0xFFFFFFFF`. Without `crc_self`, the bootloader would try to boot Bank B with a nonsensical 4 GB image size. With `crc_self`, the CRC check catches the partial write and the bootloader discards the slot entirely.

### Why store both banks?

Storing CRC and size for both banks means the bootloader can independently validate either bank from flash. If the active bank fails its CRC check on boot (e.g. due to flash bit-rot), the bootloader can attempt the other bank and potentially recover — without the system ever requiring JTAG intervention.

---

## 4. The Two Metadata Slots

The metadata is stored in two separate erase blocks: a primary slot at `0x600000` and a backup slot at `0x610000`. They always contain identical data, but they are written in a specific order that eliminates the power-cut vulnerability in the metadata write itself.

The write sequence is always: **backup first, primary second.**

To understand why this order matters, consider what happens if power is cut during the metadata write:

- **Power cut during backup write:** The primary slot still contains the old, valid metadata. The system boots from the old active bank. The backup slot is corrupt but that is fine — it will be overwritten on the next successful update.
- **Power cut during primary write:** The backup slot already contains the correct new metadata. The bootloader reads the primary, finds it corrupt, falls back to the backup, finds it valid, and boots from the new bank correctly. It also opportunistically restores the primary slot from the backup so the next boot is clean.
- **Both slots corrupt (e.g. first boot, or flash wiped):** The bootloader defaults to Bank A at `0x200000`, which is the hardcoded factory fallback.

This design means there is no point in the entire update process — from the first erased byte to the last written metadata byte — where a power cut can leave the system unbootable.

---

## 5. The Update Write Sequence

When a new image arrives over UDP and passes RAM-level verification (CRC32 + SHA-256), the following sequence runs in `QSPI_Write_Full_Image()`. Each phase is a separate function that returns `SystemP_FAILURE` on error, allowing clean early-return without `goto` statements. The active bank is never touched at any point.

**Phase 1 — Determine the write target.** Read the current metadata to find the active bank. The inactive bank is the write target. If metadata is corrupt, Bank A is treated as active (so we write to Bank B).

**Phase 2 — Erase the inactive bank.** Erase all flash blocks that cover the new image's extent. If this fails, return immediately. The active bank is untouched and the metadata still points to it.

**Phase 3 — Write the new image.** Write the RAM buffer to the inactive bank one page (256 bytes) at a time. If any page write fails, return immediately. The active bank is still untouched.

**Phase 4 — Read-back verify.** Read the newly written region back from flash and compare it byte-for-byte against the RAM buffer. This catches flash write errors that the write operation itself does not report. If any mismatch is found, return immediately. The active bank is still untouched.

**Phase 5 — Build the new metadata.** Copy the current metadata struct and update only the fields for the newly written bank (offset, size, CRC). Set `active_bank` to the newly written bank. The other bank's fields are preserved unchanged — this is critical because they represent a valid fallback image.

**Phase 6 — Write the new metadata.** Write to the backup slot first, then the primary slot. After this step completes successfully, the bootloader will boot from the new bank on the next reset.

The function returns `SystemP_FAILURE` at any phase from 2–6 without touching the active bank's contents. Only Phase 6 changes what the bootloader will do — and even that is protected by the two-slot design.

---

## 6. The Boot Sequence

The bootloader's `Boot_Switch()` function follows this decision tree on every reset:

```
Read primary metadata
  └─ Valid? → use it
  └─ Corrupt? → read backup metadata
      └─ Valid? → use it, restore primary
      └─ Corrupt? → default to Bank A

Validate active bank CRC from flash
  └─ Pass? → boot from active bank
  └─ Fail? → validate fallback bank CRC from flash
      └─ Pass? → boot from fallback bank
      └─ Fail? → HALT (log error, do not reset-loop)
```

The CRC validation on boot is important and is often omitted in simpler systems. It protects against flash bit-rot: a bank that was correctly written months ago might have developed single-bit errors. By re-validating the CRC on every boot, the system detects this before attempting to execute potentially corrupt code.

The halt on double failure is deliberate. An infinite reset loop would wear the flash (NOR flash has a finite erase/write cycle count) and could mask the underlying problem. Halting and logging is the safer behaviour — it requires a deliberate recovery action (JTAG reflash) rather than silently cycling.

---

## 7. Failure Safety Matrix

The following table covers every possible failure point and its outcome:

| Failure Point | Active Bank | Metadata State | Outcome on Next Boot |
|---|---|---|---|
| Power cut during Bank B erase | Untouched | Unchanged (→ Bank A) | Boots from Bank A normally |
| Power cut during Bank B write | Untouched | Unchanged (→ Bank A) | Boots from Bank A normally |
| Power cut during Bank B verify | Untouched | Unchanged (→ Bank A) | Boots from Bank A normally |
| Power cut during backup meta write | Untouched | Primary still → Bank A | Boots from Bank A, backup rebuilt on next update |
| Power cut during primary meta write | New image in Bank B | Backup → Bank B | Reads backup, boots from Bank B correctly |
| Both meta slots corrupt | Either | Neither valid | Defaults to Bank A (factory fallback) |
| Active bank CRC fails on boot | Old image | Unchanged | Tries fallback bank, boots if valid |
| Both banks CRC fail on boot | — | — | Halts, requires JTAG recovery |

---

## 8. First Boot Behaviour

On the very first boot after factory programming, no metadata has ever been written to flash. Both metadata slots will read as `0xFFFFFFFF` (erased), causing both the magic check and the CRC check to fail. The bootloader defaults to Bank A at `0x200000` with `bank_a_size = 0`. The bootloader will attempt to boot from that offset — it is assumed that the factory image was programmed there by JTAG.

After the first successful OTA update, the metadata is written for the first time and all subsequent boots use the normal read-priority logic.

---

## 9. Integration Points

The following table shows which source file owns each responsibility:

| Responsibility | File |
|---|---|
| Flash erase, write, verify | `qspi_user.c` |
| Metadata read/write/validate | `qspi_user.c` |
| Dual-bank write decision | `qspi_user.c` → `QSPI_Write_Full_Image()` |
| Boot decision and CPU load | `bootloader_user.c` → `Boot_Switch()` |
| UDP receive and RAM assembly | `firmware_update_user.c` |
| SHA-256 hardware acceleration | `sha256_user.c` (DTHE) |
| Per-chunk CRC32 and image CRC32 | `crc.h` / `udp_file.cpp` |
| Protocol definitions (packet types) | `fw_protocol.h` |
| Ethernet init and UDP send/receive | `enet_init.c` |

---

## 10. Key Design Principles

**Never touch the active bank.** This is the single most important rule. Every flash operation during an update targets the inactive bank only. The active bank remains readable and bootable throughout.

**Metadata is the last thing written.** The image is fully written and verified before the metadata is updated. This makes the metadata write the only point of commitment — before it, the system is in the old state; after it, the system is in the new state. There is no in-between state that is unbootable.

**Write the backup slot before the primary.** This eliminates the only remaining failure window in the metadata commit itself.

**Validate on every boot, not just after update.** Re-checking the active bank's CRC on every boot catches flash degradation that happened between updates, not just corruption that happened during the update.

**No goto statements.** Error handling uses helper functions that return `SystemP_SUCCESS` or `SystemP_FAILURE`, with the caller performing a clean early return. This keeps each function's control flow linear and auditable — important in a SIL-4 safety context.

**Preserve the other bank's metadata.** When writing new metadata, only the fields for the newly written bank are updated. The other bank's offset, size, and CRC remain intact. This means the other bank is always a known, potentially valid fallback that the bootloader can attempt independently.
