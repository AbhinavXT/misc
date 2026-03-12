/*
 * qspi_user.c
 *
 * Dual-bank firmware flash management for the TI AM2634.
 *
 * Core guarantee:
 *   The active bank is NEVER touched during a firmware update.  A new image
 *   is always written to the inactive bank.  Metadata is updated only after
 *   the inactive bank has been fully written and read-back verified.  The
 *   backup metadata slot is always written before the primary, so a power
 *   cut at any point leaves at least one valid metadata copy recoverable.
 *
 * Failure safety matrix:
 *
 *   Power cut during bank erase       → active bank untouched, meta unchanged.
 *   Power cut during bank write       → active bank untouched, meta unchanged.
 *   Power cut during bank verify      → active bank untouched, meta unchanged.
 *   Power cut during backup meta wr   → primary meta still points to old bank.
 *   Power cut during primary meta wr  → backup meta has new state; recoverable.
 *   Both meta slots corrupt           → hardcoded fallback to Bank A.
 *
 * No goto statements are used anywhere in this file.  Early-exit logic is
 * expressed through helper functions returning int32_t (SystemP_SUCCESS /
 * SystemP_FAILURE), which the caller checks with a simple if-return pattern.
 */

#include <string.h>
#include <stdbool.h>
#include "common_include.h"
#include "qspi_user.h"
#include "ti_drivers_config.h"
#include "ti_board_open_close.h"

#define QSPI_PAGE_SIZE  256U

/* -----------------------------------------------------------------------
 * compute_metadata_crc
 *
 * Hashes every field in BootMetadata EXCEPT the trailing crc_self field.
 * We achieve this by hashing only the first N-4 bytes of the struct,
 * since crc_self sits at the very end and sizeof(uint32_t) == 4.
 * ----------------------------------------------------------------------- */
static uint32_t compute_metadata_crc(const BootMetadata* meta)
{
    uint32_t len = (uint32_t)(sizeof(BootMetadata) - sizeof(uint32_t));
    return crcFast((const unsigned char*)meta, (int)len);
}

/* -----------------------------------------------------------------------
 * erase_flash_region
 *
 * Erases all flash blocks that cover [flash_offset, flash_offset + size).
 * Returns SystemP_SUCCESS if all blocks erased cleanly, SystemP_FAILURE
 * on the first block that fails.
 *
 * Extracted into its own function so QSPI_Write_Full_Image can call it
 * without the need for goto-based cleanup.
 * ----------------------------------------------------------------------- */
static int32_t erase_flash_region(uint32_t flash_offset, uint32_t size,
                                   uint32_t blockSize)
{
    uint32_t firstBlock = flash_offset / blockSize;
    uint32_t lastBlock  = (flash_offset + size + blockSize - 1) / blockSize;

    PRINTF("[QSPI] Erasing blocks %u to %u...\r\n", firstBlock, lastBlock - 1);

    for (uint32_t b = firstBlock; b < lastBlock; b++) {
        int32_t status = Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], b);
        if (status != SystemP_SUCCESS) {
            PRINTF("[QSPI] ERROR: erase failed at block %u\r\n", b);
            return SystemP_FAILURE;
        }
    }

    PRINTF("[QSPI] Erase complete.\r\n");
    return SystemP_SUCCESS;
}

/* -----------------------------------------------------------------------
 * write_flash_region
 *
 * Writes [buffer, buffer+imageSize) to flash starting at flash_offset,
 * one page at a time.  Returns SystemP_SUCCESS or SystemP_FAILURE.
 * ----------------------------------------------------------------------- */
static int32_t write_flash_region(uint32_t flash_offset,
                                   const uint8_t* buffer,
                                   uint32_t imageSize,
                                   uint32_t pageSize)
{
    uint32_t written = 0;

    PRINTF("[QSPI] Writing %u bytes at 0x%08X...\r\n", imageSize, flash_offset);

    while (written < imageSize) {
        uint32_t chunkLen = imageSize - written;
        if (chunkLen > pageSize) chunkLen = pageSize;

        int32_t status = Flash_write(gFlashHandle[CONFIG_FLASH0],
                                     flash_offset + written,
                                     (uint8_t*)(buffer + written),
                                     chunkLen);
        if (status != SystemP_SUCCESS) {
            PRINTF("[QSPI] ERROR: write failed at 0x%08X\r\n",
                   flash_offset + written);
            return SystemP_FAILURE;
        }
        written += chunkLen;
    }

    PRINTF("[QSPI] Write complete: %u bytes.\r\n", written);
    return SystemP_SUCCESS;
}

/* -----------------------------------------------------------------------
 * verify_flash_region
 *
 * Reads back [flash_offset, flash_offset+imageSize) from flash and
 * compares it byte-for-byte against the original RAM buffer.
 * Returns SystemP_SUCCESS if identical, SystemP_FAILURE on any mismatch.
 * ----------------------------------------------------------------------- */
static int32_t verify_flash_region(uint32_t flash_offset,
                                    const uint8_t* buffer,
                                    uint32_t imageSize)
{
    uint8_t  verifyBuf[QSPI_PAGE_SIZE];
    uint32_t verified = 0;

    PRINTF("[QSPI] Verifying...\r\n");

    while (verified < imageSize) {
        uint32_t chunkLen = imageSize - verified;
        if (chunkLen > QSPI_PAGE_SIZE) chunkLen = QSPI_PAGE_SIZE;

        int32_t status = Flash_read(gFlashHandle[CONFIG_FLASH0],
                                    flash_offset + verified,
                                    verifyBuf,
                                    chunkLen);
        if (status != SystemP_SUCCESS) {
            PRINTF("[QSPI] ERROR: read-back failed at 0x%08X\r\n",
                   flash_offset + verified);
            return SystemP_FAILURE;
        }

        if (memcmp(buffer + verified, verifyBuf, chunkLen) != 0) {
            PRINTF("[QSPI] ERROR: verify mismatch at 0x%08X\r\n",
                   flash_offset + verified);
            return SystemP_FAILURE;
        }

        verified += chunkLen;
    }

    PRINTF("[QSPI] Verification PASSED.\r\n");
    return SystemP_SUCCESS;
}

/* -----------------------------------------------------------------------
 * write_one_metadata_slot
 *
 * Erases the single flash block containing flash_offset and writes the
 * metadata struct to it.  Used by QSPI_Write_Metadata to write both slots.
 * Returns SystemP_SUCCESS or SystemP_FAILURE.
 * ----------------------------------------------------------------------- */
static int32_t write_one_metadata_slot(uint32_t flash_offset,
                                        const BootMetadata* meta)
{
    uint32_t blk = 0, page = 0;

    int32_t status = Flash_offsetToBlkPage(gFlashHandle[CONFIG_FLASH0],
                                           flash_offset, &blk, &page);
    if (status != SystemP_SUCCESS) {
        PRINTF("[META] Flash_offsetToBlkPage failed for 0x%08X\r\n", flash_offset);
        return SystemP_FAILURE;
    }

    status = Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], blk);
    if (status != SystemP_SUCCESS) {
        PRINTF("[META] Erase failed at block %u (offset 0x%08X)\r\n",
               blk, flash_offset);
        return SystemP_FAILURE;
    }

    status = Flash_write(gFlashHandle[CONFIG_FLASH0],
                         flash_offset,
                         (uint8_t*)meta,
                         sizeof(BootMetadata));
    if (status != SystemP_SUCCESS) {
        PRINTF("[META] Write failed at 0x%08X\r\n", flash_offset);
        return SystemP_FAILURE;
    }

    return SystemP_SUCCESS;
}

/* -----------------------------------------------------------------------
 * read_and_validate_slot
 *
 * Reads one metadata slot from flash into 'out' and validates it.
 * Returns true if magic and crc_self both pass, false otherwise.
 * ----------------------------------------------------------------------- */
static bool read_and_validate_slot(uint32_t flash_offset, BootMetadata* out)
{
    int32_t status = Flash_read(gFlashHandle[CONFIG_FLASH0],
                                flash_offset,
                                (uint8_t*)out,
                                sizeof(BootMetadata));
    if (status != SystemP_SUCCESS) {
        PRINTF("[META] Flash_read failed at 0x%08X\r\n", flash_offset);
        return false;
    }

    return QSPI_Validate_Metadata(out);
}

/* -----------------------------------------------------------------------
 * QSPI_Validate_Metadata  (public)
 * ----------------------------------------------------------------------- */
bool QSPI_Validate_Metadata(const BootMetadata* meta)
{
    /* Erased flash reads as 0xFFFFFFFF — fast rejection before CRC */
    if (meta->magic != METADATA_MAGIC) {
        return false;
    }

    /* Verify structural integrity — catches partial writes and bit errors */
    if (meta->crc_self != compute_metadata_crc(meta)) {
        return false;
    }

    /* Sanity check the bank field — any value other than 0 or 1 is corrupt */
    if (meta->active_bank != BANK_A && meta->active_bank != BANK_B) {
        return false;
    }

    return true;
}

/* -----------------------------------------------------------------------
 * QSPI_Read_Metadata  (public)
 *
 * Read priority:
 *   1. Primary slot (QSPI_METADATA_OFFSET)        — used if valid.
 *   2. Backup slot  (QSPI_METADATA_BACKUP_OFFSET) — used if primary corrupt.
 *   3. Hardcoded Bank A default                   — used if both corrupt.
 * ----------------------------------------------------------------------- */
int32_t QSPI_Read_Metadata(BootMetadata* out)
{
    /* Try primary slot first */
    if (read_and_validate_slot(QSPI_METADATA_OFFSET, out)) {
        PRINTF("[META] Primary slot valid. Active bank: %s\r\n",
               out->active_bank == BANK_A ? "A" : "B");
        return SystemP_SUCCESS;
    }

    PRINTF("[META] Primary slot corrupt — trying backup.\r\n");

    /* Try backup slot */
    if (read_and_validate_slot(QSPI_METADATA_BACKUP_OFFSET, out)) {
        PRINTF("[META] Backup slot valid. Active bank: %s\r\n",
               out->active_bank == BANK_A ? "A" : "B");

        /* Opportunistically restore the primary from the backup so
         * the next boot finds the primary slot healthy again.       */
        PRINTF("[META] Restoring primary slot from backup...\r\n");
        write_one_metadata_slot(QSPI_METADATA_OFFSET, out);

        return SystemP_SUCCESS;
    }

    /* Both slots corrupt — build a safe default pointing to Bank A.
     * This handles the first boot after factory programming where no
     * metadata has ever been written.                                */
    PRINTF("[META] Both slots corrupt — defaulting to Bank A.\r\n");

    memset(out, 0, sizeof(BootMetadata));
    out->magic         = METADATA_MAGIC;
    out->active_bank   = BANK_A;
    out->bank_a_offset = QSPI_BANK_A_OFFSET;
    out->bank_a_size   = 0;
    out->bank_a_crc    = 0;
    out->bank_b_offset = QSPI_BANK_B_OFFSET;
    out->bank_b_size   = 0;
    out->bank_b_crc    = 0;
    out->crc_self      = compute_metadata_crc(out);

    return SystemP_FAILURE;
}

/* -----------------------------------------------------------------------
 * QSPI_Write_Metadata  (public)
 *
 * Always writes the backup slot FIRST, then the primary slot.
 * This ensures at least one valid slot exists even if power is cut
 * mid-way through writing the primary slot.
 * ----------------------------------------------------------------------- */
int32_t QSPI_Write_Metadata(BootMetadata* meta)
{
    /* Always recompute crc_self immediately before writing */
    meta->magic    = METADATA_MAGIC;
    meta->crc_self = compute_metadata_crc(meta);

    /* Step 1 — backup slot first.
     * If power cuts here, the primary slot still has the old valid metadata. */
    PRINTF("[META] Writing backup slot (0x%08X)...\r\n",
           QSPI_METADATA_BACKUP_OFFSET);

    if (write_one_metadata_slot(QSPI_METADATA_BACKUP_OFFSET, meta)
            != SystemP_SUCCESS) {
        PRINTF("[META] ERROR: backup slot write failed.\r\n");
        return SystemP_FAILURE;
    }

    /* Step 2 — primary slot second.
     * If power cuts here, the backup slot has the correct new metadata. */
    PRINTF("[META] Writing primary slot (0x%08X)...\r\n",
           QSPI_METADATA_OFFSET);

    if (write_one_metadata_slot(QSPI_METADATA_OFFSET, meta)
            != SystemP_SUCCESS) {
        PRINTF("[META] ERROR: primary slot write failed.\r\n");
        return SystemP_FAILURE;
    }

    PRINTF("[META] Both slots written. Active bank: %s  size=%u  CRC=0x%08X\r\n",
           meta->active_bank == BANK_A ? "A" : "B",
           meta->active_bank == BANK_A ? meta->bank_a_size : meta->bank_b_size,
           meta->active_bank == BANK_A ? meta->bank_a_crc  : meta->bank_b_crc);

    return SystemP_SUCCESS;
}

/* -----------------------------------------------------------------------
 * QSPI_Write_Full_Image  (public)
 *
 * Full dual-bank update sequence.  Each phase is a separate helper that
 * returns SystemP_FAILURE on error, allowing clean early-return without
 * goto.  The active bank is never touched; if any phase fails before the
 * metadata write, the system simply continues booting from the current bank.
 * ----------------------------------------------------------------------- */
int32_t QSPI_Write_Full_Image(const uint8_t* buffer, uint32_t imageSize,
                               uint32_t imageCrc)
{
    if (imageSize == 0 || imageSize > QSPI_BANK_SIZE) {
        PRINTF("[QSPI] ERROR: invalid image size %u (max %u)\r\n",
               imageSize, QSPI_BANK_SIZE);
        return SystemP_FAILURE;
    }

    Flash_Attrs* flashAttrs = Flash_getAttrs(CONFIG_FLASH0);
    if (flashAttrs == NULL) {
        PRINTF("[QSPI] ERROR: Flash_getAttrs returned NULL\r\n");
        return SystemP_FAILURE;
    }

    uint32_t blockSize = flashAttrs->blockSize;
    uint32_t pageSize  = flashAttrs->pageSize;

    /* ---- Phase 1: Read current metadata to learn the active bank ---- */
    BootMetadata current_meta;
    QSPI_Read_Metadata(&current_meta);

    uint8_t  inactive_bank = (current_meta.active_bank == BANK_A)
                             ? BANK_B : BANK_A;
    uint32_t write_offset  = (inactive_bank == BANK_A)
                             ? QSPI_BANK_A_OFFSET : QSPI_BANK_B_OFFSET;

    PRINTF("[QSPI] Active  : Bank %s\r\n",
           current_meta.active_bank == BANK_A ? "A" : "B");
    PRINTF("[QSPI] Writing : Bank %s at 0x%08X (%u bytes)\r\n",
           inactive_bank == BANK_A ? "A" : "B", write_offset, imageSize);

    /* ---- Phase 2: Erase the inactive bank ---- */
    if (erase_flash_region(write_offset, imageSize, blockSize)
            != SystemP_SUCCESS) {
        /* Active bank untouched — safe to return failure */
        return SystemP_FAILURE;
    }

    /* ---- Phase 3: Write the new image to the inactive bank ---- */
    if (write_flash_region(write_offset, buffer, imageSize, pageSize)
            != SystemP_SUCCESS) {
        /* Active bank untouched — safe to return failure */
        return SystemP_FAILURE;
    }

    /* ---- Phase 4: Read-back verify the inactive bank ---- */
    if (verify_flash_region(write_offset, buffer, imageSize)
            != SystemP_SUCCESS) {
        /* Active bank untouched — safe to return failure */
        return SystemP_FAILURE;
    }

    /* ---- Phase 5: Build new metadata — preserve the old bank's entry ---- */
    BootMetadata new_meta;
    memcpy(&new_meta, &current_meta, sizeof(BootMetadata));

    /* Update only the slot we just wrote; the other bank's fields are kept
     * intact so the bootloader can still fall back to it if needed.        */
    if (inactive_bank == BANK_A) {
        new_meta.bank_a_offset = QSPI_BANK_A_OFFSET;
        new_meta.bank_a_size   = imageSize;
        new_meta.bank_a_crc    = imageCrc;
    } else {
        new_meta.bank_b_offset = QSPI_BANK_B_OFFSET;
        new_meta.bank_b_size   = imageSize;
        new_meta.bank_b_crc    = imageCrc;
    }
    new_meta.active_bank = inactive_bank;

    /* ---- Phase 6: Write metadata (backup first, then primary) ---- */
    if (QSPI_Write_Metadata(&new_meta) != SystemP_SUCCESS) {
        PRINTF("[QSPI] ERROR: metadata write failed.\r\n");
        return SystemP_FAILURE;
    }

    PRINTF("[QSPI] === DUAL-BANK UPDATE COMPLETE ===\r\n");
    PRINTF("[QSPI] New active bank: %s  offset=0x%08X  size=%u\r\n",
           inactive_bank == BANK_A ? "A" : "B", write_offset, imageSize);

    return SystemP_SUCCESS;
}

/* -----------------------------------------------------------------------
 * Calculate_Appimage_CRC  (public)
 *
 * Reads an image from flash in page-sized chunks and computes CRC32
 * incrementally.  Called by the bootloader to validate a bank before boot.
 * ----------------------------------------------------------------------- */
uint32_t Calculate_Appimage_CRC(uint32_t flash_offset, uint32_t image_size)
{
    uint32_t crc_state   = crcInitState();
    uint32_t data_offset = 0;
    uint8_t  rx_buff[QSPI_PAGE_SIZE];

    while (data_offset < image_size) {
        uint32_t bytes_to_read = image_size - data_offset;
        if (bytes_to_read > QSPI_PAGE_SIZE) bytes_to_read = QSPI_PAGE_SIZE;

        memset(rx_buff, 0, QSPI_PAGE_SIZE);

        int32_t status = Flash_read(gFlashHandle[CONFIG_FLASH0],
                                    flash_offset + data_offset,
                                    rx_buff,
                                    bytes_to_read);
        if (status != SystemP_SUCCESS) {
            PRINTF("[QSPI] CRC read failed at 0x%08X\r\n",
                   flash_offset + data_offset);
            return 0;
        }

        crc_state    = crcUpdate(crc_state, rx_buff, (int)bytes_to_read);
        data_offset += bytes_to_read;
    }

    return crcFinalize(crc_state);
}
