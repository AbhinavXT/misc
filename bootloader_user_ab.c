/*
 * bootloader_user.c
 *
 * Dual-bank boot switch logic.
 *
 * Boot sequence:
 *   1. Read metadata (primary → backup → hardcoded Bank A default).
 *   2. Validate the active bank's image CRC from flash.
 *   3. If CRC fails, try the other bank as a fallback.
 *   4. Boot from whichever bank is valid.
 *
 * This means even if the active bank's image is somehow corrupt on flash
 * (e.g. flash bit rot), the bootloader will attempt the other bank rather
 * than hanging indefinitely.
 */

#include "common_include.h"
#include "qspi_user.h"
#include "bootloader_user.h"

extern Bootloader_FlashArgs gBootloader0Args;

/* -----------------------------------------------------------------------
 * validate_bank
 *
 * Reads the image at 'flash_offset' of 'size' bytes from flash and
 * computes its CRC32, comparing against 'expected_crc'.
 *
 * Returns true if the bank's CRC matches (image is intact on flash),
 * false if the CRC mismatches or size is 0 (no image present).
 * ----------------------------------------------------------------------- */
static bool validate_bank(uint32_t flash_offset, uint32_t size, uint32_t expected_crc)
{
    if (size == 0) {
        PRINTF("[BOOT] Bank at 0x%08X: no image (size=0)\r\n", flash_offset);
        return false;
    }

    PRINTF("[BOOT] Validating bank at 0x%08X (%u bytes)...\r\n",
           flash_offset, size);

    uint32_t actual_crc = Calculate_Appimage_CRC(flash_offset, size);

    if (actual_crc != expected_crc) {
        PRINTF("[BOOT] CRC MISMATCH: expected=0x%08X got=0x%08X\r\n",
               expected_crc, actual_crc);
        return false;
    }

    PRINTF("[BOOT] CRC OK (0x%08X)\r\n", actual_crc);
    return true;
}

/* -----------------------------------------------------------------------
 * boot_from_offset
 *
 * Sets the bootloader's appImageOffset to 'offset' and runs the standard
 * TI multi-core boot sequence.  Does not return on success.
 * ----------------------------------------------------------------------- */
static void boot_from_offset(uint32_t offset)
{
    PRINTF("[BOOT] Booting from offset 0x%08X\r\n", offset);

    gBootloader0Args.appImageOffset = offset;

    int32_t status = SystemP_SUCCESS;

    Bootloader_BootImageInfo bootImageInfo;
    Bootloader_Params        bootParams;
    Bootloader_Handle        bootHandle;

    Bootloader_Params_init(&bootParams);
    Bootloader_BootImageInfo_init(&bootImageInfo);

    bootHandle = Bootloader_open(CONFIG_BOOTLOADER0, &bootParams);
    if (bootHandle == NULL) {
        PRINTF("[BOOT] ERROR: Bootloader_open failed\r\n");
        return;
    }

    status = Bootloader_parseMultiCoreAppImage(bootHandle, &bootImageInfo);
    PRINTF("[BOOT] parseMultiCoreAppImage: %d\r\n", status);

    if ((status == SystemP_SUCCESS) &&
        (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_1)))
    {
        bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1].clkHz =
            Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS1_1);
        Bootloader_profileAddCore(CSL_CORE_ID_R5FSS1_1);
        status = Bootloader_loadCpu(bootHandle,
                     &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1]);
    }
    if ((status == SystemP_SUCCESS) &&
        (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_0)))
    {
        bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0].clkHz =
            Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS1_0);
        Bootloader_profileAddCore(CSL_CORE_ID_R5FSS1_0);
        status = Bootloader_loadCpu(bootHandle,
                     &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0]);
    }
    if ((status == SystemP_SUCCESS) &&
        (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_1)))
    {
        bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1].clkHz =
            Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_1);
        Bootloader_profileAddCore(CSL_CORE_ID_R5FSS0_1);
        status = Bootloader_loadCpu(bootHandle,
                     &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1]);
    }
    if ((status == SystemP_SUCCESS) &&
        (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_0)))
    {
        bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0].clkHz =
            Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_0);
        Bootloader_profileAddCore(CSL_CORE_ID_R5FSS0_0);
        status = Bootloader_loadSelfCpu(bootHandle,
                     &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0], TRUE);
    }

    QSPI_Handle qspiHandle = QSPI_getHandle(CONFIG_QSPI0);

    if ((status == SystemP_SUCCESS) &&
        (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_1)))
        status = Bootloader_runCpu(bootHandle,
                     &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1]);

    if ((status == SystemP_SUCCESS) &&
        (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_0)))
        status = Bootloader_runCpu(bootHandle,
                     &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0]);

    if ((status == SystemP_SUCCESS) &&
        (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_1)))
        status = Bootloader_runCpu(bootHandle,
                     &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1]);

    if ((status == SystemP_SUCCESS) &&
        (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_0)))
    {
        if (bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0].rprcOffset
                != BOOTLOADER_INVALID_ID)
        {
            status = Bootloader_rprcImageLoad(bootHandle,
                         &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0]);
        }

        Bootloader_profileAddProfilePoint("CPU load");

        if (status == SystemP_SUCCESS)
        {
            Bootloader_profileUpdateAppimageSize(
                Bootloader_getMulticoreImageSize(bootHandle));
            Bootloader_profileUpdateMediaAndClk(
                BOOTLOADER_MEDIA_FLASH,
                QSPI_getInputClk(qspiHandle));
            Bootloader_profileAddProfilePoint("SBL End");
            Bootloader_profilePrintProfileLog();
            DebugP_log("Switching to application...\r\n");
        }

        /* Release self core — this call does not return on success */
        status = Bootloader_runSelfCpu(bootHandle, &bootImageInfo);
    }

    /* Reaching here means boot failed */
    Bootloader_close(bootHandle);
    PRINTF("[BOOT] ERROR: boot sequence failed (status=%d)\r\n", status);
}

/* -----------------------------------------------------------------------
 * Boot_Switch  (public)
 *
 * Main boot decision logic:
 *   1. Read metadata to find the intended active bank.
 *   2. Validate that bank's image CRC from flash.
 *   3. If it fails, try the other bank as a CRC-validated fallback.
 *   4. Boot from whichever bank passes validation.
 *   5. If both fail (extreme case), log and hang — nothing safe to boot.
 * ----------------------------------------------------------------------- */
void Boot_Switch(void)
{
    BootMetadata meta;
    int32_t meta_status = QSPI_Read_Metadata(&meta);

    if (meta_status != SystemP_SUCCESS) {
        PRINTF("[BOOT] Metadata corrupt — using Bank A default.\r\n");
    }

    /* Determine the primary and fallback boot targets from metadata */
    uint32_t primary_offset  = (meta.active_bank == BANK_A)
                               ? meta.bank_a_offset : meta.bank_b_offset;
    uint32_t primary_size    = (meta.active_bank == BANK_A)
                               ? meta.bank_a_size   : meta.bank_b_size;
    uint32_t primary_crc     = (meta.active_bank == BANK_A)
                               ? meta.bank_a_crc    : meta.bank_b_crc;

    uint32_t fallback_offset = (meta.active_bank == BANK_A)
                               ? meta.bank_b_offset : meta.bank_a_offset;
    uint32_t fallback_size   = (meta.active_bank == BANK_A)
                               ? meta.bank_b_size   : meta.bank_a_size;
    uint32_t fallback_crc    = (meta.active_bank == BANK_A)
                               ? meta.bank_b_crc    : meta.bank_a_crc;

    PRINTF("[BOOT] Primary  : Bank %s  offset=0x%08X  size=%u\r\n",
           meta.active_bank == BANK_A ? "A" : "B",
           primary_offset, primary_size);
    PRINTF("[BOOT] Fallback : Bank %s  offset=0x%08X  size=%u\r\n",
           meta.active_bank == BANK_A ? "B" : "A",
           fallback_offset, fallback_size);

    /* Try the primary (intended active) bank first */
    if (validate_bank(primary_offset, primary_size, primary_crc)) {
        boot_from_offset(primary_offset);
        /* boot_from_offset does not return on success */
    }

    /* Primary bank invalid — try the fallback bank */
    PRINTF("[BOOT] Primary bank invalid — trying fallback bank.\r\n");

    if (validate_bank(fallback_offset, fallback_size, fallback_crc)) {
        PRINTF("[BOOT] Booting from fallback bank.\r\n");
        boot_from_offset(fallback_offset);
        /* boot_from_offset does not return on success */
    }

    /* Both banks are invalid — nothing safe to boot */
    PRINTF("[BOOT] FATAL: both banks invalid. System halted.\r\n");
    PRINTF("[BOOT] Please reflash the device via JTAG.\r\n");

    /* Hang here — no infinite reset loop which could wear flash */
    while (1) { ClockP_usleep(1000000); }
}
