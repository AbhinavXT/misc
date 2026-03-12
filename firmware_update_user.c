/*
 * firmware_update_user.c
 *
 * Receives a firmware appimage — either over UDP (on the VCC card) or over
 * CAN-FD (on Input/Output/Analog cards) — verifies it in RAM, then writes
 * it to QSPI flash via the dual-bank mechanism.
 *
 * When the VCC card receives an image destined for another card type over
 * UDP, it re-transmits the entire verified image over CAN using the same
 * MetaHeader + ChunkHeader protocol so that the receiving card's
 * Process_Firmware_Update_Messages() can handle the CAN data path with
 * zero changes — it just sees the same byte stream via a different transport.
 *
 * Protocol (two packet types, distinguished by first byte):
 *
 *   PKT_TYPE_META (0x00) — sent once before any data:
 *     Byte  0      : type        = 0x00
 *     Byte  1      : card_type
 *     Bytes 2-5    : totalSize   (uint32_t)
 *     Bytes 6-9    : image_crc   (uint32_t) CRC32 of full image
 *     Bytes 10-41  : image_sha256 (uint8_t[32])
 *
 *   PKT_TYPE_DATA (0x01) — one per chunk:
 *     Byte  0      : type        = 0x01
 *     Bytes 1-4    : offset      (uint32_t) byte position in image
 *     Bytes 5-8    : size        (uint32_t) payload bytes
 *     Bytes 9-12   : chunk_crc   (uint32_t) CRC32 of this chunk's payload
 *     Bytes 13+    : raw image data
 *
 * Over CAN-FD, each firmware DATA packet is exactly CAN_FW_CHUNK_SIZE (48)
 * bytes of image data, making the total packet 13 + 48 = 61 bytes —
 * exactly one DATA_FRAME_PAYLOAD_SIZE.  One CAN frame == one firmware chunk.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <security_common/drivers/crypto/dthe/dthe_sha.h>

#include "common_include.h"
#include "firmware_update_user.h"
#include "sha256_user.h"
#include "qspi_user.h"
#include "mcan_user.h"


/* Aligned for DTHE SHA hardware DMA requirements */
static uint8_t  gImageBuffer[MAX_IMAGE_SIZE] __attribute__((aligned(128)));

static uint32_t gExpectedSize  = 0;
static uint32_t gBytesReceived = 0;
static bool     gTransferDone  = false;
static bool     gMetaReceived  = false;
static uint8_t  gCardType      = 0;

static uint32_t gExpectedCRC   = 0;
static uint8_t  gExpectedSHA256[32];

extern uint32_t firmware_pkt_recvd_time;
extern uint8_t  self_controller_id;

/* -----------------------------------------------------------------------
 * reset_transfer_state  (internal helper)
 *
 * Clears the image buffer and all tracking variables after a failed or
 * completed transfer.  Keeping this in one place avoids the duplication
 * that was previously scattered across the three failure paths in
 * handle_data_packet.
 * ----------------------------------------------------------------------- */
static void reset_transfer_state(void)
{
    memset(gImageBuffer, 0xFF, MAX_IMAGE_SIZE);
    gBytesReceived = 0;
    gMetaReceived  = false;
}

/* -----------------------------------------------------------------------
 * handle_meta_packet
 * ----------------------------------------------------------------------- */
static void handle_meta_packet(const uint8_t* buffer, uint32_t len)
{
    if (len < sizeof(MetaHeader)) {
        PRINTF("[FW_UPDATE] META: packet too short (%u bytes)\r\n", len);
        return;
    }

    MetaHeader meta;
    memcpy(&meta, buffer, sizeof(meta));

    gCardType      = meta.card_type;
    gExpectedSize  = meta.totalSize;
    gBytesReceived = 0;
    gTransferDone  = false;
    gExpectedCRC   = meta.image_crc;
    memcpy(gExpectedSHA256, meta.image_sha256, 32U);
    memset(gImageBuffer, 0xFF, MAX_IMAGE_SIZE);
    gMetaReceived  = true;

    PRINTF("[FW_UPDATE] META: card=%u  size=%u  CRC=0x%08X\r\n",
           meta.card_type, gExpectedSize, gExpectedCRC);

    firmware_pkt_recvd_time = ClockP_getTicks();
}

/* -----------------------------------------------------------------------
 * handle_data_packet
 * ----------------------------------------------------------------------- */
static void handle_data_packet(const uint8_t* buffer, uint32_t len)
{
    if (!gMetaReceived) {
        PRINTF("[FW_UPDATE] DATA: no META yet, ignoring\r\n");
        return;
    }

    if (len < sizeof(ChunkHeader)) {
        PRINTF("[FW_UPDATE] DATA: runt packet (%u bytes)\r\n", len);
        return;
    }

    ChunkHeader hdr;
    memcpy(&hdr, buffer, sizeof(hdr));

    uint32_t dataLen = len - (uint32_t)sizeof(ChunkHeader);
    if (dataLen > hdr.size) {
        dataLen = hdr.size;
    }

    if ((hdr.offset + dataLen) > MAX_IMAGE_SIZE) {
        PRINTF("[FW_UPDATE] DATA: overflow (offset=%u len=%u)\r\n",
               hdr.offset, dataLen);
        return;
    }

    /* Per-chunk CRC32 — drop only this chunk on mismatch */
    uint32_t calc_crc = crcFast(buffer + sizeof(ChunkHeader), hdr.size);
    if (hdr.chunk_crc != calc_crc) {
        PRINTF("[FW_UPDATE] DATA: CRC mismatch at offset %u "
               "(exp=0x%08X got=0x%08X) — dropping chunk\r\n",
               hdr.offset, hdr.chunk_crc, calc_crc);
        return;
    }

    memcpy(&gImageBuffer[hdr.offset], buffer + sizeof(ChunkHeader), dataLen);
    gBytesReceived += dataLen;
    firmware_pkt_recvd_time = ClockP_getTicks();

    if (gBytesReceived < gExpectedSize) {
        return;
    }

    /* ---- All chunks in RAM — run final integrity checks ---- */
    PRINTF("[FW_UPDATE] All %u bytes received. Verifying...\r\n", gBytesReceived);

    /* Check 1: whole-image CRC32 */
    uint32_t calc_image_crc = crcFast(gImageBuffer, (int)gExpectedSize);
    if (calc_image_crc != gExpectedCRC) {
        PRINTF("[FW_UPDATE] CRC32 FAIL (exp=0x%08X got=0x%08X)\r\n",
               gExpectedCRC, calc_image_crc);
        reset_transfer_state();
        return;
    }
    PRINTF("[FW_UPDATE] CRC32 OK (0x%08X)\r\n", calc_image_crc);

    /* Check 2: whole-image SHA-256 via DTHE hardware */
    uint8_t calc_sha256[32] = {0};
    uint32_t sha_status = Calculate_SHA256(gImageBuffer, gExpectedSize, calc_sha256);
    if (sha_status != DTHE_SHA_RETURN_SUCCESS) {
        PRINTF("[FW_UPDATE] SHA-256 compute failed (status=%u)\r\n", sha_status);
        reset_transfer_state();
        return;
    }

    if (memcmp(calc_sha256, gExpectedSHA256, 32U) != 0) {
        PRINTF("[FW_UPDATE] SHA-256 FAIL\r\n");
        reset_transfer_state();
        return;
    }

    PRINTF("[FW_UPDATE] SHA-256 OK. Image ready.\r\n");
    gTransferDone = true;
}

/* -----------------------------------------------------------------------
 * Process_Firmware_Update_Messages  (public)
 *
 * Entry point for both the UDP receive path (enet_init.c) and the CAN
 * receive path (multi_mcan_user.c → StoreDataInSpecificRxBuffer).
 * Both paths hand raw bytes here — the logic is identical regardless
 * of which transport delivered them.
 * ----------------------------------------------------------------------- */
void Process_Firmware_Update_Messages(const uint8_t* buffer, uint32_t len)
{
    if (len == 0) return;

    switch (buffer[0])
    {
        case PKT_TYPE_META: handle_meta_packet(buffer, len); break;
        case PKT_TYPE_DATA: handle_data_packet(buffer, len); break;
        default:
            PRINTF("[FW_UPDATE] Unknown type 0x%02X\r\n", buffer[0]);
            break;
    }
}

/* -----------------------------------------------------------------------
 * Check_And_Write_Firmware  (public)
 *
 * Returns:
 *    0  — idle, nothing to do
 *    1  — flash write succeeded (self-targeted image)
 *   -1  — flash write failed
 *    2  — image was for another card; forwarded over CAN
 * ----------------------------------------------------------------------- */
int Check_And_Write_Firmware(void)
{
    if (!gTransferDone) {
        return 0;
    }

    gTransferDone = false;
    gMetaReceived = false;

    /* If we are the VCC card and the image targets a different card type,
     * forward it over CAN instead of writing it to our own flash.         */
    if (self_controller_id == VCC_1_CONTROLLER_1_ID)
    {
        if ((gCardType == FIRMWARE_INPUT_CARD_TYPE)  ||
            (gCardType == FIRMWARE_OUTPUT_CARD_TYPE) ||
            (gCardType == FIRMWARE_ANALOG_CARD_TYPE))
        {
            PRINTF("[FW_UPDATE] Forwarding image (card=%u) over CAN.\r\n", gCardType);
            Transmit_AppImage_On_CAN();
            return 2;
        }
    }

    PRINTF("[FW_UPDATE] Writing %u bytes to flash...\r\n", gExpectedSize);

    int32_t status = QSPI_Write_Full_Image(gImageBuffer, gExpectedSize, gExpectedCRC);

    if (status == SystemP_SUCCESS) {
        PRINTF("[FW_UPDATE] *** FLASH WRITE SUCCESS ***\r\n");
        return 1;
    }

    PRINTF("[FW_UPDATE] *** FLASH WRITE FAILED ***\r\n");
    return -1;
}

/* -----------------------------------------------------------------------
 * Transmit_AppImage_On_CAN  (public)
 *
 * Called by Check_And_Write_Firmware when the VCC card has received and
 * verified an image for a non-VCC card type.  Re-transmits the full
 * verified image over CAN using exactly the same MetaHeader + ChunkHeader
 * protocol that UDP uses, so the receiving card's
 * Process_Firmware_Update_Messages() requires zero changes.
 *
 * Transport mapping:
 *   FIRMWARE_INPUT_CARD_TYPE  → BUFF_VCC_TO_INPUT_FIRMWARE_MSG
 *   FIRMWARE_OUTPUT_CARD_TYPE → BUFF_VCC_TO_OUTPUT_FIRMWARE_MSG
 *   FIRMWARE_ANALOG_CARD_TYPE → BUFF_VCC_TO_ANALOG_FIRMWARE_MSG
 *
 * Frame sizing:
 *   Each CAN-FD frame's DATA_FRAME_PAYLOAD_SIZE is 61 bytes.
 *   MetaHeader  = 42 bytes → fits in one frame with 19 bytes to spare.
 *   ChunkHeader = 13 bytes + CAN_FW_CHUNK_SIZE(48) = 61 bytes → exactly
 *   one frame per firmware chunk.  No multi-frame reassembly needed.
 *
 * Both CAN instances (0 and 1) receive the same transmission because both
 * MCUs on the target card share the same CAN bus and need the image.
 * ----------------------------------------------------------------------- */
void Transmit_AppImage_On_CAN(void)
{
    /* ---- Determine which CAN message buffer to use ---- */
    uint8_t buff_num = 0;
    if (gCardType == FIRMWARE_INPUT_CARD_TYPE)
    {
        buff_num = BUFF_VCC_TO_INPUT_FIRMWARE_MSG;
    }
    else if (gCardType == FIRMWARE_OUTPUT_CARD_TYPE)
    {
        buff_num = BUFF_VCC_TO_OUTPUT_FIRMWARE_MSG;
    }
    else if (gCardType == FIRMWARE_ANALOG_CARD_TYPE)
    {
        buff_num = BUFF_VCC_TO_ANALOG_FIRMWARE_MSG;
    }
    else
    {
        PRINTF("[FW_UPDATE] CAN TX: unknown card type %u\r\n", gCardType);
        return;
    }

    /* ---------------------------------------------------------------
     * Step 1: Send the MetaHeader as the first CAN frame.
     *
     * The receiving card's Process_Firmware_Update_Messages() expects
     * to see a PKT_TYPE_META before any data packets — identical to the
     * UDP path.  We pack the MetaHeader into the CAN payload buffer and
     * send it as a single frame.
     * --------------------------------------------------------------- */
    {
        MetaHeader meta;
        meta.type      = PKT_TYPE_META;
        meta.card_type = gCardType;
        meta.totalSize = gExpectedSize;
        meta.image_crc = gExpectedCRC;
        memcpy(meta.image_sha256, gExpectedSHA256, 32U);

        /* Use a 64-byte staging buffer (full CAN frame payload size) so
         * TransmitCANMessage always sees a full-sized buffer.  The extra
         * bytes after sizeof(meta) are zero-padded and ignored by the
         * receiver because Process_Firmware_Update_Messages only reads
         * sizeof(MetaHeader) bytes for a META packet.                    */
        uint8_t tx_buf[DATA_FRAME_PAYLOAD_SIZE];
        memset(tx_buf, 0, sizeof(tx_buf));
        memcpy(tx_buf, &meta, sizeof(meta));

        PRINTF("[FW_UPDATE] CAN TX: sending META (card=%u size=%u)\r\n",
               gCardType, gExpectedSize);

        TransmitCANMessage(tx_buf, sizeof(meta), buff_num, 0);
        TransmitCANMessage(tx_buf, sizeof(meta), buff_num, 1);
    }

    /* ---------------------------------------------------------------
     * Step 2: Send the firmware image as DATA chunks.
     *
     * Each iteration builds one ChunkHeader + CAN_FW_CHUNK_SIZE bytes
     * of image data into a 61-byte buffer (DATA_FRAME_PAYLOAD_SIZE),
     * which becomes exactly one CAN-FD frame on the wire.
     *
     * The receiving card calls Process_Firmware_Update_Messages() for
     * each CAN frame it reads from the FIFO, so it sees an unbroken
     * stream of complete DATA packets, one per call.
     * --------------------------------------------------------------- */
    uint32_t offset = 0;
    uint32_t total  = gExpectedSize;
    uint32_t chunk_count = 0;

    PRINTF("[FW_UPDATE] CAN TX: sending %u bytes in %u chunks...\r\n",
           total,
           (total + CAN_FW_CHUNK_SIZE - 1U) / CAN_FW_CHUNK_SIZE);

    while (offset < total)
    {
        /* Last chunk may be smaller than CAN_FW_CHUNK_SIZE */
        uint32_t this_chunk = total - offset;
        if (this_chunk > CAN_FW_CHUNK_SIZE)
        {
            this_chunk = CAN_FW_CHUNK_SIZE;
        }

        /* Build the firmware DATA packet into a fixed-size staging buffer.
         * The buffer is exactly DATA_FRAME_PAYLOAD_SIZE (61) bytes:
         *   [0..12]  ChunkHeader  (13 bytes)
         *   [13..60] firmware payload  (up to 48 bytes, zero-padded)     */
        uint8_t tx_buf[DATA_FRAME_PAYLOAD_SIZE];
        memset(tx_buf, 0, sizeof(tx_buf));

        ChunkHeader hdr;
        hdr.type      = PKT_TYPE_DATA;
        hdr.offset    = offset;
        hdr.size      = this_chunk;
        hdr.chunk_crc = crcFast(gImageBuffer + offset, (int)this_chunk);

        memcpy(tx_buf,                 &hdr,                     sizeof(hdr));
        memcpy(tx_buf + sizeof(hdr),   gImageBuffer + offset,    this_chunk);

        /* The total packet size passed to TransmitCANMessage is
         * sizeof(ChunkHeader) + this_chunk.  For the last chunk this
         * may be less than DATA_FRAME_PAYLOAD_SIZE.  TransmitCANMessage
         * pads the CAN frame to 64 bytes automatically via MCAN_DATA_FRAME. */
        uint16_t pkt_len = (uint16_t)(sizeof(ChunkHeader) + this_chunk);

        /* Send on both CAN instances so both MCUs on the target card
         * receive the same image simultaneously.                           */
        TransmitCANMessage(tx_buf, pkt_len, buff_num, 0);
        TransmitCANMessage(tx_buf, pkt_len, buff_num, 1);

        offset += this_chunk;
        chunk_count++;
    }

    PRINTF("[FW_UPDATE] CAN TX: done. %u chunks sent.\r\n", chunk_count);
}
