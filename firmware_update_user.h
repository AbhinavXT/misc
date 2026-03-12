#ifndef FIRMWARE_UPDATE_USER_H
#define FIRMWARE_UPDATE_USER_H

/* Maximum supported firmware image size (1 MB) */
#define MAX_IMAGE_SIZE      (1U * 1024U * 1024U)

#define PKT_TYPE_META       0x00U
#define PKT_TYPE_DATA       0x01U

/*
 * CAN_FW_CHUNK_SIZE — payload bytes per firmware DATA chunk when transmitting
 * over CAN-FD.
 *
 * Each CAN-FD frame carries a MCAN_DATA_FRAME which has:
 *   3  bytes  STRUCT_MESSAGE_HEADER_CAN  (message_length uint16 + frame_num uint8)
 *   61 bytes  payload                   (DATA_FRAME_PAYLOAD_SIZE)
 *
 * A firmware DATA packet = ChunkHeader (13 bytes) + firmware chunk bytes.
 * To make exactly one CAN frame == one firmware chunk (zero reassembly),
 * the firmware payload portion must be:
 *   DATA_FRAME_PAYLOAD_SIZE(61) - sizeof(ChunkHeader)(13) = 48 bytes
 *
 * A 575 KB image at 48 bytes/chunk ~= 12,300 CAN frames.
 * At CAN-FD 5 Mbps data phase that takes roughly 200 ms — fine for a bootloader.
 */
#define CAN_FW_CHUNK_SIZE   48U

/* ---- Packet layout structs (shared between UDP and CAN paths) ---- */

typedef struct __attribute__((packed)) {
    uint8_t  type;             /* PKT_TYPE_META (0x00) */
    uint8_t  card_type;        /* FIRMWARE_VCC/INPUT/OUTPUT/ANALOG_CARD_TYPE */
    uint32_t totalSize;        /* full image size in bytes */
    uint32_t image_crc;        /* CRC32 over the full image */
    uint8_t  image_sha256[32]; /* SHA-256 of the full image */
} MetaHeader;                  /* 42 bytes — fits in one CAN frame payload */

typedef struct __attribute__((packed)) {
    uint8_t  type;             /* PKT_TYPE_DATA (0x01) */
    uint32_t offset;           /* byte position of this chunk in the full image */
    uint32_t size;             /* payload bytes in this packet */
    uint32_t chunk_crc;        /* CRC32 over this chunk's payload bytes only */
} ChunkHeader;                 /* 13 bytes — leaves 48 bytes for firmware payload */


void Process_Firmware_Update_Messages(const uint8_t* buffer, uint32_t len);
int  Check_And_Write_Firmware(void);
void Transmit_AppImage_On_CAN(void);

#endif /* FIRMWARE_UPDATE_USER_H */
