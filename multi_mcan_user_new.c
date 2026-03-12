#include "ti_drivers_open_close.h"
#include "ti_drivers_config.h"

#include "common_include.h"
#include "mcan.h"
#include "gpio_user.h"
#include "mcan_user.h"
#include "firmware_update_user.h"


#define MCAN_INTR_MASK_FIFO0 (MCAN_IR_RF0N_MASK | \
        MCAN_IR_RF0W_MASK | \
        MCAN_IR_RF0F_MASK | \
        MCAN_IR_RF0L_MASK | \
        MCAN_IR_HPM_MASK |  \
        MCAN_IR_TC_MASK |   \
        MCAN_IR_TCF_MASK |  \
        MCAN_IR_TFE_MASK |  \
        MCAN_IR_TEFN_MASK | \
        MCAN_IR_TEFW_MASK | \
        MCAN_IR_TEFF_MASK | \
        MCAN_IR_TEFL_MASK | \
        MCAN_IR_TSW_MASK |  \
        MCAN_IR_MRAF_MASK | \
        MCAN_IR_TOO_MASK |  \
        MCAN_IR_DRX_MASK |  \
        MCAN_IR_BEC_MASK |  \
        MCAN_IR_BEU_MASK |  \
        MCAN_IR_ELO_MASK |  \
        MCAN_IR_EP_MASK |   \
        MCAN_IR_EW_MASK |   \
        MCAN_IR_BO_MASK |   \
        MCAN_IR_WDI_MASK |  \
        MCAN_IR_PEA_MASK |  \
        MCAN_IR_PED_MASK |  \
        MCAN_IR_ARA_MASK)

#define MCAN_INTR_MASK_FIFO1  (MCAN_IR_RF1N_MASK | \
        MCAN_IR_RF1W_MASK | \
        MCAN_IR_RF1F_MASK | \
        MCAN_IR_RF1L_MASK)


extern uint8_t self_controller_id;

HwiP_Object       gMcanHwiObject;

uint32_t intrStatus_mcan0 = 0, intrStatus_mcan1 = 0,intrStatus_mcan2 = 0;

void *mcan_isr_array[] = {&MCAN0_IntrISR, &MCAN1_IntrISR, &MCAN2_IntrISR};
uint32_t mcan_intr_num_line0[3] = {CONFIG_MCAN0_INTR, CONFIG_MCAN1_INTR, CONFIG_MCAN2_INTR};
uint8_t mcan_priority_array[3] = {2, 3, 4};

uint32_t mcan_intr_num_line1[3] = {CONFIG_MCAN0_INTR1, CONFIG_MCAN1_INTR1, CONFIG_MCAN2_INTR1};

volatile uint8_t frame_tx_complete = 0;

uint8_t all_tx_buffs_occupied_can0 = 0;
uint8_t all_tx_buffs_occupied_can1 = 0;
uint8_t all_tx_buffs_occupied_can2 = 0; // not used currently

uint32_t mcan_intrStatus = 0;

/* -----------------------------------------------------------------------
 * Firmware-update receive ring buffer
 *
 * Problem this solves:
 *   ProcessIncomingCANFIFO drains the hardware FIFO (64 slots) as fast as
 *   possible.  If we called Process_Firmware_Update_Messages() directly
 *   inside that drain loop, the per-packet work (CRC32, memcpy into the
 *   1 MB image buffer, and occasionally SHA-256) would stall the drain long
 *   enough for the hardware FIFO to overflow while the VCC card keeps
 *   transmitting at CAN-FD 5 Mbps.  Lost frames = corrupt image.
 *
 * Solution - producer/consumer split:
 *   ProcessIncomingCANFIFO  (called from main loop)  = PRODUCER
 *     Copies raw payload bytes + metadata into the ring buffer.
 *     Does nothing else.  Stays fast.
 *
 *   mcan_checks             (called from main loop)  = CONSUMER
 *     Reads one slot at a time from the ring buffer and calls
 *     Process_Firmware_Update_Messages().  Runs at main-loop speed.
 *
 * Sizing:
 *   FW_RX_RING_SIZE = 128 slots.  The hardware FIFO holds at most 64 frames.
 *   128 slots means even if mcan_checks is skipped for two full FIFO fills,
 *   we still don't lose data.
 *   Each slot is DATA_FRAME_PAYLOAD_SIZE (61) + 4 bytes overhead = 65 bytes,
 *   so total buffer is 128 * 65 = 8320 bytes in SRAM.
 * ----------------------------------------------------------------------- */
#define FW_RX_RING_SIZE     128U

typedef struct
{
    uint16_t msg_id;                        /* 11-bit CAN standard ID */
    uint16_t len;                           /* valid bytes in data[]  */
    uint8_t  data[DATA_FRAME_PAYLOAD_SIZE]; /* raw firmware pkt bytes */
} CanFwRxSlot;

static CanFwRxSlot  gFwRxRing[FW_RX_RING_SIZE];
static uint32_t     gFwRxWriteIdx = 0;  /* advanced by producer (FIFO drain) */
static uint32_t     gFwRxReadIdx  = 0;  /* advanced by consumer (mcan_checks) */
static uint32_t     gFwRxDropped  = 0;  /* overrun counter for diagnostics    */

/* Ring full: next write would land on the read index */
#define FW_RX_RING_FULL()   (((gFwRxWriteIdx + 1U) % FW_RX_RING_SIZE) == gFwRxReadIdx)
/* Ring empty: read and write indices coincide */
#define FW_RX_RING_EMPTY()  (gFwRxWriteIdx == gFwRxReadIdx)


unsigned int mcan_err_intr_status[8] = {
                                        MCAN_INTR_SRC_PROTOCOL_ERR_DATA,
                                        MCAN_INTR_SRC_PROTOCOL_ERR_ARB,
                                        MCAN_INTR_SRC_BUS_OFF_STATUS,
                                        MCAN_INTR_SRC_ERR_PASSIVE,
                                        MCAN_INTR_SRC_ERR_LOG_OVRFLW,
                                        MCAN_INTR_SRC_BIT_ERR_UNCORRECTED,
                                        MCAN_INTR_SRC_BIT_ERR_CORRECTED,
                                        MCAN_INTR_SRC_MSG_RAM_ACCESS_FAILURE
};


uint16_t STD_MSG_RX_ACC_ID[APP_MCAN_STD_ID_FILTER_CNT_RX] =
{
 CAN_VCC_TO_INPUT_FIRMWARE_MSG, // for input
 CAN_VCC_TO_OUTPUT_FIRMWARE_MSG,// for output
 CAN_VCC_TO_ANALOG_FIRMWARE_MSG,// for analog
};


// for vcc only
uint16_t STD_MSG_TX_ACC_ID[APP_MCAN_STD_ID_FILTER_CNT_TX] =
{
 CAN_VCC_TO_INPUT_FIRMWARE_MSG,
 CAN_VCC_TO_OUTPUT_FIRMWARE_MSG,
 CAN_VCC_TO_ANALOG_FIRMWARE_MSG,
};


int32_t MonitorCANErr(uint8_t can_instance)
{
    uint32_t base_addr = 0;
    if(can_instance == 0x00)
    {
        base_addr = CSL_MCAN0_MSG_RAM_U_BASE;
        mcan_intrStatus = intrStatus_mcan0;
    }
    else if(can_instance == 0x01)
    {
        base_addr = CSL_MCAN1_MSG_RAM_U_BASE;
        mcan_intrStatus = intrStatus_mcan1;
    }
    else if(can_instance == 0x02)
    {
        base_addr = CSL_MCAN2_MSG_RAM_U_BASE;
        mcan_intrStatus = intrStatus_mcan2;
    }
    else
    {
        PRINTF("MON WRONG CAN INSTANCE\n");
        return 0;
    }


    int32_t mcan_err_res = 0;


    uint32_t gMcanBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(base_addr);

    MCAN_ProtocolStatus protstatus;
    memset(&protstatus,0,sizeof(protstatus));

    MCAN_getProtocolStatus(gMcanBaseAddr, &protstatus);

    //     PRINTF("PSR %d %d %d %d %d %d %d \n",protstatus.lastErrCode,protstatus.errPassive,protstatus.act,protstatus.dlec,protstatus.busOffStatus,
    //            protstatus.tdcv,protstatus.pxe);

    if(protstatus.busOffStatus == 1)
    {
        PRINTF("Node is in bus off state \n");
        mcan_err_res |= 0x01;
    }


    //    if(protstatus.errPassive == 1)
    //    {
    //        PRINTF("NODE is in error passive state\n");
    //        mcan_err_res |= 0x02;
    //    }

    if(protstatus.lastErrCode > 0 && protstatus.lastErrCode < 7)
    {
        PRINTF("LEC %d\n",protstatus.lastErrCode);
        mcan_err_res |= 0x02;
    }

    //     PRINTF("\n\nMCAN: [%d] PROTOCOL: \nACT:[%d]\tBUSOFF:[%d]\tDLEC:[%d]\tERRPSV:[%d]\tLEC:[%d]\tPXE:[%d]\tRBRS:[%d]\tRESI:[%d]\tRFDF:[%d]\tTDCV:[%d]\tWARN:[%d]\n\n", can_instance, protstatus.act, protstatus.busOffStatus, protstatus.dlec, protstatus.errPassive, protstatus.lastErrCode, protstatus.pxe, protstatus.rbrs, protstatus.resi, protstatus.rfdf, protstatus.tdcv, protstatus.warningStatus);


    if(protstatus.dlec > 0 && protstatus.dlec < 7)
    {
        PRINTF("DLEC %d\n",protstatus.dlec);
        mcan_err_res |= 0x04;
    }

    MCAN_ErrCntStatus       errCounter;

    MCAN_getErrCounters(gMcanBaseAddr, &errCounter);
    // DebugP_assert((0U == errCounter.recErrCnt) &&
    //             (0U == errCounter.canErrLogCnt));

    //  PRINTF("ERR COUNTERS %d %d  %d %d\n",errCounter.canErrLogCnt,errCounter.recErrCnt,errCounter.rpStatus,errCounter.transErrLogCnt);

    if(errCounter.transErrLogCnt > 127) // 1// 255)
    {
        //set the init bit in CCCR register
        PRINTF("Tnans err count [%d] \n" , errCounter.transErrLogCnt);
        mcan_err_res |= 0x08;
    }

    //        uint32_t transErrLogCnt;
    /**< Transmit Error Counter */
    if(errCounter.recErrCnt > 127)
    {
        PRINTF("recErrCnt  count [%d] \n" , errCounter.recErrCnt);
        mcan_err_res |= 0x10;
    }

    for(int i = 0; i < 8; i++)
    {
        if(mcan_intrStatus == mcan_err_intr_status[i])
        {
            mcan_err_res |= 0x20;
            PRINTF("MCAN INTR ERR: [%d]\n", i);
        }
    }


    return mcan_err_res;
}


/*
 * mcan_checks  — CONSUMER side of the ring buffer
 *
 * Called from the main while(1) loop on every iteration.
 *
 * This function processes one slot from the ring buffer per call rather
 * than draining the entire ring in a tight loop.  The reason is the same
 * as why we split produce and consume in the first place: we want the
 * main loop to remain responsive.  Processing one packet per iteration
 * means:
 *   - The main loop still calls ethernet_checks() and Check_And_Write_Firmware()
 *     promptly on every iteration.
 *   - At ~12,300 packets for a 575 KB image, and assuming the main loop
 *     runs at least 10,000 iterations/second, the entire image is consumed
 *     in just over 1 second — well within any reasonable bootloader budget.
 *   - The ring buffer (128 slots) provides enough cushion so that brief
 *     pauses in the main loop don't cause the producer to overrun.
 *
 * If you ever need to drain faster (e.g. the main loop has other heavy
 * work), you can change this to a while(!FW_RX_RING_EMPTY()) loop with a
 * bounded iteration count.
 */
void mcan_checks(void)
{
    if (FW_RX_RING_EMPTY())
    {
        return;
    }

    /* Read the next slot from the ring buffer */
    CanFwRxSlot* slot = &gFwRxRing[gFwRxReadIdx];

    /*
     * Call the transport-agnostic firmware update handler.
     * It sees the same byte stream as the UDP path — either a MetaHeader
     * (buff[0] == PKT_TYPE_META) or a ChunkHeader+data (buff[0] == PKT_TYPE_DATA).
     * All integrity logic (per-chunk CRC32, whole-image CRC32, SHA-256) lives
     * there and is unchanged.
     */
    Process_Firmware_Update_Messages(slot->data, (uint32_t)slot->len);

    /* Advance the read index — must happen AFTER we're done reading the slot */
    gFwRxReadIdx = (gFwRxReadIdx + 1U) % FW_RX_RING_SIZE;
}


/*
 * StoreDataInSpecificRxBuffer  — PRODUCER side of the ring buffer
 *
 * Called by ProcessIncomingCANFIFO for every frame drained from the
 * hardware FIFO.  Its only job is to decide whether this message ID is
 * a firmware update packet, and if so, push the raw payload bytes into
 * the ring buffer as quickly as possible.
 *
 * It deliberately does NOT call Process_Firmware_Update_Messages() here.
 * The CRC computation, memcpy into gImageBuffer, and potential SHA-256
 * at end-of-transfer all take meaningful time.  If that work happened
 * here, the FIFO drain loop would slow down and the 64-slot hardware
 * FIFO could overflow while the VCC card is still transmitting at
 * CAN-FD 5 Mbps.  All processing is deferred to mcan_checks().
 *
 * On ring-full: the slot is dropped and gFwRxDropped is incremented.
 * This should never happen in normal operation (128 slots >> 64 FIFO
 * slots), but the counter gives you a diagnostic signal if it does.
 */
void StoreDataInSpecificRxBuffer(uint8_t* buff, uint16_t pkt_msg_id,
                                  uint16_t msg_len, uint8_t curr_frame_num)
{
    switch (pkt_msg_id)
    {
        case CAN_VCC_TO_INPUT_FIRMWARE_MSG:
        case CAN_VCC_TO_OUTPUT_FIRMWARE_MSG:
        case CAN_VCC_TO_ANALOG_FIRMWARE_MSG:
        {
            /* Safety clamp — guard against a corrupt message_length field */
            uint16_t safe_len = msg_len;
            if (safe_len > DATA_FRAME_PAYLOAD_SIZE)
            {
                safe_len = DATA_FRAME_PAYLOAD_SIZE;
            }

            if (FW_RX_RING_FULL())
            {
                /* Ring overrun — drop this frame and count it */
                gFwRxDropped++;
                PRINTF("[MCAN] RX ring FULL — dropped frame (total=%u)\r\n",
                       gFwRxDropped);
                break;
            }

            /* Copy into the next available ring slot */
            CanFwRxSlot* slot = &gFwRxRing[gFwRxWriteIdx];
            slot->msg_id = pkt_msg_id;
            slot->len    = safe_len;
            memcpy(slot->data, buff, safe_len);

            /* Advance the write index — must be the LAST write so the
             * consumer never sees a partially-filled slot.              */
            gFwRxWriteIdx = (gFwRxWriteIdx + 1U) % FW_RX_RING_SIZE;
            break;
        }

        default:
            /* Non-firmware CAN message — add other handlers here */
            break;
    }

    return;
}


static void mcan_ConfigTxMsg(MCAN_TxBufElement *txMsg, uint8_t idx)
{
    /* Initialize message to transmit */
    MCAN_initTxBufElement(txMsg);
    /* Standard message identifier 11 bit, stored into ID[28-18] */
    txMsg->id  = ((STD_MSG_TX_ACC_ID[idx] & MCAN_STD_ID_MASK) << MCAN_STD_ID_SHIFT);
    txMsg->dlc = MCAN_DATA_SIZE_64BYTES; /* Payload size is 64 bytes */
    txMsg->fdf = TRUE; /* CAN FD Frame Format */
    txMsg->xtd = FALSE; /* Extended id not configured */
    txMsg->brs = TRUE; //TRUE;
}

static void mcan_ConfigTxMsgNonBlocking(MCAN_TxBufElement *txMsg, uint8_t msg_id)
{
    /* Initialize message to transmit */
    MCAN_initTxBufElement(txMsg);
    /* Standard message identifier 11 bit, stored into ID[28-18] */
    txMsg->id  = ((msg_id & MCAN_STD_ID_MASK) << MCAN_STD_ID_SHIFT);
    txMsg->dlc = MCAN_DATA_SIZE_64BYTES; /* Payload size is 64 bytes */
    txMsg->fdf = TRUE; /* CAN FD Frame Format */
    txMsg->xtd = FALSE; /* Extended id not configured */
    txMsg->brs = TRUE; //TRUE;
}

void TransmitCANMessage(uint8_t* tx_buff, uint16_t msg_len, uint8_t buff_num, uint8_t can_instance)
{
    uint8_t is_master_controller = 1;

    if(is_master_controller == 1)
    {
        int32_t                 status = SystemP_SUCCESS;
        MCAN_TxBufElement       txMsg;
        uint8_t curr_frame_num = 0;
        uint16_t offset = 0;

        uint32_t base_addr = 0;
        if(can_instance == 0x00)
        {
            base_addr = CONFIG_MCAN0_BASE_ADDR;
        }
        else if(can_instance == 0x01)
        {
            base_addr = CONFIG_MCAN1_BASE_ADDR;
        }
        else if(can_instance == 0x02)
        {
            base_addr = CONFIG_MCAN2_BASE_ADDR;
        }
        else
        {
            PRINTF("THIS WRONG CAN INSTANCE\n");
            return;
        }

        uint32_t gMcanBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(base_addr);

        uint8_t free_buff_num = 0;

        volatile uint32_t txstatus = MCAN_getTxBufReqPend(gMcanBaseAddr);

        while(!(txstatus & (1U << free_buff_num)))
        {
            free_buff_num++;
        }

        while (offset < msg_len)
        {
            mcan_ConfigTxMsg(&txMsg, buff_num);

            MCAN_DATA_FRAME data_frame;
            memset(&data_frame, 0, sizeof(data_frame));

            data_frame.msg_header.message_length = msg_len;
            data_frame.msg_header.frame_num = curr_frame_num;

            int bytes_to_copy = (msg_len - offset >= DATA_FRAME_PAYLOAD_SIZE) ? DATA_FRAME_PAYLOAD_SIZE : (msg_len - offset);
            memcpy(data_frame.payload, tx_buff + offset, bytes_to_copy);
            memcpy(&txMsg.data, &data_frame, MCAN_MAX_FRAME_SIZE);

            status = MCAN_txBufTransIntrEnable(gMcanBaseAddr, buff_num, (uint32_t)TRUE);
            DebugP_assert(status == CSL_PASS);

            /* Write message to Msg RAM */
            MCAN_writeMsgRam(gMcanBaseAddr, MCAN_MEM_TYPE_BUF, buff_num, &txMsg);

            /* Add request for transmission, This function will trigger transmission */
            status = MCAN_txBufAddReq(gMcanBaseAddr, buff_num);
            DebugP_assert(status == CSL_PASS);

            while(frame_tx_complete == 0);
            frame_tx_complete = 0;

            curr_frame_num++;
            offset += bytes_to_copy;

            memset(&txMsg, 0, sizeof(txMsg));
        }
    }
}


void TransmitCANMessageNonBlocking(uint8_t* tx_buff, uint16_t msg_len, uint8_t buff_num, uint8_t can_instance)
{
    int32_t                 status = SystemP_SUCCESS;
    MCAN_TxBufElement       txMsg;
    uint8_t curr_frame_num = 0;
    uint16_t offset = 0;

    uint32_t base_addr = 0;
    if(can_instance == 0x00)
    {
        base_addr = CONFIG_MCAN0_BASE_ADDR;
    }
    else if(can_instance == 0x01)
    {
        base_addr = CONFIG_MCAN1_BASE_ADDR;
    }
    else if(can_instance == 0x02)
    {
        base_addr = CONFIG_MCAN2_BASE_ADDR;
    }
    else
    {
        PRINTF("TRANS WRONG CAN INSTANCE\n");
        return;
    }

    uint32_t gMcanBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(base_addr);

    uint8_t free_buff_num = 0;

    volatile uint32_t txstatus = MCAN_getTxBufReqPend(gMcanBaseAddr);

    uint8_t msg_id = STD_MSG_TX_ACC_ID[buff_num];

    while (offset < msg_len)
    {
        txstatus = MCAN_getTxBufReqPend(gMcanBaseAddr);
        while((txstatus & (1U << free_buff_num)))
        {
            free_buff_num++;
            if(free_buff_num == 32)
            {
                free_buff_num = 0;

                if(can_instance == 0x00)
                {
                    all_tx_buffs_occupied_can0 = 1;
                }
                else if(can_instance == 0x01)
                {
                    all_tx_buffs_occupied_can1 = 1;
                }
                else if(can_instance == 0x02)
                {
                    all_tx_buffs_occupied_can2 = 1;
                }
                else
                {
                    ;
                }

                return;

            }
        }

        mcan_ConfigTxMsgNonBlocking(&txMsg, msg_id);

        MCAN_DATA_FRAME data_frame;
        memset(&data_frame, 0, sizeof(data_frame));

        data_frame.msg_header.message_length = msg_len;
        data_frame.msg_header.frame_num = curr_frame_num;

        int bytes_to_copy = (msg_len - offset >= DATA_FRAME_PAYLOAD_SIZE) ? DATA_FRAME_PAYLOAD_SIZE : (msg_len - offset);
        memcpy(data_frame.payload, tx_buff + offset, bytes_to_copy);
        memcpy(&txMsg.data, &data_frame, MCAN_MAX_FRAME_SIZE);

        /* Write message to Msg RAM */
        MCAN_writeMsgRam(gMcanBaseAddr, MCAN_MEM_TYPE_BUF, free_buff_num, &txMsg);

        /* Add request for transmission, This function will trigger transmission */
        status = MCAN_txBufAddReq(gMcanBaseAddr, free_buff_num);
        DebugP_assert(status == CSL_PASS);


        if(msg_len > DATA_FRAME_PAYLOAD_SIZE)
        {
            ;
        }

        curr_frame_num++;
        offset += bytes_to_copy;

        memset(&txMsg, 0, sizeof(txMsg));
    }
}


void MCAN0_IntrISR(void *arg)
{
    uint32_t gMcanBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_MCAN0_BASE_ADDR);

    intrStatus_mcan0 = MCAN_getIntrStatus(gMcanBaseAddr);
    MCAN_clearIntrStatus(gMcanBaseAddr, intrStatus_mcan0);

    return;
}

void MCAN1_IntrISR(void *arg)
{
    uint32_t gMcanBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_MCAN1_BASE_ADDR);

    intrStatus_mcan1 = MCAN_getIntrStatus(gMcanBaseAddr);
    MCAN_clearIntrStatus(gMcanBaseAddr, intrStatus_mcan1);

    return;
}

void MCAN2_IntrISR(void *arg)
{
    uint32_t gMcanBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_MCAN2_BASE_ADDR);

    intrStatus_mcan2 = MCAN_getIntrStatus(gMcanBaseAddr);
    MCAN_clearIntrStatus(gMcanBaseAddr, intrStatus_mcan2);

    return;
}


static void mcan_InitStdFilterElemParams(MCAN_StdMsgIDFilterElement *stdFiltElem, uint32_t bufNum)
{
    /* sfid1 defines the ID of the standard message to be stored. */
    stdFiltElem->sfid1 = STD_MSG_RX_ACC_ID[bufNum]; // STD_MSG_RX_ACC_ID[bufNum]; //STD_MSG_RX_ACC_ID[0];
    /* As buffer mode is selected, sfid2 should be bufNum[0 - 63] */
    stdFiltElem->sfid2 = 0; //STD_MSG_RX_ACC_ID[last_element];

    /* Store message in FIFO 0 */
    stdFiltElem->sfec  = MCAN_STD_FILT_ELEM_FIFO0;

    /* range filter is applied for fifo */
    stdFiltElem->sft   = MCAN_STD_FILT_TYPE_DUAL;

    return;
}


/*
 * ProcessIncomingCANFIFO
 *
 * Called from the main while(1) loop (not from an ISR).  Drains the
 * hardware Rx FIFO as fast as possible — every frame is copied into the
 * ring buffer via StoreDataInSpecificRxBuffer and the FIFO slot is
 * immediately acknowledged so the hardware can reuse it.
 *
 * No firmware processing happens here.  The ISR already cleared the
 * interrupt flag in the IntrISR handler; this function just reads the data.
 */
void ProcessIncomingCANFIFO(uint8_t can_instance, uint8_t intr_line)
{
    MCAN_ErrCntStatus  errCounter;
    MCAN_RxBufElement  rxMsg;
    MCAN_DATA_FRAME    data_frame;

    uint32_t base_addr = 0;
    if (can_instance == 0x00)
    {
        base_addr = CONFIG_MCAN0_BASE_ADDR;
    }
    else if (can_instance == 0x01)
    {
        base_addr = CONFIG_MCAN1_BASE_ADDR;
    }
    else if (can_instance == 0x02)
    {
        base_addr = CONFIG_MCAN2_BASE_ADDR;
    }
    else
    {
        PRINTF("FIFO WRONG CAN INSTANCE\n");
        return;
    }

    uint8_t fifo_num = (intr_line == MCAN_INTR_LINE_NUM_0)
                       ? MCAN_RX_FIFO_NUM_0 : MCAN_RX_FIFO_NUM_1;

    uint32_t gMcanBaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(base_addr);

    MCAN_getErrCounters(gMcanBaseAddr, &errCounter);

    MCAN_RxFIFOStatus fifoStatus;
    fifoStatus.num = fifo_num;
    MCAN_getRxFIFOStatus(gMcanBaseAddr, &fifoStatus);

    while (fifoStatus.fillLvl > 0)
    {
        uint32_t fifoIdx = fifoStatus.getIdx;

        MCAN_readMsgRam(gMcanBaseAddr, MCAN_MEM_TYPE_FIFO,
                        fifoIdx, fifo_num, &rxMsg);

        memset(&data_frame, 0, sizeof(data_frame));
        memcpy(&data_frame, &rxMsg.data, sizeof(data_frame));

        uint16_t msg_len        = data_frame.msg_header.message_length;
        uint8_t  curr_frame_num = data_frame.msg_header.frame_num;
        uint16_t pkt_msg_id     = rxMsg.id >> MCAN_STD_ID_SHIFT;

        /*
         * Push into the ring buffer — this is the only work done here.
         * data_frame.payload contains the raw firmware packet bytes
         * (MetaHeader or ChunkHeader+data) packed by Transmit_AppImage_On_CAN.
         * msg_len is the exact byte count the transmitter stored in
         * STRUCT_MESSAGE_HEADER_CAN.message_length, so the consumer knows
         * how many bytes to pass to Process_Firmware_Update_Messages.
         */
        StoreDataInSpecificRxBuffer(data_frame.payload, pkt_msg_id,
                                    msg_len, curr_frame_num);

        memset(&rxMsg,      0, sizeof(rxMsg));
        memset(&data_frame, 0, sizeof(data_frame));

        /* Acknowledge FIFO slot — frees it for the next incoming frame */
        MCAN_writeRxFIFOAck(gMcanBaseAddr, fifo_num, fifoIdx);

        MCAN_getRxFIFOStatus(gMcanBaseAddr, &fifoStatus);
    }
}

void mcan_InitMsgRamConfigParams(MCAN_MsgRAMConfigParams *msgRAMConfigParams)
{
    int32_t status;

    MCAN_initMsgRamConfigParams(msgRAMConfigParams);

    /* Configure the user required msg ram params */
    msgRAMConfigParams->lss = APP_MCAN_STD_ID_FILTER_CNT_MAX; //for fifo this can be configured to be 1 in macro can_fifo_receive
    msgRAMConfigParams->lse = APP_MCAN_EXT_ID_FILTER_CNT;
    msgRAMConfigParams->txBufCnt = APP_MCAN_TX_BUFF_CNT;
    msgRAMConfigParams->txFIFOCnt = APP_MCAN_TX_FIFO_CNT;
    /* Buffer/FIFO mode is selected */
    msgRAMConfigParams->txBufMode = MCAN_TX_MEM_TYPE_BUF;
    msgRAMConfigParams->txEventFIFOCnt = APP_MCAN_TX_EVENT_FIFO_CNT;

    msgRAMConfigParams->rxFIFO0Cnt           = (uint32_t)64U;//(uint32_t)0U;
    msgRAMConfigParams->rxFIFO0WaterMark     = (uint32_t)1U;//(uint32_t)0U;
    msgRAMConfigParams->rxFIFO0OpMode        = (uint32_t)1U; //(uint32_t)0U;

    msgRAMConfigParams->rxFIFO1Cnt = 64U;
    msgRAMConfigParams->rxFIFO1WaterMark     = (uint32_t)1U;
    msgRAMConfigParams->rxFIFO1OpMode        = (uint32_t)1U;


    msgRAMConfigParams->rxBufElemSize        = MCAN_ELEM_SIZE_64BYTES;
    msgRAMConfigParams->rxFIFO0ElemSize      = MCAN_ELEM_SIZE_64BYTES;
    msgRAMConfigParams->rxFIFO1ElemSize      = MCAN_ELEM_SIZE_64BYTES;
    msgRAMConfigParams->txBufElemSize        = MCAN_ELEM_SIZE_64BYTES;



    status = MCAN_calcMsgRamParamsStartAddr(msgRAMConfigParams);
    DebugP_assert(status == CSL_PASS);

    return;
}


void mcan_EnableIntr(uint8_t can_instance, uint8_t line_num)
{
    uint32_t base_addr = 0;
    if(can_instance == 0x00)
    {
        base_addr = CONFIG_MCAN0_BASE_ADDR;
    }
    else if(can_instance == 0x01)
    {
        base_addr = CONFIG_MCAN1_BASE_ADDR;
    }
    else if(can_instance == 0x02)
    {
        base_addr = CONFIG_MCAN2_BASE_ADDR;
    }
    else
    {
        PRINTF("ENABLE INTR WRONG CAN INSTANCE\n");
        return;
    }

    uint32_t gMcanBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(base_addr);

    HwiP_Params hwiPrms;
    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.callback    = mcan_isr_array[can_instance];
    hwiPrms.priority    = mcan_priority_array[can_instance];

    if(line_num == MCAN_INTR_LINE_NUM_0)
    {
        hwiPrms.intNum      = mcan_intr_num_line0[can_instance];

        HwiP_construct(&gMcanHwiObject, &hwiPrms);

        MCAN_enableIntr(gMcanBaseAddr, MCAN_INTR_MASK_ALL, (uint32_t)TRUE);
        MCAN_enableIntr(gMcanBaseAddr,
                        MCAN_INTR_SRC_RES_ADDR_ACCESS, (uint32_t)FALSE);


        /* Select Interrupt Line 0 */
        MCAN_selectIntrLine(gMcanBaseAddr, MCAN_INTR_MASK_FIFO0, MCAN_INTR_LINE_NUM_0);
        /* Enable Interrupt Line */
        MCAN_enableIntrLine(gMcanBaseAddr, MCAN_INTR_LINE_NUM_0, (uint32_t)TRUE);
    }
    else if(line_num == MCAN_INTR_LINE_NUM_1)
    {
        hwiPrms.intNum      = mcan_intr_num_line1[can_instance];

        HwiP_construct(&gMcanHwiObject, &hwiPrms);

        MCAN_enableIntr(gMcanBaseAddr, MCAN_INTR_MASK_ALL, (uint32_t)TRUE);
        MCAN_enableIntr(gMcanBaseAddr,
                        MCAN_INTR_SRC_RES_ADDR_ACCESS, (uint32_t)FALSE);

        /* Select Interrupt Line 1 */
        MCAN_selectIntrLine(gMcanBaseAddr,  MCAN_INTR_MASK_FIFO1 , MCAN_INTR_LINE_NUM_1);
        /* Enable Interrupt Line */
        MCAN_enableIntrLine(gMcanBaseAddr, MCAN_INTR_LINE_NUM_1, (uint32_t)TRUE);
    }
    else
    {
        ;
    }

    return;
}



void MCAN_SetConfig(Bool enableInternalLpbk, uint8_t can_instance)
{
    MCAN_StdMsgIDFilterElement stdFiltElem[APP_MCAN_STD_ID_FILTER_CNT_RX] = {0U};
    MCAN_InitParams            initParams = {0U};
    MCAN_ConfigParams          configParams = {0U};
    MCAN_MsgRAMConfigParams    msgRAMConfigParams = {0U};
    MCAN_BitTimingParams       bitTimes = {0U};
    uint32_t                   i;

    uint32_t base_addr = 0;
    if(can_instance == 0x00)
    {
        base_addr = CONFIG_MCAN0_BASE_ADDR;
    }
    else if(can_instance == 0x01)
    {
        base_addr = CONFIG_MCAN1_BASE_ADDR;
    }
    else if(can_instance == 0x02)
    {
        base_addr = CONFIG_MCAN2_BASE_ADDR;
    }
    else
    {
        PRINTF("SETCFG WRONG CAN INSTANCE\n");
        return;
    }

    uint32_t gMcanBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(base_addr);

    /* Initialize MCAN module initParams */
    MCAN_initOperModeParams(&initParams);
    /* CAN FD Mode and Bit Rate Switch Enabled */

    initParams.fdMode          = TRUE;//FALSE;
    initParams.brsEnable       = TRUE;

    /* Initialize MCAN module Global Filter Params */
    MCAN_initGlobalFilterConfigParams(&configParams);

    /* Initialize MCAN module Bit Time Params */
    /* Configuring default 1Mbps and 5Mbps as nominal and data bit-rate resp */
    MCAN_initSetBitTimeParams(&bitTimes);

    /* Initialize MCAN module Message Ram Params */
    mcan_InitMsgRamConfigParams(&msgRAMConfigParams);


    if((self_controller_id >= INPUT_CARD_1_CONTROLLER_1_ID) && (self_controller_id <= INPUT_CARD_4_CONTROLLER_2_ID))
    {
        mcan_InitStdFilterElemParams(&stdFiltElem[0], BUFF_VCC_TO_INPUT_FIRMWARE_MSG);
    }

    if((self_controller_id >= OUTPUT_CARD_1_CONTROLLER_1_ID) && (self_controller_id <= OUTPUT_CARD_1_CONTROLLER_2_ID))
    {
        mcan_InitStdFilterElemParams(&stdFiltElem[0], BUFF_VCC_TO_OUTPUT_FIRMWARE_MSG);
    }

    if((self_controller_id >= ANALOG_CARD_1_CONTROLLER_1_ID) && (self_controller_id <= ANALOG_CARD_1_CONTROLLER_2_ID))
    {
        mcan_InitStdFilterElemParams(&stdFiltElem[0], BUFF_VCC_TO_ANALOG_FIRMWARE_MSG);
    }

    //    for (i = 0U; i < APP_MCAN_STD_ID_FILTER_CNT_RX; i++)
    //    {
    //        mcan_InitStdFilterElemParams(&stdFiltElem[i], i);
    //    }
    /* wait for memory initialization to happen */
    while (FALSE == MCAN_isMemInitDone(gMcanBaseAddr))
    {}

    /* Put MCAN in SW initialization mode */
    MCAN_setOpMode(gMcanBaseAddr, MCAN_OPERATION_MODE_SW_INIT);
    while (MCAN_OPERATION_MODE_SW_INIT != MCAN_getOpMode(gMcanBaseAddr))
    {}

    /* Initialize MCAN module */
    MCAN_init(gMcanBaseAddr, &initParams);
    /* Configure MCAN module Gloabal Filter */
    MCAN_config(gMcanBaseAddr, &configParams);
    /* Configure Bit timings */
    MCAN_setBitTime(gMcanBaseAddr, &bitTimes);
    /* Configure Message RAM Sections */
    MCAN_msgRAMConfig(gMcanBaseAddr, &msgRAMConfigParams);
    /* Set Extended ID Mask */
    MCAN_setExtIDAndMask(gMcanBaseAddr, APP_MCAN_EXT_ID_MASK);

#ifdef CAN_FIFO_RECEIVE

    //MCAN_addStdMsgIDFilter(gMcanBaseAddr, 0, &stdFiltElem[0]);
    for (i = 0U; i < APP_MCAN_STD_ID_FILTER_CNT_RX; i++)
    {
        MCAN_addStdMsgIDFilter(gMcanBaseAddr, i, &stdFiltElem[i]);
    }

#else

    /* Configure Standard ID filter element */

    for (i = 0U; i < APP_MCAN_STD_ID_FILTER_CNT_RX; i++)
    {
        MCAN_addStdMsgIDFilter(gMcanBaseAddr, i, &stdFiltElem[i]);
    }
#endif

    if (TRUE == enableInternalLpbk)
    {
        MCAN_lpbkModeEnable(gMcanBaseAddr, MCAN_LPBK_MODE_INTERNAL, TRUE);
    }

    /* Take MCAN out of the SW initialization mode */
    MCAN_setOpMode(gMcanBaseAddr, MCAN_OPERATION_MODE_NORMAL);
    while (MCAN_OPERATION_MODE_NORMAL != MCAN_getOpMode(gMcanBaseAddr));

    return;
}

void MCAN_Init(uint8_t can_instance)
{
    /* Configure MCAN module, Enable External LoopBack Mode */
    MCAN_SetConfig(APP_MCAN_LOOPBACK_MODE_DISABLE, can_instance);

    mcan_EnableIntr(can_instance,MCAN_INTR_LINE_NUM_0);
    mcan_EnableIntr(can_instance,MCAN_INTR_LINE_NUM_1);

    all_tx_buffs_occupied_can0 = 0;
    all_tx_buffs_occupied_can1 = 0;
    all_tx_buffs_occupied_can2 = 0;

    return;
}
