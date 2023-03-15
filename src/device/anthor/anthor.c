#include <device/anthor/anthor.h>

#if defined(DEVICE_ANCHOR)

#define APP_NAME "anthor v0.0"
LOG_MODULE_REGISTER(device_anthor, LOG_LEVEL_DBG);

#define SRC_ADDR 0x1122334455667788  /* this is the address of the initiator */
#define DEST_ADDR 0x8877665544332211 /* this is the address of the responder */
#define DEST_PAN_ID 0x4321           /* this is the PAN ID used in this example */

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2          // sequence number byte index in MHR
#define RESP_MSG_POLL_RX_TS_IDX 0 // index in the MAC payload for Poll RX time
#define RESP_MSG_RESP_TX_TS_IDX 5 // index in the MAC payload for Response TX time
#define RESP_MSG_TS_LEN 4

/* Note, the key index of 0 is forbidden to send as key index. Thus index 1 is the first.
 * This example uses this index for the key table for the encryption of responder's data */
#define RESPONDER_KEY_INDEX 2

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 3000

mac_frame_802_15_4_format_t mac_frame;

static dwt_aes_config_t aes_config = {.key_load = AES_KEY_Load,           // load the key into the AES engine see Note 14 below
                                      .key_size = AES_KEY_128bit,         // use 128bit key
                                      .key_src = AES_KEY_Src_Register,    // the key source is IC registers
                                      .aes_core_type = AES_core_type_CCM, // Use CCM core
                                      .aes_key_otp_type = AES_key_RAM,
                                      .key_addr = 0};

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};
/* Optional keys according to the key index - In AUX security header*/
static const dwt_aes_key_t keys_options[NUM_OF_KEY_OPTIONS] = {{0x00010203, 0x04050607, 0x08090A0B, 0x0C0D0E0F, 0x00000000, 0x00000000, 0x00000000, 0x00000000},
                                                               {0x11223344, 0x55667788, 0x99AABBCC, 0xDDEEFF00, 0x00000000, 0x00000000, 0x00000000, 0x00000000},
                                                               {0xFFEEDDCC, 0xBBAA9988, 0x77665544, 0x33221100, 0x00000000, 0x00000000, 0x00000000, 0x00000000}};

/* Poll message from the initiator to the responder */
static uint8_t rx_poll_msg[] = {'P', 'o', 'l', 'l', ' ', 'm', 'e', 's', 's', 'a', 'g', 'e'};
/* Response message to the initiator. The first 8 bytes are used for Poll RX time and Response TX time.*/
static uint8_t tx_resp_msg[] = {0, 0, 0, 0, 0, 0, 0, 0, 'R', 'e', 's', 'p', 'o', 'n', 's', 'e'};

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code can handle. */
#define RX_BUF_LEN 127 /* The received frame cannot be bigger than 127 if STD PHR mode is used */
static uint8_t rx_buffer[RX_BUF_LEN];

/* Timestamps of frames transmission/reception. */
// static uint64_t poll_rx_ts;
// static uint64_t resp_tx_ts;

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 5 below. */
extern dwt_txconfig_t txconfig_options;

dwt_aes_job_t aes_job_rx, aes_job_tx;

dwt_mic_size_e dwt_mic_size_from_bytes(uint8_t mic_size_in_bytes);

void anthor_init_dw3000()
{
    dw3000_hw_init();
    dw3000_hw_reset();
    dw3000_hw_init_interrupt();
    dw3000_spi_speed_fast();

    while (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) == DWT_ERROR)
    {
        LOG_DBG("DEV Probe FAILED");
        k_sleep(K_SECONDS(1));
    }
    while (!dwt_checkidlerc())
        ;
    while (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        LOG_DBG("DEV INIT FAILED");
        k_sleep(K_SECONDS(1));
    }
    while (dwt_configure(&config))
    {
        LOG_DBG("DEV CONFIG FAILED");
        k_sleep(K_SECONDS(1));
    }

    LOG_DBG("DEV CONFIG SUCCEED");

    k_sleep(K_SECONDS(1));

    dwt_configuretxrf(&txconfig_options);

    dwt_setrxantennadelay(RX_ANT_DLY); // set RX antenna delay time
    dwt_settxantennadelay(TX_ANT_DLY); // set TX antenna delay time

    // dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);

    dwt_setinterrupt(DWT_INT_TXFRS_BIT_MASK | DWT_INT_RXFCG_BIT_MASK, 0, DWT_ENABLE_INT);

    dwt_setcallbacks(&anthor_tx_done_cb, &anthor_rx_ok_cb, &anthor_rx_timeout_cb, &anthor_rx_err_cb, &anthor_spi_err_cb, &anthor_spi_ready_cb, &anthor_dual_spi_cb);

    dw3000_hw_interrupt_enable();

    // LOG_DBG("check irq %d ", dwt_checkirq());
}

void anthor_tx_done_cb(const dwt_cb_data_t *cb_data)
{
    // dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    LOG_DBG("TX done");
    // dwt_setinterrupt(DWT_INT_TXFRS_BIT_MASK, 0, DWT_ENABLE_INT);
}

void anthor_rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    LOG_DBG("RX OK");

    /* Read data length that was received */
    uint16_t frame_len = cb_data->datalength;
    aes_config.mode = AES_Decrypt; /* configure for decryption*/
    PAYLOAD_PTR_802_15_4(&mac_frame) = rx_buffer;
    int8_t status = rx_aes_802_15_4(&mac_frame, frame_len, &aes_job_rx, sizeof(rx_buffer), keys_options, DEST_ADDR, SRC_ADDR, &aes_config);
    if (status != AES_RES_OK)
    {
        /* report any errors */
        switch (status)
        {
        case AES_RES_ERROR_LENGTH:
            LOG_DBG("AES length error");
            break;
        case AES_RES_ERROR:
            LOG_DBG("ERROR AES");
            break;
        case AES_RES_ERROR_FRAME:
            LOG_DBG("Error Frame");
            break;
        case AES_RES_ERROR_IGNORE_FRAME:
            LOG_DBG("wrong destination address");
            break; // Got frame with wrong destination address
        }
    }
    else
    {

        /* Retrieve poll reception timestamp. */
        uint64_t poll_rx_ts = get_rx_timestamp_u64();

        resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);

        anthor_tx_msg_work_t *msg_work = (anthor_tx_msg_work_t *)k_heap_alloc(&tx_msg_heap, sizeof(anthor_tx_msg_work_t), K_NO_WAIT);
        if (msg_work != NULL)
        {
            memcpy(msg_work->msg.data, tx_resp_msg, sizeof(tx_resp_msg));

            msg_work->msg.data_len = sizeof(tx_resp_msg);
            msg_work->dest_pan_id = DEST_PAN_ID;
            msg_work->dest_addr = DEST_ADDR;
            msg_work->src_addr = SRC_ADDR;

            k_work_init(&msg_work->work, anthor_tx_msg);
            k_work_submit(&msg_work->work);
        }
        else
        {
            LOG_DBG("heap alloc failed.");
        }
    }
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}
void anthor_rx_timeout_cb(const dwt_cb_data_t *cb_data) {}
void anthor_rx_err_cb(const dwt_cb_data_t *cb_data) {}
void anthor_spi_err_cb(const dwt_cb_data_t *cb_data) {}
void anthor_spi_ready_cb(const dwt_cb_data_t *cb_data) {}
void anthor_dual_spi_cb(const dwt_cb_data_t *cb_data) {}

void anthor_tx_msg(struct k_work *item)
{
    uint8_t nonce[13];

    anthor_tx_msg_work_t *msg_work = CONTAINER_OF(item, anthor_tx_msg_work_t, work);

    LOG_DBG("anthor tx msg work");

    /* Program the correct key to be used */
    dwt_set_keyreg_128(&keys_options[RESPONDER_KEY_INDEX - 1]);
    /* Set the key index for the frame */
    MAC_FRAME_AUX_KEY_IDENTIFY_802_15_4(&mac_frame) = RESPONDER_KEY_INDEX;

    /* Increment the sequence number */
    MAC_FRAME_SEQ_NUM_802_15_4(&mac_frame)
    ++;

    /* Update the frame count */
    mac_frame_update_aux_frame_cnt(&mac_frame, mac_frame_get_aux_frame_cnt(&mac_frame) + 1);

    /* Configure the AES job */
    aes_job_tx.mode = AES_Encrypt;        /* this is encyption job */
    aes_job_tx.src_port = AES_Src_Tx_buf; /* dwt_do_aes will take plain text to the TX buffer */
    aes_job_tx.dst_port = AES_Dst_Tx_buf; /* dwt_do_aes will replace the original plain text TX buffer with encrypted one */
    aes_job_tx.header_len = MAC_FRAME_HEADER_SIZE(&mac_frame);
    aes_job_tx.header = (uint8_t *)MHR_802_15_4_PTR(&mac_frame); /* plain-text header which will not be encrypted */
    aes_job_tx.payload = msg_work->msg.data;                     /* payload to be sent */
    aes_job_tx.payload_len = msg_work->msg.data_len;             /* payload length */
    aes_job_tx.mic_size = mac_frame_get_aux_mic_size(&mac_frame);
    aes_job_tx.nonce = nonce; /* set below once MHR is set*/
    
    aes_config.mode = AES_Encrypt;
    aes_config.mic = dwt_mic_size_from_bytes(aes_job_tx.mic_size);
    dwt_configure_aes(&aes_config);

    /* Update the MHR (reusing the received MHR, thus need to swap SRC/DEST addresses */
    mac_frame_set_pan_ids_and_addresses_802_15_4(&mac_frame,
                                                 msg_work->dest_pan_id,
                                                 msg_work->dest_addr,
                                                 msg_work->src_addr);

    /* construct the nonce from the MHR */
    mac_frame_get_nonce(&mac_frame, nonce);
    /* perform the encryption, the TX buffer will contain a full MAC frame with encrypted payload */

    uint32_t resp_tx_time = (get_sys_timestamp_u64() + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);
    uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
    resp_msg_set_ts(&msg_work->msg.data[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

    int8_t status = dwt_do_aes(&aes_job_tx, aes_config.aes_core_type);
    if (status < 0)
    {
        LOG_DBG("AES length error");
    }
    else if (status & DWT_AES_ERRORS)
    {
        LOG_DBG("ERROR AES");
    }
    else
    {
        /* configure the frame control and start transmission */
        dwt_writetxfctrl(aes_job_tx.header_len + aes_job_tx.payload_len + aes_job_tx.mic_size + FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging. */
        dwt_forcetrxoff();
        int ret = dwt_starttx(DWT_START_TX_DELAYED);

        /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 10 below. */
        if (ret == DWT_SUCCESS)
        {
            /* Poll DW IC until TX frame sent event set. See NOTE 6 below. */
            waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);

            /* Clear TXFRS event. */
            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
            LOG_DBG("TX SUCCESS");
        }
        else
        {
            LOG_DBG("TX ERROR");
        }
    }
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    k_heap_free(&tx_msg_heap, msg_work);
    // free(msg_work);
}
void anthor_rx_msg(struct k_work *item){

}
void anthor_responder(void *p1, void *p2, void *p3)
{
    LOG_DBG(APP_NAME);

    /*Configure the TX and RX AES jobs, the TX job is used to encrypt the Response message,
     * the RX job is used to decrypt the Poll message */
    aes_job_rx.mode = AES_Decrypt; /* Mode is set to decryption */
    aes_job_rx.src_port = AES_Src_Rx_buf_0; /* Take encrypted frame from the RX buffer */
    aes_job_rx.dst_port = AES_Dst_Rx_buf_0; /* Decrypt the frame to the same RX buffer : this will destroy original RX frame */
    aes_job_rx.header_len = MAC_FRAME_HEADER_SIZE(&mac_frame);  /* Set the header length (mac_frame contains the MAC header) */
    aes_job_rx.header = (uint8_t *)MHR_802_15_4_PTR(&mac_frame); /* Set the pointer to plain-text header which will not be encrypted */
    aes_job_rx.payload = rx_buffer; /* the decrypted RX MAC frame payload will be read out of IC into this buffer */

    /* Activate reception immediately. */
    dwt_forcetrxoff();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    LOG_DBG("RX START");

    while (1)
    {
        dwt_forcetrxoff();
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        k_sleep(K_SECONDS(10));
    }
}

#endif