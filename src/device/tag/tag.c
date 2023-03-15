#include <device/tag/tag.h>

#if defined(DEVICE_TAG)

LOG_MODULE_REGISTER(device_tag, LOG_LEVEL_DBG);

/* application name */
#define APP_NAME "TAG v0.0"

dwt_mic_size_e dwt_mic_size_from_bytes(uint8_t mic_size_in_bytes);

/* Initiator data */
#define DEST_ADDR 0x1122334455667788 /* this is the address of the responder */
#define SRC_ADDR 0x8877665544332211  /* this is the address of the initiator */
#define DEST_PAN_ID 0x4321           /* this is the PAN ID used in this example */

/* Note, the key index of 0 is forbidden to send as key index. Thus index 1 is the first.
 * This example uses this index for the key table for the encryption of initiator's data */
#define INITIATOR_KEY_INDEX 1

/* Delay between frames, in UWB microseconds. */
// #define POLL_TX_TO_RESP_RX_DLY_UUS (1320 + CPU_PROCESSING_TIME)
#define POLL_TX_TO_RESP_RX_DLY_UUS 0

/* Receive response timeout. 0 is disable*/
#define RESP_RX_TIMEOUT_UUS 0

/* Default antenna delay values for 64 MHz PRF.*/
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

#define START_RECEIVE_DATA_LOCATION 8 // MAC payload user data starts at index 8 (e.g. 'R' - in above response message)

/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2          // sequence number byte index in MHR
#define RESP_MSG_POLL_RX_TS_IDX 0 // index in the MAC payload for Poll RX time
#define RESP_MSG_RESP_TX_TS_IDX 5 // index in the MAC payload for Response TX time
#define RESP_MSG_TS_LEN 4

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code can handle. */
#define RX_BUF_LEN 127 /* The received frame cannot be bigger than 127 if STD PHR mode is used */
static uint8_t rx_buffer[RX_BUF_LEN];

/*802_15_4 frame*/
mac_frame_802_15_4_format_t mac_frame = {
    .mhr_802_15_4.frame_ctrl[0] = 0x09,
    .mhr_802_15_4.frame_ctrl[1] = 0xEC,
    .mhr_802_15_4.sequence_num = 0x00,
    .mhr_802_15_4.dest_pan_id[0] = 0x21,
    .mhr_802_15_4.dest_pan_id[1] = 0x43,
    .mhr_802_15_4.aux_security.security_ctrl = 0x0F,
};
static dwt_aes_config_t aes_config = {.key_load = AES_KEY_Load,           // load the key into AES engine see Note 15 below
                                      .key_size = AES_KEY_128bit,         // use 128bit key
                                      .key_src = AES_KEY_Src_Register,    // the key source is IC registers
                                      .aes_core_type = AES_core_type_CCM, // Use CCM core
                                      .aes_key_otp_type = AES_key_RAM,
                                      .key_addr = 0};

/* Optional keys according to the key index - In AUX security header*/
static const dwt_aes_key_t keys_options[NUM_OF_KEY_OPTIONS] = {{0x00010203, 0x04050607, 0x08090A0B, 0x0C0D0E0F, 0x00000000, 0x00000000, 0x00000000, 0x00000000},
                                                               {0x11223344, 0x55667788, 0x99AABBCC, 0xDDEEFF00, 0x00000000, 0x00000000, 0x00000000, 0x00000000},
                                                               {0xFFEEDDCC, 0xBBAA9988, 0x77665544, 0x33221100, 0x00000000, 0x00000000, 0x00000000, 0x00000000}};

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

/* MAC payload data of the frames used in the ranging process. See NOTE 3 below. */
/* Poll message from the initiator to the responder */
static uint8_t tx_poll_msg[] = {'P', 'o', 'l', 'l', ' ', 'm', 'e', 's', 's', 'a', 'g', 'e'};
/* Response message to the initiator. The first 8 bytes are used for Poll RX time and Response TX time.*/
static uint8_t rx_resp_msg[] = {0, 0, 0, 0, 0, 0, 0, 0, 'R', 'e', 's', 'p', 'o', 'n', 's', 'e'};

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 2 below. */
extern dwt_txconfig_t txconfig_options;

dwt_aes_job_t aes_job_tx, aes_job_rx;

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;
void tag_init_dw3000()
{
    dw3000_hw_init();
    dw3000_hw_reset();
    dw3000_hw_init_interrupt();

    dw3000_spi_speed_fast();

    if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) == DWT_ERROR)
    {
        LOG_DBG("DEV Probe FAILED");
        while (1)
            k_sleep(K_SECONDS(1));
    }
    while (!dwt_checkidlerc())
        ;
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        LOG_DBG("DEV INIT FAILED");
        while (1)
            k_sleep(K_SECONDS(1));
    }
    if (dwt_configure(&config))
    {
        LOG_DBG("DEV CONFIG FAILED");
        while (1)
            k_sleep(K_SECONDS(1));
    }
    else
        LOG_DBG("DEV CONFIG SUCCEED");

    k_sleep(K_SECONDS(1));

    dwt_configuretxrf(&txconfig_options);

    dwt_setrxantennadelay(RX_ANT_DLY); // set RX antenna delay time
    dwt_settxantennadelay(TX_ANT_DLY); // set TX antenna delay time

    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);

    // dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    dwt_setinterrupt(DWT_INT_TXFRS_BIT_MASK|DWT_INT_RXFCG_BIT_MASK, 0, DWT_ENABLE_INT);

    dwt_setcallbacks(&tag_tx_done_cb, &tag_rx_ok_cb, &tag_rx_timeout_cb, &tag_rx_err_cb, &tag_spi_err_cb, &tag_spi_ready_cb, &tag_dual_spi_cb);
    dw3000_hw_interrupt_enable();

    // LOG_DBG("check irq %d ", dwt_checkirq());
}
void tag_tx_done_cb(const dwt_cb_data_t *cb_data)
{
    // dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    LOG_DBG("TX done");
    // dwt_setinterrupt(DWT_INT_TXFRS_BIT_MASK, 0, DWT_ENABLE_INT);
}
void tag_rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    /* Clear good RX frame event in the DW IC status register. */
    // dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

    LOG_DBG("RX OK");

    /* Read data length that was received */
    uint16_t frame_len = cb_data->datalength; // in interrupts must use cb_data->datalength

    aes_config.mode = AES_Decrypt;
    aes_config.key_load = AES_KEY_Load;
    PAYLOAD_PTR_802_15_4(&mac_frame) = rx_buffer; /* Set the MAC pyload ptr */

    /* This example assumes that initiator and responder are sending encrypted data */
    int8_t status = rx_aes_802_15_4(&mac_frame, frame_len, &aes_job_rx, sizeof(rx_buffer), keys_options, DEST_ADDR, SRC_ADDR, &aes_config);
    if (status == AES_RES_OK)
    {
        /* Check that the frame is the expected response from the companion "SS TWR AES responder" example.
         * ignore the 8 first bytes of the response message as they contain the poll and response timestamps */
        // if (memcmp(&rx_buffer[START_RECEIVE_DATA_LOCATION], &rx_resp_msg[START_RECEIVE_DATA_LOCATION], aes_job_rx.payload_len - START_RECEIVE_DATA_LOCATION) == 0)
        // {
            uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
            int32_t rtd_init, rtd_resp;
            float clockOffsetRatio;

            /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
            poll_tx_ts = dwt_readtxtimestamplo32();
            resp_rx_ts = dwt_readrxtimestamplo32();

            /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
            clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

            /* Get timestamps embedded in response message. */
            resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
            resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

            /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
            rtd_init = resp_rx_ts - poll_tx_ts;
            rtd_resp = resp_tx_ts - poll_rx_ts;

            tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
            distance = tof * SPEED_OF_LIGHT;
            LOG_DBG("distance: %f", distance);
        // }
    }
    else
    {
        LOG_DBG("AER RES NOT OK");
        switch (status)
        {
        case AES_RES_ERROR_LENGTH:
            LOG_DBG("Length AES error");
            break;
        case AES_RES_ERROR:
            LOG_DBG("ERROR AES");
            break;
        case AES_RES_ERROR_FRAME:
            LOG_DBG("Error Frame");
            break;
        case AES_RES_ERROR_IGNORE_FRAME:
            break; // Got frame not for us
        }
    }
    // dwt_setinterrupt(DWT_INT_RXFCG_BIT_MASK, 0, DWT_ENABLE_INT);
}
void tag_rx_timeout_cb(const dwt_cb_data_t *cb_data)
{
}
void tag_rx_err_cb(const dwt_cb_data_t *cb_data)
{
}
void tag_spi_err_cb(const dwt_cb_data_t *cb_data)
{
}
void tag_spi_ready_cb(const dwt_cb_data_t *cb_data)
{
}
void tag_dual_spi_cb(const dwt_cb_data_t *cb_data)
{
}

void tag_initiator(void *p1, void *p2, void *p3)
{
    static uint32_t frame_cnt = 0; /* See Note 13 */
    static uint8_t seq_cnt = 0x0A; /* Frame sequence number, incremented after each transmission. */
    uint8_t nonce[13];             /* 13-byte nonce used in this example as per IEEE802.15.4 */
    dwt_aes_job_t aes_job_tx;

    LOG_DBG(APP_NAME);


    /*Configure the TX and RX AES jobs, the TX job is used to encrypt the Poll message,
     * the RX job is used to decrypt the Response message */
    aes_job_tx.mode = AES_Encrypt;                               /* this is encryption job */
    aes_job_tx.src_port = AES_Src_Tx_buf;                        /* dwt_do_aes will take plain text to the TX buffer */
    aes_job_tx.dst_port = AES_Dst_Tx_buf;                        /* dwt_do_aes will replace the original plain text TX buffer with encrypted one */
    aes_job_tx.nonce = nonce;                                    /* pointer to the nonce structure*/
    aes_job_tx.header = (uint8_t *)MHR_802_15_4_PTR(&mac_frame); /* plain-text header which will not be encrypted */
    aes_job_tx.header_len = MAC_FRAME_HEADER_SIZE(&mac_frame);
    aes_job_tx.payload = tx_poll_msg;             /* payload to be encrypted */
    aes_job_tx.payload_len = sizeof(tx_poll_msg); /* size of payload to be encrypted */

    aes_job_rx.mode = AES_Decrypt;          /* this is decryption job */
    aes_job_rx.src_port = AES_Src_Rx_buf_0; /* The source of the data to be decrypted is the IC RX buffer */
    aes_job_rx.dst_port = AES_Dst_Rx_buf_0; /* Decrypt the encrypted data to the IC RX buffer : this will destroy original RX frame */
    aes_job_rx.header_len = MAC_FRAME_HEADER_SIZE(&mac_frame);
    aes_job_rx.header = (uint8_t *)MHR_802_15_4_PTR(&mac_frame); /* plain-text header which will not be encrypted */
    aes_job_rx.payload = rx_buffer;                              /* pointer to where the decrypted data will be copied to when read from the IC*/

    while (1)
    {
        /* Program the correct key to be used */
        dwt_set_keyreg_128(&keys_options[INITIATOR_KEY_INDEX - 1]);
        /* Set the key index for the frame */
        MAC_FRAME_AUX_KEY_IDENTIFY_802_15_4(&mac_frame) = INITIATOR_KEY_INDEX;

        /* Update MHR to the correct SRC and DEST addresses and construct the 13-byte nonce
         * (same MAC frame structure is used to store both received data and transmitted data - thus SRC and DEST addresses
         * need to be updated before each transmission */
        mac_frame_set_pan_ids_and_addresses_802_15_4(&mac_frame, DEST_PAN_ID, DEST_ADDR, SRC_ADDR);
        mac_frame_get_nonce(&mac_frame, nonce);

        aes_job_tx.mic_size = mac_frame_get_aux_mic_size(&mac_frame);
        aes_config.mode = AES_Encrypt;
        aes_config.mic = dwt_mic_size_from_bytes(aes_job_tx.mic_size);
        dwt_configure_aes(&aes_config);

        /* The AES job will take the TX frame data and and copy it to DW IC TX buffer before transmission. */
        int8_t status;
        status = dwt_do_aes(&aes_job_tx, aes_config.aes_core_type);
        /* Check for errors */
        if (status < 0)
        {
            LOG_DBG("AES length error"); /* Error */
        }
        else if (status & DWT_AES_ERRORS)
        {
            LOG_DBG("ERROR AES"); /* Error */
        }
        else
        {
            /* configure the frame control and start transmission */
            dwt_writetxfctrl(aes_job_tx.header_len + aes_job_tx.payload_len + aes_job_tx.mic_size + FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging. */
            /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
             * set by dwt_setrxaftertxdelay() has elapsed. */

            LOG_DBG("start TX");
            dwt_forcetrxoff();
            dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

            // dwt_setinterrupt(DWT_INT_TXFRS_BIT_MASK | DWT_INT_RXFCG_BIT_MASK, 0, DWT_ENABLE_INT);

            k_sleep(K_SECONDS(10));
        }

        MAC_FRAME_SEQ_NUM_802_15_4(&mac_frame) = ++seq_cnt;
        mac_frame_update_aux_frame_cnt(&mac_frame, ++frame_cnt);
    }
}

#endif
