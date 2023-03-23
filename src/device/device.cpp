#include "device/device.hpp"

LOG_MODULE_REGISTER(device, LOG_LEVEL_DBG);

Device *Device::device_ptr = nullptr;

Device::Device()
{
    device_ptr = this;
    {
        dw3000_hw_init();
        dw3000_hw_reset();
        dw3000_hw_init_interrupt();
        dw3000_spi_speed_fast();

        if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) == DWT_ERROR)
        {
            LOG_DBG("DEV Probe FAILED");
        }
        while (!dwt_checkidlerc())
            ;
        if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
        {
            LOG_DBG("DEV INIT FAILED");
        }
        if (dwt_configure(&config))
        {
            LOG_DBG("DEV CONFIG FAILED");
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
    }
    // config
    {
        // dwt config
        {
            config.chan = 5;              // Channel number.
    config.txPreambLength = DWT_PLEN_128; // Preamble length. Used in TX only.
    config.rxPAC = DWT_PAC8;              // Preamble acquisition chunk size. Used in RX only.
    config.txCode = 9;                    // TX preamble code. Used in TX only.
    config.rxCode = 9;                    // RX preamble code. Used in RX only.
    config.sfdType = DWT_SFD_DW_8;        // use non-standard 8 symbol
    config.dataRate = DWT_BR_6M8;         // Data rate.
    config.phrMode = DWT_PHRMODE_STD;     // PHY header mode.
    config.phrRate = DWT_PHRRATE_STD;     // PHY header rate.
    config.sfdTO = (129 + 8 - 8);         // SFD timeout Used in RX only. (preamble length + 1 + SFD length - PAC size).
    config.stsMode = DWT_STS_MODE_OFF;    // STS disabled
    config.stsLength = DWT_STS_LEN_64;    // STS length see allowed values in Enum dwt_sts_lengths_e
    config.pdoaMode = DWT_PDOA_M0;        // PDOA mode off

    dwt_set_keyreg_128(&keys_options[KEY_INDEX - 1]); // set aes key
}
// 802_15_4 frame
{
    mac_frame.mhr_802_15_4.frame_ctrl[0] = 0x09;
    mac_frame.mhr_802_15_4.frame_ctrl[1] = 0xEC;
    mac_frame.mhr_802_15_4.sequence_num = 0x00;
    // mac_frame.mhr_802_15_4.dest_pan_id
    // mac_frame.mhr_802_15_4.dest_addr
    // mac_frame.mhr_802_15_4.src_addr
    mac_frame.mhr_802_15_4.aux_security.security_ctrl = 0x0f;
    // mac_frame.mhr_802_15_4.aux_security.frame_counter
    mac_frame.mhr_802_15_4.aux_security.key_indentifier = KEY_INDEX;
    // mac_frame.payload_ptr
}
// aes config
{
    aes_config.aes_otp_sel_key_block = AES_key_otp_sel_1st_128;
    aes_config.aes_key_otp_type = AES_key_RAM;
    aes_config.aes_core_type = AES_core_type_CCM;
    // aes_config.mic
    aes_config.key_src = AES_KEY_Src_Register;
    aes_config.key_load = AES_KEY_Load;
    aes_config.key_addr = 0;
    aes_config.key_size = AES_KEY_128bit;
    // aes_config.mode
}
{
    // aes_job_rx.nonce
    // aes_job_rx.payload = rx_buffer;
    aes_job_rx.header = (uint8_t *)MHR_802_15_4_PTR(&mac_frame);
    aes_job_rx.header_len = MAC_FRAME_HEADER_SIZE(&mac_frame);
    // aes_job_rx.payload_len
    aes_job_rx.src_port = AES_Src_Rx_buf_0;
    aes_job_rx.dst_port = AES_Dst_Rx_buf_0;
    aes_job_rx.mode = AES_Decrypt;
    // aes_job_rx.mic_size
}
{
    // aes_job_tx.nonce
    // aes_job_tx.payload
    aes_job_tx.header = (uint8_t *)MHR_802_15_4_PTR(&mac_frame);
    aes_job_tx.header_len = MAC_FRAME_HEADER_SIZE(&mac_frame);
    // aes_job_tx.payload_len
    aes_job_tx.src_port = AES_Src_Tx_buf;
    aes_job_tx.dst_port = AES_Dst_Tx_buf;
    aes_job_tx.mode = AES_Encrypt;
    // aes_job_rx.mic_size
}
}

{ // interrupt
    dwt_setinterrupt(DWT_INT_TXFRS_BIT_MASK | DWT_INT_RXFCG_BIT_MASK, 0, DWT_ENABLE_INT);

    dwt_setcallbacks(
        &Device::tx_done_cb,
        &Device::rx_ok_cb,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr);

    dw3000_hw_interrupt_enable();
}
}

void Device::app(void *, void *, void *)
{
}

uint64_t Device::tx_msg(uint8_t *msg, uint16_t len, uint64_t dest_addr, uint8_t mode)
{
    // dwt_set_keyreg_128(&keys_options[KEY_INDEX - 1]); // set aes key
    mac_frame_set_pan_ids_and_addresses_802_15_4(&mac_frame, PAN_ID, dest_addr, DEVICE_ADDR);
    if (mode == DWT_START_TX_DELAYED)
    {
        uint32_t resp_tx_time = (get_sys_timestamp_u64() + (TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
        dwt_setdelayedtrxtime(resp_tx_time);
        uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
        set_msg_dly_ts(msg, len, resp_tx_ts);
    }
    {
        uint8_t nonce[13]; // 13-byte nonce used in this example as per IEEE802.15.4.
        mac_frame_get_nonce(&mac_frame, nonce);

        aes_job_tx.nonce = nonce;     // pointer to the nonce structure
        aes_job_tx.payload = msg;     // payload to be encrypted
        aes_job_tx.payload_len = len; // size of payload to be encrypted
        aes_job_tx.mic_size = mac_frame_get_aux_mic_size(&mac_frame);

        aes_config.mode = AES_Encrypt;
        aes_config.mic = dwt_mic_size_from_bytes(mac_frame_get_aux_mic_size(&mac_frame));
        dwt_configure_aes(&aes_config);
    }

    int8_t status = dwt_do_aes(&aes_job_tx, aes_config.aes_core_type);
    if (status < 0)
    {
        LOG_DBG("TX: AES length error");
    }
    else if (status & DWT_AES_ERRORS)
    {
        LOG_DBG("TX: ERROR AES");
    }
    else
    {
        dwt_writetxfctrl(aes_job_tx.header_len + aes_job_tx.payload_len + aes_job_tx.mic_size + FCS_LEN, 0, 1);
        LOG_DBG("start TX");
        dwt_forcetrxoff();
        if (mode == DWT_START_TX_IMMEDIATE)
        {
            if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS)
            {
                waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);
                dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
                LOG_DBG("TX SUCCESS");
            }
                        else
            {
                LOG_DBG("TX ERROR");
            }
        }
        if (mode == DWT_START_TX_DELAYED)
        {
            if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS)
            {
                waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);
                dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
                LOG_DBG("TX SUCCESS");
            }
            else
            {
                LOG_DBG("TX ERROR");
            }
        }
    }
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    
    mac_frame.mhr_802_15_4.sequence_num++;
    mac_frame_update_aux_frame_cnt(&mac_frame, mac_frame_get_aux_frame_cnt(&mac_frame) + 1);

    return get_tx_timestamp_u64();
}

void Device::set_msg_dly_ts(uint8_t *msg, uint16_t len, uint64_t ts)
{
    uint8_t msg_type = msg[0];

    switch (msg_type)
    {
    case MSG_RESPONDER:
        msg::msg_responder *m = (msg::msg_responder *)msg;
        m->resp_tx_ts = ts;
        break;
    }
}

void Device::tx_done_cb(const dwt_cb_data_t *cb_data)
{
    LOG_DBG("TX done");
}

void Device::rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    uint16_t frame_len = cb_data->datalength;

    device_ptr->aes_config.mode = AES_Decrypt;

    device_ptr->aes_job_rx.payload = device_ptr->rx_buffer;
    PAYLOAD_PTR_802_15_4(&device_ptr->mac_frame) = device_ptr->rx_buffer;

    uint64_t src_addr;
    uint64_t dst_addr;
    int16_t msg_len;
    int8_t status = rx_aes_802_15_4(&device_ptr->mac_frame,
                                    frame_len,
                                    &device_ptr->aes_job_rx,
                                    sizeof(device_ptr->rx_buffer),
                                    keys_options,
                                    &src_addr,
                                    &dst_addr,
                                    &device_ptr->aes_config,
                                    &msg_len);
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
        }
    }
    else if (dst_addr == DEVICE_ADDR || dst_addr == BROADCAST_ADDR)
    {
        uint8_t *msg = device_ptr->rx_buffer;
        device_ptr->msg_process_cb(msg, msg_len, src_addr, dst_addr, get_rx_timestamp_u64());
    }
    else
    {
        LOG_DBG("wrong destination address");
    }
}

void Device::msg_process_cb(uint8_t *msg, uint16_t msg_len, uint64_t src_addr, uint64_t dst_addr, uint64_t rx_ts)
{
    LOG_DBG("Rx msg from %016llx to %016llx (len %d)", src_addr, dst_addr, msg_len);
}