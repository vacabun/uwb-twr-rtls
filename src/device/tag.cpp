#include <device/tag.hpp>

LOG_MODULE_REGISTER(tag, LOG_LEVEL_DBG);

Tag::Tag()
{
    frame_cnt = 0;
    seq_cnt = 0x0A;

    /*802_15_4 frame*/
    {
        mac_frame.mhr_802_15_4.frame_ctrl[0] = 0x09;
        mac_frame.mhr_802_15_4.frame_ctrl[1] = 0xEC;
        mac_frame.mhr_802_15_4.sequence_num = 0x00;
        mac_frame.mhr_802_15_4.dest_pan_id[0] = 0x21;
        mac_frame.mhr_802_15_4.dest_pan_id[0] = 0x43;
        mac_frame.mhr_802_15_4.aux_security.security_ctrl = 0x0f;
    }
    {
        aes_config.key_load = AES_KEY_Load;
        aes_config.key_size = AES_KEY_128bit;         // use 128bit key
        aes_config.key_src = AES_KEY_Src_Register;    // the key source is IC registers
        aes_config.aes_core_type = AES_core_type_CCM; // Use CCM core
        aes_config.aes_key_otp_type = AES_key_RAM;
        aes_config.key_addr = 0;
    }
    /*Configure the TX and RX AES jobs, the TX job is used to encrypt the Poll message, the RX job is used to decrypt the Response message */
    {
        aes_job_tx.mode = AES_Encrypt;                               // this is encryption job
        aes_job_tx.src_port = AES_Src_Tx_buf;                        // dwt_do_aes will take plain text to the TX buffer
        aes_job_tx.dst_port = AES_Dst_Tx_buf;                        // dwt_do_aes will replace the original plain text TX buffer with encrypted one
        aes_job_tx.nonce = nonce;                                    // pointer to the nonce structure
        aes_job_tx.header = (uint8_t *)MHR_802_15_4_PTR(&mac_frame); // plain-text header which will not be encrypted
        aes_job_tx.header_len = MAC_FRAME_HEADER_SIZE(&mac_frame);   //
        // aes_job_tx.payload = tx_poll_msg;                            // payload to be encrypted
        // aes_job_tx.payload_len = sizeof(tx_poll_msg);                // size of payload to be encrypted
    }
    {
        aes_job_rx.mode = AES_Decrypt;                               // this is decryption job
        aes_job_rx.src_port = AES_Src_Rx_buf_0;                      // The source of the data to be decrypted is the IC RX buffer
        aes_job_rx.dst_port = AES_Dst_Rx_buf_0;                      // Decrypt the encrypted data to the IC RX buffer : this will destroy original RX frame
        aes_job_rx.header_len = MAC_FRAME_HEADER_SIZE(&mac_frame);   //
        aes_job_rx.header = (uint8_t *)MHR_802_15_4_PTR(&mac_frame); // plain-text header which will not be encrypted
        aes_job_rx.payload = rx_buffer;                              // pointer to where the decrypted data will be copied to when read from the IC
    }

     this->set_callback();

    dw3000_hw_interrupt_enable();
}
void Tag::tx_msg(msg::msg_initiator msg, uint64_t dest_addr)
{
    {
        aes_job_tx.payload = (uint8_t *)&msg; // payload to be encrypted
        aes_job_tx.payload_len = sizeof(msg); // size of payload to be encrypted
    }

    // LOG_DBG("msg size: %d", sizeof(msg));

    dwt_set_keyreg_128(&keys_options[KEY_INDEX - 1]);
    MAC_FRAME_AUX_KEY_IDENTIFY_802_15_4(&mac_frame) = KEY_INDEX;

    mac_frame_set_pan_ids_and_addresses_802_15_4(&mac_frame, PAN_ID, dest_addr, DEVICE_ADDR);

    {
        mac_frame_get_nonce(&mac_frame, nonce);
        aes_job_tx.mic_size = mac_frame_get_aux_mic_size(&mac_frame);
        aes_config.mode = AES_Encrypt;
        aes_config.mic = dwt_mic_size_from_bytes(aes_job_tx.mic_size);
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
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    }

    MAC_FRAME_SEQ_NUM_802_15_4(&mac_frame) = ++seq_cnt;
    mac_frame_update_aux_frame_cnt(&mac_frame, ++frame_cnt);
}
void Tag::app(void *p1, void *p2, void *p3)
{
    while (1)
    {
        { // app
            msg::msg_initiator msg;
            tx_msg(msg, 0x1122334455667788);
        }
        k_sleep(K_SECONDS(10));
    }
}
void tx_done(const dwt_cb_data_t *cb_data)
{
    LOG_DBG("TX done");
}
void Tag::set_callback()
{

    tx_done_cb_func = [&](const dwt_cb_data_t *cb_data) -> void
    {
        this->tx_done_cb(cb_data);
    };
    rx_ok_cb_func = [&](const dwt_cb_data_t *cb_data) -> void
    {
        this->rx_ok_cb(cb_data);
    };
    dwt_setcallbacks(
        // tx_done_cb_func.target<void(const dwt_cb_data_t *)>(),
        &tx_done,
        rx_ok_cb_func.target<void(const dwt_cb_data_t *)>(),
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr);
}

void Tag::tx_done_cb(const dwt_cb_data_t *cb_data)
{
    LOG_DBG("TX done");
}
void Tag::rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    LOG_DBG("RX OK");
    uint16_t frame_len = cb_data->datalength;
    aes_config.mode = AES_Decrypt;
    aes_config.key_load = AES_KEY_Load;
    PAYLOAD_PTR_802_15_4(&mac_frame) = rx_buffer;
    uint64_t src_addr;
    int8_t status = rx_aes_802_15_4(&mac_frame, frame_len, &aes_job_rx, sizeof(rx_buffer), keys_options, &src_addr, DEVICE_ADDR, &aes_config);
    if (status == AES_RES_OK)
    {
        LOG_DBG("msg source address: %lld", src_addr);

        uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
        int32_t rtd_init, rtd_resp;
        float clockOffsetRatio;

        /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
        poll_tx_ts = dwt_readtxtimestamplo32();
        resp_rx_ts = dwt_readrxtimestamplo32();

        /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

        // /* Get timestamps embedded in response message. */
        // resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
        // resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

        /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
        rtd_init = resp_rx_ts - poll_tx_ts;
        rtd_resp = resp_tx_ts - poll_rx_ts;

        double tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        double distance = tof * SPEED_OF_LIGHT;
        LOG_DBG("distance: %f", distance);
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
}
