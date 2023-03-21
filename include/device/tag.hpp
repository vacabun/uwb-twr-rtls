#ifndef _TAG_HPP_
#define _TAG_HPP_

#if defined(DEVICE_TAG)

#include "device/device.hpp"
#include <functional>

// #define DEST_ADDR 0x1122334455667788 /* this is the address of the responder */
#define DEVICE_ADDR 0x8877665544332211  /* this is the address of the initiator */
#define PAN_ID 0x4321           /* this is the PAN ID used in this example */

class Tag : public Device
{
public:
    Tag();
    void app(void *p1, void *p2, void *p3);
    void set_callback();
    void tx_msg(msg::msg_initiator msg, uint64_t dest_addr);
    std::function<void(const dwt_cb_data_t *)> tx_done_cb_func;
    std::function<void(const dwt_cb_data_t *)> rx_ok_cb_func;
    void tx_done_cb(const dwt_cb_data_t *cb_data);
    void rx_ok_cb(const dwt_cb_data_t *cb_data);
    uint32_t frame_cnt;
    uint8_t seq_cnt;   // Frame sequence number, incremented after each transmission.
    uint8_t nonce[13]; // 13-byte nonce used in this example as per IEEE802.15.4.
    dwt_aes_job_t aes_job_tx;
    dwt_aes_job_t aes_job_rx;
    mac_frame_802_15_4_format_t mac_frame;
    dwt_aes_config_t aes_config;
    uint8_t rx_buffer[RX_BUF_LEN];
};

#endif

#endif
