#ifndef _DEVICE_HPP_
#define _DEVICE_HPP_

#include <functional>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#ifdef __cplusplus
extern "C"
{
#endif
#include "dw3000.h"
#ifdef __cplusplus
}
#endif
#include <shared_data/config_options.h>
#include <shared_data/shared_defines.h>
#include <shared_data/shared_functions.h>
#include <network/MAC_802_15_4/mac_802_15_4.hpp>
#include <network/MAC_802_15_4/key.hpp>
#include "msg/msg_initiator.hpp"

#ifdef __cplusplus
extern "C"
{
#endif
    dwt_mic_size_e dwt_mic_size_from_bytes(uint8_t mic_size_in_bytes);
#ifdef __cplusplus
}
#endif
extern dwt_txconfig_t txconfig_options;

/* Default antenna delay values for 64 MHz PRF.*/
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Buffer to store received response message.
   The received frame cannot be bigger than 127 if STD PHR mode is used */
#define RX_BUF_LEN 127

/* Note, the key index of 0 is forbidden to send as key index. Thus index 1 is the first. */
#define KEY_INDEX 1

#define BROADCAST_ADDR 0x0000000000000000
#define PAN_ID 0x4321 /* this is the PAN ID used in this example */

#if defined(DEVICE_TAG)
#define DEVICE_ADDR 0x8877665544332211 /* this is the address of the initiator */
#endif

#if defined(DEVICE_ANCHOR)
#define DEVICE_ADDR 0x1122334455667788 /* this is the address of the initiator */
#endif

typedef void (*FuncPtr)(uint8_t *, int16_t, uint64_t, uint64_t);

class Device
{
private:
public:
    /* data */
    static Device *device_ptr;
    dwt_config_t config;
    dwt_aes_config_t aes_config;

    dwt_aes_job_t aes_job_tx;
    dwt_aes_job_t aes_job_rx;
    mac_frame_802_15_4_format_t mac_frame;

    uint32_t frame_cnt;
    uint8_t seq_cnt; // Frame sequence number, incremented after each transmission.

    uint8_t rx_buffer[RX_BUF_LEN];

    Device();
    void app(void *, void *, void *);
    void tx_msg(uint8_t *msg, uint16_t len, uint64_t dest_addr);
    static void tx_done_cb(const dwt_cb_data_t *cb_data);
    static void rx_ok_cb(const dwt_cb_data_t *cb_data);
    virtual void msg_process_cb(uint8_t *msg, int16_t msg_len, uint64_t src_addr, uint64_t dst_addr);
};

#endif