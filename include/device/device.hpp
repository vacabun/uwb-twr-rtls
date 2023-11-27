#ifndef _DEVICE_HPP_
#define _DEVICE_HPP_

#include <functional>
#include <string>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

#ifdef CONFIG_DW3000

#include "dw3000.h"
dwt_mic_size_e dwt_mic_size_from_bytes(uint8_t mic_size_in_bytes);
extern dwt_txconfig_t txconfig_options;
/* Default antenna delay values for 64 MHz PRF.*/
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
/* Delay tx frames, in UWB microseconds.*/
#define TX_DLY_UUS 4000
/* Buffer to store received response message.
   The received frame cannot be bigger than 127 if STD PHR mode is used */
#define RX_BUF_LEN 127
/* Note, the key index of 0 is forbidden to send as key index. Thus index 1 is the first. */
#define KEY_INDEX 1

#endif /* CONFIG_DW3000 */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#include "msg/twr_poll.hpp"
#include "msg/twr_response.hpp"
#include "msg/twr_final.hpp"
#include "msg/twr_report.hpp"

#define BROADCAST_ADDR 0x0000000000000000

typedef void (*FuncPtr)(uint8_t *, int16_t, uint64_t, uint64_t);

typedef struct
{
    struct k_work work;
    uint8_t msg[128];
    uint16_t len;
    uint64_t src_addr;
    uint64_t dst_addr;
    uint64_t rx_ts;
} rx_work_msg;

class Device
{
private:
public:
    /* data */
    uint64_t device_address;
    uint16_t pan_id;

    static Device *device_ptr;
    dwt_config_t config;
    dwt_aes_config_t aes_config;

    dwt_aes_job_t aes_job_tx;
    dwt_aes_job_t aes_job_rx;
    mac_frame_802_15_4_format_t mac_frame;

    uint8_t rx_buffer[RX_BUF_LEN];

    Device();
    void app(void *, void *, void *);
    uint64_t tx_msg(uint8_t *msg, uint16_t len, uint64_t dest_addr, uint8_t mode);
    void set_msg_dly_ts(uint8_t *msg, uint16_t len, uint64_t ts);
    static void tx_done_cb(const dwt_cb_data_t *cb_data);
    static void rx_ok_cb(const dwt_cb_data_t *cb_data);
    static void rx_work_handler(struct k_work *item);
    virtual void msg_process_cb(uint8_t *msg, uint16_t msg_len, uint64_t src_addr, uint64_t dst_addr, uint64_t rx_ts);
};

#endif