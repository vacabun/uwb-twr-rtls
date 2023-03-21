#ifndef _DEVICE_HPP_
#define _DEVICE_HPP_

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
    dwt_mic_size_e dwt_mic_size_from_bytes(uint8_t mic_size_in_bytes);

#ifdef __cplusplus
}
#endif
extern dwt_txconfig_t txconfig_options;

#include <shared_data/config_options.h>
#include <shared_data/shared_defines.h>
#include <shared_data/shared_functions.h>
#include <network/MAC_802_15_4/mac_802_15_4.hpp>
#include <network/MAC_802_15_4/key.hpp>
#include "msg/msg_initiator.hpp"
/* Default antenna delay values for 64 MHz PRF.*/
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Buffer to store received response message.
   The received frame cannot be bigger than 127 if STD PHR mode is used */
#define RX_BUF_LEN 127

/* Note, the key index of 0 is forbidden to send as key index. Thus index 1 is the first. */
#define KEY_INDEX 1
class Device
{
private:
    /* data */

    dwt_config_t config;

public:
    Device();
    bool init_dw3000();
    void app(void *, void *, void *);

    void set_callback();
    void tx_done_cb(const dwt_cb_data_t *cb_data);
    void rx_ok_cb(const dwt_cb_data_t *cb_data);
    void rx_timeout_cb(const dwt_cb_data_t *cb_data);
    void rx_err_cb(const dwt_cb_data_t *cb_data);
    void spi_err_cb(const dwt_cb_data_t *cb_data);
    void spi_ready_cb(const dwt_cb_data_t *cb_data);
    void dual_spi_cb(const dwt_cb_data_t *cb_data);
};

#endif