#ifndef _DEVICE_HPP_
#define _DEVICE_HPP_

#include <functional>
#include <zephyr/kernel.h>

/* Default antenna delay values for 64 MHz PRF.*/
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
/* Delay tx frames, in UWB microseconds.*/
#define TX_DLY_UUS 3000

#define BROADCAST_ADDR 0x0000000000000000

/* Buffer to store received response message.
The received frame cannot be bigger than 127 if STD PHR mode is used */
#define RX_BUF_LEN 127

#if CONFIG_DW3000
/* Note, the key index of 0 is forbidden to send as key index. Thus index 1 is the first. */
#define KEY_INDEX 1
#endif

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */
#if CONFIG_DW3000
#include "dw3000.h"
    dwt_mic_size_e dwt_mic_size_from_bytes(uint8_t mic_size_in_bytes);
    extern dwt_txconfig_t txconfig_options;
#endif /* CONFIG_DW3000 */
#if CONFIG_DW1000
#include "dw1000.h"
#endif /* CONFIG_DW1000 */
#ifdef __cplusplus
}
#endif /* __cplusplus */

#if CONFIG_DW3000
#define dw_hw_init dw3000_hw_init
#define dw_hw_reset dw3000_hw_reset
#define dw_hw_init_interrupt dw3000_hw_init_interrupt
#define dw_spi_speed_fast dw3000_spi_speed_fast
#define dw_hw_interrupt_enable dw3000_hw_interrupt_enable

#define DW_INIT_CONFIG_PARAMETER DWT_DW_INIT

#endif

#if CONFIG_DW1000
#define dw_hw_init dw1000_hw_init
#define dw_hw_reset dw1000_hw_reset
#define dw_hw_init_interrupt dw1000_hw_init_interrupt
#define dw_spi_speed_fast dw1000_spi_speed_fast
#define dw_hw_interrupt_enable dw1000_hw_interrupt_enable

#define DW_INIT_CONFIG_PARAMETER DWT_LOADUCODE
#endif

typedef void (*FuncPtr)(uint8_t *, int16_t, uint64_t, uint64_t);

#if CONFIG_DW1000
#include "frame_header.h"
#define DWT_MSG_TYPR srd_msg_dsss
#endif
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
    uint8_t rx_buffer[RX_BUF_LEN];
#if CONFIG_DW3000
    dwt_aes_config_t aes_config;
    dwt_aes_job_t aes_job_tx;
    dwt_aes_job_t aes_job_rx;
    mac_frame_802_15_4_format_t mac_frame;
#endif
#if CONFIG_DW1000
    uint16_t device_address16;
    DWT_MSG_TYPR msg_f_send;
    uint8_t frame_seq_num = 0;
#endif
    Device();
    void app(void *, void *, void *);
    uint64_t tx_msg(uint8_t *msg, uint16_t len, uint64_t dest_addr, uint8_t mode);
    void set_msg_dly_ts(uint8_t *msg, uint16_t len, uint64_t ts);

    static void tx_done_cb(const dwt_cb_data_t *cb_data);
    static void rx_ok_cb(const dwt_cb_data_t *cb_data);
    static void rx_err_cb(const dwt_cb_data_t *cb_data);
    static void rx_work_handler(struct k_work *item);
    virtual void msg_process_cb(uint8_t *msg, uint16_t msg_len, uint64_t src_addr, uint64_t dst_addr, uint64_t rx_ts);
};

#endif