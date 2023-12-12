#include <device/tag.hpp>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>

#if defined(DEVICE_TAG)

LOG_MODULE_REGISTER(tag, LOG_LEVEL);

SYS_HASHMAP_DEFINE_STATIC(poll_tx_ts_map);
SYS_HASHMAP_DEFINE_STATIC(measure_res);
#if defined(DS_TWR)
SYS_HASHMAP_DEFINE_STATIC(poll_rx_ts_map);
SYS_HASHMAP_DEFINE_STATIC(resp_rx_ts_map);
SYS_HASHMAP_DEFINE_STATIC(final_tx_ts_map);
#endif
K_SEM_DEFINE(measure_finish_sem, 0, 1);

Tag::Tag()
{
}

void Tag::app(void *p1, void *p2, void *p3)
{
    LOG_DBG("Tag app start.");
#if CONFIG_DW3000
    uint64_t dst_addr_list[] = {
        0x1000000000000001,
        0x1000000000000002,
        0x1000000000000003};
    uint8_t dst_addr_list_size = sizeof(dst_addr_list) / (sizeof(uint64_t));

    while (1)
    {
        sys_hashmap_clear(&poll_tx_ts_map, NULL, NULL);
        sys_hashmap_clear(&measure_res, NULL, NULL);

        for (int i = 0; i < dst_addr_list_size; i++)
        {
            msg::twr_poll msg;
            uint64_t dst_addr = dst_addr_list[i];
            uint64_t tx_ts = tx_msg((uint8_t *)&msg, sizeof(msg), dst_addr, DWT_START_TX_IMMEDIATE);
            sys_hashmap_insert(&poll_tx_ts_map, dst_addr, tx_ts, NULL);

            k_sem_take(&measure_finish_sem, K_MSEC(100));
        }

        package_res();
        k_sleep(K_SECONDS(1));
    }
#elif CONFIG_DW1000

    /* Write frame data to DW1000 and prepare transmission. See NOTE 4 below.*/
    uint8 msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0, 0};
    dwt_writetxdata(sizeof(msg), msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(msg), 0, 0);  /* Zero offset in TX buffer, no ranging. */
    dwt_forcetrxoff();
    /* Start transmission. */ 
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    dwt_starttx(DWT_START_TX_IMMEDIATE);

    /* Poll DW1000 until TX frame sent event set. See NOTE 5 below.
     * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
     * function to access it.*/
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
    {
        // k_sleep(K_SECONDS(1));
    };

    /* Clear TX frame sent event. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
#endif
    while (1)
    {
        k_sleep(K_SECONDS(1));
    }
}

void Tag::package_res(void)
{
    char package_str[64];

    sprintf(package_str, "{\"addr\":\"%016llx\",\"data\":[", device_address);
    for (int i = 0; i < strlen(package_str); i++)
    {
        char c = package_str[i];
        uart_poll_out(DEVICE_DT_GET(DT_ALIAS(serial)), (uint8_t)c);
    }

    sys_hashmap_foreach(
        &measure_res,
        [](uint64_t key, uint64_t value, void *cookie)
        {
            uint64_t anthor_addr = key;
            uint64_t distance = value;
            char *package_str = (char *)cookie;

            sprintf(package_str, "{\"addr\":\"%016llx\",\"distance\":%lld}", anthor_addr, distance);

            for (int i = 0; i < strlen(package_str); i++)
            {
                char c = package_str[i];
                uart_poll_out(DEVICE_DT_GET(DT_ALIAS(serial)), (uint8_t)c);
            }

            // LOG_DBG("\"addr\":\"%016llx\",\"distance\":%lld", anthor_addr, distance);
        },
        (void *)&package_str);

    sprintf(package_str, "]}\r\n");
    for (int i = 0; i < strlen(package_str); i++)
    {
        char c = package_str[i];
        uart_poll_out(DEVICE_DT_GET(DT_ALIAS(serial)), (uint8_t)c);
    }
}
int64_t Tag::dw_ts_reduce(uint64_t a, uint64_t b)
{

#define TS_CYCLE 0x10000000000
    if (a < b)
    {
        return (int64_t)(TS_CYCLE + a) - (int64_t)b;
    }
    else
    {
        return (int64_t)a - (int64_t)b;
    }
}
void Tag::msg_process_cb(uint8_t *msg_recv, uint16_t msg_recv_len, uint64_t src_addr, uint64_t dst_addr, uint64_t rx_ts)
{
#if CONFIG_DW3000
    switch (msg_recv[0])
    {
#if defined(SS_TWR)
    case (MSG_TWR_RESPONSE):
    {
        msg::twr_response *msg = (msg::twr_response *)msg_recv;

        uint64_t poll_tx_ts;
        uint64_t poll_rx_ts = msg->poll_rx_ts;
        uint64_t resp_tx_ts = msg->resp_tx_ts;
        uint64_t resp_rx_ts = rx_ts;
        sys_hashmap_get(&poll_tx_ts_map, src_addr, &poll_tx_ts);

        int64_t rtd_init = dw_ts_reduce(resp_rx_ts, poll_tx_ts);
        int64_t rtd_resp = dw_ts_reduce(resp_tx_ts, poll_rx_ts);

        float clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

        double tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        double distance = tof * SPEED_OF_LIGHT;

        if (distance < 0)
            distance = 0;

        sys_hashmap_insert(&measure_res, src_addr, (uint64_t)(distance * 1000), NULL);

        LOG_INF("distance from %016llx to %016llx is %lf", src_addr, device_address, distance);

        k_sem_give(&measure_finish_sem);

        break;
    }
#endif
#if defined(DS_TWR)
    case (MSG_TWR_RESPONSE):
    {
        msg::twr_response *msg = (msg::twr_response *)msg_recv;

        sys_hashmap_insert(&poll_rx_ts_map, src_addr, msg->poll_rx_ts, NULL);
        sys_hashmap_insert(&resp_rx_ts_map, src_addr, rx_ts, NULL);

        msg::twr_final msg_tx;

        uint64_t tx_ts = tx_msg((uint8_t *)&msg_tx, sizeof(msg_tx), src_addr, DWT_START_TX_IMMEDIATE);

        sys_hashmap_insert(&final_tx_ts_map, src_addr, tx_ts, NULL);

        break;
    }
    case (MSG_TWR_REPORT):
    {
        msg::twr_report *msg = (msg::twr_report *)msg_recv;

        uint64_t poll_tx_ts;
        uint64_t poll_rx_ts;
        uint64_t resp_tx_ts = msg->resp_tx_ts;
        uint64_t resp_rx_ts;
        uint64_t final_tx_ts;
        uint64_t final_rx_ts = msg->final_rx_ts;

        sys_hashmap_get(&poll_tx_ts_map, src_addr, &poll_tx_ts);
        sys_hashmap_get(&poll_rx_ts_map, src_addr, &poll_rx_ts);
        sys_hashmap_get(&resp_rx_ts_map, src_addr, &resp_rx_ts);
        sys_hashmap_get(&final_tx_ts_map, src_addr, &final_tx_ts);

        int64_t Ra = dw_ts_reduce(resp_rx_ts, poll_tx_ts);
        int64_t Rb = dw_ts_reduce(final_rx_ts, resp_tx_ts);
        int64_t Da = dw_ts_reduce(final_tx_ts, resp_rx_ts);
        int64_t Db = dw_ts_reduce(resp_tx_ts, poll_rx_ts);

        int64_t tof_dtu = (int64_t)((double)(Ra * Rb - Da * Db) / (double)(Ra + Rb + Da + Db));
        double tof = tof_dtu * DWT_TIME_UNITS;
        double distance = tof * SPEED_OF_LIGHT;

        if (distance < 0)
            distance = 0;

        sys_hashmap_insert(&measure_res, src_addr, (uint64_t)(distance * 1000), NULL);

        LOG_INF("distance from %016llx to %016llx is %lf", src_addr, device_address, distance);

        k_sem_give(&measure_finish_sem);
    }
#endif
    }
#endif
}

#endif
