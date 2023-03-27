#include <device/tag.hpp>

#if defined(DEVICE_TAG)

LOG_MODULE_REGISTER(tag, LOG_LEVEL);

SYS_HASHMAP_DEFAULT_DEFINE_STATIC(poll_tx_ts_map);
SYS_HASHMAP_DEFAULT_DEFINE_STATIC(measure_res);
#if defined(DS_TWR)
SYS_HASHMAP_DEFAULT_DEFINE_STATIC(poll_rx_ts_map);
SYS_HASHMAP_DEFAULT_DEFINE_STATIC(resp_rx_ts_map);
SYS_HASHMAP_DEFAULT_DEFINE_STATIC(final_tx_ts_map);
#endif
K_SEM_DEFINE(measure_finish_sem, 0, 1);

Tag::Tag()
{
}

void Tag::app(void *p1, void *p2, void *p3)
{
    LOG_DBG("Tag app start.");
    std::vector<uint64_t> dst_addr_list;
    dst_addr_list.push_back(0x1000000000000001);
    dst_addr_list.push_back(0x1000000000000002);
    dst_addr_list.push_back(0x1000000000000003);
    while (1)
    {

        sys_hashmap_clear(&poll_tx_ts_map, NULL, NULL);
        sys_hashmap_clear(&measure_res, NULL, NULL);

        { // app
            for (int i = 0; i < dst_addr_list.size(); i++)
            {
                msg::twr_poll msg;
                uint64_t dst_addr = dst_addr_list[i];
                uint64_t tx_ts = tx_msg((uint8_t *)&msg, sizeof(msg), dst_addr, DWT_START_TX_IMMEDIATE);
                sys_hashmap_insert(&poll_tx_ts_map, dst_addr, tx_ts, NULL);

                k_sem_take(&measure_finish_sem, K_MSEC(50));
            }
        }
        package_res();
        k_sleep(K_SECONDS(10));
    }
}

void Tag::package_res(void)
{
    char package_str[512];

    sprintf(package_str, "{\"addr\":\"%016llx\",\"data\":[", DEVICE_ADDR);

    sys_hashmap_foreach(
        &measure_res,
        [](uint64_t key, uint64_t value, void *cookie)
        {
            uint64_t anthor_addr = key;
            uint64_t distance = value;
            char *package_str = (char *)cookie;

            char str[64];
            sprintf(str, "{\"addr\":\"%016llx\",\"distance\":%lld}", anthor_addr, distance);
            strcat(package_str, str);
            // LOG_DBG("\"addr\":\"%016llx\",\"distance\":%lld", anthor_addr, distance);
        },
        (void *)&package_str);

    strcat(package_str, "]}");
    for (int i = 0; i < strlen(package_str); i++)
    {
        char c = package_str[i];
        uart_poll_out(DEVICE_DT_GET(DT_ALIAS(serial)), (uint8_t)c);
    }
}
void Tag::msg_process_cb(uint8_t *msg_recv, uint16_t msg_recv_len, uint64_t src_addr, uint64_t dst_addr, uint64_t rx_ts)
{
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

        int64_t rtd_init = resp_rx_ts - poll_tx_ts;
        int64_t rtd_resp = resp_tx_ts - poll_rx_ts;

        float clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

        double tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        double distance = tof * SPEED_OF_LIGHT;

        if (distance < 0)
            distance = 0;

        sys_hashmap_insert(&measure_res, src_addr, (uint64_t)(distance * 1000), NULL);

        LOG_INF("distance from %016llx to %016llx is %lf", src_addr, DEVICE_ADDR, distance);

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

        uint64_t Ra = (resp_rx_ts - poll_tx_ts);
        uint64_t Rb = (final_rx_ts - resp_tx_ts);
        uint64_t Da = (final_tx_ts - resp_rx_ts);
        uint64_t Db = (resp_tx_ts - poll_rx_ts);
        int64_t tof_dtu = (int64_t)((double)(Ra * Rb - Da * Db) / (double)(Ra + Rb + Da + Db));
        double tof = tof_dtu * DWT_TIME_UNITS;
        double distance = tof * SPEED_OF_LIGHT;

        LOG_INF("distance from %016llx to %016llx is %lf", src_addr, (uint64_t)DEVICE_ADDR, distance);
    }
#endif
    }
}

#endif