#include <device/tag.hpp>

#if defined(DEVICE_TAG)

LOG_MODULE_REGISTER(tag, LOG_LEVEL);

SYS_HASHMAP_DEFAULT_DEFINE_STATIC(poll_tx_ts_map);

K_SEM_DEFINE(measure_finish_sem, 0, 1);

Tag::Tag()
{
}
void Tag::app(void *p1, void *p2, void *p3)
{
    LOG_DBG("Tag app start.");
    while (1)
    {
        { // app

            std::vector<uint64_t> dst_addr_list;
            dst_addr_list.push_back(0x1122334455667788);
            dst_addr_list.push_back(0x0000000000000001);
            dst_addr_list.push_back(0x0000000000000002);
            dst_addr_list.push_back(0x0000000000000003);
            for (int i = 0; i < dst_addr_list.size(); i++)
            {
                msg::twr_poll msg;
                uint64_t dst_addr = dst_addr_list[i];
                uint64_t tx_ts = tx_msg((uint8_t *)&msg, sizeof(msg), dst_addr, DWT_START_TX_IMMEDIATE);
                sys_hashmap_insert(&poll_tx_ts_map, dst_addr, tx_ts, NULL);

                k_sem_take(&measure_finish_sem, K_SECONDS(2));
            }
        }
        k_sleep(K_SECONDS(1));
    }
}
void Tag::msg_process_cb(uint8_t *msg_recv, uint16_t msg_recv_len, uint64_t src_addr, uint64_t dst_addr, uint64_t rx_ts)
{
    switch (msg_recv[0])
    {
#if defined(SS_TWR)
    case MSG_TWR_RESPONSE:
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

        LOG_DBG("distance from %016llx to %016llx is %lf", src_addr, DEVICE_ADDR, distance);

        k_sem_give(&measure_finish_sem);

        break;
    }
#endif
#if defined(DS_TWR)
    case MSG_TWR_RESPONSE:
    {
        break;
    }
#endif
    }
}

#endif