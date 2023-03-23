#include <device/tag.hpp>

#if defined(DEVICE_TAG)

LOG_MODULE_REGISTER(tag, LOG_LEVEL_DBG);

SYS_HASHMAP_CXX_DEFINE(poll_tx_ts_map);

Tag::Tag()
{
}
void Tag::app(void *p1, void *p2, void *p3)
{
    LOG_DBG("Tag app start.");
    while (1)
    {
        { // app
            msg::msg_initiator msg;
            uint64_t dst_addr = 0x1122334455667788;

            uint64_t tx_ts = tx_msg((uint8_t *)&msg, sizeof(msg), dst_addr, DWT_START_TX_IMMEDIATE);
            sys_hashmap_insert(&poll_tx_ts_map, dst_addr, tx_ts, NULL);

            LOG_DBG("ts %016llx", tx_ts);
        }
        k_sleep(K_SECONDS(10));
    }
}
void Tag::msg_process_cb(uint8_t *msg_recv, uint16_t msg_recv_len, uint64_t src_addr, uint64_t dst_addr, uint64_t rx_ts)
{
    LOG_DBG("Rx msg from %016llx to %016llx", src_addr, dst_addr);
    LOG_HEXDUMP_DBG(msg_recv, msg_recv_len, "recv msg");

    uint8_t msg_type = msg_recv[0];

    switch (msg_type)
    {
    case MSG_RESPONDER:

        LOG_DBG("recv msg responder");
        msg::msg_responder *msg = (msg::msg_responder *)msg_recv;

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

        break;
    }
}

#endif