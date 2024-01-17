#include <zephyr/logging/log.h>
#include <device/anthor.hpp>

#include <msg/twr_poll.hpp>
#include <msg/twr_final.hpp>
#include <msg/twr_response.hpp>
#include <msg/twr_report.hpp>

#if CONFIG_DEVICE_TYPE_ANCHOR

LOG_MODULE_REGISTER(anthor, LOG_LEVEL);
#if CONFIG_DS_TWR

SYS_HASHMAP_DEFINE_STATIC(response_tx_ts_map);

#endif
Anthor::Anthor()
{
}
void Anthor::app(void *p1, void *p2, void *p3)
{
    LOG_DBG("Anthor app start.");
    while (1)
    {
#if CONFIG_DW1000
        // extern k_mutex transceiver_mutex;
        // k_mutex_lock(&transceiver_mutex, K_NO_WAIT);
        // {
        //     dwt_forcetrxoff();
        //     dwt_rxenable(DWT_START_RX_IMMEDIATE);
        // }
        // k_mutex_unlock(&transceiver_mutex);
#endif
#if CONFIG_DW3000
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
#endif
        k_sleep(K_SECONDS(1));
    }
}

void Anthor::msg_process_cb(uint8_t *msg_recv, uint16_t msg_recv_len, uint64_t src_addr, uint64_t dst_addr, uint64_t rx_ts)
{
    LOG_DBG("msg_process_cb %016llx %016llx %d", src_addr, dst_addr, msg_recv[0]);

    switch (msg_recv[0])
    {
#if CONFIG_SS_TWR
    case (MSG_TWR_POLL):
    {
        msg::twr_response msg;
        msg.poll_rx_ts = rx_ts;

        tx_msg((uint8_t *)&msg, sizeof(msg), src_addr, DWT_START_TX_DELAYED);

        break;
    }

#endif

#if CONFIG_DS_TWR
    case (MSG_TWR_POLL):
    {
        msg::twr_response msg;
        msg.poll_rx_ts = rx_ts;

        uint64_t tx_ts =
            tx_msg((uint8_t *)&msg, sizeof(msg), src_addr, DWT_START_TX_IMMEDIATE);
        sys_hashmap_insert(&response_tx_ts_map, src_addr, tx_ts, NULL);

        break;
    }
    case (MSG_TWR_FINAL):
    {
        msg::twr_report msg;
        msg.final_rx_ts = rx_ts;
        uint64_t ts;
        sys_hashmap_get(&response_tx_ts_map, src_addr, &ts);
        sys_hashmap_remove(&response_tx_ts_map, src_addr, NULL);
        msg.resp_tx_ts = ts;
        tx_msg((uint8_t *)&msg, sizeof(msg), src_addr, DWT_START_TX_IMMEDIATE);

        break;
    }
#endif
    }
}

#endif
