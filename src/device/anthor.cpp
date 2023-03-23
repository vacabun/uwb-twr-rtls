#include <device/anthor.hpp>

#if defined(DEVICE_ANCHOR)

LOG_MODULE_REGISTER(anthor, LOG_LEVEL_DBG);

Anthor::Anthor()
{
}
void Anthor::app(void *p1, void *p2, void *p3)
{
    LOG_DBG("Anthor app start.");
    while (1)
    {
        { // app
            dwt_forcetrxoff();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
        k_sleep(K_SECONDS(10));
    }
}

void Anthor::msg_process_cb(uint8_t *msg_recv, uint16_t msg_recv_len, uint64_t src_addr, uint64_t dst_addr, uint64_t rx_ts)
{
    LOG_DBG("Rx msg from %016llx to %016llx", src_addr, dst_addr);
    LOG_HEXDUMP_DBG(msg_recv, msg_recv_len, "recv msg");

    uint8_t msg_type = msg_recv[0];

    switch (msg_type)
    {
    case (MSG_INITIATOR):
        LOG_DBG("recv msg initiator");

        msg::msg_responder msg;
        msg.poll_rx_ts = rx_ts;
        
        tx_msg((uint8_t *)&msg, sizeof(msg), src_addr, DWT_START_TX_DELAYED);

        LOG_HEXDUMP_DBG((uint8_t *)&msg, sizeof(msg), "tx msg");

        break;
    }
}

#endif
