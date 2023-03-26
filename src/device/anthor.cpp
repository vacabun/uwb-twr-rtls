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
        k_sleep(K_SECONDS(1));
    }
}

void Anthor::msg_process_cb(uint8_t *msg_recv, uint16_t msg_recv_len, uint64_t src_addr, uint64_t dst_addr, uint64_t rx_ts)
{
    switch (msg_recv[0])
    {
    case (MSG_TWR_POLL):
        msg::twr_response msg;
        msg.poll_rx_ts = rx_ts;

        tx_msg((uint8_t *)&msg, sizeof(msg), src_addr, DWT_START_TX_DELAYED);

        break;
    }
}

#endif
