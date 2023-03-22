#include <device/tag.hpp>

#if defined(DEVICE_TAG)

LOG_MODULE_REGISTER(tag, LOG_LEVEL_DBG);

Tag::Tag()
{
}
void Tag::app(void *p1, void *p2, void *p3)
{
    while (1)
    {
        { // app
            msg::msg_initiator msg;
            tx_msg((uint8_t *)&msg, sizeof(msg), 0x1122334455667788);
        }
        k_sleep(K_SECONDS(10));
    }
}
void Tag::msg_process_cb(uint8_t *msg, int16_t msg_len, uint64_t src_addr, uint64_t dst_addr)
{
    LOG_DBG("Rx msg from %016llx to %016llx (len %d)", src_addr, dst_addr, msg_len);
}
#endif