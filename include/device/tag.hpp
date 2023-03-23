#ifndef _TAG_HPP_
#define _TAG_HPP_

#if defined(DEVICE_TAG)

#include "device/device.hpp"
#include <zephyr/sys/hash_map.h>
class Tag : public Device
{
public:
    Tag();
    void app(void *p1, void *p2, void *p3);

    void msg_process_cb(uint8_t *msg_recv, uint16_t msg_recv_len, uint64_t src_addr, uint64_t dst_addr, uint64_t rx_ts);
    
};

#endif

#endif
