#ifndef _TAG_HPP_
#define _TAG_HPP_

#if defined(DEVICE_TAG)

#include "device/device.hpp"

class Tag : public Device
{
public:
    Tag();
    void app(void *p1, void *p2, void *p3);

    void msg_process_cb(uint8_t *msg, int16_t msg_len, uint64_t src_addr, uint64_t dst_addr);
};

#endif

#endif
