#ifndef _ANTHOR_H_
#define _ANTHOR_H_

#if defined(DEVICE_ANCHOR)

#include "device/device.hpp"


class Anthor : public Device
{
public:
    Anthor();
    void app(void *p1, void *p2, void *p3);
    void msg_process_cb(uint8_t *msg, uint16_t msg_len, uint64_t src_addr, uint64_t dst_addr,uint64_t rx_ts);
};


#endif

#endif
