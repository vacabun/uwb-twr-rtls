#ifndef _NODE_HPP_
#define _NODE_HPP_

#if defined(DEVICE_NODE)

#include "device/device.hpp"


#include <string>

class Node : public Device
{
public:
    Node();
    void app(void *p1, void *p2, void *p3);
    int64_t dw_ts_reduce(uint64_t a, uint64_t b);
    void msg_process_cb(uint8_t *msg_recv, uint16_t msg_recv_len, uint64_t src_addr, uint64_t dst_addr, uint64_t rx_ts);

};

#endif

#endif
