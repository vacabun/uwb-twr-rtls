#ifndef _NODE_HPP_
#define _NODE_HPP_

#if CONFIG_DEVICE_TYPE_NODE

#include "device/device.hpp"
#include <zephyr/sys/ring_buffer.h>

#include <string>

#define RING_BUF_SIZE 1024
#define JSON_BUF_SIZE 256

class Node : public Device
{
public:
    uint8_t json_buf[JSON_BUF_SIZE];
    size_t json_buf_len = 0;

    Node();
    void app(void *p1, void *p2, void *p3);
    int64_t dw_ts_reduce(uint64_t a, uint64_t b);
    void msg_process_cb(uint8_t *msg_recv, uint16_t msg_recv_len, uint64_t src_addr, uint64_t dst_addr, uint64_t rx_ts);
    static void serial_isr(const struct device *dev, void *user_data);
    int parse_json(uint8_t *json_buf, size_t json_buf_len);
};

#endif

#endif
