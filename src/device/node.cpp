#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/hash_map.h>
#include <zephyr/data/json.h>
#include <zephyr/sys/ring_buffer.h>

#include <msg/twr_poll.hpp>
#include <msg/twr_report.hpp>
#include <msg/twr_response.hpp>
#include <msg/twr_final.hpp>

#include <device/node.hpp>

#define SPEED_OF_LIGHT (299702547)

#if defined(DEVICE_NODE)

LOG_MODULE_REGISTER(node, LOG_LEVEL);

SYS_HASHMAP_DEFINE_STATIC(poll_tx_ts_map);
SYS_HASHMAP_DEFINE_STATIC(response_tx_ts_map);
#if defined(DS_TWR)
SYS_HASHMAP_DEFINE_STATIC(poll_rx_ts_map);
SYS_HASHMAP_DEFINE_STATIC(resp_rx_ts_map);
SYS_HASHMAP_DEFINE_STATIC(final_tx_ts_map);
#endif
SYS_HASHMAP_DEFINE_STATIC(measure_res);
K_SEM_DEFINE(measure_finish_sem, 0, 1);

struct ring_buf serial_rx_ring_buf;

Node::Node()
{
}

struct command
{
    int type;
    const char *p1;
    const char *p2;
};

static const struct json_obj_descr command_descr[] = {
    JSON_OBJ_DESCR_PRIM(struct command, type, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct command, p1, JSON_TOK_STRING),
    JSON_OBJ_DESCR_PRIM(struct command, p2, JSON_TOK_STRING),
};

void Node::app(void *p1, void *p2, void *p3)
{
    LOG_DBG("Node app start.");

    const struct device *dev = DEVICE_DT_GET(DT_ALIAS(serial));
    uart_irq_callback_user_data_set(dev, &Node::serial_isr, NULL);
    uart_irq_rx_enable(dev);

    ring_buf_init(&serial_rx_ring_buf, RING_BUF_SIZE, (uint8_t *)malloc(RING_BUF_SIZE));

    while (1)
    {

        if (ring_buf_space_get(&serial_rx_ring_buf) < RING_BUF_SIZE)
        {
            uint8_t tmp_buf[128];
            int len = ring_buf_get(&serial_rx_ring_buf, tmp_buf, sizeof(tmp_buf) - 1);
            if (json_buf_len + len < JSON_BUF_SIZE)
            {
                memcpy(json_buf + json_buf_len, tmp_buf, len);
                json_buf_len += len;
                json_buf[json_buf_len] = '\0';

                // Try to parse JSON
                int ret = parse_json(json_buf, json_buf_len);
                if (ret == 0)
                {
                    // JSON parsed successfully, clear the buffer
                    json_buf_len = 0;
                }
                else if (ret == -1)
                {
                    // JSON is incomplete, keep the data in the buffer
                }
                else
                {
                    // Error occurred, clear the buffer
                    json_buf_len = 0;
                }
            }
            else
            {
                // Buffer is full, clear it
                json_buf_len = 0;
            }
        }

        k_sleep(K_SECONDS(1));
    }
}
int Node::parse_json(uint8_t *json_buf, size_t json_buf_len)
{
    struct command cmd_results;
    int ret = json_obj_parse((char *)json_buf, json_buf_len,
                             command_descr,
                             ARRAY_SIZE(command_descr),
                             &cmd_results);

    if (ret < 0)
    {
        LOG_ERR("JSON Parse Error: %d", ret);
    }
    else
    {
        LOG_INF("json_obj_parse return code: %d", ret);
        
        LOG_INF("type: %d", cmd_results.type);
        LOG_INF("p1: %s", cmd_results.p1);
        LOG_INF("p2: %s", cmd_results.p2);

    }
    return ret;
}
void Node::serial_isr(const struct device *dev, void *user_data)
{
    while (uart_irq_update(dev) && uart_irq_is_pending(dev))
    {
        if (uart_irq_rx_ready(dev))
        {
            uint8_t ch;
            uart_fifo_read(dev, &ch, 1);
            ring_buf_put(&serial_rx_ring_buf, &ch, 1);
        }
    }
}

int64_t Node::dw_ts_reduce(uint64_t a, uint64_t b)
{

#define TS_CYCLE 0x10000000000
    if (a < b)
    {
        return (int64_t)(TS_CYCLE + a) - (int64_t)b;
    }
    else
    {
        return (int64_t)a - (int64_t)b;
    }
}

void Node::msg_process_cb(uint8_t *msg_recv, uint16_t msg_recv_len, uint64_t src_addr, uint64_t dst_addr, uint64_t rx_ts)
{
    switch (msg_recv[0])
    {
#if defined(SS_TWR)
    case (MSG_TWR_POLL):
    {
        msg::twr_response msg;
        msg.poll_rx_ts = rx_ts;

        tx_msg((uint8_t *)&msg, sizeof(msg), src_addr, DWT_START_TX_DELAYED);

        break;
    }
    case (MSG_TWR_RESPONSE):
    {

        msg::twr_response *msg = (msg::twr_response *)msg_recv;

        uint64_t poll_tx_ts;
        uint64_t poll_rx_ts = msg->poll_rx_ts;
        uint64_t resp_tx_ts = msg->resp_tx_ts;
        uint64_t resp_rx_ts = rx_ts;
        sys_hashmap_get(&poll_tx_ts_map, src_addr, &poll_tx_ts);

        int64_t rtd_init = dw_ts_reduce(resp_rx_ts, poll_tx_ts);
        int64_t rtd_resp = dw_ts_reduce(resp_tx_ts, poll_rx_ts);

        float clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

        double tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        double distance = tof * SPEED_OF_LIGHT;

        if (distance < 0)
            distance = 0;

        sys_hashmap_insert(&measure_res, src_addr, (uint64_t)(distance * 1000), NULL);

        LOG_INF("distance from %016llx to %016llx is %lf", src_addr, device_address, distance);

        k_sem_give(&measure_finish_sem);

        break;
    }
#endif
#if defined(DS_TWR)
    case (MSG_TWR_POLL):
    {
        msg::twr_response msg;
        msg.poll_rx_ts = rx_ts;

        uint64_t tx_ts = tx_msg((uint8_t *)&msg, sizeof(msg), src_addr, DWT_START_TX_IMMEDIATE);
        sys_hashmap_insert(&response_tx_ts_map, src_addr, tx_ts, NULL);

        break;
    }
    case (MSG_TWR_RESPONSE):
    {
        msg::twr_response *msg = (msg::twr_response *)msg_recv;
        sys_hashmap_insert(&poll_rx_ts_map, src_addr, msg->poll_rx_ts, NULL);
        sys_hashmap_insert(&resp_rx_ts_map, src_addr, rx_ts, NULL);

        msg::twr_final msg_tx;

        uint64_t tx_ts = tx_msg((uint8_t *)&msg_tx, sizeof(msg_tx), src_addr, DWT_START_TX_IMMEDIATE);

        sys_hashmap_insert(&final_tx_ts_map, src_addr, tx_ts, NULL);

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
    case (MSG_TWR_REPORT):
    {
        msg::twr_report *msg = (msg::twr_report *)msg_recv;

        uint64_t poll_tx_ts;
        uint64_t poll_rx_ts;
        uint64_t resp_tx_ts = msg->resp_tx_ts;
        uint64_t resp_rx_ts;
        uint64_t final_tx_ts;
        uint64_t final_rx_ts = msg->final_rx_ts;
        // LOG_DBG("poll_tx_ts %lld", poll_tx_ts);
        // LOG_DBG("poll_rx_ts %lld", poll_rx_ts);
        // LOG_DBG("resp_tx_ts %lld", resp_tx_ts);
        // LOG_DBG("resp_rx_ts %lld", resp_rx_ts);
        // LOG_DBG("final_tx_ts %lld", final_tx_ts);
        // LOG_DBG("final_rx_ts %lld", final_rx_ts);

        sys_hashmap_get(&poll_tx_ts_map, src_addr, &poll_tx_ts);
        sys_hashmap_get(&poll_rx_ts_map, src_addr, &poll_rx_ts);
        sys_hashmap_get(&resp_rx_ts_map, src_addr, &resp_rx_ts);
        sys_hashmap_get(&final_tx_ts_map, src_addr, &final_tx_ts);

        int64_t Ra = dw_ts_reduce(resp_rx_ts, poll_tx_ts);
        int64_t Rb = dw_ts_reduce(final_rx_ts, resp_tx_ts);
        int64_t Da = dw_ts_reduce(final_tx_ts, resp_rx_ts);
        int64_t Db = dw_ts_reduce(resp_tx_ts, poll_rx_ts);

        int64_t tof_dtu = (int64_t)((double)(Ra * Rb - Da * Db) / (double)(Ra + Rb + Da + Db));
        double tof = tof_dtu * DWT_TIME_UNITS;
        double distance = tof * SPEED_OF_LIGHT;

        if (distance < 0)
            distance = 0;

        sys_hashmap_insert(&measure_res, src_addr, (uint64_t)(distance * 1000), NULL);

        LOG_INF("distance from %016llx to %016llx is %lfcm", src_addr, device_address, distance);

        k_sem_give(&measure_finish_sem);
    }
#endif
    }
}

#endif
