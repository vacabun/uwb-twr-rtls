#ifndef _MSG_TWR_RESPONSE_HPP_
#define _MSG_TWR_RESPONSE_HPP_

#include "msg/msg.hpp"

namespace msg
{
    class twr_response
    {
    public:
        twr_response();
        uint8_t type;
        uint64_t poll_rx_ts;
#if CONFIG_SS_TWR
        uint64_t resp_tx_ts;
#endif
    };
}

#endif
