#ifndef _MSG_RESPONDER_HPP_
#define _MSG_RESPONDER_HPP_

#include "msg/msg.hpp"

namespace msg
{
    class msg_responder
    {
    public:
        msg_responder();
        uint8_t type;
        uint64_t poll_rx_ts;
        uint64_t resp_tx_ts;
    };
}

#endif