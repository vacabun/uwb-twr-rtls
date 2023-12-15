#ifndef _MSG_TWR_REPORT_HPP_
#define _MSG_TWR_REPORT_HPP_

#include "msg/msg.hpp"

namespace msg
{
    class twr_report
    {
    public:
        twr_report();
        uint8_t type;
        uint64_t resp_tx_ts;
        uint64_t final_rx_ts;
    };
}

#endif
