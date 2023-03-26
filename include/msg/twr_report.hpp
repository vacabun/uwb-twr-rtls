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
    };
}

#endif