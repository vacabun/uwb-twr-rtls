#ifndef _MSG_HPP_
#define _MSG_HPP_

#include <zephyr/kernel.h>

enum msg_type_def
{
    MSG_TWR_POLL = 0,
    MSG_TWR_RESPONSE,
    MSG_TWR_FINAL,
    MSG_TWR_REPORT,
};

#endif
