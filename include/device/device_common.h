#ifndef _DEVICE_COMMON_H_
#define _DEVICE_COMMON_H_


#include <zephyr/kernel.h>

#define MAX_MSG_LEN 32

typedef struct
{
     // uint16_t dest_pan_id;
     // uint64_t dest_addr;
     // uint64_t src_addr;
     uint16_t data_len;
     uint8_t data[MAX_MSG_LEN];
} msg_t;



#endif