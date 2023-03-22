#ifndef _ANTHOR_H_
#define _ANTHOR_H_

#if defined(DEVICE_ANCHOR)

#include "device/device.hpp"


class Anthor : public Device
{
public:
    Anthor();
    void app(void *p1, void *p2, void *p3);
};


#endif

#endif
