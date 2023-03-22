#include <device/anthor.hpp>

#if defined(DEVICE_ANCHOR)

LOG_MODULE_REGISTER(anthor, LOG_LEVEL_DBG);

Anthor::Anthor()
{
}
void Anthor::app(void *p1, void *p2, void *p3)
{
    LOG_DBG("Anthor app start.");
    while (1)
    {
        { // app
            dwt_forcetrxoff();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
        k_sleep(K_SECONDS(10));
    }
}
#endif