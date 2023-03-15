/*! ----------------------------------------------------------------------------
 *  @file    read_dev_id.c
 *  @brief   This example just read DW IC's device ID. It can be used to verify
 *           the SPI comms are working correctly.
 *
 * @attention
 *
 * Copyright 2018-2020 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include "dw3000.h"

#ifdef TEST_READING_DEV_ID

LOG_MODULE_REGISTER(test_read_dev_id, LOG_LEVEL_DBG);

void read_dev_id(void *p1, void *p2, void *p3)
{

    dw3000_hw_init();
    dw3000_hw_reset();
    dw3000_hw_init_interrupt();
    dw3000_spi_speed_fast();

    k_sleep(K_SECONDS(1));

    uint16_t err = dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

    if (err != DWT_ERROR)
    {
        uint32_t dev_id = dwt_readdevid();
        LOG_DBG("CHECK DEV ID OK : %x", dev_id);
        while (1)
            ;
    }
    else
    {
        LOG_DBG("CHECK DEV ID FAILED");
        while (1)
            ;
    }
}
#endif
/*****************************************************************************************************************************************************
 * NOTES:
 ****************************************************************************************************************************************************/
