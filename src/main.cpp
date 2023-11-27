/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

// use swd debug
#if CONFIG_DEBUG_SWD == 1
    #include "stm32f4xx_ll_system.h"
#endif

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart), "Console device is not ACM CDC UART device");
LOG_MODULE_REGISTER(main, LOG_LEVEL);

#define DEFAULT_STACKSIZE 4096

// rx workqueue
#define DEVICE_RX_WORK_QUEUE_STACK_SIZE 1024
#define DEVICE_RX_WORK_QUEUE_PRIORITY 5
K_THREAD_STACK_DEFINE(device_rx_work_q_stack_area, DEVICE_RX_WORK_QUEUE_STACK_SIZE);
struct k_work_q device_rx_work_q;

#if defined(DEVICE_TAG)
    #include "device/tag.hpp"
    static K_THREAD_STACK_DEFINE(tag_stack, DEFAULT_STACKSIZE);
    static struct k_thread tag_thread;
#endif // DEVICE_TAG

#if defined(DEVICE_ANCHOR)
    #include <device/anthor.hpp>
    static K_THREAD_STACK_DEFINE(anthor_responder_stack, DEFAULT_STACKSIZE);
    static struct k_thread anthor_responder_thread;
#endif // DEVICE_ANCHOR

int main(void)
{
	// swd debug
#if CONFIG_DEBUG_SWD == 1
	LL_DBGMCU_EnableDBGSleepMode();
#endif

	/* usb config */
	if (usb_enable(NULL))
		return 0;
	k_sleep(K_MSEC(1000));

	/* uart config */
    struct uart_config uart_cfg ={
		baudrate: 1000000,
		parity: UART_CFG_PARITY_NONE,
		stop_bits: UART_CFG_STOP_BITS_1,
		data_bits: UART_CFG_DATA_BITS_8,
		flow_ctrl: UART_CFG_FLOW_CTRL_NONE
	};
    int ret = uart_configure(DEVICE_DT_GET(DT_ALIAS(serial)), &uart_cfg);
    if (ret != 0)
    {
        LOG_DBG("Failed to configure UART: %d", ret);
    }
	else{
		LOG_DBG("UART configured successfully");
	}


	/* work config */
	k_work_queue_init(&device_rx_work_q);
	k_work_queue_start(&device_rx_work_q,
					   device_rx_work_q_stack_area,
					   K_THREAD_STACK_SIZEOF(device_rx_work_q_stack_area),
					   DEVICE_RX_WORK_QUEUE_PRIORITY, NULL);

#if defined(DEVICE_TAG)
	Tag tag;

	std::function<void(void *, void *, void *)> func = [&](void *arg1, void *arg2, void *arg3)
	{
		tag.app(arg1, arg2, arg3);
	};

	k_thread_create(
		&tag_thread,
		tag_stack,
		DEFAULT_STACKSIZE,
		[](void *arg1, void *arg2, void *arg3)
		{
			std::function<void(void *, void *, void *)> *func_ptr =
				static_cast<std::function<void(void *, void *, void *)> *>(arg1);
			(*func_ptr)(arg2, arg3, nullptr);
		},
		&func,
		&tag_thread,
		NULL,
		K_PRIO_COOP(7),
		0,
		K_NO_WAIT);

#endif //_DEVICE_TAG_

#if defined(DEVICE_ANCHOR)
	Anthor anthor;

	std::function<void(void *, void *, void *)> func = [&](void *arg1, void *arg2, void *arg3)
	{
		anthor.app(arg1, arg2, arg3);
	};

	k_thread_create(
		&anthor_responder_thread,
		anthor_responder_stack,
		DEFAULT_STACKSIZE,
		[](void *arg1, void *arg2, void *arg3)
		{
			std::function<void(void *, void *, void *)> *func_ptr =
				static_cast<std::function<void(void *, void *, void *)> *>(arg1);
			(*func_ptr)(arg2, arg3, nullptr);
		},
		&func,
		&anthor_responder_thread,
		NULL,
		K_PRIO_COOP(7),
		0,
		K_NO_WAIT);
#endif // _DEVICE_ANCHOR_

	return 0;
}
