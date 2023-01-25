/*
 * Copyright (c) 2023, AMD-Xilinx
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/ipm.h>

/* only RPU CORE ID 0 is supported for now */
#ifndef RPU_CORE_ID
#define RPU_CORE_ID 0
#endif

/* mailbox nodes are labeld in the format host_remote_mailbox */
#if (RPU_CORE_ID == 0)
#define apu_mbox_label rpu0_apu_mailbox
#define rpu_mbox_label rpu0_rpu1_mailbox
#define TEST_STR "ping from RPU0"
#elif (RPU_CORE_ID == 1)
#define apu_mbox_label rpu1_apu_mailbox
#define rpu_mbox_label rpu1_rpu0_mailbox
#define TEST_STR "ping from RPU1"
#endif

struct xlnx_ipi_data {
	size_t len;
	void *user_data;
	uint8_t data[];
};

static void ipm_callback(const struct device *dev, void *context,
			 uint32_t id, volatile void *data)
{
	struct xlnx_ipi_data *msg = (struct xlnx_ipi_data *)data;

	printk("RPU%d rx: %s\n", RPU_CORE_ID, msg->data);
}

void main(void)
{
	const struct device *ipm;
	int status;
	int id = 0;
	char *test_str = TEST_STR;

	ipm = DEVICE_DT_GET(DT_NODELABEL(apu_mbox_label));
	if (!device_is_ready(ipm)) {
		printk("error: device isn't ready\n");
		while (1);
	}

	ipm_register_callback(ipm, ipm_callback, NULL);

	ipm_set_enabled(ipm, 1);

	while (1) {
		/* Ping Remote processor */
		status = ipm_send(ipm, 1, id, test_str, strlen(test_str)+1);
		if (status) {
			printk("ipm_send() failed: %d\n", status);
		}

		k_sleep(K_SECONDS(60));
	}
}
