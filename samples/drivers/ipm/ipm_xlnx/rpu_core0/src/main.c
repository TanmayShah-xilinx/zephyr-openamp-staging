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

#define RPU_CORE_ID 0

#define apu_mbox_label rpu0_apu_mailbox
#define rpu_mbox_label rpu0_rpu1_mailbox
#define TEST_STR       "ping from RPU0"

struct xlnx_ipi_data {
	size_t len;
	void *user_data;
	uint8_t data[];
};

static void apu_ipm_callback(const struct device *dev, void *context, uint32_t id, volatile void *data)
{
	struct xlnx_ipi_data *msg = (struct xlnx_ipi_data *)data;
	int status;

	printk("RPU%d rx: %s\n", RPU_CORE_ID, msg->data);

	/* echo back to Remote processor */
	status = ipm_send(dev, 0, id, msg->data, msg->len);
	if (status) {
		printk("ipm_send() failed: %d\n", status);
	}
}

void main(void)
{
	const struct device *apu_ipm;

	printk("starting rpu0 ipm demo\n");

	/* ping APU */
	apu_ipm = DEVICE_DT_GET(DT_NODELABEL(apu_mbox_label));
	if (!device_is_ready(apu_ipm)) {
		printk("error: device isn't ready\n");
		while (1) {
			;
		}
	}

	ipm_register_callback(apu_ipm, apu_ipm_callback, NULL);

	if (ipm_set_enabled(apu_ipm, 1) != 0) {
		printk("interrupt is not enabled\n");
	}

	while (1) {
	}
}
