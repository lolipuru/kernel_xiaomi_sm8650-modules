
/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022, Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include <linux/interrupt.h>
enum qts_client {
	QTS_CLIENT_PRIMARY_TOUCH,
	QTS_CLIENT_SECONDARY_TOUCH,
	QTS_CLIENT_MAX
};
enum qts_bus_type {
	QTS_BUS_TYPE_NONE,
	QTS_BUS_TYPE_I2C,
	QTS_BUS_TYPE_SPI,
	QTS_BUS_TYPE_SPI_V2,
	QTS_BUS_TYPE_MAX
};
struct qts_vendor_callback_ops {
	int (*enable_touch_irq)(void *data, bool en);
	int (*get_irq_num)(void *data);
	int (*pre_la_tui_enable)(void *data);
	int (*post_la_tui_enable)(void *data);
	int (*pre_la_tui_disable)(void *data);
	int (*post_la_tui_disable)(void *data);
};
struct qts_vendor_data {
	struct i2c_client *client;
	struct spi_device *spi;
	void *vendor_data;
	struct qts_vendor_callback_ops qts_vendor_ops;
	u32 client_type;
	u32 bus_type;
};
int qts_client_register(struct qts_vendor_data qts_vendor_data);
