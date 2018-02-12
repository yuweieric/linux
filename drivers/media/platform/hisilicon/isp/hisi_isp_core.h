/*
 * Copyright (c) 2016-2017 Linaro Ltd.
 * Copyright (c) 2016-2017 HiSilicon Technologies Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef HISI_ISP_CORE_DEF_H
#define HISI_ISP_CORE_DEF_H

#include <linux/sched.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/version.h>
#include <linux/list.h>
#include <linux/file.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/completion.h>
#include <linux/io.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/compat.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/regulator/consumer.h>

#include "hisi_isp_reg_offset.h"
#include "hisi_isp_i2c.h"

#define COMP_HISI_ISP_NAME      "hisilicon,hisi-isp"
#define DEV_NAME_ISP            "hisi_isp"

#define USED        0x01
#define UNUSED      0x00

/* #define HISP_DEBUG */

#if defined(HISP_DEBUG)
#define ISP_ERR(msg, ...) \
	pr_err("[ISP] %s: "msg, __func__, ## __VA_ARGS__)
#define ISP_WARNING(msg, ...) \
	pr_warn("[ISP] %s: "msg, __func__, ## __VA_ARGS__)
#define ISP_INFO(msg, ...) \
	pr_info("[ISP] %s: "msg, __func__, ## __VA_ARGS__)
#define ISP_DEBUG(msg, ...) \
	pr_debug("[ISP] %s: "msg, __func__, ## __VA_ARGS__)
#else
#define ISP_ERR(msg, ...) \
	pr_err("[ISP] %s: "msg, __func__, ## __VA_ARGS__)
#define ISP_WARNING(msg, ...) \
	pr_warn("[ISP] %s: "msg, __func__, ## __VA_ARGS__)
#define ISP_INFO(msg, ...) \
	pr_info("[ISP] %s: "msg, __func__, ## __VA_ARGS__)
#define ISP_DEBUG(msg, ...)
#endif

#define REG_SET(addr, val) writel(val, addr)
#define REG_GET(addr) readl(addr)

#define MASK0(name)  ((1<<(name##_LEN))-1)
#define MASK1(name)  (((1<<(name##_LEN))-1) << (name##_OFFSET))

/* operation on the field of a variable */
#define REG_GET_FIELD(reg, name) \
	(((reg) >> (name##_OFFSET)) & MASK0(name))

#define REG_SET_FIELD(reg, name, val) \
	(reg = ((reg) & ~MASK1(name)) | \
		(((val) & MASK0(name)) << (name##_OFFSET)))

struct stream_info {
	int cam_id;
	int w;
	int h;
	int format;
	int pool_size;
};

struct frame_info {
	int cam_id;
	int frame_num;
	int offset;
};

#define ISP_IOC_MAGIC       'I'

#define ISP_IOC_STREAM_ON     _IOWR(ISP_IOC_MAGIC, 0, struct stream_info)

#define ISP_IOC_FRAME_REQUEST _IOWR(ISP_IOC_MAGIC, 1, struct frame_info)

#define ISP_IOC_STREAM_OFF    _IOWR(ISP_IOC_MAGIC, 2, struct stream_info)

struct isp_device {
	struct miscdevice dev;
	char __iomem *base;
	char __iomem *base_gpio;
	uint32_t isp_irq_vic1;
	uint32_t isp_irq_frproc0;
	struct clk *xclk;
	struct clk *xclk1;
	int xclk_freq;
	struct regulator *io_regulator;

	void *virt_addr;
	dma_addr_t hw_addr;
	unsigned int pool_size;
	size_t frame_size;
	size_t frame_count;
	size_t isp_frame_num;
	size_t hal_frame_num;
	size_t cur_frame_num;
	struct completion frame_comp;
	bool streaming;

	unsigned short addr;
	unsigned int flag;
	unsigned int speed_cfg;

	int csi_index;
	int dt;
	int pf;
	int w;
	int h;

	int csi_lane;
	int pwdn_gpio;
	int rst_gpio;
	struct isp_i2c_client_t *client;
	struct sensor_info {
		unsigned short addr;
		unsigned int flag;
		unsigned int speed_cfg;

		int csi_index;
		int dt;
		int pf;
		int csi_lane;

	    int pwdn_gpio;
		int rst_gpio;

		int (*camera_sensor_power)(struct isp_i2c_client_t *, int);
		int (*camera_sensor_set_fmt)(struct isp_i2c_client_t *,
				unsigned int, unsigned int);
		int (*camera_stream_on_off)(struct isp_i2c_client_t *, int);
		bool probe;
	} sensor[2];
};
#endif /* HISI_ISP_CORE_DEF_H */
