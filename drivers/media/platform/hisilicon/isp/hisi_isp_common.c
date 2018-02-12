/*
 * Copyright (c) 2016-2017 Linaro Ltd.
 * Copyright (c) 2016-2017 HiSilicon Technologies Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include "hisi_isp_common.h"
#include "hisi_isp_cvdr.h"
#include "hisi_isp_sr.h"
#include "hisi_isp_csi.h"
#include "hisi_isp_isr.h"

int frame_num2Offset(struct isp_device *dev, int frame_num)
{
	return (dev->frame_size * (frame_num % dev->frame_count));
}

void isp_config_smmu_bypass(struct isp_device *dev)
{
	REG_SET(dev->base + ISP_BASE_ADDR_SMMU, 0x1);
}

int isp_ispss_reset_all(struct isp_device *dev)
{
	char __iomem *base = dev->base;

	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x060, 0xFFFFFFFF);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x064, 0xFFFFFFFF);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x068, 0xFFFFFFFF);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x06C, 0xFFFFFFFF);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x070, 0xFFFFFFFF);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x074, 0xFFFFFFFF);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x078, 0xFFFFFFFF);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x374, 0x00000003);
	mdelay(1);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x060, 0x00000000);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x064, 0x00000000);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x068, 0x00000000);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x06C, 0x00000000);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x070, 0x00000000);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x074, 0x00000000);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x078, 0x00000000);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x374, 0x00000000);
	mdelay(1);
	return 0;
}
EXPORT_SYMBOL(isp_ispss_reset_all);

void isp_ispss_enable_clock(struct isp_device *dev)
{
	char __iomem *base = dev->base;
	/* enable all clock of isp sub-modules */
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x010, 0xffffffff);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x014, 0xffffffff);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x018, 0xffffffff);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x01C, 0xffffffff);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x020, 0xffffffff);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x024, 0xffffffff);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x028, 0xffffffff);
	REG_SET(base + ISP_BASE_ADDR_ISPSS_CTRL + 0x364, 0x00000003);
	mdelay(1);
}
EXPORT_SYMBOL(isp_ispss_enable_clock);

int isp_start_streaming(struct isp_device *dev, int index)
{
	int ret;
	struct isp_i2c_client_t *i2c_client = NULL;

	dev->addr = dev->sensor[index].addr;
	dev->flag = dev->sensor[index].flag;
	dev->speed_cfg = dev->sensor[index].speed_cfg;
	dev->csi_index = dev->sensor[index].csi_index;
	dev->dt = dev->sensor[index].dt;
	dev->pf = dev->sensor[index].pf;
	dev->csi_lane = dev->sensor[index].csi_lane;
	dev->pwdn_gpio = dev->sensor[index].pwdn_gpio;
	dev->rst_gpio = dev->sensor[index].rst_gpio;

	isp_ispss_enable_clock(dev);

	mdelay(100);

	isp_ispss_reset_all(dev);

	mdelay(100);

	dev->client = create_isp_i2c_client(dev);
	if (!dev->client)
		return -ENODEV;

	i2c_client = dev->client;
	ret = i2c_client->ops->open(i2c_client);
	if (ret) {
		destroy_isp_i2c_client(i2c_client);
		return -EBUSY;
	}

	isp_csi_enable(dev, dev->csi_lane, 0);

	ret = dev->sensor[index].camera_sensor_power(i2c_client, 1);
	ret += dev->sensor[index].camera_sensor_set_fmt(i2c_client,
			dev->w, dev->h);
	ret += dev->sensor[index].camera_stream_on_off(i2c_client, 1);
	if (ret) {
		dev->sensor[index].camera_sensor_power(i2c_client, 0);
		destroy_isp_i2c_client(i2c_client);
		return -EBUSY;
	}

	isp_clear_irq_state(dev);

	enable_irq(dev->isp_irq_frproc0);
	enable_irq(dev->isp_irq_vic1);

	isp_enable_irq(dev);

	isp_cvdr_init(dev);
	isp_cvdr_config(dev, dev->hw_addr);

	isp_config_smmu_bypass(dev);

	isp_sr_config(dev);
	isp_sr_go(dev, (1 << (4*dev->csi_index)));

	return 0;
}

int isp_stop_streaming(struct isp_device *dev, int index)
{
	struct isp_i2c_client_t *i2c_client = dev->client;

	if (!i2c_client)
		return -ENODEV;

	disable_irq(dev->isp_irq_frproc0);
	disable_irq(dev->isp_irq_vic1);

	dev->sensor[index].camera_stream_on_off(i2c_client, 0);
	dev->sensor[index].camera_sensor_power(i2c_client, 0);
	i2c_client->ops->close(i2c_client);
	destroy_isp_i2c_client(i2c_client);

	return 0;
}
