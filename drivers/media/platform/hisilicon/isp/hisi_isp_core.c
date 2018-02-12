/*
 * Copyright (c) 2016-2017 Linaro Ltd.
 * Copyright (c) 2016-2017 HiSilicon Technologies Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include "hisi_isp_core.h"
#include "hisi_isp_isr.h"
#include "hisi_isp_common.h"
#include <linux/kthread.h>
#include <asm/cacheflush.h>
#include <linux/dma-contiguous.h>

#define DOVDD_VOLTAGE_DIGITAL_IO           1800000

static int isp_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long pfn;
	struct miscdevice *miscdev = file->private_data;
	struct isp_device *dev = container_of(miscdev, struct isp_device, dev);

	ISP_ERR("enter\n");
	if (vma->vm_end - vma->vm_start > dev->pool_size) {
		ISP_ERR("over pool size\n");
		return -EINVAL;
	}

	pfn = __phys_to_pfn(dev->hw_addr);

	return remap_pfn_range(vma, vma->vm_start, pfn + vma->vm_pgoff,
			vma->vm_end - vma->vm_start, vma->vm_page_prot);
}

static int isp_stream_on(struct isp_device *dev, struct stream_info *info)
{
	int total;

	if (info->cam_id >= 2)
		return -EINVAL;

	if (dev->sensor[info->cam_id].probe == false) {
		ISP_ERR("cam_id:%d is not support", info->cam_id);
		return -EINVAL;
	}

	dev->w = info->w;
	dev->h = info->h;
	dev->frame_size = PAGE_ALIGN(info->w * info->h * 2);
	total = dev->pool_size / dev->frame_size;
	if (total < 3) {
		dev->streaming = false;
		ISP_ERR("pool is too small or frame size is too large");
		return -EINVAL;
	}

	dev->frame_count = total;
	dev->isp_frame_num = 0;
	dev->hal_frame_num = 0;
	dev->streaming = true;
	dev->cur_frame_num = 0;

	isp_start_streaming(dev, info->cam_id);

	info->pool_size = dev->pool_size;

	return 0;
}

static void
isp_flush_frame_range(struct isp_device *dev, struct frame_info *info)
{
	int page_num = 0;
	int num = 0;
	struct page *p;

	page_num = dev->frame_size / PAGE_SIZE;
	if (dev->frame_size % PAGE_SIZE > 0)
		page_num++;

	for (num = 0; num < page_num; num++) {
		p = phys_to_page(dev->hw_addr + info->offset + PAGE_SIZE * num);
		flush_dcache_page(p);
	}
}

static int isp_frame_request(struct isp_device *dev, struct frame_info *info)
{

	ISP_ERR("frame request enter %d\n", info->frame_num);

	if (info->frame_num != 0 && info->frame_num != dev->hal_frame_num + 1)
		return -EINVAL;

	if (info->frame_num >= dev->isp_frame_num) {
		ISP_ERR("wait for frame complete+++\n");
		wait_for_completion(&dev->frame_comp);
		ISP_ERR("frame complete---\n");
	}

	info->offset = frame_num2Offset(dev, info->frame_num);

	isp_flush_frame_range(dev, info);

	dev->hal_frame_num = info->frame_num;
	ISP_ERR("frame request exit %d\n", info->frame_num);

	return 0;
}

static int isp_stream_off(struct isp_device *dev, struct stream_info *info)
{
	if (info->cam_id >= 2)
		return -EINVAL;

	if (dev->sensor[info->cam_id].probe == false)
		return -EINVAL;

	isp_stop_streaming(dev, info->cam_id);

	dev->streaming = false;

	return 0;
}

static long isp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *miscdev = filp->private_data;
	struct isp_device *dev = container_of(miscdev, struct isp_device, dev);
	int ret;
	struct stream_info stream;
	struct frame_info frame;

	if (!dev)
		return -ENODEV;

	switch (cmd) {
	case ISP_IOC_STREAM_ON:
		{
			if (dev->streaming == true)
				return -EINVAL;

			if (copy_from_user(&stream, (void __user *)arg,
						sizeof(stream))) {
				return -EFAULT;
			}
			ret = isp_stream_on(dev, &stream);
			if (ret)
				return -EFAULT;
			if (copy_to_user((void __user *)arg,
						&stream, _IOC_SIZE(cmd))) {
				ISP_ERR("copy to user failed!\n");
				return -EFAULT;
			}
			break;
		}
	case ISP_IOC_FRAME_REQUEST:
		{
			if (dev->streaming == false)
				return -EINVAL;

			if (copy_from_user(&frame, (void __user *)arg,
						sizeof(frame))) {
				return -EFAULT;
			}

			ret = isp_frame_request(dev, &frame);
			if (ret)
				return -EFAULT;

			if (copy_to_user((void __user *)arg,
						&frame, _IOC_SIZE(cmd))) {
				ISP_ERR("copy to user failed!\n");
				return -EFAULT;
			}

			break;
		}
	case ISP_IOC_STREAM_OFF:
		{
			if (dev->streaming == false)
				return -EINVAL;

			if (copy_from_user(&stream, (void __user *)arg,
						sizeof(stream))) {
				return -EFAULT;
			}
			ret = isp_stream_off(dev, &stream);
			if (ret)
				return -EFAULT;
			if (copy_to_user((void __user *)arg,
						&stream, _IOC_SIZE(cmd))) {
				ISP_ERR("copy to user failed!\n");
				return -EFAULT;
			}
			break;
		}
	default:
		{
			ISP_ERR("unknown cmd!\n");
			return -ENOTTY;
		}
	}

	return 0;
}

static int isp_release(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct isp_device *dev = container_of(miscdev, struct isp_device, dev);

	dev->streaming = false;

	return 0;
}

static int isp_open(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations isp_fops = {
	.owner          = THIS_MODULE,
	.open           = isp_open,
	.release        = isp_release,
	.unlocked_ioctl = isp_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = NULL,
#endif
	.mmap           = isp_mmap,
};

static int isp_getdts(struct platform_device *pdev, struct isp_device *dev)
{
	struct device *device = &pdev->dev;
	struct device_node *np = device->of_node;

	if (!np)
		return -ENODEV;

	dev->isp_irq_vic1 = irq_of_parse_and_map(np, 0);
	if (!dev->isp_irq_vic1)
		return -ENXIO;

	dev->isp_irq_frproc0 = irq_of_parse_and_map(np, 1);
	if (!dev->isp_irq_frproc0)
		return -ENXIO;

	of_property_read_u32(np, "clock-rates",
			&dev->xclk_freq);

	of_property_read_u32(np, "pool-size",
			&dev->pool_size);

	ISP_INFO("clock rate %d, pool size 0x%x",
			dev->xclk_freq, dev->pool_size);

	return 0;
}

static int isp_probe(struct platform_device *pdev)
{
	int ret;
	struct isp_device *idev;
	struct device *device = &pdev->dev;
	struct device_node *np = device->of_node;

	ISP_INFO("enter!\n");

	idev = devm_kzalloc(&pdev->dev, sizeof(*idev), GFP_KERNEL);
	if (!idev)
		return -ENOMEM;

	if (isp_getdts(pdev, idev)) {
		ISP_ERR("failed to dts info.\n");
		return -EINVAL;
	}

	idev->base = of_iomap(np, 0);
	if (!idev->base) {
		ISP_ERR("failed to get base resource.\n");
		return -ENXIO;
	}

	ret = devm_request_irq(&pdev->dev, idev->isp_irq_vic1,
			isp_vic1_handler, 0, "isp-vic1", idev);
	if (ret)
		return ret;

	ret = devm_request_irq(&pdev->dev, idev->isp_irq_frproc0,
			isp_frproc0_handler, 0, "isp-frproc0", idev);
	if (ret)
		return ret;

	disable_irq(idev->isp_irq_frproc0);
	disable_irq(idev->isp_irq_vic1);

	idev->io_regulator = devm_regulator_get(&pdev->dev, "DOVDD");
	if (IS_ERR(idev->io_regulator)) {
		idev->io_regulator = NULL;
		return -EINVAL;
	}
	regulator_set_voltage(idev->io_regulator,
			DOVDD_VOLTAGE_DIGITAL_IO,
			DOVDD_VOLTAGE_DIGITAL_IO);

	idev->xclk = devm_clk_get(&pdev->dev, "clk_gate_isp_snclk0");
	if (IS_ERR(idev->xclk)) {
		ISP_ERR("could not get xclk\n");
		return -EINVAL;
	}
	clk_set_rate(idev->xclk, idev->xclk_freq);

	idev->xclk1 = devm_clk_get(&pdev->dev, "clk_gate_isp_snclk1");
	if (IS_ERR(idev->xclk1)) {
		ISP_ERR("could not get xclk1\n");
		return -EINVAL;
	}
	clk_set_rate(idev->xclk1, idev->xclk_freq);

	idev->virt_addr = dma_alloc_coherent(device,
			idev->pool_size, &(idev->hw_addr), GFP_KERNEL);
	if (!idev->virt_addr) {
		ISP_ERR("dma_alloc_coherent (ISP) alloc err!\n");
		return -ENOMEM;
	}

	memset(idev->virt_addr, 0, idev->pool_size);

	idev->dev.minor = MISC_DYNAMIC_MINOR;
	idev->dev.name = "isp";
	idev->dev.fops = &isp_fops;
	idev->dev.parent = NULL;
	ret = misc_register(&idev->dev);
	if (ret) {
		dma_free_coherent(device,
			idev->pool_size, idev->virt_addr, idev->hw_addr);
		return ret;
	}

	init_completion(&idev->frame_comp);

	platform_set_drvdata(pdev, idev);

	ISP_INFO("exit!\n");

	return 0;
}

static int isp_remove(struct platform_device *pdev)
{
	struct device *device = &pdev->dev;
	struct isp_device *idev = platform_get_drvdata(pdev);

	if (!idev) {
		pr_err("%s: get drvdata failed\n", __func__);
		return -EINVAL;
	}
	ISP_INFO("enter!\n");

	dma_free_coherent(device,
			idev->pool_size, idev->virt_addr, idev->hw_addr);

	misc_deregister(&idev->dev);

	ISP_INFO("exit!\n");
	return 0;
}

static const struct of_device_id hisi_isp_match_table[] = {
	{
		.compatible = COMP_HISI_ISP_NAME,
		.data = NULL,
	},
	{},
};
MODULE_DEVICE_TABLE(of, hisi_isp_match_table);

static struct platform_driver isp_driver = {
	.probe = isp_probe,
	.remove = isp_remove,
	.driver = {
		.name = DEV_NAME_ISP,
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(hisi_isp_match_table),
	},
};

static int __init isp_init(void)
{
	int ret = 0;

	ISP_INFO("enter!\n");

	ret = platform_driver_register(&isp_driver);
	if (ret)
		return ret;

	ISP_INFO("exit!\n");

	return ret;
}

static void __exit isp_exit(void)
{
	ISP_INFO("enter!\n");

	platform_driver_unregister(&isp_driver);

	ISP_INFO("exit!\n");
}

module_init(isp_init);
module_exit(isp_exit);

MODULE_DESCRIPTION("Hisilicon Isp Driver");
MODULE_LICENSE("GPL v2");
