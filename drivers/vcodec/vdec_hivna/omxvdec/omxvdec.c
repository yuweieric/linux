/*
 * vdec driver interface
 *
 * Copyright (c) 2017 Hisilicon Limited
 *
 * Author: gaoyajun<gaoyajun@hisilicon.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation.
 *
 */

#include <linux/clk.h>
#include <linux/of.h>
#include <linux/io.h>

#include "omxvdec.h"
#include "../vfmw_v4.0/linux_kernel_osal.h"
#include "../vfmw_v4.0/vfmw_dts.h"
#include "../vfmw_v4.0/scd_drv.h"
#include "../vfmw_v4.0/format/vdm_drv.h"
#include "../vfmw_v4.0/vfmw_intf.h"
/*lint -e774*/

#define PCTRL_PERI              (0xE8A090A4)
#define PCTRL_PERI_SATA0        (0xE8A090BC)

static int gIsNormalInit = 0;

static int       gIsDeviceDetected  = 0;
static struct class *g_OmxVdecClass     = NULL;
static const char g_OmxVdecDrvName[] = OMXVDEC_NAME;
static dev_t         g_OmxVdecDevNum;
static OMXVDEC_ENTRY g_OmxVdecEntry;
 
//Modified for 64-bit platform
typedef enum {
	T_IOCTL_ARG,
	T_IOCTL_ARG_COMPAT,
	T_BUTT,
} COMPAT_TYPE_E;

typedef enum {
	KIRIN_970_ES,
	KIRIN_970_CS,
	KIRIN_980,
	KIRIN_BUTT,
} KIRIN_PLATFORM_E;

#define CHECK_PARA_SIZE_RETURN(size, para_size, command)                 \
do {                                                                     \
	if (size != para_size) {                                         \
		printk(KERN_CRIT "%s: prarameter_size is error\n", command); \
		return -EINVAL;                                             \
	}                                                                \
}while(0)

#define CHECK_RETURN(cond, else_print)                                    \
do {                                                                     \
	if (!(cond)) {                                                    \
		printk(KERN_CRIT "%s : %s\n", __func__, else_print);    \
		return -EINVAL;                                          \
	}                                                                \
}while(0)

#define CHECK_SCENE_EQ_RETURN(cond, else_print)                                    \
do {                                                                     \
	if (cond) {                                                    \
		printk(KERN_INFO "%s : %s\n", __func__, else_print);    \
		return -EIO;                                          \
	}                                                                \
}while(0)
 
static int omxvdec_setup_cdev(OMXVDEC_ENTRY *omxvdec, const struct file_operations *fops)
{
	int rc;
	struct device *dev;

	g_OmxVdecClass = class_create(THIS_MODULE, "omxvdec_class");
	if (IS_ERR(g_OmxVdecClass)) {
		rc = PTR_ERR(g_OmxVdecClass);
		g_OmxVdecClass = NULL;
		printk(KERN_CRIT "%s call class_create failed, rc : %d\n", __func__, rc);
		return rc;
	}

	rc = alloc_chrdev_region(&g_OmxVdecDevNum, 0, 1, "hisi video decoder");
	if (rc) {
		printk(KERN_CRIT "%s call alloc_chrdev_region failed, rc : %d\n", __func__, rc);
		goto cls_destroy;
	}

	dev = device_create(g_OmxVdecClass, NULL, g_OmxVdecDevNum, NULL, OMXVDEC_NAME);
	if (IS_ERR(dev)) {
		rc = PTR_ERR(dev);
		printk(KERN_CRIT "%s call device_create failed, rc : %d\n", __func__, rc);
		goto unregister_region;
	}

	cdev_init(&omxvdec->cdev, fops);
	omxvdec->cdev.owner = THIS_MODULE;
	omxvdec->cdev.ops = fops;
	rc = cdev_add(&omxvdec->cdev, g_OmxVdecDevNum, 1);
	if (rc < 0) {
		printk(KERN_CRIT "%s call cdev_add failed, rc : %d\n", __func__, rc);
		goto dev_destroy;
	}

	return HI_SUCCESS;

dev_destroy:
	device_destroy(g_OmxVdecClass, g_OmxVdecDevNum);
unregister_region:
	unregister_chrdev_region(g_OmxVdecDevNum, 1);
cls_destroy:
	class_destroy(g_OmxVdecClass);
	g_OmxVdecClass = NULL;

	return rc;
}

static int omxvdec_cleanup_cdev(OMXVDEC_ENTRY *omxvdec)
{
	if (!g_OmxVdecClass) {
		printk(KERN_CRIT "%s: Invalid g_OmxVdecClass is NULL", __func__);
		return HI_FAILURE;
	}

	cdev_del(&omxvdec->cdev);
	device_destroy(g_OmxVdecClass, g_OmxVdecDevNum);
	unregister_chrdev_region(g_OmxVdecDevNum, 1);
	class_destroy(g_OmxVdecClass);
	g_OmxVdecClass = NULL;

	return HI_SUCCESS;
}

static int omxvdec_open(struct inode *inode, struct file *file)
{
	int ret = -EBUSY;
	OMXVDEC_ENTRY *omxvdec = NULL;

	omxvdec = container_of(inode->i_cdev, OMXVDEC_ENTRY, cdev);

	VDEC_MUTEX_LOCK(&omxvdec->omxvdec_mutex);
	VDEC_MUTEX_LOCK(&omxvdec->vdec_mutex_scd);
	VDEC_MUTEX_LOCK(&omxvdec->vdec_mutex_vdh);

	if (omxvdec->open_count < MAX_OPEN_COUNT) {
		omxvdec->open_count++;
		if (omxvdec->open_count == 1) {
			ret = VDEC_Regulator_Enable();
			if (ret != HI_SUCCESS) {
				printk(KERN_CRIT "%s : VDEC_Regulator_Enable failed\n", __func__);
				goto error0;
			}
			ret = VCTRL_OpenVfmw();
			if (ret != HI_SUCCESS) {
				printk(KERN_CRIT "%s : vfmw open failed\n", __func__);
				goto error1;
			}
			gIsNormalInit = 1;
		}

		file->private_data = omxvdec;
		ret = HI_SUCCESS;
	} else {
		printk(KERN_CRIT "%s open omxvdec instance too much\n", __func__);
		ret = -EBUSY;
	}

	printk(KERN_INFO "%s, open_count : %d\n", __func__, omxvdec->open_count);
	goto exit;

error1:
	(void)VDEC_Regulator_Disable();
error0:
	omxvdec->open_count--;
exit:
	VDEC_MUTEX_UNLOCK(&omxvdec->vdec_mutex_vdh);
	VDEC_MUTEX_UNLOCK(&omxvdec->vdec_mutex_scd);
	VDEC_MUTEX_UNLOCK(&omxvdec->omxvdec_mutex);

	return ret;
}

static int omxvdec_release(struct inode *inode, struct file *file)
{
	OMXVDEC_ENTRY *omxvdec = NULL;
	int ret = HI_SUCCESS;

	omxvdec = file->private_data;
	if (omxvdec == NULL) {
		printk(KERN_CRIT  "%s: invalid omxvdec is null\n", __func__);
		return -EFAULT;
	}

	VDEC_MUTEX_LOCK(&omxvdec->omxvdec_mutex);
	VDEC_MUTEX_LOCK(&omxvdec->vdec_mutex_scd);
	VDEC_MUTEX_LOCK(&omxvdec->vdec_mutex_vdh);

	if (file->private_data == NULL) {
		printk(KERN_CRIT "%s: invalid file->private_data is null\n", __func__);
		ret = -EFAULT;
		goto exit;
	}
 
	if (omxvdec->open_count > 0)
		omxvdec->open_count--;

	if (omxvdec->open_count == 0) {
		VCTRL_CloseVfmw();
		VDEC_Regulator_Disable();
		gIsNormalInit = 0;
	}
 	file->private_data = NULL;

	printk(KERN_INFO "exit %s , open_count : %d\n", __func__, omxvdec->open_count);
exit:
	VDEC_MUTEX_UNLOCK(&omxvdec->vdec_mutex_vdh);
	VDEC_MUTEX_UNLOCK(&omxvdec->vdec_mutex_scd);
	VDEC_MUTEX_UNLOCK(&omxvdec->omxvdec_mutex);

	return ret;
}

/* Modified for 64-bit platform */
static int omxvdec_compat_get_data(COMPAT_TYPE_E eType, void __user *pUser, void *pData)
{
	int ret = HI_SUCCESS;
	int s32Data = 0;
	compat_ulong_t CompatData = 0;
	OMXVDEC_IOCTL_MSG *pIoctlMsg = (OMXVDEC_IOCTL_MSG *)pData;

	if (NULL == pUser || NULL == pData) {
		printk(KERN_CRIT "%s: param is null\n", __func__);
		return HI_FAILURE;
	}

	switch (eType) {
	case T_IOCTL_ARG:
		if (copy_from_user(pIoctlMsg, pUser, sizeof(*pIoctlMsg))) {
			printk(KERN_CRIT "%s puser copy failed\n", __func__);
			ret = HI_FAILURE;
		}
		break;
	case T_IOCTL_ARG_COMPAT: {
		COMPAT_IOCTL_MSG __user *pCompatMsg = pUser;

		ret |= get_user(s32Data, &(pCompatMsg->chan_num));
		pIoctlMsg->chan_num = s32Data;

		ret |= get_user(s32Data, &(pCompatMsg->in_size));
		pIoctlMsg->in_size = s32Data;

		ret |= get_user(s32Data, &(pCompatMsg->out_size));
		pIoctlMsg->out_size = s32Data;

		ret |= get_user(CompatData, &(pCompatMsg->in));
		pIoctlMsg->in   = (void *) ((unsigned long)CompatData);

		ret |= get_user(CompatData, &(pCompatMsg->out));
		pIoctlMsg->out = (void *) ((unsigned long)CompatData);
	}
		break;
	default:
		printk(KERN_CRIT "%s: unkown type %d\n", __func__, eType);
		ret = HI_FAILURE;
		break;
	}

	return ret;
}

static long omxvdec_ioctl_common(struct file *file, unsigned int cmd, unsigned long arg, COMPAT_TYPE_E type)
{
	int ret;
	int x_scene;
	OMXVDEC_IOCTL_MSG  vdec_msg;
	void           *u_arg   = (void *)arg;
	OMXVDEC_ENTRY     *omxvdec = file->private_data;

	OMXSCD_REG_CFG_S  scd_reg_cfg;
	SCD_STATE_REG_S   scd_state_reg;
	OMXVDH_REG_CFG_S  vdm_reg_cfg;
	VDMHAL_BACKUP_S   vdm_state_reg;
	int            vdm_is_run;

	x_scene = VCTRL_Scen_Ident(cmd);
	CHECK_SCENE_EQ_RETURN(x_scene == 1, "xxx scene");

	CHECK_RETURN(omxvdec != NULL, "omxvdec is null");

	ret = omxvdec_compat_get_data(type, u_arg, &vdec_msg);
	CHECK_RETURN(ret == HI_SUCCESS, "compat data get failed");

	switch (cmd) {
	case VDEC_IOCTL_VDM_PROC:
		CHECK_PARA_SIZE_RETURN(sizeof(vdm_reg_cfg), vdec_msg.in_size, "VDEC_IOCTL_VDM_PROC_IN");
		CHECK_PARA_SIZE_RETURN(sizeof(vdm_state_reg), vdec_msg.out_size, "VDEC_IOCTL_VDM_PROC_OUT");
		if (copy_from_user(&vdm_reg_cfg, vdec_msg.in, sizeof(vdm_reg_cfg))) {
			printk(KERN_CRIT "VDEC_IOCTL_VDM_PROC : copy_from_user failed\n");
			return -EFAULT;
		}

		VDEC_MUTEX_LOCK(&omxvdec->vdec_mutex_vdh);
		dsb(sy);

		ret = VCTRL_VDMHal_Process(&vdm_reg_cfg, &vdm_state_reg);
		if (ret != HI_SUCCESS) {
			printk(KERN_CRIT "VCTRL_VDMHal_Process failed\n");
			VDEC_MUTEX_UNLOCK(&omxvdec->vdec_mutex_vdh);
			return -EIO;
		}

		VDEC_MUTEX_UNLOCK(&omxvdec->vdec_mutex_vdh);
		if (copy_to_user(vdec_msg.out, &vdm_state_reg, sizeof(vdm_state_reg))) {
			printk(KERN_CRIT "VDEC_IOCTL_VDM_PROC : copy_to_user failed\n");
			return -EFAULT;
		}
		break;

	case VDEC_IOCTL_GET_VDM_HWSTATE:
		CHECK_PARA_SIZE_RETURN(sizeof(vdm_is_run), vdec_msg.out_size, "VDEC_IOCTL_GET_VDM_HWSTATE");
		VDEC_MUTEX_LOCK(&omxvdec->vdec_mutex_vdh);
		vdm_is_run = VCTRL_VDMHAL_IsRun();

		VDEC_MUTEX_UNLOCK(&omxvdec->vdec_mutex_vdh);
		if (copy_to_user(vdec_msg.out, &vdm_is_run, sizeof(vdm_is_run))) {
			printk(KERN_CRIT "VDEC_IOCTL_GET_VDM_HWSTATE : copy_to_user failed\n");
			return -EFAULT;
		}
		break;

	case VDEC_IOCTL_SCD_PROC:
		CHECK_PARA_SIZE_RETURN(sizeof(scd_reg_cfg), vdec_msg.in_size, "VDEC_IOCTL_SCD_PROC_IN");
		CHECK_PARA_SIZE_RETURN(sizeof(scd_state_reg), vdec_msg.out_size, "VDEC_IOCTL_SCD_PROC_OUT");
		if (copy_from_user(&scd_reg_cfg, vdec_msg.in, sizeof(scd_reg_cfg))) {
			printk(KERN_CRIT "VDEC_IOCTL_SCD_PROC :  copy_from_user failed\n");
			return -EFAULT;
		}

		VDEC_MUTEX_LOCK(&omxvdec->vdec_mutex_scd);
		dsb(sy);

		ret = VCTRL_SCDHal_Process(&scd_reg_cfg, &scd_state_reg);
		if (ret != HI_SUCCESS) {
			printk(KERN_CRIT "VCTRL_SCDHal_Process failed\n");
			VDEC_MUTEX_UNLOCK(&omxvdec->vdec_mutex_scd);
			return -EIO;
		}

		VDEC_MUTEX_UNLOCK(&omxvdec->vdec_mutex_scd);
		if (copy_to_user(vdec_msg.out, &scd_state_reg, sizeof(scd_state_reg))) {
			printk(KERN_CRIT "VDEC_IOCTL_SCD_PROC : copy_to_user failed\n");
			return -EFAULT;
		}
		break;

	default:
		/* could not handle ioctl */
		printk(KERN_CRIT "%s %d:  cmd : %d is not supported\n", __func__, __LINE__, _IOC_NR(cmd));
		return -ENOTTY;
	}

	return 0;
}

static long omxvdec_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return omxvdec_ioctl_common(file, cmd, arg, T_IOCTL_ARG);
}
/* Modified for 64-bit platform */
#ifdef CONFIG_COMPAT
static long omxvdec_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void *user_ptr = compat_ptr(arg);
	return  omxvdec_ioctl_common(file, cmd, (unsigned long)user_ptr, T_IOCTL_ARG_COMPAT);
}
#endif

static const struct file_operations omxvdec_fops = {
	.owner          = THIS_MODULE,
	.open           = omxvdec_open,
	.unlocked_ioctl = omxvdec_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = omxvdec_compat_ioctl,
#endif
	.release        = omxvdec_release,
};

static int omxvdec_probe(struct platform_device *pltdev)
{
	int ret;

	if (gIsDeviceDetected == 1) {
		printk(KERN_DEBUG "Already probe omxvdec\n");
		return 0;
	}

	platform_set_drvdata(pltdev, NULL);

	memset(&g_OmxVdecEntry, 0, sizeof(OMXVDEC_ENTRY)); /* unsafe_function_ignore: memset */
 	VDEC_INIT_MUTEX(&g_OmxVdecEntry.omxvdec_mutex);
	VDEC_INIT_MUTEX(&g_OmxVdecEntry.vdec_mutex_scd);
	VDEC_INIT_MUTEX(&g_OmxVdecEntry.vdec_mutex_vdh);

	ret = omxvdec_setup_cdev(&g_OmxVdecEntry, &omxvdec_fops);
	if (ret < 0) {
		printk(KERN_CRIT "%s call omxvdec_setup_cdev failed\n", __func__);
		goto cleanup0;
	}

	ret = VDEC_Regulator_Probe(&pltdev->dev);
	if (ret != HI_SUCCESS) {
		printk(KERN_CRIT "%s call Regulator_Initialize failed\n", __func__);
		goto cleanup1;
	}

	g_OmxVdecEntry.device = &pltdev->dev;
	platform_set_drvdata(pltdev, &g_OmxVdecEntry);
	gIsDeviceDetected = 1;

	return 0;

cleanup1:
	omxvdec_cleanup_cdev(&g_OmxVdecEntry);

cleanup0:
	return ret;
}

static int omxvdec_remove(struct platform_device *pltdev)
{
	OMXVDEC_ENTRY *omxvdec = NULL;

	omxvdec = platform_get_drvdata(pltdev);
	if (omxvdec != NULL) {
		if (IS_ERR(omxvdec)) {
			printk(KERN_ERR "call platform_get_drvdata err, errno : %ld\n", PTR_ERR(omxvdec));
		} else {
			omxvdec_cleanup_cdev(omxvdec);
			VDEC_Regulator_Remove(&pltdev->dev);
			platform_set_drvdata(pltdev, NULL);
			gIsDeviceDetected = 0;
		}
	}

	return 0;
}

static int omxvdec_suspend(struct platform_device *pltdev, pm_message_t state)
{
	int ret;

	printk(KERN_INFO "%s enter\n", __func__);

	if (gIsNormalInit != 0)
		VCTRL_Suspend();

	ret = VDEC_Regulator_Disable();
	if (ret != HI_SUCCESS)
		printk(KERN_CRIT "%s disable regulator failed\n", __func__);

	printk(KERN_INFO "%s success\n", __func__);

	return HI_SUCCESS;
}

static int omxvdec_resume(struct platform_device *pltdev)
{
	int ret;
	CLK_RATE_E resume_clk = VDEC_CLK_RATE_NORMAL;

	printk(KERN_INFO "%s enter\n", __func__);
	VDEC_Regulator_GetClkRate(&resume_clk);

	if (gIsNormalInit != 0) {
		ret = VDEC_Regulator_Enable();
		if (ret != HI_SUCCESS) {
			printk(KERN_CRIT "%s enable regulator failed\n", __func__);
			return HI_FAILURE;
		}

		ret = VDEC_Regulator_SetClkRate(resume_clk);
		if (ret != HI_SUCCESS)
		{
			printk(KERN_CRIT "%s, set clk failed\n", __func__);
		}

		VCTRL_Resume();
	}

	printk(KERN_INFO "%s success\n", __func__);

	return HI_SUCCESS;
}

static void omxvdec_device_release(struct device *dev)
{
	return;
}

static struct platform_driver omxvdec_driver = {

	.probe   = omxvdec_probe,
	.remove  = omxvdec_remove,
	.suspend = omxvdec_suspend,
	.resume  = omxvdec_resume,
	.driver  = {
		.name  = (char*) g_OmxVdecDrvName,
		.owner = THIS_MODULE,
		.of_match_table = Hisi_Vdec_Match_Table
	},
};

static struct platform_device omxvdec_device = {

	.name = g_OmxVdecDrvName,
	.id   = -1,
	.dev  = {
		.platform_data = NULL,
		.release       = omxvdec_device_release,
	},
};

int __init OMXVDEC_DRV_ModInit(void)
{
	int ret;

	ret = platform_device_register(&omxvdec_device);
	if (ret < 0) {
		printk(KERN_CRIT "%s call platform_device_register failed\n", __func__);
		return ret;
	}

	ret = platform_driver_register(&omxvdec_driver);
	if (ret < 0) {
		printk(KERN_CRIT "%s call platform_driver_register failed\n", __func__);
		goto exit;
	}
#ifdef MODULE
	printk(KERN_INFO "Load hi_omxvdec.ko :%d success\n", OMXVDEC_VERSION);
#endif

	return HI_SUCCESS;
exit:
	platform_device_unregister(&omxvdec_device);

	return ret;
}

void __exit OMXVDEC_DRV_ModExit(void)
{
	platform_driver_unregister(&omxvdec_driver);
	platform_device_unregister(&omxvdec_device);

#ifdef MODULE
	printk(KERN_INFO "Unload hi_omxvdec.ko : %d success\n", OMXVDEC_VERSION);
#endif

}

module_init(OMXVDEC_DRV_ModInit);
module_exit(OMXVDEC_DRV_ModExit);

MODULE_AUTHOR("gaoyajun@hisilicon.com");
MODULE_DESCRIPTION("vdec driver");
MODULE_LICENSE("GPL");
