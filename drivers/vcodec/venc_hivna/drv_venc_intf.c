
/*
* Copyright (C), 2001-2011, Hisilicon Tech. Co., Ltd.
*
* File Name	 : viu.c
* Version	   : Initial Draft
* Author		: Hisilicon multimedia software group
* Created	   :
* Description   :
*
* History	   :
* 1.Date		: 2010/03/17
* Author	  : j00131665
* Modification: Created file
*/
#include <linux/device.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>

#include "drv_venc_osal.h"
#include "drv_venc.h"
#include "venc_regulator.h"
#include "drv_venc.h"
#define VERSION_STRING    "1234"
#define PCTRL_PERI 0xE8A090A4
#define PCTRL_PERI_SATA0 (0xE8A090BC)
#define MAX_OPEN_COUNT  3
/*lint -e750 -e838 -e715*/
#ifndef VM_RESERVED /*for kernel up to 3.7.0 version*/
# define  VM_RESERVED   (VM_DONTEXPAND | VM_DONTDUMP)
#endif

/*============Deviece===============*/
typedef struct {
	dev_t dev;
	struct device* venc_device;
	//struct device* venc_device_2;
	struct cdev cdev;
	struct class* venc_class;
}VENC_ENTRY;

typedef enum {
	KIRIN_960,
	KIRIN_970_ES,
	KIRIN_970_CS,
}KIRIN_PLATFORM_E;

typedef struct {
	MEM_BUFFER_S internalbuffer;
	MEM_BUFFER_S imagebuffer;
	MEM_BUFFER_S streambuffer[MAX_STREAMBUF_NUM];
	MEM_BUFFER_S streamheadbuffer;
}VENC_MEM_INFO;

struct semaphore g_VencMutex;

static int g_vencOpenFlag = 0;
static int g_vencDevDetected = 0;

//VENC device open times
atomic_t g_VencCount = ATOMIC_INIT(0);

int VENC_DRV_Resume(struct platform_device *pltdev);
int VENC_DRV_Suspend(struct platform_device *pltdev, pm_message_t state);

static int  VENC_DRV_SetupCdev(VENC_ENTRY *venc, const struct file_operations *fops);
static int  VENC_DRV_CleanupCdev(VENC_ENTRY *venc);
static int  VENC_DRV_Probe(struct platform_device * pltdev);
static int  VENC_DRV_Remove(struct platform_device *pltdev);

extern VeduEfl_IpCtx_S   VeduIpCtx;
extern U_FUNC_VCPI_RAWINT  g_hw_done_type;
extern VEDU_OSAL_EVENT   g_hw_done_event;
extern unsigned int gVencIsFPGA;

static int venc_drv_waithwdone(U_FUNC_VCPI_RAWINT *hw_done_type)
{
	int Ret = HI_FAILURE;

	Ret = VENC_DRV_OsalWaitEvent(&g_hw_done_event, msecs_to_jiffies(500));/*lint !e712 !e747 */

	if (Ret != 0) {
		hw_done_type->u32 = 0;
		HI_ERR_VENC("wait timeout, Ret value is %d\n", Ret);
		return Ret;
	}

	*hw_done_type = g_hw_done_type;
	return Ret;
}

static int venc_drv_register_info(VENC_REG_INFO_S *regcfginfo)
{
	int Ret = HI_SUCCESS;
	CMD_TYPE cmd  = regcfginfo->cmd;
	switch (cmd) {
	case VENC_SET_CFGREG:
		if (regcfginfo->bResetReg == 1)
		{
			Ret = VENC_HAL_ResetReg();
			if (Ret != HI_SUCCESS)
			{
				HI_ERR_VENC("reset venc hal reset reg, Ret:%d\n", Ret);
				break;
			}
		}

		VeduHal_CfgReg(regcfginfo);

		VENC_HAL_StartEncode((S_HEVC_AVC_REGS_TYPE*)(VeduIpCtx.pRegBase));/*lint !e826 */

		Ret = venc_drv_waithwdone(&regcfginfo->hw_done_type) ;

		if((Ret == HI_SUCCESS ) && (!regcfginfo->hw_done_type.bits.vcpi_rint_vedu_timeout))
		{
			VENC_HAL_Get_Reg_Venc(regcfginfo);
			HI_DBG_VENC("get venc hal reg info\n");
		}
		break;
	case VENC_SET_CFGREGSIMPLE:
		VeduHal_CfgRegSimple(regcfginfo);

		VENC_HAL_StartEncode((S_HEVC_AVC_REGS_TYPE*)(VeduIpCtx.pRegBase));/*lint !e826 */

		Ret = venc_drv_waithwdone(&regcfginfo->hw_done_type) ;

		if((Ret == HI_SUCCESS ) && (!regcfginfo->hw_done_type.bits.vcpi_rint_vedu_timeout))
		{
			VENC_HAL_Get_Reg_Venc(regcfginfo);
			HI_DBG_VENC("get venc hal reg info\n");
		}
		break;
	default:
		HI_ERR_VENC("cmd type unknown:0x%x in default case\n", cmd);
		Ret = HI_FAILURE;
		break;
	}
	return Ret;
}
static int VENC_DRV_Open(struct inode *finode, struct file  *ffile)
{
	int Ret = 0;

	Ret = HiVENC_DOWN_INTERRUPTIBLE(&g_VencMutex);
	if (Ret) {
		HI_FATAL_VENC("Open down interruptible failed\n");
		return HI_FAILURE;
	}

	if (atomic_read(&g_VencCount) == MAX_OPEN_COUNT) {
		HI_FATAL_VENC("open venc too much\n");
		HiVENC_UP_INTERRUPTIBLE(&g_VencMutex);
		return -EAGAIN;
	}

	if (atomic_inc_return(&g_VencCount) == 1) {
		Ret = VENC_DRV_BoardInit();
		if (Ret != HI_SUCCESS) {
			HI_FATAL_VENC("board init failed, ret value is %d\n", Ret);
			atomic_dec(&g_VencCount);
			HiVENC_UP_INTERRUPTIBLE(&g_VencMutex);
			return HI_FAILURE;
		}
		Ret = VENC_DRV_EflOpenVedu();
		if (Ret != HI_SUCCESS) {
			HI_FATAL_VENC("venc firmware layer open failed, ret value is %d\n", Ret);
			atomic_dec(&g_VencCount);
			VENC_DRV_BoardDeinit();
			HiVENC_UP_INTERRUPTIBLE(&g_VencMutex);
			return HI_FAILURE;
		}
	}

	g_vencOpenFlag = 1;
	HiVENC_UP_INTERRUPTIBLE(&g_VencMutex);

	HI_INFO_VENC("Open venc device successfully\n");
	return HI_SUCCESS;
}

static int VENC_DRV_Close(struct inode *finode, struct file  *ffile)
{
	int Ret = 0;

	Ret = HiVENC_DOWN_INTERRUPTIBLE(&g_VencMutex);
	if (Ret) {
		HI_FATAL_VENC("Close down interruptible failed\n");
		return HI_FAILURE;
	}

	if (atomic_dec_and_test(&g_VencCount)) {
		Ret = VENC_DRV_EflCloseVedu();
		if (Ret != HI_SUCCESS) {
			HI_FATAL_VENC("venc firmware layer close failed, ret value is %d\n", Ret);
			VENC_DRV_BoardDeinit();
			HiVENC_UP_INTERRUPTIBLE(&g_VencMutex);
			return HI_FAILURE;
		}

		VENC_DRV_BoardDeinit();
		g_vencOpenFlag = 0;
	}

	HiVENC_UP_INTERRUPTIBLE(&g_VencMutex);

	HI_INFO_VENC("Close venc device successfully\n");
	return HI_SUCCESS;
}

int VENC_DRV_Suspend(struct platform_device *pltdev, pm_message_t state)
{
	int Ret = 0;
	HI_INFO_VENC("enter\n");

	Ret = HiVENC_DOWN_INTERRUPTIBLE(&g_VencMutex);
	if (Ret) {
		HI_ERR_VENC("Suspend down interruptible failed\n");
		return HI_FAILURE;
	}

	if (!g_vencOpenFlag) {
		HI_INFO_VENC("venc device already suspend\n");
		HiVENC_UP_INTERRUPTIBLE(&g_VencMutex);
		return HI_SUCCESS;
	}

	Ret = VENC_DRV_EflSuspendVedu();
	if (Ret != HI_SUCCESS) {
		HI_FATAL_VENC("venc firmware layer suspend failed, ret value is %d\n", Ret);
		HiVENC_UP_INTERRUPTIBLE(&g_VencMutex);
		return HI_FAILURE;
	}

	VENC_DRV_BoardDeinit();
	g_hw_done_event.flag = 0;
	HiVENC_UP_INTERRUPTIBLE(&g_VencMutex);

	HI_INFO_VENC("exit\n");
	return HI_SUCCESS;
}/*lint !e715*/

int VENC_DRV_Resume(struct platform_device *pltdev)
{
	int Ret = 0;
	HI_INFO_VENC("enter\n");

	Ret = HiVENC_DOWN_INTERRUPTIBLE(&g_VencMutex);
	if (Ret) {
		HI_FATAL_VENC("Resume down interruptible failed\n");
		return HI_FAILURE;
	}

	if (!g_vencOpenFlag) {
		HI_INFO_VENC("venc device already resume\n");
		HiVENC_UP_INTERRUPTIBLE(&g_VencMutex);
		return 0;
	}
	Ret = VENC_DRV_BoardInit();
	if (Ret != HI_SUCCESS) {
		HI_FATAL_VENC("board init failed, ret value is %d\n", Ret);
		atomic_dec(&g_VencCount);
		HiVENC_UP_INTERRUPTIBLE(&g_VencMutex);
		return HI_FAILURE;
	}
	Ret = VENC_DRV_EflResumeVedu();
	if (Ret != HI_SUCCESS) {
		HI_FATAL_VENC("venc firmware layer resume failed, ret value is %d\n", Ret);
		atomic_dec(&g_VencCount);
		HiVENC_UP_INTERRUPTIBLE(&g_VencMutex);
		return HI_FAILURE;
	}

	HiVENC_UP_INTERRUPTIBLE(&g_VencMutex);
	HI_INFO_VENC("exit\n");
	return HI_SUCCESS;
}/*lint !e715*/

static long VENC_Ioctl(struct file *file, unsigned int ucmd, unsigned long uarg)
{
	int  Ret;
	int  cmd  = (int)ucmd;
	void *arg = (void *)uarg;
	VENC_REG_INFO_S *regcfginfo =  NULL;
	VENC_MEM_INFO VencMapInfo;

	if (!arg) {
		//HiVENC_UP_INTERRUPTIBLE(&g_VencMutex);
		HI_FATAL_VENC("uarg is NULL\n");
		return HI_FAILURE;
	}

	switch (cmd) {
	case CMD_VENC_START_ENCODE:/*lint !e30 !e142*/
		VeduIpCtx.TaskRunning = 1;
		regcfginfo = (VENC_REG_INFO_S *)arg;
		HiMemSet((void*)&VencMapInfo, 0, sizeof(VencMapInfo));

		VENC_DRV_OsalInitEvent(&g_hw_done_event, 0);

		Ret = venc_drv_register_info(regcfginfo);
		VeduIpCtx.TaskRunning = 0;
		HI_DBG_VENC("venc cfg reg info, Ret:%d\n", Ret);
		break;
	default:
		HI_ERR_VENC("venc cmd unknown:0x%x\n", ucmd);
		Ret = HI_FAILURE;
		break;
	}
	//HiVENC_UP_INTERRUPTIBLE(&g_VencMutex);
	return Ret;
}
static long VENC_DRV_Ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long Ret;

	Ret = HiVENC_DOWN_INTERRUPTIBLE(&g_VencMutex);
	if (Ret != 0)
	{
		HI_FATAL_VENC("Ioctl, down interruptible failed\n");
		return Ret;
	}
	Ret = (long)HI_DRV_UserCopy(file, cmd, arg, VENC_Ioctl);
	HiVENC_UP_INTERRUPTIBLE(&g_VencMutex);
	return Ret;
}

static struct file_operations VENC_FOPS =
{
	.owner          = THIS_MODULE,/*lint !e64 */
	.open           = VENC_DRV_Open,
	.unlocked_ioctl = VENC_DRV_Ioctl,
	.compat_ioctl   = VENC_DRV_Ioctl,
	.release        = VENC_DRV_Close,
};/*lint !e785 */

static const struct of_device_id venc_of_match[] = {
	{ .compatible = "hisi,kirin970-venc", },/*lint !e785 */
	{ }/*lint !e785 */
};

static struct platform_driver Venc_driver = {
	.probe   = VENC_DRV_Probe,
	.remove  = VENC_DRV_Remove,
	.suspend = VENC_DRV_Suspend,
	.resume  = VENC_DRV_Resume,
	.driver  = {
	.name    = "hi_venc",
	.owner   = THIS_MODULE,/*lint !e64 */
	.of_match_table = venc_of_match
	},/*lint !e785 */
};/*lint !e785 */

static struct platform_device Venc_device = {
	.name = "hi_venc",
	.id   = -1,
	.dev  = {
	.platform_data = NULL,
	.release   = NULL,
	},/*lint !e785 */
};/*lint !e785 */

static int VENC_DRV_SetupCdev(VENC_ENTRY *venc, const struct file_operations *fops)
{
	int err = 0;

	HI_INFO_VENC("enter %s()\n", __func__);
	err = alloc_chrdev_region(&venc->dev, 0, 1, "hi_venc");
	if (err < 0) {
		return HI_FAILURE;
	}

	HiMemSet((void*)&(venc->cdev), 0, sizeof(struct cdev));
	cdev_init(&(venc->cdev), &VENC_FOPS);

	venc->cdev.owner = THIS_MODULE;/*lint !e64 */
	venc->cdev.ops = &VENC_FOPS;
	err = cdev_add(&(venc->cdev), venc->dev, 1);

	/*在/sys/class/目录下创建设备类别目录hi_venc*/
	venc->venc_class = class_create(THIS_MODULE, "hi_venc");/*lint !e64 */
	if (IS_ERR(venc->venc_class)) {
		err = PTR_ERR(venc->venc_class);/*lint !e712 */
		HI_ERR_VENC("Fail to create hi_venc class\n");
		goto unregister_region;
		//return HI_FAILURE;/*lint !e438 */
	}

	/*在/dev/目录和/sys/class/hi_venc目录下分别创建设备文件hi_venc*/
	venc->venc_device = device_create(venc->venc_class, NULL, venc->dev, "%s", "hi_venc");
	if (IS_ERR(venc->venc_device)) {
		err = PTR_ERR(venc->venc_device);/*lint !e712 */
		HI_ERR_VENC("Fail to create hi_venc device\n");
		goto cls_destroy;
		//return HI_FAILURE;/*lint !e438 */
	}

	HI_INFO_VENC("exit %s()\n", __func__);
	return HI_SUCCESS;

cls_destroy:
	class_destroy(venc->venc_class);
	venc->venc_class = NULL;
unregister_region:
	unregister_chrdev_region(venc->dev, 1);
	return 0;
}/*lint !e550 */

static int VENC_DRV_CleanupCdev(VENC_ENTRY *venc)
{
	/*销毁设备类别和设备*/
	if (venc->venc_class) {
		device_destroy(venc->venc_class,venc->dev);
		class_destroy(venc->venc_class);
	}

	cdev_del(&(venc->cdev));
	unregister_chrdev_region(venc->dev,1);

	return 0;
}

static int VENC_DRV_Probe(struct platform_device * pltdev)
{
	int ret = HI_FAILURE;
	VENC_ENTRY *venc = NULL;

	HI_INFO_VENC("omxvenc prepare to probe\n");
	HiVENC_INIT_MUTEX(&g_VencMutex);
	if (g_vencDevDetected) {
		HI_INFO_VENC("venc device detected already\n");
		return HI_SUCCESS;
	}

	venc = HiMemVAlloc(sizeof(VENC_ENTRY));/*lint !e747 */
	if (!venc) {
		HI_FATAL_VENC("call vmalloc failed\n");
		return ret;
	}

	HiMemSet((void *)venc, 0, sizeof(VENC_ENTRY));
	ret = VENC_DRV_SetupCdev(venc, &VENC_FOPS);
	if (ret < 0) {
		HI_ERR_VENC("setup char device failed\n");
		goto free;
	}

	platform_set_drvdata(pltdev, venc);
	g_vencDevDetected = 1;

	ret = Venc_Regulator_Init(pltdev);
	if (ret < 0) {
		HI_FATAL_VENC("init regulator failed\n");
		goto cleanup;
	}

	HI_INFO_VENC("omxvenc probe successfully\n");
	return ret;

cleanup:
	VENC_DRV_CleanupCdev(venc);
free:
	HiMemVFree(venc);
	return ret;
}

static int VENC_DRV_Remove(struct platform_device *pltdev)
{
	VENC_ENTRY *venc = NULL;
	HI_INFO_VENC("omxvenc prepare to remove\n");

	venc = platform_get_drvdata(pltdev);
	if (venc) {
		VENC_DRV_CleanupCdev(venc);
		Venc_Regulator_Deinit(pltdev);
	}
	else {
		HI_ERR_VENC("get platform drvdata err\n");
	}

	platform_set_drvdata(pltdev,NULL);
	HiMemVFree(venc);
	g_vencDevDetected = 0;

	HI_INFO_VENC("remove omxvenc successfully\n");
	return 0;
}

int __init VENC_DRV_ModInit(void)
{
	int ret = 0;
	HI_INFO_VENC("enter %s()\n", __func__);

	ret = platform_device_register(&Venc_device);
	if (ret < 0) {
		HI_ERR_VENC("regist platform device failed\n");
		return ret;
	}

	ret = platform_driver_register(&Venc_driver);/*lint !e64 */
	if (ret < 0) {
		HI_ERR_VENC("regist platform driver failed\n");
		goto exit;
	}
	HI_INFO_VENC("success\n");
#ifdef MODULE
	HI_INFO_VENC("Load hi_venc.ko success\t(%s)\n", VERSION_STRING);
#endif
	HI_INFO_VENC("exit %s()\n", __func__);
	return HI_SUCCESS;
exit:
	platform_device_unregister(&Venc_device);
#ifdef MODULE
	HI_ERR_VENC("Load hi_venc.ko failed\t(%s)\n", VERSION_STRING);
#endif
	return ret;
}

void VENC_DRV_ModExit(void)
{
	HI_INFO_VENC("enter %s()\n", __func__);
	platform_driver_unregister(&Venc_driver);
	platform_device_unregister(&Venc_device);

	HI_INFO_VENC("exit %s()\n", __func__);
	return;
}
/*lint -e528*/
module_init(VENC_DRV_ModInit); /*lint !e528*/
module_exit(VENC_DRV_ModExit); /*lint !e528*/
/*lint -e753*/
MODULE_LICENSE("Dual BSD/GPL"); /*lint !e753*/

