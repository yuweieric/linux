/*
 * vfmw interface
 *
 * Copyright (c) 2017 Hisilicon Limited
 *
 * Author: gaoyajun<gaoyajun@hisilicon.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/kern_levels.h>
#include <linux/printk.h>

//#include "public.h"
#include "vfmw_intf.h"

#include "../omxvdec/omxvdec.h"
#ifdef HIVDEC_SMMU_SUPPORT
#include "smmu.h"
#endif
#include "./format/vdm_hal_api.h"
#include "vfmw_osal_ext.h"
#include "linux_kernel_osal.h"
#ifndef IRQF_DISABLED
#define IRQF_DISABLED               (0x00000020)
#endif
#define VDM_TIMEOUT               (400)//ms
#define VDM_FPGA_TIMEOUT          (500000)//ms
#define SCD_TIMEOUT              (400)//ms
#define SCD_FPGA_TIMEOUT          (200000)//ms
#define SCEN_IDENT                (0x828)
#define MAP_SIZE                  (256 * 1024)

#define TIME_PERIOD(begin, end) ((end >= begin)? (end-begin):(0xffffffff - begin + end))

// cppcheck-suppress *
#define  VCTRL_ASSERT_RET(cond, else_print)                                      \
do {                                                                             \
	if (!(cond)) {                                                           \
		printk(KERN_ERR "%s %d %s\n", __func__, __LINE__, else_print ); \
		return VCTRL_ERR;                                                \
	}                                                                        \
}while(0)

static DRV_MEM_S g_RegsBaseAddr;

Vfmw_Osal_Func_Ptr g_vfmw_osal_fun_ptr;

void VCTRL_Suspend(void)
{
	unsigned char isScdSleep = 0;
	unsigned char isVdmSleep = 0;
	unsigned int  SleepCount = 0;
	unsigned int BeginTime, EntrTime, CurTime;

	EntrTime = VFMW_OSAL_GetTimeInMs();

	SCDDRV_PrepareSleep();

	VDMHAL_PrepareSleep();

	BeginTime = VFMW_OSAL_GetTimeInMs();
	do {
		if (SCDDRV_SLEEP_STAGE_SLEEP == SCDDRV_GetSleepStage())
			isScdSleep = 1;

		if (VDMHAL_GetSleepStage() == VDMDRV_SLEEP_STAGE_SLEEP)
			isVdmSleep = 1;

		if ((isScdSleep == 1) && (isVdmSleep == 1)) {
			break;
		} else {
			if (SleepCount > 30) {
				if (isScdSleep != 1) {
					printk(KERN_ERR "Force scd sleep\n");
					SCDDRV_ForceSleep();
				}
				if (isVdmSleep != 1) {
					printk(KERN_ERR "Force vdm sleep\n");
					VDMHAL_ForceSleep();
				}
				break;
			}

			VFMW_OSAL_mSleep(10);
			SleepCount++;
		}
	} while ((isScdSleep != 1) || (isVdmSleep != 1));
 
	CurTime = VFMW_OSAL_GetTimeInMs();
	printk(KERN_INFO "Vfmw suspend totally take %d ms\n", TIME_PERIOD(EntrTime, CurTime));

	return;
}

void VCTRL_Resume(void)
{
	unsigned int EntrTime, CurTime;

	EntrTime = VFMW_OSAL_GetTimeInMs();

	SMMU_InitGlobalReg();

	SCDDRV_ExitSleep();

	VDMHAL_ExitSleep(); 

	CurTime = VFMW_OSAL_GetTimeInMs();
	printk(KERN_INFO "Vfmw resume totally take %d ms\n", TIME_PERIOD(EntrTime, CurTime));

	return;
}

static int VCTRL_ISR(int irq, void *dev_id)
{
	unsigned int D32;
	D32 = RD_SCDREG(REG_SCD_INI_CLR)&0x1;
	if (D32 == 1)
		SCDDRV_ISR();

	RD_VREG(VREG_INT_STATE, D32, 0);
	if (D32 == 1)
		VDMHAL_ISR(0);

	return IRQ_HANDLED;
}

static int VCTRL_RequestIrq(unsigned int IrqNumNorm, unsigned int IrqNumProt, unsigned int IrqNumSafe)
{
#if !defined(VDM_BUSY_WAITTING)
	if (VFMW_OSAL_RequestIrq(IrqNumNorm, VCTRL_ISR, IRQF_DISABLED, "vdec_norm_irq", NULL) != 0) {    //for 2.6.24рт╨С
		printk(KERN_ERR "Request vdec norm irq %d failed\n", IrqNumNorm);
		return VCTRL_ERR;
	}
#endif
	return VCTRL_OK;
}

static void VCTRL_FreeIrq(unsigned int IrqNumNorm, unsigned int IrqNumProt, unsigned int IrqNumSafe)
{
#if !defined(VDM_BUSY_WAITTING)
	VFMW_OSAL_FreeIrq(IrqNumNorm, NULL);
#endif
}

static int VCTRL_HalInit(void)
{
#ifdef HIVDEC_SMMU_SUPPORT
	if (SMMU_Init() != SMMU_OK) {
		printk(KERN_ERR "SMMU_Init failed\n");
		return VCTRL_ERR;
	}
#endif

	SCDDRV_init();
	VDMHAL_IMP_Init();
	SMMU_InitGlobalReg();

	return VCTRL_OK;
}

static void VCTRL_HalDeInit(void)
{
#ifdef HIVDEC_SMMU_SUPPORT
	SMMU_DeInit();
#endif
	VDMHAL_IMP_DeInit();
	SCDDRV_DeInit();
}

int VCTRL_OpenDrivers(void)
{
	MEM_RECORD_S *pstMem;
	int ret   = VCTRL_ERR;

	pstMem = &g_RegsBaseAddr.stVdhReg;
	if (MEM_MapRegisterAddr(gVdhRegBaseAddr, MAP_SIZE, pstMem) == MEM_MAN_OK) {
		if (MEM_AddMemRecord(pstMem->PhyAddr, pstMem->VirAddr, pstMem->Length) != MEM_MAN_OK) {
			printk(KERN_ERR "%s %d MEM_AddMemRecord failed\n", __func__, __LINE__);
			goto exit;
		}
	} else {
		printk(KERN_ERR "Map vdh register failed! gVdhRegBaseAddr : %pK, gVdhRegRange : %d\n",
			(void *)(uintptr_t)gVdhRegBaseAddr, gVdhRegRange);
		goto exit;
	}

	ret = VCTRL_RequestIrq(gVdecIrqNumNorm, gVdecIrqNumProt, gVdecIrqNumSafe);
	if (ret != VCTRL_OK) {
		printk(KERN_ERR "VCTRL_RequestIrq failed\n");
		goto exit;
	}

	if (VCTRL_HalInit() != VCTRL_OK) {
		printk(KERN_ERR "VCTRL_HalInit failed\n");
		goto exit;
	}

	VFMW_OSAL_InitEvent(G_SCDHWDONEEVENT, 0);
	VFMW_OSAL_InitEvent(G_VDMHWDONEEVENT, 0);

	return VCTRL_OK;

exit:
	VCTRL_CloseVfmw();
	return VCTRL_ERR;
}

int VCTRL_OpenVfmw(void)
{
	memset(&g_RegsBaseAddr, 0, sizeof(g_RegsBaseAddr)); /* unsafe_function_ignore: m
emset */

	MEM_InitMemManager();
	if (VCTRL_OpenDrivers() != VCTRL_OK) {
		printk(KERN_ERR "OpenDrivers fail\n");
		return VCTRL_ERR;
	} 

	return VCTRL_OK;
}

int VCTRL_CloseVfmw(void)
{
	MEM_RECORD_S *pstMem;

	VCTRL_HalDeInit();

	pstMem = &g_RegsBaseAddr.stVdhReg;
	if (pstMem->Length != 0) {
		MEM_UnmapRegisterAddr(pstMem->PhyAddr, pstMem->VirAddr, pstMem->Length);
		MEM_DelMemRecord(pstMem->PhyAddr, pstMem->VirAddr, pstMem->Length);
		memset(&g_RegsBaseAddr.stVdhReg, 0, sizeof(g_RegsBaseAddr.stVdhReg)); /* unsafe_function_ignore: m
emset */
	}

	VCTRL_FreeIrq(gVdecIrqNumNorm, gVdecIrqNumProt, gVdecIrqNumSafe);
 
	return VCTRL_OK;
}

int VCTRL_VDMHal_Process(OMXVDH_REG_CFG_S *pVdmRegCfg, VDMHAL_BACKUP_S *pVdmRegState)
{
	int ret = HI_SUCCESS;
	VDMDRV_SLEEP_STAGE_E sleepState;

	sleepState = VDMHAL_GetSleepStage();
	if (VDMDRV_SLEEP_STAGE_SLEEP == sleepState) {
		printk(KERN_INFO "vdm sleep state\n");
		return HI_FAILURE;
	}

	if (pVdmRegCfg->vdh_reset_flag)
		VDMHAL_IMP_ResetVdm(0);
 
	VFMW_OSAL_InitEvent(G_VDMHWDONEEVENT, 0);
	ret = VDMHAL_HwDecProc(pVdmRegCfg);

	if (ret) {
		printk(KERN_ERR "%s config error\n", __func__);
	} else {
		ret = VFMW_OSAL_WaitEvent(G_VDMHWDONEEVENT, VDM_TIMEOUT);
		if (ret == HI_SUCCESS) {
			VDMHAL_GetRegState(pVdmRegState);
		} else {
			printk(KERN_ERR "VFMW_OSAL_WaitEvent wait time out\n");
			VDMHAL_IMP_ResetVdm(0);
		}
	}

	sleepState = VDMHAL_GetSleepStage();
	if (sleepState == VDMDRV_SLEEP_STAGE_PREPARE)
		VDMHAL_SetSleepStage(VDMDRV_SLEEP_STAGE_SLEEP);

	return ret;
}

int VCTRL_SCDHal_Process(OMXSCD_REG_CFG_S *pScdRegCfg,SCD_STATE_REG_S *pScdStateReg)
{
	int ret = HI_SUCCESS;
	SCDDRV_SLEEP_STAGE_E sleepState;
	CONFIG_SCD_CMD cmd = pScdRegCfg->cmd;

	sleepState = SCDDRV_GetSleepStage();
	if (SCDDRV_SLEEP_STAGE_SLEEP == sleepState) {
		printk(KERN_INFO "SCD sleep state\n");
		return HI_FAILURE;
	}

	if (pScdRegCfg->SResetFlag) {
		if (SCDDRV_ResetSCD() != HI_SUCCESS) {
			printk(KERN_ERR "VDEC_IOCTL_SCD_WAIT_HW_DONE  Reset SCD failed\n");
			return HI_FAILURE;
		}
	}
	
	switch (cmd) {
	case CONFIG_SCD_REG_CMD:
		VFMW_OSAL_InitEvent(G_SCDHWDONEEVENT, 0);
		ret = SCDDRV_WriteReg(&pScdRegCfg->SmCtrlReg );
		if (ret != HI_SUCCESS) {
			printk(KERN_ERR "SCD busy\n");
			return HI_FAILURE;
		}

		ret = VFMW_OSAL_WaitEvent(G_SCDHWDONEEVENT,SCD_TIMEOUT);
		if (ret == HI_SUCCESS) {
			SCDDRV_GetRegState(pScdStateReg);
		} else {
			printk(KERN_INFO "VDEC_IOCTL_SCD_WAIT_HW_DONE  wait time out\n");
			SCDDRV_ResetSCD();
		}

		sleepState = SCDDRV_GetSleepStage();
		if (sleepState == SCDDRV_SLEEP_STAGE_PREPARE) {
			SCDDRV_SetSleepStage(SCDDRV_SLEEP_STAGE_SLEEP);
		}
		break;

	default:
		printk(KERN_ERR " cmd type unknown:%d\n", cmd);
		return HI_FAILURE;
	}

	return ret;
}

int VCTRL_VDMHAL_IsRun(void)
{
	return VDMHAL_IsVdmRun(0);
}

int VCTRL_Scen_Ident(unsigned int cmd)
{
	if ((RD_SCDREG(SCEN_IDENT) == 1) && (cmd != VDEC_IOCTL_SET_CLK_RATE))
		return 1;

	return 0;
}

int VFMW_DRV_ModInit(void)
{
	OSAL_InitInterface();
	VFMW_OSAL_SemaInit(G_SCD_SEM);
	VFMW_OSAL_SemaInit(G_VDH_SEM);
	VFMW_OSAL_SemaInit(G_BPD_SEM);

	VFMW_OSAL_SpinLockInit(G_SPINLOCK_SCD);
	VFMW_OSAL_SpinLockInit(G_SPINLOCK_VDH);
	VFMW_OSAL_SpinLockInit(G_SPINLOCK_RECORD);
	VFMW_OSAL_InitEvent(G_SCDHWDONEEVENT, 0);
	VFMW_OSAL_InitEvent(G_VDMHWDONEEVENT, 0);

#ifdef MODULE
	printk(KERN_INFO "%s : Load hi_vfmw.ko (%d) success\n", __func__, VFMW_VERSION_NUM);
#endif

	return 0;
}

void VFMW_DRV_ModExit(void)
{
#ifdef MODULE
	printk(KERN_INFO "%s : Unload hi_vfmw.ko (%d) success\n",__func__, VFMW_VERSION_NUM);
#endif

	return;
}

module_init(VFMW_DRV_ModInit);
module_exit(VFMW_DRV_ModExit);

MODULE_AUTHOR("gaoyajun");
MODULE_LICENSE("GPL");
