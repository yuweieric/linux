/*
 * vdec driver for scd master
 *
 * Copyright (c) 2017 Hisilicon Limited
 *
 * Author: gaoyajun<gaoyajun@hisilicon.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation.
 *
 */
//#include "public.h"
#include "scd_drv.h"
#include "vfmw_intf.h"
#include "linux_kernel_osal.h"
#include "./format/vdm_hal_api.h"
#ifdef HIVDEC_SMMU_SUPPORT
#include "smmu.h"
#endif

static SCDDRV_SLEEP_STAGE_E  s_eScdDrvSleepStage = SCDDRV_SLEEP_STAGE_NONE;
static SCD_STATE_REG_S gScdStateReg;
static SCD_STATE_E s_SCDState = SCD_IDLE;

static void PrintScdVtrlReg(void);

int SCDDRV_ResetSCD(void)
{
	unsigned int tmp;
	unsigned int i;
	unsigned int reg_rst_ok;
	unsigned int reg;
	unsigned int *pScdResetReg   = NULL;
	unsigned int *pScdResetOkReg = NULL;

	pScdResetReg   = (unsigned int *) MEM_Phy2Vir(gSOFTRST_REQ_Addr);
	pScdResetOkReg = (unsigned int *) MEM_Phy2Vir(gSOFTRST_OK_ADDR);

	if (pScdResetReg == NULL || pScdResetOkReg == NULL) {
		printk(KERN_CRIT "scd reset register map fail\n");
		return VF_ERR_SYS;
	}

	tmp = RD_SCDREG(REG_SCD_INT_MASK);


	reg = *(volatile unsigned int *)pScdResetReg;
	*(volatile unsigned int *)pScdResetReg = reg | (unsigned int) (1 << SCD_RESET_CTRL_BIT);

	for (i = 0; i < 100; i++) {
		reg_rst_ok = *(volatile unsigned int *)pScdResetOkReg;
		if (reg_rst_ok & (1 << SCD_RESET_OK_BIT))
			break;
		VFMW_OSAL_uDelay(10);
	}

	if (i >= 100)
		printk(KERN_CRIT "%s reset failed\n", __func__);
	else
		printk(KERN_INFO "%s reset success\n", __func__);

	*(volatile unsigned int *)pScdResetReg = reg & (unsigned int) (~(1 << SCD_RESET_CTRL_BIT));


	WR_SCDREG(REG_SCD_INT_MASK, tmp);

	s_SCDState = SCD_IDLE;
	return FMW_OK;
}

int SCDDRV_PrepareSleep(void)
{
	int ret = SCDDRV_OK;

	VFMW_OSAL_SemaDown(G_SCD_SEM);
	if (s_eScdDrvSleepStage == SCDDRV_SLEEP_STAGE_NONE) {
		if (SCD_IDLE == s_SCDState) {
			printk(KERN_INFO "%s, idle state \n", __func__);
			s_eScdDrvSleepStage = SCDDRV_SLEEP_STAGE_SLEEP;
		} else {
			printk(KERN_INFO "%s, decoded state \n", __func__);
			s_eScdDrvSleepStage = SCDDRV_SLEEP_STAGE_PREPARE;
		}

		ret = SCDDRV_OK;
	} else {
		ret = SCDDRV_ERR;
	}

	VFMW_OSAL_SemaUp(G_SCD_SEM);
	return ret;
}

SCDDRV_SLEEP_STAGE_E SCDDRV_GetSleepStage(void)
{
	return s_eScdDrvSleepStage;
}

void SCDDRV_SetSleepStage(SCDDRV_SLEEP_STAGE_E sleepState)
{
	VFMW_OSAL_SemaDown(G_SCD_SEM);
	s_eScdDrvSleepStage = sleepState;
	VFMW_OSAL_SemaUp(G_SCD_SEM);
}

void SCDDRV_ForceSleep(void)
{
	printk(KERN_INFO "%s, force state \n", __func__);
	VFMW_OSAL_SemaDown(G_SCD_SEM);
	if (s_eScdDrvSleepStage != SCDDRV_SLEEP_STAGE_SLEEP) {
		SCDDRV_ResetSCD();
		s_eScdDrvSleepStage = SCDDRV_SLEEP_STAGE_SLEEP;
	}
	VFMW_OSAL_SemaUp(G_SCD_SEM);
}

void SCDDRV_ExitSleep(void)
{
	VFMW_OSAL_SemaDown(G_SCD_SEM);
	s_eScdDrvSleepStage = SCDDRV_SLEEP_STAGE_NONE;
	VFMW_OSAL_SemaUp(G_SCD_SEM);
}

int SCDDRV_WriteReg(SCD_CONFIG_REG_S *pSmCtrlReg )
{
	if (s_SCDState != SCD_IDLE)
		return SCDDRV_ERR;

	s_SCDState = SCD_WORKING;
	WR_SCDREG(REG_SCD_INI_CLR, 1);

	// LIST_ADDRESS
	WR_SCDREG(REG_LIST_ADDRESS, (unsigned int)pSmCtrlReg->DownMsgPhyAddr);

	// UP_ADDRESS
	WR_SCDREG(REG_UP_ADDRESS, (unsigned int) pSmCtrlReg->UpMsgPhyAddr);

	// UP_LEN
	WR_SCDREG(REG_UP_LEN, (unsigned int) pSmCtrlReg->UpLen);

	// BUFFER_FIRST
	WR_SCDREG(REG_BUFFER_FIRST, (unsigned int) pSmCtrlReg->BufferFirst);

	// BUFFER_LAST
	WR_SCDREG(REG_BUFFER_LAST, (unsigned int) pSmCtrlReg->BufferLast);

	// BUFFER_INI
	WR_SCDREG(REG_BUFFER_INI, (unsigned int) pSmCtrlReg->BufferIni);

	// SCD_PROTOCOL 
	WR_SCDREG(REG_SCD_PROTOCOL, (unsigned int) ((pSmCtrlReg->ScdLowdlyEnable << 8)
		| ((pSmCtrlReg->SliceCheckFlag << 4) & 0x10)
		| (pSmCtrlReg->ScdProtocol & 0x0f)));
#ifdef HIVDEC_SMMU_SUPPORT
	SMMU_SetMasterReg(SCD, SECURE_OFF, SMMU_ON);
#endif
 
#ifndef SCD_BUSY_WAITTING
	WR_SCDREG(REG_SCD_INT_MASK, 0);
#endif

	PrintScdVtrlReg();

	// SCD_START
	WR_SCDREG(REG_SCD_START, 0);
	WR_SCDREG(REG_SCD_START, (unsigned int) (pSmCtrlReg->ScdStart & 0x01));

	return SCDDRV_OK;
}

void SCDDRV_SaveStateReg(void)
{
	gScdStateReg.ScdProtocol = RD_SCDREG(REG_SCD_PROTOCOL);
	gScdStateReg.Scdover     = RD_SCDREG(REG_SCD_OVER);
	gScdStateReg.ScdInt      = RD_SCDREG(REG_SCD_INT);
	gScdStateReg.ScdNum      = RD_SCDREG(REG_SCD_NUM);
	gScdStateReg.ScdRollAddr = RD_SCDREG(REG_ROLL_ADDR);
	gScdStateReg.SrcEaten    = RD_SCDREG(REG_SRC_EATEN);
	gScdStateReg.UpLen       = RD_SCDREG(REG_UP_LEN);
}

void SCDDRV_init(void)
{
	memset(&gScdStateReg, 0, sizeof(gScdStateReg));
	s_eScdDrvSleepStage = SCDDRV_SLEEP_STAGE_NONE;
	s_SCDState = SCD_IDLE;
}

void SCDDRV_DeInit(void)
{
	s_eScdDrvSleepStage = SCDDRV_SLEEP_STAGE_NONE;
	s_SCDState = SCD_IDLE;
}

void SCDDRV_ISR(void)
{
	int dat = 0;

	dat = RD_SCDREG(REG_SCD_OVER) & 0x01;
	if ((dat & 1) == 0) {
		printk(KERN_CRIT "End0: SM_SCDIntServeProc()\n");
		return;
	}

	SCDDRV_SaveStateReg();
	WR_SCDREG(REG_SCD_INI_CLR, 1);
	VFMW_OSAL_GiveEvent(G_SCDHWDONEEVENT);
}

void SCDDRV_GetRegState(SCD_STATE_REG_S *pScdStateReg)
{
	memcpy(pScdStateReg, &gScdStateReg, sizeof(*pScdStateReg));
	s_SCDState = SCD_IDLE;
}

int WaitSCDFinish(void)
{
	int i;

	if (SCD_WORKING == s_SCDState) {
		for (i = 0; i < SCD_TIME_OUT_COUNT; i++) {
			if ((RD_SCDREG(REG_SCD_OVER) & 1))
				return SCDDRV_OK;
		}

		return SCDDRV_ERR;
	} else {
		return SCDDRV_OK;
	}
}

static void PrintScdVtrlReg(void)
{
	SCD_CONFIG_REG_S SmCtrlReg;
	memset(&SmCtrlReg, 0, sizeof(SmCtrlReg));

	SmCtrlReg.DownMsgPhyAddr = RD_SCDREG(REG_LIST_ADDRESS);
	SmCtrlReg.UpMsgPhyAddr   = RD_SCDREG(REG_UP_ADDRESS);
	SmCtrlReg.UpLen          = RD_SCDREG(REG_UP_LEN);
	SmCtrlReg.BufferFirst    = RD_SCDREG(REG_BUFFER_FIRST);
	SmCtrlReg.BufferLast     = RD_SCDREG(REG_BUFFER_LAST);
	SmCtrlReg.BufferIni      = RD_SCDREG(REG_BUFFER_INI);
	SmCtrlReg.ScdProtocol    = RD_SCDREG(REG_SCD_PROTOCOL);
	SmCtrlReg.ScdStart       = RD_SCDREG(REG_SCD_START);
}

#ifdef ENV_ARMLINUX_KERNEL
int SCDDRV_IsScdIdle(void)
{
	int ret = SCDDRV_OK;
	if (SCD_IDLE == s_SCDState) {
		ret = SCDDRV_OK;
	} else if (SCD_WORKING == s_SCDState) {
		ret = SCDDRV_ERR;
	} else {
		ret = SCDDRV_ERR;
		printk(KERN_ERR "%s : s_SCDState : %d is wrong\n", __func__, s_SCDState);
	}
	return ret;
}
#endif
