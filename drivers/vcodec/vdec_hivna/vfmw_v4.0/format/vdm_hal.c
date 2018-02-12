/*
 * vdm hal interface
 *
 * Copyright (c) 2017 Hisilicon Limited
 *
 * Author: gaoyajun<gaoyajun@hisilicon.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation.
 *
 */

#include "vfmw.h"
#include "mem_manage.h"
//#include "public.h"
#include "scd_drv.h"
#include "vdm_hal_api.h"
#include "vdm_hal_local.h"
#include "omxvdec.h"
#include "linux_kernel_osal.h"

#ifdef VFMW_MPEG2_SUPPORT
#include "vdm_hal_mpeg2.h"
#endif
#ifdef VFMW_H264_SUPPORT
#include "vdm_hal_h264.h"
#endif
#ifdef VFMW_HEVC_SUPPORT
#include "vdm_hal_hevc.h"
#endif
#ifdef VFMW_MPEG4_SUPPORT
#include "vdm_hal_mpeg4.h"
#endif
#ifdef VFMW_VP8_SUPPORT
#include "vdm_hal_vp8.h"
#endif
#ifdef VFMW_VP9_SUPPORT
#include "vdm_hal_vp9.h"
#endif
#include "vfmw_intf.h"
#ifdef HIVDEC_SMMU_SUPPORT
#include "smmu.h"
#endif

VDMHAL_HWMEM_S  g_HwMem[MAX_VDH_NUM];
VDMHAL_BACKUP_S g_VdmRegState;

static VDMDRV_SLEEP_STAGE_E s_eVdmDrvSleepState = VDMDRV_SLEEP_STAGE_NONE;
static VDMDRV_STATEMACHINE_E s_VdmState = VDM_IDLE_STATE;

void VDMHAL_EnableInt(int VdhId)
{
	unsigned int D32  = 0xFFFFFFFE;
	int *p32 = NULL;

	if (VdhId != 0) {
		printk(KERN_ERR "VDH ID is wrong\n");
		return;
	}

	if (g_HwMem[VdhId].pVdmRegVirAddr == NULL) {
		p32 = (int *) MEM_Phy2Vir(gVdhRegBaseAddr);
		if (p32 == NULL) {
			printk(KERN_ERR "vdm register virtual address not mapped, reset failed\n");
			return;
		}

		g_HwMem[VdhId].pVdmRegVirAddr = p32;
	}

	WR_VREG(VREG_INT_MASK, D32, VdhId);

	return;
}

int VDMHAL_CfgRpReg(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
	int D32 = 0;
	
	WR_VREG(VREG_AVM_ADDR, pVdhRegCfg->VdhAvmAddr, 0);

	D32 = 0x2000C203;
	WR_VREG(VREG_BASIC_CFG1, D32, 0);

	D32 = 0x00300C03;
	WR_VREG(VREG_SED_TO, D32, 0);
	WR_VREG(VREG_ITRANS_TO, D32, 0);
	WR_VREG(VREG_PMV_TO, D32, 0);
	WR_VREG(VREG_PRC_TO, D32, 0);
	WR_VREG(VREG_RCN_TO, D32, 0);
	WR_VREG(VREG_DBLK_TO, D32, 0);
	WR_VREG(VREG_PPFD_TO, D32, 0);

	return VDMHAL_OK;
}

void VDMHAL_IMP_Init(void)
{
	memset(g_HwMem, 0, sizeof(g_HwMem));
	memset(&g_VdmRegState, 0, sizeof(g_VdmRegState));

	g_HwMem[0].pVdmRegVirAddr  = (int *) MEM_Phy2Vir(gVdhRegBaseAddr);

	VDMHAL_IMP_GlbReset();
	s_eVdmDrvSleepState = VDMDRV_SLEEP_STAGE_NONE;
	s_VdmState = VDM_IDLE_STATE;
}

void VDMHAL_IMP_DeInit(void)
{
	s_eVdmDrvSleepState = VDMDRV_SLEEP_STAGE_NONE;
	s_VdmState = VDM_IDLE_STATE;
}

void VDMHAL_IMP_ResetVdm(int VdhId)
{
	int i;
	int tmp = 0;
	unsigned int reg;
	unsigned int reg_rst_ok;
	unsigned int *pVdmResetVirAddr;
	unsigned int *pVdmResetOkVirAddr;

	pVdmResetVirAddr   = (unsigned int *) MEM_Phy2Vir(gSOFTRST_REQ_Addr);
	pVdmResetOkVirAddr = (unsigned int *) MEM_Phy2Vir(gSOFTRST_OK_ADDR);

	if ((pVdmResetVirAddr == NULL)
		|| (pVdmResetOkVirAddr == NULL)
		|| (g_HwMem[VdhId].pVdmRegVirAddr == NULL)) {
		printk(KERN_ERR "map vdm register fail, vir(pVdmResetVirAddr) : (%pK), vir(pVdmResetOkVirAddr) : (%pK)\n", pVdmResetVirAddr, pVdmResetOkVirAddr);
		return;
	}

	RD_VREG(VREG_INT_MASK, tmp, VdhId);

	/* require mfde reset */
	reg = *(volatile unsigned int *)pVdmResetVirAddr;
	*(volatile unsigned int *)pVdmResetVirAddr = reg | (unsigned int) (1 << MFDE_RESET_CTRL_BIT);

	/* wait for reset ok */
	for (i = 0; i < 100; i++) {
		reg_rst_ok = *(volatile unsigned int *)pVdmResetOkVirAddr;
		if (reg_rst_ok & (1 << MFDE_RESET_OK_BIT))
			break;
		VFMW_OSAL_uDelay(10);
	}

	if (i >= 100)
		printk(KERN_ERR "%s reset failed\n", __func__);

	/* clear reset require */
	*(volatile unsigned int *)pVdmResetVirAddr = reg & (unsigned int) (~(1 << MFDE_RESET_CTRL_BIT));


	WR_VREG(VREG_INT_MASK, tmp, VdhId);
	s_VdmState = VDM_IDLE_STATE;

	return;
}

void VDMHAL_IMP_GlbReset(void)
{
	int i;
	unsigned int reg, reg_rst_ok;
	unsigned int *pResetVirAddr   = NULL;
	unsigned int *pResetOKVirAddr = NULL;

	pResetVirAddr   = (unsigned int *) MEM_Phy2Vir(gSOFTRST_REQ_Addr);
	pResetOKVirAddr = (unsigned int *) MEM_Phy2Vir(gSOFTRST_OK_ADDR);

	if (pResetVirAddr == NULL || pResetOKVirAddr == NULL) {
		printk(KERN_ERR "VDMHAL_GlbReset: map vdm register fail, vir(pResetVirAddr) : (%pK), vir(pResetOKVirAddr) : (%pK)\n", pResetVirAddr, pResetOKVirAddr);
		return;
	}


	/* require all reset, include mfde scd bpd */
	reg = *(volatile unsigned int *)pResetVirAddr;
	*(volatile unsigned int *)pResetVirAddr = reg | (unsigned int) (1 << ALL_RESET_CTRL_BIT);

	/* wait for reset ok */
	for (i = 0; i < 100; i++) {
		reg_rst_ok = *(volatile unsigned int *)pResetOKVirAddr;
		if (reg_rst_ok & (1 << ALL_RESET_OK_BIT))
			break;
		VFMW_OSAL_uDelay(10);
	}

	if (i >= 100)
		printk(KERN_ERR "Glb Reset Failed\n");

	/* clear reset require */
	*(volatile unsigned int *)pResetVirAddr = reg & (unsigned int) (~(1 << ALL_RESET_CTRL_BIT));


	return;
}

void VDMHAL_IMP_ClearIntState(int VdhId)
{
	int *p32;
	unsigned int D32 = 0xFFFFFFFF;

	if (VdhId > (MAX_VDH_NUM - 1)) {
		printk(KERN_ERR "%s: VdhId : %d is more than %d\n", __func__, VdhId, (MAX_VDH_NUM - 1));
		return;
	}

	if (g_HwMem[VdhId].pVdmRegVirAddr == NULL) {
		if ((p32 = (int *) MEM_Phy2Vir(gVdhRegBaseAddr)) != NULL) {
			g_HwMem[VdhId].pVdmRegVirAddr = p32;
		} else {
			printk(KERN_ERR " %s %d vdm register virtual address not mapped, reset failed\n", __func__, __LINE__);
			return;
		}
	}

	WR_VREG(VREG_INT_STATE, D32, VdhId);

	return;
}

int VDMHAL_IMP_CheckReg(REG_ID_E reg_id, int VdhId)
{
	int *p32;
	int dat = 0;
	unsigned int reg_type;

	if (VdhId > (MAX_VDH_NUM - 1)) {
		printk(KERN_ERR "%s: Invalid VdhId is %d\n", __func__, VdhId);
		return VDMHAL_ERR;
	}

	if (g_HwMem[VdhId].pVdmRegVirAddr == NULL) {
		if ((p32 = (int *) MEM_Phy2Vir(gVdhRegBaseAddr)) != NULL) {
			g_HwMem[VdhId].pVdmRegVirAddr = p32;
		} else {
			printk(KERN_ERR " %s %d vdm register virtual address not mapped, reset failed\n", __func__, __LINE__);
			return 0;
		}
	}

	switch (reg_id) {
	case VDH_STATE_REG:
		reg_type = VREG_VDH_STATE;
		break;

	case INT_STATE_REG:
		reg_type = VREG_INT_STATE;
		break;

	case INT_MASK_REG:
		reg_type = VREG_INT_MASK;
		break;

	case VCTRL_STATE_REG:
		reg_type = VREG_VCTRL_STATE;
		break;

	default:
		printk(KERN_ERR "%s: unkown reg_id is %d\n", __func__, reg_id);
		return 0;
	}

	RD_VREG(reg_type, dat, 0);
	return dat;
}

int VDMHAL_IMP_PrepareDec(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
	VDMHAL_HWMEM_S *pHwMem = &(g_HwMem[0]);
	int *p32;

	if (NULL == pHwMem->pVdmRegVirAddr)
	{
		if (NULL != (p32 = (int *)MEM_Phy2Vir(gVdhRegBaseAddr)))
		{
			pHwMem->pVdmRegVirAddr = p32;
		}
		else
		{
			printk(KERN_ERR "vdm register virtual address not mapped, VDMHAL_PrepareDecfailed\n");
			return VDMHAL_ERR;
		}
	}
	if (VFMW_AVS == pVdhRegCfg->VidStd)
		WR_SCDREG(REG_AVS_FLAG, 0x00000001);
	else
		WR_SCDREG(REG_AVS_FLAG, 0x00000000);

	WR_SCDREG(REG_VDH_SELRST, 0x00000001);

	switch (pVdhRegCfg->VidStd) {
#ifdef VFMW_H264_SUPPORT
	case VFMW_H264:
		return H264HAL_StartDec(pVdhRegCfg);
#endif
#ifdef VFMW_HEVC_SUPPORT
	case VFMW_HEVC:
		return HEVCHAL_StartDec(pVdhRegCfg);
#endif
#ifdef VFMW_MPEG2_SUPPORT
	case VFMW_MPEG2:
		return MP2HAL_StartDec(pVdhRegCfg);
#endif
#ifdef VFMW_MPEG4_SUPPORT
	case VFMW_MPEG4:
		return MP4HAL_StartDec(pVdhRegCfg);
#endif
#ifdef VFMW_VP8_SUPPORT
	case VFMW_VP8:
		return VP8HAL_StartDec(pVdhRegCfg);
#endif
#ifdef VFMW_VP9_SUPPORT
	case VFMW_VP9:
		return VP9HAL_StartDec(pVdhRegCfg);
#endif
#ifdef VFMW_MVC_SUPPORT
	case VFMW_MVC:
		return H264HAL_StartDec(pVdhRegCfg);
#endif
	default:
		break;
	}

	return VDMHAL_ERR;
}

int VDMHAL_IsVdmRun(int VdhId)
{
	int Data32 = 0;

	if (g_HwMem[VdhId].pVdmRegVirAddr == NULL) {
		printk(KERN_ERR "VDM register not mapped yet\n");
		return 0;
	}

	RD_VREG(VREG_VCTRL_STATE, Data32, VdhId);

	return (Data32 == 1 ? 0 : 1);
}

int VDMHAL_IMP_BackupInfo(void)
{
	int i = 0;
	int regTmp;
	g_VdmRegState.Int_State_Reg = VDMHAL_IMP_CheckReg(INT_STATE_REG, 0);

	RD_VREG(VREG_BASIC_CFG1, g_VdmRegState.BasicCfg1, 0);
	RD_VREG(VREG_VDH_STATE, g_VdmRegState.VdmState, 0);

	RD_VREG(VREG_MB0_QP_IN_CURR_PIC, g_VdmRegState.Mb0QpInCurrPic, 0);
	RD_VREG(VREG_SWITCH_ROUNDING, g_VdmRegState.SwitchRounding, 0);

	{
		RD_VREG(VREG_SED_STA, g_VdmRegState.SedSta, 0);
		RD_VREG(VREG_SED_END0, g_VdmRegState.SedEnd0, 0);
		RD_VREG(VREG_DEC_CYCLEPERPIC, g_VdmRegState.DecCyclePerPic, 0);
		RD_VREG(VREG_RD_BDWIDTH_PERPIC, g_VdmRegState.RdBdwidthPerPic, 0);
		RD_VREG(VREG_WR_BDWIDTH_PERPIC, g_VdmRegState.WrBdWidthPerPic, 0);
		RD_VREG(VREG_RD_REQ_PERPIC, g_VdmRegState.RdReqPerPic, 0);
		RD_VREG(VREG_WR_REQ_PERPIC, g_VdmRegState.WrReqPerPic, 0);
		RD_VREG(VREG_LUMA_SUM_LOW, g_VdmRegState.LumaSumLow, 0);
		RD_VREG(VREG_LUMA_SUM_HIGH, g_VdmRegState.LumaSumHigh, 0);
	}
	for (i = 0; i < 32; i++) {
		regTmp = VREG_LUMA_HISTORGRAM + i * 4;
		RD_VREG(regTmp, g_VdmRegState.LumaHistorgam[i], 0);
	}

	return VDMHAL_OK;
}

void VDMHAL_GetRegState(VDMHAL_BACKUP_S *pVdmRegState)
{
	memcpy(pVdmRegState, &g_VdmRegState, sizeof(*pVdmRegState));
	s_VdmState = VDM_IDLE_STATE;
}

int VDMHAL_IMP_PrepareRepair(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
	VDMHAL_HWMEM_S *pHwMem = &(g_HwMem[0]);
	int *p32;
	int ret;

	if (NULL == pVdhRegCfg)
	{
		printk(KERN_ERR "%s: parameter is NULL\n", __func__);
		return VDMHAL_ERR;
	}
	if ( NULL == pHwMem->pVdmRegVirAddr )
	{
		if ( NULL != (p32 = (int *)MEM_Phy2Vir(gVdhRegBaseAddr)) )
		{
			pHwMem->pVdmRegVirAddr = p32;
		}
		else
		{
			printk(KERN_ERR "vdm register virtual address not mapped, VDMHAL_PrepareRepair failed\n");
			return VDMHAL_ERR;
		}
	}
	if (pVdhRegCfg->RepairTime == FIRST_REPAIR) {
		if (pVdhRegCfg->ValidGroupNum0 > 0)
			ret = VDMHAL_CfgRpReg(pVdhRegCfg);
		else
			ret = VDMHAL_ERR;
	} else if (pVdhRegCfg->RepairTime == SECOND_REPAIR) {
		printk(KERN_ERR "SECOND_REPAIR Parameter Error\n");
		ret = VDMHAL_ERR;
	} else {
		printk(KERN_ERR " parameter error\n");
		ret = VDMHAL_ERR;
	}

	return ret;
}

void VDMHAL_IMP_StartHwRepair(int VdhId)
{
	int D32 = 0;

	RD_VREG(VREG_BASIC_CFG0, D32, VdhId);
 
	D32 = 0x4000000; 
	WR_VREG(VREG_BASIC_CFG0, D32, VdhId);

#ifdef HIVDEC_SMMU_SUPPORT 
	SMMU_SetMasterReg(MFDE, SECURE_OFF, SMMU_ON); 
#endif

	VDMHAL_IMP_ClearIntState(VdhId);
	VDMHAL_EnableInt(VdhId);

	VFMW_OSAL_Mb();
	WR_VREG(VREG_VDH_START, 0, VdhId);
	WR_VREG(VREG_VDH_START, 1, VdhId);
	WR_VREG(VREG_VDH_START, 0, VdhId);

	return;
}

void VDMHAL_IMP_StartHwDecode(int VdhId)
{

#ifdef HIVDEC_SMMU_SUPPORT 
	SMMU_SetMasterReg(MFDE, SECURE_OFF, SMMU_ON); 
#endif

	VDMHAL_IMP_ClearIntState(VdhId);
	VDMHAL_EnableInt(VdhId);

	VFMW_OSAL_Mb();
	WR_VREG(VREG_VDH_START, 0, 0);
	WR_VREG(VREG_VDH_START, 1, 0);
	WR_VREG(VREG_VDH_START, 0, 0);

	return;
}

void VDMHAL_ISR(int VdhId)
{
	VDMHAL_IMP_BackupInfo();
	VDMHAL_IMP_ClearIntState(VdhId);
	VFMW_OSAL_GiveEvent(G_VDMHWDONEEVENT);
}

int VDMHAL_HwDecProc(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
	int ret = VDMDRV_ERR;

	s_VdmState = VDM_DECODE_STATE;

	if (pVdhRegCfg->VdhStartHwDecFlag == 1) {
		ret = VDMHAL_IMP_PrepareDec(pVdhRegCfg);
		if (ret == VDMDRV_OK) {
			VDMHAL_IMP_StartHwDecode(0);
		} else {
			ret = VDMDRV_ERR;
			printk(KERN_ERR "%s prepare dec fail \n", __func__);
		}
	} else if (pVdhRegCfg->VdhStartRepairFlag == 1) {
		ret = VDMHAL_IMP_PrepareRepair(pVdhRegCfg);
		if (ret == VDMDRV_OK) {
			VDMHAL_IMP_StartHwRepair(0);
		} else {
			ret = VDMDRV_ERR;
			printk(KERN_ERR "%s prepare repair fail \n", __func__);
		}
	} else {
		ret = VDMDRV_ERR;
		printk(KERN_ERR "%s process type error \n", __func__);
	}

	if (ret != VDMDRV_OK) {
		s_VdmState = VDM_IDLE_STATE;
	}
	return ret;
}

int VDMHAL_PrepareSleep(void)
{
	int ret = VDMDRV_OK;

	VFMW_OSAL_SemaDown(G_VDH_SEM);
	if (s_eVdmDrvSleepState == VDMDRV_SLEEP_STAGE_NONE) {
		if (VDM_IDLE_STATE == s_VdmState) {
			printk(KERN_INFO "%s, idle state \n", __func__);
			s_eVdmDrvSleepState = VDMDRV_SLEEP_STAGE_SLEEP;
		} else {
			printk(KERN_INFO "%s, work state \n", __func__);
			s_eVdmDrvSleepState = VDMDRV_SLEEP_STAGE_PREPARE;
		}

		ret = VDMDRV_OK;
	} else {
		ret = VDMDRV_ERR;
	}

	VFMW_OSAL_SemaUp(G_VDH_SEM);
	return ret;
}

void VDMHAL_ForceSleep(void)
{
	printk(KERN_INFO "%s, force state \n", __func__);
	VFMW_OSAL_SemaDown(G_VDH_SEM);
	if (s_eVdmDrvSleepState != VDMDRV_SLEEP_STAGE_SLEEP) {
		VDMHAL_IMP_ResetVdm(0);
		s_eVdmDrvSleepState = VDMDRV_SLEEP_STAGE_SLEEP;
	}

	VFMW_OSAL_SemaUp(G_VDH_SEM);
}

void VDMHAL_ExitSleep(void)
{
	VFMW_OSAL_SemaDown(G_VDH_SEM);
	s_eVdmDrvSleepState = VDMDRV_SLEEP_STAGE_NONE;
	VFMW_OSAL_SemaUp(G_VDH_SEM);
}

VDMDRV_SLEEP_STAGE_E VDMHAL_GetSleepStage(void)
{
	return s_eVdmDrvSleepState;
}

void VDMHAL_SetSleepStage(VDMDRV_SLEEP_STAGE_E sleepState)
{
	VFMW_OSAL_SemaDown(G_VDH_SEM);
	s_eVdmDrvSleepState = sleepState;
	VFMW_OSAL_SemaUp(G_VDH_SEM);
}
