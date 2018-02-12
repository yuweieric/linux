/*
 * vdec mem_manager
 *
 * Copyright (c) 2017 Hisilicon Limited
 *
 * Author: gaoyajun<gaoyajun@hisilicon.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation.
 *
 */
#include "mem_manage.h"
#include "sysconfig.h"
#include "vfmw_osal_ext.h"
//#include "public.h"
#include "linux_kernel_osal.h"

#define    MAX_MEM_MAN_RECORD_NUM    (MAX_CHAN_NUM*32)
MEM_RECORD_S  s_MemRecord[MAX_MEM_MAN_RECORD_NUM];

void MEM_InitMemManager(void)
{
	VFMW_OSAL_SpinLock(G_SPINLOCK_RECORD);

	memset(s_MemRecord, 0, sizeof(s_MemRecord));

	VFMW_OSAL_SpinUnLock(G_SPINLOCK_RECORD);

}

int MEM_AddMemRecord(unsigned int PhyAddr, void *VirAddr, unsigned int Length)
{
	int i;
	char IsErrorFlag = 0;
	int TargetPos  = -1;
	int ret        = MEM_MAN_ERR;

	VFMW_OSAL_SpinLock(G_SPINLOCK_RECORD);

	for (i = 0; i < MAX_MEM_MAN_RECORD_NUM; i++) {
		if ((s_MemRecord[i].PhyAddr <= PhyAddr) && (PhyAddr < s_MemRecord[i].PhyAddr + s_MemRecord[i].Length)) {
			IsErrorFlag = 1;
			break;
		}

		if (s_MemRecord[i].Length == 0 && TargetPos == -1)
			TargetPos = i;
	}

	if (IsErrorFlag == 1) {
		printk(KERN_CRIT "%s conflict occured\n ", __func__);
		ret = MEM_MAN_ERR;
	} else if (TargetPos == -1) {
		printk(KERN_CRIT "%s no free record slot\n ", __func__);
		ret = MEM_MAN_ERR;
	} else {
		s_MemRecord[TargetPos].PhyAddr = PhyAddr;
		s_MemRecord[TargetPos].VirAddr = VirAddr;
		s_MemRecord[TargetPos].Length  = Length;
		ret = MEM_MAN_OK;
	}

	VFMW_OSAL_SpinUnLock(G_SPINLOCK_RECORD);

	return ret;
}

int MEM_DelMemRecord(unsigned int PhyAddr, void *VirAddr, unsigned int Length)
{
	int i;

	VFMW_OSAL_SpinLock(G_SPINLOCK_RECORD);
	for (i = 0; i < MAX_MEM_MAN_RECORD_NUM; i++) {
		if (s_MemRecord[i].Length == 0)
			continue;

		if (PhyAddr == s_MemRecord[i].PhyAddr && VirAddr == s_MemRecord[i].VirAddr &&
		    Length == s_MemRecord[i].Length) {
			s_MemRecord[i].Length  = 0;
			s_MemRecord[i].PhyAddr = 0;
			s_MemRecord[i].VirAddr = 0;
			VFMW_OSAL_SpinUnLock(G_SPINLOCK_RECORD);

			return MEM_MAN_OK;
		}
	}
	VFMW_OSAL_SpinUnLock(G_SPINLOCK_RECORD);

	return MEM_MAN_ERR;
}

void *MEM_Phy2Vir(unsigned int PhyAddr)
{
	unsigned int i;
	unsigned char *VirAddr = NULL;

	for (i = 0; i < MAX_MEM_MAN_RECORD_NUM; i++) {
		if (s_MemRecord[i].Length == 0)
			continue;

		if ((PhyAddr >= s_MemRecord[i].PhyAddr) && (PhyAddr < s_MemRecord[i].PhyAddr + s_MemRecord[i].Length)) {
			VirAddr = s_MemRecord[i].VirAddr + (PhyAddr - s_MemRecord[i].PhyAddr);
			break;
		}
	}

	return (void *) VirAddr;
}

unsigned int MEM_Vir2Phy(unsigned char *VirAddr)
{
	unsigned int i;

	unsigned int PhyAddr = 0;
	for (i = 0; i < MAX_MEM_MAN_RECORD_NUM; i++) {
		if (s_MemRecord[i].Length == 0)
			continue;

		if ((VirAddr >= s_MemRecord[i].VirAddr) && (VirAddr < s_MemRecord[i].VirAddr + s_MemRecord[i].Length)) {
			PhyAddr = s_MemRecord[i].PhyAddr + (VirAddr - s_MemRecord[i].VirAddr);
			break;
		}
	}

	return PhyAddr;
}

void MEM_WritePhyWord(unsigned int PhyAddr, unsigned int Data32)
{
	unsigned int *pDst;

	pDst = (unsigned int *) MEM_Phy2Vir(PhyAddr);
	if (pDst != NULL)
		writel(Data32, pDst);
}

unsigned int MEM_ReadPhyWord(unsigned int PhyAddr)
{
	unsigned int *pDst;
	unsigned int Data32;

	Data32 = 0;
	pDst = (unsigned int *) MEM_Phy2Vir(PhyAddr);
	if (pDst != NULL) {
		Data32 = readl((volatile unsigned int *)pDst);
	}

	return Data32;
}

int  MEM_MapRegisterAddr(unsigned int RegStartPhyAddr, unsigned int RegByteLen, MEM_RECORD_S *pMemRecord)
{
	unsigned char *ptr;

	if (pMemRecord == NULL || RegStartPhyAddr == 0 || RegByteLen == 0 || VFMW_OSAL_RegisterMap == NULL)
		return MEM_MAN_ERR;

	memset(pMemRecord, 0, sizeof(*pMemRecord));

	ptr = VFMW_OSAL_RegisterMap(RegStartPhyAddr, RegByteLen);

	if (ptr != NULL) {
		pMemRecord->PhyAddr = RegStartPhyAddr;
		pMemRecord->VirAddr = ptr;
		pMemRecord->Length = RegByteLen;
		return MEM_MAN_OK;
	}

	return MEM_MAN_ERR;
}

void MEM_UnmapRegisterAddr(unsigned int PhyAddr, unsigned char *VirAddr, unsigned int Size)
{
	if (PhyAddr == 0 || VirAddr == 0 || VFMW_OSAL_RegisterUnMap == NULL)
		return;

	VFMW_OSAL_RegisterUnMap(VirAddr, Size);

	return;
}
