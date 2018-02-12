/*
 * vdec driver for smmu master
 *
 * Copyright (c) 2017 Hisilicon Limited
 *
 * Author: gaoyajun<gaoyajun@hisilicon.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation.
 *
 */

#ifdef ENV_ARMLINUX_KERNEL
#include <asm/memory.h>
#include <linux/types.h>
#include <linux/gfp.h>
#endif
#include <linux/printk.h>
#include "smmu.h"
//#include "public.h"
#include "mem_manage.h"
#include "linux_kernel_osal.h"
#include <linux/io.h>
#include "../omxvdec/omxvdec.h"

#define SMRx_ID_SIZE            32
#define SMMU_RWERRADDR_SIZE     128

#define HIVDEC_SMMU_COMMON_OFFSET       (0x20000)
#define HIVDEC_SMMU_MASTER_OFFSET       (0xF000)

#define HIVDEC_SMMU_COMMON_BASE_ADDR (gVdhRegBaseAddr + HIVDEC_SMMU_COMMON_OFFSET)
#define HIVDEC_SMMU_MASTER_BASE_ADDR (gVdhRegBaseAddr + HIVDEC_SMMU_MASTER_OFFSET)

//SMMU common and Master(MFDE/SCD/BPD) virtual base address
typedef struct {
	int *pSMMUCommonBaseVirAddr;
	int *pSMMUMasterBaseVirAddr;
	int *pSMMUMFDERegVirAddr;
	int *pSMMUBPDRegVirAddr;
	int *pSMMUSCDRegVirAddr;
} SMMU_REG_VIR_S;

SMMU_REG_VIR_S gSmmuRegVir;
MEM_DESC_S gAllocMem_RD;
MEM_DESC_S gAllocMem_WR;

int gSmmuInitFlag   = 0;
int gMfdeSecureFlag = 0;
int gMfdeSmmuFlag   = 1;
int gScdSecureFlag  = 0;
int gScdSmmuFlag    = 1;
int gBpdSecureFlag  = 0;
int gBpdSmmuFlag    = 0;

//smmu common regs r/w
#define RD_SMMU_COMMON_VREG( reg, dat )               \
do {                    \
	(dat) = readl(((volatile int*)((char*)gSmmuRegVir.pSMMUCommonBaseVirAddr + (reg)))); \
} while(0)

#define WR_SMMU_COMMON_VREG( reg, dat )               \
do {                    \
    writel((dat), ((volatile int*)((char*)gSmmuRegVir.pSMMUCommonBaseVirAddr + (reg)))); \
} while(0)
//smmu master regs r/w
#define RD_SMMU_MASTER_VREG( reg, dat )               \
do {                    \
	(dat) = readl(((volatile int*)((char*)gSmmuRegVir.pSMMUMasterBaseVirAddr + (reg)))); \
} while(0)

#define WR_SMMU_MASTER_VREG( reg, dat )               \
do {                    \
	writel((dat), ((volatile int*)((char*)gSmmuRegVir.pSMMUMasterBaseVirAddr + (reg)))); \
} while(0)

//mfde regs r/w
#define RD_SMMU_MFDE_VREG( reg, dat )               \
do {                    \
	(dat) = readl(((volatile int*)((char*)gSmmuRegVir.pSMMUMFDERegVirAddr + (reg)))); \
} while(0)

#define WR_SMMU_MFDE_VREG( reg, dat )               \
do {                    \
    writel((dat), ((volatile int*)((char*)gSmmuRegVir.pSMMUMFDERegVirAddr + (reg)))); \
} while(0)
//bpd regs r/w
#define RD_SMMU_BPD_VREG( reg, dat )               \
do {                    \
	(dat) = readl(((volatile int*)((char*)gSmmuRegVir.pSMMUBPDRegVirAddr + (reg)))); \
} while(0)

#define WR_SMMU_BPD_VREG( reg, dat )               \
do {                    \
    writel((dat), ((volatile int*)((char*)gSmmuRegVir.pSMMUBPDRegVirAddr + (reg)))); \
} while(0)
//scd regs r/w
#define RD_SMMU_SCD_VREG( reg, dat )               \
do {                    \
	(dat) = readl(((volatile int*)((char*)gSmmuRegVir.pSMMUSCDRegVirAddr + (reg)))); \
} while(0)

#define WR_SMMU_SCD_VREG( reg, dat )               \
do {                    \
    writel((dat), ((volatile int*)((char*)gSmmuRegVir.pSMMUSCDRegVirAddr + (reg)))); \
} while(0)

/**
 *function: set SMMU common register
 *addr: register's vir addr
 *val: value to be set
 *bw: bit width
 *bs: bit start
 */
static void set_common_reg(unsigned int addr, int val, int bw, int bs)
{
	int mask = (1UL << bw) - 1UL;
	int tmp = 0;

	RD_SMMU_COMMON_VREG(addr, tmp);
	tmp &= ~(mask << bs);/*lint !e502*/
	WR_SMMU_COMMON_VREG(addr, tmp | ((val & mask) << bs));
}

/**
 *function: set SMMU master register
 *addr: register's vir addr
 *val: value to be set
 *bw: bit width
 *bs: bit start
 */
static void set_master_reg(unsigned int addr, int val, int bw, int bs)
{
	int mask = (1UL << bw) - 1UL;
	int tmp = 0;

	RD_SMMU_MASTER_VREG(addr, tmp);
	tmp &= ~(mask << bs);/*lint !e502*/
	WR_SMMU_MASTER_VREG(addr, (tmp | ((val & mask) << bs)));

}

/**
 *function: set mfde/scd/bpd register
 *master_type: MFDE/SCD/BPD
 *addr: register's vir addr
 *val: value to be set
 *bw: bit width
 *bs: bit start
 */
static void set_vdh_master_reg(SMMU_MASTER_TYPE master_type, unsigned int addr, int val, int bw, int bs)
{
	int mask = (1UL << bw) - 1UL;
	int tmp = 0;

	switch (master_type) {
	case MFDE:
		RD_SMMU_MFDE_VREG(addr, tmp);
		tmp &= ~(mask << bs);/*lint !e502*/
		WR_SMMU_MFDE_VREG(addr, tmp | ((val & mask) << bs));
		break;

	case BPD:
		RD_SMMU_BPD_VREG(addr, tmp);
		tmp &= ~(mask << bs);/*lint !e502*/
		WR_SMMU_BPD_VREG(addr, tmp | ((val & mask) << bs));
		break;

	case SCD:
		RD_SMMU_SCD_VREG(addr, tmp);
		tmp &= ~(mask << bs);/*lint !e502*/
		WR_SMMU_SCD_VREG(addr, tmp | ((val & mask) << bs));
		break;

	default:
		break;
	}
}

/**
 *function: set mfde mmu cfg register
 */
static void set_mmu_cfg_reg_mfde(SMMU_MASTER_TYPE master_type, unsigned int secure_en, unsigned int mmu_en)
{
	if (mmu_en) {    //MMU enable
		set_vdh_master_reg(master_type, REG_MFDE_MMU_CFG_EN, 0x1, 1, 12);    //[12]mmu_en=1
		if (secure_en) {  //secure
			printk(KERN_INFO "IN %s not support this mode: mmu_en:secure\n", __func__);
			printk(KERN_INFO "%s, secure_en:%d, mmu_en:%d\n", __func__, secure_en, mmu_en);
			set_vdh_master_reg(master_type, REG_MFDE_MMU_CFG_SECURE, 0x1, 1, 31);    //[31]secure_en=1
		} else { //non-secure
			set_vdh_master_reg(master_type, REG_MFDE_MMU_CFG_SECURE, 0x0, 1, 31);    //[31]secure_en=0
		}
	} else {    //MMU disable
		set_vdh_master_reg(master_type, REG_MFDE_MMU_CFG_EN, 0x0, 1, 12);    //[12]mmu_en=0
		if (secure_en) {    //secure
			set_vdh_master_reg(master_type, REG_MFDE_MMU_CFG_SECURE, 0x1, 1, 31);    //[31]secure_en=1
		} else {    //non-secure
			printk(KERN_INFO "IN %s not support this mode: non_mmu:non_secure\n", __func__);
			printk(KERN_INFO "%s, secure_en:%d, mmu_en:%d\n", __func__, secure_en, mmu_en);
			set_vdh_master_reg(master_type, REG_MFDE_MMU_CFG_SECURE, 0x0, 1, 31);	//[31]secure_en=0
		}
	}
}

/**
 *function: set bpd mmu cfg register
 */
static void set_mmu_cfg_reg_bpd(SMMU_MASTER_TYPE master_type, unsigned int secure_en, unsigned int mmu_en)
{
	if (mmu_en) {    //MMU enable
		set_vdh_master_reg(master_type, REG_BPD_MMU_CFG, 0x1, 1, 21);    //[21]mmu_en=1
		if (secure_en) {    //secure
			printk(KERN_INFO "IN %s not support this mode: mmu_en:secure\n", __func__);
			printk(KERN_INFO "%s, secure_en:%d, mmu_en:%d\n", __func__, secure_en, mmu_en);
			set_vdh_master_reg(master_type, REG_BPD_MMU_CFG, 0x1, 1, 20);    //[20]secure_en=1
		} else {    //non-secure
			set_vdh_master_reg(master_type, REG_BPD_MMU_CFG, 0x0, 1, 20);    //[20]secure_en=0
		}
	} else {    //MMU disable
		set_vdh_master_reg(master_type, REG_BPD_MMU_CFG, 0x0, 1, 21);    //[21]mmu_en=0
		if (secure_en) {    //secure
			set_vdh_master_reg(master_type, REG_BPD_MMU_CFG, 0x1, 1, 20);    //[20]secure_en=1
		} else {    //non-secure
			printk(KERN_INFO "IN %s not support this mode: non_mmu:non_secure\n", __func__);
			printk(KERN_INFO "%s, secure_en:%d, mmu_en:%d\n", __func__, secure_en, mmu_en);
			set_vdh_master_reg(master_type, REG_BPD_MMU_CFG, 0x0, 1, 20);	//[20]secure_en=0
		}
	}
}

/**
 *function: set scd mmu cfg register
 */
static void set_mmu_cfg_reg_scd(SMMU_MASTER_TYPE master_type, unsigned int secure_en, unsigned int mmu_en)
{
	if (mmu_en) {   //MMU enable
		set_vdh_master_reg(master_type, REG_SCD_MMU_CFG, 0x1, 1, 9);//[9]mmu_en=1
		set_vdh_master_reg(master_type, REG_SCD_MMU_CFG, 0x1, 1, 13);//[13]rdbuf_mmu_en=1
		if (secure_en) {    //secure
			printk(KERN_INFO "IN %s not support this mode: mmu_en:secure\n", __func__);
			printk(KERN_INFO "%s, secure_en:%d, mmu_en:%d\n", __func__, secure_en, mmu_en);
			set_vdh_master_reg(master_type, REG_SCD_MMU_CFG, 0x1, 1, 7);    //[7]secure_en=1
		} else {    //non-secure
			set_vdh_master_reg(master_type, REG_SCD_MMU_CFG, 0x0, 1, 7);    //[7]secure_en=0
		}
	} else {    //MMU disable
		set_vdh_master_reg(master_type, REG_SCD_MMU_CFG, 0x0, 1, 9);    //[9]mmu_en=0
		if (secure_en) {    //secure
			set_vdh_master_reg(master_type, REG_SCD_MMU_CFG, 0x1, 1, 7);    //[7]secure_en=1
		} else {    //non-secure
			printk(KERN_INFO "IN %s not support this mode: non_mmu:non_secure\n", __func__);
			printk(KERN_INFO "%s, secure_en:%d, mmu_en:%d\n", __func__, secure_en, mmu_en);
			set_vdh_master_reg(master_type, REG_SCD_MMU_CFG, 0x0, 1, 7);	//[7]secure_en=0
		}
	}
}

static int smmu_mem_alloc(unsigned int size, MEM_DESC_S *pMemDesc)
{
	void *virt_addr = NULL;

	if (pMemDesc == NULL) {
		printk(KERN_ERR "%s: invalid param pMemDesc is NULL\n", __func__);
		return SMMU_ERR;
	}

	if (pMemDesc->VirAddr != 0L) {
		printk(KERN_ERR "%s param StartVirAddr %pK is not NULL\n", __func__, (void*)pMemDesc->VirAddr);
		return SMMU_ERR;
	}

	if (MEM_CMA_ZERO == pMemDesc->MemType)
		virt_addr = kzalloc(size, GFP_KERNEL | GFP_DMA);    //restrict [0 ~ 4G]
	else
		virt_addr = kmalloc(size, GFP_KERNEL | GFP_DMA);    //restrict [0 ~ 4G]

	if (!virt_addr) {
		printk(KERN_ERR "%s Alloc virt_addr failed\n", __func__);
		return SMMU_ERR;
	}

	pMemDesc->VirAddr = (unsigned long long)virt_addr;
	pMemDesc->PhyAddr = __pa(virt_addr);/*lint !e648*/

	return SMMU_OK;
}

static void smmu_mem_dealloc(MEM_DESC_S *pMemDesc)
{
	if (pMemDesc == NULL) {
		printk(KERN_ERR "%s : Invalid pMemDesc is NULL\n", __func__);
		return;
	}

	if (pMemDesc->VirAddr == 0L) {
		printk(KERN_ERR "%s : Invalid pMemDesc->VirAddr is NULL\n", __func__);
		return;
	}

	kfree((void *)pMemDesc->VirAddr);
	pMemDesc->VirAddr = 0L;
	return;
}

/**
 *function: Alloc MEM for TLB miss .
 */
#ifdef ENV_ARMLINUX_KERNEL
static int alloc_smmu_tlb_miss_addr(void)
{
	int ret = SMMU_ERR;

	gAllocMem_RD.MemType = MEM_CMA_ZERO;
	ret = smmu_mem_alloc(SMMU_RWERRADDR_SIZE, &gAllocMem_RD);
	if (ret != MEM_MAN_OK) {
		printk(KERN_ERR "%s kzalloc mem for smmu rderr failed\n", __func__);
		return SMMU_ERR;
	}

	gAllocMem_WR.MemType = MEM_CMA_ZERO;
	ret = smmu_mem_alloc(SMMU_RWERRADDR_SIZE, &gAllocMem_WR);
	if (ret != MEM_MAN_OK) {
		printk(KERN_ERR "%s kzalloc mem for smmu wrerr failed\n", __func__);
		smmu_mem_dealloc(&gAllocMem_RD);
		return SMMU_ERR;
	}

	return SMMU_OK;
}
#endif

/**
 *function: init SMMU global registers.
 */
void SMMU_InitGlobalReg(void)
{
	unsigned int i = 0;

	if (gSmmuInitFlag != 1) {
		printk(KERN_DEBUG "%s Smmu initialization failed\n", __func__);
		return;
	}
	//0000 0000 0000 1111 0000 0000 0011 1000 --> 0x000f0038
	set_common_reg(SMMU_SCR, 0x0, 1, 0);//SMMU_SCR[0].glb_bypass
	set_common_reg(SMMU_SCR, 0x3, 2, 1);//SMMU_SCR[1].rqos_en SMMU_SCR[2].wqos_en

	//SMRX_S had set default value. Only need to set SMMU_SMRx_NS secure SID  bypass
	//SMMU_SMRx[0]smr_bypass=0(non-bypass); SMMU_SMRx[2:3]smr_ptw_qos=0x3;
	for (i = 0; i < SMRx_ID_SIZE; i += 2) {
		set_common_reg(SMMU_SMRx_NS + i*0x4, 0x1C, 32, 0);//0x00000003 none secure
	}

	for (i = 1; i < SMRx_ID_SIZE; i += 2) {
		set_common_reg(SMMU_SMRx_NS + i*0x4, 0x1D, 32, 0);//0x00000002 secure
	}
	set_common_reg(SMMU_CB_TTBR0, gSmmuPageBase, 32, 0);
	set_common_reg(SMMU_FAMA_CTRL1_NS, (gSmmuPageBase>>32)&0x7F, 32, 0);
	set_common_reg(SMMU_CB_TTBCR, 0x1, 1, 0);

	if (gAllocMem_RD.PhyAddr != 0 && gAllocMem_WR.PhyAddr != 0) {
		set_common_reg(SMMU_ERR_RDADDR, (gAllocMem_RD.PhyAddr & 0xFFFFFFFF), 32, 0);
		set_common_reg(SMMU_ADDR_MSB, (gAllocMem_RD.PhyAddr>>32)&0x7F, 7, 0);

		set_common_reg(SMMU_ERR_WRADDR, (gAllocMem_WR.PhyAddr & 0xFFFFFFFF), 32, 0);
		set_common_reg(SMMU_ADDR_MSB, (gAllocMem_WR.PhyAddr>>32)&0x7F, 7, 7);
	}
	//glb_bypass, 0x0: normal mode, 0x1: bypass mode
	set_master_reg(SMMU_MSTR_GLB_BYPASS, 0x0, 32, 0);    //master mmu enable
}

/**
 *function: set MFDE/SCD/BPD mmu cfg register, MMU or secure.
 */
void SMMU_SetMasterReg(SMMU_MASTER_TYPE master_type, unsigned char secure_en, unsigned char mmu_en)
{
	switch (master_type) {
	case MFDE:
		set_mmu_cfg_reg_mfde(master_type, secure_en, mmu_en);
		gMfdeSecureFlag = secure_en;
		gMfdeSmmuFlag = mmu_en;
		break;

	case SCD:
		set_mmu_cfg_reg_scd(master_type, secure_en, mmu_en);
		gScdSecureFlag = secure_en;
		gScdSmmuFlag = mmu_en;
		break;

	case BPD:
		set_mmu_cfg_reg_bpd(master_type, secure_en, mmu_en);
		gBpdSecureFlag = secure_en;
		gBpdSmmuFlag = mmu_en;
		break;

	default:
		printk(KERN_ERR "%s unkown master type %d\n", __func__, master_type);
		break;
	}
}

void SMMU_IntServProc(void)
{
	int tmp = -1;
	RD_SMMU_COMMON_VREG(SMMU_INTSTAT_NS, tmp);
	RD_SMMU_MASTER_VREG(SMMU_MSTR_DBG_0, tmp);
	RD_SMMU_MASTER_VREG(SMMU_MSTR_DBG_1, tmp);
	RD_SMMU_MASTER_VREG(SMMU_MSTR_DBG_2, tmp);
	RD_SMMU_MASTER_VREG(SMMU_MSTR_DBG_3, tmp);
	RD_SMMU_MASTER_VREG(SMMU_MSTR_DBG_4, tmp);
	RD_SMMU_MASTER_VREG(SMMU_MSTR_DBG_5, tmp);
	RD_SMMU_MASTER_VREG(SMMU_MSTR_DBG_6, tmp);
}

/**
 *function: get registers virtual address, and alloc mem for TLB miss.
 */
int SMMU_Init(void)
{
	int ret = SMMU_ERR;
	memset(&gSmmuRegVir, 0, sizeof(gSmmuRegVir));

	gSmmuRegVir.pSMMUMFDERegVirAddr = (int *) MEM_Phy2Vir(gVdhRegBaseAddr);
	if (gSmmuRegVir.pSMMUMFDERegVirAddr == NULL) {
		printk(KERN_ERR "%s pSMMUMFDERegVirAddr is NULL, SMMU Init failed\n", __func__);
		return SMMU_ERR;
	}

	gSmmuRegVir.pSMMUSCDRegVirAddr = (int *) MEM_Phy2Vir(gScdRegBaseAddr);
	if (gSmmuRegVir.pSMMUSCDRegVirAddr == NULL) {
		printk(KERN_ERR "%s pSMMUSCDRegVirAddr is NULL, SMMU Init failed\n", __func__);
		return SMMU_ERR;
	}

	gSmmuRegVir.pSMMUBPDRegVirAddr = (int *) MEM_Phy2Vir(gBpdRegBaseAddr);
	if (gSmmuRegVir.pSMMUBPDRegVirAddr == NULL) {
		printk(KERN_ERR "%s pSMMUBPDRegVirAddr is NULL, SMMU Init failed\n", __func__);
		return SMMU_ERR;
	}

	gSmmuRegVir.pSMMUCommonBaseVirAddr = (int *) MEM_Phy2Vir(HIVDEC_SMMU_COMMON_BASE_ADDR);
	if (gSmmuRegVir.pSMMUCommonBaseVirAddr == NULL) {
		printk(KERN_ERR "%s pSMMUCommonBaseVirAddr is NULL, SMMU Init failed\n", __func__);
		return SMMU_ERR;
	}

	gSmmuRegVir.pSMMUMasterBaseVirAddr = (int *) MEM_Phy2Vir(HIVDEC_SMMU_MASTER_BASE_ADDR);
	if (gSmmuRegVir.pSMMUMasterBaseVirAddr == NULL) {
		printk(KERN_ERR "%s pSMMUMasterBaseVirAddr is NULL, SMMU Init failed\n", __func__);
		return SMMU_ERR;
	}

	memset(&gAllocMem_RD, 0, sizeof(gAllocMem_RD));
	memset(&gAllocMem_WR, 0, sizeof(gAllocMem_WR));

#ifdef ENV_ARMLINUX_KERNEL
	ret = alloc_smmu_tlb_miss_addr();
	if (ret != SMMU_OK) {
		printk(KERN_ERR "%s alloc_smmu_tlb_miss_addr failed\n", __func__);
		return SMMU_ERR;
	}
#endif

	gSmmuInitFlag = 1;

	return SMMU_OK;
}

/**
 *function: free mem of SMMU_ERR_RDADDR and SMMU_ERR_WRADDR.
 */
void SMMU_DeInit(void)
{
	if (gAllocMem_RD.PhyAddr != 0)
		smmu_mem_dealloc(&gAllocMem_RD);

	if (gAllocMem_WR.PhyAddr != 0)
		smmu_mem_dealloc(&gAllocMem_WR);
}
