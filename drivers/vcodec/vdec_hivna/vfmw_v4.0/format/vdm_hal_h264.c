/*
 * vdec hal for h264
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
#include "vdm_hal_api.h"
#include "vdm_hal_local.h"
#include "vdm_hal_h264.h"

#include <linux/printk.h>
int H264HAL_StartDec(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
	unsigned int D32;

	D32 = 0;
	((BASIC_CFG0 *)(&D32))->mbamt_to_dec         = ((BASIC_CFG0 *)(&pVdhRegCfg->VdhBasicCfg0))->mbamt_to_dec;
	((BASIC_CFG0 *)(&D32))->load_qmatrix_flag    = 1;
	((BASIC_CFG0 *)(&D32))->marker_bit_detect_en = 0;
	((BASIC_CFG0 *)(&D32))->ac_last_detect_en    = 0;
	((BASIC_CFG0 *)(&D32))->coef_idx_detect_en   = 1;
	((BASIC_CFG0 *)(&D32))->vop_type_detect_en   = 0; 
	((BASIC_CFG0 *)(&D32))->sec_mode_en          = 0; 
	WR_VREG(VREG_BASIC_CFG0, D32, 0);

	D32 = 0;
	((BASIC_CFG1 *)(&D32))->video_standard       = 0x0;
	//((BASIC_CFG1 *)(&D32))->ddr_stride           = ((BASIC_CFG1 *)(&pVdhRegCfg->VdhBasicCfg1))->ddr_stride;
	((BASIC_CFG1 *)(&D32))->fst_slc_grp          = ((BASIC_CFG1 *)(&pVdhRegCfg->VdhBasicCfg1))->fst_slc_grp;
	((BASIC_CFG1 *)(&D32))->mv_output_en         = ((BASIC_CFG1 *)(&pVdhRegCfg->VdhBasicCfg1))->mv_output_en;
	((BASIC_CFG1 *)(&D32))->uv_order_en          = ((BASIC_CFG1 *)(&pVdhRegCfg->VdhBasicCfg1))->uv_order_en;
	((BASIC_CFG1 *)(&D32))->vdh_2d_en            = ((BASIC_CFG1 *)(&pVdhRegCfg->VdhBasicCfg1))->vdh_2d_en;
	((BASIC_CFG1 *)(&D32))->max_slcgrp_num       = 2;
	((BASIC_CFG1 *)(&D32))->compress_en          = ((BASIC_CFG1 *)(&pVdhRegCfg->VdhBasicCfg1))->compress_en;
	((BASIC_CFG1 *)(&D32))->ppfd_en              = 0;
	((BASIC_CFG1 *)(&D32))->line_num_output_en   = 0;
	WR_VREG(VREG_BASIC_CFG1, D32, 0);

	D32 = 0;
	((AVM_ADDR *)(&D32))->av_msg_addr = (pVdhRegCfg->VdhAvmAddr) & 0xFFFFFFF0;
	WR_VREG(VREG_AVM_ADDR, D32, 0);

	D32 = 0;
	((VAM_ADDR *)(&D32))->va_msg_addr = (pVdhRegCfg->VdhVamAddr) & 0xFFFFFFF0;
	WR_VREG(VREG_VAM_ADDR, D32, 0);

	D32 = 0;
	((STREAM_BASE_ADDR *)(&D32))->stream_base_addr = (pVdhRegCfg->VdhStreamBaseAddr) & 0xFFFFFFF0;
	WR_VREG(VREG_STREAM_BASE_ADDR, D32, 0);

	D32 = RD_SCDREG(REG_EMAR_ID);
	if (pVdhRegCfg->VdhEmarId == 0) {
		D32 = D32 & (~(0x100));
	} else {
		D32 = D32 | 0x100;
	}
	WR_SCDREG(REG_EMAR_ID, D32);

	D32 = 0x00300C03;
	WR_VREG(VREG_SED_TO, D32, 0);
	WR_VREG(VREG_ITRANS_TO, D32, 0);
	WR_VREG(VREG_PMV_TO, D32, 0);
	WR_VREG(VREG_PRC_TO, D32, 0);
	WR_VREG(VREG_RCN_TO, D32, 0);
	WR_VREG(VREG_DBLK_TO, D32, 0);
	WR_VREG(VREG_PPFD_TO, D32, 0);

	D32 = 0;
	((YSTADDR_1D *)(&D32))->ystaddr_1d = (pVdhRegCfg->VdhYstAddr) & 0xFFFFFFF0;
	WR_VREG(VREG_YSTADDR_1D, D32, 0);

	WR_VREG(VREG_YSTRIDE_1D, pVdhRegCfg->VdhYstride, 0);

	WR_VREG(VREG_UVOFFSET_1D, pVdhRegCfg->VdhUvoffset, 0);

	D32 = 0;
	WR_VREG(VREG_HEAD_INF_OFFSET, D32, 0);

	WR_VREG(VREG_REF_PIC_TYPE, pVdhRegCfg->VdhRefPicType, 0);

	if (pVdhRegCfg->VdhFfAptEn == 0x2) {
		D32 = 0x2;
	} else {
		D32 = 0x0;
	}
	WR_VREG(VREG_FF_APT_EN, D32, 0);

	//UVSTRIDE_1D
	WR_VREG( VREG_UVSTRIDE_1D, pVdhRegCfg->VdhUvstride, 0 );

	//CFGINFO_ADDR
	WR_VREG(VREG_CFGINFO_ADDR, pVdhRegCfg->VdhCfgInfoAddr, 0);

	//DDR_INTERLEAVE_MODE
	D32 = 0x03;
	WR_VREG(VREG_DDR_INTERLEAVE_MODE, D32, 0);

	return VDMHAL_OK;
}
