/*
 * Copyright (c) 2016-2017 Linaro Ltd.
 * Copyright (c) 2016-2017 HiSilicon Technologies Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef __CVDR_DRV_PRIV_H__
#define __CVDR_DRV_PRIV_H__

/* Define the union U_CVDR_CFG */
union U_CVDR_CFG {
	/* Define the struct bits */
	struct {
		unsigned int axiwrite_du_threshold : 6; /* [5..0] */
		unsigned int reserved_0             : 2; /* [7..6] */
		unsigned int du_threshold_reached  : 8; /* [15..8] */
		unsigned int max_axiread_id         : 5; /* [20..16] */
		unsigned int reserved_1             : 3; /* [23..21] */
		unsigned int max_axiwrite_id       : 5; /* [28..24] */
		unsigned int reserved_2             : 1; /* [29] */
		unsigned int force_rd_clk_on        : 1; /* [30] */
		unsigned int force_wr_clk_on        : 1; /* [31] */
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_CVDR_DEBUG_EN */
union U_CVDR_DEBUG_EN {
	/* Define the struct bits */
	struct {
		unsigned int wr_peak_en            : 1;
		unsigned int reserved_0            : 7;
		unsigned int rd_peak_en            : 1;
		unsigned int reserved_1            : 23;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_CVDR_DEBUG */
union U_CVDR_DEBUG {
	/* Define the struct bits */
	struct {
		unsigned int wr_peak               : 8;
		unsigned int rd_peak               : 8;
		unsigned int reserved_0            : 16;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_CVDR_WR_QOS_CFG */
union U_CVDR_WR_QOS_CFG {
	/* Define the struct bits */
	struct {
		unsigned int    wr_qos_threshold_01_stop  : 4;
		unsigned int    wr_qos_threshold_01_start : 4;
		unsigned int    wr_qos_threshold_10_stop  : 4;
		unsigned int    wr_qos_threshold_10_start : 4;
		unsigned int    wr_qos_threshold_11_stop  : 4;
		unsigned int    wr_qos_threshold_11_start : 4;
		unsigned int    reserved_0            : 2;
		unsigned int    wr_qos_min            : 2;
		unsigned int    wr_qos_max            : 2;
		unsigned int    wr_qos_sr             : 2;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_CVDR_RD_QOS_CFG */
union U_CVDR_RD_QOS_CFG {
	/* Define the struct bits */
	struct {
		unsigned int    rd_qos_threshold_01_stop  : 4;
		unsigned int    rd_qos_threshold_01_start : 4;
		unsigned int    rd_qos_threshold_10_stop  : 4;
		unsigned int    rd_qos_threshold_10_start : 4;
		unsigned int    rd_qos_threshold_11_stop  : 4;
		unsigned int    rd_qos_threshold_11_start : 4;
		unsigned int    reserved_0            : 2;
		unsigned int    rd_qos_min            : 2;
		unsigned int    rd_qos_max            : 2;
		unsigned int    rd_qos_sr             : 2;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_OTHER_RO */
union U_CVDR_OTHER_RO {
	/* Define the struct bits  */
	struct {
		unsigned int other_ro  : 32;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_OTHER_RW */
union U_CVDR_OTHER_RW {
	/* Define the struct bits  */
	struct {
		unsigned int other_rw  : 32;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};


/* Define the union U_VP_WR_CFG */
union U_VP_WR_CFG {
	/* Define the struct bits */
	struct {
		unsigned int vpwr_pixel_format     : 4;
		unsigned int vpwr_pixel_expansion  : 1;
		unsigned int reserved_0            : 10;
		unsigned int vpwr_last_page        : 17;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_VP_WR_AXI_FS */
union U_VP_WR_AXI_FS {
	/* Define the struct bits */
	struct {
		unsigned int reserved_0               : 4;
		unsigned int vpwr_address_frame_start : 28;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_VP_WR_AXI_LINE */
union U_VP_WR_AXI_LINE {
	/* Define the struct bits */
	struct {
		unsigned int vpwr_line_stride  : 10;
		unsigned int reserved_0        : 5;
		unsigned int vpwr_line_wrap    : 14;
		unsigned int reserved_1        : 3;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_CVDR_RT_VP_WR_IF_CFG */
union U_VP_WR_IF_CFG {
	/* Define the struct bits */
	struct {
		unsigned int reserved_0            : 16;
		unsigned int vp_wr_stop_enable_du_threshold_reached : 1;
		unsigned int vp_wr_stop_enable_flux_ctrl : 1;
		unsigned int vp_wr_stop_enable_pressure : 1;
		unsigned int reserved_1            : 5;
		unsigned int vp_wr_stop_ok      : 1;
		unsigned int vp_wr_stop         : 1;
		unsigned int reserved_2            : 5;
		unsigned int vpwr_prefetch_bypass : 1;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};


/* Define the union U_LIMITER_VP_WR */
union U_LIMITER_VP_WR {
	/* Define the struct bits */
	struct {
		unsigned int vpwr_access_limiter_0 : 4;
		unsigned int vpwr_access_limiter_1 : 4;
		unsigned int vpwr_access_limiter_2 : 4;
		unsigned int vpwr_access_limiter_3 : 4;
		unsigned int reserved_0            : 8;
		unsigned int vpwr_access_limiter_reload : 4;
		unsigned int reserved_1            : 4;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_VP_RD_CFG */
union U_VP_RD_CFG {
	/* Define the struct bits */
	struct {
		unsigned int vprd_pixel_format    : 4;
		unsigned int vprd_pixel_expansion : 1;
		unsigned int vprd_allocated_du    : 5;
		unsigned int reserved_0           : 5;
		unsigned int vprd_last_page       : 17;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;

};

/* Define the union U_VP_RD_LWG */
union U_VP_RD_LWG {
	/* Define the struct bits */
	struct {
		unsigned int vprd_line_size           : 13;
		unsigned int reserved_0               : 3;
		unsigned int vprd_horizontal_blanking : 8;
		unsigned int reserved_1               : 8;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_VP_RD_FHG */
union U_VP_RD_FHG {
	/* Define the struct bits */
	struct {
		unsigned int vprd_frame_size        : 13;
		unsigned int reserved_0             : 3;
		unsigned int vprd_vertical_blanking : 8;
		unsigned int reserved_1             : 8;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_VP_RD_AXI_FS */
union U_VP_RD_AXI_FS {
	/* Define the struct bits */
	struct {
		unsigned int reserved_0            : 4;
		unsigned int vprd_axi_frame_start  : 28;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_VP_RD_AXI_LINE */
union U_VP_RD_AXI_LINE {
	/* Define the struct bits */
	struct {
		unsigned int vprd_line_stride   : 10;
		unsigned int reserved_0         : 6;
		unsigned int vprd_line_wrap     : 13;
		unsigned int reserved_1         : 3;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_VP_RD_IF_CFG */
union U_VP_RD_IF_CFG {
	/* Define the struct bits */
	struct {
		unsigned int    reserved_0            : 16;
		unsigned int    vp_rd_stop_enable_du_threshold_reached : 1;
		unsigned int    vp_rd_stop_enable_flux_ctrl : 1;
		unsigned int    vp_rd_stop_enable_pressure : 1;
		unsigned int    reserved_1            : 5;
		unsigned int    vp_rd_stop_ok         : 1;
		unsigned int    vp_rd_stop            : 1;
		unsigned int    reserved_2            : 5;
		unsigned int    vprd_prefetch_bypass  : 1;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_VP_RD_DEBUG */
union U_VP_RD_DEBUG {
	/* Define the struct bits  */
	struct {
		unsigned int vp_rd_debug  : 32;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_LIMITER_VP_RD */
union U_LIMITER_VP_RD {
	/* Define the struct bits */
	struct {
		unsigned int vprd_access_limiter_0 : 4;
		unsigned int vprd_access_limiter_1 : 4;
		unsigned int vprd_access_limiter_2 : 4;
		unsigned int vprd_access_limiter_3 : 4;
		unsigned int reserved_0            : 8;
		unsigned int vprd_access_limiter_reload : 4;
		unsigned int reserved_1            : 4;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_NR_WR_CFG */
union U_NR_WR_CFG {
	/* Define the struct bits */
	struct {
		unsigned int reserved_0            : 16;
		unsigned int nr_wr_stop_enable_du_threshold_reached : 1;
		unsigned int nr_wr_stop_enable_flux_ctrl : 1;
		unsigned int nr_wr_stop_enable_pressure : 1;
		unsigned int reserved_1          : 5;
		unsigned int nr_wr_stop_ok       : 1;
		unsigned int nr_wr_stop          : 1;
		unsigned int reserved_2          : 5;
		unsigned int nrwr_enable         : 1;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_NR_WR_DEBUG */
union U_NR_WR_DEBUG {
	/* Define the struct bits  */
	struct {
		unsigned int nr_wr_debug  : 32;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_LIMITER_NR_WR */
union U_LIMITER_NR_WR {
	/* Define the struct bits */
	struct {
		unsigned int nrwr_access_limiter_0 : 4;
		unsigned int nrwr_access_limiter_1 : 4;
		unsigned int nrwr_access_limiter_2 : 4;
		unsigned int nrwr_access_limiter_3 : 4;
		unsigned int reserved_0            : 8;
		unsigned int nrwr_access_limiter_reload : 4;
		unsigned int reserved_1            : 4;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_NR_RD_CFG */
union U_NR_RD_CFG {
	/* Define the struct bits */
	struct {
		unsigned int reserved_0          : 5;
		unsigned int nrrd_allocated_du   : 5;
		unsigned int reserved_1          : 6;
		unsigned int nr_rd_stop_enable_du_threshold_reached : 1;
		unsigned int nr_rd_stop_enable_flux_ctrl : 1;
		unsigned int nr_rd_stop_enable_pressure : 1;
		unsigned int reserved_2          : 5;
		unsigned int nr_rd_stop_ok       : 1;
		unsigned int nr_rd_stop          : 1;
		unsigned int reserved_3          : 5;
		unsigned int nrrd_enable         : 1;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_NR_RD_DEBUG */
union U_NR_RD_DEBUG {
	/* Define the struct bits  */
	struct {
		unsigned int nr_rd_debug          : 32;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_LIMITER_NR_RD */
union U_LIMITER_NR_RD {
	/* Define the struct bits */
	struct {
		unsigned int nrrd_access_limiter_0 : 4;
		unsigned int nrrd_access_limiter_1 : 4;
		unsigned int nrrd_access_limiter_2 : 4;
		unsigned int nrrd_access_limiter_3 : 4;
		unsigned int reserved_0            : 8;
		unsigned int nrrd_access_limiter_reload : 4;
		unsigned int reserved_1            : 4;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_DEBUG */
union U_DEBUG {
	/* Define the struct bits  */
	struct {
		unsigned int debug                : 32  ; /* [31..0]  */
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_AXI_CFG_NR_WR */
union U_AXI_CFG_NR_WR {
	/* Define the struct bits */
	struct {
		unsigned int nr_wr_mid   : 6;
		unsigned int reserved_0  : 26;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;
};

/* Define the union U_AXI_CFG_NR_RD */
union U_AXI_CFG_NR_RD {
	/* Define the struct bits */
	struct {
		unsigned int nr_rd_mid   : 6;
		unsigned int reserved_0  : 26;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;

};

/* Define the union U_AXI_CFG_VP_WR */
union U_AXI_CFG_VP_WR {
	/* Define the struct bits */
	struct {
		unsigned int vp_wr_mid   : 6;
		unsigned int reserved_0  : 26;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;

};

/* Define the union U_AXI_CFG_VP_RD */
union U_AXI_CFG_VP_RD {
	/* Define the struct bits */
	struct {
		unsigned int vp_rd_mid  : 6;
		unsigned int reserved_0 : 26;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;

};

/* Define the union U_SPARE */
union U_SPARE {
	/* Define the struct bits  */
	struct {
		unsigned int spare  : 32;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;

};

/* Define the union U_VP_WR_SMOOTHING */
union U_VP_WR_SMOOTHING {
	/* Define the struct bits */
	struct {
		unsigned int vpwr_smoothing_access_limiter_0 : 4;
		unsigned int vpwr_smoothing_access_limiter_1 : 4;
		unsigned int vpwr_smoothing_access_limiter_2 : 4;
		unsigned int vpwr_smoothing_access_limiter_3 : 4;
		unsigned int vpwr_smoothing_threshold        : 7;
		unsigned int reserved_0                      : 9;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;

};

/* Define the union U_VP_WR_DEBUG */
union U_VP_WR_DEBUG {
	/* Define the struct bits  */
	struct {
		unsigned int vp_wr_debug  : 32;
	} bits;

	/* Define an unsigned member */
	unsigned int    u32;

};

struct S_VP_WR {
	union U_VP_WR_CFG            VP_WR_CFG;
	union U_VP_WR_AXI_FS         VP_WR_AXI_FS;
	union U_VP_WR_AXI_LINE       VP_WR_AXI_LINE;
	union U_VP_WR_IF_CFG         VP_WR_IF_CFG;
};

struct S_VP_RD {
	union U_VP_RD_CFG            VP_RD_CFG;
	union U_VP_RD_LWG            VP_RD_LWG;
	union U_VP_RD_FHG            VP_RD_FHG;
	union U_VP_RD_AXI_FS         VP_RD_AXI_FS;
	union U_VP_RD_AXI_LINE       VP_RD_AXI_LINE;
	union U_VP_RD_IF_CFG         VP_RD_IF_CFG;
	unsigned int                 VP_RD_DEBUG;
};

struct S_NR_WR {
	union U_NR_WR_CFG            NR_WR_CFG;
	union U_NR_WR_DEBUG          NR_WR_DEBUG;
	union U_LIMITER_NR_WR        LIMITER_NR_WR;
	unsigned int                 reserved_0;
};

struct S_NR_RD {
	union U_NR_RD_CFG            NR_RD_CFG;
	union U_NR_RD_DEBUG          NR_RD_DEBUG;
	union U_LIMITER_NR_RD        LIMITER_NR_RD;
	unsigned int                 reserved_0;
};

/* Define the global struct */
struct S_CVDR_REGS_TYPE {
	union U_CVDR_CFG          CVDR_CFG             ; /* 0x0 */
	union U_CVDR_DEBUG_EN     CVDR_DEBUG_EN        ; /* 0x4 */
	union U_CVDR_DEBUG        CVDR_DEBUG           ; /* 0x8 */
	union U_CVDR_WR_QOS_CFG   CVDR_WR_QOS_CFG      ; /* 0xc */
	union U_CVDR_RD_QOS_CFG   CVDR_RD_QOS_CFG      ; /* 0x10 */
	union U_CVDR_OTHER_RO     CVDR_OTHER_RO        ; /* 0x14 */
	union U_CVDR_OTHER_RW     CVDR_OTHER_RW        ; /* 0x18 */
	struct S_VP_WR             VP_WR[38]            ; /* 0x1c~0x278 */
	unsigned int        reserved_0[73]       ; /* 0x27c~0x39c */
	union U_LIMITER_VP_WR     LIMITER_VP_WR[38]    ; /* 0x400~0x494 */
	unsigned int        reserved_1[2]        ; /* 0x498~0x49c */
	struct S_VP_RD             VP_RD[22]            ; /* 0x500~0x7bc */
	unsigned int        reserved_2[48]       ; /* 0x7c0~0x87c */
	union U_LIMITER_VP_RD     LIMITER_VP_RD[22]    ; /* 0x880~0x8d4 */
	unsigned int        reserved_3[10]       ; /* 0x8d8~0x8fc */
	struct S_NR_WR             NR_WR[4]             ; /* 0x900~0x93c */
	unsigned int        reserved_4[48]       ; /* 0x940~0x9fc */
	struct S_NR_RD            NR_RD[8]            ; /* 0xa00~0xa7c */
	unsigned int        reserved_5[32]       ; /* 0xa80~0xafc */
	union U_DEBUG             DEBUG[16]            ; /* 0xb00~0xb3c */
	unsigned int        reserved_6[48]       ; /* 0xb40~0xbfc */
	union U_AXI_CFG_NR_WR     AXI_CFG_NR_WR[4]     ; /* 0xc00~0xc0c */
	unsigned int        reserved_7[12]       ; /* 0xc10~0xc3c */
	union U_AXI_CFG_NR_RD     AXI_CFG_NR_RD[8]     ; /* 0xc40~0xc5c */
	unsigned int        reserved_8[8]        ; /* 0xc60~0xc7c */
	union U_AXI_CFG_VP_WR     AXI_CFG_VP_WR[38]    ; /* 0xc80~0xd14 */
	unsigned int        reserved_9[26]       ; /* 0xd18~0xd7c */
	union U_AXI_CFG_VP_RD     AXI_CFG_VP_RD[22]    ; /* 0xd80~0xdd4 */
	unsigned int        reserved_10[10]      ; /* 0xdd8~0xdfc */
	union U_SPARE             SPARE[4]             ; /* 0xe00~0xe0c */
	union U_VP_WR_SMOOTHING   VP_WR_SMOOTHING[20]  ; /* 0xe10~0xe5c */
	unsigned int        reserved_11[40]      ; /* 0xe60~0xefc */
	union U_VP_WR_DEBUG       VP_WR_DEBUG[38]      ; /* 0xf00~0xf94 */
};

#endif /* __CVDR_DRV_PRIV_H__ */
