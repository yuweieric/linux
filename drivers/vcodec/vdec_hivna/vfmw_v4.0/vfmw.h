#ifndef __VDEC_FIRMWARE_H__
#define __VDEC_FIRMWARE_H__

#define  VFMW_VERSION_NUM       (2017032400)
#define  TVP_CHAN_NUM            (0) 
#define  MAX_CHAN_NUM            (32) 
#define  MAX_FRAME_NUM           (32)

#define  VDEC_OK                (0)
#define  VDEC_ERR               (-1)
#define  VF_ERR_SYS             (-20)

typedef enum {
	VFMW_START_RESERVED = 0,
	VFMW_H264           = 0,
	VFMW_VC1,
	VFMW_MPEG4,
	VFMW_MPEG2,
	VFMW_H263,
	VFMW_DIVX3,
	VFMW_AVS,
	VFMW_JPEG,
	VFMW_REAL8 = 8,
	VFMW_REAL9 = 9,
	VFMW_VP6   = 10,
	VFMW_VP6F,
	VFMW_VP6A,
	VFMW_VP8,
	VFMW_VP9,
	VFMW_SORENSON,
	VFMW_MVC,
	VFMW_HEVC,
	VFMW_RAW,
	VFMW_USER,    /*## vfmw simply provide frame path. for external decoder, eg. mjpeg ## */
	VFMW_END_RESERVED
} VID_STD_E;

/*memory type*/
typedef enum {
	MEM_ION = 0,    // ion default
	MEM_ION_CTG,    // ion contigeous
	MEM_CMA,        // kmalloc
	MEM_CMA_ZERO,    // kzalloc
} MEM_TYPE_E;

/* memroy description */
typedef struct {
	unsigned char IsSecure;
	MEM_TYPE_E MemType;
	unsigned long long PhyAddr;
	unsigned int Length;
	unsigned long long VirAddr;
} MEM_DESC_S;

typedef struct {
	unsigned int IsFPGA;
	unsigned int VdecIrqNumNorm;
	unsigned int VdecIrqNumProt;
	unsigned int VdecIrqNumSafe;
	unsigned int VdhRegBaseAddr;
	unsigned int VdhRegRange;
	unsigned long long SmmuPageBaseAddr;
	unsigned int PERICRG_RegBaseAddr;
} VFMW_DTS_CONFIG_S;

#endif    // __VDEC_FIRMWARE_H__
