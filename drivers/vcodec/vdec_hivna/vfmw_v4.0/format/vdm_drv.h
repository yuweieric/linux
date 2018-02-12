#ifndef __VDM_DRV_HEADER__
#define __VDM_DRV_HEADER__
#include "../vfmw.h"
#include "../sysconfig.h"

#define VDMDRV_OK                 (0)
#define VDMDRV_ERR                (-1)

#define MSG_SLOT_SIZE             (256)

#define LUMA_HISTORGAM_NUM        (32)

#define HEVC_ONE_MSG_SLOT_LEN     (320)  // 64*5
#define MAX_FRAME_NUM             (32)
#ifdef VFMW_HEVC_SUPPORT
#define USE_MSG_SLOT_SIZE         HEVC_ONE_MSG_SLOT_LEN
#else
#define USE_MSG_SLOT_SIZE         MSG_SLOT_SIZE
#endif
typedef enum {
	VDH_SHAREFD_MESSAGE_POOL = 0,
	VDH_SHAREFD_STREAM_BUF   = 1,
	VDH_SHAREFD_PMV_BUF      = 2,
	VDH_SHAREFD_FRM_BUF      = 3,
	VDH_SHAREFD_MAX          = (VDH_SHAREFD_FRM_BUF + MAX_FRAME_NUM)
}VDH_SHAREFD;
typedef enum {
	VDH_STATE_REG   = 1,
	INT_STATE_REG   = 2,
	INT_MASK_REG    = 3,
	VCTRL_STATE_REG = 4,
} REG_ID_E;

typedef enum {
	VDM_IDLE_STATE     = 0,
	VDM_DECODE_STATE   = 1,
	VDM_REPAIR_STATE_0 = 2,
	VDM_REPAIR_STATE_1 = 3
} VDMDRV_STATEMACHINE_E;

typedef enum {
	VDMDRV_SLEEP_STAGE_NONE = 0,
	VDMDRV_SLEEP_STAGE_PREPARE,
	VDMDRV_SLEEP_STAGE_SLEEP
} VDMDRV_SLEEP_STAGE_E;

typedef enum {
	FIRST_REPAIR = 0,
	SECOND_REPAIR
} REPAIRTIME_S;

typedef enum hi_CONFIG_VDH_CMD {
	CONFIG_VDH_AfterDec_CMD = 200,
	CONFIG_VDH_ACTIVATEDEC_CMD
} CONFIG_VDH_CMD;

typedef struct {
	unsigned int vdh_reset_flag;
	unsigned int GlbResetFlag;
	int VdhStartRepairFlag;
	int VdhStartHwDecFlag;
	int VdhBasicCfg0;
	int VdhBasicCfg1;
	int VdhAvmAddr;
	int VdhVamAddr;
	int VdhStreamBaseAddr;
	int VdhEmarId;
	int VdhYstAddr;
	int VdhYstride;
	int VdhUvstride;//VREG_UVSTRIDE_1D
	int VdhCfgInfoAddr;//CFGINFO_ADDR
	int VdhUvoffset;
	int VdhRefPicType;
	int VdhFfAptEn;
	REPAIRTIME_S RepairTime;
	VID_STD_E VidStd;
	int ValidGroupNum0;
	int vdh_share_fd[VDH_SHAREFD_MAX];
	unsigned int vdhFrmBufNum;
	int IsFrmBufRemap;
	int IsPmvBufRemap;
	int IsAllBufRemap;
} OMXVDH_REG_CFG_S;

typedef struct {
	// vdm register base vir addr
	int *pVdmRegVirAddr;

	// vdm hal base addr
	unsigned int HALMemBaseAddr;
	int HALMemSize;
	int VahbStride;

	/* message pool */
	unsigned int MsgSlotAddr[256];
	int ValidMsgSlotNum;

	/* vlc code table */
	unsigned int H264TabAddr;     /* 32 Kbyte */
	unsigned int MPEG2TabAddr;    /* 32 Kbyte */
	unsigned int MPEG4TabAddr;    /* 32 Kbyte */
	unsigned int AVSTabAddr;      /* 32 Kbyte */
	unsigned int VC1TabAddr;
	/* cabac table */
	unsigned int H264MnAddr;
	/* nei info for vdh for hevc  */
	unsigned int  sed_top_phy_addr;
	unsigned int  pmv_top_phy_addr;
	unsigned int  pmv_left_phy_addr;
	unsigned int  rcn_top_phy_addr;
	unsigned int  mn_phy_addr;
	unsigned int  tile_segment_info_phy_addr;
	unsigned int  dblk_left_phy_addr;
	unsigned int  dblk_top_phy_addr;
	unsigned int  sao_left_phy_addr;
	unsigned int  sao_top_phy_addr;
	unsigned int  ppfd_phy_addr;
	int ppfd_buf_len;

	/*nei info for vdh */
	unsigned int SedTopAddr;    /* len = 64*4*x */
	unsigned int PmvTopAddr;    /* len = 64*4*x */
	unsigned int RcnTopAddr;    /* len = 64*4*x */
	unsigned int ItransTopAddr;
	unsigned int DblkTopAddr;
	unsigned int PpfdBufAddr;
	unsigned int PpfdBufLen;

	unsigned int IntensityConvTabAddr;
	unsigned int BitplaneInfoAddr;
	unsigned int Vp6TabAddr;
	unsigned int Vp8TabAddr;
	
	/* VP9 */
	unsigned int DblkLeftAddr;
	unsigned int Vp9ProbTabAddr;
	unsigned int Vp9ProbCntAddr;

	unsigned char *luma_2d_vir_addr;
	unsigned int luma_2d_phy_addr;
	unsigned char *chrom_2d_vir_addr;
	unsigned int chrom_2d_phy_addr;
} VDMHAL_HWMEM_S;

typedef struct {
	unsigned int Int_State_Reg;

	unsigned int BasicCfg1;
	unsigned int VdmState;
	unsigned int Mb0QpInCurrPic;
	unsigned int SwitchRounding;
	unsigned int SedSta;
	unsigned int SedEnd0;

	unsigned int DecCyclePerPic;
	unsigned int RdBdwidthPerPic;
	unsigned int WrBdWidthPerPic;
	unsigned int RdReqPerPic;
	unsigned int WrReqPerPic;
	unsigned int LumaSumHigh;
	unsigned int LumaSumLow;
	unsigned int LumaHistorgam[LUMA_HISTORGAM_NUM];
} VDMHAL_BACKUP_S;

extern VDMHAL_HWMEM_S g_HwMem[MAX_VDH_NUM];
#endif
