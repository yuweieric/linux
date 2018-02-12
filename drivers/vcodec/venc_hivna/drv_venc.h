#ifndef __DRV_VENC_H__
#define __DRV_VENC_H__

#include "Vedu_RegAll_Kirin970.h"
#include "drv_venc_efl.h"
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/hisi/hisi-iommu.h>
#include <linux/iommu.h>

#ifndef _M_IX86
typedef unsigned long long      HI_U64;
#else
typedef __int64                 HI_U64;
#endif

#ifndef NULL
#define NULL             0L
#endif
#define HI_SUCCESS       (0)
#define HI_FAILURE       (-1)
#define MAX_STREAMBUF_NUM  (16)
#define IOC_TYPE_VENC	  'V'

extern unsigned int b_Regular_down_flag;

typedef enum
{
	VENC_SET_CFGREG = 100,
	VENC_SET_CFGREGSIMPLE
}CMD_TYPE;

typedef enum {
	VENC_CLK_RATE_LOW = 0,
	VENC_CLK_RATE_NORMAL,
	VENC_CLK_RATE_HIGH,
} VENC_CLK_TYPE;

typedef struct
{
	int   InteralShareFd;
	int   ImageShareFd;
	int   StreamShareFd[MAX_STREAMBUF_NUM];
	int   StreamHeadShareFd;
}VENC_MEM_INFO_S;

typedef struct
{
	CMD_TYPE cmd;

	int bResetReg;
	int bClkCfg;
	int bFirstNal2Send;
	unsigned int   bSecureFlag;
	U_FUNC_VCPI_RAWINT    hw_done_type;
	S_HEVC_AVC_REGS_TYPE_CFG all_reg;
	VENC_CLK_TYPE clk_type;
	VENC_MEM_INFO_S mem_info;
}VENC_REG_INFO_S;

#define CMD_VENC_START_ENCODE          _IOWR(IOC_TYPE_VENC, 0x32, VENC_REG_INFO_S)

int VENC_DRV_BoardInit(void);
void VENC_DRV_BoardDeinit(void);
int  VENC_DRV_MemProcAdd(void);
void VENC_DRV_MemProcDel(void);

void VENC_HAL_ClrAllInt(S_HEVC_AVC_REGS_TYPE * pVeduReg);
void VENC_HAL_DisableAllInt(S_HEVC_AVC_REGS_TYPE * pVeduReg);
int    VENC_HAL_ResetReg(void);
void VENC_HAL_StartEncode(S_HEVC_AVC_REGS_TYPE * pVeduReg);
void VENC_HAL_Get_CfgRegSimple(VENC_REG_INFO_S * pVeduReg);
void VENC_HAL_Get_Reg_Venc(VENC_REG_INFO_S * pVeduReg);
void VeduHal_CfgReg_IntraSet(VENC_REG_INFO_S * channelcfg);
void VeduHal_CfgReg_LambdaSet(VENC_REG_INFO_S * channelcfg);
void VeduHal_CfgReg_QpgmapSet(VENC_REG_INFO_S * channelcfg);
void VeduHal_CfgReg_AddrSet(VENC_REG_INFO_S * channelcfg);
void VeduHal_CfgReg_SlcHeadSet(VENC_REG_INFO_S * channelcfg);
void VeduHal_CfgReg_SMMUSet(VENC_REG_INFO_S * channelcfg);
void VeduHal_CfgReg_PREMMUSet(VENC_REG_INFO_S * channelcfg);
void VeduHal_CfgRegSimple(VENC_REG_INFO_S * channelcfg);
void VeduHal_CfgReg(VENC_REG_INFO_S * regcfginfo);
void VENC_HAL_SetSmmuAddr(S_HEVC_AVC_REGS_TYPE * pVeduReg);
#endif //__DRV_VENC_H__

