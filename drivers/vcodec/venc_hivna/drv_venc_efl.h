#ifndef __DRV_VENC_EFL_H__
#define __DRV_VENC_EFL_H__

#include "hi_drv_mem.h"
#include "drv_venc.h"

enum {
	VEDU_H265 = 0,
	VEDU_H264 = 1
};

typedef struct {
	unsigned int    IpFree;       /* for channel control */
	unsigned long long CurrHandle;//HI_U64 CurrHandle;   /* used in ISR */
	unsigned int   *pRegBase;
	void  *pChnLock;     /* lock ChnCtx[MAX_CHN] */
	void  *pTask_Frame;  /* for both venc & omxvenc */
	void  *pTask_Stream; /* juse for omxvenc */
	unsigned int    StopTask;
	unsigned int    TaskRunning;  /* to block Close IP */
	unsigned int    bReEncode;
} VeduEfl_IpCtx_S;

typedef struct {
	unsigned int IsFPGA;
	unsigned int VeduIrqNumNorm;
	unsigned int VeduIrqNumProt;
	unsigned int VeduIrqNumSafe;
	unsigned int VencRegBaseAddr;
	unsigned int VencRegRange;
	unsigned int normalRate;
	unsigned int highRate;
	unsigned int lowRate;
	unsigned long long SmmuPageBaseAddr;//HI_U64 SmmuPageBaseAddr;
} VeduEfl_DTS_CONFIG_S;

int	VENC_DRV_EflOpenVedu(void);
int	VENC_DRV_EflCloseVedu(void);
int	VENC_DRV_EflResumeVedu(void);
int    VENC_DRV_EflSuspendVedu(void);
int    VENC_SetDtsConfig(VeduEfl_DTS_CONFIG_S* info);

/*************************************************************************************/


#endif //__DRV_VENC_EFL_H__

