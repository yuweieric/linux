#ifndef __VFMW_INTF_H__
#define __VFMW_INTF_H__
//#include "../omxvdec/memory.h"
#include "./format/vdm_drv.h"
#include "scd_drv.h"

#define VCTRL_OK                0
#define VCTRL_ERR              -1
#define MSG_POOL_ADDR_CHECK

typedef struct hiDRV_MEM_S {
	MEM_RECORD_S stVdhReg;
} DRV_MEM_S;
 
int VCTRL_OpenDrivers(void);

int VCTRL_OpenVfmw(void);

int VCTRL_CloseVfmw(void);

int VCTRL_VDMHal_Process(OMXVDH_REG_CFG_S *pVdmRegCfg, VDMHAL_BACKUP_S *pVdmRegStatei);

int VCTRL_SCDHal_Process(OMXSCD_REG_CFG_S *pScdRegCfg, SCD_STATE_REG_S *pScdStateReg);

int VCTRL_VDMHAL_IsRun(void);

void VCTRL_Suspend(void);

void VCTRL_Resume(void);

int VCTRL_Scen_Ident(unsigned int cmd);

#endif
