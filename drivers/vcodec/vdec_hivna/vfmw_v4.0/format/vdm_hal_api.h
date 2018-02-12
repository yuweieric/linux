#ifndef _VDM_HAL_API_HEADER_
#define _VDM_HAL_API_HEADER_

#include "mem_manage.h"
#include "vfmw.h"
#include "vdm_hal_local.h"
#include "vfmw_intf.h"

void VDMHAL_IMP_ResetVdm(int VdhId);
void VDMHAL_IMP_GlbReset(void);
void VDMHAL_IMP_ClearIntState(int VdhId);
int VDMHAL_IMP_CheckReg(REG_ID_E reg_id, int VdhId);
void VDMHAL_IMP_StartHwRepair(int VdhId);
void VDMHAL_IMP_StartHwDecode(int VdhId);
int VDMHAL_IMP_PrepareDec(OMXVDH_REG_CFG_S *pVdhRegCfg);
int VDMHAL_IMP_PrepareRepair(OMXVDH_REG_CFG_S *pVdhRegCfg);
int VDMHAL_IMP_BackupInfo(void);
void VDMHAL_IMP_GetCharacter(void);
void VDMHAL_IMP_WriteScdEMARID(void);
void VDMHAL_IMP_Init(void);
void VDMHAL_IMP_DeInit(void);
void VDMHAL_ISR(int VdhId);
int VDMHAL_HwDecProc(OMXVDH_REG_CFG_S *pVdhRegCfg);
void VDMHAL_GetRegState(VDMHAL_BACKUP_S *pVdmRegState);
int VDMHAL_IsVdmRun(int VdhId);
int VDMHAL_PrepareSleep(void);
void VDMHAL_ForceSleep(void);
void VDMHAL_ExitSleep(void);
VDMDRV_SLEEP_STAGE_E VDMHAL_GetSleepStage(void);
void VDMHAL_SetSleepStage(VDMDRV_SLEEP_STAGE_E sleepState); 
#endif
