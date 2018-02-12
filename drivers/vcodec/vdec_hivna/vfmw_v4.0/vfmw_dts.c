#include "vfmw_dts.h"
#include "sysconfig.h"
//#include "public.h"
#include "omxvdec.h"
#include <linux/printk.h>

unsigned int  gIsFPGA              = 0;
unsigned int  gVdhRegBaseAddr      = 0;
unsigned int  gScdRegBaseAddr      = 0;
unsigned int  gBpdRegBaseAddr      = 0;
unsigned int  gVdhRegRange         = 0;
unsigned int  gSOFTRST_REQ_Addr    = 0;
unsigned int  gSOFTRST_OK_ADDR     = 0;
unsigned long long  gSmmuPageBase        = 0;
unsigned int  gPERICRG_RegBaseAddr = 0;

/* irq num */
unsigned int  gVdecIrqNumNorm           = 0;
unsigned int  gVdecIrqNumProt           = 0;
unsigned int  gVdecIrqNumSafe           = 0;

int VFMW_SetDtsConfig(VFMW_DTS_CONFIG_S *pDtsConfig)
{
	if (pDtsConfig == NULL) {
		printk(KERN_ERR "%s : pDtsConfig is NULL\n", __func__);
		return VDEC_ERR;
	}

	if (pDtsConfig->VdecIrqNumNorm == 0 || pDtsConfig->VdecIrqNumProt == 0 || pDtsConfig->VdecIrqNumSafe == 0   ||
	    pDtsConfig->VdhRegBaseAddr == 0 || pDtsConfig->VdhRegRange == 0    || pDtsConfig->SmmuPageBaseAddr == 0 ||
	    pDtsConfig->PERICRG_RegBaseAddr == 0) {
		printk(KERN_ERR "%s invalid param: IsFPGA : %d, VdecIrqNumNorm : %d, VdecIrqNumProt : %d, VdecIrqNumSafe : %d, VdhRegBaseAddr : %pK, VdhRegSize : %d, SmmuPageBaseAddr : %pK, PERICRG_RegBaseAddr : %pK\n", __func__,
			pDtsConfig->IsFPGA, pDtsConfig->VdecIrqNumNorm, pDtsConfig->VdecIrqNumProt, pDtsConfig->VdecIrqNumSafe, (void *)(uintptr_t)(pDtsConfig->VdhRegBaseAddr), pDtsConfig->VdhRegRange, (void *)(uintptr_t)(pDtsConfig->SmmuPageBaseAddr), (void *)(uintptr_t)(pDtsConfig->PERICRG_RegBaseAddr));
		return VDEC_ERR;
	}

	gIsFPGA              = pDtsConfig->IsFPGA;
	gVdecIrqNumNorm      = pDtsConfig->VdecIrqNumNorm;
	gVdecIrqNumProt      = pDtsConfig->VdecIrqNumProt;
	gVdecIrqNumSafe      = pDtsConfig->VdecIrqNumSafe;

	gVdhRegBaseAddr      = pDtsConfig->VdhRegBaseAddr;
	gVdhRegRange         = pDtsConfig->VdhRegRange;
	gSmmuPageBase        = pDtsConfig->SmmuPageBaseAddr;
	gPERICRG_RegBaseAddr = pDtsConfig->PERICRG_RegBaseAddr;

	gScdRegBaseAddr      = gVdhRegBaseAddr + SCD_REG_OFFSET;
	gBpdRegBaseAddr      = gVdhRegBaseAddr + BPD_REG_OFFSET;
	gSOFTRST_REQ_Addr    = gVdhRegBaseAddr + SOFTRST_REQ_OFFSET;
	gSOFTRST_OK_ADDR     = gVdhRegBaseAddr + SOFTRST_OK_OFFSET;
#if 0
		printk(KERN_ERR "%s invalid param: IsFPGA : %d, VdecIrqNumNorm : %d, VdecIrqNumProt : %d, VdecIrqNumSafe : %d, VdhRegBaseAddr : 0x%x, VdhRegSize : %d, SmmuPageBaseAddr : 0x%x, PERICRG_RegBaseAddr : 0x%x,range = 0x%x\n", __func__,
			pDtsConfig->IsFPGA, pDtsConfig->VdecIrqNumNorm, pDtsConfig->VdecIrqNumProt, pDtsConfig->VdecIrqNumSafe, (pDtsConfig->VdhRegBaseAddr), pDtsConfig->VdhRegRange, (pDtsConfig->SmmuPageBaseAddr),(uintptr_t)(pDtsConfig->PERICRG_RegBaseAddr),pDtsConfig->VdhRegRange);

#endif
	return VDEC_OK;
}

int VFMW_GetDtsConfig(VFMW_DTS_CONFIG_S *pDtsConfig)
{
	if (pDtsConfig == NULL) {
		printk(KERN_ERR "%s FATAL: pDtsConfig is NULL\n", __func__);
		return VDEC_ERR;
	}

	pDtsConfig->IsFPGA              = gIsFPGA;
	pDtsConfig->VdecIrqNumNorm      = gVdecIrqNumNorm;
	pDtsConfig->VdecIrqNumProt      = gVdecIrqNumProt;
	pDtsConfig->VdecIrqNumSafe      = gVdecIrqNumSafe;

	pDtsConfig->VdhRegBaseAddr      = gVdhRegBaseAddr;
	pDtsConfig->VdhRegRange         = gVdhRegRange;
	pDtsConfig->SmmuPageBaseAddr    = gSmmuPageBase;

	pDtsConfig->PERICRG_RegBaseAddr = gPERICRG_RegBaseAddr;

	return VDEC_OK;
}

#ifdef ENV_ARMLINUX_KERNEL
EXPORT_SYMBOL(VFMW_SetDtsConfig);
#endif
