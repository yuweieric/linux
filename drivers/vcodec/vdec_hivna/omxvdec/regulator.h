#ifndef __VDEC_REGULATOR_H__
#define __VDEC_REGULATOR_H__

#include "../vfmw_v4.0/vfmw_dts.h"
#include "omxvdec.h"
#include <linux/platform_device.h>

typedef enum {
	VDEC_CLK_RATE_LOW = 0,
	VDEC_CLK_RATE_NORMAL,
	VDEC_CLK_RATE_HIGH,
	VDEC_CLK_RATE_BUTT,
}CLK_RATE_E;
 
static const struct of_device_id Hisi_Vdec_Match_Table[] = {
	{.compatible = "hisi,kirin970-vdec",},
	{ }
}; 
int  VDEC_Regulator_Probe(struct device *dev);
int  VDEC_Regulator_Remove(struct device *dev);
int  VDEC_Regulator_Enable(void);
int  VDEC_Regulator_Disable(void);
void VDEC_Regulator_GetClkRate(CLK_RATE_E *pClkRate);
int  VDEC_Regulator_SetClkRate(CLK_RATE_E eClkRate);

#endif

