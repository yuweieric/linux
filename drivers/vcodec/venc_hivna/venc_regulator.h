#ifndef __VENC_REGULATOR_H__
#define __VENC_REGULATOR_H__
#include "drv_venc.h"
#include <linux/fs.h>
#include <linux/platform_device.h>

int  Venc_Regulator_Init(struct platform_device *pdev);
void Venc_Regulator_Deinit(struct platform_device *pdev);
int  Venc_Regulator_Enable(void);
int  Venc_Regulator_Disable(void);
#endif

