/*--------------------------------------------------------------------------------------------------------------------------*/
/*!!Warning: This is a key information asset of Huawei Tech Co.,Ltd                                                         */
/*CODEMARK:kOyQZYzjDpyGdBAEC2GaWinjiDDUykL9e8pckESWBbMVmSWkBuyJO01cTiy3TdzKxGk0oBQa
mSMf7J4FkTpfvzHyMxSEsfcbL/G0fFswaAZ8tsS4we+PBWC6a/UNlzCWIaw+Ujkv9NAY+as0
fg7WZIRvw27AjvRqJbkRJvqFUORSa6KPQaSBMxCxJTGTTf//sQbjPOyYldN0OVR9ut4HFO4U
ZguGQVqcOAJQbE96v6175DqhuprKgQB8R+2fu7VD3qtX+ZJh/t0512oqv+e8YA==*/
/*--------------------------------------------------------------------------------------------------------------------------*/
#ifndef __VFMW_SYSCONFIG_HEADER__
#define __VFMW_SYSCONFIG_HEADER__

#include "vfmw.h"

/* valid vdh num */
#define MAX_VDH_NUM              (1)
/* register offset */
#define SCD_REG_OFFSET           (0xc000)
#define BPD_REG_OFFSET           (0xd000)

#define SOFTRST_REQ_OFFSET       (0xcc0c)//(0xf80c)
#define SOFTRST_OK_OFFSET        (0xcc10)//(0xf810)

#define ALL_RESET_CTRL_BIT       (0)
#define MFDE_RESET_CTRL_BIT      (1)
#define SCD_RESET_CTRL_BIT       (2)
#define BPD_RESET_CTRL_BIT       (3)

#define ALL_RESET_OK_BIT         (0)
#define MFDE_RESET_OK_BIT        (1)
#define SCD_RESET_OK_BIT         (2)
#define BPD_RESET_OK_BIT         (3)

/* FPGA flag */
extern unsigned int  gIsFPGA;

/* register base addr & range */
extern unsigned int  gVdhRegBaseAddr;
extern unsigned int  gScdRegBaseAddr;
extern unsigned int  gBpdRegBaseAddr;
extern unsigned int  gVdhRegRange;
extern unsigned int  gSOFTRST_REQ_Addr;
extern unsigned int  gSOFTRST_OK_ADDR;

/* smmu page table base addr */
extern unsigned long long  gSmmuPageBase;

/* peri crg base addr */
extern unsigned int  gPERICRG_RegBaseAddr;

/* irq num */
extern unsigned int  gVdecIrqNumNorm;
extern unsigned int  gVdecIrqNumProt;
extern unsigned int  gVdecIrqNumSafe;

#endif
