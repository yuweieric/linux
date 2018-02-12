#ifndef __PUBLIC_H__
#define __PUBLIC_H__

#include "vfmw.h"

/* 0X0 : ALWYS, 0X1: ALWYS and FATAL, 0X3: ALWYS and FATAL and ERROR */
#define  DEFAULT_PRINT_ENABLE   (0x3)

typedef enum {
	DEV_SCREEN = 1,
	DEV_SYSLOG,
	DEV_FILE,
	DEV_MEM
} PRINT_DEVICE_TYPE;


#if 0
#define dprint_vfmw_nothing(type, fmt, arg...)    ({do{}while(0);0;})

#define dprint_sos_kernel(type, fmt, arg...)                          \
do{                                                                   \
    if ((PRN_ALWS == type) || (0 != (DEFAULT_PRINT_ENABLE & (1LL << type)))) \
    {    \
         printk(KERN_ALERT "VDEC S: "fmt, ##arg);                                  \
    }    \
}while(0)

#define dprint_linux_kernel(type, fmt, arg...)                        \
do{                                                                   \
    if ((PRN_ALWS == type) || (0 != (DEFAULT_PRINT_ENABLE & (1LL << type))))  \
    {    \
            printk(KERN_ALERT "VDEC : "fmt, ##arg);      \
    }    \
}while(0)

#ifdef HI_ADVCA_FUNCTION_RELEASE
#define dprint(type, fmt, arg...)  dprint_vfmw_nothing(type, fmt, ##arg)
#else
 
#ifdef ENV_ARMLINUX_KERNEL
#define dprint(type, fmt, arg...)  dprint_linux_kernel(type, fmt, ##arg)
#else
#define dprint(type, fmt, arg...)  dprint_vfmw_nothing(type, fmt, ##arg)
#endif
 
#endif
#else
#ifdef HI_ADVCA_FUNCTION_RELEASE
#define dprint(type, fmt, arg...)  
#else
//#define dprint(type, fmt, arg...)  printk(type fmt, ##arg)
#define dprint(type, fmt, arg...)  
#endif
#endif
#endif
