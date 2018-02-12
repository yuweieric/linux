#ifndef __OMXVDEC_H__
#define __OMXVDEC_H__

#include "regulator.h"
#include "../vfmw_v4.0/public.h"
#include "../vfmw_v4.0/scd_drv.h"

#include <linux/ioctl.h> 
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/kthread.h>
#include <linux/seq_file.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#if defined(__KERNEL__)
#include <linux/version.h>
#endif

#define OMXVDEC_NAME                    "hi_vdec"
#define PATH_LEN                        (256)
#define OMXVDEC_VERSION         (2017032300)
#define MAX_OPEN_COUNT          (32)

#ifndef NULL
#define NULL              (0L)
#endif
 
#define HI_SUCCESS        (0)
#define HI_FAILURE        (-1)

#define RETURN_FAIL_IF_COND_IS_TRUE(cond, str)  \
do {                                       \
    if (cond)                              \
    {                                      \
        printk(KERN_CRIT "[%s : %d]- %s\n", __func__, __LINE__, str); \
        return HI_FAILURE;                 \
    }                                      \
}while(0)

#define VDEC_INIT_MUTEX(lock)              \
do {                                       \
	mutex_init(lock);                  \
} while(0)

#define VDEC_MUTEX_LOCK(lock)              \
do {                                       \
	mutex_lock(lock);                  \
} while(0)

#define VDEC_MUTEX_UNLOCK(lock)            \
do {                                       \
	mutex_unlock(lock);                \
} while(0)

typedef struct {
	unsigned char    u8IsMapVirtual;
	unsigned char    u8IsMapped;
	unsigned int      u32ShareFd;
	unsigned int      u32StartPhyAddr;
	unsigned int      u32Size;
	void *pStartVirAddr;
} MEM_BUFFER_S; 

typedef struct hi_OMXVDEC_IOCTL_MSG {
	int chan_num;
	int in_size;
	int out_size;
	void *in;
	void *out;
} OMXVDEC_IOCTL_MSG;

//Modified for 64-bit platform
typedef struct hi_COMPAT_IOCTL_MSG {
	int chan_num;
	int in_size;
	int out_size;
	compat_ulong_t in;
	compat_ulong_t out;
} COMPAT_IOCTL_MSG;

typedef struct {
	unsigned int open_count;
	atomic_t nor_chan_num;
	atomic_t sec_chan_num;
	MEM_BUFFER_S com_msg_pool;
	struct mutex omxvdec_mutex;
	struct mutex vdec_mutex_scd;
	struct mutex vdec_mutex_vdh;
	struct cdev cdev;
	struct device *device;
} OMXVDEC_ENTRY;

typedef int(*VDEC_PROC_CMD) (OMXVDEC_IOCTL_MSG *pVdecMsg);

#define VDEC_IOCTL_MAGIC 'v'

#define VDEC_IOCTL_SET_CLK_RATE      \
	_IO(VDEC_IOCTL_MAGIC, 20)

#define VDEC_IOCTL_GET_VDM_HWSTATE      \
	_IO(VDEC_IOCTL_MAGIC, 21)

#define VDEC_IOCTL_SCD_PROC      \
	_IO(VDEC_IOCTL_MAGIC, 22)

#define VDEC_IOCTL_VDM_PROC      \
	_IO(VDEC_IOCTL_MAGIC, 23)

#endif
