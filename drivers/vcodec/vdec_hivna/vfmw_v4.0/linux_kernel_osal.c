/*
 * osal
 *
 * Copyright (c) 2017 Hisilicon Limited
 *
 * Author: gaoyajun<gaoyajun@hisilicon.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation.
 *
 */

//#include "public.h"
#include "linux_kernel_osal.h"

#ifdef ENV_ARMLINUX_KERNEL

/* SpinLock */
OSAL_IRQ_SPIN_LOCK g_SpinLock_SCD;
OSAL_IRQ_SPIN_LOCK g_SpinLock_VDH;
OSAL_IRQ_SPIN_LOCK g_SpinLock_Record;

/* Mutext */
OSAL_TASK_MUTEX g_IntEvent;

OSAL_TASK_MUTEX g_ScdHwDoneEvent;
OSAL_TASK_MUTEX g_VdmHwDoneEvent;

/* Semaphore */
OSAL_SEMA g_SCDSem;
OSAL_SEMA g_VDHSem;
OSAL_SEMA g_BPDSem;

/* Extern */
extern Vfmw_Osal_Func_Ptr g_vfmw_osal_fun_ptr;

#define OSAL_Print  printk
#define MAX_WAIT_EVENT_CNT  100
#define TIME_PERIOD(begin, end) ((end >= begin) ? (end - begin) : (0xffffffff - begin + end))

unsigned int OSAL_GetTimeInMs(void)
{
	unsigned long long SysTime;

	SysTime = sched_clock();
	do_div(SysTime, 1000000);

	return (unsigned int) SysTime;
}

unsigned int OSAL_GetTimeInUs(void)
{
	unsigned long long SysTime;

	SysTime = sched_clock();
	do_div(SysTime, 1000);

	return (unsigned int) SysTime;
}

static inline int OSAL_InitEvent(OSAL_EVENT *pEvent, int InitVal)
{
	pEvent->flag = InitVal;
	init_waitqueue_head(&(pEvent->queue_head));
	return OSAL_OK;
}

static inline int OSAL_GiveEvent(OSAL_EVENT *pEvent)
{
	pEvent->flag = 1;
	wake_up_interruptible(&(pEvent->queue_head));

	return OSAL_OK;
}

static inline int OSAL_WaitEvent(OSAL_EVENT *pEvent, int msWaitTime)
{
	int ret;
	unsigned int cnt = 0;

	unsigned int start_time, cur_time;
	start_time = VFMW_OSAL_GetTimeInMs();

	do {
		ret = wait_event_interruptible_timeout((pEvent->queue_head), (pEvent->flag != 0), (msecs_to_jiffies(msWaitTime)));/*lint !e666*/
		if (ret < 0) {
			cur_time = VFMW_OSAL_GetTimeInMs();
			if (TIME_PERIOD(start_time, cur_time) > (unsigned int)msWaitTime) {
				printk(KERN_CRIT "wait event time out, time : %d, cnt: %d\n", TIME_PERIOD(start_time, cur_time), cnt);
				ret = 0;
				break;
			}
		}
		cnt++;
	} while ((pEvent->flag == 0) && (ret < 0));

	if (cnt > MAX_WAIT_EVENT_CNT) {
		printk(KERN_CRIT "the max cnt of wait_event interrupts by singal is %d\n", cnt);
	}

	if (ret  == 0) {
		printk(KERN_CRIT "wait pEvent signal timeout\n");
	}

	pEvent->flag = 0;    //(pEvent->flag>0)? (pEvent->flag-1): 0;

	return (ret != 0) ? OSAL_OK : OSAL_ERR;
}

unsigned char *OSAL_RegisterMap(unsigned int PhyAddr, unsigned int Size)
{
	return (unsigned char *) ioremap_nocache(PhyAddr, Size);
}

void OSAL_RegisterUnMap(unsigned char *VirAddr, unsigned int Size)
{
	iounmap(VirAddr);
	return;
}

int OSAL_FileWrite(char *buf, int len, struct file *filp)
{
	int writelen;
	mm_segment_t oldfs;

	if (filp == NULL)
		return -ENOENT;

	if (filp->f_op->write == NULL)
		return -ENOSYS;

	if (((filp->f_flags & O_ACCMODE) & (O_WRONLY | O_RDWR)) == 0)
		return -EACCES;

	oldfs = get_fs();
	set_fs(KERNEL_DS);/*lint !e501*/
	writelen = filp->f_op->write(filp, buf, len, &filp->f_pos);
	set_fs(oldfs);

	return writelen;
}

static inline void OSAL_SEMA_INTIT(OSAL_SEMA *pSem)
{
	sema_init(pSem, 1);
}

static inline int OSAL_DOWN_INTERRUPTIBLE(OSAL_SEMA *pSem)
{
	return down_interruptible(pSem);
}

static inline void OSAL_UP(OSAL_SEMA *pSem)
{
	up(pSem);
}

static inline void OSAL_SpinLockIRQInit(OSAL_IRQ_SPIN_LOCK *pIntrMutex)
{
	spin_lock_init(&pIntrMutex->irq_lock);
	pIntrMutex->isInit = 1;
}

static inline int OSAL_SpinLockIRQ(OSAL_IRQ_SPIN_LOCK *pIntrMutex)
{
	if (pIntrMutex->isInit == 0) {
		spin_lock_init(&pIntrMutex->irq_lock);
		pIntrMutex->isInit = 1;
	}
	spin_lock_irqsave(&pIntrMutex->irq_lock, pIntrMutex->irq_lockflags);

	return OSAL_OK;
}

static inline int OSAL_SpinUnLockIRQ(OSAL_IRQ_SPIN_LOCK *pIntrMutex)
{
	spin_unlock_irqrestore(&pIntrMutex->irq_lock, pIntrMutex->irq_lockflags);

	return OSAL_OK;
}

void OSAL_Mb(void)
{
	mb();
}

void OSAL_uDelay(unsigned long usecs)
{
	udelay(usecs);
}

void OSAL_mSleep(unsigned int msecs)
{
	msleep(msecs);
}

int OSAL_RequestIrq(unsigned int irq, OSAL_IRQ_HANDLER_t handler, unsigned long flags, const char *name, void *dev)
{
	return request_irq(irq, (irq_handler_t) handler, flags, name, dev);
}

void OSAL_FreeIrq(unsigned int irq, void *dev)
{
	free_irq(irq, dev);
}

void *OSAL_AllocVirMem(int Size)
{
	return vmalloc(Size);
}

void OSAL_FreeVirMem(void *p)
{
	if (p)
		vfree(p);
}

OSAL_IRQ_SPIN_LOCK *GetSpinLockByEnum(SpinLockType LockType)
{
	OSAL_IRQ_SPIN_LOCK *pSpinLock = NULL;

	switch (LockType) {
	case G_SPINLOCK_SCD:
		pSpinLock = &g_SpinLock_SCD;
		break;

	case G_SPINLOCK_RECORD:
		pSpinLock = &g_SpinLock_Record;
		break;

	case G_SPINLOCK_VDH:
		pSpinLock = &g_SpinLock_VDH;
		break;

	default:
		printk(KERN_ERR "%s unkown SpinLockType %d\n", __func__, LockType);
		break;
	}

	return pSpinLock;
}

void OSAL_SpinLockInit(SpinLockType LockType)
{
	OSAL_IRQ_SPIN_LOCK *pSpinLock = NULL;

	pSpinLock = GetSpinLockByEnum(LockType);

	OSAL_SpinLockIRQInit(pSpinLock);
}

int OSAL_SpinLock(SpinLockType LockType)
{
	OSAL_IRQ_SPIN_LOCK *pSpinLock = NULL;

	pSpinLock = GetSpinLockByEnum(LockType);

	return OSAL_SpinLockIRQ(pSpinLock);
}

int OSAL_SpinUnLock(SpinLockType LockType)
{
	OSAL_IRQ_SPIN_LOCK *pSpinLock = NULL;

	pSpinLock = GetSpinLockByEnum(LockType);

	return OSAL_SpinUnLockIRQ(pSpinLock);
}

OSAL_SEMA *GetSemByEnum(SemType Sem)
{
	OSAL_SEMA *pSem = NULL;

	switch (Sem) {
	case G_SCD_SEM:
		pSem = &g_SCDSem;
		break;

	case G_VDH_SEM:
		pSem = &g_VDHSem;
		break;

	case G_BPD_SEM:
		pSem = &g_BPDSem;
		break;

	default:
		printk(KERN_ERR "%s unkown SemType %d\n", __func__, Sem);
		break;
	}

	return pSem;
}

void OSAL_SemInit(SemType Sem)
{
	OSAL_SEMA *pSem = NULL;

	pSem = GetSemByEnum(Sem);

	OSAL_SEMA_INTIT(pSem);
}

int OSAL_SemDown(SemType Sem)
{
	OSAL_SEMA *pSem = NULL;

	pSem = GetSemByEnum(Sem);

	return OSAL_DOWN_INTERRUPTIBLE(pSem);
}

void OSAL_SemUp(SemType Sem)
{
	OSAL_SEMA *pSem = NULL;

	pSem = GetSemByEnum(Sem);

	OSAL_UP(pSem);
}

int OSAL_InitWaitQue(MutexType mutextType, int initVal)
{
	int retVal = OSAL_ERR;

	switch (mutextType) {
	case G_SCDHWDONEEVENT:
		retVal = OSAL_InitEvent(&g_ScdHwDoneEvent, initVal);
		break;

	case G_VDMHWDONEEVENT:
		retVal = OSAL_InitEvent(&g_VdmHwDoneEvent, initVal);
		break;

	default:
		break;
	}
	return retVal;
}

int OSAL_WakeupWaitQue(MutexType mutexType)
{
	int retVal = OSAL_ERR;

	switch (mutexType) {
	case G_SCDHWDONEEVENT:
		retVal = OSAL_GiveEvent(&g_ScdHwDoneEvent);
		break;

	case G_VDMHWDONEEVENT:
		retVal = OSAL_GiveEvent(&g_VdmHwDoneEvent);
		break;

	default:
		break;
	}

	return retVal;
}

int OSAL_WaitWaitQue(MutexType mutexType, int waitTimeInMs)
{
	int retVal = OSAL_ERR;

	switch (mutexType) {
	case G_SCDHWDONEEVENT:
		retVal = OSAL_WaitEvent(&g_ScdHwDoneEvent, waitTimeInMs);
		break;

	case G_VDMHWDONEEVENT:
		retVal = OSAL_WaitEvent(&g_VdmHwDoneEvent, waitTimeInMs);
		break;

	default:
		break;
	}

	return retVal;
}

void OSAL_InitInterface(void)
{
	memset(&g_vfmw_osal_fun_ptr, 0, sizeof(g_vfmw_osal_fun_ptr));

	g_vfmw_osal_fun_ptr.pfun_Osal_GetTimeInMs   = OSAL_GetTimeInMs;
	g_vfmw_osal_fun_ptr.pfun_Osal_GetTimeInUs   = OSAL_GetTimeInUs;
	g_vfmw_osal_fun_ptr.pfun_Osal_SpinLockInit  = OSAL_SpinLockInit;
	g_vfmw_osal_fun_ptr.pfun_Osal_SpinLock      = OSAL_SpinLock;
	g_vfmw_osal_fun_ptr.pfun_Osal_SpinUnLock    = OSAL_SpinUnLock;
	g_vfmw_osal_fun_ptr.pfun_Osal_SemaInit      = OSAL_SemInit;
	g_vfmw_osal_fun_ptr.pfun_Osal_SemaDown      = OSAL_SemDown;
	g_vfmw_osal_fun_ptr.pfun_Osal_SemaUp        = OSAL_SemUp;
	g_vfmw_osal_fun_ptr.pfun_Osal_Print         = OSAL_Print;
	g_vfmw_osal_fun_ptr.pfun_Osal_mSleep        = OSAL_mSleep;
	g_vfmw_osal_fun_ptr.pfun_Osal_Mb            = OSAL_Mb;
	g_vfmw_osal_fun_ptr.pfun_Osal_uDelay        = OSAL_uDelay;
	g_vfmw_osal_fun_ptr.pfun_Osal_InitEvent     = OSAL_InitWaitQue;
	g_vfmw_osal_fun_ptr.pfun_Osal_GiveEvent     = OSAL_WakeupWaitQue;
	g_vfmw_osal_fun_ptr.pfun_Osal_WaitEvent     = OSAL_WaitWaitQue;
	g_vfmw_osal_fun_ptr.pfun_Osal_RequestIrq    = OSAL_RequestIrq;
	g_vfmw_osal_fun_ptr.pfun_Osal_FreeIrq       = OSAL_FreeIrq;
	g_vfmw_osal_fun_ptr.pfun_Osal_RegisterMap   = OSAL_RegisterMap;
	g_vfmw_osal_fun_ptr.pfun_Osal_RegisterUnMap = OSAL_RegisterUnMap;
	g_vfmw_osal_fun_ptr.pfun_Osal_AllocVirMem   = OSAL_AllocVirMem;
	g_vfmw_osal_fun_ptr.pfun_Osal_FreeVirMem    = OSAL_FreeVirMem;
}

#endif
