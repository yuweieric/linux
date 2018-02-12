#include <linux/kthread.h>
#include <linux/delay.h>

#include "drv_venc_osal.h"
#include "hi_drv_mem.h"

#define TIME_PERIOD(begin, end) ((end >= begin) ? (end - begin) : (0xffffffff - begin + end))

/*lint -e747 -e712 -e732 -e715 -e774 -e845 -e438 -e838*/
unsigned int  g_VencPrintEnable = 0xf;

char   *pszMsg[((char)VENC_ALW) + 1] = {"FATAL","ERR","WARN","IFO","DBG"}; /*lint !e785 */
char g_VencPrintMsg[1024];

static void (*ptrVencCallBack)(void);

static irqreturn_t VENC_DRV_OsalVencISR(int Irq, void *DevID)
{
	(*ptrVencCallBack)();
	return IRQ_HANDLED;
}

int VENC_DRV_OsalIrqInit( unsigned int Irq, void (*ptrCallBack)(void))
{
	int ret = 0;

	if (Irq != 0) {
		ptrVencCallBack = ptrCallBack;
		ret = request_irq(Irq, VENC_DRV_OsalVencISR, 0, "DT_device", NULL);
	} else {
		HI_FATAL_VENC("params is invaild\n");
		ret = HI_FAILURE;
	}

	if (ret == 0) {
		return HI_SUCCESS;
	} else {
		HI_FATAL_VENC("request irq failed\n");
		return HI_FAILURE;
	}
}

void VENC_DRV_OsalIrqFree(unsigned int Irq)
{
	free_irq(Irq, NULL);
}

int VENC_DRV_OsalLockCreate(void **phLock)
{
	spinlock_t *pLock = NULL;

	pLock = (spinlock_t *)vmalloc(sizeof(spinlock_t));

	if (!pLock) {
		HI_FATAL_VENC("vmalloc failed\n");
		return HI_FAILURE;
	}

	spin_lock_init( pLock );
	*phLock = pLock;

	return HI_SUCCESS;
}

void VENC_DRV_OsalLockDestroy( void* hLock )
{
	if (hLock) {
		vfree((void *)hLock);
		//hLock = NULL;
	}
}

/************************************************************************/
/* 初始化事件                                                           */
/************************************************************************/
int VENC_DRV_OsalInitEvent(VEDU_OSAL_EVENT *pEvent, int InitVal)
{
	pEvent->flag = InitVal;
	init_waitqueue_head(&(pEvent->queue_head));
	return HI_SUCCESS;
}

/************************************************************************/
/* 发出事件唤醒                                                             */
/************************************************************************/
int VENC_DRV_OsalGiveEvent(VEDU_OSAL_EVENT *pEvent)
{
	pEvent->flag = 1;
	wake_up(&(pEvent->queue_head));
	return HI_SUCCESS;
}

HI_U64 get_sys_time(void)
{
        HI_U64 sys_time;

        sys_time = sched_clock();
        do_div(sys_time, 1000000);

        return sys_time;
}

/************************************************************************/
/* 等待事件                                                             */
/* 事件发生返回OSAL_OK，超时返回OSAL_ERR 若condition不满足就阻塞等待    */
/* 被唤醒返回 0 ，超时返回非-1                                          */
/************************************************************************/
int VENC_DRV_OsalWaitEvent(VEDU_OSAL_EVENT *pEvent, unsigned int msWaitTime)
{
	int l_ret = 0;
	unsigned int cnt   = 0;

	HI_U64 start_time, cur_time;
	start_time = get_sys_time();

	do {
		l_ret = wait_event_interruptible_timeout((pEvent->queue_head), (pEvent->flag != 0), (msecs_to_jiffies(msWaitTime))); /*lint !e665 !e666 !e40 !e713 !e578*/
		if (l_ret < 0) {
			cur_time = get_sys_time();
			if (TIME_PERIOD(start_time, cur_time) > (HI_U64)msWaitTime) {
				HI_FATAL_VENC("wait event time out, time : %lld, cnt: %d\n", TIME_PERIOD(start_time, cur_time), cnt);
				l_ret = 0;
				break;
			}
		}
		cnt++;
	} while ((pEvent->flag == 0) && (l_ret < 0));

	if (cnt > 100) {
		HI_FATAL_VENC("the max cnt of wait_event interrupts by singal is %d\n", cnt);
	}

	if (l_ret  == 0) {
		HI_FATAL_VENC("wait pEvent signal timeout\n");
	}

	pEvent->flag = 0;//(pEvent->flag>0)? (pEvent->flag-1): 0;
	return (l_ret != 0) ? HI_SUCCESS : HI_FAILURE;
}

int HiMemCpy(void*a_pHiDstMem, void *a_pHiSrcMem, size_t a_Size)
{
	if ((!a_pHiDstMem) || (!a_pHiSrcMem)) {
		HI_FATAL_VENC("params is invaild\n");
		return HI_FAILURE;
	}

	memcpy((void *)a_pHiDstMem, (void *)a_pHiSrcMem, a_Size); /* unsafe_function_ignore: memcpy */
	return HI_SUCCESS;
}

int HiMemSet(void *a_pHiDstMem, int a_Value, size_t a_Size)
{
	if (!a_pHiDstMem) {
		HI_FATAL_VENC("params is invaild\n");
		return HI_FAILURE;
	}

	memset((void *)a_pHiDstMem, a_Value, a_Size); /* unsafe_function_ignore: memset */

	return HI_SUCCESS;
}

void HiSleepMs(unsigned int a_MilliSec)
{
	msleep(a_MilliSec);
}

unsigned int*  HiMmap(unsigned int Addr ,unsigned int Range)
{
	unsigned int *res_addr = NULL;
	res_addr = (unsigned int *)ioremap(Addr, Range);
	return res_addr;
}

void HiMunmap(unsigned int * pMemAddr)
{
	if (!pMemAddr) {
		HI_FATAL_VENC("params is invaild\n");
		return;
	}

	iounmap(pMemAddr);
}

int HiStrNCmp(const char* pStrName,const char* pDstName,int nSize)
{
	int ret = 0;
	if (pStrName && pDstName) {
		ret = strncmp(pStrName,pDstName,nSize);
		return ret;
	}

	return HI_FAILURE;
}

void *HiMemVAlloc(unsigned int nMemSize)
{
	void * memaddr = NULL;
	if (nMemSize) {
		memaddr = vmalloc(nMemSize);
	}
	return memaddr;
 }

void HiMemVFree(void *pMemAddr)
{
	if (pMemAddr) {
		vfree((void *)pMemAddr);
	}
}

void HiVENC_INIT_MUTEX(void *pSem)
{
	if (pSem) {
		sema_init((struct semaphore *)pSem, 1);
	}
}

int HiVENC_DOWN_INTERRUPTIBLE(void *pSem)
{
	int Ret = -1;
	if (pSem) {
		Ret = down_interruptible((struct semaphore *)pSem);
	}
	return  Ret;
}

void  HiVENC_UP_INTERRUPTIBLE(void *pSem)
{
	if (pSem) {
		up((struct semaphore *)pSem);
	}
}

void HI_PRINT(unsigned int type,char *file, int line , char *function, char *msg, ... )
{
	va_list args;
	unsigned int uTotalChar;

	if ( ((1 << type) & g_VencPrintEnable) == 0 && (type != VENC_ALW) )  /*lint !e701 */
		return ;

	va_start(args, msg);

	uTotalChar = vsnprintf(g_VencPrintMsg, sizeof(g_VencPrintMsg), msg, args);  /* unsafe_function_ignore: vsnprintf */
	g_VencPrintMsg[sizeof(g_VencPrintMsg) - 1] = '\0';

	va_end(args);

	if (uTotalChar <= 0 || uTotalChar >= 1023) /*lint !e775 */
		return;

	printk(KERN_ALERT "%s:<%d:%s>%s \n", pszMsg[type], line, function, g_VencPrintMsg);
	return;
}


