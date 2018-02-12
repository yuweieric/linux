#ifndef  __DRV_VENC_OSAL_H__
#define  __DRV_VENC_OSAL_H__

#include <linux/rtc.h>

typedef struct hiKERN_EVENT_S
{
	wait_queue_head_t   queue_head;
	int              flag;
} KERN_EVENT_S;

typedef KERN_EVENT_S  VEDU_OSAL_EVENT;
typedef unsigned long VEDU_LOCK_FLAG;

#define  MESCS_TO_JIFFIES(time)                         msecs_to_jiffies(time)
#define  HiWaitEvent( pEvent, flag)                     wait_event_interruptible((pEvent), (flag))
#define  HiWaitEventTimeOut( pEvent, flag, msWaitTime)  wait_event_interruptible_timeout((pEvent), (flag), (msWaitTime))

extern unsigned int  g_VencPrintEnable;

typedef enum {
	VENC_FATAL = 0,
	VENC_ERR,
	VENC_WARN,
	VENC_INFO,
	VENC_DBG,
	VENC_ALW,
}VENC_PRINT_TYPE;

#define HI_FATAL_VENC(fmt,...) HI_PRINT(VENC_FATAL,(char *)__FILE__, (int)__LINE__, (char *)__FUNCTION__, fmt, ##__VA_ARGS__)
#define HI_ERR_VENC(fmt,...)   HI_PRINT(VENC_ERR,(char *)__FILE__, (int)__LINE__, (char *)__FUNCTION__, fmt, ##__VA_ARGS__)
#define HI_WARN_VENC(fmt,...)  HI_PRINT(VENC_WARN,(char *)__FILE__, (int)__LINE__, (char *)__FUNCTION__, fmt,##__VA_ARGS__)
#define HI_INFO_VENC(fmt,...)  HI_PRINT(VENC_INFO,(char *)__FILE__, (int)__LINE__, (char *)__FUNCTION__, fmt,##__VA_ARGS__)
#define HI_DBG_VENC(fmt,...)   HI_PRINT(VENC_DBG,(char *)__FILE__, (int)__LINE__, (char *)__FUNCTION__, fmt,##__VA_ARGS__)

void                   HI_PRINT(unsigned int type, char *file, int line, char *function, char *msg, ... );
unsigned int*      HiMmap(unsigned int Addr ,unsigned int Range);
void                    HiMunmap(unsigned int * pMemAddr);
int                       HiStrNCmp(const char* pStrName,const char* pDstName,int nSize);
void                    HiSleepMs(unsigned int a_MilliSec);
void*                  HiMemVAlloc(unsigned int nMemSize);
void                    HiMemVFree(void * pMemAddr);
int                       HiMemSet(void * a_pHiDstMem, int a_Value, size_t a_Size);
int                       HiMemCpy(void * a_pHiDstMem, void * a_pHiSrcMem, size_t a_Size);
void                    HiVENC_INIT_MUTEX(void* pSem);
int                       HiVENC_DOWN_INTERRUPTIBLE(void* pSem);
void                     HiVENC_UP_INTERRUPTIBLE(void* pSem);
int                       VENC_DRV_OsalIrqInit(unsigned int Irq, void(*ptrCallBack)(void));
void                    VENC_DRV_OsalIrqFree(unsigned int Irq);
int                      VENC_DRV_OsalLockCreate (void** phLock);
void                   VENC_DRV_OsalLockDestroy(void* hLock);
unsigned int       GetTimeInUs(void);
int                     VENC_DRV_OsalInitEvent( VEDU_OSAL_EVENT *pEvent, int InitVal );
int                     VENC_DRV_OsalGiveEvent( VEDU_OSAL_EVENT *pEvent );
int                     VENC_DRV_OsalWaitEvent( VEDU_OSAL_EVENT *pEvent, unsigned int msWaitTime );


#endif //__DRV_VENC_OSAL_H__

