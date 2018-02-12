/*
* Copyright (C), 2001-2011, Hisilicon Tech. Co., Ltd.
*
* File Name     : hi_type.h
* Version       : Initial Draft
* Author        : Hisilicon multimedia software group
* Created       : 2005/4/23
*
* Last Modified :
* Description   : The common data type defination
* Function List :
*
* History       :
* 1.Date        : 2008/06/28
* Author      : c42025
* Modification: modified definition for HI_S8
*
* 2.Date        :   2008/10/31
* Author      :   z44949
* Modification:   Translate the chinese comment
*/
#ifndef __HI_DRV_MEM_H__
#define __HI_DRV_MEM_H__
#include "drv_venc.h"
#include <linux/seq_file.h>
#include <linux/vmalloc.h>
#include <linux/hisi/hisi_ion.h>


#define MAX_MEM_NAME_LEN       (15)
#define MAX_KMALLOC_MEM_NODE   (16)    /*1 channel need 2 node ,there is have max 8 channels*/
#define MAX_ION_MEM_NODE       (200)
#define SMMU_RWERRADDR_SIZE    (128)

typedef struct {
	unsigned long long RdAddr;  // HI_U64
	unsigned long long WrAddr;   // HI_U64
} VENC_SMMU_ERR_ADDR;

typedef struct {
	void  *pStartVirAddr;
	unsigned long long   u64StartPhyAddr;   // HI_U64
	unsigned int   u32Size;
	unsigned char    u8IsMapped;
	unsigned int    u32ShareFd;
} MEM_BUFFER_S;

typedef struct  {
	char            node_name[MAX_MEM_NAME_LEN];
	char            zone_name[MAX_MEM_NAME_LEN];
	void*           virt_addr;
	unsigned long long        phys_addr;   // HI_U64
	unsigned int             size;
	struct ion_handle   *handle;
} venc_mem_buf;

/***********************************************************************************
    memory menage relative functions
***********************************************************************************/
int  HI_DRV_UserCopy(struct file *file, unsigned int  cmd, unsigned long arg, long (*func)(struct file *file, unsigned int cmd, unsigned long uarg));

/**************************************platform.h**************************************************/
int  DRV_MEM_INIT(void);
int  DRV_MEM_EXIT(void);
int  DRV_MEM_KAlloc(const char* bufName, const char *zone_name, MEM_BUFFER_S *psMBuf);
int  DRV_MEM_KFree(const MEM_BUFFER_S *psMBuf);

/**************************************************************************************/


#endif /* __HI_DRV_MEM_H__ */

