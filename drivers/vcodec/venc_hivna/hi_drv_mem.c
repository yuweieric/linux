#include "drv_venc_osal.h"
#include "hi_drv_mem.h"
#include <asm/uaccess.h>


#define  MAX_BUFFER_SIZE (10*1024)

char *g_sbuf = NULL;
int   g_venc_node_num = 0;

struct semaphore    g_VencMemSem;
venc_mem_buf g_venc_mem_node[MAX_KMALLOC_MEM_NODE];

VENC_SMMU_ERR_ADDR g_smmu_err_mem;

int DRV_MEM_INIT(void)
{
	char *sbuf;
	int s32Ret;
	MEM_BUFFER_S  MEM_SMMU_RD_ADDR;
	MEM_BUFFER_S  MEM_SMMU_WR_ADDR;

	HiVENC_INIT_MUTEX(&g_VencMemSem);

	sbuf = HiMemVAlloc(MAX_BUFFER_SIZE);
	if (!sbuf) {
		HI_FATAL_VENC("call vmalloc failed\n");
		return HI_FAILURE;
	}

	HiMemSet((void *)&g_venc_mem_node, 0, MAX_KMALLOC_MEM_NODE*sizeof(g_venc_mem_node[0]));
	HiMemSet((void *)&g_smmu_err_mem, 0, sizeof(g_smmu_err_mem));
	HiMemSet((void *)&MEM_SMMU_RD_ADDR, 0, sizeof(MEM_BUFFER_S));
	HiMemSet((void *)&MEM_SMMU_WR_ADDR, 0, sizeof(MEM_BUFFER_S));

	MEM_SMMU_RD_ADDR.u32Size = SMMU_RWERRADDR_SIZE;
	s32Ret = DRV_MEM_KAlloc("SMMU_RDERR", "OMXVENC", &MEM_SMMU_RD_ADDR);
	if (s32Ret != HI_SUCCESS ) {
		HI_ERR_VENC("SMMU_RDERR alloc failed\n");
		goto err_sbuf_exit;
	}

	MEM_SMMU_WR_ADDR.u32Size = SMMU_RWERRADDR_SIZE;
	s32Ret = DRV_MEM_KAlloc("SMMU_WRERR", "OMXVENC", &MEM_SMMU_WR_ADDR);
	if (s32Ret != HI_SUCCESS ) {
		HI_ERR_VENC("SMMU_WRERR alloc failed\n");
		goto err_rd_smmu_exit;
	}

	g_smmu_err_mem.RdAddr = MEM_SMMU_RD_ADDR.u64StartPhyAddr;//config alloc phyaddr,in order system don't dump
	g_smmu_err_mem.WrAddr = MEM_SMMU_WR_ADDR.u64StartPhyAddr;
	g_sbuf = sbuf;
	HiMemSet((void *)g_sbuf, 0, MAX_BUFFER_SIZE);

	return HI_SUCCESS;

err_rd_smmu_exit:
	DRV_MEM_KFree(&MEM_SMMU_RD_ADDR);
err_sbuf_exit:
	HiMemVFree(sbuf);
	
	return HI_FAILURE;

}

int DRV_MEM_EXIT(void)
{
	int i;

	if (g_sbuf) {
		HiMemVFree(g_sbuf);
		g_sbuf = NULL;
	}

	/* Exit kfree mem for register's VEDU_COMN1_REGS.COMN1_SMMU_ERR_RDADDRR*/
	for (i = 0; i < MAX_KMALLOC_MEM_NODE; i++) {
		if (g_venc_mem_node[i].virt_addr != NULL) {
			kfree(g_venc_mem_node[i].virt_addr);
			HiMemSet(&g_venc_mem_node[i], 0, sizeof(g_venc_mem_node[i]));
		}
	}

	g_venc_node_num = 0;

	return HI_SUCCESS;
}

/* kalloc */
int DRV_MEM_KAlloc(const char* bufName, const char *zone_name, MEM_BUFFER_S *psMBuf)
{
	unsigned int  i;
	int  ret = HI_FAILURE;
	void *virt_addr = NULL;

	if (psMBuf == NULL || psMBuf->u32Size == 0) {
		HI_FATAL_VENC("invalid Param, psMBuf is NULL or size is zero\n");
		return ret;
	}

	if (HiVENC_DOWN_INTERRUPTIBLE(&g_VencMemSem)) {
		HI_FATAL_VENC("Kalloc, down_interruptible failed\n");
		return ret;
	}

	for (i = 0; i < MAX_KMALLOC_MEM_NODE; i++) {
		if ((0 == g_venc_mem_node[i].phys_addr) && (g_venc_mem_node[i].virt_addr == NULL)) {
			break;
		}
	}

	if (i == MAX_KMALLOC_MEM_NODE) {
		HI_FATAL_VENC("No free ion mem node\n");
		goto err_exit;
	}

	virt_addr = kmalloc(psMBuf->u32Size, GFP_KERNEL | GFP_DMA);/*lint !e747*/
	if (IS_ERR_OR_NULL(virt_addr)) {
		HI_FATAL_VENC("call kzalloc failed, size : %d\n", psMBuf->u32Size);
		goto err_exit;
	}

	memset(virt_addr, 0, psMBuf->u32Size); /* unsafe_function_ignore: memset */ /*lint !e668*/

	psMBuf->pStartVirAddr   = virt_addr;
	psMBuf->u64StartPhyAddr = __pa(virt_addr);/*lint !e648 !e834 !e712*/

	snprintf(g_venc_mem_node[i].node_name, MAX_MEM_NAME_LEN, "%s", bufName);  /* unsafe_function_ignore: snprintf */

	snprintf(g_venc_mem_node[i].zone_name, MAX_MEM_NAME_LEN, "%s", zone_name);  /* unsafe_function_ignore: snprintf */

	g_venc_mem_node[i].virt_addr = psMBuf->pStartVirAddr;
	g_venc_mem_node[i].phys_addr = psMBuf->u64StartPhyAddr;
	g_venc_mem_node[i].size      = psMBuf->u32Size;

	g_venc_node_num++;

	ret = HI_SUCCESS;

err_exit:
	HiVENC_UP_INTERRUPTIBLE(&g_VencMemSem);
	return ret;   /*lint !e593*/
}   /*lint !e593*/

/* kfree */
int DRV_MEM_KFree(const MEM_BUFFER_S *psMBuf)
{
        unsigned int  i;
        int  ret = HI_FAILURE;

        if (NULL == psMBuf || psMBuf->pStartVirAddr == NULL || psMBuf->u64StartPhyAddr == 0) {
                HI_FATAL_VENC("invalid Parameters\n");
                return ret;
        }

        if (HiVENC_DOWN_INTERRUPTIBLE(&g_VencMemSem)) {
                HI_FATAL_VENC("Kfree, down interruptible failed\n");
                return ret;
        }


        for (i=0; i<MAX_KMALLOC_MEM_NODE; i++) {
                if ((psMBuf->u64StartPhyAddr == g_venc_mem_node[i].phys_addr) &&
                        (psMBuf->pStartVirAddr == g_venc_mem_node[i].virt_addr))
                {
                        break;
                }
        }

        if(i == MAX_KMALLOC_MEM_NODE) {
                HI_FATAL_VENC("No free ion mem node\n");
                goto err_exit;
        }

        kfree(g_venc_mem_node[i].virt_addr);
        HiMemSet(&g_venc_mem_node[i], 0, sizeof(g_venc_mem_node[i]));/*lint !e866 */
        g_venc_node_num = (g_venc_node_num > 0)?(g_venc_node_num-1):0;

        ret = HI_SUCCESS;

err_exit:
        HiVENC_UP_INTERRUPTIBLE(&g_VencMemSem);
        return ret;
}

int HI_DRV_UserCopy(struct file *file, unsigned int cmd, unsigned long arg,
			long (*func)(struct file *file, unsigned int cmd, unsigned long uarg))
{
	//HI_CHAR  sbuf[768];
	void  *parg = NULL;
	int   err   = -EINVAL;

	/*  Copy arguments into temp kernel buffer  */
	if (!(void __user*)arg) {
		HI_FATAL_VENC("arg is NULL\n");
		goto out;
	}

	if (_IOC_SIZE(cmd) <= MAX_BUFFER_SIZE) {
		parg = g_sbuf;
	} else {
		HI_FATAL_VENC("cmd size is too long\n");
		goto out;
	}

	if (!parg) {
		HI_FATAL_VENC("parg is NULL\n");
		goto out;
	}
	err = -EFAULT;
	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(parg, (void __user*)arg, _IOC_SIZE(cmd))) {/*lint !e747 */
			HI_FATAL_VENC("copy_from_user failed, cmd value is 0x%x\n", cmd);
			goto out;
		}
	}

	/* call driver */
	err = func(file, cmd, (long)(parg)); /*lint !e732 !e712*/
	if (err == -ENOIOCTLCMD)
		err = -EINVAL;
	if (err < 0)
		goto out;

	/*  Copy results into user buffer  */
	switch (_IOC_DIR(cmd)) {
	case _IOC_READ:
	case (_IOC_WRITE | _IOC_READ):
		if (copy_to_user((void __user *)arg, parg, _IOC_SIZE(cmd))) {/*lint !e747 */
			HI_FATAL_VENC("copy_to_user failed, cmd value is 0x%x\n", cmd);
			err = -EFAULT;
		}
		break;
	default:
		goto out;
	}
out:
	return err;
}

