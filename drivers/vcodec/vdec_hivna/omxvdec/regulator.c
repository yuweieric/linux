/*
 * vdec regulator manager
 *
 * Copyright (c) 2017 Hisilicon Limited
 *
 * Author: gaoyajun<gaoyajun@hisilicon.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation.
 *
 */

#include "regulator.h"

#include <linux/hisi/hisi-iommu.h>
#include <linux/iommu.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/printk.h>

/*lint -e774*/
#define VDEC_REGULATOR_NAME     "ldo_vdec"
#define MEDIA_REGULATOR_NAME    "ldo_media"
#define VCODEC_CLOCK_NAME       "clk_gate_vdecfreq"
#define VCODEC_CLK_RATE         "dec_clk_rate"

static unsigned int  g_clock_values[] = {450000000, 300000000, 185000000};
static unsigned int  g_VdecClkRate_l  = 185000000;
static unsigned int  g_VdecClkRate_n  = 300000000;
static unsigned int  g_VdecClkRate_h  = 450000000;
static unsigned int  g_CurClkRate     = 0;
static int g_VdecPowerOn    = 0;

static struct  clk          *g_PvdecClk        = NULL;
static struct  regulator    *g_VdecRegulator   = NULL;
static struct  regulator    *g_MediaRegulator  = NULL;
static struct  iommu_domain *g_VdecSmmuDomain  = NULL;

struct mutex g_RegulatorMutex;
static VFMW_DTS_CONFIG_S g_DtsConfig;
static CLK_RATE_E g_ResumeClkType = VDEC_CLK_RATE_LOW;

#ifdef HIVDEC_SMMU_SUPPORT
/*----------------------------------------
    func: iommu enable intf
 ----------------------------------------*/
static int VDEC_Enable_Iommu(struct device *dev)
{
	g_VdecSmmuDomain = hisi_ion_enable_iommu(NULL);
	if (NULL == g_VdecSmmuDomain) {
		printk(KERN_ERR "%s hisi_ion_enable_iommu failed!\n", __func__);
		return HI_FAILURE;
	}

	return HI_SUCCESS;
}

static void VDEC_Disable_Iommu(struct device *dev)
{
	g_VdecSmmuDomain = NULL;
	if(g_VdecSmmuDomain && dev) {
		g_VdecSmmuDomain = NULL;
	}
}

static unsigned long long VDEC_GetSmmuBasePhy(struct device *dev)
{
	struct iommu_domain_data *domain_data = NULL;

	if (VDEC_Enable_Iommu(dev) == HI_FAILURE)
		return 0;

	domain_data = (struct iommu_domain_data *)(g_VdecSmmuDomain->priv);

	return (unsigned long long) (domain_data->phy_pgd_base);
}

#endif

static int read_clock_rate_value(struct device_node *np, unsigned int index, unsigned int *clock)
{
	int ret;
	ret = of_property_read_u32_index(np, VCODEC_CLK_RATE, index, clock);
	if (ret) {
		printk(KERN_CRIT "read clock rate[%d] failed\n", index);
		*clock = g_clock_values[index];
		return HI_FAILURE;
	}

	return HI_SUCCESS;
}

static int VDEC_Init_ClockRate(struct device *dev)
{
	int ret;
	struct clk *pvdec_clk  = NULL;

	pvdec_clk = devm_clk_get(dev, VCODEC_CLOCK_NAME);
	if (IS_ERR_OR_NULL(pvdec_clk)) {
		printk(KERN_CRIT "%s can not get clock\n", __func__);
		return HI_FAILURE;
	}

	g_PvdecClk = pvdec_clk;
	ret  = read_clock_rate_value(dev->of_node, 0, &g_VdecClkRate_h);
	ret += read_clock_rate_value(dev->of_node, 1, &g_VdecClkRate_n);
	ret += read_clock_rate_value(dev->of_node, 2, &g_VdecClkRate_l);
	RETURN_FAIL_IF_COND_IS_TRUE(ret, "read clock failed");

	g_CurClkRate = g_VdecClkRate_l;

	return HI_SUCCESS;
}

static int VDEC_GetDtsConfigInfo(struct device *dev, VFMW_DTS_CONFIG_S *pDtsConfig)
{
	int ret;
	struct device_node *np_crg = NULL;
	struct device_node *np     = dev->of_node;
	struct resource res;

	RETURN_FAIL_IF_COND_IS_TRUE(dev->of_node == NULL, "device node is null");
	RETURN_FAIL_IF_COND_IS_TRUE(pDtsConfig == NULL, "dts config is null");

	pDtsConfig->VdecIrqNumNorm = irq_of_parse_and_map(np, 0);
	RETURN_FAIL_IF_COND_IS_TRUE(pDtsConfig->VdecIrqNumNorm == 0, "get irq num failed");

	/*
	 FIXME irq_of_parse_and_map(np, 1);
	 FIXME irq_of_parse_and_map(np, 2);
	*/
	pDtsConfig->VdecIrqNumProt = 323;
	pDtsConfig->VdecIrqNumSafe = 324;

	/* Get reg base addr & size, return 0 if success */
	ret = of_address_to_resource(np, 0, &res);
	RETURN_FAIL_IF_COND_IS_TRUE(ret, "of_address_to_resource failed");

	pDtsConfig->VdhRegBaseAddr = res.start;
	pDtsConfig->VdhRegRange = resource_size(&res);

#ifdef HIVDEC_SMMU_SUPPORT
	/* Get reg base addr, return 0 if failed */
	pDtsConfig->SmmuPageBaseAddr = VDEC_GetSmmuBasePhy(dev);
	RETURN_FAIL_IF_COND_IS_TRUE(pDtsConfig->SmmuPageBaseAddr == 0, "get smmu base addr failed");
#endif

	np_crg = of_find_compatible_node(NULL, NULL, "hisilicon,media2-crg");
	RETURN_FAIL_IF_COND_IS_TRUE(!np_crg, "can't find media2-crg node");

	ret = of_address_to_resource(np_crg, 0, &res);
	RETURN_FAIL_IF_COND_IS_TRUE(ret, "of_address_to_resource failed");
	pDtsConfig->PERICRG_RegBaseAddr = res.start;

	ret = VDEC_Init_ClockRate(dev);
	RETURN_FAIL_IF_COND_IS_TRUE(ret != HI_SUCCESS, "init clock failed");

	return HI_SUCCESS;
}

/******************************** SHARE FUNC **********************************/

/*----------------------------------------
    func: regulator probe entry
 ----------------------------------------*/
int VDEC_Regulator_Probe(struct device *dev)
{
	int ret;
	g_VdecRegulator = NULL;
	g_MediaRegulator = NULL;

	if (dev == NULL) {
		printk(KERN_CRIT "%s, invalid params", __func__);
		return HI_FAILURE;
	}

	memset(&g_DtsConfig, 0, sizeof(g_DtsConfig)); /* unsafe_function_ignore: memset */
	ret = VDEC_GetDtsConfigInfo(dev, &g_DtsConfig);
	if (ret != HI_SUCCESS) {
		printk(KERN_CRIT "%s Regulator_GetDtsConfigInfo failed\n", __func__);
		return HI_FAILURE;
	}

	ret = VFMW_SetDtsConfig(&g_DtsConfig);
	if (ret != HI_SUCCESS) {
		printk(KERN_CRIT "%s VFMW_SetDtsConfig failed\n", __func__);
		return HI_FAILURE;
	}
	VDEC_INIT_MUTEX(&g_RegulatorMutex);

	return HI_SUCCESS;
}

/*----------------------------------------
    func: regulator deinitialize
 ----------------------------------------*/
int VDEC_Regulator_Remove(struct device * dev)
{
	VDEC_MUTEX_LOCK(&g_RegulatorMutex);

	VDEC_Disable_Iommu(dev);
	g_VdecRegulator = NULL;
	g_MediaRegulator = NULL;
	g_PvdecClk      = NULL;

	VDEC_MUTEX_UNLOCK(&g_RegulatorMutex);

	return HI_SUCCESS;
}

/*----------------------------------------
    func: enable regulator
 ----------------------------------------*/
int VDEC_Regulator_Enable(void)
{
	int ret;
	VDEC_MUTEX_LOCK(&g_RegulatorMutex);

	if (g_VdecPowerOn == 1) {
		VDEC_MUTEX_UNLOCK(&g_RegulatorMutex);
		return HI_SUCCESS;
	}

	if (g_PvdecClk == NULL) {
		printk(KERN_CRIT "%s: invalid g_PvdecClk is NULL\n", __func__);
		goto error_exit;
	}

	ret = clk_prepare_enable(g_PvdecClk);
	if (ret != 0) {
		printk(KERN_CRIT "%s clk_prepare_enable failed\n", __func__);
		goto error_exit;
	}

	ret  = clk_set_rate(g_PvdecClk, g_VdecClkRate_l);
	if (ret)
	{
		printk(KERN_CRIT "%s Failed to clk_set_rate:%u, return %d\n", __func__, g_VdecClkRate_l, ret);
		goto error_unprepare_clk;
	}

	printk(KERN_INFO "vdec regulator enable\n");
	g_VdecPowerOn = 1;

	VDEC_MUTEX_UNLOCK(&g_RegulatorMutex);

	return HI_SUCCESS;

error_unprepare_clk:
	clk_disable_unprepare(g_PvdecClk);
error_exit:
	VDEC_MUTEX_UNLOCK(&g_RegulatorMutex);

	return HI_FAILURE;
}

/*----------------------------------------
    func: disable regulator
 ----------------------------------------*/
int VDEC_Regulator_Disable(void)
{
	int ret;

	VDEC_MUTEX_LOCK(&g_RegulatorMutex);

	if (g_VdecPowerOn == 0) {
		VDEC_MUTEX_UNLOCK(&g_RegulatorMutex);
		return HI_SUCCESS;
	}

	if (g_PvdecClk == NULL) {
		printk(KERN_CRIT "%s g_PvdecClk is NULL\n", __func__);
		goto error_exit;
	}

	ret = clk_set_rate(g_PvdecClk, g_VdecClkRate_l);
	if (ret) {
		printk(KERN_CRIT "%s Failed to clk_set_rate:%u, return %d\n", __func__, g_VdecClkRate_l, ret);
		//goto error_exit;//continue, no return
	}

	clk_disable_unprepare(g_PvdecClk);

	g_VdecPowerOn = 0;

	printk(KERN_INFO "vdec regulator disable\n");

	VDEC_MUTEX_UNLOCK(&g_RegulatorMutex);

	return HI_SUCCESS;

error_exit:
	VDEC_MUTEX_UNLOCK(&g_RegulatorMutex);

	return HI_FAILURE;
}

/*----------------------------------------
    func: get decoder clock rate
 ----------------------------------------*/
void VDEC_Regulator_GetClkRate(CLK_RATE_E *pClkRate)
{
	VDEC_MUTEX_LOCK(&g_RegulatorMutex);
	*pClkRate = g_ResumeClkType;
	VDEC_MUTEX_UNLOCK(&g_RegulatorMutex);
}

int VDEC_Regulator_SetClkRate(CLK_RATE_E eClkRate)
{
	int ret          = 0;
	unsigned int rate         = 0;
	unsigned char need_set_flag = 1;

	VDEC_MUTEX_LOCK(&g_RegulatorMutex);

	if (g_DtsConfig.IsFPGA) {
		VDEC_MUTEX_UNLOCK(&g_RegulatorMutex);
		return HI_SUCCESS;
	}

	if (IS_ERR_OR_NULL(g_PvdecClk)) {
		printk(KERN_ERR "Couldn't get clk [%s]\n", __func__);
		goto error_exit;
	}

	rate = (unsigned int) clk_get_rate(g_PvdecClk);
	switch (eClkRate) {
	case VDEC_CLK_RATE_LOW:
		if (g_VdecClkRate_l == rate) {
			need_set_flag = 0;
		} else {
			rate = g_VdecClkRate_l;
			need_set_flag = 1;
		}
		break;

	case VDEC_CLK_RATE_NORMAL:
		if (g_VdecClkRate_n == rate) {
			need_set_flag = 0;
		}
		else {
			rate = g_VdecClkRate_n;
			need_set_flag = 1;
		}
		break;

	case VDEC_CLK_RATE_HIGH:
		if (g_VdecClkRate_h == rate) {
			need_set_flag = 0;
		} else {
			rate = g_VdecClkRate_h;
			need_set_flag = 1;
		}
		break;

	default:
		printk(KERN_ERR "[%s] unsupport clk rate enum %d\n", __func__, eClkRate);
		goto error_exit;
	}

	if (need_set_flag == 1) {
		ret = clk_set_rate(g_PvdecClk, rate);
		if (ret != 0) {
			printk(KERN_ERR "Failed to clk_set_rate %u HZ[%s] ret : %d\n", rate, __func__, ret);
			goto error_exit;
		}
		g_CurClkRate = rate;
		g_ResumeClkType = eClkRate;
	}

	VDEC_MUTEX_UNLOCK(&g_RegulatorMutex);

	return HI_SUCCESS;

error_exit:
	VDEC_MUTEX_UNLOCK(&g_RegulatorMutex);

	return HI_FAILURE;
}
