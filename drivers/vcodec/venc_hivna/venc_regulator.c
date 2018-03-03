#include <linux/clk.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/regulator/consumer.h>
#include <linux/hisi/hisi-iommu.h>

#include "venc_regulator.h"
#include "drv_venc_osal.h"
#include "drv_venc.h"

#define VENC_CLK_RATE         "enc_clk_rate"
#define VENC_REGULATOR_NAME   "ldo_venc"
#define MEDIA_REGULATOR_NAME  "ldo_media"
#define VENC_CLOCK_NAME       "clk_gate_vencfreq"

static  struct clk       *g_PvencClk        = NULL;
struct  iommu_domain     *g_hisi_mmu_domain = NULL;
static  VeduEfl_DTS_CONFIG_S g_VencDtsConfig;
static  VENC_CLK_TYPE g_currClk = VENC_CLK_RATE_LOW;

static unsigned int  g_vencQosMode  = 0x2;
static int g_VencPowerOn = 0;
/*lint -e838 -e747 -e774 -e845*/
static int Venc_Enable_Iommu(struct platform_device *pdev)
{
	struct iommu_domain *hisi_domain = NULL;
	struct iommu_domain_data* domain_data = NULL;
	//uint64_t phy_pgd_base = 0;

	if ((!pdev ) || (!(&pdev->dev))){
		HI_ERR_VENC("%s,  invalid Parameters\n", __func__);
		return HI_FAILURE;
	}

	hisi_domain = hisi_ion_enable_iommu(NULL);
	if (!hisi_domain) {
		HI_ERR_VENC("%s, hisi_ion_enable_iommu failed\n", __func__);
		return HI_FAILURE;
	}

	g_hisi_mmu_domain = hisi_domain;
	domain_data = (struct iommu_domain_data *)(g_hisi_mmu_domain->priv);
	if (domain_data == NULL){
		return HI_FAILURE;
	}

	return HI_SUCCESS;
}

static int Venc_Disable_Iommu(struct platform_device *pdev)
{
	if( g_hisi_mmu_domain && pdev) {
		g_hisi_mmu_domain = NULL;
		return HI_SUCCESS;
	}

	return HI_FAILURE;
}

static int Venc_GetDtsConfigInfo(struct platform_device *pdev, VeduEfl_DTS_CONFIG_S *pDtsConfig)
{
	unsigned int rate_h = 0;
	unsigned int rate_n = 0;
	unsigned int rate_l = 0;
	int ret    = HI_FAILURE;
	struct resource res;
	struct clk *pvenc_clk    = NULL;
	struct device_node *np   = NULL;
	struct device *dev       = &pdev->dev;
	struct iommu_domain_data *domain_data = NULL;

	if (!dev) {
		HI_FATAL_VENC("invalid argument, dev is NULL\n");
		return HI_FAILURE;
	}

	np = dev->of_node;

	HiMemSet(&res, 0, sizeof(res));
	if ((!np) || (!pDtsConfig)) {
		HI_FATAL_VENC("invalid argument np or pDtsConfig is NULL\n");
		return HI_FAILURE;
	}

	/* 1 read IRQ num from dts */
	pDtsConfig->VeduIrqNumNorm = irq_of_parse_and_map(np, 0);
	if (pDtsConfig->VeduIrqNumNorm == 0) {
		HI_FATAL_VENC("parse and map irq VeduIrqNumNorm failed\n");
		return HI_FAILURE;
	}

	pDtsConfig->VeduIrqNumProt = irq_of_parse_and_map(np, 1);
	if (pDtsConfig->VeduIrqNumProt == 0) {
		HI_FATAL_VENC("parse and map irq VeduIrqNumProt failed\n");
		return HI_FAILURE;
	}

	pDtsConfig->VeduIrqNumSafe = irq_of_parse_and_map(np, 2);
	if (pDtsConfig->VeduIrqNumSafe == 0) {
		HI_FATAL_VENC("parse and map irq VeduIrqNumSafe failed\n");
		return HI_FAILURE;
	}

	/* 2 read venc register start address, range */
	ret = of_address_to_resource(np, 0, &res);
	if (ret) {
		HI_FATAL_VENC("address to resource failed, ret value is %d\n", ret);
		return HI_FAILURE;
	}
	pDtsConfig->VencRegBaseAddr = res.start;/*lint !e712 */
	pDtsConfig->VencRegRange    = resource_size(&res);/*lint !e712 */

	/* 3 read venc clk rate [low, high], venc clock */
	pvenc_clk  = devm_clk_get(dev, VENC_CLOCK_NAME);
	if (IS_ERR_OR_NULL(pvenc_clk)) {
		HI_FATAL_VENC("can not get venc clock, pvenc_clk is 0x%pK\n", pvenc_clk);
		return HI_FAILURE;
	}
	g_PvencClk  = pvenc_clk;

	ret = of_property_read_u32_index(np, VENC_CLK_RATE, 0, &rate_h);
	ret += of_property_read_u32_index(np, VENC_CLK_RATE, 1, &rate_n);
	ret += of_property_read_u32_index(np, VENC_CLK_RATE, 2, &rate_l);
	if (ret) {
		HI_FATAL_VENC("can not get venc rate, return %d\n", ret);
		return HI_FAILURE;
	}
	pDtsConfig->highRate   = rate_h;
	pDtsConfig->normalRate = rate_n;
	pDtsConfig->lowRate    = rate_l;
	HI_INFO_VENC("venc_clk_rate: highRate:%u, normalRate:%u, lowRate:%u\n",  pDtsConfig->highRate, pDtsConfig->normalRate, pDtsConfig->lowRate);

	/* 4 get venc qos mode */
	ret = of_property_read_u32(np, "venc_qos_mode", &g_vencQosMode);
	if (ret) {
		g_vencQosMode = 0x2;
		HI_ERR_VENC("get venc qos mode failed set default\n");
	}

	domain_data = (struct iommu_domain_data *)(g_hisi_mmu_domain->priv);
	if (domain_data) {
		pDtsConfig->SmmuPageBaseAddr = (uint64_t)(domain_data->phy_pgd_base);
		HI_INFO_VENC("SmmuPageBaseAddr is 0x%pK\n", __func__, pDtsConfig->SmmuPageBaseAddr);
	}

	return HI_SUCCESS;
}

int Venc_Regulator_Init(struct platform_device *pdev)
{
	int ret = 0;

	if (!pdev) {
		HI_FATAL_VENC("invalid argument\n");
		return HI_FAILURE;
	}

	/* 1 create smmu domain */
	ret = Venc_Enable_Iommu(pdev);
	if (ret < 0) {
		HI_FATAL_VENC("enable venc iommu failed\n");
		return HI_FAILURE;
	}

	/* 2 read venc dts info from dts */
	HiMemSet(&g_VencDtsConfig, 0, sizeof(VeduEfl_DTS_CONFIG_S));
	ret = Venc_GetDtsConfigInfo(pdev, &g_VencDtsConfig);
	if (ret != HI_SUCCESS) {
		HI_FATAL_VENC("get venc DTS config info failed\n");
		return HI_FAILURE;
	}

	/* 3 set dts into to efi */
	ret = VENC_SetDtsConfig(&g_VencDtsConfig);
	if (ret != HI_SUCCESS) {
		HI_FATAL_VENC("set venc DTS config info failed\n");
		return HI_FAILURE;
	}

	return HI_SUCCESS;
}

void Venc_Regulator_Deinit(struct platform_device *pdev)
{
	if (pdev)
		Venc_Disable_Iommu(pdev);
}

int Venc_Regulator_Enable(void)
{
	int ret = HI_FAILURE;
	if (1 == g_VencPowerOn) {
		return HI_SUCCESS;
	}
	if(IS_ERR_OR_NULL(g_PvencClk)) {
		HI_FATAL_VENC("invalid_argument g_PvencClk:0x%pK\n",
				g_PvencClk);
		return HI_FAILURE;
	}

	ret = clk_prepare_enable(g_PvencClk);
	if (ret != 0) {
		HI_FATAL_VENC("prepare clk enable failed\n");
		return HI_FAILURE;
	}

	ret = clk_set_rate(g_PvencClk, g_VencDtsConfig.lowRate);
	if(ret != 0) {
		HI_FATAL_VENC("set clk low rate failed\n");
		goto on_error_prepare_clk;
	}

	g_currClk = VENC_CLK_RATE_LOW;

	ret = clk_set_rate(g_PvencClk, g_VencDtsConfig.lowRate);
	if(ret != 0) {
		HI_FATAL_VENC("set clk low rate failed\n");
		goto on_error_prepare_clk;
	}
	g_VencPowerOn = 1;

	HI_INFO_VENC("++\n");
	return HI_SUCCESS;

on_error_prepare_clk:
	clk_disable_unprepare(g_PvencClk);

	return HI_FAILURE;
}

int Venc_Regulator_Disable(void)
{
	int ret = HI_FAILURE;
	HI_INFO_VENC("Venc_Regulator_Disable\n");

	if (0 == g_VencPowerOn) {
		return HI_SUCCESS;
	}

	if(IS_ERR_OR_NULL(g_PvencClk)) {
		HI_FATAL_VENC("invalid_argument g_PvencClk:0x%pK\n",g_PvencClk);
		return HI_FAILURE;
	}

	ret = clk_set_rate(g_PvencClk, g_VencDtsConfig.lowRate);
	if(ret != 0) {
		HI_ERR_VENC("set clk lowrate:%u failed\n", g_VencDtsConfig.lowRate);
		//return HI_FAILURE;//continue, no need return
	}
	g_currClk = VENC_CLK_RATE_LOW;
	clk_disable_unprepare(g_PvencClk);

	g_VencPowerOn = 0;
	HI_INFO_VENC("--\n");
	return HI_SUCCESS;
}/*lint !e715 */
