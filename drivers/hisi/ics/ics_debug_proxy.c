#include "ipu_clock.h"
#include "ipu_smmu_drv.h"
#include "ics_debug_proxy.h"

struct ioctl_out_params {
	bool ret_directly;
	void *memory_node;
	//TODO: add more out params here
};

extern struct cambricon_ipu_private *adapter;
extern int regulator_ip_vipu_enable(void);
extern int regulator_ip_vipu_disable(void);
extern void ipu_reset_proc(unsigned int addr);
extern void ipu_interrupt_init(void);
extern long ipu_set_profile(struct cambricon_ipu_private *, unsigned long, struct ioctl_out_params *);

int call_regulator_ip_vipu_enable(void)
{
	return regulator_ip_vipu_enable();
}
EXPORT_SYMBOL(call_regulator_ip_vipu_enable);

int call_regulator_ip_vipu_disable(void)
{
	return regulator_ip_vipu_disable();
}
EXPORT_SYMBOL(call_regulator_ip_vipu_disable);

void call_ipu_reset_proc(unsigned int addr)
{
	ipu_reset_proc(addr);
}
EXPORT_SYMBOL(call_ipu_reset_proc);

void call_ipu_interrupt_init(void)
{
	ipu_interrupt_init();
}
EXPORT_SYMBOL(call_ipu_interrupt_init);

void * get_ipu_adapter(void)
{
	return adapter;
}
EXPORT_SYMBOL(get_ipu_adapter);

int call_ipu_clock_start(void *clock)
{
	return ipu_clock_start(clock);
}
EXPORT_SYMBOL(call_ipu_clock_start);

int call_ipu_clock_set_rate(void *clock, unsigned int clock_rate)
{
	return ipu_clock_set_rate(clock, clock_rate);
}
EXPORT_SYMBOL(call_ipu_clock_set_rate);

void call_ipu_clock_stop(void *clock)
{
	ipu_clock_stop(clock);
}
EXPORT_SYMBOL(call_ipu_clock_stop);

void call_ipu_smmu_init(unsigned long ttbr0, unsigned long smmu_rw_err_phy_addr, bool port_sel, bool hardware_start)
{
	ipu_smmu_init(ttbr0, smmu_rw_err_phy_addr, port_sel, hardware_start);
}
EXPORT_SYMBOL(call_ipu_smmu_init);

void call_ipu_set_profile(unsigned long profile)
{
	ipu_set_profile(0, profile, 0);
}
EXPORT_SYMBOL(call_ipu_set_profile);

