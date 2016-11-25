/*
 * Texas Instruments TUSB422 Power Delivery
 *
 * Author: Dan Murphy <dmurphy@ti.com>
 *
 * Copyright: (C) 2016 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/hrtimer.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include "tusb422_linux.h"
#include "tcpci.h"
#include "tcpm.h"
#include "tusb422_common.h"
#include "usb_pd_pal.h"
#include "usb_pd_policy_engine.h"

struct tusb422_pwr_delivery {
	struct device *dev;
	struct power_supply *ps;
	struct regmap *regmap;
	struct i2c_client *client;
	struct src_pdo_t *source_pwr;
	struct snk_pdo_t *sink_pwr;
	struct gpio_desc *alert_gpio;
	struct gpio_desc *vbus_src_gpio;
	struct gpio_desc *vbus_snk_gpio;
	struct gpio_desc *vconn_gpio;
	struct gpio_desc *vbus_hv_gpio;
	struct gpio_desc *vbus_5v_gpio;
    struct hrtimer timer;
    bool timer_expired;
	struct work_struct work;
	void (*call_back) (unsigned int);
	tcpc_config_t *configuration;
	usb_pd_port_config_t *port_config;
	int alert_irq;
	int id;
	int alert_status;
};

static struct tusb422_pwr_delivery *tusb422_pd;

/* Remove this line to disable debug */
#define TUSB422_DEBUG

#ifdef TUSB422_DEBUG
/* The registers can be accessed via
 * cat /sys/class/i2c-adapter/i2c-1/1-0020/registers
 * And written through echo for example
 * echo "CFG_STAND_OUT 0x00" > /sys/class/i2c-adapter/i2c-2/2-0020/registers
 */
struct tusb422_reg {
	const char *name;
	uint8_t reg;
} tusb422_regs[] = {
	{ "VEN_ID_0", TUSB422_VENDOR_ID_0 },
	{ "VEN_ID_1", TUSB422_VENDOR_ID_1 },
	{ "PROD_ID_0", TUSB422_PRODUCT_ID_0 },
	{ "PROD_ID_1", TUSB422_PRODUCT_ID_1 },
	{ "DEV_ID_0", TUSB422_DEV_ID_0 },
	{ "DEV_ID_1", TUSB422_DEV_ID_1 },
	{ "TYPEC_REV_0",  TUSB422_USBTYPEC_REV_0 },
	{ "TYPEC_REV_1", TUSB422_USBTYPEC_REV_1 },
	{ "USBPD_REV_0", TUSB422_USBPD_REV_VER_0 },
	{ "USBPD_REV_1", TUSB422_USBPD_REV_VER_1 },
	{ "TCPC_REV_0", TUSB422_PD_INTERFACE_REV_0 },
	{ "TCPC_REV_1", TUSB422_PD_INTERFACE_REV_1 },
	{ "CFG_STD_OUT", TUSB422_CFG_STD_OUT },
	{ "ALERT_0", TUSB422_ALERT_0 },
	{ "ALERT_1", TUSB422_ALERT_1 },
	{ "ALERT_MASK_0", TUSB422_ALERT_MASK_0 },
	{ "ALERT_MASK_1", TUSB422_ALERT_MASK_1 },
	{ "POWER_STATUS_MASK", TUSB422_POWER_STATUS_MASK },
	{ "FAULT_STAT_MASK", TUSB422_FAULT_STATUS_MASK },
	{ "TCPC_CTRL", TUSB422_TCPC_CONTROL },
	{ "ROLE_CTRL", TUSB422_ROLE_CONTROL },
	{ "FAULT_CTRL", TUSB422_FAULT_CONTROL },
	{ "PWR_CTRL", TUSB422_PWR_CONTROL },
	{ "CC_STATUS", TUSB422_CC_STATUS },
	{ "POWER_STATUS", TUSB422_PWR_STATUS },
	{ "FAULT_STATUS", TUSB422_FAULT_STATUS },
	{ "TI_INT_STATUS", TUSB422_INT_STATUS },
	{ "TI_INT_STATUS_MASK", TUSB422_INT_STATUS_MASK },
};

static int tusb422_check_vendor(struct device *dev)
{
	int ret = 0;
	int i = 0;
	unsigned int read_buf[4] = {0};
	struct tusb422_pwr_delivery *data = dev_get_drvdata(dev);

	for(i = 0; i < 4; i++) {
		ret = regmap_read(data->regmap, i, &read_buf[i]);
		if(ret < 0) {
			return ret;
		}
	}

	return 0;
}

static ssize_t tusb422_registers_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	unsigned i, n, reg_count;
	unsigned int read_buf;
    unsigned int cc1, cc2;
    struct tusb422_pwr_delivery *data = dev_get_drvdata(dev);
    tcpc_device_t *tcpc_dev = tcpm_get_device(0);
    usb_pd_port_t *pd_dev = usb_pd_pe_get_device(0);


	reg_count = sizeof(tusb422_regs) / sizeof(tusb422_regs[0]);
	for (i = 0, n = 0; i < reg_count; i++) {
		regmap_read(data->regmap, tusb422_regs[i].reg, &read_buf);
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "%-20s = 0x%02X\n",
			       tusb422_regs[i].name,
			       read_buf);
	}

    n += scnprintf(buf + n, PAGE_SIZE - n,
                   "---------------------------------------\n");

    n += scnprintf(buf + n, PAGE_SIZE - n,
                   "Type-C State = %s\n",
                   tcstate2string[tcpc_dev->state]);

    if (*pd_dev->state < PE_NUM_STATES) {
        n += scnprintf(buf + n, PAGE_SIZE - n,
                       "USB-PD State = %s\n",
                       pdstate2string[*pd_dev->state]);
    }

    cc1 = TCPC_CC1_STATE(tcpc_dev->cc_status);
    cc2 = TCPC_CC2_STATE(tcpc_dev->cc_status);

    if ((tcpc_dev->state == TCPC_STATE_ATTACHED_SNK) ||
        (tcpc_dev->state == TCPC_STATE_DEBUG_ACC_SNK)) {

        if (cc1 > cc2)
        {
            // CC1 used for USB-PD.
            n += scnprintf(buf + n, PAGE_SIZE - n,
                           "Source Current Advertisement (CC1) = %s\n",
                           (tcpc_dev->src_current_adv == CC_SNK_STATE_DEFAULT) ? "500/900 mA" :
                           (tcpc_dev->src_current_adv == CC_SNK_STATE_POWER15) ? "1.5A" :
                           (tcpc_dev->src_current_adv == CC_SNK_STATE_POWER30) ? "3.0A" : "N/A");

            n += scnprintf(buf + n, PAGE_SIZE - n,
                           "CC2 = %s\n",
                           (cc2 == CC_SNK_STATE_DEFAULT) ? "500/900 mA" :
                           (cc2 == CC_SNK_STATE_POWER15) ? "1.5A" :
                           (cc2 == CC_SNK_STATE_POWER30) ? "3.0A" : "Open");

        }
        else
        {
            // CC2 used for USB-PD.
            n += scnprintf(buf + n, PAGE_SIZE - n,
                           "Source Current Advertisement (CC2) = %s\n",
                           (tcpc_dev->src_current_adv == CC_SNK_STATE_DEFAULT) ? "500/900 mA" :
                           (tcpc_dev->src_current_adv == CC_SNK_STATE_POWER15) ? "1.5A" :
                           (tcpc_dev->src_current_adv == CC_SNK_STATE_POWER30) ? "3.0A" : "N/A");

            n += scnprintf(buf + n, PAGE_SIZE - n,
                           "CC1 = %s\n",
                           (cc1 == CC_SNK_STATE_DEFAULT) ? "500/900 mA" :
                           (cc1 == CC_SNK_STATE_POWER15) ? "1.5A" :
                           (cc1 == CC_SNK_STATE_POWER30) ? "3.0A" : "Open");
        }
    }
    else
    {
        if (!(tcpc_dev->cc_status & CC_STATUS_LOOKING4CONNECTION)){
            if (!(tcpc_dev->cc_status & CC_STATUS_CONNECT_RESULT)) {  /* presenting Rp */

                n += scnprintf(buf + n, PAGE_SIZE - n,
                               "CC1 state = %s\n",
                               (cc1 == CC_SRC_STATE_OPEN) ? "Open" :
                               (cc1 == CC_SRC_STATE_RA) ? "Ra" :
                               (cc1 == CC_SRC_STATE_RD) ? "Rd" : "?");

                n += scnprintf(buf + n, PAGE_SIZE - n,
                               "CC2 state = %s\n",
                               (cc2 == CC_SRC_STATE_OPEN) ? "Open" :
                               (cc2 == CC_SRC_STATE_RA) ? "Ra" :
                               (cc2 == CC_SRC_STATE_RD) ? "Rd" : "?");
            }
        }
    }

    n += scnprintf(buf + n, PAGE_SIZE - n,
                   "---------------------------------------\n");

	return n;
}

static ssize_t tusb422_registers_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	unsigned i, reg_count, value;
	int error = 0;
	char name[30];
	struct tusb422_pwr_delivery *data = dev_get_drvdata(dev);

	if (count >= 30) {
		pr_err("%s:input too long\n", __func__);
		return -1;
	}

	if (sscanf(buf, "%s %x", name, &value) != 2) {
		pr_err("%s:unable to parse input\n", __func__);
		return -1;
	}

	reg_count = sizeof(tusb422_regs) / sizeof(tusb422_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, tusb422_regs[i].name)) {
			error = regmap_write(data->regmap, tusb422_regs[i].reg, value);
			if (error) {
				pr_err("%s:Failed to write %s\n",
					__func__, name);
				return -1;
			}
			return count;
		}
	}
	pr_err("%s:no such register %s\n", __func__, name);
	return -1;
}

static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
		tusb422_registers_show, tusb422_registers_store);

static struct attribute *tusb422_attrs[] = {
	&dev_attr_registers.attr,
	NULL
};

static const struct attribute_group tusb422_attr_group = {
	.attrs = tusb422_attrs,
};
#endif

static const struct reg_default tusb422_reg_defs[] = {
	{ TUSB422_ALERT_0, 0x0},
	{ TUSB422_ALERT_1, 0x0},
	{ TUSB422_ALERT_MASK_0, 0xff },
	{ TUSB422_ALERT_MASK_1, 0xf },
	{ TUSB422_POWER_STATUS_MASK, 0xff },
	{ TUSB422_FAULT_STATUS_MASK, 0x7f },
	{ TUSB422_CFG_STD_OUT, 0x60 },
	{ TUSB422_TCPC_CONTROL, 0x0 },
	{ TUSB422_ROLE_CONTROL, 0x1f },
	{ TUSB422_FAULT_CONTROL, 0x6 },
	{ TUSB422_PWR_CONTROL, 0x60 },
	{ TUSB422_CC_STATUS, 0 },
	{ TUSB422_PWR_STATUS, 0 },
	{ TUSB422_FAULT_STATUS,0 },
};

int tusb422_write(int reg, int value, int num_of_regs)
{
//	printk("%s:reg 0x%X, val 0x%X, num of regs %i\n", __func__, reg, value, num_of_regs);
	regmap_raw_write_async(tusb422_pd->regmap, reg, &value, num_of_regs);

	return 0;
};

int tusb422_write_block(int reg, int *data, int num_of_regs)
{
//	printk("%s:reg 0x%X, val 0x%X, num of regs %i\n", __func__, reg, value, num_of_regs);
	regmap_raw_write_async(tusb422_pd->regmap, reg, data, num_of_regs);

	return 0;
};

int tusb422_read(int reg, int *value, int num_of_regs)
{
	int ret;

//	printk("%s: reg 0x%X, num of regs %i\n", __func__, reg, num_of_regs);
	ret = regmap_raw_read(tusb422_pd->regmap, reg, value, num_of_regs);
//	printk("%s: value 0x%X %i\n", __func__, (uint16_t) *value, ret);

	return ret;
};

int tusb422_modify_reg(int reg, int clr_mask, int set_mask)
{
	regmap_update_bits(tusb422_pd->regmap, reg, (clr_mask | set_mask), set_mask);
	return 0;
};

void tusb422_set_timer_func(void (*function)(unsigned int))
{
	tusb422_pd->call_back=function;
};

void tusb422_clr_timer_func(void)
{
	tusb422_pd->call_back = NULL;
};

int tusb422_start_timer(unsigned int timeout_ms)
{
//	printk("%s: Enter\n", __func__);

    hrtimer_try_to_cancel(&tusb422_pd->timer);
//    hrtimer_cancel(&tusb422_pd->timer);

	if (hrtimer_active(&tusb422_pd->timer))
    {
        printk("\n##### Timer active\n\n");
		return -1;
    }

	hrtimer_start(&tusb422_pd->timer, ms_to_ktime(timeout_ms), HRTIMER_MODE_REL);

	return 0;
};

int tusb422_stop_timer(void)
{
//	printk("%s: Enter\n", __func__);
	hrtimer_cancel(&tusb422_pd->timer);

	return 0;
};

// BQ - use for short sleeps < 20ms.
void tusb422_msleep(int msecs)
{
    udelay(msecs * 1000);
}

int tusb422_set_vbus(int vbus_sel)
{
//	printk("%s: sel = %i\n", __func__, vbus_sel);

	if (vbus_sel == VBUS_SRC_5V) {
		/* Disable high voltage. */
		gpiod_direction_output(tusb422_pd->vbus_hv_gpio, 0);
		/* Enable 5V. */
		gpiod_direction_output(tusb422_pd->vbus_5v_gpio, 1);
		/* Enable SRC switch. */
		gpiod_direction_output(tusb422_pd->vbus_src_gpio, 0);
	} else if (vbus_sel == VBUS_SRC_HI_VOLT) {
		/* Disable 5v */
		gpiod_direction_output(tusb422_pd->vbus_5v_gpio, 0);
		/* Enable high voltage. */
		gpiod_direction_output(tusb422_pd->vbus_hv_gpio, 1);
		/* Enable SRC switch. */
		gpiod_direction_output(tusb422_pd->vbus_src_gpio, 0);
	} else if (vbus_sel == VBUS_SNK) {
		/* Enable SNK switch. */
		gpiod_direction_output(tusb422_pd->vbus_snk_gpio, 0);
	}

	return 0;
};

int tusb422_clr_vbus(int vbus_sel)
{
//	printk("%s: sel = %i\n", __func__, vbus_sel);

	if (vbus_sel == VBUS_SRC_5V) {
		/* Disable SRC switch. */
		gpiod_direction_output(tusb422_pd->vbus_src_gpio, 1);
		/* Disable 5V. */
		gpiod_direction_output(tusb422_pd->vbus_5v_gpio, 0);
	} else if (vbus_sel == VBUS_SRC_HI_VOLT) {
		/* Disable high voltage. */
		gpiod_direction_output(tusb422_pd->vbus_hv_gpio, 0);
	} else if (vbus_sel == VBUS_SNK) {
		/* Disable SNK switch. */
		gpiod_direction_output(tusb422_pd->vbus_snk_gpio, 1);
	}

	return 0;
};

static irqreturn_t tusb422_event_handler(int irq, void *private)
{
//	printk("%s: Enter\n", __func__);

	tusb422_pd->alert_status = 1;

//    tcpm_alert_event(0);

    schedule_work(&tusb422_pd->work);

//	tcpm_connection_task();
//	usb_pd_task();

//	tusb422_pd->alert_status = 0;

	return IRQ_HANDLED;
};

static int tusb422_of_get_gpios(struct tusb422_pwr_delivery *tusb422_pd)
{
	int ret;

	tusb422_pd->alert_gpio = devm_gpiod_get(tusb422_pd->dev, "ti,alert", GPIOD_IN);
	if (IS_ERR(tusb422_pd->alert_gpio)) {
		ret = PTR_ERR(tusb422_pd->alert_gpio);
		if (ret != -ENOENT && ret != -ENOSYS) {
			dev_err(tusb422_pd->dev,
				"Failed to allocate data_ready gpio\n");
			return ret;
		}
		tusb422_pd->alert_gpio = NULL;
	} else {
		gpiod_direction_input(tusb422_pd->alert_gpio);
		tusb422_pd->alert_irq = gpiod_to_irq(tusb422_pd->alert_gpio);
	}

#ifndef USB_PD_PAL

	tusb422_pd->vbus_snk_gpio = devm_gpiod_get(tusb422_pd->dev, "ti,vbus-snk",
					GPIOD_OUT_LOW);
	if (IS_ERR(tusb422_pd->vbus_snk_gpio)) {
		ret = PTR_ERR(tusb422_pd->vbus_snk_gpio);
		if (ret != -ENOENT && ret != -ENOSYS) {
			dev_err(tusb422_pd->dev,
				"Failed to allocate data_ready gpio\n");
			return ret;
		}
		tusb422_pd->vbus_snk_gpio = NULL;
	}

	tusb422_pd->vbus_src_gpio = devm_gpiod_get(tusb422_pd->dev, "ti,vbus-src",
					GPIOD_OUT_LOW);
	if (IS_ERR(tusb422_pd->vbus_src_gpio)) {
		ret = PTR_ERR(tusb422_pd->vbus_src_gpio);
		if (ret != -ENOENT && ret != -ENOSYS) {
			dev_err(tusb422_pd->dev,
				"Failed to allocate data_ready gpio\n");
			return ret;
		}
		tusb422_pd->vbus_src_gpio = NULL;
	}

	tusb422_pd->vconn_gpio = devm_gpiod_get(tusb422_pd->dev, "ti,vconn-en",
					GPIOD_OUT_LOW);
	if (IS_ERR(tusb422_pd->vconn_gpio)) {
		ret = PTR_ERR(tusb422_pd->vconn_gpio);
		if (ret != -ENOENT && ret != -ENOSYS) {
			dev_err(tusb422_pd->dev,
				"Failed to allocate data_ready gpio\n");
			return ret;
		}
		tusb422_pd->vconn_gpio = NULL;
	}

	tusb422_pd->vbus_hv_gpio = devm_gpiod_get(tusb422_pd->dev, "ti,vbus-hv",
					GPIOD_OUT_LOW);
	if (IS_ERR(tusb422_pd->vbus_hv_gpio)) {
		ret = PTR_ERR(tusb422_pd->vbus_hv_gpio);
		if (ret != -ENOENT && ret != -ENOSYS) {
			dev_err(tusb422_pd->dev,
				"Failed to allocate data_ready gpio\n");
			return ret;
		}
		tusb422_pd->vbus_hv_gpio = NULL;
	}

	tusb422_pd->vbus_5v_gpio = devm_gpiod_get(tusb422_pd->dev, "ti,vbus-5v",
					GPIOD_OUT_LOW);
	if (IS_ERR(tusb422_pd->vbus_5v_gpio)) {
		ret = PTR_ERR(tusb422_pd->vbus_5v_gpio);
		if (ret != -ENOENT && ret != -ENOSYS) {
			dev_err(tusb422_pd->dev,
				"Failed to allocate data_ready gpio\n");
			return ret;
		}
		tusb422_pd->vbus_5v_gpio = NULL;
	}
#endif

	return 0;
};

static int tusb422_of_init(struct tusb422_pwr_delivery *tusb422_pd)
{
	struct device_node *of_node = tusb422_pd->dev->of_node;
	struct device_node *pp;
	unsigned int supply_type;
	unsigned int min_volt, current_flow, peak_current, pdo;
	unsigned int max_volt, max_current, max_power, fast_role_support;
	unsigned int op_current, min_current, op_power, priority;
	int ret;
	int num_of_sink = 0, num_of_src = 0;

	struct device *dev = tusb422_pd->dev;

	tusb422_pd->port_config = devm_kzalloc(dev,
			sizeof(*tusb422_pd->port_config), GFP_KERNEL);

	if (!tusb422_pd->port_config)
		return -ENOMEM;

	if (of_property_read_bool(of_node, "ti,usb_comm_capable"))
		tusb422_pd->port_config->usb_comm_capable = true;

	if (of_property_read_bool(of_node, "ti,usb_suspend_supported"))
		tusb422_pd->port_config->usb_suspend_supported = true;

	if (of_property_read_bool(of_node, "ti,externally_powered"))
		tusb422_pd->port_config->externally_powered = true;

	if (of_property_read_bool(of_node, "ti,dual_role_data"))
		tusb422_pd->port_config->dual_role_data = true;

	if (of_property_read_bool(of_node, "ti,unchunked_msg_support"))
		tusb422_pd->port_config->unchunked_msg_support = true;

	if (of_property_read_bool(of_node, "ti,higher_capability"))
		tusb422_pd->port_config->higher_capability = true;

	if (of_property_read_bool(of_node, "ti,giveback_flag"))
		tusb422_pd->port_config->giveback_flag = true;

	if (of_property_read_bool(of_node, "ti,no_usb_suspend"))
		tusb422_pd->port_config->no_usb_suspend = true;

	ret = of_property_read_u16(of_node, "ti,src_settling_time_ms",
			&tusb422_pd->port_config->src_settling_time_ms);
	if (ret)
		printk("%s: Missing src_settling_time_ms\n", __func__);

    if (tusb422_pd->port_config->src_settling_time_ms < 50)
    {
        // Mandate at least 50ms settling time.
        tusb422_pd->port_config->src_settling_time_ms = 50;
    }

	ret = of_property_read_u32(of_node, "ti,fast_role_support",
			&fast_role_support);
	if (ret)
		printk("%s: Missing fast_role_support\n", __func__);
	else
		tusb422_pd->port_config->fast_role_support = (fast_role_t) fast_role_support;

	ret = of_property_read_u32(of_node, "ti,priority", &priority);
	if (ret)
		printk("%s: Missing priority\n", __func__);
	else
		tusb422_pd->port_config->priority = (pdo_priority_t) priority;


	for_each_child_of_node(of_node, pp) {
		ret = of_property_read_u32(pp, "ti,current-flow",
					   &current_flow);
		if (ret)
			return ret;

		ret = of_property_read_u32(pp, "ti,supply-type",
					   &supply_type);
		if (ret)
			return ret;

		ret = of_property_read_u32(pp, "ti,min-voltage",
					   &min_volt);
		if (ret)
			return ret;
		ret = of_property_read_u32(pp, "ti,max-voltage",
					   &max_volt);
		if (ret)
			return ret;

		ret = of_property_read_u32(pp, "ti,pdo_number",
					   &pdo);
		if (ret)
			return ret;

		switch (supply_type) {
		case SUPPLY_TYPE_BATTERY:
			ret = of_property_read_u32(pp, "ti,peak_current",
						   &peak_current);
			if (ret)
				return ret;

			if (current_flow == 0) {
				num_of_src++;
				ret = of_property_read_u32(pp, "ti,max_power",
							   &max_power);
				if (ret)
					return ret;

				tusb422_pd->port_config->src_caps[pdo].MaxPower = max_power;
			} else if (current_flow == 1) {
				num_of_sink++;
			} else {
				printk("%s: Undefined current flow\n", __func__);
			}

	/*MaxPower
	MaxOperatingPower
	MinOperatingPower
	OperationalPower*/

			break;
		case SUPPLY_TYPE_FIXED:
			ret = of_property_read_u32(pp, "ti,peak_current",
						   &peak_current);
			if (ret)
				printk("%s: Missing peak current\n", __func__);

			if (current_flow == 0) {
				num_of_src++;
				ret = of_property_read_u32(pp, "ti,max_current",
							   &max_current);
				if (ret)
					printk("%s: Missing max current\n", __func__);

				tusb422_pd->port_config->src_caps[pdo].SupplyType = supply_type;
				tusb422_pd->port_config->src_caps[pdo].PeakI = peak_current;
				tusb422_pd->port_config->src_caps[pdo].MinV = PDO_VOLT(min_volt);
				tusb422_pd->port_config->src_caps[pdo].MaxV = PDO_VOLT(max_volt);
				tusb422_pd->port_config->src_caps[pdo].MaxI = PDO_CURR(max_current);

			} else if (current_flow == 1) {
				num_of_sink++;

				ret = of_property_read_u32(pp, "ti,max-operating-curr",
							   &max_current);
				if (ret)
					printk("%s: Missing max op current\n", __func__);

				ret = of_property_read_u32(pp, "ti,min-operating-curr",
							   &min_current);
				if (ret)
					printk("%s: Missing min op current\n", __func__);

				ret = of_property_read_u32(pp, "ti,operational-curr",
							   &op_current);
				if (ret)
					printk("%s: Missing op_curr\n", __func__);

				ret = of_property_read_u32(pp, "ti,operational-pwr",
							   &op_power);
				if (ret)
					printk("%s: Missing op_power\n", __func__);

				tusb422_pd->port_config->snk_caps[pdo].SupplyType = supply_type;
				tusb422_pd->port_config->snk_caps[pdo].PeakI = peak_current;
				tusb422_pd->port_config->snk_caps[pdo].MinV = PDO_VOLT(min_volt);
				tusb422_pd->port_config->snk_caps[pdo].MaxV = PDO_VOLT(max_volt);
				tusb422_pd->port_config->snk_caps[pdo].MaxOperatingCurrent = PDO_CURR(max_current);
				tusb422_pd->port_config->snk_caps[pdo].MinOperatingCurrent = PDO_CURR(min_current);
				tusb422_pd->port_config->snk_caps[pdo].OperationalCurrent = PDO_CURR(op_current);
				tusb422_pd->port_config->snk_caps[pdo].MaxOperatingPower = 0; /* N/A */
				tusb422_pd->port_config->snk_caps[pdo].MinOperatingPower = 0; /* N/A */
				tusb422_pd->port_config->snk_caps[pdo].OperationalPower = op_power;  /* N/A */
			} else {
				printk("%s: Undefined current flow\n", __func__);
			}

			break;
		case SUPPLY_TYPE_VARIABLE:
	/*PeakI
	MaxI
	MaxOperatingCurrent
	MinOperatingCurrent
	OperationalCurrent*/
			break;
		default:
			return -ENODEV;
			break;
		};
	};

	tusb422_pd->port_config->num_snk_pdos = num_of_sink;
	tusb422_pd->port_config->num_src_pdos = num_of_src;

	usb_pd_init(tusb422_pd->port_config);

	return 0;
}

static enum hrtimer_restart tusb422_timer_tasklet(struct hrtimer *hrtimer)
{
	struct tusb422_pwr_delivery *tusb422_pwr = container_of(hrtimer, struct tusb422_pwr_delivery, timer);

    tusb422_pd->timer_expired = true;
    schedule_work(&tusb422_pwr->work);

	return HRTIMER_NORESTART;
}

static void tusb422_work(struct work_struct *work)
{
	struct tusb422_pwr_delivery *tusb422_pwr = container_of(work, struct tusb422_pwr_delivery, work);
//	int i;
//	printk("%s: Enter\n", __func__);

	if (tusb422_pd->alert_status)
    {
        tusb422_pd->alert_status = 0;
        tcpm_alert_event(0);
    }

////	printk("%s: alert_status %i\n", __func__, tusb422_pwr->alert_status);
//	if (tusb422_pwr->alert_status) {
//		for(i = 0; i <= 2000; i++) {
//            udelay(500);
////			printk("%s: alert_status %i\n", __func__, tusb422_pwr->alert_status);
//			if (tusb422_pwr->alert_status == 0)
//				break;
//		}
//	}

    if (tusb422_pd->timer_expired)
    {
        tusb422_pd->timer_expired = false;

        if (tusb422_pwr->call_back)
        {
    //        printk("%s: %pF\n", __func__, tusb422_pwr->call_back);
            tusb422_pwr->call_back(0);
        }
        else
        {
            printk("%s: call back NULL\n", __func__);
        }
    }

//    disable_irq(tusb422_pwr->alert_irq);
    tcpm_connection_task();
    usb_pd_task();
//    enable_irq(tusb422_pwr->alert_irq);

    return;
};

static const struct regmap_config tusb422_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0xff,
	.reg_defaults = tusb422_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(tusb422_reg_defs),
	.cache_type = REGCACHE_NONE,
};

static int tusb422_set_config(struct tusb422_pwr_delivery *tusb422_pd)
{
	struct device *dev = tusb422_pd->dev;
	struct device_node *of_node = tusb422_pd->dev->of_node;
	unsigned int role, flags, rp_val;
	int ret;

	tusb422_pd->configuration = devm_kzalloc(dev,
			sizeof(*tusb422_pd->configuration), GFP_KERNEL);

	if (!tusb422_pd->configuration)
		return -ENOMEM;


	ret = of_property_read_u32(of_node, "ti,role", &role);
	if (ret)
		return ret;

	ret = of_property_read_u32(of_node, "ti,rp_val", &rp_val);
	if (ret)
		return ret;

	ret = of_property_read_u32(of_node, "ti,flags", &flags);
	if (ret)
		printk("%s: Missing ti,flags setting to 0\n", __func__);

	tusb422_pd->configuration->role = (tc_role_t) role;
	tusb422_pd->configuration->flags = (uint16_t) flags;
	tusb422_pd->configuration->rp_val = (tcpc_role_rp_val_t) rp_val;
	tusb422_pd->configuration->slave_addr = tusb422_pd->client->addr;
	tusb422_pd->configuration->intf = tusb422_pd->client->adapter->nr;

	ret = tcpm_init(tusb422_pd->configuration);

	return ret;
};

static int tusb422_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	int ret;

	tusb422_pd = devm_kzalloc(dev, sizeof(*tusb422_pd), GFP_KERNEL);
	if (!tusb422_pd)
		return -ENOMEM;

	tusb422_pd->client = client;
	i2c_set_clientdata(client, tusb422_pd);

	tusb422_pd->dev = &client->dev;

	tusb422_of_get_gpios(tusb422_pd);

	tusb422_pd->regmap = devm_regmap_init_i2c(client, &tusb422_regmap_config);
	if (IS_ERR(tusb422_pd->regmap)) {
		ret = PTR_ERR(tusb422_pd->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	ret = tusb422_check_vendor(tusb422_pd->dev);
	if(ret < 0) {
		dev_err(&client->dev, "%s: tusb422_check_vendor fail!\n", __func__);
		return -EIO;
	}
#if 0
	if (tusb422_pd->alert_irq > 0) {
		ret = devm_request_threaded_irq(&client->dev, tusb422_pd->alert_irq,
			NULL,
			tusb422_event_handler,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			"tusb422_event", tusb422_pd);
		if (ret) {
			dev_err(&client->dev, "unable to request IRQ\n");
			return -EIO;
		}
	}
#else
	if (tusb422_pd->alert_irq > 0) {
        ret = request_irq(tusb422_pd->alert_irq, tusb422_event_handler,
                          IRQF_TRIGGER_FALLING | IRQF_NO_THREAD | IRQF_NO_SUSPEND,
                          "tusb422_event", tusb422_pd);
        if (ret) {
            dev_err(&client->dev, "unable to request IRQ\n");
            return -EIO;
        }
    }

    enable_irq_wake(tusb422_pd->alert_irq);
#endif
	hrtimer_init(&tusb422_pd->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	INIT_WORK(&tusb422_pd->work, tusb422_work);

    tusb422_pd->timer_expired = false;
    tusb422_pd->alert_status = 0;

	tusb422_pd->timer.function = tusb422_timer_tasklet;

	ret = tusb422_of_init(tusb422_pd);
	ret = tusb422_set_config(tusb422_pd);
	if (ret) {
		dev_err(&client->dev, "%s: No TUSB422 found\n", __func__);
		ret = -ENODEV;
		goto no_dev;
	}

	tcpm_alert_event(0);
	schedule_work(&tusb422_pd->work);

#ifdef TUSB422_DEBUG
	ret = sysfs_create_group(&client->dev.kobj, &tusb422_attr_group);
	if (ret < 0)
		dev_err(&client->dev, "Failed to create sysfs: %d\n", ret);
#endif
	return ret;

no_dev:
	cancel_work_sync(&tusb422_pd->work);
	return ret;
};

static int tusb422_remove(struct i2c_client *client)
{
	cancel_work_sync(&tusb422_pd->work);
#ifdef TUSB422_DEBUG
	sysfs_remove_group(&client->dev.kobj, &tusb422_attr_group);
#endif
	return 0;
};

static const struct i2c_device_id tusb422_id[] = {
	{ TUSB422_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tusb422_id);

#ifdef CONFIG_OF
static const struct of_device_id tusb422_pd_ids[] = {
	{ .compatible = "ti,tusb422-usb-pd" },
	{ }
};
MODULE_DEVICE_TABLE(of, tusb422_pd_ids);
#endif

static struct i2c_driver tusb422_i2c_driver = {
	.driver = {
		.name = TUSB422_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tusb422_pd_ids),
	},
	.probe = tusb422_i2c_probe,
	.remove = tusb422_remove,
	.id_table = tusb422_id,
};
module_i2c_driver(tusb422_i2c_driver);

MODULE_LICENSE("GPL v2");
