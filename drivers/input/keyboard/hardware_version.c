// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022, Pico Immersive Pte. Ltd ("Pico"). All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
** ========================================================================
** File:
**     hardware_version.c
**
** Description:
**     Show hardware version
**     cat /sys/bus/platform/drivers/hw_version/soc:hw_version/version
**
** Bug 2774: add hardware version
** ========================================================================
*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <linux/pinctrl/consumer.h>
#include <linux/syscore_ops.h>
#include <linux/hardware_version.h>

#define HW_VERSION_DEV_NAME "hw_version"
#define MAX         5
#define ATOI(X) ( (X) - 48)
#define VERSION_ERROR(a)              (((int) (a)) < 0)

static int hw_gpio[MAX] = {0};
static int hw_version = 0;

struct hwinfo {
	unsigned int code;
	int gpio;
	int active_low;
	const char *desc;
	unsigned int type;
	int wakeup;
	int wakeup_event_action;
	int debounce_interval;
	bool can_disable;
	int value;
	unsigned int irq;
};

struct hwinfo_platform_data {
	const struct hwinfo *id;
	int n_ids;
	unsigned int poll_interval;
	unsigned int rep:1;
	int (*enable)(struct device *dev);
	void (*disable)(struct device *dev);
	const char *name;
};

struct hw_version_data {
	const struct hwinfo *id;
};

struct hw_version_drvdata {
	const struct hwinfo_platform_data *pdata;
	struct pinctrl *key_pinctrl;
	struct hw_version_data data[0];
};
static int hw_version_atoi(char *str)
{
    int ver = (str[0] - 48)*10 + (str[1] - 48);
    return ver;
}

static ssize_t hw_version_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int hw_ver = 0;
	int i;
        if (VERSION_ERROR(hw_version)){
            for(i=0; i< MAX; i++)
            {
                hw_ver |= (gpio_get_value(hw_gpio[i]) << i);
            }
        }else{
                hw_ver = hw_version;
        }
	return sprintf(buf, "B%d\n", hw_ver);
}
int hardware_version_num = 0;
int hw_version_get(void)
{
	int i;
	if (VERSION_ERROR(hw_version)){
		for(i = 0; i < MAX; i++)
		{
			hardware_version_num |= (gpio_get_value(hw_gpio[i]) << i);
		}
        }else{
                hardware_version_num = hw_version;
        }
	return hardware_version_num;
}
EXPORT_SYMBOL(hardware_version_num);
EXPORT_SYMBOL_GPL(hw_version_get);

static ssize_t hw_version_store(struct device *dev,
				     struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}


static DEVICE_ATTR(version, 0664, hw_version_show,hw_version_store);

static struct attribute *hw_version_attrs[] = {
	&dev_attr_version.attr,
	NULL,
};

static struct attribute_group hw_version_attr_group = {
	.attrs = hw_version_attrs,
};

static int hw_version_pinctrl_configure(struct hw_version_drvdata *ddata,
							bool active)
{
	struct pinctrl_state *set_state;
	int retval;
	if (active) {
		set_state =
			pinctrl_lookup_state(ddata->key_pinctrl,
						"tlmm_hardware_version_active");
		if (IS_ERR(set_state)) {
			printk("%s:annot get ts pinctrl active state\n",__func__);
			return PTR_ERR(set_state);
		}

	} else {
		set_state =
			pinctrl_lookup_state(ddata->key_pinctrl,
						"tlmm_hardware_version_suspend");
		if (IS_ERR(set_state)) {
			printk("[%s]: cannot get gpiokey pinctrl sleep state\n",__func__);
			return PTR_ERR(set_state);
		}
	}
	retval = pinctrl_select_state(ddata->key_pinctrl, set_state);
	if (retval) {
		printk("[%s]: cannot set ts pinctrl active state\n",__func__);
		return retval;
	}

	return 0;
}


static struct hwinfo_platform_data *
hw_version_get_devtree_pdata(struct device *dev)
{
	struct device_node *node, *pp;
	struct hwinfo_platform_data *pdata;
	struct hwinfo *id;
	int count;
	int i = 0,error = 0;
	node = dev->of_node;
	if (!node)
		return ERR_PTR(-ENODEV);

	count = of_get_child_count(node);
	if (count == 0)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(dev,
			     sizeof(*pdata) + count * sizeof(*id),
			     GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->id = (struct hwinfo *)(pdata + 1);
	pdata->n_ids = count;
	for_each_child_of_node(node, pp)
	{
		int gpio;
		enum of_gpio_flags flags;

		if (!of_find_property(pp, "gpios", NULL))
		{
			pdata->n_ids--;
			dev_warn(dev, "Found button without gpios\n");
			continue;
		}

		gpio = of_get_gpio_flags(pp, 0, &flags);
		if (gpio < 0)
		{
			error = gpio;
			if (error != -EPROBE_DEFER)
				dev_err(dev,
					"Failed to get gpio flags, error: %d\n",
					error);
			return ERR_PTR(error);
		}
		if(i < MAX){
			hw_gpio[i] = gpio;
		}
		i++;
	}

	if (pdata->n_ids == 0)
		return ERR_PTR(-EINVAL);

	return pdata;
}



static int hw_version_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hwinfo_platform_data *pdata = dev_get_platdata(dev);
	struct hw_version_drvdata *ddata;
	int error;
	size_t size;
	if (!pdata) {
		pdata = hw_version_get_devtree_pdata(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	size = sizeof(struct hw_version_drvdata) +
			pdata->n_ids * sizeof(struct hw_version_data);
	ddata = devm_kzalloc(dev, size, GFP_KERNEL);

	if (!ddata) {
		dev_err(dev, "failed to allocate state\n");
		return -ENOMEM;
	}
	ddata->pdata = pdata;
	platform_set_drvdata(pdev, ddata);
	ddata->key_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(ddata->key_pinctrl)) {
		if (PTR_ERR(ddata->key_pinctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		pr_debug("Target does not use pinctrl\n");
		ddata->key_pinctrl = NULL;
	}

	if (ddata->key_pinctrl) {
		error = hw_version_pinctrl_configure(ddata, true);
		if (error) {
			dev_err(dev, "cannot set ts pinctrl active state\n");
			return error;
		}
	}

	error = sysfs_create_group(&pdev->dev.kobj, &hw_version_attr_group);
	if (error)
	{
		printk("[%s]: Unable to export keys/switches, error: %d\n", __func__, error);
	}
	hw_version_get();
	return 0;


}
static int hw_version_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &hw_version_attr_group);
	return 0;
}


static const struct of_device_id hw_version_of_match[] = {
	{ .compatible = "hw_version", },
	{ },
};
MODULE_DEVICE_TABLE(of, hw_version_of_match);


static struct platform_driver hw_version_device_driver = {
	.probe		= hw_version_probe,
	.remove		= hw_version_remove,
	.driver		= {
		.name	= "hw_version",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(hw_version_of_match),
	}
};

static int __init hw_version_init(void)
{
       char *hw_str = NULL;
       hw_str = strstr(boot_command_line, "hwinfo=");
	if (!hw_str){
	    hw_version = -1;
	}else{
            hw_str += 7;
            hw_version = hw_version_atoi(hw_str);
	}
	return platform_driver_register(&hw_version_device_driver);
}

static void __exit hw_version_exit(void)
{
	platform_driver_unregister(&hw_version_device_driver);
}

arch_initcall(hw_version_init);
module_exit(hw_version_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Clark Tian <clark.tian>");
MODULE_DESCRIPTION("hw_version driver for hardware version");
MODULE_ALIAS("platform:hw_version");
