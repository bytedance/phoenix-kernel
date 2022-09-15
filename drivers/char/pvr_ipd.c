// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022, Pico Immersive Pte. Ltd ("Pico"). All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/workqueue.h>
#include <linux/pinctrl/consumer.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/iio/consumer.h>


#define DRIVER_NAME "ipd"
#define IPD_PW_ENABLE		1
#define IPD_PW_DISABLE		0
static int major = 0;
static int ipd_voltage = 0;
static atomic_t ipd_status = ATOMIC_INIT(0);
struct delayed_work dwork;
DECLARE_WAIT_QUEUE_HEAD(ipd_list);

struct pvr_ipddev {
	struct platform_device *pdev;
	int ipd_pw_ctrl;
};
static struct pvr_ipddev *pvr_data;

static void ipd_schedule_read(void)
{
	schedule_delayed_work(&dwork, HZ/10);
}
static void ipd_worker(struct work_struct *work)
{
	struct iio_channel *gpio7C_chan;
    int voltage = 0;
    int rc = 0;
    gpio7C_chan = iio_channel_get(&pvr_data->pdev->dev, "gpio1C_voltage");
    rc = iio_read_channel_processed(gpio7C_chan, &voltage);
	voltage /= 10000;
	if(ipd_voltage != voltage){
		dev_dbg(&pvr_data->pdev->dev,"ipd_voltage=%d, voltage=%d\n",ipd_voltage,voltage);
		ipd_voltage = voltage;
		wake_up_interruptible(&ipd_list);
		atomic_set(&ipd_status, 1);
	}
	ipd_schedule_read();
}
static ssize_t ipd_value_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct iio_channel *gpio7C_chan;
    int voltage = 0;
    int rc = 0;
    gpio7C_chan = iio_channel_get(dev, "gpio1C_voltage");
    rc = iio_read_channel_processed(gpio7C_chan, &voltage);
    dev_dbg(&pvr_data->pdev->dev,"rc=%d, voltage=%d\n",rc,voltage);
    return sprintf(buf, "%ld\n", voltage);
}
static ssize_t ipd_value_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	if(buf[0] == 48){
	    dev_dbg(&pvr_data->pdev->dev,"ipd set value=%c,ipd disable\n",buf[0]);
	    gpio_set_value(pvr_data->ipd_pw_ctrl, IPD_PW_DISABLE);
	}else if(buf[0] == 49){
	    dev_dbg(&pvr_data->pdev->dev,"ipd set value=%c,ipd enable\n",buf[0]);
	    gpio_set_value(pvr_data->ipd_pw_ctrl, IPD_PW_ENABLE);
	}else{
	    dev_dbg(&pvr_data->pdev->dev,"ipd set invaild value=%c\n",buf[0]);
	}
	return count;
}
static int ipd_open (struct inode *pnode, struct file *filp)
{
     nonseekable_open(pnode, filp);
     ipd_schedule_read();
     return 0;
}
static ssize_t ipd_read (struct file *filp, char __user *buf, size_t count, loff_t *offp)
{
    struct iio_channel *gpio7C_chan;
    unsigned long missing;
    ssize_t	 status = 0;
    int voltage = 0;
    int rc = 0;
    int i = 0;
    unsigned char ipd_buf[4] = {0};
    gpio7C_chan = iio_channel_get(&pvr_data->pdev->dev, "gpio1C_voltage");
    rc = iio_read_channel_processed(gpio7C_chan, &voltage);
    if(!rc)
	dev_dbg(&pvr_data->pdev->dev,"iio_read_channel_processed err!!\n");
    //voltage /= 10000;
    dev_dbg(&pvr_data->pdev->dev,"rc=%d, voltage=%d\n",rc,voltage);
    for(i = 0; i < 4; i++){
	ipd_buf[i] = (voltage >> (3-i)*8);
	dev_dbg(&pvr_data->pdev->dev,"ipd_buf[%d]=%d\n",i,ipd_buf[i]);
    }
    missing = copy_to_user(buf, ipd_buf, sizeof(ipd_buf));
    if(missing > 0){
	dev_dbg(&pvr_data->pdev->dev,"copy_to_user miss!\n");
	status = -EFAULT;
    }

    return status;
}

static ssize_t ipd_write(struct file *filp, const char __user *buf, size_t count, loff_t *offp)
{
    return 0;
}
static int ipd_release (struct inode *pnode, struct file *filp)
{
     return 0;
}
static unsigned int ipd_poll(struct file *filp, poll_table * wait)
{
	int mask = 0;
	poll_wait(filp, &ipd_list, wait);
	if(atomic_read(&ipd_status)){
		mask = POLLIN | POLLRDNORM;
		atomic_set(&ipd_status, 0);
	}
	return mask;
}
static int ipd_pm_suspend(struct device *dev)
{
	if (pvr_data){
		gpio_set_value(pvr_data->ipd_pw_ctrl, IPD_PW_DISABLE);
		dev_dbg(&pvr_data->pdev->dev, "ipd_pm_suspend\n");
	}
	return 0;
}

static int ipd_pm_resume(struct device *dev)
{
	if (pvr_data){
		gpio_set_value(pvr_data->ipd_pw_ctrl, IPD_PW_ENABLE);
		dev_dbg(&pvr_data->pdev->dev, "ipd_pm_resume\n");
	}
	return 0;
}
static DEVICE_ATTR( ipd_value, 0664, ipd_value_show, ipd_value_store);

static struct attribute *ipd_value_attributes[] = {
        &dev_attr_ipd_value.attr,
        NULL
};

static struct attribute_group ipd_value_attribute_group = {
        .attrs = ipd_value_attributes
};
static const struct of_device_id ipd_match[] = {
	{
		.compatible = "pico,pvr-ipd",
	},
	{},
};
MODULE_DEVICE_TABLE(of, ipd_match);
static int ipd_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	if (pdev->dev.of_node && !of_match_device(ipd_match, &pdev->dev)) {
		dev_err(&pdev->dev, "buggy DT: spidev listed directly in DT\n");
		WARN_ON(pdev->dev.of_node &&
			!of_match_device(ipd_match, &pdev->dev));
	}
	pvr_data = devm_kzalloc(&pdev->dev, sizeof(struct pvr_ipddev),
						GFP_KERNEL);
	if (!pvr_data)
		return -ENOMEM;

	pvr_data->pdev = pdev;
	if((pvr_data->ipd_pw_ctrl= of_get_named_gpio(np, "ipd-control", 0)) < 0){
			dev_dbg(&pdev->dev,"of_get_named_gpio ipd-control error!\n");
			return pvr_data->ipd_pw_ctrl;
	}
	if (gpio_is_valid(pvr_data->ipd_pw_ctrl)) {
		if (gpio_request(pvr_data->ipd_pw_ctrl, "ipd-control")){
			dev_dbg(&pdev->dev, "ipd-control not available\n");
		}else{
			 gpio_direction_output(pvr_data->ipd_pw_ctrl, 0);
			gpio_set_value(pvr_data->ipd_pw_ctrl, IPD_PW_ENABLE);
		}
	}
	INIT_DELAYED_WORK(&dwork, ipd_worker);
	if (sysfs_create_group(&pdev->dev.kobj, &ipd_value_attribute_group) < 0) {
			dev_dbg(&pdev->dev, "create ipd_value_attribute_group node failure!\n");
	}
	dev_dbg(&pdev->dev,"ipd init succeed\n");
	return 0;
}

static int ipd_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct dev_pm_ops ipd_dev_pm_ops = {
	.suspend = ipd_pm_suspend,
	.resume = ipd_pm_resume,
};

static struct file_operations fops = {
      .owner = THIS_MODULE,
      .read = ipd_read,
      .write = ipd_write,
      .open = ipd_open,
      .poll = ipd_poll,
      .release = ipd_release,
};

static struct class *ipd_class;
static struct platform_driver ipd_driver = {
	.probe		= ipd_probe,
	.remove		= ipd_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = ipd_match,
		.pm = &ipd_dev_pm_ops,
	},
};
static int __init ipd_init(void)
{
    struct device *ipd_device;
    int retval;
    int err;

    major = register_chrdev(0, "ipd_chrdev", &fops);
    if(major < 0){
            retval = major;
            goto chrdev_err;
    }

    ipd_class = class_create(THIS_MODULE,"ipd_class");
    if(IS_ERR(ipd_class)){
           retval =  PTR_ERR(ipd_class);
           goto class_err;
    }

    err = platform_driver_register(&ipd_driver);
    if (err)
	  goto device_err;

    ipd_device = device_create(ipd_class,NULL, MKDEV(major, 0), NULL,"ipd");
    if(IS_ERR(ipd_device)){
          retval = PTR_ERR(ipd_device);
          goto device_err;
     }


    return 0;
device_err:
         class_destroy(ipd_class);
class_err:
    unregister_chrdev(major, "ipd_chrdev");
chrdev_err:
    return retval;
}
static void __exit ipd_exit(void)
{
      unregister_chrdev(major, "ipd_chrdev");
      device_destroy(ipd_class,MKDEV(major, 0));
      class_destroy(ipd_class);
	  gpio_free(pvr_data->ipd_pw_ctrl);
      cancel_delayed_work_sync(&dwork);
}
late_initcall(ipd_init);
module_exit(ipd_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("clark");
MODULE_DESCRIPTION("used for studing linux drivers");
