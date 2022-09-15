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
#include <linux/workqueue.h>
#include <linux/pinctrl/consumer.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/iio/consumer.h>

//#define CAM_TEMP_DEBUG 1

#define DRIVER_NAME "cam_temp"
static int major = 0;

struct device *pvrcam_temp_dev = NULL;
struct delayed_work cam_work;
static int G5_V = 0;//PM8150L GPIO 5 voltage
static int G7_V = 0;//PM8150L GPIO 7 voltage
static int M1_T = 0;//PM8150L AMUX1 temperature
static int M3_T = 0;//PM8150L AMUX3 temperature
static atomic_t cam_temp_status = ATOMIC_INIT(0);
#if 0
const float Rp=100000.0; //100K
const float T2 = (273.15+25.0);;//T2
const float Bx = 4250.0;//B=4250K
const float Ka = 273.15;
float Get_Temp(int voltage)
{
	float Rt;
	float temp;
	int vdd = 1550000;
	Rt = ((double)voltage*30)/(vdd-voltage);
	Rt *= 1000.0f;
	temp = Rt/Rp;
	temp = log(temp);//ln(Rt/Rp)
	temp/=Bx;//ln(Rt/Rp)/B
	temp+=(1/T2);
	temp = 1/(temp);
	temp-=Ka;
	return temp;
}
#endif
DECLARE_WAIT_QUEUE_HEAD(cam_temp_list);
static int cam_temp_check(int G5V, int G7V, int M1T, int M3T)
{
	if((G5V != G5_V) || (M1T != M1_T) || (M3T != M3_T) || (G7V != G7_V))
	{
	#ifdef IPD_DEBUG
		printk("%s update\n",__func__);
	#endif
		G5_V = G5V;
		M1_T = M1T;
		M3_T = M3T;
		G7_V = G7V;
		return 1;
	}else{
		return 0;
	}
}
static loff_t cam_temp_lseek(struct file *file, loff_t offset, int orig)
{
	printk("%s\n",__func__);
	return 0;
}
static void cam_temp_schedule_read(void)
{
	//printk("%s\n",__func__);
	schedule_delayed_work(&cam_work, HZ);
}
static void cam_temp_worker(struct work_struct *work)
{
	struct iio_channel *gpio5_v;
	struct iio_channel *gpio7_v;
	struct iio_channel *amux1_v;
	struct iio_channel *amux3_v;
	int gpio5_voltage = 0;
	int gpio7_voltage = 0;
	int amux1_voltage = 0;
	int amux3_voltage = 0;
	int rc = 0;
	gpio5_v = iio_channel_get(pvrcam_temp_dev, "gpio5t_voltage");
	gpio7_v = iio_channel_get(pvrcam_temp_dev, "gpio7t_voltage");
	amux1_v = iio_channel_get(pvrcam_temp_dev, "amux1t_voltage");
	amux3_v = iio_channel_get(pvrcam_temp_dev, "amux3t_voltage");
	rc = iio_read_channel_processed(gpio5_v, &gpio5_voltage);
	if(!rc)
		printk("%s: gpio5_v iio_read_channel_processed err!!\n",__func__);

	rc = iio_read_channel_processed(gpio7_v, &gpio7_voltage);
	if(!rc)
		printk("%s: gpio7_v iio_read_channel_processed err!!\n",__func__);

	rc = iio_read_channel_processed(amux1_v, &amux1_voltage);
	if(!rc)
		printk("%s: amux1_v iio_read_channel_processed err!!\n",__func__);

	rc = iio_read_channel_processed(amux3_v, &amux3_voltage);
	if(!rc)
		printk("%s: amux3_v iio_read_channel_processed err!!\n",__func__);
#ifdef CAM_TEMP_DEBUG
	printk("%s:  %d, %d, %d, %d\n",__func__, gpio5_voltage, gpio7_voltage, amux1_voltage, amux3_voltage);
#endif
	if(cam_temp_check(gpio5_voltage, gpio7_voltage, amux1_voltage, amux3_voltage)){
		wake_up_interruptible(&cam_temp_list);
		atomic_set(&cam_temp_status, 1);
	}
	cam_temp_schedule_read();
}

static ssize_t cam_temp_value_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct iio_channel *gpio5_v;
	struct iio_channel *gpio7_v;
	struct iio_channel *amux1_v;
	struct iio_channel *amux3_v;
	int gpio5_voltage = 0;
	int gpio7_voltage = 0;
	int amux1_voltage = 0;
	int amux3_voltage = 0;
	int rc = 0;
	gpio5_v = iio_channel_get(pvrcam_temp_dev, "gpio5t_voltage");
	gpio7_v = iio_channel_get(pvrcam_temp_dev, "gpio7t_voltage");
	amux1_v = iio_channel_get(pvrcam_temp_dev, "amux1t_voltage");
	amux3_v = iio_channel_get(pvrcam_temp_dev, "amux3t_voltage");
	rc = iio_read_channel_processed(gpio5_v, &gpio5_voltage);
	if(!rc)
		printk("%s: gpio5_v iio_read_channel_processed err!!\n",__func__);

	rc = iio_read_channel_processed(gpio7_v, &gpio7_voltage);
	if(!rc)
		printk("%s: gpio7_v iio_read_channel_processed err!!\n",__func__);

	rc = iio_read_channel_processed(amux1_v, &amux1_voltage);
	if(!rc)
		printk("%s: amux1_v iio_read_channel_processed err!!\n",__func__);

	rc = iio_read_channel_processed(amux3_v, &amux3_voltage);
	if(!rc)
		printk("%s: amux3_v iio_read_channel_processed err!!\n",__func__);
#ifdef CAM_TEMP_DEBUG
	printk("%s: voltage gpio5 = %d\n",__func__, gpio5_voltage);
	printk("%s: voltage gpio7 = %d\n",__func__, gpio7_voltage);
	printk("%s: voltage AMUX1 = %d\n",__func__, amux1_voltage);
	printk("%s: voltage AMUX3 = %d\n",__func__, amux3_voltage);
#endif
        return sprintf(buf, "%d\n%d\n%d\n%d\n", gpio5_voltage,gpio7_voltage,amux1_voltage,amux3_voltage);
}

static int cam_temp_open (struct inode *pnode, struct file *filp)
{
     nonseekable_open(pnode, filp);
     cam_temp_schedule_read();
     return 0;
}



static ssize_t cam_temp_read (struct file *filp, char __user *buf, size_t count, loff_t *offp)
{
    struct iio_channel *gpio5_v;
    struct iio_channel *gpio7_v;
    struct iio_channel *amux1_v;
    struct iio_channel *amux3_v;

    unsigned long missing;
    ssize_t	 status = 0;
    int gpio5_voltage = 0;
    int gpio7_voltage = 0;
    int amux1_voltage = 0;
    int amux3_voltage = 0;
    int rc = 0;
    int i;
    unsigned char cam_temp_buf[12] = {0};
    //printk("%s\n",__func__);
    gpio5_v = iio_channel_get(pvrcam_temp_dev, "gpio5t_voltage");
    gpio7_v = iio_channel_get(pvrcam_temp_dev, "gpio7t_voltage");

    amux1_v = iio_channel_get(pvrcam_temp_dev, "amux1t_voltage");
    amux3_v = iio_channel_get(pvrcam_temp_dev, "amux3t_voltage");

    rc = iio_read_channel_processed(gpio5_v, &gpio5_voltage);
    if(!rc)
	printk("%s: gpio5_v iio_read_channel_processed err!!\n",__func__);

    rc = iio_read_channel_processed(gpio7_v, &gpio7_voltage);
    if(!rc)
	printk("%s: gpio7_v iio_read_channel_processed err!!\n",__func__);

    rc = iio_read_channel_processed(amux1_v, &amux1_voltage);
    if(!rc)
	printk("%s: amux1_v iio_read_channel_processed err!!\n",__func__);

    rc = iio_read_channel_processed(amux3_v, &amux3_voltage);
    if(!rc)
	printk("%s: amux3_v iio_read_channel_processed err!!\n",__func__);
#ifdef CAM_TEMP_DEBUG
    printk("%s: voltage gpio5 = %d, 0x%x\n",__func__, gpio5_voltage,gpio5_voltage);
    printk("%s: voltage gpio7 = %d\n",__func__, gpio7_voltage);
    printk("%s: voltage AMUX1 = %d, 0x%x\n",__func__, amux1_voltage,amux1_voltage);
    printk("%s: voltage AMUX3 = %d, 0x%x\n",__func__, amux3_voltage,amux3_voltage);
#endif
    for(i = 0; i < 16; i++){
	if(i < 4){
		cam_temp_buf[i] = (gpio5_voltage >> (3 - i) * 8);
		//printk("%s cam_temp_buf[%d]=0x%X\n",__func__,i,cam_temp_buf[i]);
	}
	if((i > 3) && (i < 8)){
		cam_temp_buf[i] = (gpio7_voltage >> (3 - i -4) * 8);
		//printk("%s cam_temp_buf[%d]=0x%X\n",__func__,i,cam_temp_buf[i]);
	}
	if((i > 7) && (i < 12)){
		cam_temp_buf[i] = (amux1_voltage >> (3 - i - 8)*8);
		//printk("%s cam_temp_buf[%d]=0x%X\n",__func__,i,cam_temp_buf[i]);
	}
	if((i > 11)&&(i < 16)){
		cam_temp_buf[i] = (amux3_voltage >> (3 - i - 12)*8);
		//printk("%s cam_temp_buf[%d]=0x%X\n",__func__,i,cam_temp_buf[i]);
	}
    }

    missing = copy_to_user(buf, (void*)cam_temp_buf, sizeof(cam_temp_buf));
    if(missing > 0){
	printk("%s: copy_to_user miss!\n",__func__);
	status = -EFAULT;
    }

    return status;
}

static ssize_t cam_temp_write(struct file *filp, const char __user *buf, size_t count, loff_t *offp)
{
    return 0;
}
static int cam_temp_release (struct inode *pnode, struct file *filp)
{
     return 0;
}
static unsigned int cam_temp_poll(struct file *filp, poll_table * wait)
{
	int mask = 0;
	poll_wait(filp, &cam_temp_list, wait);
	if(atomic_read(&cam_temp_status)){
		mask = POLLIN | POLLRDNORM;
		atomic_set(&cam_temp_status, 0);
	}
	return mask;
}

static DEVICE_ATTR( cam_temp_value, 0664, cam_temp_value_show, NULL);

static struct attribute *cam_temp_value_attributes[] = {
        &dev_attr_cam_temp_value.attr,
        NULL
};

static struct attribute_group cam_temp_value_attribute_group = {
        .attrs = cam_temp_value_attributes
};

static int cam_temp_probe(struct platform_device *pdev)
{
	//struct device_node *np = pdev->dev.of_node;
	pvrcam_temp_dev = &pdev->dev;
	INIT_DELAYED_WORK(&cam_work, cam_temp_worker);
	if (sysfs_create_group(&pdev->dev.kobj, &cam_temp_value_attribute_group) < 0) {
			printk("%s: create cam_temp_value_attribute_group node failure!\n", __func__);
	}
	return 0;
}

static int cam_temp_remove(struct platform_device *pdev)
{
	return 0;
}

static struct file_operations fops = {
      .owner = THIS_MODULE,
      .read = cam_temp_read,
      .write = cam_temp_write,
      .open = cam_temp_open,
      .poll = cam_temp_poll,
      .llseek = cam_temp_lseek,
      .release = cam_temp_release,
};

static struct class *cam_temp_class;
static const struct of_device_id cam_temp_match[] = {
	{
		.compatible = "pico,pvr-cam_temp",
	},
	{},
};
MODULE_DEVICE_TABLE(of, cam_temp_match);

static struct platform_driver cam_temp_driver = {
	.probe		= cam_temp_probe,
	.remove		= cam_temp_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = cam_temp_match,
	},
};
static int __init cam_temp_init(void)
{
    struct device *cam_temp_device;
    int retval;
    int err;

    major = register_chrdev(0, "cam_temp_chrdev", &fops);
    if(major < 0){
            retval = major;
            goto chrdev_err;
    }

    cam_temp_class = class_create(THIS_MODULE,"cam_temp_class");
    if(IS_ERR(cam_temp_class)){
           retval =  PTR_ERR(cam_temp_class);
           goto class_err;
    }

    err = platform_driver_register(&cam_temp_driver);
    if (err)
	  goto device_err;

    cam_temp_device = device_create(cam_temp_class,NULL, MKDEV(major, 0), NULL,"cam_temp");
    if(IS_ERR(cam_temp_device)){
          retval = PTR_ERR(cam_temp_device);
          goto device_err;
     }

    return 0;
device_err:
         class_destroy(cam_temp_class);
class_err:
    unregister_chrdev(major, "cam_temp_chrdev");
chrdev_err:
    return retval;
}
static void __exit cam_temp_exit(void)
{
      unregister_chrdev(major, "cam_temp_chrdev");
      device_destroy(cam_temp_class,MKDEV(major, 0));
      class_destroy(cam_temp_class);
      cancel_delayed_work_sync(&cam_work);
}
late_initcall(cam_temp_init);
//module_init(cam_temp_init);
module_exit(cam_temp_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("hanbo");
MODULE_DESCRIPTION("used for studing linux drivers");
