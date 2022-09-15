/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>

#include "flashlight-core.h"
//#include "flashlight-dt.h"

#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-label"

/* device tree should be defined in flashlight-dt.h */
#ifndef AW36429_DTNAME
#define AW36429_DTNAME "awinic,aw36429"
#endif
#ifndef AW36429_DTNAME_I2C
#define AW36429_DTNAME_I2C   "awinic,aw36429_led"
#endif

#define AW36429_NAME "aw36429"
#define AW36429_IDTBL_NAME "aw36429-awinic"
#define AW36406_IDTBL_NAME "aw36406-awinic"
#define AW36429_DEV_DRV_NAME "AWINIC-aw36429"
#define AW36429_DRIVER_VERSION	"V1.1.0"

#define REG_CHIP_ID              0x00
#define AW36429_ID               0x17

#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 5

/* define registers */
#define AW36429_REG_CHIPID           (0x00)
#define AW36429_REG_ENABLE           (0x01)
#define AW36429_MASK_ENABLE_LED1     (0x01)
#define AW36429_MASK_ENABLE_LED2     (0x02)
#define AW36429_MASK_ENABLE_LED3     (0x04)
#define AW36429_MASK_ENABLE_LED4     (0x08)
#define AW36429_DISABLE              (0x00)
#define AW36429_ENABLE_LED1          (0x01)
#define AW36429_ENABLE_LED1_TORCH    (0x21)
#define AW36429_ENABLE_LED1_FLASH    (0x31)
#define AW36429_ENABLE_LED2          (0x02)
#define AW36429_ENABLE_LED2_TORCH    (0x22)
#define AW36429_ENABLE_LED2_FLASH    (0x32)
#define AW36429_ENABLE_LED3          (0x04)
#define AW36429_ENABLE_LED3_TORCH    (0x24)
#define AW36429_ENABLE_LED3_FLASH    (0x34)
#define AW36429_ENABLE_LED4          (0x08)
#define AW36429_ENABLE_LED4_TORCH    (0x28)
#define AW36429_ENABLE_LED4_FLASH    (0x38)

#define AW36429_REG_FLASH_LEVEL_LED1 (0x03)
#define AW36429_REG_TORCH_LEVEL_LED1 (0x04)
#define AW36429_REG_FLASH_LEVEL_LED2 (0x05)
#define AW36429_REG_TORCH_LEVEL_LED2 (0x06)
#define AW36429_REG_FLASH_LEVEL_LED3 (0x07)
#define AW36429_REG_TORCH_LEVEL_LED3 (0x08)
#define AW36429_REG_FLASH_LEVEL_LED4 (0x09)
#define AW36429_REG_TORCH_LEVEL_LED4 (0x0A)

#define AW36429_REG_TIMING_CONF      (0x0B)
#define AW36429_TORCH_RAMP_TIME      (0x10)
#define AW36429_FLASH_TIMEOUT        (0x09)

#define AW36429_CHIP_STANDBY         (0x00)
#define AW36429_CHIP_SOFT_RST        (0x55)

/* define channel, level */
#define AW36429_CHANNEL_NUM          4
#define AW36429_CHANNEL_CH1          0
#define AW36429_CHANNEL_CH2          1
#define AW36429_CHANNEL_CH3          2
#define AW36429_CHANNEL_CH4          3
/* define level */
#define AW36429_LEVEL_NUM 26
#define AW36429_LEVEL_TORCH 7

#define AW_I2C_RETRIES			5
#define AW_I2C_RETRY_DELAY		2

/* define mutex and work queue */
static DEFINE_MUTEX(aw36429_mutex);
static DEFINE_MUTEX(fl_mutex);
static struct work_struct aw36429_work_ch1;
static struct work_struct aw36429_work_ch2;
static struct work_struct aw36429_blinkoff_work_ch1;
static struct work_struct aw36429_blinkoff_work_ch2;

static LIST_HEAD(flashlight_list);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t aw36429_get_reg(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t aw36429_set_reg(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static DEVICE_ATTR(aw36429reg, 0660, aw36429_get_reg,  aw36429_set_reg);

struct i2c_client *aw36429_flashlight_client;

struct aw36429_chip_data *aw36429_chip_priv;

/* define usage count */
static int use_count;

/* define i2c */
static struct i2c_client *aw36429_i2c_client=NULL;

/* platform data */
struct aw36429_platform_data {
	u8 torch_pin_enable;         /* 1: TX1/TORCH pin isa hardware TORCH enable */
	u8 pam_sync_pin_enable;      /* 1: TX2 Mode The ENVM/TX2 is a PAM Sync. on input */
	u8 thermal_comp_mode_enable; /* 1: LEDI/NTC pin in Thermal Comparator Mode */
	u8 strobe_pin_disable;       /* 1: STROBE Input disabled */
	u8 vout_mode_enable;         /* 1: Voltage Out Mode enable */
};

const struct flashlight_device_id flashlight_id[]=
	{
			{0,0,0,"flashlights-aw36429",0,0},
	};

const int flashlight_device_num = 
	sizeof(flashlight_id)/sizeof(struct flashlight_device_id);


/******************************************************************************
 * aw36429 operations
 *****************************************************************************/
static const unsigned char aw36429_torch_level[AW36429_LEVEL_NUM] = {
    0x06, 0x0F, 0x17, 0x1F, 0x27, 0x2F, 0x37, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static const unsigned char aw36429_flash_level[AW36429_LEVEL_NUM] = {
	0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x10, 0x14, 0x19,
	0x1D, 0x21, 0x25, 0x2A, 0x2E, 0x32, 0x37, 0x3B, 0x3F, 0x43,
	0x48, 0x4C, 0x50, 0x54, 0x59, 0x5D};

static volatile unsigned char aw36429_reg_enable=0;
static volatile int aw36429_level_ch1 = -1;
static volatile int aw36429_level_ch2 = -1;
static volatile int aw36429_level_ch3 = -1;
static volatile int aw36429_level_ch4 = -1;

struct aw36406_i2c_hwdata {
		int gpio_en;
		int vdd_en;
};

static const struct aw36406_i2c_hwdata st_aw36406_i2c_hwdata = {
		.gpio_en = 1,
		.vdd_en = 50,
};

static int aw36429_is_torch(int level)
{
	if (level >= AW36429_LEVEL_TORCH)
		return -1;

	return 0;
}

static int aw36429_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= AW36429_LEVEL_NUM)
		level = AW36429_LEVEL_NUM - 1;

	return level;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// i2c write and read
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int aw36429_i2c_write(struct i2c_client *client, unsigned char reg, unsigned char val)
{
	int ret;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (ret < 0) {
			pr_info("%s: i2c_write addr=0x%02X, data=0x%02X, cnt=%d, error=%d\n",
				   __func__, reg, val, cnt, ret);
		} else {
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

/* i2c wrapper function */
static int aw36429_i2c_read(struct i2c_client *client, unsigned char reg, unsigned char *val)
{
	int ret;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret < 0) {
			pr_info("%s: i2c_read addr=0x%02X, cnt=%d, error=%d\n",
				   __func__, reg, cnt, ret);
		} else {
			*val = ret;
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static void aw36429_soft_reset(void)
{
	aw36429_i2c_write(aw36429_i2c_client, AW36429_REG_CHIPID,
						AW36429_CHIP_SOFT_RST);
	msleep(5);
}

static void aw36429_set_default(void)
{
	struct reg_val aw36429_defalt[] = {
		{AW36429_REG_FLASH_LEVEL_LED1,		0x18},
		{AW36429_REG_TORCH_LEVEL_LED1,		0x43},
		{AW36429_REG_FLASH_LEVEL_LED2,		0x18},
		{AW36429_REG_TORCH_LEVEL_LED2,		0x43},
		{AW36429_REG_FLASH_LEVEL_LED3,		0x18},
		{AW36429_REG_TORCH_LEVEL_LED3,		0x43},
		{AW36429_REG_FLASH_LEVEL_LED4,		0x18},
		{AW36429_REG_TORCH_LEVEL_LED4,		0x43},
	};

	int array_size = sizeof(aw36429_defalt)/sizeof(aw36429_defalt[0]);
	int i=0;

	for(;i < array_size;i++){
		aw36429_i2c_write(aw36429_i2c_client,aw36429_defalt[i].reg,aw36429_defalt[i].reg_val);
	}
}

/* flashlight enable function */
static int aw36429_enable_ch1(void)
{
	unsigned char reg, val;

	reg = AW36429_REG_ENABLE;
	if (!aw36429_is_torch(aw36429_level_ch1)) {
		/* torch mode */
		aw36429_reg_enable |= AW36429_ENABLE_LED1_TORCH;
	} else {
		/* flash mode */
		aw36429_reg_enable |= AW36429_ENABLE_LED1_FLASH;
	}
	val = aw36429_reg_enable;

	return aw36429_i2c_write(aw36429_i2c_client, reg, val);
}

static int aw36429_enable_ch2(void)
{
	unsigned char reg, val;

	reg = AW36429_REG_ENABLE;
	if (!aw36429_is_torch(aw36429_level_ch2)) {
		/* torch mode */
		aw36429_reg_enable |= AW36429_ENABLE_LED2_TORCH;
	} else {
		/* flash mode */
		aw36429_reg_enable |= AW36429_ENABLE_LED2_FLASH;
	}
	val = aw36429_reg_enable;

	return aw36429_i2c_write(aw36429_i2c_client, reg, val);
}

static int aw36429_enable_ch3(void)
{
	unsigned char reg, val;

	reg = AW36429_REG_ENABLE;
	if (!aw36429_is_torch(aw36429_level_ch3)) {
		/* torch mode */
		aw36429_reg_enable |= AW36429_ENABLE_LED3_TORCH;
	} else {
		/* flash mode */
		aw36429_reg_enable |= AW36429_ENABLE_LED3_FLASH;
	}
	val = aw36429_reg_enable;

	return aw36429_i2c_write(aw36429_i2c_client, reg, val);
}

static int aw36429_enable_ch4(void)
{
	unsigned char reg, val;

	reg = AW36429_REG_ENABLE;
	if (!aw36429_is_torch(aw36429_level_ch4)) {
		/* torch mode */
		aw36429_reg_enable |= AW36429_ENABLE_LED4_TORCH;
	} else {
		/* flash mode */
		aw36429_reg_enable |= AW36429_ENABLE_LED4_FLASH;
	}
	val = aw36429_reg_enable;

	return aw36429_i2c_write(aw36429_i2c_client, reg, val);
}

int aw36429_sub_enable_ch3(int en)
{
	unsigned char reg, val;
	reg = AW36429_REG_ENABLE;
	if(en==1){
		aw36429_reg_enable |= (AW36429_ENABLE_LED3_FLASH | AW36429_ENABLE_LED4_FLASH);
	}else{
		aw36429_reg_enable  = AW36429_DISABLE ;
	}
	val = aw36429_reg_enable;
	return aw36429_i2c_write(aw36429_i2c_client, reg, val);
}

static int aw36429_enable(int channel)
{
	if (channel == AW36429_CHANNEL_CH1)
		aw36429_enable_ch1();
	else if (channel == AW36429_CHANNEL_CH2)
		aw36429_enable_ch2();
	else if (channel == AW36429_CHANNEL_CH3)
		aw36429_enable_ch3();
	else if (channel == AW36429_CHANNEL_CH4)
		aw36429_enable_ch4();
	else {
		printk("Error channel\n");
		return -1;
	}

	return 0;
}

/* flashlight disable function */
static int aw36429_disable_ch1(void)
{
	unsigned char reg, val;
#if 0
	reg = AW36429_REG_ENABLE;
	if (aw36429_reg_enable & AW36429_MASK_ENABLE_LED2) {
		/* if LED 2 is enable, disable LED 1 */
		aw36429_reg_enable &= (~AW36429_ENABLE_LED1);
	} else {
		/* if LED 2 is enable, disable LED 1 and clear mode */
		aw36429_reg_enable &= (~AW36429_ENABLE_LED1_FLASH);
	}
	val = aw36429_reg_enable;

	return aw36429_i2c_write(aw36429_i2c_client, reg, val);
#endif

	reg = AW36429_REG_ENABLE;

	aw36429_reg_enable &= (~AW36429_ENABLE_LED1_FLASH);

	val = aw36429_reg_enable;

	return aw36429_i2c_write(aw36429_i2c_client, reg, val);
}

static int aw36429_disable_ch2(void)
{
	unsigned char reg, val;
#if 0
	reg = AW36429_REG_ENABLE;
	if (aw36429_reg_enable & AW36429_MASK_ENABLE_LED1) {
		/* if LED 1 is enable, disable LED 2 */
		aw36429_reg_enable &= (~AW36429_ENABLE_LED2);
	} else {
		/* if LED 1 is enable, disable LED 2 and clear mode */
		aw36429_reg_enable &= (~AW36429_ENABLE_LED2_FLASH);
	}
	val = aw36429_reg_enable;

	return aw36429_i2c_write(aw36429_i2c_client, reg, val);
#endif

	reg = AW36429_REG_ENABLE;

	aw36429_reg_enable &= (~AW36429_ENABLE_LED2_FLASH);
	
	val = aw36429_reg_enable;

	return aw36429_i2c_write(aw36429_i2c_client, reg, val);
	
}

static int aw36429_disable_ch3(void)
{
	unsigned char reg, val;
#if 0
	reg = AW36429_REG_ENABLE;
	if (aw36429_reg_enable & AW36429_MASK_ENABLE_LED3) {
		/* if LED 1 is enable, disable LED 2 */
		aw36429_reg_enable &= (~AW36429_ENABLE_LED2);
	}
	/*
	else {
		// if LED 1 is enable, disable LED 2 and clear mode
		aw36429_reg_enable &= (~AW36429_ENABLE_LED2_FLASH);
	}
	*/
	val = aw36429_reg_enable;

	return aw36429_i2c_write(aw36429_i2c_client, reg, val);
#endif
	reg = AW36429_REG_ENABLE;

	aw36429_reg_enable &= (~AW36429_ENABLE_LED3_FLASH);

	val = aw36429_reg_enable;
	
	return aw36429_i2c_write(aw36429_i2c_client, reg, val);

}

static int aw36429_disable_ch4(void)
{
	unsigned char reg, val;
	
	reg = AW36429_REG_ENABLE;

	aw36429_reg_enable &= (~AW36429_ENABLE_LED4_FLASH);

	val = aw36429_reg_enable;
	
	return aw36429_i2c_write(aw36429_i2c_client, reg, val);

}

static int aw36429_disable(int channel)
{
	if (channel == AW36429_CHANNEL_CH1)
		aw36429_disable_ch1();
	else if (channel == AW36429_CHANNEL_CH2)
		aw36429_disable_ch2();
	else if (channel == AW36429_CHANNEL_CH3)
		aw36429_disable_ch3();
	else if (channel == AW36429_CHANNEL_CH4)
		aw36429_disable_ch4();	
	else {
		printk("Error channel\n");
		return -1;
	}

	return 0;
}

static int aw36429_sub_disable(int channel)
{
	if (channel == AW36429_CHANNEL_CH3){
			aw36429_disable_ch3();
	}else {
		printk("Error channel\n");
		return -1;
	}
	return 0;
}

/* set flashlight level */
static int aw36429_set_level_ch1(int level)
{
	int ret;
	unsigned char reg, val;

	level = aw36429_verify_level(level);

	/* set torch brightness level */
	reg = AW36429_REG_TORCH_LEVEL_LED1;
	val = aw36429_torch_level[level];
	ret = aw36429_i2c_write(aw36429_i2c_client, reg, val);

	aw36429_level_ch1 = level;

	/* set flash brightness level */
	reg = AW36429_REG_FLASH_LEVEL_LED1;
	val = aw36429_flash_level[level];
	ret = aw36429_i2c_write(aw36429_i2c_client, reg, val);

	return ret;
}

int aw36429_set_level_ch2(int level)
{
	int ret;
	unsigned char reg, val;

	level = aw36429_verify_level(level);

	/* set torch brightness level */
	reg = AW36429_REG_TORCH_LEVEL_LED2;
	val = aw36429_torch_level[level];
	ret = aw36429_i2c_write(aw36429_i2c_client, reg, val);

	aw36429_level_ch2 = level;

	/* set flash brightness level */
	reg = AW36429_REG_FLASH_LEVEL_LED2;
	val = aw36429_flash_level[level];
	ret = aw36429_i2c_write(aw36429_i2c_client, reg, val);

	return ret;
}

static int aw36429_set_level(int channel, int level)
{
	if (channel == AW36429_CHANNEL_CH1)
		aw36429_set_level_ch1(level);
	else if (channel == AW36429_CHANNEL_CH2)
		aw36429_set_level_ch2(level);
	else {
		printk("Error channel\n");
		return -1;
	}

	return 0;
}

/* flashlight init */
int aw36429_init(void)
{
	int ret;
	unsigned char reg, val;

    msleep(2);

	/* clear enable register */
	reg = AW36429_REG_ENABLE;
	val = AW36429_DISABLE;
	ret = aw36429_i2c_write(aw36429_i2c_client, reg, val);

	aw36429_reg_enable = val;

	/* set torch current ramp time and flash timeout */
	reg = AW36429_REG_TIMING_CONF;
	val = AW36429_TORCH_RAMP_TIME | AW36429_FLASH_TIMEOUT;
	ret = aw36429_i2c_write(aw36429_i2c_client, reg, val);

	return ret;
}
int aw36429_sub_init(void)
{
	int ret;
	unsigned char reg, val;

    msleep(2);

	/* clear enable register */
	reg = AW36429_REG_ENABLE;
	val = AW36429_DISABLE;
	ret = aw36429_i2c_write(aw36429_i2c_client, reg, val);

	aw36429_reg_enable = val;

	/* set torch current ramp time and flash timeout */
	reg = AW36429_REG_TIMING_CONF;
	val = AW36429_TORCH_RAMP_TIME | AW36429_FLASH_TIMEOUT;
	ret = aw36429_i2c_write(aw36429_i2c_client, reg, val);

	return ret;
}

/* flashlight uninit */
int aw36429_uninit(void)
{
	aw36429_disable(AW36429_CHANNEL_CH1);
	aw36429_disable(AW36429_CHANNEL_CH2);
	aw36429_disable(AW36429_CHANNEL_CH3);
	aw36429_disable(AW36429_CHANNEL_CH4);	

	return 0;
}
int aw36429_sub_uninit(void)
{
	aw36429_sub_disable(AW36429_CHANNEL_CH3);

	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer aw36429_timer_ch1;
static struct hrtimer aw36429_timer_ch2;
static struct hrtimer aw36429_blinkoff_timer_ch1;
static struct hrtimer aw36429_blinkoff_timer_ch2;
static unsigned int aw36429_timeout_ms[AW36429_CHANNEL_NUM];
static unsigned long aw36429_blinkoff_ms[AW36429_CHANNEL_NUM];

static void aw36429_work_disable_ch1(struct work_struct *data)
{
	printk("%s ht work queue callback\n",__func__);
	aw36429_disable_ch1();

	
}

static void aw36429_work_disable_ch2(struct work_struct *data)
{
	printk("%s lt work queue callback\n",__func__);
	aw36429_disable_ch2();
}

static enum hrtimer_restart aw36429_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&aw36429_work_ch1);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart aw36429_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&aw36429_work_ch2);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart aw36429_blinkoff_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&aw36429_blinkoff_work_ch1);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart aw36429_blinkoff_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&aw36429_blinkoff_work_ch2);
	return HRTIMER_NORESTART;
}

int aw36429_blinkoff_timer_start(int channel, ktime_t ktime)
{
	printk("%s channel = %d\n",__func__,channel);

	if (channel == AW36429_CHANNEL_CH1)
		hrtimer_start(&aw36429_blinkoff_timer_ch1, ktime, HRTIMER_MODE_REL);
	else if (channel == AW36429_CHANNEL_CH2)
		hrtimer_start(&aw36429_blinkoff_timer_ch2, ktime, HRTIMER_MODE_REL);
	else {
		printk("Error channel\n");
		return -1;
	}

	return 0;
}

int aw36429_timer_start(int channel, ktime_t ktime)
{
	printk("%s channel = %d\n",__func__,channel);

	if (channel == AW36429_CHANNEL_CH1)
		hrtimer_start(&aw36429_timer_ch1, ktime, HRTIMER_MODE_REL);
	else if (channel == AW36429_CHANNEL_CH2)
		hrtimer_start(&aw36429_timer_ch2, ktime, HRTIMER_MODE_REL);
	else {
		printk("Error channel\n");
		return -1;
	}

	return 0;
}

int aw36429_timer_cancel(int channel)
{
	if (channel == AW36429_CHANNEL_CH1)
		hrtimer_cancel(&aw36429_timer_ch1);
	else if (channel == AW36429_CHANNEL_CH2)
		hrtimer_cancel(&aw36429_timer_ch2);
	else {
		printk("Error channel\n");
		return -1;
	}

	return 0;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int aw36429_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg fl_arg={0};
	int channel;
	ktime_t ktime;
	

	unsigned long ret = 0;
#if 0
	fl_arg = kzalloc(sizeof(struct flashlight_dev_arg),GFP_KERNEL);
	if (!fl_arg)
		return -ENOMEM;
#endif
	printk("%s \n",__func__);

	//fl_arg = (struct flashlight_dev_arg *)arg;
	

	//printk("%s channel %d,timeout %d ms\n",__func__,fl_arg->channel,fl_arg->arg);


	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:

			ret = copy_from_user(&fl_arg, (struct flashlight_dev_arg __user *)arg, sizeof(struct flashlight_dev_arg));
	if(ret){
		pr_err("%s copy_from_user error,ret=%d\n",__func__,ret);

		return -EFAULT;
	}

	channel = fl_arg.channel;
	
		/* verify channel */
	if (channel < 0 || channel >= AW36429_CHANNEL_NUM) {
		printk("Failed with error channel\n");
		return -EINVAL;
	}
		printk("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg.arg);
		aw36429_timeout_ms[channel] = fl_arg.arg;
		break;

	case FLASH_IOC_SET_DUTY:
			ret = copy_from_user(&fl_arg, (struct flashlight_dev_arg __user *)arg, sizeof(struct flashlight_dev_arg));
	if(ret){
		pr_err("%s copy_from_user error,ret=%d\n",__func__,ret);

		return -EFAULT;
	}
		channel = fl_arg.channel;
		printk("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg.arg);
		aw36429_set_level(channel, fl_arg.arg);
		break;

	case FLASH_IOC_SET_ONOFF:
					ret = copy_from_user(&fl_arg, (struct flashlight_dev_arg __user *)arg, sizeof(struct flashlight_dev_arg));
	if(ret){
		pr_err("%s copy_from_user error,ret=%d\n",__func__,ret);

		return -EFAULT;
	}
	channel = fl_arg.channel;
		printk("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg.arg);
		if (fl_arg.arg == 1) {

			printk("%s line%d,aw36429_timeout_ms[%d] = %d\n",__func__,__LINE__,channel,aw36429_timeout_ms[channel]);
			if (aw36429_timeout_ms[channel]) {

				printk("%s line%d,channel = %d\n",__func__,__LINE__,channel);
				ktime = ktime_set(aw36429_timeout_ms[channel] / 1000,
						(aw36429_timeout_ms[channel] % 1000) * 1000000);
				aw36429_timer_start(channel, ktime);
			}
			aw36429_enable(channel);
		} else {
			aw36429_disable(channel);
			aw36429_timer_cancel(channel);
		}
		break;

	case FLASH_IOC_SET_BLINK_OFF_TIMEOUT:{
		struct flashlight_dev_arg blink_off_tm;
		
		ret = copy_from_user(&blink_off_tm, (void *)arg, sizeof(struct flashlight_dev_arg));
		if(ret){
			pr_err("%s line%d copy_from_user error,ret=%d\n",__func__,__LINE__,ret);
			
			ret = -EFAULT;
		}
		
		printk("FLASH_IOC_SET_BLINK_OFF_TIMEOUT (channel %d): %d ms\n",blink_off_tm.channel,blink_off_tm.blink_off_ms);

		aw36429_blinkoff_ms[blink_off_tm.channel] = blink_off_tm.blink_off_ms;
		
		break;
		}

case FLASH_IOC_STROBE_SET:{
		struct flashlight_gpioctl_arg gpioctl;

		ret = copy_from_user(&gpioctl, (void *)arg, sizeof(struct flashlight_gpioctl_arg));
		if(ret){
			pr_err("%s line%d copy_from_user error,ret=%d\n",__func__,__LINE__,ret);
			
			ret = -EFAULT;
		}

		printk("%s line%d gpio %d,status = %d\n",__func__,__LINE__,gpioctl.gpio,gpioctl.status);
		aw36429_gpio_en(aw36429_chip_priv,gpioctl.status);

		break;
		}

	default:
		printk("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg.arg);

		return -ENOTTY;
	}

	return 0;
}

static int aw36429_open(void)
{
	/* Actual behavior move to set driver function since power saving issue */
	return 0;
}

static int aw36429_release(void)
{
	/* uninit chip and clear usage count */
	mutex_lock(&aw36429_mutex);
	use_count--;
	if (!use_count)
		aw36429_uninit();
	if (use_count < 0)
		use_count = 0;
	mutex_unlock(&aw36429_mutex);

	printk("Release: %d\n", use_count);

	return 0;
}

static int aw36429_set_driver(int set)
{
	/* init chip and set usage count */
	mutex_lock(&aw36429_mutex);
	if (!use_count)
		aw36429_init();
	use_count++;
	mutex_unlock(&aw36429_mutex);

	printk("Set driver: %d\n", use_count);

	return 0;
}

static ssize_t aw36429_strobe_store(struct flashlight_arg arg)
{
	aw36429_set_driver(1);
	aw36429_set_level(arg.ct, arg.level);
	aw36429_enable(arg.ct);
	msleep(arg.dur);
	aw36429_disable(arg.ct);
	//aw36429_release(NULL);
	aw36429_set_driver(0);
	return 0;
}

static struct flashlight_operations aw36429_ops = {
	aw36429_open,
	aw36429_release,
	aw36429_ioctl,
	aw36429_strobe_store,
	aw36429_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int aw36429_gpio_en(struct aw36429_chip_data *data, int enable)
{
	int ret = 0;

	gpio_set_value(data->gpio_en, enable);
	pr_err("%s gpio_en = %d\n",__func__,gpio_get_value(data->gpio_3v3en));

	return ret;
}

static int aw36429_3v3_en(struct aw36429_chip_data *data, int enable)
{
	int ret = 0;

	gpio_set_value(data->gpio_3v3en, enable);
	// pr_err("aw36429 gpio->3v3en = %d\n",gpio_get_value(data->gpio_3v3en));

	return ret;
}

static int aw36429_1v8_en(struct aw36429_chip_data *data, int enable)
{
	int ret = 0;

	//gpio_set_value(data->gpio_1v8en, enable);
	ret = gpio_direction_output(data->gpio_1v8en, enable);
	pr_err("aw36429 ret = %d,gpio_1v8en = %d\n",ret,gpio_get_value(data->gpio_1v8en));

	return ret;
}

static int aw36429_read_chipid(struct aw36429_chip_data *aw36429)
{
    int ret = -1;
    unsigned char cnt = 0;
    unsigned char reg_val = 0;
  
    while(cnt < AW_READ_CHIPID_RETRIES) {
        ret = aw36429_i2c_read(aw36429->client, REG_CHIP_ID, &reg_val);
        if (ret < 0) {
            dev_err(&aw36429->client->dev,
                "%s: failed to read register aw36429_REG_ID: %d\n",
                __func__, ret);
            return -EIO;
        }
        switch (reg_val) {
        case AW36429_ID:
            pr_info("%s aw36429 detected\n", __func__);
            aw36429->chip_id = AW36429_ID;
            return 0;
        default:
            pr_info("%s unsupported device revision (0x%x)\n",
                __func__, reg_val );
            break;
        }
        cnt ++;
 
        msleep(AW_READ_CHIPID_RETRY_DELAY);
    }
 
    return -EINVAL;
}

static int aw36429_chip_init(struct aw36429_chip_data *aw36429_data)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * aw36429_init();
	 */

	int ret = 0;
	
	aw36429_data->gpio_en = of_get_named_gpio(aw36429_data->client->dev.of_node, "awinic,aw36429-strobe", 0); 
	if (gpio_is_valid(aw36429_data->gpio_en)) {
		ret = devm_gpio_request_one(&aw36429_data->client->dev, aw36429_data->gpio_en,GPIOF_OUT_INIT_LOW, "awinic,aw36429-strobe");
		if (ret < 0) {
			dev_err(&aw36429_data->client->dev,"request strobe gpio error,ret=%d\n",ret);
			return ret;
		}   
	}

	aw36429_data->gpio_3v3en = of_get_named_gpio(aw36429_data->client->dev.of_node, "awinic,aw36429-3v3en", 0); 
	if (gpio_is_valid(aw36429_data->gpio_3v3en)) {
		ret = devm_gpio_request_one(&aw36429_data->client->dev, aw36429_data->gpio_3v3en,GPIOF_OUT_INIT_LOW, "awinic,aw36429-3v3en");
		if (ret < 0) {
			dev_err(&aw36429_data->client->dev,"request 3v3en gpio error,ret=%d\n",ret);
			return ret;
		}   
	}	

	aw36429_data->gpio_1v8en = of_get_named_gpio(aw36429_data->client->dev.of_node, "awinic,aw36429-1v8en", 0); 
	if (gpio_is_valid(aw36429_data->gpio_1v8en)) {
		ret = devm_gpio_request_one(&aw36429_data->client->dev, aw36429_data->gpio_1v8en,GPIOF_OUT_INIT_LOW, "awinic,aw36429-1v8en");
		if (ret < 0) {
			dev_err(&aw36429_data->client->dev,"request gpio_1v8en gpio error,ret=%d\n",ret);
			return ret;
		}   
	}

	return 0;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//AW36429 Debug file
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t aw36429_get_reg(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned char reg_val;
	unsigned char i;
	ssize_t len = 0;
	for(i=0;i<=0x0B;i++) /*read reg val 0x00-0x0B*/
	{
		aw36429_i2c_read(aw36429_i2c_client,i,&reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg0x%02x = 0x%02X \n", i,reg_val);
	}
	/*0X0F*/
		aw36429_i2c_read(aw36429_i2c_client,0x0F,&reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg0x0F = 0x%02X \n", reg_val);
	/*0X10*/
		aw36429_i2c_read(aw36429_i2c_client,0x10,&reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg0x10 = 0x%02X \n", reg_val);

	len += snprintf(buf+len, PAGE_SIZE-len, "\r\n");
	return len;
}

static ssize_t aw36429_set_reg(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	unsigned int databuf[2];
	if(2 == sscanf(buf,"%x %x",&databuf[0], &databuf[1]))
	{
		aw36429_i2c_write(aw36429_i2c_client,databuf[0],databuf[1]);
	}
	return len;
}


static int aw36429_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	err = device_create_file(dev, &dev_attr_aw36429reg);
	return err;
}

#if 0
/*
 * Register devices
 *
 * Please DO NOT register flashlight device driver,
 * until success to probe hardware.
 */
int flashlight_dev_register(
		const char *name, struct flashlight_operations *dev_ops)
{
	struct flashlight_dev *fdev;
	int type_index, ct_index, part_index;
	int i;

	for (i = 0; i < flashlight_device_num; i++) {
		if (!strncmp(name, flashlight_id[i].name,
					FLASHLIGHT_NAME_SIZE)) {
			type_index = flashlight_id[i].type;
			ct_index = flashlight_id[i].ct;
			part_index = flashlight_id[i].part;

			if (flashlight_verify_index(
						type_index,
						ct_index,
						part_index)) {
				pr_err("Failed to register device (%s)\n",
						flashlight_id[i].name);
				continue;
			}

			pr_info("%s %d %d %d\n", flashlight_id[i].name,
					type_index, ct_index, part_index);

			mutex_lock(&fl_mutex);
			fdev = kzalloc(sizeof(*fdev), GFP_KERNEL);
			if (!fdev) {
				mutex_unlock(&fl_mutex);
				return -ENOMEM;
			}
			fdev->ops = dev_ops;
			fdev->dev_id = flashlight_id[i];
			fdev->low_pt_level = -1;
			fdev->charger_status = FLASHLIGHT_CHARGER_READY;
			list_add_tail(&fdev->node, &flashlight_list);
			mutex_unlock(&fl_mutex);
		}
	}

	return 0;
}
#endif

static int aw36429_misc_open(struct inode *inode, struct file *file)
{
	int rc = 0;

	printk(KERN_NOTICE"%s\n",__func__);

	return rc;
}

static int aw36429_misc_release(struct inode *inode, struct file *file)
{
	int rc = 0;

	printk(KERN_NOTICE"%s\n",__func__);

	return rc;
}

static long aw36429_misc_unlocked_ioctl(struct file *file,
				unsigned int cmd, unsigned long arg)
{
	int rc = 0;

	printk(KERN_NOTICE"%s,cmd = %d\n",__func__,cmd);

	aw36429_ioctl(cmd,arg);

	return rc;
}

static long aw36429_misc_compat_ioctl(struct file *file,
				unsigned int cmd, unsigned long arg)
{
	int rc = 0;

	printk(KERN_NOTICE"%s,cmd = %d\n",__func__,cmd);

	aw36429_ioctl(cmd,arg);

	return rc;
}

static const struct file_operations aw36429_misc_fops = {
	.owner		= THIS_MODULE,
	.open		= aw36429_misc_open,
	.release	= aw36429_misc_release,

	.unlocked_ioctl	= aw36429_misc_unlocked_ioctl,
	.compat_ioctl	= aw36429_misc_compat_ioctl,
};

static struct miscdevice aw36429_misc_device = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= AW36429_NAME,
	.fops	= &aw36429_misc_fops,
};

static int aw36429_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct aw36429_chip_data *chip;
	struct aw36429_platform_data *pdata = client->dev.platform_data;
	int err;

	printk(KERN_NOTICE"%s Probe start.\n",__func__);

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct aw36429_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	/* init platform data */
	if (!pdata) {
		pr_err("Platform data does not exist\n");
		#if 0
		pdata = kzalloc(sizeof(struct aw36429_platform_data), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_init_pdata;
		}
		#endif
		chip->no_pdata = 1;
	}
	//chip->pdata = pdata;
	
	i2c_set_clientdata(client, chip);
	aw36429_i2c_client = client;

	/* init chip hw */
	aw36429_chip_init(chip);

	aw36429_1v8_en(chip,1);

    // enable power to aw36429
	aw36429_3v3_en(chip,1);

	msleep(2);

    /* aw36429 chip id */
    err = aw36429_read_chipid(chip);
    if (err < 0) {
        dev_err(&aw36429_i2c_client->dev, "%s: aw36429_read_chipid failed ret=%d\n", __func__, err);
        goto err_free;
    }
	
	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* soft rst */
	aw36429_soft_reset();

	/* set default config for aw36429*/
	aw36429_set_default();
	

	/* init work queue */
	INIT_WORK(&aw36429_work_ch1, aw36429_work_disable_ch1);
	INIT_WORK(&aw36429_work_ch2, aw36429_work_disable_ch2);

	/* init timer */
	hrtimer_init(&aw36429_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw36429_timer_ch1.function = aw36429_timer_func_ch1;
	hrtimer_init(&aw36429_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw36429_timer_ch2.function = aw36429_timer_func_ch2;
	aw36429_timeout_ms[AW36429_CHANNEL_CH1] = 1000;
	aw36429_timeout_ms[AW36429_CHANNEL_CH2] = 1000;
	aw36429_timeout_ms[AW36429_CHANNEL_CH3] = 1000;
	aw36429_timeout_ms[AW36429_CHANNEL_CH4] = 1000;

	hrtimer_init(&aw36429_blinkoff_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw36429_blinkoff_timer_ch1.function = aw36429_timer_func_ch1;
	hrtimer_init(&aw36429_blinkoff_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw36429_blinkoff_timer_ch1.function = aw36429_timer_func_ch2;

	/* register flashlight operations */
	#if 0
	if (flashlight_dev_register(AW36429_NAME, &aw36429_ops)) {
		printk("Failed to register flashlight device.\n");
		err = -EFAULT;
		goto err_free;
	}
	#endif
	/* clear usage count */
	use_count = 0;
	
	aw36429_create_sysfs(client);

	err = misc_register(&aw36429_misc_device);
	if (err) {
		pr_err("cannot register miscdev aw36429_misc_device (err=%d)\n", err);
		goto err_free;
	}

	aw36429_chip_priv = chip;

	printk(KERN_NOTICE"%s Probe ok.\n",__func__);

	return 0;

err_free:
	kfree(chip->pdata);
err_init_pdata:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
err_out:
	return err;
}

static int aw36429_i2c_remove(struct i2c_client *client)
{
	struct aw36429_chip_data *chip = i2c_get_clientdata(client);

	printk(KERN_NOTICE"aw36429 i2c Remove start.\n");

	/* flush work queue */
	flush_work(&aw36429_work_ch1);
	flush_work(&aw36429_work_ch2);

	/* unregister flashlight operations */
	//flashlight_dev_unregister(AW36429_NAME);

	misc_deregister(&aw36429_misc_device);

	/* free resource */
	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);

	printk(KERN_NOTICE"aw36429 i2c Remove done.\n");

	return 0;
}

static void aw36429_i2c_shutdown(struct i2c_client *client)
{
	printk(KERN_NOTICE"aw36429 shutdown start.\n");

	aw36429_i2c_write(aw36429_i2c_client, AW36429_REG_ENABLE,
						AW36429_CHIP_STANDBY);

	printk(KERN_NOTICE"aw36429 shutdown done.\n");
}

static const struct i2c_device_id aw36429_i2c_id[] = {
	{AW36429_IDTBL_NAME, 0},
	{AW36406_IDTBL_NAME, (kernel_ulong_t)&st_aw36406_i2c_hwdata},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id aw36429_i2c_of_match[] = {
	{.compatible = AW36429_DTNAME_I2C},
	{},
};
#endif

static struct i2c_driver aw36429_i2c_driver = {
	.driver = {
		   .name = AW36429_DEV_DRV_NAME,
#ifdef CONFIG_OF
		   .of_match_table = aw36429_i2c_of_match,
#endif
		   },
	.probe = aw36429_i2c_probe,
	.remove = aw36429_i2c_remove,
	.shutdown = aw36429_i2c_shutdown,
	.id_table = aw36429_i2c_id,
};

static int __init aw36429_i2c_init(void)
{
    int ret = 0;
 
    printk(KERN_NOTICE"aw36429 i2c driver init\n");
 
    ret = i2c_add_driver(&aw36429_i2c_driver);
    if(ret){
        pr_err("fail to add aw36429 device into i2c,ret=%d\n",ret);
        return ret;
    }
 
    return 0;
}
module_init(aw36429_i2c_init);

static void __exit aw36429_i2c_exit(void)
{
	printk(KERN_NOTICE"aw36429 i2c driver destroy\n");

	device_remove_file(&aw36429_i2c_client->dev,&dev_attr_aw36429reg);

    i2c_del_driver(&aw36429_i2c_driver);
}
module_exit(aw36429_i2c_exit);

//module_i2c_driver(aw36429_i2c_driver);

#if 0
/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int aw36429_probe(struct platform_device *dev)
{
	printk("aw36429_probe Probe start.\n");

	if (i2c_add_driver(&aw36429_i2c_driver)) {
		printk("Failed to add i2c driver.\n");
		return -1;
	}

	printk("aw36429_probe Probe done.\n");

	return 0;
}

static int aw36429_remove(struct platform_device *dev)
{
	printk("aw36429_remove Remove start.\n");

	i2c_del_driver(&aw36429_i2c_driver);

	printk("aw36429_remove Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id aw36429_of_match[] = {
	{.compatible = AW36429_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, aw36429_of_match);
#else
static struct platform_device aw36429_platform_device[] = {
	{
		.name = AW36429_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, aw36429_platform_device);
#endif

static struct platform_driver aw36429_platform_driver = {
	.probe = aw36429_probe,
	.remove = aw36429_remove,
	.driver = {
		.name = AW36429_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = aw36429_of_match,
#endif
	},
};

static int __init flashlight_aw36429_init(void)
{
	int ret;

	printk("flashlight_aw36429-Init start.\n");
	pr_info("%s: driver version: %s\n", __func__, AW36429_DRIVER_VERSION);

#ifndef CONFIG_OF
	ret = platform_device_register(&aw36429_platform_device);
	if (ret) {
		printk("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&aw36429_platform_driver);
	if (ret) {
		printk("Failed to register platform driver\n");
		return ret;
	}

	printk("flashlight_aw36429 Init done.\n");

	return 0;
}

static void __exit flashlight_aw36429_exit(void)
{
	printk("flashlight_aw36429-Exit start.\n");

	platform_driver_unregister(&aw36429_platform_driver);

	printk("flashlight_aw36429 Exit done.\n");
}


module_init(flashlight_aw36429_init);
module_exit(flashlight_aw36429_exit);
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joseph <zhangzetao@awinic.com.cn>");
MODULE_DESCRIPTION("AW Flashlight AW36429 Driver");
