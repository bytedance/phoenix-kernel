/*
* aw3750x.c
*
* Version: v0.2.0
*
* Copyright (c) 2021 AWINIC Technology CO., LTD
*
* Author: Alex <shiqiang@awinic.com>
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the
* Free Software Foundation; either version 2 of the License, or (at your
* option) any later version.
*/

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/leds.h>
#include <linux/string.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#include "aw3750x.h"
#include "aw3750x_reg.h"

#define AW3750X_DRIVER_VERSION	"V0.2.0"

static int aw3750x_enn_gpio;
static int aw3750x_enp_gpio;


static int aw3750x_i2c_write(struct aw3750x_power *aw3750x, u8 reg, u8 val)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret =
		i2c_smbus_write_byte_data(aw3750x->client, reg, val);
		if (ret < 0) {
			pr_info("%s: i2c_write cnt=%d error=%d\n",
				__func__, cnt, ret);
		} else {
			break;
		}
		cnt++;
		usleep_range(2000, 3000);
	}

	return ret;
}

static int aw3750x_i2c_read(struct aw3750x_power *aw3750x, u8 reg, u8 *val)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw3750x->client, reg);
		if (ret < 0) {
			pr_info("%s: i2c_read cnt=%d error=%d\n",
				__func__, cnt, ret);
		} else {
			*val = ret;
			break;
		}
		cnt++;
		usleep_range(2000, 3000);
	}
	return ret;
}

static int aw3750x_i2c_bit_write(struct aw3750x_power *aw3750x,
					u8 reg_addr,
					u16 mask,
					u8 reg_data)
{
	uint8_t reg_val = 0;
	int ret = 0;

	ret = aw3750x_i2c_read(aw3750x, reg_addr, &reg_val);
	reg_val &= mask;
	reg_val |= reg_data;
	ret = aw3750x_i2c_write(aw3750x, reg_addr, reg_val);

	return ret;
}

static int aw3750x_i2c_bit_clear(struct aw3750x_power *aw3750x, u8 reg_addr, u16 mask)
{
	uint8_t reg_val = 0;
	int ret = 0;

	ret = aw3750x_i2c_read(aw3750x, reg_addr, &reg_val);
	reg_val &= mask;
	ret = aw3750x_i2c_write(aw3750x, reg_addr, reg_val);

	return ret;
}


static int aw3750x_soft_reset(struct aw3750x_power *aw3750x)
{
	int ret = 0;

	pr_info("%s\n", __func__);

	ret = aw3750x_i2c_bit_write(aw3750x, AW3750X_REG_APPS,
						AW_SOFT_RESET_MASK,
						AW3750X_SOFT_RESET);
	if (ret < 0) {
		pr_info("%s aw3750x_i2c_bit_write failed\n", __func__);
		return ret;
	}
	mdelay(1);
	return 0;
}

static int aw3750x_disn_enable(struct aw3750x_power *aw3750x)
{
	int ret;

	pr_info("%s\n", __func__);

	ret = aw3750x_i2c_bit_write(aw3750x, AW3750X_REG_APPS,
						AW3750X_DISN_MASK,
						AW3750X_DISN_ENABLE);
	if (ret < 0) {
		pr_info("%s aw3750x_i2c_bit_write failed\n", __func__);
		return ret;
	}

	return 0;
}

static int aw3750x_disn_disable(struct aw3750x_power *aw3750x)
{
	int ret;

	pr_info("%s\n", __func__);

	ret = aw3750x_i2c_bit_clear(aw3750x, AW3750X_REG_APPS, AW3750X_DISN_MASK);
	if (ret < 0) {
		pr_info("%s aw3750x_i2c_bit_clear failed\n", __func__);
		return ret;
	}

	return 0;
}
static int aw3750x_disp_enable(struct aw3750x_power *aw3750x)
{
	int ret;

	pr_info("%s\n", __func__);

	ret = aw3750x_i2c_bit_write(aw3750x, AW3750X_REG_APPS,
						AW3750X_DISP_MASK,
						AW3750X_DISP_ENABLE);
	if (ret < 0) {
		pr_info("%s aw3750x_i2c_bit_write failed\n", __func__);
		return ret;
	}

	return 0;
}

static int aw3750x_disp_disable(struct aw3750x_power *aw3750x)
{
	int ret;

	pr_info("%s\n", __func__);

	ret = aw3750x_i2c_bit_clear(aw3750x, AW3750X_REG_APPS, AW3750X_DISP_MASK);
	if (ret < 0) {
		pr_info("%s aw3750x_i2c_bit_clear failed\n", __func__);
		return ret;
	}

	return 0;
}


/***************************sys attribute*********************************/
static ssize_t aw3750x_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw3750x_power *aw3750x = dev_get_drvdata(dev);

	unsigned char i = 0, reg_val = 0;
	ssize_t len = 0;
	int ret = 0;

	pr_info("%s\n", __func__);

	for (i = 0; i < AW3750X_REG_MAX; i++) {
		if (!(aw3750x_reg_access[i] & REG_RD_ACCESS))
		continue;
		ret = aw3750x_i2c_read(aw3750x, i, &reg_val);
		if (ret < 0) {
			pr_info("aw3750x_i2c_read failed,ret = %d\n", ret);
			break;
		}
		len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%02x\n",
				i, reg_val);
	}

	return len;
}

static ssize_t aw3750x_reg_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct aw3750x_power *aw3750x = dev_get_drvdata(dev);

	unsigned int databuf[2] = {0};

	pr_info("%s\n", __func__);

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw3750x_i2c_write(aw3750x, (unsigned char)databuf[0],
				       (unsigned char)databuf[1]);
	}

	return len;
}

static ssize_t aw3750x_power_outp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int ret = 0;
	struct aw3750x_power *aw3750x = dev_get_drvdata(dev);

	pr_info("%s\n", __func__);

	ret = aw3750x->regulators[0]
		     ->desc->ops->get_voltage_sel(aw3750x->regulators[0]);
	if (ret < 0) {
		pr_info("get_voltage_sel failed,ret = %d\n", ret);
		return ret;
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "outp : %d\n", ret);

	return len;
}

static ssize_t aw3750x_power_outp_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct aw3750x_power *aw3750x = dev_get_drvdata(dev);
	int ret = 0;
	unsigned int databuf[1] = {0};

	pr_info("%s\n", __func__);

	if (sscanf(buf, "%x", &databuf[0]) == 1) {
		if (databuf[0] >= MIN_VOLTAGE_VAL && databuf[0] <= MAX_VOLTAGE_VAL) {
			ret = aw3750x->regulators[1]->desc->ops->set_voltage_sel(aw3750x->regulators[1],
			(unsigned char)databuf[0]);
			if (ret < 0) {
				dev_err(dev, "%s set_voltage_sel error\n", __func__);
				return ret;
			}
		}
	}

	return len;
}

static ssize_t aw3750x_power_outn_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int ret = 0;
	struct aw3750x_power *aw3750x = dev_get_drvdata(dev);

	pr_info("%s\n", __func__);

	ret = aw3750x->regulators[1]
		     ->desc->ops->get_voltage_sel(aw3750x->regulators[1]);
	if (ret < 0 && ret > -MIN_VOLTAGE) {
		dev_err(dev, "%s get_voltage_sel failed\n", __func__);
		return ret;
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "outn : %d\n", ret);

	return len;
}

static ssize_t aw3750x_power_outn_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct aw3750x_power *aw3750x = dev_get_drvdata(dev);
	int ret = 0;
	unsigned int databuf[1] = {0};

	pr_info("%s\n", __func__);

	if (sscanf(buf, "%x", &databuf[0]) == 1) {
		if (databuf[0] >= MIN_VOLTAGE_VAL && databuf[0] <= MAX_VOLTAGE_VAL) {
			ret = aw3750x->regulators[1]->desc->ops->set_voltage_sel(aw3750x->regulators[0],
			(unsigned char)databuf[0]);
			if (ret < 0) {
				dev_err(dev, "%s set_voltage_sel error\n",
					__func__);
				return ret;
			}
		}
	}

	return len;
}

static ssize_t aw3750x_enn_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct aw3750x_power *aw3750x = dev_get_drvdata(dev);
	int ret = 0;

	pr_info("%s\n", __func__);

	ret = aw3750x->regulators[1]
		     ->desc->ops->is_enabled(aw3750x->regulators[1]);
	if (ret < 0) {
		dev_err(dev, "%s is_enabled error\n", __func__);
		return ret;
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "outn status: %d\n", ret);

	return len;
}

static ssize_t aw3750x_enn_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct aw3750x_power *aw3750x = dev_get_drvdata(dev);
	int ret = 0;
	int val = 0;

	pr_info("%s\n", __func__);

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0) {
		dev_err(dev, "%s kstrtouint failed\n", __func__);
		return ret;
	}

	if (val == 1) {
		ret = aw3750x->regulators[1]
			     ->desc->ops->enable(aw3750x->regulators[1]);
		if (ret < 0) {
			dev_err(dev, "%s enable error\n", __func__);
			return ret;
		}
	} else if (val == 0) {
		ret = aw3750x->regulators[1]
			     ->desc->ops->disable(aw3750x->regulators[1]);
		if (ret < 0) {
			dev_err(dev, "%s disable error\n", __func__);
			return ret;
		}
	} else {
		dev_err(dev, "%s input val error\n", __func__);
		return -EINVAL;
	}
	return len;
}

static ssize_t aw3750x_enp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{

	ssize_t len = 0;
	struct aw3750x_power *aw3750x = dev_get_drvdata(dev);
	int ret = 0;

	pr_info("%s\n", __func__);

	ret = aw3750x->regulators[0]
		     ->desc->ops->is_enabled(aw3750x->regulators[0]);
	if (ret < 0) {
		dev_err(dev, "%s is_enabled error\n", __func__);
		return ret;
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "outp status: %d\n", ret);

	return len;
}

static ssize_t aw3750x_enp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct aw3750x_power *aw3750x = dev_get_drvdata(dev);
	int ret = 0;
	int val = 0;

	pr_info("%s\n", __func__);

	ret = kstrtouint(buf, 10, &val);
	if (ret < 0) {
		dev_err(dev, "%s kstrtouint failed\n", __func__);
		return ret;
	}
	pr_info("%s val = %d\n", __func__, val);

	if (val == 1) {
		ret = aw3750x->regulators[0]
			     ->desc->ops->enable(aw3750x->regulators[0]);
		if (ret < 0) {
			dev_err(dev, "%s enable error\n", __func__);
			return ret;
		}
	} else if (val == 0) {
		ret = aw3750x->regulators[0]
			     ->desc->ops->disable(aw3750x->regulators[0]);
		if (ret < 0) {
			dev_err(dev, "%s disable error\n", __func__);
			return ret;
		}
	} else {
		dev_err(dev, "%s input val error\n", __func__);
		return -EINVAL;
	}
	return len;
}

int aw3750x_enp_enn_set(int val)
{
    int ret = 0;

    if (val == 1) {
        msleep(8);
        if (gpio_is_valid(aw3750x_enp_gpio)) {
            gpio_direction_output(aw3750x_enp_gpio, 1);
        }
        msleep(8);
        if (gpio_is_valid(aw3750x_enn_gpio)) {
            gpio_direction_output(aw3750x_enn_gpio, 1);
        }
        msleep(8);
    } else if (val == 0) {
        if (gpio_is_valid(aw3750x_enn_gpio)) {
            gpio_direction_output(aw3750x_enn_gpio, 0);
        }
        msleep(2);
        if (gpio_is_valid(aw3750x_enp_gpio)) {
            gpio_direction_output(aw3750x_enp_gpio, 0);
        }
        msleep(8);
    } else {
        pr_err("%s input val error\n", __func__);
        return -EINVAL;
    }

    return ret;
}
EXPORT_SYMBOL(aw3750x_enp_enn_set);

static ssize_t aw3750x_current_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int ret = 0;
	struct aw3750x_power *aw3750x = dev_get_drvdata(dev);

	pr_info("%s\n", __func__);

	ret = aw3750x->regulators[0]
		     ->desc->ops->get_current_limit(aw3750x->regulators[0]);
	if (ret < 0) {
		dev_err(dev, "%s get_current_limit error\n", __func__);
		return ret;
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "modes: %d\n", ret);

	return len;
}

static ssize_t aw3750x_current_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct aw3750x_power *aw3750x = dev_get_drvdata(dev);
	int ret = 0;
	int databuf[2] = {0};

	pr_info("%s\n", __func__);

	if (sscanf(buf, "%d %d", &databuf[0], &databuf[1]) == 2) {
		ret = aw3750x->regulators[1]->desc->ops->
				set_current_limit(aw3750x->regulators[1],
						  databuf[0], databuf[1]);
		if (ret < 0) {
			dev_err(dev, "%s set_current_limit error\n", __func__);
			return ret;
		}
	}

	return len;
}

static ssize_t aw3750x_disn_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct aw3750x_power *aw3750x = dev_get_drvdata(dev);
	int ret = 0;
	int val = 0;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0) {
		dev_err(dev, "%s kstrtouint failed\n", __func__);
		return ret;
	}
	if (val > 0) {
		ret = aw3750x_disn_enable(aw3750x);
		if (ret < 0) {
			dev_err(dev, "%s aw3750x_disn_enable error\n",
				__func__);
			return ret;
		}
	} else if (val == 0) {
		ret = aw3750x_disn_disable(aw3750x);
		if (ret < 0) {
			dev_err(dev, "%s aw3750x_disn_disable error\n",
				__func__);
			return ret;
		}
	} else {
		return -EINVAL;
	}
	return len;
}

static ssize_t aw3750x_disp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct aw3750x_power *aw3750x = dev_get_drvdata(dev);
	int ret = 0;
	int val = 0;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0) {
		dev_err(dev, "%s kstrtouint failed\n", __func__);
		return ret;
	}
	if (val > 0) {
		ret = aw3750x_disp_enable(aw3750x);
		if (ret < 0) {
			dev_err(dev, "%s aw3750x_disp_enable error\n",
				__func__);
			return ret;
		}
	} else if (val == 0) {
		ret = aw3750x_disp_disable(aw3750x);
		if (ret < 0) {
			dev_err(dev, "%s aw3750x_disp_disable error\n",
				__func__);
			return ret;
		}
	} else {
		return -EINVAL;
	}
	return len;
}

static ssize_t aw3750x_listoutp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct aw3750x_power *aw3750x = dev_get_drvdata(dev);
	int ret = 0;
	int val = 0;

	ret = kstrtouint(buf, 10, &val);
	if (ret < 0) {
		dev_err(dev, "%s kstrtouint failed\n", __func__);
		return ret;
	}
	if (val < 0 || val >= N_VOLTAGE) {
		dev_err(dev, "%s,error n_voltage = %d\n", __func__, val);
		return -EINVAL;
	}

	ret =
	aw3750x->regulators[1]->desc->ops->list_voltage(aw3750x->regulators[1],
							val);
	if (ret < 0) {
		dev_err(dev, "%s list_voltage error\n", __func__);
		return ret;
	}
	pr_info("%s, list_voltage result = %d\n", __func__, ret);
	return len;
}

static ssize_t aw3750x_listoutn_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct aw3750x_power *aw3750x = dev_get_drvdata(dev);
	int ret = 0;
	int val = 0;

	pr_info("%s,enter\n", __func__);
	ret = kstrtouint(buf, 10, &val);
	if (ret < 0) {
		dev_err(dev, "%s kstrtouint failed\n", __func__);
		return ret;
	}
	if (val < 0 || val >= N_VOLTAGE)
		return -EINVAL;

	ret =
	aw3750x->regulators[0]->desc->ops->list_voltage(aw3750x->regulators[0],
							val);
	if (ret < 0) {
		dev_err(dev, "%s list_voltage error\n", __func__);
		return ret;
	}
	pr_info("%s, list_voltage result = %d\n", __func__, ret);
	return len;
}

static ssize_t aw3750x_reset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct aw3750x_power *aw3750x = dev_get_drvdata(dev);
	int ret = 0;
	int val = 0;

	pr_info("%s,enter\n", __func__);
	ret = kstrtouint(buf, 0, &val);
	if (ret < 0) {
		dev_err(dev, "%s kstrtouint failed\n", __func__);
		return ret;
	}

	if (val == 1) {
		ret = aw3750x_soft_reset(aw3750x);
		if (ret < 0) {
			dev_err(dev, "%s aw3750x_soft_reset error\n", __func__);
			return ret;
		}
	} else {
		return -EINVAL;
	}
	return len;
}

static DEVICE_ATTR(reg, 0664, aw3750x_reg_show, aw3750x_reg_store);
static DEVICE_ATTR(outp, 0664, aw3750x_power_outp_show,
			       aw3750x_power_outp_store);
static DEVICE_ATTR(outn, 0664, aw3750x_power_outn_show,
			       aw3750x_power_outn_store);
static DEVICE_ATTR(enn, 0664, aw3750x_enn_show, aw3750x_enn_store);
static DEVICE_ATTR(enp, 0664, aw3750x_enp_show, aw3750x_enp_store);
static DEVICE_ATTR(current_mode, 0664, aw3750x_current_mode_show,
				       aw3750x_current_mode_store);
static DEVICE_ATTR(disn, 0664, NULL, aw3750x_disn_store);
static DEVICE_ATTR(disp, 0664, NULL, aw3750x_disp_store);
static DEVICE_ATTR(list_outp, 0664, NULL, aw3750x_listoutp_store);
static DEVICE_ATTR(list_outn, 0664, NULL, aw3750x_listoutn_store);
static DEVICE_ATTR(reset, 0664, NULL, aw3750x_reset_store);

static struct attribute *aw3750x_power_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_outp.attr,
	&dev_attr_outn.attr,
	&dev_attr_enn.attr,
	&dev_attr_enp.attr,
	&dev_attr_current_mode.attr,
	&dev_attr_disn.attr,
	&dev_attr_disp.attr,
	&dev_attr_list_outp.attr,
	&dev_attr_list_outn.attr,
	&dev_attr_reset.attr,
	NULL,
};

static struct attribute_group aw3750x_power_attr_group = {
	.attrs = aw3750x_power_attributes
};

/* *********************************driver********************************* */
static int
aw3750x_gpio_init(struct aw3750x_power *aw3750x, struct i2c_client *i2c)
{
	int ret;

	pr_info("%s enter\n", __func__);

    if (gpio_is_valid(aw3750x->enp_gpio)) {
        ret = gpio_request(aw3750x->enp_gpio, "aw3750x enp_gpio");
        if (ret) {
            pr_err("request for aw3750x enp_gpio failed\n",ret);
        }
    }

    if (gpio_is_valid(aw3750x->enn_gpio)) {
        ret = gpio_request(aw3750x->enn_gpio, "aw3750x enn_gpio");
        if (ret) {
            pr_err("request for aw3750x enn_gpio failed\n",ret);
        }
    }
	return 0;
}

static int
aw3750x_parse_dt(struct i2c_client *i2c, struct aw3750x_power *aw3750x,
		 struct device_node *np)
{
	int rc = 0;
	struct device *dev = &i2c->dev;

	pr_info("%s enter\n", __func__);

	rc = of_property_read_u32(np, "aw3750x_gpio_ctrl",
					&aw3750x->aw3750x_gpio_ctrl);
	if (rc < 0) {
		dev_err(&aw3750x->client->dev,
			"Failure reading aaw3750x_gpio_ctrl, rc = %d\n", rc);
	}

	if (aw3750x->aw3750x_gpio_ctrl == 1) {
		aw3750x->enn_gpio = of_get_named_gpio(np, "enn-gpio", 0);
		if (gpio_is_valid(aw3750x->enn_gpio)) {
			dev_info(dev, "%s: enn gpio provided ok.\n", __func__);
		} else {
			dev_err(dev, "%s: no enn gpio provided.\n", __func__);
			return -EIO;
		}

        aw3750x_enn_gpio = aw3750x->enn_gpio;
		aw3750x->enp_gpio = of_get_named_gpio(np, "enp-gpio", 0);
		if (gpio_is_valid(aw3750x->enp_gpio)) {
			dev_info(dev, "%s: enp gpio provided ok.\n", __func__);
		} else {
			dev_err(dev, "%s: no enp gpio provided.\n", __func__);
			return -EIO;
		}

        aw3750x_enp_gpio = aw3750x->enp_gpio;

		rc = aw3750x_gpio_init(aw3750x, i2c);
		if (rc < 0) {
			dev_err(&aw3750x->client->dev,
				"aw3750x_gpio_init failed, rc = %d\n", rc);
			return rc;
		}
		pr_info("%s: ctrl by gpio\n", __func__);
	} else {
		pr_info("%s: ctrl by IIC\n", __func__);
	}

	rc = of_property_read_u32(np, "outp", &aw3750x->outp);
	if (rc < 0) {
		dev_err(&aw3750x->client->dev,
			"Failure reading outp, rc = %d\n", rc);
		goto read_err;
	} else {
		pr_info("%s,outp = %#x\n",  __func__, aw3750x->outp);
	}

	rc = of_property_read_u32(np, "outn", &aw3750x->outn);
	if (rc < 0) {
		dev_err(&aw3750x->client->dev,
			"Failure reading outn, rc = %d\n", rc);
		goto read_err;
	} else {
		pr_info("%s,outn = %#x\n",  __func__, aw3750x->outn);
	}

	rc = of_property_read_u32(np, "limit", &aw3750x->limit);
	if (rc < 0) {
		dev_err(&aw3750x->client->dev,
			"Failure reading limit, rc = %d\n", rc);
		goto read_err;
	} else {
		pr_info("%s,power limit = %d\n", __func__, aw3750x->limit);
	}

	return 0;
read_err:
	return rc;
}

static int aw3750x_power_probe(struct i2c_client *i2c,
			   const struct i2c_device_id *id)
{
	int ret = 0;
	struct aw3750x_power *aw3750x = NULL;
	struct device_node *node = i2c->dev.of_node;

	pr_info("%s\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	aw3750x = devm_kzalloc(&i2c->dev,
			(sizeof(struct aw3750x_power)), GFP_KERNEL);
	if (!aw3750x)
		return -ENOMEM;

	aw3750x->dev = &i2c->dev;
	aw3750x->client = i2c;

	if (node) {
		ret = aw3750x_parse_dt(i2c, aw3750x, node);
		if (ret < 0)
			goto chip_error;
	} else {
		pr_info("%s, failed device node is null!\n", __func__);
		goto chip_error;
	}

	i2c_set_clientdata(i2c, aw3750x);

	ret = sysfs_create_group(&aw3750x->dev->kobj,
				&aw3750x_power_attr_group);
	if (ret < 0) {
		dev_err(&aw3750x->client->dev, "power sysfs ret: %d\n", ret);
		goto regulator_error;
	}

	pr_err("%s ok!\n", __func__);

	return 0;
	sysfs_remove_group(&aw3750x->dev->kobj,
					&aw3750x_power_attr_group);
regulator_error:
chip_error:
	devm_kfree(&i2c->dev, aw3750x);
	aw3750x = NULL;
    pr_err("%s failed\n", __func__);
	return ret;
}

static int aw3750x_power_remove(struct i2c_client *i2c)
{
	struct aw3750x_power *aw3750x = i2c_get_clientdata(i2c);
	int i = 2;

	pr_info("%s\n", __func__);
	sysfs_remove_group(&aw3750x->dev->kobj, &aw3750x_power_attr_group);
	while (--i >= 0)
		devm_regulator_unregister(aw3750x->dev, aw3750x->regulators[i]);

	devm_kfree(&i2c->dev, aw3750x);
	aw3750x = NULL;

	return 0;
}

static const struct i2c_device_id aw3750x_power_id[] = {
	{"aw3750x_led", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, aw3750x_power_id);

static const struct of_device_id aw3750x_match_table[] = {
	{.compatible = "awinic,aw3750x",},
	{ },
};

static struct i2c_driver aw3750x_power_driver = {
	.driver = {
		.name = "aw3750x",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw3750x_match_table),
	},
	.probe = aw3750x_power_probe,
	.remove = aw3750x_power_remove,
	.id_table = aw3750x_power_id,
};

static int __init aw3750x_module_init(void)
{
	pr_info("%s: driver version: %s\n", __func__, AW3750X_DRIVER_VERSION);
	return i2c_add_driver(&aw3750x_power_driver);
}
module_init(aw3750x_module_init);

static void __exit aw3750x_module_exit(void)
{
	i2c_del_driver(&aw3750x_power_driver);
}
module_exit(aw3750x_module_exit);

MODULE_AUTHOR("<shiqiang@awinic.com.cn>");
MODULE_DESCRIPTION("AWINIC AW3750X Power driver");
MODULE_LICENSE("GPL v2");
