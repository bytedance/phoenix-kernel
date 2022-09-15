/*
 * Copyright (c) aw3750x, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_AW3750X_LED_H__
#define __LINUX_AW3750X_LED_H__

#define AW37501_VERSION_ID		0x16
#define AW37502_VERSION_ID		0x02
#define AW37503_VERSION_ID		0x04

#define AW3750X_REG_VOUTP		0x00
#define AW3750X_REG_VOUTN		0x01
#define AW3750X_REG_APPS		0x03
#define AW3750X_REG_CTRL		0x04
#define AW3750X_REG_PRO			0x21
#define AW3750X_REG_VERSION		0x44

#define AW3750X_REG_MAX			0x2f
#define AW3750X_OPEN_CMD		0x4C
#define AW3750X_CLOSE_CMD		0x00

#define AW3750X_OUT_CLEAR		0xE0

/*register read/write access*/
#define REG_NONE_ACCESS			0
#define REG_RD_ACCESS			(1 << 0)
#define REG_WR_ACCESS			(1 << 1)
#define AW_I2C_RETRIES			5

#define AW3750X_SOFT_RESET		(1 << 5)
#define AW_SOFT_RESET_MASK		~(1 << 5)

struct aw3750x_pinctrl {
	struct pinctrl *pinctrl;
	struct pinctrl_state *aw_enn_default;
	struct pinctrl_state *aw_enn_high;
	struct pinctrl_state *aw_enn_low;
	struct pinctrl_state *aw_enp_default;
	struct pinctrl_state *aw_enp_high;
	struct pinctrl_state *aw_enp_low;
};

struct aw3750x_power {
	struct i2c_client *client;
	struct device *dev;
	struct mutex lock;
	struct aw3750x_pinctrl pinctrl;
	struct regulator_init_data *init_data;
	struct regulator_dev *regulators[2];

	int enn_gpio;
	int enp_gpio;
	int offset;
	int limit;
	int power_mode;
	int read_power_mode;
	int outp;
	int outn;
	int read_outp;
	int read_outn;
	int aw3750x_gpio_ctrl;
	u8 aw3750x_version_id;

};

const unsigned char aw3750x_reg_access[AW3750X_REG_MAX] = {
	[AW3750X_REG_VOUTP] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW3750X_REG_VOUTN] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW3750X_REG_APPS]  = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW3750X_REG_CTRL]  = REG_RD_ACCESS|REG_WR_ACCESS,
};


#endif
