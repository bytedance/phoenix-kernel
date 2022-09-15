// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022, Pico Immersive Pte. Ltd ("Pico"). All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <soc/qcom/boot_stats.h>
extern void __iomem *GPIOMAPBASE;

#define APQ_NR_GPIOS 180
#define REG_SIZE 0x1000
#define STATUS_OFFSET 0x10
#define APQ_GPIO_CFG(n)       (GPIOMAPBASE + (REG_SIZE * n))
#define APQ_GPIO_IN_OUT(n)    (GPIOMAPBASE + 4 + (REG_SIZE * n))
#define APQ_GPIO_PULL(x)    ((x.ctrl) & 0x3)
#define APQ_GPIO_FUNC(x)    (((x.ctrl) >> 2) & 0xF)
#define APQ_GPIO_DRV(x)     (((x.ctrl) >> 6) & 0x7)
#define APQ_GPIO_OUT_EN(x)  (((x.ctrl) >> 9) & 0x1)
#define APQ_GPIO_IN_VAL(x)  ((x.inout) & 0x1)
#define APQ_GPIO_OUT_VAL(x) (((x.inout) >> 1) & 0x1)
#define PMIC_GPIO(a, b) { .regname = a, .regaddr = b }
#define PMIC_LDO(a, b) { .regname = a, .regaddr = b }

typedef void (*store_func) (struct device * dev, void *data, size_t num);
typedef int (*show_func) (struct seq_file * m, void *data, size_t num);

static void apq_gpio_store(struct device *dev, void *data, size_t num);
static int apq_gpio_show(struct seq_file *m, void *data, size_t num);

static void pm8150_gpio_store(struct device *dev, void *data, size_t num);
static int pm8150_gpio_show(struct seq_file *m, void *data, size_t num);
static void pm8150_ldo_store(struct device *dev, void *data, size_t num);
static int pm8150_ldo_show(struct seq_file *m, void *data, size_t num);

static void pm8150a_gpio_store(struct device *dev, void *data, size_t num);
static int pm8150a_gpio_show(struct seq_file *m, void *data, size_t num);
static void pm8150a_ldo_store(struct device *dev, void *data, size_t num);
static int pm8150a_ldo_show(struct seq_file *m, void *data, size_t num);

static void pm8150b_gpio_store(struct device *dev, void *data, size_t num);
static int pm8150b_gpio_show(struct seq_file *m, void *data, size_t num);

static u32 debug_mask;
static bool sleep_saved;
#ifdef USE_DEBUGFS
static struct dentry *debugfs;
#else
/*procfs in /proc/power_debug/ 	*/
static struct proc_dir_entry *power_debug_proc_entry = NULL;
#endif

#define DEBUG 1
/* APQ GPIO */
struct apq_gpio {
	uint32_t ctrl;
	uint32_t inout;
};

enum gpiomux_pull {
	GPIOMUX_PULL_NONE = 0,
	GPIOMUX_PULL_DOWN,
	GPIOMUX_PULL_KEEPER,
	GPIOMUX_PULL_UP,
};

enum {
	DUMP_APQ_GPIO,
	DUMP_PM8150_GPIO,
	DUMP_PM8150_LDO,
	DUMP_PM8150A_GPIO,
	DUMP_PM8150A_LDO,
	DUMP_PM8150B_GPIO,
	DUMP_DEV_NUM
};


/* Registers fields decoding arrays */
static const char *apq_pull_map[4] = {
	[GPIOMUX_PULL_NONE] = "none",
	[GPIOMUX_PULL_DOWN] = "pd",
	[GPIOMUX_PULL_KEEPER] = "keeper",
	[GPIOMUX_PULL_UP] = "pu",
};

struct reg_property {
	char *regname;
	int regaddr;
};


struct ldo_property {
	char *name;
	uint8_t Onoff_value;
};

struct gpio_property {
	char *name;
	uint8_t Onoff_value;
};


struct dump_desc {
	const char *name;
	show_func show;
	store_func store;
	struct device *dev;
	size_t item_size;
	size_t item_count;
	void *sleep_data;
};

static const struct reg_property pm8150_reg_gpio_table[] = {
	/*pm8150 10 GPIOs*/
	PMIC_GPIO("GPIO1_STATUS", 0xC008),
	PMIC_GPIO("GPIO2_STATUS", 0xC108),
	PMIC_GPIO("GPIO3_STATUS", 0xC208),
	PMIC_GPIO("GPIO4_STATUS", 0xC308),
	PMIC_GPIO("GPIO5_STATUS", 0xC408),
	PMIC_GPIO("GPIO6_STATUS", 0xC508),
	PMIC_GPIO("GPIO7_STATUS", 0xC608),
	PMIC_GPIO("GPIO8_STATUS", 0xC708),
	PMIC_GPIO("GPIO9_STATUS", 0xC808),
	PMIC_GPIO("GPIO10_STATUS", 0xC908),

	PMIC_GPIO("GPIO1_DIG_OUT", 0xC044),
	PMIC_GPIO("GPIO2_DIG_OUT", 0xC144),
	PMIC_GPIO("GPIO3_DIG_OUT", 0xC244),
	PMIC_GPIO("GPIO4_DIG_OUT", 0xC344),
	PMIC_GPIO("GPIO5_DIG_OUT", 0xC444),
	PMIC_GPIO("GPIO6_DIG_OUT", 0xC544),
	PMIC_GPIO("GPIO7_DIG_OUT", 0xC644),
	PMIC_GPIO("GPIO8_DIG_OUT", 0xC744),
	PMIC_GPIO("GPIO9_DIG_OUT", 0xC844),
	PMIC_GPIO("GPIO10_DIG_OUT", 0xC944),

	PMIC_GPIO("GPIO1_MODE_CTRL",0xC040),
	PMIC_GPIO("GPIO2_MODE_CTRL",0xC140),
	PMIC_GPIO("GPIO3_MODE_CTRL",0xC240),
	PMIC_GPIO("GPIO4_MODE_CTRL",0xC340),
	PMIC_GPIO("GPIO5_MODE_CTRL",0xC440),
	PMIC_GPIO("GPIO6_MODE_CTRL",0xC540),
	PMIC_GPIO("GPIO7_MODE_CTRL",0xC640),
	PMIC_GPIO("GPIO8_MODE_CTRL",0xC740),
	PMIC_GPIO("GPIO9_MODE_CTRL",0xC840),
	PMIC_GPIO("GPIO10_MODE_CTRL",0xC940),
};

static const struct reg_property pm8150a_reg_gpio_table[] = {
	/*PM8150A 12 gpios*/
	PMIC_GPIO("GPIO1_STATUS", 0x4C008),
	PMIC_GPIO("GPIO2_STATUS", 0x4C108),
	PMIC_GPIO("GPIO3_STATUS", 0x4C208),
	PMIC_GPIO("GPIO4_STATUS", 0x4C308),
	PMIC_GPIO("GPIO5_STATUS", 0x4C408),
	PMIC_GPIO("GPIO6_STATUS", 0x4C508),
	PMIC_GPIO("GPIO7_STATUS", 0x4C608),
	PMIC_GPIO("GPIO8_STATUS", 0x4C708),
	PMIC_GPIO("GPIO9_STATUS", 0x4C808),
	PMIC_GPIO("GPIO10_STATUS", 0x4C908),
	PMIC_GPIO("GPIO11_STATUS", 0x4CA08),
	PMIC_GPIO("GPIO12_STATUS", 0x4CB08),

	PMIC_GPIO("GPIO1_DIG_OUT", 0x4C044),
	PMIC_GPIO("GPIO2_DIG_OUT", 0x4C144),
	PMIC_GPIO("GPIO3_DIG_OUT", 0x4C244),
	PMIC_GPIO("GPIO4_DIG_OUT", 0x4C344),
	PMIC_GPIO("GPIO5_DIG_OUT", 0x4C444),
	PMIC_GPIO("GPIO6_DIG_OUT", 0x4C544),
	PMIC_GPIO("GPIO7_DIG_OUT", 0x4C644),
	PMIC_GPIO("GPIO8_DIG_OUT", 0x4C744),
	PMIC_GPIO("GPIO9_DIG_OUT", 0x4C844),
	PMIC_GPIO("GPIO10_DIG_OUT", 0x4C944),
	PMIC_GPIO("GPIO11_DIG_OUT", 0x4CA44),
	PMIC_GPIO("GPIO12_DIG_OUT", 0x4CB44),

	PMIC_GPIO("GPIO1_MODE_CTRL",0x4C040),
	PMIC_GPIO("GPIO2_MODE_CTRL",0x4C140),
	PMIC_GPIO("GPIO3_MODE_CTRL",0x4C240),
	PMIC_GPIO("GPIO4_MODE_CTRL",0x4C340),
	PMIC_GPIO("GPIO5_MODE_CTRL",0x4C440),
	PMIC_GPIO("GPIO6_MODE_CTRL",0x4C540),
	PMIC_GPIO("GPIO7_MODE_CTRL",0x4C640),
	PMIC_GPIO("GPIO8_MODE_CTRL",0x4C740),
	PMIC_GPIO("GPIO9_MODE_CTRL",0x4C840),
	PMIC_GPIO("GPIO10_MODE_CTRL",0x4C940),
	PMIC_GPIO("GPI011_MODE_CTRL",0x4CA40),
	PMIC_GPIO("GPI012_MODE_CTRL",0x4CB40),
};


static const struct reg_property pm8150b_reg_gpio_table[] = {
	/*PM8150B 12 gpios*/
	PMIC_GPIO("GPIO1_STATUS", 0x2C008),
	PMIC_GPIO("GPIO2_STATUS", 0x2C108),
	PMIC_GPIO("GPIO3_STATUS", 0x2C208),
	PMIC_GPIO("GPIO4_STATUS", 0x2C308),
	PMIC_GPIO("GPIO5_STATUS", 0x2C408),
	PMIC_GPIO("GPIO6_STATUS", 0x2C508),
	PMIC_GPIO("GPIO7_STATUS", 0x2C608),
	PMIC_GPIO("GPIO8_STATUS", 0x2C708),
	PMIC_GPIO("GPIO9_STATUS", 0x2C808),
	PMIC_GPIO("GPIO10_STATUS", 0x2C908),
	PMIC_GPIO("GPIO11_STATUS", 0x2CA08),
	PMIC_GPIO("GPIO12_STATUS", 0x2CB08),

	PMIC_GPIO("GPIO1_DIG_OUT", 0x2C044),
	PMIC_GPIO("GPIO2_DIG_OUT", 0x2C144),
	PMIC_GPIO("GPIO3_DIG_OUT", 0x2C244),
	PMIC_GPIO("GPIO4_DIG_OUT", 0x2C344),
	PMIC_GPIO("GPIO5_DIG_OUT", 0x2C444),
	PMIC_GPIO("GPIO6_DIG_OUT", 0x2C544),
	PMIC_GPIO("GPIO7_DIG_OUT", 0x2C644),
	PMIC_GPIO("GPIO8_DIG_OUT", 0x2C744),
	PMIC_GPIO("GPIO9_DIG_OUT", 0x2C844),
	PMIC_GPIO("GPIO10_DIG_OUT", 0x2C944),
	PMIC_GPIO("GPIO11_DIG_OUT", 0x2CA44),
	PMIC_GPIO("GPIO12_DIG_OUT", 0x2CB44),

	PMIC_GPIO("GPIO1_MODE_CTRL",0x2C040),
	PMIC_GPIO("GPIO2_MODE_CTRL",0x2C140),
	PMIC_GPIO("GPIO3_MODE_CTRL",0x2C240),
	PMIC_GPIO("GPIO4_MODE_CTRL",0x2C340),
	PMIC_GPIO("GPIO5_MODE_CTRL",0x2C440),
	PMIC_GPIO("GPIO6_MODE_CTRL",0x2C540),
	PMIC_GPIO("GPIO7_MODE_CTRL",0x2C640),
	PMIC_GPIO("GPIO8_MODE_CTRL",0x2C740),
	PMIC_GPIO("GPIO9_MODE_CTRL",0x2C840),
	PMIC_GPIO("GPIO10_MODE_CTRL",0x2C940),
	PMIC_GPIO("GPI011_MODE_CTRL",0x2CA40),
	PMIC_GPIO("GPI012_MODE_CTRL",0x2CB40),
};

static const struct reg_property pmx55_reg_gpio_table[] = {
	/*pmx55 11 GPIOs*/
	PMIC_GPIO("GPIO1_STATUS", 0x8C008),
	PMIC_GPIO("GPIO2_STATUS", 0x8C108),
	PMIC_GPIO("GPIO3_STATUS", 0x8C208),
	PMIC_GPIO("GPIO4_STATUS", 0x8C308),
	PMIC_GPIO("GPIO5_STATUS", 0x8C408),
	PMIC_GPIO("GPIO6_STATUS", 0x8C508),
	PMIC_GPIO("GPIO7_STATUS", 0x8C608),
	PMIC_GPIO("GPIO8_STATUS", 0x8C708),
	PMIC_GPIO("GPIO9_STATUS", 0x8C808),
	PMIC_GPIO("GPIO10_STATUS", 0x8C908),
	PMIC_GPIO("GPIO11_STATUS", 0x8CA08),

	PMIC_GPIO("GPIO1_DIG_OUT", 0x8C044),
	PMIC_GPIO("GPIO2_DIG_OUT", 0x8C144),
	PMIC_GPIO("GPIO3_DIG_OUT", 0x8C244),
	PMIC_GPIO("GPIO4_DIG_OUT", 0x8C344),
	PMIC_GPIO("GPIO5_DIG_OUT", 0x8C444),
	PMIC_GPIO("GPIO6_DIG_OUT", 0x8C544),
	PMIC_GPIO("GPIO7_DIG_OUT", 0x8C644),
	PMIC_GPIO("GPIO8_DIG_OUT", 0x8C744),
	PMIC_GPIO("GPIO9_DIG_OUT", 0x8C844),
	PMIC_GPIO("GPIO10_DIG_OUT", 0x8C944),
	PMIC_GPIO("GPIO11_DIG_OUT", 0x8CA44),

	PMIC_GPIO("GPIO1_MODE_CTRL",0x8C040),
	PMIC_GPIO("GPIO2_MODE_CTRL",0x8C140),
	PMIC_GPIO("GPIO3_MODE_CTRL",0x8C240),
	PMIC_GPIO("GPIO4_MODE_CTRL",0x8C340),
	PMIC_GPIO("GPIO5_MODE_CTRL",0x8C440),
	PMIC_GPIO("GPIO6_MODE_CTRL",0x8C540),
	PMIC_GPIO("GPIO7_MODE_CTRL",0x8C640),
	PMIC_GPIO("GPIO8_MODE_CTRL",0x8C740),
	PMIC_GPIO("GPIO9_MODE_CTRL",0x8C840),
	PMIC_GPIO("GPIO10_MODE_CTRL",0x8C940),
	PMIC_GPIO("GPIO11_MODE_CTRL",0x8CA40),
};
static const struct reg_property pm8150_reg_ldo_table[] = {
	/* pm8150 18 ldos*/
	PMIC_LDO("L1_CTRL_EN_CTL", 0x14046),
	PMIC_LDO("L2_CTRL_EN_CTL", 0x14146),
	PMIC_LDO("L3_CTRL_EN_CTL", 0x14246),
	PMIC_LDO("L4_CTRL_EN_CTL", 0x14346),
	PMIC_LDO("L5_CTRL_EN_CTL", 0x14446),
	PMIC_LDO("L6_CTRL_EN_CTL", 0x14546),
	PMIC_LDO("L7_CTRL_EN_CTL", 0x14646),
	PMIC_LDO("L8_CTRL_EN_CTL", 0x14746),
	PMIC_LDO("L9_CTRL_EN_CTL", 0x14846),
	PMIC_LDO("L10_CTRL_EN_CTL", 0x14946),
	PMIC_LDO("L11_CTRL_EN_CTL", 0x14A46),
	PMIC_LDO("L12_CTRL_EN_CTL", 0x14B46),
	PMIC_LDO("L13_CTRL_EN_CTL", 0x14C46),
	PMIC_LDO("L14_CTRL_EN_CTL", 0x14D46),
	PMIC_LDO("L15_CTRL_EN_CTL", 0x14E46),
	PMIC_LDO("L16_CTRL_EN_CTL", 0x14F46),
	PMIC_LDO("L17_CTRL_EN_CTL", 0x15046),
	PMIC_LDO("L18_CTRL_EN_CTL", 0x15146),

	 /*pm8150 10 smps*/
	PMIC_LDO("S1_CTRL_EN_CTL", 0x11446),
	PMIC_LDO("S2_CTRL_EN_CTL", 0x11746),
	PMIC_LDO("S3_CTRL_EN_CTL", 0x11A46),
	PMIC_LDO("S4_CTRL_EN_CTL", 0x11D46),
	PMIC_LDO("S5_CTRL_EN_CTL", 0x12046),
	PMIC_LDO("S6_CTRL_EN_CTL", 0x12346),
	PMIC_LDO("S7_CTRL_EN_CTL", 0x12646),
	PMIC_LDO("S8_CTRL_EN_CTL", 0x12946),
	PMIC_LDO("S9_CTRL_EN_CTL", 0x12C46),
	PMIC_LDO("S10_CTRL_EN_CTL", 0x12F46),
};

static const struct reg_property pm8150a_reg_ldo_table[] = {
	/* PM8150A 11 ldos*/
	PMIC_LDO("L1_CTRL_EN_CTL", 0x54046),
	PMIC_LDO("L2_CTRL_EN_CTL", 0x54146),
	PMIC_LDO("L3_CTRL_EN_CTL", 0x54246),
	PMIC_LDO("L4_CTRL_EN_CTL", 0x54346),
	PMIC_LDO("L5_CTRL_EN_CTL", 0x54446),
	PMIC_LDO("L6_CTRL_EN_CTL", 0x54546),
	PMIC_LDO("L7_CTRL_EN_CTL", 0x54646),
	PMIC_LDO("L8_CTRL_EN_CTL", 0x54746),
	PMIC_LDO("L9_CTRL_EN_CTL", 0x54846),
	PMIC_LDO("L10_CTRL_EN_CTL", 0x54946),
	PMIC_LDO("L11_CTRL_EN_CTL", 0x54A46),

	/*PM8150A 8 smps*/
	PMIC_LDO("S1_CTRL_EN_CTL", 0x51446),
	PMIC_LDO("S2_CTRL_EN_CTL", 0x51746),
	PMIC_LDO("S3_CTRL_EN_CTL", 0x51A46),
	PMIC_LDO("S4_CTRL_EN_CTL", 0x51D46),
	PMIC_LDO("S5_CTRL_EN_CTL", 0x52046),
	PMIC_LDO("S6_CTRL_EN_CTL", 0x52346),
	PMIC_LDO("S7_CTRL_EN_CTL", 0x52646),
	PMIC_LDO("S8_CTRL_EN_CTL", 0x52946),

	/*misc registers*/
        PMIC_LDO("MISC_PM8150A_RGB_LED_EN", 0x5D046),//front red led, leds-qti-tri-led.c
        PMIC_LDO("MISC_PM8150A_FLASH_TORCH_LED_EN", 0x5D346),//flash torch, leds-qpnp-flash-v2.c
};

static const struct reg_property pmx55_reg_ldo_table[] = {
	/* pmx55 16 ldos*/
	PMIC_LDO("L1_CTRL_EN_CTL", 0x94046),
	PMIC_LDO("L2_CTRL_EN_CTL", 0x94146),
	PMIC_LDO("L3_CTRL_EN_CTL", 0x94246),
	PMIC_LDO("L4_CTRL_EN_CTL", 0x94346),
	PMIC_LDO("L5_CTRL_EN_CTL", 0x94446),
	PMIC_LDO("L6_CTRL_EN_CTL", 0x94546),
	PMIC_LDO("L7_CTRL_EN_CTL", 0x94646),
	PMIC_LDO("L8_CTRL_EN_CTL", 0x94746),
	PMIC_LDO("L9_CTRL_EN_CTL", 0x94846),
	PMIC_LDO("L10_CTRL_EN_CTL", 0x94946),
	PMIC_LDO("L11_CTRL_EN_CTL", 0x94A46),
	PMIC_LDO("L12_CTRL_EN_CTL", 0x94B46),
	PMIC_LDO("L13_CTRL_EN_CTL", 0x94C46),
	PMIC_LDO("L14_CTRL_EN_CTL", 0x94D46),
	PMIC_LDO("L15_CTRL_EN_CTL", 0x94E46),
	PMIC_LDO("L16_CTRL_EN_CTL", 0x94F46),

	 /*pmx55 7 smps*/
	PMIC_LDO("S1_CTRL_EN_CTL", 0x91446),
	PMIC_LDO("S2_CTRL_EN_CTL", 0x91746),
	PMIC_LDO("S3_CTRL_EN_CTL", 0x91A46),
	PMIC_LDO("S4_CTRL_EN_CTL", 0x91D46),
	PMIC_LDO("S5_CTRL_EN_CTL", 0x92046),
	PMIC_LDO("S6_CTRL_EN_CTL", 0x92346),
	PMIC_LDO("S7_CTRL_EN_CTL", 0x92646),
};
static const u32 sdm865_tile_offsets[] = {0x100000,0x500000, 0x900000};
static const unsigned int n_tile_offsets = ARRAY_SIZE(sdm865_tile_offsets);

/*TZ gpio*/
int gpio_tz[] = {28,29,30,31,40,41,42,43};

/* List of dump targets */
static struct dump_desc dump_devices[] = {
	[DUMP_APQ_GPIO] = {
		.name = "apq_gpio",
		.store = apq_gpio_store,
		.show = apq_gpio_show,
		.dev = NULL,
		.item_size = sizeof(struct apq_gpio),
		.item_count = APQ_NR_GPIOS,
	},

	[DUMP_PM8150A_LDO] = {
		/*phoenix use pm8150l, the hardware registers is same as pm8150a*/
		.name = "pm8150l_ldo",
		.store = pm8150a_ldo_store,
		.show = pm8150a_ldo_show,
		.dev = NULL,
		.item_size = sizeof(struct ldo_property),
		.item_count = ARRAY_SIZE(pm8150a_reg_ldo_table),
	}
	,

	[DUMP_PM8150A_GPIO] = {
		/*phoenix use pm8150l, the hardware registers is same as pm8150a*/
		.name = "pm8150l_gpio",
		.store = pm8150a_gpio_store,
		.show = pm8150a_gpio_show,
		.dev = NULL,
		.item_size = sizeof(struct gpio_property),
		.item_count = ARRAY_SIZE(pm8150a_reg_gpio_table),
	}
	,

	[DUMP_PM8150_LDO] = {
		.name = "pm8150_ldo",
		.store = pm8150_ldo_store,
		.show = pm8150_ldo_show,
		.dev = NULL,
		.item_size = sizeof(struct ldo_property),
		.item_count = ARRAY_SIZE(pm8150_reg_ldo_table),
	}
	,

	[DUMP_PM8150_GPIO] = {
		.name = "pm8150_gpio",
		.store = pm8150_gpio_store,
		.show = pm8150_gpio_show,
		.dev = NULL,
		.item_size = sizeof(struct gpio_property),
		.item_count = ARRAY_SIZE(pm8150_reg_gpio_table)
	}
	,

	[DUMP_PM8150B_GPIO] = {
		.name = "pm8150b_gpio",
		.store = pm8150b_gpio_store,
		.show = pm8150b_gpio_show,
		.dev = NULL,
		.item_size = sizeof(struct gpio_property),
		.item_count = ARRAY_SIZE(pm8150b_reg_gpio_table),
	}
};

static int tz_ctrl(int num)
{
	int i;
	int size;

	size = sizeof(gpio_tz) / sizeof(gpio_tz[0]);
	for (i = 0; i < size; i++) {
		if(num == gpio_tz[i])
			return 1;
	}
	return 0;
}


u32 sdm865_pinctrl_find_base(u32 gpio_id)
{
	int i;
	u32 val;

	if (gpio_id >= APQ_NR_GPIOS)
		return 0;
	for (i = 0; i < n_tile_offsets; i++) {
		val = readl_relaxed(APQ_GPIO_CFG(gpio_id)+ sdm865_tile_offsets[i]
				+ STATUS_OFFSET);
		if (val)
			return sdm865_tile_offsets[i];
	}
	return 0;
}
EXPORT_SYMBOL(sdm865_pinctrl_find_base);

static void apq_gpio_store(struct device *unused, void *data, size_t num)
{
	int gpio_id;
	u32 base;

	struct apq_gpio *gpios = (struct apq_gpio *)data;
	memset(data,0, sizeof(struct apq_gpio) * num);

	if (num > APQ_NR_GPIOS){
		pr_err("apq gpio numbers is out of bound\n");
		return ;
	}

	for(gpio_id = 0; gpio_id < num; gpio_id++){
		if (tz_ctrl(gpio_id)) {
			continue;
		}
		base = sdm865_pinctrl_find_base(gpio_id);
		gpios[gpio_id].ctrl = readl(APQ_GPIO_CFG(gpio_id) + base) & 0x3FF;
		gpios[gpio_id].inout = readl((APQ_GPIO_IN_OUT(gpio_id) + base)) & 0x3;
	}
}

static int apq_gpio_show(struct seq_file *m, void *data, size_t num)
{
	int i = 0;
	struct apq_gpio *gpios = (struct apq_gpio *)data;

	pr_debug("GPIOMAPBASE=0x%p \n", GPIOMAPBASE);
	seq_printf(m, "+-----------APQ gpio show---------------+\n");
	seq_printf(m, "+--+-----+-----+-----+------+------+\n");
	seq_printf(m, "|#  | dir | val | drv | func | pull |\n");
	seq_printf(m, "+--+-----+-----+-----+------+------+\n");

	for (i = 0; i < num; i++) {
		if (tz_ctrl(i)) {
			seq_printf(m, "|%03u|%-5s|%-5s|%-4s |%-6s|%-6s|\n", i,
					"TZ", "TZ", "TZ", "TZ", "TZ");
			continue;
		}

		seq_printf(m, "|%03u|%-5s|%-5u|%-2umA |%-6u|%-6s|\n", i,
				   APQ_GPIO_OUT_EN(gpios[i]) ? "out" : "in",
				   APQ_GPIO_OUT_EN(gpios[i]) ?
				   APQ_GPIO_OUT_VAL(gpios[i]) :
				   APQ_GPIO_IN_VAL(gpios[i]),
				   APQ_GPIO_DRV(gpios[i]) * 2 + 2,
				   APQ_GPIO_FUNC(gpios[i]),
				   apq_pull_map[APQ_GPIO_PULL(gpios[i])]
				  );
	}

	seq_printf(m, "+--+-----+-----+-----+------+------+\n");
    return 0;
}

extern int read_pmic_data(u8 sid, u16 addr, u8 * buf, int len);
//extern int read_pmic_data_pmx55(u8 sid, u16 addr, u8 * buf, int len);
void pm8150b_gpio_store(struct device *dev, void *data, size_t num)
{
	int index;
	uint8_t sid;
	uint8_t buf[1];
	uint16_t addr;
	int8_t ret;

	struct gpio_property *gpiomap_on_off = (struct gpio_property *)data;

	for (index = 0;
		 index < (sizeof(pm8150b_reg_gpio_table) / sizeof(struct reg_property));
		 index++) {
		sid = (pm8150b_reg_gpio_table[index].regaddr >> 16) & 0xF;
		addr = pm8150b_reg_gpio_table[index].regaddr & 0xFFFF;

		ret = read_pmic_data(sid, addr, buf, 1);
		if (ret < 0) {
			pr_err("SPMI read failed, err = %d\n", ret);
			goto done;
		}

		gpiomap_on_off[index].name = pm8150b_reg_gpio_table[index].regname;
		gpiomap_on_off[index].Onoff_value = buf[0];
		pr_debug("%s ,  ret=%d, value=0x%x\n", __func__, ret, buf[0]);
	}

done:
	return;
}

int pm8150b_gpio_show(struct seq_file *m, void *data, size_t num)
{
	int i = 0;
	struct gpio_property *gpiomap_on_off = (struct gpio_property *)data;

	seq_printf(m, "+--+------------+--PM8150B gpio_show----------+---\n");

	for (i = 0; i < num / 3; i++) {
		seq_printf(m, "|%-13s| value=0x%02x | %-7s | %-10s\n", gpiomap_on_off[i].name,
				   gpiomap_on_off[i].Onoff_value & 0xFF,
				   (gpiomap_on_off[i].Onoff_value >> 7) == 1 ? "enable" : "disable",
				   (gpiomap_on_off[i].Onoff_value & 0x01) == 1 ? "input_high" : "input_low"
				   );
	}

	seq_printf(m, "+--+-----------+-----+----+---+-----------+----+--\n");
	for (i = num / 3; i < (num/3)*2; i++) {
		seq_printf(m, "|%-13s| invert=%d | output_source=0x%02x\n", gpiomap_on_off[i].name,
				gpiomap_on_off[i].Onoff_value >> 7,(gpiomap_on_off[i].Onoff_value & 0x0f));
	}

	seq_printf(m, "+--+-----------+-----+----+---+-----------+----+--\n");

	for (i = (num/3)*2; i < num; i++) {
		seq_printf(m, "|%-13s| GPIO_MODE=0x%02x\n", gpiomap_on_off[i].name,
				gpiomap_on_off[i].Onoff_value & 0x3);
	}

	seq_printf(m, "+--+-----------+-----+----+---+-----------+----+--\n");
	return 0;
}

static void pm8150a_ldo_store(struct device *ssbi_dev, void *data, size_t num)
{
	int index;
	uint8_t sid;
	uint8_t buf[1];
	uint16_t addr;
	int8_t ret;

	struct ldo_property *ldo_on_off = (struct ldo_property *)data;

	for (index = 0;
		 index < (sizeof(pm8150a_reg_ldo_table) / sizeof(struct reg_property));
		 index++) {
		sid = (pm8150a_reg_ldo_table[index].regaddr >> 16) & 0xF;
		addr = pm8150a_reg_ldo_table[index].regaddr & 0xFFFF;

		ret = read_pmic_data(sid, addr, buf, 1);
		if (ret < 0) {
			pr_err("%s SPMI read failed, err = %d\n", __func__, ret);
			goto done;
		}
		ldo_on_off[index].name = pm8150a_reg_ldo_table[index].regname;
		ldo_on_off[index].Onoff_value = buf[0];
		pr_debug("%s ,  ret=%d, value=0x%x\n", __func__, ret, buf[0]);
	}

done:
	return;

}

static int pm8150a_ldo_show(struct seq_file *m, void *data, size_t num)
{
	int i = 0;
	struct ldo_property *ldo_on_off = (struct ldo_property *)data;

	seq_printf(m, "+--+------+----PM8150A ldo_show: On/Off--+---+--------\n");

	for (i = 0; i < num; i++) {
		if (!strncmp("MISC_PM8150A_RGB_LED_EN", pm8150a_reg_ldo_table[i].regname, strlen("MISC_PM8150A_RGB_LED_EN"))) {
			seq_printf(m, "|%-15s| on_off=0x%x\n", ldo_on_off[i].name,
				   (ldo_on_off[i].Onoff_value >> 5) & 0x07);
		} else {
			seq_printf(m, "|%-15s| on_off=0x%x\n", ldo_on_off[i].name,
				   (ldo_on_off[i].Onoff_value >> 7) & 0x01);
		}
	}

	seq_printf(m, "+--+-------+-----+----+---+-----------+----+---------\n");
	return 0;
}

void pm8150a_gpio_store(struct device *dev, void *data, size_t num)
{
	int index;
	uint8_t sid;
	uint8_t buf[1];
	uint16_t addr;
	int8_t ret;

	struct gpio_property *gpiomap_on_off = (struct gpio_property *)data;

	for (index = 0;
		 index < (sizeof(pm8150a_reg_gpio_table) / sizeof(struct reg_property));
		 index++) {
		sid = (pm8150a_reg_gpio_table[index].regaddr >> 16) & 0xF;
		addr = pm8150a_reg_gpio_table[index].regaddr & 0xFFFF;

		ret = read_pmic_data(sid, addr, buf, 1);
		if (ret < 0) {
			pr_err("SPMI read failed, err = %d\n", ret);
			goto done;
		}
		gpiomap_on_off[index].name = pm8150a_reg_gpio_table[index].regname;
		gpiomap_on_off[index].Onoff_value = buf[0];
		pr_debug("%s ,  ret=%d, value=0x%x\n", __func__, ret, buf[0]);
	}

done:
	return;
}

int pm8150a_gpio_show(struct seq_file *m, void *data, size_t num)
{
	int i = 0;
	struct gpio_property *gpiomap_on_off = (struct gpio_property *)data;

	seq_printf(m, "+--+------------+--PM8150A gpio_show----------+---\n");

	for (i = 0; i < num / 3; i++) {
		seq_printf(m, "|%-13s| value=0x%02x | %-7s | %-10s\n", gpiomap_on_off[i].name,
				   gpiomap_on_off[i].Onoff_value & 0xFF,
				   (gpiomap_on_off[i].Onoff_value >> 7) == 1 ? "enable" : "disable",
				   (gpiomap_on_off[i].Onoff_value & 0x01) == 1 ? "input_high" : "input_low"
				   );
	}

	seq_printf(m, "+--+-----------+-----+----+---+-----------+----+--\n");

	for (i = num / 3; i < (num/3)*2; i++) {
		seq_printf(m, "|%-13s| invert=%d | output_source=0x%02x\n", gpiomap_on_off[i].name,
				gpiomap_on_off[i].Onoff_value >> 7,(gpiomap_on_off[i].Onoff_value & 0x0f));
	}

	seq_printf(m, "+--+-----------+-----+----+---+-----------+----+--\n");

	for (i = (num/3)*2; i < num; i++) {
		seq_printf(m, "|%-13s| GPIO_MODE=0x%02x\n", gpiomap_on_off[i].name,
				gpiomap_on_off[i].Onoff_value & 0x3);
	}

	seq_printf(m, "+--+-----------+-----+----+---+-----------+----+--\n");

	return 0;
}


static int pm8150_ldo_show(struct seq_file *m, void *data, size_t num)
{
	int i = 0;
	struct ldo_property *ldo_on_off = (struct ldo_property *)data;

	seq_printf(m, "+--+------+-pm8150 ldo_show: On/Off--+---+--------\n");

	for (i = 0; i < num; i++) {
		seq_printf(m, "|%-15s| on_off=0x%x\n", ldo_on_off[i].name,
			   (ldo_on_off[i].Onoff_value >> 7) & 0x01);
	}

	seq_printf(m, "+--+-------+-----+----+---+-----------+----+------\n");
	return 0;
}

static void pm8150_ldo_store(struct device *ssbi_dev, void *data, size_t num)
{
	int index;
	uint8_t sid;
	uint8_t buf[1];
	uint16_t addr;
	int8_t ret;

	struct ldo_property *ldo_on_off = (struct ldo_property *)data;

	for (index = 0;
		 index < (sizeof(pm8150_reg_ldo_table) / sizeof(struct reg_property));
		 index++) {
		sid = (pm8150_reg_ldo_table[index].regaddr >> 16) & 0xF;
		addr = pm8150_reg_ldo_table[index].regaddr & 0xFFFF;

		ret = read_pmic_data(sid, addr, buf, 1);
		if (ret < 0) {
			pr_err("SPMI read failed, err = %d\n", ret);
			goto done;
		}
		ldo_on_off[index].name = pm8150_reg_ldo_table[index].regname;
		ldo_on_off[index].Onoff_value = buf[0];
		pr_debug("%s ,  ret=%d, value=0x%x\n", __func__, ret, buf[0]);
	}

done:
	return;

}

static void pm8150_gpio_store(struct device *dev, void *data, size_t num)
{
	int index;
	uint8_t sid;
	uint8_t buf[1];
	uint16_t addr;
	int8_t ret;

	struct gpio_property *gpiomap_on_off = (struct gpio_property *)data;
	for (index = 0;
			index < (sizeof(pm8150_reg_gpio_table)/sizeof(struct reg_property)); index++)
	{
		sid = (pm8150_reg_gpio_table[index].regaddr >> 16 &0xF);
		addr = pm8150_reg_gpio_table[index].regaddr & 0xFFFF;

		ret = read_pmic_data(sid,addr,buf,1);
		if (ret < 0)
		{
			pr_err("SPMI read failed,err = %d\n",ret);
			goto done;
		}

		gpiomap_on_off[index].name = pm8150_reg_gpio_table[index].regname;
		gpiomap_on_off[index].Onoff_value = buf[0];
		pr_debug("%s,ret=%d,value=0x%x\n",__func__,ret,buf[0]);
	}

done:
	return;
}

static int pm8150_gpio_show(struct seq_file *m,void *data,size_t num)
{
	int i;
	struct gpio_property *gpiomap_on_off = (struct gpio_property *)data;

	seq_printf(m, "+--+------------+--pm8150 gpio_show----------+---\n");
	for (i = 0; i < num / 3; i++) {
		seq_printf(m, "|%-13s| value=0x%02x | %-7s | %-10s\n", gpiomap_on_off[i].name,
				   gpiomap_on_off[i].Onoff_value & 0xFF,
				   (gpiomap_on_off[i].Onoff_value >> 7) == 1 ? "enable" : "disable",
				   (gpiomap_on_off[i].Onoff_value & 0x01) == 1 ? "input_high" : "input_low"
		);
	}
	seq_printf(m, "+--+-----------+-----+----+---+-----------+----+--\n");

	for (i = num / 3; i < (num/3)*2; i++) {
		seq_printf(m, "|%-13s| invert=%d | output_source=0x%02x\n", gpiomap_on_off[i].name,
				gpiomap_on_off[i].Onoff_value >> 7,(gpiomap_on_off[i].Onoff_value & 0x0f));
	}

	seq_printf(m, "+--+-----------+-----+----+---+-----------+----+--\n");

	for (i = (num/3)*2; i < num; i++) {
		seq_printf(m, "|%-13s| GPIO_MODE=0x%02x\n", gpiomap_on_off[i].name,
				gpiomap_on_off[i].Onoff_value & 0x3);
	}

	seq_printf(m, "+--+-----------+-----+----+---+-----------+----+--\n");
	return 0;
}

static int dump_current(struct seq_file *m,void *unused)
{
	struct dump_desc *dump_device = (struct dump_desc *)m->private;
	void *data;

	data = kcalloc(dump_device->item_count,dump_device->item_size,GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	dump_device->store(dump_device->dev,data,dump_device->item_count);
	dump_device->show(m,data,dump_device->item_count);
	kfree(data);
	return 0;
}

static int current_open(struct inode *inode, struct file *file)
{
#ifdef USE_DEBUGFS
	return single_open(file, dump_current, inode->i_private);
#else
	return single_open(file, dump_current, PDE_DATA(inode));
#endif
}

static struct file_operations current_fops = {
	.open = current_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};


static int dump_sleep(struct seq_file *m,void *unused)
{
	struct dump_desc *dump_device = (struct dump_desc *)m->private;
	void *data;

	pr_debug("%s dump_sleep,sleep_saved=%d, sleep_data=%p\n",__func__,sleep_saved,
			dump_device->sleep_data);

	if (sleep_saved && dump_device->sleep_data)
	{
		data = dump_device->sleep_data;
		dump_device->show(m,data,dump_device->item_count);
		return 0;
	}else
	{
		seq_printf(m,"not recorded\n");
		return 0;
	}
}



static int sleep_open(struct inode *inode, struct file *file)
{
#ifdef USE_DEBUGFS
	return single_open(file, dump_sleep, inode->i_private);
#else
	return single_open(file, dump_sleep, PDE_DATA(inode));
#endif
}


static const struct file_operations sleep_fops = {
	.open = sleep_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

#ifdef USE_DEBUGFS
static int populate(struct dentry *base,const char *dir,
							struct dump_desc *dump_device)
{
	struct dentry *local_base;
	local_base = debugfs_create_dir(dir,base);
	if (!local_base)
		return -ENOMEM;

	if(!debugfs_create_file("current",0444,local_base,(void *)dump_device,&current_fops))
		return -ENOMEM;

	if(!debugfs_create_file("sleep",0444,local_base,(void *)dump_device,&sleep_fops))
		return -ENOMEM;

	return 0;
}
#else
/*
 *@	procfs method
 * */
static int populate(struct proc_dir_entry *base,const char *dir,
							struct dump_desc *dump_device)
{
	struct proc_dir_entry *local_base;
	local_base = proc_mkdir(dir,base);
	if (!local_base)
		return -ENOMEM;

	if(!proc_create_data("current",0444,local_base, &current_fops, (void *)dump_device))
		return -ENOMEM;

	if(!proc_create_data("sleep",0444,local_base, &sleep_fops, (void *)dump_device))
		return -ENOMEM;

	return 0;
}
#endif
static int enable_set(void *data,u64 val)
{
	int i;
	debug_mask = (u32)val;
	sleep_saved = false;

	for (i = 0; i < DUMP_DEV_NUM; i++)
	{
		struct dump_desc *dump_device = &dump_devices[i];
		if (val)
		{
			if (dump_device->sleep_data)
				continue;
			dump_device->sleep_data = kcalloc(dump_device->item_count,
					dump_device->item_size,GFP_KERNEL);
			if (!dump_device->sleep_data)
				return -ENOMEM;
		}else {
			kfree(dump_device->sleep_data);
			dump_device->sleep_data = NULL;
		}
	}

	return 0;
}

static int enable_get(void *data,u64 *val)
{
	*val = (u64)debug_mask;
	sleep_saved = false;
	return 0;
}

static int fault_reason_get(void *data,u64 *val)
{
	*val = get_poff_fault_reason();
	return 0;
}
static int pon_reason_get(void *data,u64 *val)
{
	*val = get_boot_reason();
	return 0;
}
static int on_reason_get(void *data,u64 *val)
{
	*val = get_on_reason();
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(enable_fops, enable_get, enable_set, "0x%08llx\n");
DEFINE_SIMPLE_ATTRIBUTE(fault_reason_fops, fault_reason_get, NULL, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(pon_reason_fops, pon_reason_get, NULL, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(on_reason_fops, on_reason_get, NULL, "0x%llx\n");

static int __init power_debug_init(void)
{
	int ret = 0;
	int i;

#ifdef USE_DEBUGFS
	debugfs = debugfs_create_dir("power_debug",NULL);
	if (!debugfs)
	{
		pr_err("can't create the debugfs dir power_debug\n");
		return -ENOMEM;
	}

	if (!debugfs_create_file("enable",0644,debugfs,NULL,&enable_fops)){
		ret = -ENOMEM;
		goto fail;
	}

	if (!debugfs_create_file("fault_reason",0644,debugfs,NULL,&fault_reason_fops)){
		ret = -ENOMEM;
		goto fail;
	}

	if (!debugfs_create_file("pon_reason",0644,debugfs,NULL,&pon_reason_fops)){
		ret = -ENOMEM;
		goto fail;
	}

	if (!debugfs_create_file("on_reason",0644,debugfs,NULL,&on_reason_fops)){
		ret = -ENOMEM;
		goto fail;
	}
#else
	power_debug_proc_entry = proc_mkdir("power_debug", NULL);
	if (!power_debug_proc_entry){
		printk("power_debug failed to mkdir for procfs\n");
		return -1;
	}

	if(! proc_create("enable", 0644, power_debug_proc_entry, &enable_fops)) {
		printk("power_debug failed to create enable for procfs\n");
		remove_proc_entry("power_debug", NULL);
		return -1;
	}

	if(! proc_create("fault_reason", 0644, power_debug_proc_entry, &fault_reason_fops)) {
		printk("power_debug failed to create fault_reason for procfs\n");
		remove_proc_entry("power_debug", NULL);
		return -1;
	}

	if(! proc_create("pon_reason", 0644, power_debug_proc_entry, &pon_reason_fops)) {
		printk("power_debug failed to create pon_reason for procfs\n");
		remove_proc_entry("power_debug", NULL);
		return -1;
	}

	if(! proc_create("on_reason", 0644, power_debug_proc_entry, &on_reason_fops)) {
		printk("power_debug failed to create on_reason for procfs\n");
		remove_proc_entry("power_debug", NULL);
		return -1;
	}
#endif
	for (i = 0; i < DUMP_DEV_NUM; i++)
	{
		struct dump_desc *dump_device = &dump_devices[i];
#ifdef USE_DEBUGFS
		ret = populate(debugfs,dump_device->name,dump_device);
#else
		ret = populate(power_debug_proc_entry,dump_device->name,dump_device);
#endif
		if (ret)
			goto fail;
	}

	pr_debug("power_debug_init OK\n");
	return 0;

fail:
#ifdef USE_DEBUGFS
	debugfs_remove_recursive(debugfs);
#else
	remove_proc_entry("power_debug", NULL);
#endif
	pr_err("power_debug_init failed\n");
	return ret;
}


void power_debug_collapse(void)
{
	int i;
	if (debug_mask) {
		pr_debug("%s save sleep state\n", __func__);
		for (i = 0; i < DUMP_DEV_NUM; i++) {
			struct dump_desc *dump_device = &dump_devices[i];
			if (dump_device->sleep_data)
				dump_device->store(dump_device->dev,
				dump_device->sleep_data,
				dump_device->item_count);
		}
		sleep_saved = true;
	}
}

EXPORT_SYMBOL(power_debug_collapse);

late_initcall(power_debug_init);
