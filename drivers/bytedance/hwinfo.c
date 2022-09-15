// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022, Pico Immersive Pte. Ltd ("Pico"). All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/kobject.h>
#include <soc/qcom/socinfo.h>
#include <soc/qcom/boot_stats.h>
#include <asm-generic/bug.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/cpufreq.h>
#include <asm/system_misc.h>
#include <linux/of_gpio.h>
#include "hwinfo.h"

extern void fts_tp_version_get(char *buf, int buf_size);
static int hwinfo_probe(struct platform_device *pdev);

#define MAX_HWINFO_SIZE 100

#define GPIO_CONFIG_BOARD_ID0 "board-gpio-0"
#define GPIO_CONFIG_BOARD_ID1 "board-gpio-1"

typedef struct {
	char *hwinfo_name;
	char hwinfo_buf[MAX_HWINFO_SIZE];
	int  init_flag;
} hwinfo_t;

struct hwinfo_dev {
	struct platform_driver hwinfo_drv_pf;
};

static struct of_device_id hwinfo_match_table[] = {
	{	.compatible = "bytedance,hwinfo",
	},
};

static struct hwinfo_dev hw_hwinfo_dev = {
	.hwinfo_drv_pf = {
		.probe = hwinfo_probe,
		.driver = {
			.name = "hwinfo_platform",
			.owner = THIS_MODULE,
			.of_match_table = hwinfo_match_table,
		},
	},
};

static int board_id_num = 0;

#define KEYWORD(_name) \
	[_name] = {.hwinfo_name = __stringify(_name), \
		.hwinfo_buf = {0}, \
		.init_flag = 0,},
static hwinfo_t hwinfo[HWINFO_MAX] =
{
#include "hwinfo.h"
};
#undef KEYWORD
typedef struct mid_match {
	int index;
	const char *name;
} mid_match_t;

static int hwinfo_read_file(char *file_name, char buf[], int buf_size)
{
	struct file *fp;
	mm_segment_t fs;
	loff_t pos = 0;
	ssize_t len = 0;

	if (file_name == NULL || buf == NULL)
		return -1;

	fp = filp_open(file_name, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		//printk(KERN_CRIT "file not found\n");
		return -1;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);
	len = vfs_read(fp, buf, buf_size, &pos);
	//printk(KERN_INFO "buf= %s,size = %ld\n", buf, len);
	filp_close(fp, NULL);
	set_fs(fs);
	return 0;
}

#if USE_MEMINFO
extern void si_meminfo(struct sysinfo *val);
static unsigned long long get_mem_total(void)
{
	struct sysinfo meminfo;
	si_meminfo(&meminfo);
	printk("MemTotal: %u\n", meminfo.totalram << (PAGE_SHIFT - 10));
	return meminfo.totalram;
}
#endif

int nfc_support_value = -1;
int ufs2_support_value = -1;
typedef struct product_version {
	int index;
	const char *product_version;
} product_version_t;
typedef struct hw_version {
	int index;
	const char *hw_version;
} hw_version_t;

static hw_version_t hw_version_table[] = {
	{ .index = 0x0,  .hw_version = "B0"	},
	{ .index = 0x1,  .hw_version = "B1"	},
	{ .index = 0x2,  .hw_version = "B2"	},
	{ .index = 0x3,  .hw_version = "B3"	},
	{ .index = 0x4,  .hw_version = "B4"	},
	{ .index = 0x5,  .hw_version = "B5"	},
	{ .index = 0x6,  .hw_version = "B6"	},
	{ .index = 0x7,  .hw_version = "Unknown" },
};

#if 0
#define MEM_4G      4*1024*1024
#define MEM_6G      6*1024*1024
#define MEM_8G      8*1024*1024
#define MEM_12G     12*1024*1024
#define UFS_64G     64
#define UFS_128G    128
#define UFS_256G    256
#define UFS_SIZE_FILE   "/sys/block/sda/size"
static int get_ufs_size(void)
{
	char buf[32] = {0};
	unsigned long long  ufs_size = 0;
	int ret = 0;
	ret = hwinfo_read_file(UFS_SIZE_FILE, buf, sizeof(buf));
	if (ret != 0) {
		printk(KERN_ERR "hwinfo:no found %s", UFS_SIZE_FILE);
	} else {
		sscanf(buf, "%llu", &ufs_size);
		ret = ufs_size >> 21;
		printk(KERN_INFO "%s(): ufs-size:%llu: size: %d\n", __func__, ufs_size, ret);
	}
	return ret;
}
#endif

static const char * get_board_id_name(void)
{
	char *boardid_name = "Unknown";
	int i = 0;

	for (i = 0; i < sizeof(hw_version_table)/sizeof(hw_version_t); i++) {
		if (hw_version_table[i].index == board_id_num){
			return hw_version_table[i].hw_version;
		}
	}

	return boardid_name;
}

static int get_version_id(void)
{
 	const char *board_id_name = NULL;

	board_id_num = get_hw_version();

	board_id_name = get_board_id_name();

	return sprintf(hwinfo[version_id].hwinfo_buf, "%s:0x%x", board_id_name, board_id_num);
}

#if USE_EMMC
static mid_match_t emmc_table[] = {
{
	.index = 17,
	.name = "Toshiba"
},
{
	.index = 19,
	.name = "Micron"
},
{
	.index = 69,
	.name = "Sandisk"
},
{
	.index = 21,
	.name = "Samsung"
},
{
	.index = 0x90,
	.name = "Hynix"
},
/*UFS*/
{
	.index = 0xCE,
	.name = "Samsung"
},
{
	.index = 0xAD,
	.name = "Hynix"
},
{
	.index = 0x98,
	.name = "Toshiba"
},
};
static const char *foreach_emmc_table(int index)
{
int i = 0;
for (; i < sizeof(emmc_table) / sizeof(mid_match_t); i++) {
	if (index == emmc_table[i].index)
		return emmc_table[i].name;
}
return NULL;
}
#define EMMC_MAX_NUM     1
#define EMMC_SIZE_FILE   "/sys/block/mmcblkX/size"
#define EMMC_MANFID_FILE "/sys/block/mmcblkX/device/manfid"
static void get_emmc_info(void)
{
int i = 0;
char buf[30] = {0};
int ret = 0;
char file_name[35] = {0};
unsigned long long  emmc_size = 0;
const char *emmc_mid_name;

if (hwinfo[emmc_capacity].init_flag == 1 || hwinfo[emmc_manufacturer].init_flag == 1)
	return;

hwinfo[emmc_capacity].init_flag = 1;
hwinfo[emmc_manufacturer].init_flag = 1;
memset(hwinfo[emmc_capacity].hwinfo_buf, 0, MAX_HWINFO_SIZE);
memset(hwinfo[emmc_manufacturer].hwinfo_buf, 0, MAX_HWINFO_SIZE);

for (i = 0; i < EMMC_MAX_NUM; i++) {
	/*read emmc emmc_capacity*/
	memset(file_name, 0, sizeof(file_name));
	strcpy(file_name, EMMC_SIZE_FILE);
	file_name[strstr(file_name, "X") - file_name] = i + '0';
	//	  printk(KERN_INFO "EMMC%d: file_name: %s\n", i, file_name);
	ret = hwinfo_read_file(file_name, buf, sizeof(buf));
	if (ret != 0) {
		printk(KERN_CRIT "hwinfo:no found %s", file_name);
		return;
	}
	//printk(KERN_INFO "hwinfo:EMMC%d: size: %s\n", i, buf);
	sscanf(buf, "%llu", &emmc_size);
	sprintf(hwinfo[emmc_capacity].hwinfo_buf + strlen(hwinfo[emmc_capacity].hwinfo_buf),
			"%lluGB", emmc_size >> 21);

	/*read emmc manufacturer*/
	memset(file_name, 0, sizeof(file_name));
	strcpy(file_name, EMMC_MANFID_FILE);
	file_name[strstr(file_name, "X") - file_name] = i + '0';
	//printk(KERN_INFO "EMMC%d: file_name: %s\n", i, file_name);

	ret = hwinfo_read_file(file_name, buf, sizeof(buf));
	if (ret != 0) {
		printk(KERN_CRIT "hwinfo:no found %s", file_name);
		return;
	}
	//	  printk(KERN_INFO "\n hwinfo:EMMC%d: manufacturer: %s\n", i, buf);
	emmc_mid_name = foreach_emmc_table((int) simple_strtol(buf, NULL, 16));
	WARN((emmc_mid_name == NULL), "cannot recognize emmc mid\n");
	if (emmc_mid_name == NULL)
		emmc_mid_name = "Unknown";

	sprintf(hwinfo[emmc_manufacturer].hwinfo_buf + strlen(hwinfo[emmc_manufacturer].hwinfo_buf),
			"%s", emmc_mid_name);
}
return;
}
#endif

static mid_match_t lpddr_table[] = {
{
	.index = 0xff,
	.name = "Micron"
},
{
	.index = 6,
	.name = "Hynix"
},
{
	.index = 1,
	.name = "Samsung"
},
{
	.index = 3,
	.name = "Micron"
},
};
static const char *foreach_lpddr_table(int index)
{
int i = 0;
for(; i<sizeof(lpddr_table)/sizeof(mid_match_t); i++) {
	if (index == lpddr_table[i].index)
		return lpddr_table[i].name;
}
return NULL;
}

#if USE_LCD
#define LCD_INFO_FILE "/sys/class/graphics/fb0/msm_fb_panel_info"
static int get_lcd_type(void)
{
char buf[200] = {0};
int  ret = 0;
char *p1 =  NULL, *p2 = NULL;
if (hwinfo[LCD_MFR].init_flag == 1 || hwinfo[TP_MFR].init_flag == 1)
	return 0;
hwinfo[LCD_MFR].init_flag = 1;
#if USE_TP
hwinfo[TP_MFR].init_flag = 1;
#endif
memset(hwinfo[LCD_MFR].hwinfo_buf, 0, MAX_HWINFO_SIZE);
ret = hwinfo_read_file(LCD_INFO_FILE, buf, sizeof(buf));
if (ret != 0)
{
	printk(KERN_CRIT "hwinfo:get lcd_type read file failed.\n");
	return -1;
}
p1 = strstr(buf, "panel_name=");
if (p1 == NULL)
{
	printk(KERN_CRIT "hwinfo:no found panel_name.\n");
	return -1;
}
p2 = strstr(p1, " ");
if (p2 == NULL)
{
	printk(KERN_CRIT "hwinfo:get lcd panel_name failed.\n");
	return -1;
}
memcpy(hwinfo[LCD_MFR].hwinfo_buf, p1 + strlen("panel_name="), abs(p2 - p1) - strlen("panel_name"));
return 0;
}
#endif

#if USE_TP_VERSION
static int get_tp_version(void)
{
char buf[200] = {0};
int fw_version = 0;
printk("%s(): enter!\n", __func__);
hwinfo[tp_version].init_flag = 1;
memset(hwinfo[tp_version].hwinfo_buf, 0, MAX_HWINFO_SIZE);
fts_tp_version_get(buf, sizeof(buf));
sscanf(buf, "%x", &fw_version);
sprintf(hwinfo[tp_version].hwinfo_buf + strlen(hwinfo[tp_version].hwinfo_buf),
		"0x%x", fw_version);
return 0;
}
#endif

#define BATTARY_TYPE_FILE       "/sys/class/power_supply/bms/battery_type"
static int get_battary_type(void)
{
char buf[64] = {0};
char tmp[64] = {0};
int ret = 0;

ret = hwinfo_read_file(BATTARY_TYPE_FILE, buf, sizeof(buf));
if (ret != 0)
{
	printk(KERN_CRIT "get_battary_type failed.");
	return -1;
}

sscanf(buf, "%s", tmp);
strcpy(hwinfo[battery_type].hwinfo_buf, tmp);
return 0;
}

#if USE_UFS
#define UFS_SIZE_FILE   "/sys/block/sda/size"
typedef struct hw_info {
int index;
char *file_name;
} hw_info_t;
hw_info_t hw_info_arrary[] =
{
{ .index = ufs_capacity      , .file_name = "/sys/block/sda/size" },
{ .index = ufs_manufacturer  , .file_name = "/sys/block/sda/device/vendor" },
{ .index = ufs_version       , .file_name = "/sys/block/sda/device/rev" },
{ .index = ufs_model         , .file_name = "/proc/scsi/scsi" },
#if USE_UFS2
{ .index = ufs1_capacity     , .file_name = "/sys/block/sua/size" },
{ .index = ufs1_manufacturer , .file_name = "/sys/block/sua/device/vendor" },
{ .index = ufs1_version      , .file_name = "/sys/block/sua/device/rev" },
#endif
};
static void get_ufs_info(void)
{
char buf[256] = {0};
int ret = 0;
unsigned long long  ufs_size = 0;
char *ptr = NULL;
int i = 0;
if (hwinfo[ufs_capacity].init_flag == 1)
	return;
for (i = 0; i < sizeof(hw_info_arrary) / sizeof(hw_info_t); i++)
{
	memset(buf, 0, sizeof(buf));
	ret = hwinfo_read_file(hw_info_arrary[i].file_name, buf, sizeof(buf));
	if (ret != 0)
	{
		printk(KERN_CRIT "hwinfo:no found %s", UFS_SIZE_FILE);
		continue;
	}
	switch (hw_info_arrary[i].index)
	{
	case ufs_model:
		hwinfo[ufs_model].hwinfo_buf[0] = '\0';
		ptr = strnstr(buf, "Model:", sizeof(buf));
		if (ptr) {
			int idx = 0;
			char *from = ptr + strlen("Model:");
			char *toto = hwinfo[ufs_model].hwinfo_buf;

			while ((*from == ' ') || (*from == '\t')) { from++; }
			for (idx = 1; idx < MAX_HWINFO_SIZE; idx++) {
				if ((*from == ' ') || (*from == '\t'))
					break;
				*toto++ = *from++;
			}
			*toto = '\0';
		}
		break;
	case ufs_capacity:
#if USE_UFS2
	case ufs1_capacity:
#endif
		sscanf(buf, "%llu", &ufs_size);
		sprintf(hwinfo[hw_info_arrary[i].index].hwinfo_buf, "%lluGB", ufs_size >> 21);
		printk("%s() UFS CAPACITY:%s\n", __func__, hwinfo[hw_info_arrary[i].index].hwinfo_buf);
		break;
	default:
		for (ptr = buf; (ptr - buf) < sizeof(buf) && *ptr != '\n'; ptr++);
		if (*ptr == '\n')
			*ptr = '\0';
		printk(KERN_INFO "\n manufacturer: %s\n", buf);
		sprintf(hwinfo[hw_info_arrary[i].index].hwinfo_buf, "%s", buf);
		break;
	}
	hwinfo[ufs_capacity].init_flag = 1;
}
return;
}
#if USE_UFS2
int get_ufs2_support(void)
{
if (ufs2_support_value == -1)
	get_version_id();
return ufs2_support_value;
}
EXPORT_SYMBOL(get_ufs2_support);
#endif
#endif

static ssize_t hwinfo_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
int i = 0;
//    printk(KERN_INFO "hwinfo sys node %s \n", attr->attr.name);
for (; i < HWINFO_MAX && strcmp(hwinfo[i].hwinfo_name, attr->attr.name) && ++i;);
switch (i)
{
#if USE_LCD
case LCD_MFR:
#if USE_TP
case TP_MFR:
#endif
	get_lcd_type();
	i = LCD_MFR;
	break;
#endif	//USE_LCD
#if USE_TP_VERSION
case tp_version:
	get_tp_version();
	break;
#endif
#if USE_UFS
case ufs_capacity:
case ufs_manufacturer:
case ufs_version:
case ufs_model:
#if USE_UFS2
case ufs1_capacity:
case ufs1_manufacturer:
case ufs1_version:
#endif
	get_ufs_info();
	break;
#endif
#if USE_EMMC
case emmc_manufacturer:
case emmc_capacity:
	get_emmc_info();
	break;
#endif
case housing_color:
	break;

#if USE_PRODUCT_VERSION
case version_id:
	get_version_id();
	break;
case battery_type:
	get_battary_type();
	break;
#endif
default:
	break;
}
return sprintf(buf, "%s=%s \n", attr->attr.name, ((i >= HWINFO_MAX || hwinfo[i].hwinfo_buf[0] == '\0') ? "unknown" : hwinfo[i].hwinfo_buf));
}
static ssize_t hwinfo_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

#define KEYWORD(_name) \
	static struct kobj_attribute hwinfo##_name##_attr = {   \
		.attr   = {                             \
			.name = __stringify(_name),     \
			.mode = 0444,                   \
		},                                      \
		.show   = hwinfo_show,                 \
		.store  = hwinfo_store,                \
	};
#include "hwinfo.h"
#undef KEYWORD
#define KEYWORD(_name)\
	[_name] = &hwinfo##_name##_attr.attr,
static struct attribute * g[] = {
#include "hwinfo.h"
	NULL
};
#undef KEYWORD
static struct attribute_group attr_group = {
	.attrs = g,
};

static int get_pon_reason(void)
{
	char *pon_reason_info = NULL;
	switch((get_boot_reason() & 0xFF))
	{
		case 0x40:
			pon_reason_info = "usb charger";
			break;
		case 0x01:
			pon_reason_info = "soft reboot";
			break;
		case 0x41:
			pon_reason_info = "usb + reboot";
			break;
		case 0x80:
			pon_reason_info = "power key";
			break;
		case 0x81:
			pon_reason_info = "hard reset";
			break;
		default:
			pon_reason_info = "unknow";
			break;
	}
	return sprintf(hwinfo[pon_reason].hwinfo_buf, "%s", pon_reason_info);
}
static int get_secure_boot_version(void)
{
	char *is_secureboot = NULL;
        is_secureboot = "NSE";
	if (get_secure_boot_value())
		is_secureboot = "SE";
	else
		is_secureboot = "NSE";
	return sprintf(hwinfo[secboot_version].hwinfo_buf, "%s", is_secureboot);
}

#if USE_NFC
int get_nfc_support(void)
{
#if USE_PRODUCT_VERSION
	if ( nfc_support_value == -1 )
		get_version_id();
#endif
	return nfc_support_value;
}
EXPORT_SYMBOL(get_nfc_support);
#endif

static int set_emmc_sn(char *src)
{
	if (src == NULL)
		return 0;
	sprintf(hwinfo[emmc_sn].hwinfo_buf, "%s", src);
	return 1;
}
__setup("androidboot.serialno=", set_emmc_sn);
static int set_qchip_id(char *src)
{
	if (src == NULL)
		return 0;
	sprintf(hwinfo[qchip_id].hwinfo_buf, "%s", src);
	return 1;
}
__setup("qchip.id=", set_qchip_id);
static int set_housing_color(char *src)
{
	if (src == NULL)
		return 0;
	sprintf(hwinfo[housing_color].hwinfo_buf, "%s", src);
	return 1;
}
__setup("housing.color=", set_housing_color);
int bytedance_hwinfo_register(enum HWINFO_E e_hwinfo, char *hwinfo_name)
{
	if ((e_hwinfo >= HWINFO_MAX) || (hwinfo_name == NULL))
		return -1;
	strncpy(hwinfo[e_hwinfo].hwinfo_buf, hwinfo_name, \
	        (strlen(hwinfo_name) >= 20 ? 19 : strlen(hwinfo_name)));
	return 0;
}
EXPORT_SYMBOL(bytedance_hwinfo_register);
int hwinfo_probe(struct platform_device *pdev)
{
	int err = 0;
	// int temp_val = 0;
	// int gpio = 0;

	// /*Board-0*/
	// temp_val = of_get_named_gpio(pdev->dev.of_node, GPIO_CONFIG_BOARD_ID0, 0);
	// if (!gpio_is_valid(temp_val)) {
	// 	printk("Unable to read hwinfo board 0 gpio\n");
	// 	err = temp_val;
	// 	return err;
	// } else {
	// 	gpio = temp_val;

	// }
	// board_info[0].gpio = gpio;
	// printk("hwinfo board_info 0  gpio=%d \n", board_info[0].gpio);

	// /*Board-1*/
	// temp_val = of_get_named_gpio(pdev->dev.of_node, GPIO_CONFIG_BOARD_ID1, 0);
	// if (!gpio_is_valid(temp_val)) {
	// 	printk("Unable to read hwinfo board 0 gpio\n");
	// 	err = temp_val;
	// 	return err;
	// } else {
	// 	gpio = temp_val;
	// }

	// board_info[1].gpio = gpio;
	// printk("hwinfo board_info 1  gpio=%d \n", board_info[1].gpio);
	return err;
}

static int __init hwinfo_init(void)
{
	struct kobject *k_hwinfo = NULL;
	const char *lpddr_mid_name = NULL;
	uint32_t lpddr_vender_id = 0;
	uint32_t _lpddr_device_type = 0;
	uint32_t _lpddr_capacity = 0;

	int err = 0;
	if ( (k_hwinfo = kobject_create_and_add("hwinfo", NULL)) == NULL ) {
		printk(KERN_CRIT "%s:hwinfo sys node create error \n", __func__);
	}
	if ( sysfs_create_group(k_hwinfo, &attr_group) ) {
		printk(KERN_CRIT "%s:sysfs_create_group failed\n", __func__);
	}

	err = platform_driver_register(&hw_hwinfo_dev.hwinfo_drv_pf);
	if (err) {
		printk("hall_pf_drv_fall regiset error %d\n", err);
		goto hall_pf_drv_fail;
	}

	lpddr_vender_id = get_lpddr_vendor_id();
	lpddr_mid_name = foreach_lpddr_table(lpddr_vender_id);
	WARN((lpddr_mid_name == NULL), "hwinfo:cannot recognize lpddr");
	if (lpddr_mid_name == NULL)
		lpddr_mid_name = "Unknown";

	sprintf(hwinfo[lpddr_manufacturer].hwinfo_buf,"%s", lpddr_mid_name);

	_lpddr_device_type = get_lpddr_device_type();
	sprintf(hwinfo[lpddr_device_type].hwinfo_buf,"%d", _lpddr_device_type);

	_lpddr_capacity = get_lpddr_capacity();
	sprintf(hwinfo[lpddr_capacity].hwinfo_buf,"%ld", _lpddr_capacity);

	get_secure_boot_version();

	get_pon_reason();

	get_version_id();

hall_pf_drv_fail:
	platform_driver_unregister(&hw_hwinfo_dev.hwinfo_drv_pf);
	return err;
}

static void __exit hwinfo_exit(void)
{
	platform_driver_unregister(&hw_hwinfo_dev.hwinfo_drv_pf);
	return ;
}
late_initcall_sync(hwinfo_init);
module_exit(hwinfo_exit);
MODULE_AUTHOR("bsp-system");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Product Hardward Info Exposure");

