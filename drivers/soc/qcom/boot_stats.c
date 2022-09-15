// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2013-2018, The Linux Foundation. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <soc/qcom/boot_stats.h>

struct boot_stats {
	uint32_t bootloader_start;
	uint32_t bootloader_end;
	uint32_t bootloader_display;
	uint32_t bootloader_load_kernel;
};

static void __iomem *mpm_counter_base;
static uint32_t mpm_counter_freq;
static struct boot_stats __iomem *boot_stats;

struct boot_shared_imem_cookie_type __iomem *boot_imem = NULL;
//PicoBSP: Alex_ma add for offlinedump -->start
extern char * log_first_idx_get(void);
extern char * log_next_idx_get(void);
//PicoBSP: Alex_ma add for offlinedump -->end
static int mpm_parse_dt(void)
{
	struct device_node *np_imem, *np_mpm2;

	np_imem = of_find_compatible_node(NULL, NULL,
				"qcom,msm-imem-boot_stats");
	if (!np_imem) {
		pr_err("can't find qcom,msm-imem node\n");
		return -ENODEV;
	}
	boot_stats = of_iomap(np_imem, 0);
	if (!boot_stats) {
		pr_err("boot_stats: Can't map imem\n");
		goto err1;
	}

	np_mpm2 = of_find_compatible_node(NULL, NULL,
				"qcom,mpm2-sleep-counter");
	if (!np_mpm2) {
		pr_err("mpm_counter: can't find DT node\n");
		goto err1;
	}

	if (of_property_read_u32(np_mpm2, "clock-frequency", &mpm_counter_freq))
		goto err2;

	if (of_get_address(np_mpm2, 0, NULL, NULL)) {
		mpm_counter_base = of_iomap(np_mpm2, 0);
		if (!mpm_counter_base) {
			pr_err("mpm_counter: cant map counter base\n");
			goto err2;
		}
	} else
		goto err2;

	return 0;

err2:
	of_node_put(np_mpm2);
err1:
	of_node_put(np_imem);
	return -ENODEV;
}
uint64_t get_boot_reason(void)
{
	if (boot_imem == NULL)
		boot_stats_init();
	return (uint64_t)boot_imem->pon_reason;
}

uint64_t get_on_reason(void)
{
	if (boot_imem == NULL)
		boot_stats_init();
	return (uint64_t)boot_imem->on_reason;
}

uint64_t get_poff_fault_reason(void)
{
	if (boot_imem == NULL)
		boot_stats_init();
	return (uint64_t)boot_imem->fault_reason;
}

uint32_t get_secure_boot_value(void)
{
	if (boot_imem == NULL)
		boot_stats_init();

	return (uint32_t)boot_imem->is_enable_secure_boot;
}

uint32_t get_lpddr_vendor_id(void)
{
	if (boot_imem == NULL)
		boot_stats_init();

	return (uint32_t)boot_imem->lpddr_vendor_id;
}

uint32_t get_lpddr_device_type(void)
{
	if (boot_imem == NULL)
		boot_stats_init();

	printk(KERN_CRIT"DUMP: show lpddr_device_type %d\n", boot_imem->lpddr_device_type);
	return (uint32_t)boot_imem->lpddr_device_type;
}

uint32_t get_lpddr_capacity(void)
{
	if (boot_imem == NULL)
		boot_stats_init();

	printk(KERN_CRIT"DUMP: show lpddr_capacity %d\n", boot_imem->lpddr_capacity);
	return (uint32_t)boot_imem->lpddr_capacity;
}

uint32_t get_slot_change_status(void)
{
	if (boot_imem == NULL)
		boot_stats_init();

	return (uint32_t)boot_imem->slot_change;
}

uint32_t get_lcd_magic(void)
{
	if (boot_imem == NULL)
		boot_stats_init();

	return (uint32_t)boot_imem->lcd_magic;
}

/* Pico-BSP: supported to command force dump */
uint32_t get_force_dump_type(void)
{
	if (boot_imem == NULL)
		boot_stats_init();

	printk(KERN_CRIT"DUMP: show force dump status %d\n", boot_imem->force_dump_enable);
	return (uint32_t)boot_imem->force_dump_enable;
}

uint32_t get_hw_version(void)
{
	if (boot_imem == NULL)
		boot_stats_init();

	return (uint32_t)boot_imem->hw_version;
}

static void print_boot_stats(void)
{
	pr_info("KPI: Bootloader start count = %u\n",
		readl_relaxed(&boot_stats->bootloader_start));
	pr_info("KPI: Bootloader end count = %u\n",
		readl_relaxed(&boot_stats->bootloader_end));
	pr_info("KPI: Bootloader display count = %u\n",
		readl_relaxed(&boot_stats->bootloader_display));
	pr_info("KPI: Bootloader load kernel count = %u\n",
		readl_relaxed(&boot_stats->bootloader_load_kernel));
	pr_info("KPI: Kernel MPM timestamp = %u\n",
		readl_relaxed(mpm_counter_base));
	pr_info("KPI: Kernel MPM Clock frequency = %u\n",
		mpm_counter_freq);

	printk(KERN_CRIT"KPI: kernel_log_buf_addr = 0x%llx , offset %lu ,0x%llx\n",
		boot_imem->kernel_log_buf_addr,offsetof(struct boot_shared_imem_cookie_type,kernel_log_buf_addr),(uint64_t)virt_to_phys((uint64_t *)0xffffff800a7c0510));
	printk(KERN_CRIT"KPI: log_first_idx = 0x%llx , offset %lu\n",
		(boot_imem->log_first_idx_addr),offsetof(struct boot_shared_imem_cookie_type,log_first_idx_addr));
	printk(KERN_CRIT"KPI: log_next_idx = 0x%llx , offset %lu \n",
		boot_imem->log_next_idx_addr,offsetof(struct boot_shared_imem_cookie_type,log_next_idx_addr));
//PicoBSP: Alex_ma add for offlinedump -->start
	printk(KERN_CRIT"KPI: --------offline_dump_enable = 0x%x\n",
		(uint32_t)boot_imem->offline_dump_enable);
	printk(KERN_CRIT"KPI: pon_reason = 0x%llx\n",
//PicoBSP: Alex_ma add for offlinedump -->end
		(uint64_t)boot_imem->pon_reason);
	printk(KERN_CRIT"KPI: on_reason = 0x%llx\n",
		(uint64_t)boot_imem->on_reason);
	printk(KERN_CRIT"KPI: fault_reason = 0x%llx\n",
		(uint64_t)boot_imem->fault_reason);
	printk(KERN_CRIT"KPI: is_enable_secure_boot = 0x%x\n",
		(uint32_t)boot_imem->is_enable_secure_boot);
	printk(KERN_CRIT"KPI: lpddr_vendor_id = 0x%x\n",
		(uint32_t)boot_imem->lpddr_vendor_id);
	//PICO-bsp: Hezh add for save uefilog current index.
	printk(KERN_CRIT"KPI: boot_index = 0x%x\n",
		(uint32_t)boot_imem->bootcycle);
}


ssize_t show_offline_dump_enable(struct kobject *kobj, struct attribute *attr,char *buf)
{
	uint32_t show_val;

	show_val = boot_imem->offline_dump_enable;
	printk(KERN_CRIT"DUMP: show_offline_dump_enable %d\n", show_val);

	return snprintf(buf, PAGE_SIZE, "%u\n", show_val);
}
//PicoBSP: Alex_ma add for offlinedump -->start
size_t store_offline_dump_enable(struct kobject *kobj, struct attribute *attr,const char *buf, size_t count)
{
	uint32_t enabled;
	int ret;

	ret = kstrtouint(buf, 0, &enabled);
	if (ret < 0)
		return ret;
	printk(KERN_CRIT"DUMP: store_offline_dump, enabled %d[0x%x]\n", enabled, enabled);

	if (!((enabled == 0x3721) || (enabled == 0) || (enabled == 1) || (enabled == 2)))
		return -EINVAL;

	boot_imem->offline_dump_enable = enabled;

	return count;
}

ssize_t show_offline_dump_happen(struct kobject *kobj, struct attribute *attr,char *buf)
{
	uint32_t show_val;

	show_val = (boot_imem->offline_dump_happen == 1) ? 1 : 0;
	printk(KERN_CRIT"KPI: show_offline_dump_happen  %d\n",show_val);

	return snprintf(buf, PAGE_SIZE, "%u\n", show_val);
}

#define PI0			"3.14159260"
#define PI1			"3.14159261"

size_t store_offline_dump_happen(struct kobject *kobj, struct attribute *attr,const char *buf, size_t count)
{
	if ( (count >= strlen(PI0)) && (0 == strncmp(buf, PI0, strlen(PI0))) ) {
		printk(KERN_CRIT"RAMDUMP: happen <%d>\n", 0);
		boot_imem->offline_dump_happen = 0;
	}
	else if ( (count >= strlen(PI1)) && (0 == strncmp(buf, PI1, strlen(PI1))) ) {
		printk(KERN_CRIT"RAMDUMP: happen <%d>\n", 1);
		boot_imem->offline_dump_happen = 1;
	}

	return count;
}
//PicoBSP: Alex_ma add for offlinedump -->end
int boot_stats_init(void)
{
	int ret;
	struct device_node *np;

	ret = mpm_parse_dt();
	if (ret < 0)
		return -ENODEV;

	np = of_find_compatible_node(NULL, NULL, "qcom,msm-imem");
	if (!np) {
		pr_err("can't find qcom,msm-imem node\n");
		return -ENODEV;
	}

	boot_imem = of_iomap(np, 0);

	boot_imem->kernel_log_buf_addr = virt_to_phys(log_buf_addr_get());
	boot_imem->log_first_idx_addr = virt_to_phys(log_first_idx_get());
	boot_imem->log_next_idx_addr = virt_to_phys(log_next_idx_get());
	printk(KERN_CRIT"boot_stats_init:  log_first_idx_get() = 0x%llu\n",(u64)log_first_idx_get());
	printk(KERN_CRIT"boot_stats_init:  log_next_idx_get() = 0x%llu\n",(u64)log_next_idx_get());
	printk(KERN_CRIT"boot_stats_init:  slot change = 0x%llu\n",boot_imem->slot_change);

	if(boot_imem->offline_dump_happen==1){
	    printk(KERN_CRIT"KPI:  --------offline_dump_happen = 0x%x\n",
			(uint32_t)boot_imem->offline_dump_happen);
	}

	print_boot_stats();

	iounmap(boot_stats);
	iounmap(mpm_counter_base);

	return 0;
}

