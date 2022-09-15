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
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/slab.h>

//Smt: [FEAT_BINDER_GET_TIME] {@
#define BUF_SIZE (3 * PAGE_SIZE)

s64 *g_binder_jiffes_buff;

static int remap_sys_time_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int remap_sys_time_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long offset = 0;
	unsigned long pfn_start = 0;
	unsigned long size = vma->vm_end - vma->vm_start;
	int ret = 0;

	if (PAGE_READONLY.pgprot != vma->vm_page_prot.pgprot) {
		printk("vm page access permissions not match;\n");
		return -EINVAL;
	}

	if (size > PAGE_SIZE) {
		printk("vm request size too larger;\n");
		return -EINVAL;
	}

	offset = vma->vm_pgoff << PAGE_SHIFT;
	pfn_start = (virt_to_phys(g_binder_jiffes_buff) >> PAGE_SHIFT) + vma->vm_pgoff;
	ret = remap_pfn_range(vma, vma->vm_start, pfn_start, size, vma->vm_page_prot);
	if (ret)
		printk("%s: remap_pfn_range failed at [0x%lx  0x%lx]\n",
		       __func__, vma->vm_start, vma->vm_end);

	return ret;
}

static const struct file_operations remap_sys_time_fops = {
	.owner = THIS_MODULE,
	.open = remap_sys_time_open,
	.mmap = remap_sys_time_mmap,
};

static struct miscdevice remap_sys_time_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "binder_get_time",
	.fops = &remap_sys_time_fops,
};

static int __init remap_sys_time_init(void)
{
	int ret = 0;

	g_binder_jiffes_buff = kzalloc(BUF_SIZE, GFP_KERNEL);
	if (!g_binder_jiffes_buff) {
		ret = -ENOMEM;
		goto err;
	}
	ret = misc_register(&remap_sys_time_misc);
	if (unlikely(ret)) {
		pr_err("failed to register misc device!\n");
		goto err;
	}
	return 0;

err:
	return ret;
}

static void __exit remap_sys_time_exit(void)
{
	kfree(g_binder_jiffes_buff);
	misc_deregister(&remap_sys_time_misc);
}

module_init(remap_sys_time_init);
module_exit(remap_sys_time_exit);
MODULE_LICENSE("GPL");
//@}
