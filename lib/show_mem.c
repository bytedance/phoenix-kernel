/*
 * Generic show_mem() implementation
 *
 * Copyright (C) 2008 Johannes Weiner <hannes@saeurebad.de>
 * All code subject to the GPL version 2.
 */

#include <linux/mm.h>
#include <linux/quicklist.h>
#include <linux/cma.h>
#include <linux/oom.h>
#if defined(CONFIG_QCOM_KGSL) && defined(CONFIG_ION)
#include "../drivers/gpu/msm/kgsl.h"
#include "../drivers/staging/android/ion/ion_kernel.h"
#endif
#include <linux/dma-buf.h>

extern unsigned long zram_meminfo(void);
extern void ion_meminfo(struct ion_meminfo_data *d);
void show_mem(unsigned int filter, nodemask_t *nodemask)
{
	pg_data_t *pgdat;
	unsigned long total = 0, reserved = 0, highmem = 0;
	struct ion_meminfo_data d = {0};

	printk("Mem-Info:\n");
	show_free_areas(filter, nodemask);

	for_each_online_pgdat(pgdat) {
		unsigned long flags;
		int zoneid;

		pgdat_resize_lock(pgdat, &flags);
		for (zoneid = 0; zoneid < MAX_NR_ZONES; zoneid++) {
			struct zone *zone = &pgdat->node_zones[zoneid];
			if (!populated_zone(zone))
				continue;

			total += zone->present_pages;
			reserved += zone->present_pages - zone->managed_pages;

			if (is_highmem_idx(zoneid))
				highmem += zone->present_pages;
		}
		pgdat_resize_unlock(pgdat, &flags);
	}

	printk("%lu pages RAM\n", total);
	printk("%lu pages HighMem/MovableOnly\n", highmem);
	printk("%lu pages reserved\n", reserved);
#ifdef CONFIG_CMA
	printk("%lu pages cma reserved\n", totalcma_pages);
#endif
#ifdef CONFIG_QUICKLIST
	printk("%lu pages in pagetable cache\n",
		quicklist_total_size());
#endif
#ifdef CONFIG_MEMORY_FAILURE
	printk("%lu pages hwpoisoned\n", atomic_long_read(&num_poisoned_pages));
#endif
#if defined(CONFIG_QCOM_KGSL) && defined(CONFIG_ION)
	ion_meminfo(&d);
	pr_err("bytedance: zram_phy=%lukB, kgsl_alloc=%lukB, kgsl_pool=%lukB, dmabuf=%lukB, ion_cache=%lukB",
	       zram_meminfo() << (PAGE_SHIFT - 10),
	       (unsigned long)(atomic_long_read(&kgsl_driver.stats.page_alloc)) >> 10,
	       (unsigned long)(kgsl_pool_size_total() << (PAGE_SHIFT - 10)),
	       (unsigned long)dma_buf_total_size() >> 10,
	       (d.cached >> 10));
#endif
}
