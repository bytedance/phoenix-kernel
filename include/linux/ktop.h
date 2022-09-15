// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022, Pico Immersive Pte. Ltd ("Pico"). All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _LINUX_KTOP_H
#define _LINUX_KTOP_H

/* debug switch */
//#define KTOP_DEBUG_PRINT
//#define KTOP_MANUAL


#define KTOP_ARRAY_SIZE 3
#define KTOP_REPORT (KTOP_ARRAY_SIZE - 1)
#define KTOP_I (KTOP_ARRAY_SIZE - 1)

struct ktop_info {
	u32			sum_exec[KTOP_ARRAY_SIZE];
	struct list_head	list_entry[KTOP_ARRAY_SIZE];
};
extern void ktop_add(struct task_struct *p, u64 delta);
#define KTOP_RP_NUM 10

extern struct timer_list ktop_timer;



#ifdef KTOP_DEBUG_PRINT
#define ktop_pr_dbg(fmt, ...) do {pr_err("ktop: " fmt, ##__VA_ARGS__);} while (0)
#else /* KTOP_DEBUG_PRINT */
#define ktop_pr_dbg(fmt, ...) do {} while (0)
#endif /* KTOP_DEBUG_PRINT */


#ifdef CONFIG_PROC_KTOP_DEBUG
extern int ktop_d_stat__examine;
extern int ktop_d_stat__add;
extern bool ktop_d_working;
extern void ktop_d_write_debug(void);
extern struct list_head ktop_d_list;
extern void ktop_d_show_debug(struct seq_file *m, void *v);
extern void ktop_d_init(void);
extern void ktop_d_add(struct task_struct *p);
#ifdef CONFIG_PROC_KTOP_DEBUG_EXACT
extern void ktop_d_show_exact(struct seq_file *m, void *v);
extern void ktop_d_add_exact(struct task_struct *p);
extern void ktop_d_write_exact(void);
extern void ktop_d_show_exact(struct seq_file *m, void *v);
extern u64 ktop_d_start;
extern u64 ktop_d_end;
extern spinlock_t ktop_d_lock;
#endif /* CONFIG_PROC_KTOP_DEBUG_EXACT */
#endif /* CONFIG_PROC_KTOP_DEBUG */


#endif /* _LINUX_KTOP_H */
