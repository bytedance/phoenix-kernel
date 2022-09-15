// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022, Pico Immersive Pte. Ltd ("Pico"). All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/sched/task.h>
#include <linux/seq_file.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/cpumask.h>
#include "../../../kernel/sched/sched.h"

#include <linux/ktop.h>

bool ktop_d_working;
int ktop_d_stat__examine;
int ktop_d_stat__add;
DEFINE_SPINLOCK(ktop_d_lock);
struct list_head ktop_d_list;
u64 ktop_d_start;
u64 ktop_d_end;


void ktop_d_add(struct task_struct *p)
{
	if (!p->sched_info.ktop_d_sum_exec) {
		get_task_struct(p);
		p->sched_info.ktop_d_sum_exec = max(1ULL, p->se.sum_exec_runtime);
		list_add(&p->sched_info.ktop_d_list_entry, &ktop_d_list);
#ifdef CONFIG_PROC_KTOP_DEBUG
		ktop_d_stat__add++;
#endif
	}
}
static int ktop_d_show_d(struct seq_file *m, void *v)
{
	struct task_struct *p, *r, *n;
	struct list_head report_list;
	bool report;
	int j = 1;

	INIT_LIST_HEAD(&report_list);
	spin_lock(&ktop_d_lock);

	if(!list_empty(&ktop_d_list)) {
		p = list_first_entry(&ktop_d_list, struct task_struct, sched_info.ktop_d_list_entry);
		list_del(&p->sched_info.ktop_d_list_entry);
		p->sched_info.ktop_d_sum_exec = p->se.sum_exec_runtime - p->sched_info.ktop_d_sum_exec;
		list_add(&p->sched_info.ktop_d_list_entry, &report_list);
	} else {
		spin_unlock(&ktop_d_lock);
		goto exit;
	}
	if(list_empty(&ktop_d_list)) {
		spin_unlock(&ktop_d_lock);
		goto report;
	}
	list_for_each_entry_safe(p, n, &ktop_d_list, sched_info.ktop_d_list_entry) {
		ktop_pr_dbg("%s() line:%d pid=%d comm=%s prev=%llu, now=%llu, sum=%llu\n", __func__, __LINE__, p->pid, p->comm, p->sched_info.ktop_d_sum_exec, p->se.sum_exec_runtime, p->se.sum_exec_runtime-p->sched_info.ktop_d_sum_exec);
		p->sched_info.ktop_d_sum_exec = p->se.sum_exec_runtime > p->sched_info.ktop_d_sum_exec ?
			(p->se.sum_exec_runtime - p->sched_info.ktop_d_sum_exec) : (p->sched_info.ktop_d_sum_exec - p->se.sum_exec_runtime);
		report = false;
		/*
		 * ktop_pr_dbg("%s() line:%d pid=%d comm=%s sum=%lu\n", __func__, __LINE__, p->pid, p->comm, p->sched_info.ktop_d_sum_exec);
		 */

		/*
		 * list_for_each_entry(r, &report_list, sched_info.ktop_d_list_entry) {
		 * 	ktop_pr_dbg("%s() line:%d  current list pid=%d comm=%s sum=%lu\n", __func__, __LINE__, r->pid, r->comm, r->sched_info.ktop_d_sum_exec);
		 * }
		 */
		list_for_each_entry(r, &report_list, sched_info.ktop_d_list_entry) {
			if (p->sched_info.ktop_d_sum_exec > r->sched_info.ktop_d_sum_exec)
				report = true;
			else
				break;
		}
		list_del(&p->sched_info.ktop_d_list_entry);
		if (report || j < KTOP_RP_NUM) {
			if(!report) {
				ktop_pr_dbg("%s() line:%d ### <KTOP_D_NUM add pid=%d ahead\n", __func__, __LINE__, p->pid);
				list_add(&p->sched_info.ktop_d_list_entry, &report_list);
			}
			else {
				ktop_pr_dbg("%s() line:%d  ### add pid=%d ahead pid=%d of sum=%lu\n", __func__, __LINE__, p->pid, r->pid, r->sched_info.ktop_d_sum_exec);
				list_add_tail(&p->sched_info.ktop_d_list_entry, &r->sched_info.ktop_d_list_entry);
			}
			j++;
			if (j > KTOP_RP_NUM) {
				p = list_first_entry(&report_list, struct task_struct, sched_info.ktop_d_list_entry);
				list_del(report_list.next);
				p->sched_info.ktop_d_sum_exec = 0;
				put_task_struct(p);
			}
		} else {
			ktop_pr_dbg("%s() line:%d  ### delete pid=%d\n", __func__, __LINE__, p->pid);
			p->sched_info.ktop_d_sum_exec = 0;
			put_task_struct(p);
		}
	}
	spin_unlock(&ktop_d_lock);
report:
	if(!list_empty(&report_list)) {
		seq_printf(m, "%-8s %-16s %s\n", "PID" , "COMM", "SUM");
		list_for_each_entry_safe_reverse(r, n, &report_list, sched_info.ktop_d_list_entry) {
			ktop_pr_dbg("%s() line:%d reporting, r=%p\n", __func__, __LINE__, r);
			if (r->sched_info.ktop_d_sum_exec > 1)
				seq_printf(m, "%-8d %-16s %lu\n", r->pid, r->comm, r->sched_info.ktop_d_sum_exec);
			list_del(&r->sched_info.ktop_d_list_entry);
			r->sched_info.ktop_d_sum_exec = 0;
			put_task_struct(r);
		}
	}
exit:
	if (!list_empty(&ktop_d_list))
		ktop_pr_dbg("ktop err of list");
	return 0;
}


static int ktop_d_show(struct seq_file *m, void *v)
{
	ktop_d_end = ktime_get_ns();
	seq_printf(m, "duration:%llu\n", ktop_d_end - ktop_d_start);

	spin_lock(&ktop_d_lock);
	ktop_d_working = false;
	spin_unlock(&ktop_d_lock);

	ktop_pr_dbg("%s() line:%d ktop_d_stat__examine=%d ktop_d_stat__add=%d\n", __func__, __LINE__, ktop_d_stat__examine, ktop_d_stat__add);
	ktop_d_show_exact(m, v);

	return ktop_d_show_d(m, v);
}

static ssize_t ktop_d_write(struct file *file, const char __user *user_buf, size_t nbytes, loff_t *ppos)
{
	char buf[32];
	size_t buf_size;
	long val;

	if (!nbytes)
		return -EINVAL;
	buf_size = min(nbytes, (sizeof(buf) - 1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	buf[buf_size - 1] = '\0';
	if (kstrtol(buf, 0, &val) != 0)
		return -EINVAL;

	INIT_LIST_HEAD(&ktop_d_list);

	ktop_d_stat__examine = 0;
	ktop_d_stat__add = 0;
	ktop_d_write_exact();
	spin_lock(&ktop_d_lock);
	ktop_d_working = true;
	spin_unlock(&ktop_d_lock);

	ktop_d_start = ktime_get_ns();
	return nbytes;
}

static int ktop_d_open(struct inode *inode, struct file *file)
{
	return single_open(file, ktop_d_show, NULL);
}

static const struct file_operations ktop_d_fops = {
	.open           = ktop_d_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.write          = ktop_d_write,
	.release        = single_release,
};


void proc_ktop_d_init(void)
{
	proc_create("ktop_d", 0666, NULL, &ktop_d_fops);
}


fs_initcall(proc_ktop_d_init);
