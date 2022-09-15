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
struct list_head ktop_d_list_exact;
void ktop_d_add_exact(struct task_struct *p)
{
	if (!p->sched_info.ktop_d_sum_exec_exact) {
		get_task_struct(p);
		p->sched_info.ktop_d_sum_exec_exact = max(1ULL, p->se.sum_exec_runtime);
		p->sched_info.utime = max(1ULL, p->utime);
		p->sched_info.stime = max(1ULL, p->stime);
		list_add(&p->sched_info.ktop_d_list_entry_exact, &ktop_d_list_exact);
	}
}

void ktop_d_show_exact(struct seq_file *m, void *v)
{
	struct task_struct *p, *r, *n;
	struct list_head report_list;
	bool report;
	int j = 1;

	INIT_LIST_HEAD(&report_list);


	spin_lock(&ktop_d_lock);

	if(!list_empty(&ktop_d_list_exact)) {
		p = list_first_entry(&ktop_d_list_exact, struct task_struct, sched_info.ktop_d_list_entry_exact);
		list_del(&p->sched_info.ktop_d_list_entry_exact);
		p->sched_info.ktop_d_sum_exec_exact = p->se.sum_exec_runtime - p->sched_info.ktop_d_sum_exec_exact;
		list_add(&p->sched_info.ktop_d_list_entry_exact, &report_list);
	} else {
		spin_unlock(&ktop_d_lock);
		goto exit;
	}
	if(list_empty(&ktop_d_list_exact)) {
		spin_unlock(&ktop_d_lock);
		goto report;
	}
	list_for_each_entry_safe(p, n, &ktop_d_list_exact, sched_info.ktop_d_list_entry_exact) {
		ktop_pr_dbg("%s() line:%d pid=%d comm=%s prev=%llu, now=%llu, sum=%llu, utime prev=%llu, now=%llu, stime prev=%llu, now=%llu\n", __func__, __LINE__, p->pid, p->comm, p->sched_info.ktop_d_sum_exec_exact, p->se.sum_exec_runtime, p->se.sum_exec_runtime-p->sched_info.ktop_d_sum_exec_exact, p->sched_info.utime, p->utime, p->sched_info.stime,p->stime);
		p->sched_info.ktop_d_sum_exec_exact = p->se.sum_exec_runtime > p->sched_info.ktop_d_sum_exec_exact ?
			(p->se.sum_exec_runtime - p->sched_info.ktop_d_sum_exec_exact) : (p->sched_info.ktop_d_sum_exec_exact - p->se.sum_exec_runtime);
		report = false;

		/*
		 * list_for_each_entry(r, &report_list, sched_info.ktop_d_list_entry_exact) {
		 * 	ktop_pr_dbg("%s() line:%d  current list pid=%d comm=%s sum=%lu\n", __func__, __LINE__, r->pid, r->comm, r->sched_info.ktop_d_sum_exec_exact);
		 * }
		 */
		list_for_each_entry(r, &report_list, sched_info.ktop_d_list_entry_exact) {
			if (p->sched_info.ktop_d_sum_exec_exact > r->sched_info.ktop_d_sum_exec_exact)
				report = true;
			else
				break;
		}
		list_del(&p->sched_info.ktop_d_list_entry_exact);
		if (report || j < KTOP_RP_NUM) {
			if(!report) {
				ktop_pr_dbg("%s() line:%d ### <KTOP_D_NUM add pid=%d ahead\n", __func__, __LINE__, p->pid);
				list_add(&p->sched_info.ktop_d_list_entry_exact, &report_list);
			}
			else {
				ktop_pr_dbg("%s() line:%d  ### add pid=%d ahead pid=%d of sum=%lu\n", __func__, __LINE__, p->pid, r->pid, r->sched_info.ktop_d_sum_exec_exact);
				list_add_tail(&p->sched_info.ktop_d_list_entry_exact, &r->sched_info.ktop_d_list_entry_exact);
			}
			j++;
			if (j > KTOP_RP_NUM) {
				p = list_first_entry(&report_list, struct task_struct, sched_info.ktop_d_list_entry_exact);
				list_del(report_list.next);
				p->sched_info.ktop_d_sum_exec_exact = 0;
				put_task_struct(p);
			}
		} else {
			ktop_pr_dbg("%s() line:%d  ### delete pid=%d\n", __func__, __LINE__, p->pid);
			p->sched_info.ktop_d_sum_exec_exact = 0;
			put_task_struct(p);
		}
	}
	spin_unlock(&ktop_d_lock);
report:
	if(!list_empty(&report_list)) {
		seq_printf(m, "%s\n", "exact cpu usage:");
		seq_printf(m, "%-8s %-16s %s\n", "PID" , "COMM", "SUM");
		list_for_each_entry_safe_reverse(r, n, &report_list, sched_info.ktop_d_list_entry_exact) {
			ktop_pr_dbg("%s() line:%d reporting, r=%p\n", __func__, __LINE__, r);
			if (r->sched_info.ktop_d_sum_exec_exact > 1)
				seq_printf(m, "%-8d %-16s %lu\n", r->pid, r->comm, r->sched_info.ktop_d_sum_exec_exact);
			list_del(&r->sched_info.ktop_d_list_entry_exact);
			r->sched_info.ktop_d_sum_exec_exact = 0;
			put_task_struct(r);
		}
	}
	if (!list_empty(&ktop_d_list_exact))
		ktop_pr_dbg("err of list_exact");
exit:
	return;
}

void ktop_d_write_exact(void)
{
	unsigned int cpu;
	struct task_struct *r;
	struct rq *rq;
	unsigned long flags;

	INIT_LIST_HEAD(&ktop_d_list_exact);
	for_each_online_cpu(cpu) {
		rq = cpu_rq(cpu);
		raw_spin_lock_irqsave(&rq->lock, flags);
		spin_lock(&ktop_d_lock);
		if(rq->curr != rq->idle) {
			ktop_d_add_exact(rq->curr);
		}
		spin_unlock(&ktop_d_lock);
		raw_spin_unlock_irqrestore(&rq->lock, flags);
	}
	list_for_each_entry(r, &ktop_d_list_exact, sched_info.ktop_d_list_entry_exact) {
		ktop_pr_dbg("%s() line:%d  input adding pid=%d comm=%s sum=%lu\n", __func__, __LINE__, r->pid, r->comm, r->sched_info.ktop_d_sum_exec);
	}

}
