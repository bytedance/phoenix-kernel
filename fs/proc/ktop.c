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


#define KTOP_INTERVAL (MSEC_PER_SEC)
static u8 cur_idx;
struct list_head ktop_list[KTOP_I];
u64 ktop_time[KTOP_I];
#define KTOP_MAX 1000
//TODO reset this ktop_n
u32 ktop_n = KTOP_MAX;
#ifdef KTOP_DEBUG_PRINT
u32 ktop_stat__add;
#endif

DEFINE_SPINLOCK(ktop_lock);

void __always_inline ktop_add(struct task_struct *p, u64 delta)
{
	unsigned long flags;

	/* if the task use few cpu time in last statistic period then ignore it */
	if (delta < 50000)
		return;

	/* The first checking of sum_exec[cur_idx] is outside of spin lock to
	 * reduce competence. However the cur_idx could got changed after first
	 * checking of sum_exec[cur_idx] and before spin lock, so doulbe check
	 * of sum_exec[cur_idx] inside spink lock is necessary */
	if (!p->sched_info.ktop.sum_exec[cur_idx]) {
		if(spin_trylock_irqsave(&ktop_lock, flags)) {
			/* TODO this could change to single ktop_n */
			if ((ktop_n < KTOP_MAX) &&
			    !p->sched_info.ktop.sum_exec[cur_idx]) {
				ktop_n++;
				p->sched_info.ktop.sum_exec[cur_idx] =
					max(1U, (u32)(p->se.sum_exec_runtime >> 20));
				get_task_struct(p);
				list_add(&p->sched_info.ktop.list_entry[cur_idx], &ktop_list[cur_idx]);
			}
			spin_unlock_irqrestore(&ktop_lock, flags);
		}
	}
#ifdef KTOP_DEBUG_PRINT
	ktop_stat__add++;
#endif
}

static void ktop_timer_func(struct timer_list * timer)
{
	struct task_struct *p;
	struct list_head *k, *l, *m;

#ifdef KTOP_DEBUG_PRINT
	ktop_pr_dbg("ktop_n=%d", ktop_n);
	ktop_pr_dbg("ktop_stat__add=%d", ktop_stat__add);
	ktop_stat__add = 0;
#endif
	if(spin_trylock(&ktop_lock)) {
		cur_idx++;
		if (cur_idx >= KTOP_I)
			cur_idx = 0;
		ktop_n = 0;
		list_for_each_safe(l, m, &ktop_list[cur_idx]) {
			k = l - cur_idx;
			p = container_of(k, struct task_struct, sched_info.ktop.list_entry[0]);
			p->sched_info.ktop.sum_exec[cur_idx] = 0;
			list_del(l);
			put_task_struct(p);
		}
		ktop_time[cur_idx] = ktime_get_boot_fast_ns();
		spin_unlock(&ktop_lock);
		mod_timer(timer, jiffies + msecs_to_jiffies(KTOP_INTERVAL));
	} else
		mod_timer(timer, jiffies + msecs_to_jiffies(20));
}
DEFINE_TIMER(ktop_timer, ktop_timer_func);

static struct mutex ktop_show_lock;
static int ktop_show(struct seq_file *m, void *v)
{
	struct task_struct *p, *r, *q;
	struct list_head report_list;
	struct list_head *k, *l, *n, *o;
	bool report;
	int h, i, j = 0, start_idx;
	u32 run_tasks = 0;
	u64 now;
	u64 delta;
	unsigned long flags;
	struct pid_namespace *ns = task_active_pid_ns(current);

	INIT_LIST_HEAD(&report_list);

	mutex_lock(&ktop_show_lock);
	spin_lock_irqsave(&ktop_lock, flags);

	start_idx = cur_idx + 1;
	if (start_idx == KTOP_I)
		start_idx =  0;

	now = ktime_get_boot_fast_ns();
	delta = now - ktop_time[start_idx];

	/* loop over all ktop period (start from most ealier period), add tasks to report list. */
	for (h = 0, i = start_idx; h < KTOP_I; h++) {
		list_for_each_safe(l, o, &ktop_list[i]) {
			int a;
			u32 sum;
			bool added = false;
			report = false;
			k = l - i;
			p = container_of(k, struct task_struct, sched_info.ktop.list_entry[0]);
			a = i;
			/* check if this task was added to report list before */
			while (a != start_idx) {
				if (a == 0)
					a =  KTOP_I - 1;
				else
					a--;
				if (p->sched_info.ktop.sum_exec[a]) {
					added = true;
					break;
				}
			}
			/* if was added to report list before, ignore it */
			if (added)
				continue;
			run_tasks ++;
			sum = (u32)(p->se.sum_exec_runtime >> 20);
			p->sched_info.ktop.sum_exec[KTOP_REPORT] =
				sum > p->sched_info.ktop.sum_exec[i] ?
				(sum - p->sched_info.ktop.sum_exec[i]) : 0;
			ktop_pr_dbg("%s() line:%d start: p=%d comm=%s sum=%u\n", __func__, __LINE__,
				    p->pid, p->comm, p->sched_info.ktop.sum_exec[KTOP_REPORT]);
			list_for_each(n, &report_list) {
				k = n - KTOP_REPORT;
				r = container_of(k, struct task_struct, sched_info.ktop.list_entry[0]);
				if (p->sched_info.ktop.sum_exec[KTOP_REPORT] >
				    r->sched_info.ktop.sum_exec[KTOP_REPORT]) {
					report = true;
					q = r;
				}
				else
					break;
			}
			if (report || j < KTOP_RP_NUM) {
				get_task_struct(p);
				if(!report)
					list_add(&p->sched_info.ktop.list_entry[KTOP_REPORT], &report_list);
				else
					list_add(&p->sched_info.ktop.list_entry[KTOP_REPORT],
						 &q->sched_info.ktop.list_entry[KTOP_REPORT]);
				j++;
				if (j > KTOP_RP_NUM) {
					k = report_list.next;
					k = k - KTOP_REPORT;
					p = container_of(k, struct task_struct, sched_info.ktop.list_entry[0]);
					list_del(report_list.next);
					put_task_struct(p);
				}
			}
		}

		i++;
		if (i == KTOP_I)
			i = 0;
	}

	spin_unlock_irqrestore(&ktop_lock, flags);

#ifdef KTOP_DEBUG_PRINT
	list_for_each(n, &report_list) {
		k = n - KTOP_REPORT;
		r = container_of(k, struct task_struct, sched_info.ktop.list_entry[0]);
		ktop_pr_dbg("%s() line:%d final: p=%d comm=%-16s sum=%u\n",
			    __func__, __LINE__, r->pid, r->comm,
			    r->sched_info.ktop.sum_exec[KTOP_REPORT]);
	}
#endif

	if(!list_empty(&report_list)) {
		char comm_tg[TASK_COMM_LEN];
		char comm_pr[TASK_COMM_LEN];
		struct user_namespace *user_ns = current_user_ns();
		seq_printf(m, "duration:%d total_tasks:%u\n", (u32)(delta >> 20), run_tasks);
		seq_printf(m, "%-9s %-16s %-11s %-9s %-7s %-16s %-9s %-16s\n", "TID", "COMM", "SUM", "PID",
			   "UID", "PROCESS-COMM", "PPID", "PARENT-COMM");
		list_for_each_prev_safe(l, o, &report_list) {
			uid_t uid;
			pid_t ppid;
			struct task_struct *parent;
			k = l - KTOP_REPORT;
			p = container_of(k, struct task_struct, sched_info.ktop.list_entry[0]);
			if (p->group_leader != p) {
				rcu_read_lock();
				q = find_task_by_vpid(p->tgid);
				if (q)
					get_task_comm(comm_tg, q);
				rcu_read_unlock();
			}
			uid = from_kuid_munged(user_ns, task_uid(p));

			rcu_read_lock(); /* locks pid_task()->comm */
			ppid = task_ppid_nr_ns(p, ns);
			parent = find_task_by_vpid(ppid);
			if (parent)
				get_task_comm(comm_pr, parent);

			seq_printf(m, "%-9d %-16s %-11u %-9d %-7d %-16s %-9d %-16s\n", p->pid, p->comm,
				   p->sched_info.ktop.sum_exec[KTOP_REPORT], p->tgid, uid,
				   (p->group_leader != p) ? (q ? comm_tg : "EXITED") : p->comm, ppid,
				   parent ? comm_pr : "EXITED");
			rcu_read_unlock();

			list_del(l);

			put_task_struct(p);
		}
	}
	mutex_unlock(&ktop_show_lock);
	return 0;
}

static ssize_t ktop_write(struct file *file, const char __user *user_buf, size_t nbytes, loff_t *ppos)
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

#ifdef KTOP_MANUAL
	if (val == 1) {
		ktop_timer.expires = jiffies + msecs_to_jiffies(KTOP_INTERVAL);
		add_timer(&ktop_timer);
	} else if (val == 2) {
		del_timer(&ktop_timer);
	}
#endif
	return nbytes;
}

static int ktop_open(struct inode *inode, struct file *file)
{
	return single_open(file, ktop_show, NULL);
}

static const struct file_operations ktop_fops = {
	.open           = ktop_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.write          = ktop_write,
	.release        = single_release,
};

static int __init proc_ktop_init(void)
{
	int i;
	for(i = 0 ; i < KTOP_I ; i++) {
		INIT_LIST_HEAD(&ktop_list[i]);
	}

	mutex_init(&ktop_show_lock);

#ifndef KTOP_MANUAL
	ktop_timer.expires = jiffies + msecs_to_jiffies(KTOP_INTERVAL);
	add_timer(&ktop_timer);
#endif

	proc_create("ktop", 0666, NULL, &ktop_fops);
	return 0;
}

fs_initcall(proc_ktop_init);
