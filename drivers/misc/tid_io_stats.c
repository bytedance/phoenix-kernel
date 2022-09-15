// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022, Pico Immersive Pte. Ltd ("Pico"). All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/atomic.h>
#include <linux/cpufreq_times.h>
#include <linux/err.h>
#include <linux/hashtable.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/proc_fs.h>
#include <linux/profile.h>
#include <linux/sched/cputime.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>

#define TID_RECORD_MAX_NUMBER  10
#define TID_IOWAIT_MAX_COUNT   64

static spinlock_t tid_io_lock;
static struct proc_dir_entry *tid_io_parent;

struct tid_io_stats {
	struct timespec64 occurrence_time;
	s64 iowait_time;
};

struct tid_io_stats_head {
	pid_t tid;
	int count;
	int index;
	s64 iowait_threshold;
	struct tid_io_stats iowait_time_max;
	struct tid_io_stats *tid_io_stats_address;
};

static struct {
	int tid_numbers;
	struct tid_io_stats_head tid_header[TID_RECORD_MAX_NUMBER];
} tid_record = {0};

static struct {
	pid_t  tid;
	s64    iowait_threshold;
	int    activate;
} tid_input = {0};

static pid_t tid_data;

static int tid_threshold_activate_show(struct seq_file *m, void *v)
{
	unsigned long flags;

	spin_lock_irqsave(&tid_io_lock, flags);
	seq_printf(m, "%d,%lld,%d\n",
		   tid_input.tid,
		   tid_input.iowait_threshold,
		   tid_input.activate);
	spin_unlock_irqrestore(&tid_io_lock, flags);
	return 0;
}

static int tid_threshold_activate_open(struct inode *inode, struct file *file)
{
	return single_open(file, tid_threshold_activate_show, PDE_DATA(inode));
}

static ssize_t tid_threshold_activate_write(struct file *file,
					    const char __user *buffer,
					    size_t count, loff_t *ppos)
{
	int i, argc;
	char input[64];
	unsigned long flags;
	struct tid_io_stats_head *p_header;

	if (count >= sizeof(input))
		return -EINVAL;

	if (copy_from_user(input, buffer, count))
		return -EFAULT;

	input[count] = '\0';

	spin_lock_irqsave(&tid_io_lock, flags);

	argc = sscanf(input, "%d,%lld,%d",
		      &tid_input.tid,
		      &tid_input.iowait_threshold,
		      &tid_input.activate);
	if (argc != 3) {
		spin_unlock_irqrestore(&tid_io_lock, flags);
		return -EINVAL;
	}

	if (tid_record.tid_numbers == TID_RECORD_MAX_NUMBER &&
	    tid_input.activate == 1) {
		spin_unlock_irqrestore(&tid_io_lock, flags);
		return -ENOSPC;
	} else if (tid_record.tid_numbers == 0 && tid_input.activate == 0) {
		spin_unlock_irqrestore(&tid_io_lock, flags);
		return -ENOENT;
	}
	/* iterate through the array to find the item that
	   contains tid_input.tid */
	for (i = 0; i < tid_record.tid_numbers; i++) {
		if (tid_input.tid == tid_record.tid_header[i].tid)
			break;
	}

	if (i == tid_record.tid_numbers && tid_input.activate == 1 &&
	    tid_input.tid > 0 && tid_input.iowait_threshold > 0) {
		/* activate tid recording operation */
		p_header = &(tid_record.tid_header[i]);
		p_header->tid_io_stats_address =
			(struct tid_io_stats *)kzalloc(
			sizeof(struct tid_io_stats) * TID_IOWAIT_MAX_COUNT,
			GFP_NOWAIT);
		if (!p_header->tid_io_stats_address) {
			spin_unlock_irqrestore(&tid_io_lock, flags);
			return -ENOMEM;
		}

		p_header->tid = tid_input.tid;
		p_header->iowait_threshold = tid_input.iowait_threshold;
		tid_record.tid_numbers++;

	} else if (i < tid_record.tid_numbers && tid_input.activate == 0) {
		/* deactivate tid recording operation */
		kfree(tid_record.tid_header[i].tid_io_stats_address);
		for (i++; i < tid_record.tid_numbers; i++)
			tid_record.tid_header[i-1] = tid_record.tid_header[i];

		memset(&tid_record.tid_header[i], 0,
		       sizeof(struct tid_io_stats_head));
		tid_record.tid_numbers--;

	} else {
		spin_unlock_irqrestore(&tid_io_lock, flags);
		return -EINVAL;
	}

	spin_unlock_irqrestore(&tid_io_lock, flags);
	pr_info("%s PID:%d record successfully",
		tid_input.activate == 1 ? "activate":"deactivate",
		tid_input.tid);

	return count;
}

static const struct file_operations tid_threshold_activate_fops = {
	.open		= tid_threshold_activate_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.write		= tid_threshold_activate_write,
	.release	= single_release,
};

static int tids_show(struct seq_file *m, void *v)
{
	int i;
	unsigned long flags;

	spin_lock_irqsave(&tid_io_lock, flags);

	for (i = 0; i < tid_record.tid_numbers; i++)
		seq_printf(m, "%d ", tid_record.tid_header[i].tid);

	spin_unlock_irqrestore(&tid_io_lock, flags);
	seq_puts(m, "\n");
	return 0;
}

static int tids_open(struct inode *inode, struct file *file)
{
	return single_open(file, tids_show, PDE_DATA(inode));
}

static const struct file_operations tids_fops = {
	.open		= tids_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static one_tid_data_show(struct seq_file *m, void *v, pid_t tid)
{
	int i, k, p_index;
	unsigned long flags;
	struct tid_io_stats *p_address = NULL;
	struct tid_io_stats *p_temp = NULL;
	struct tid_io_stats_head *p_header;

	if (tid <= 0)
		return -EINVAL;

	seq_printf(m, "%d: ", tid);
	spin_lock_irqsave(&tid_io_lock, flags);
	for (i = 0; i < tid_record.tid_numbers; i++) {
		p_header = &(tid_record.tid_header[i]);

		if (p_header->tid != tid)
			continue;

		if (p_header->count <= 0)
			break;

		p_address = p_header->tid_io_stats_address;
		p_index = p_header->index;

		if (!p_address) {
			spin_unlock_irqrestore(&tid_io_lock, flags);
			pr_err("tid_io_stats_address invalid\n");
			return -EINVAL;
		}

		if (p_header->count <= TID_IOWAIT_MAX_COUNT) {
			/* in case: memory is not full */
			for (k = 0; k < p_header->count; k++) {
				p_temp = p_address + k;
				seq_printf(m, "%lld,%ld,%lld ",
					   p_temp->occurrence_time.tv_sec,
					   p_temp->occurrence_time.tv_nsec,
					   p_temp->iowait_time);
			}
		} else {
			/* in case: memory is full */
			for (k = 0; k < TID_IOWAIT_MAX_COUNT; k++) {
				p_temp = p_address + p_index;
				seq_printf(m, "%lld,%ld,%lld ",
					   p_temp->occurrence_time.tv_sec,
					   p_temp->occurrence_time.tv_nsec,
					   p_temp->iowait_time);
				p_index = p_index >= TID_IOWAIT_MAX_COUNT-1 ?
					0 : p_index+1;
			}
		}
		break;
	}
	spin_unlock_irqrestore(&tid_io_lock, flags);
	seq_puts(m, "\n");

	return 0;
}

static int tid_data_show(struct seq_file *m, void *v)
{
	return one_tid_data_show(m, v, tid_data);
}

static int tid_data_open(struct inode *inode, struct file *file)
{
	return single_open(file, tid_data_show, PDE_DATA(inode));
}

static ssize_t tid_data_write(struct file *file,
			      const char __user *buffer,
			      size_t count, loff_t *ppos)
{
	int argc;
	char input[16];

	if (count >= sizeof(input))
		return -EINVAL;

	if (copy_from_user(input, buffer, count))
		return -EFAULT;

	input[count] = '\0';

	argc = sscanf(input, "%d", &tid_data);
	if (argc != 1)
		return -EINVAL;
	return count;
}

static const struct file_operations tid_data_fops = {
	.open		= tid_data_open,
	.read		= seq_read,
	.write		= tid_data_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int all_data_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < tid_record.tid_numbers; i++)
		one_tid_data_show(m, v, tid_record.tid_header[i].tid);
	return 0;
}

static int all_data_open(struct inode *inode, struct file *file)
{
	return single_open(file, all_data_show, PDE_DATA(inode));
}

static const struct file_operations all_data_fops = {
	.open		= all_data_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int wait_time_max_show(struct seq_file *m, void *v)
{
	int i;
	unsigned long flags;
	struct tid_io_stats_head *p_header;

	spin_lock_irqsave(&tid_io_lock, flags);
	for (i = 0; i < tid_record.tid_numbers; i++) {
		p_header = &(tid_record.tid_header[i]);
		seq_printf(m, "%d:%d:%lld,%ld,%lld ",
			   p_header->tid,
			   p_header->count,
			   p_header->iowait_time_max.occurrence_time.tv_sec,
			   p_header->iowait_time_max.occurrence_time.tv_nsec,
			   p_header->iowait_time_max.iowait_time);
	}
	spin_unlock_irqrestore(&tid_io_lock, flags);
	seq_puts(m, "\n");
	return 0;
}

static int wait_time_max_open(struct inode *inode, struct file *file)
{
	return single_open(file, wait_time_max_show, PDE_DATA(inode));
}

static const struct file_operations wait_time_max_fops = {
	.open		= wait_time_max_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_tid_io_stats_init(void)
{

	spin_lock_init(&tid_io_lock);
	tid_io_parent = proc_mkdir("tid_io", NULL);
	if (!tid_io_parent) {
		pr_err("%s: failed to create tid_io proc entry\n",
		       __func__);
		goto err;
	}

	proc_create_data("tid_threshold_activate", 0644, tid_io_parent,
			 &tid_threshold_activate_fops, NULL);

	proc_create_data("tids", 0444, tid_io_parent,
			 &tids_fops, NULL);

	proc_create_data("tid_data", 0644, tid_io_parent,
			 &tid_data_fops, NULL);

	proc_create_data("all_data", 0444, tid_io_parent,
			 &all_data_fops, NULL);

	proc_create_data("wait_time_max", 0444, tid_io_parent,
			 &wait_time_max_fops, NULL);

	return 0;

err:
	remove_proc_subtree("tid_io", NULL);
	return -ENOMEM;
}

void record_iowait(struct task_struct *p_task, s64 delta)
{
	int i;
	struct tid_io_stats *p_address;
	struct tid_io_stats *p_max;
	struct tid_io_stats_head *p_header;

	if (!spin_trylock(&tid_io_lock))
		return;

	if (tid_record.tid_numbers <= 0) {
		spin_unlock(&tid_io_lock);
		return;
	}

	for (i = 0; i < tid_record.tid_numbers; i++) {
		p_header = &(tid_record.tid_header[i]);

		if (p_task->pid != p_header->tid)
			continue;

		if (delta < p_header->iowait_threshold) {
			spin_unlock(&tid_io_lock);
			return;
		}
		p_address = p_header->tid_io_stats_address + p_header->index;
		p_address->occurrence_time = current_kernel_time64();
		p_address->iowait_time = delta;

		p_max = &(p_header->iowait_time_max);
		if (delta > p_max->iowait_time) {
			p_max->iowait_time = delta;
			p_max->occurrence_time = p_address->occurrence_time;
		}
		p_header->index = p_header->index >=
			TID_IOWAIT_MAX_COUNT-1 ? 0 : p_header->index+1;
		p_header->count++;
		spin_unlock(&tid_io_lock);
		return;
	}
	spin_unlock(&tid_io_lock);
	return;
}

early_initcall(proc_tid_io_stats_init);
