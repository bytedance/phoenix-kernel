// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022, Pico Immersive Pte. Ltd ("Pico"). All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

//Smt: [FEAT_FREEZE] {@
#include "cgroup-internal.h"

#include <linux/module.h>
#include <linux/printk.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sched.h>

typedef struct freeze_session {
	struct task_struct *task;
	bool freeze;
	struct work_struct w;
} freeze_session_t;

static struct workqueue_struct *freeze_queue = NULL;
static struct cgroup *__private_cgroup_freeze_killable;

extern int cgroup_attach_task(struct cgroup *dst_cgrp, struct task_struct *leader,
			      bool threadgroup);
static void smt_cgroup_freeze_killable_handler(struct work_struct *work)
{
	freeze_session_t *session = container_of(work, freeze_session_t, w);
	struct task_struct *task = session->task;
	struct cgroup *dst_cgrp;
	struct cgroup_subsys_state *css;

	if (!__private_cgroup_freeze_killable) {
		pr_err("smt_cgrp_freeze, cgroup freeze killable not available, %d", task->pid);
		goto out;
	}

	if (session->freeze)
		dst_cgrp = __private_cgroup_freeze_killable;
	else {
		spin_lock_irq(&css_set_lock);
		css = task_css(task, freezer_cgrp_id);

		if (css->cgroup == __private_cgroup_freeze_killable)
			dst_cgrp = &__private_cgroup_freeze_killable->root->cgrp;
		spin_unlock_irq(&css_set_lock);
	}

	if (!dst_cgrp) {
		pr_err("smt_cgrp_freeze, not found dst cgroup, %d", task->pid);
		goto out;
	}
	mutex_lock(&cgroup_mutex);
	percpu_down_write(&cgroup_threadgroup_rwsem);
	cgroup_attach_task(dst_cgrp, task, true);
	percpu_up_write(&cgroup_threadgroup_rwsem);
	mutex_unlock(&cgroup_mutex);

	pr_info("put %d to cgroup %s", task->pid, dst_cgrp->kn->name);

out:
	kfree(session);
}

u64 freezer_read_smtflags(struct cgroup_subsys_state *css, struct cftype *cft)
{
	return test_bit(CGRP_FROZEN_KILLABLE, &css->cgroup->flags);
}
EXPORT_SYMBOL_GPL(freezer_read_smtflags);

int freezer_write_smtflags(struct cgroup_subsys_state *css, struct cftype *cft, u64 val)
{
	if (val) {
		set_bit(CGRP_FROZEN_KILLABLE, &css->cgroup->flags);
		__private_cgroup_freeze_killable = css->cgroup;
	} else {
		clear_bit(CGRP_FROZEN_KILLABLE, &css->cgroup->flags);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(freezer_write_smtflags);

void smt_cgroup_freeze_killable(struct task_struct *task, bool freeze, bool sync)
{
	freeze_session_t *session = kzalloc(sizeof(freeze_session_t), GFP_ATOMIC);

	session->task = task;
	session->freeze = freeze;

	if (sync) {
		smt_cgroup_freeze_killable_handler(&session->w);
	} else {
		INIT_WORK(&session->w, smt_cgroup_freeze_killable_handler);

		queue_work(freeze_queue, &session->w);
	}
}
EXPORT_SYMBOL_GPL(smt_cgroup_freeze_killable);

static int __init smt_sysopt_init()
{
	freeze_queue = create_singlethread_workqueue("freeze queue");
	if (!freeze_queue) {
		pr_err("freeze queue create failed");
		return -1;
	}

	pr_info("Smartisan Sysopt Module inited");
	return 0;
}

module_init(smt_sysopt_init);

MODULE_LICENSE("GPL");
//@}
