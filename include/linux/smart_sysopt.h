// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022, Pico Immersive Pte. Ltd ("Pico"). All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

//Smt: [FEAT_FREEZE] {@
#ifndef _LINUX_SMART_SYSOPT_H
#define _LINUX_SMART_SYSOPT_H
#include <linux/cgroup-defs.h>

extern int freezer_write_smtflags(struct cgroup_subsys_state *css, struct cftype *cft, u64 val);
extern u64 freezer_read_smtflags(struct cgroup_subsys_state *css, struct cftype *cft);
extern void smt_cgroup_freeze_killable(struct task_struct *task, bool freeze, bool sync);
#endif	/* _LINUX_SMART_SYSOPT_H*/
//@}
