// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2020, The Linux Foundation. All rights reserved.
 */

#include "cam_req_mgr_workq.h"
#include "cam_debug_util.h"
#ifdef __CAMERA_EDITOR__
/*Add by liubo.leoliu @ pico camera, 20220727, using kthread worker with prio to instead workqueue tasks*/
#include <uapi/linux/sched/types.h>
#endif

#define WORKQ_ACQUIRE_LOCK(workq, flags) {\
	if ((workq)->in_irq) \
		spin_lock_irqsave(&(workq)->lock_bh, (flags)); \
	else \
		spin_lock_bh(&(workq)->lock_bh); \
}

#define WORKQ_RELEASE_LOCK(workq, flags) {\
	if ((workq)->in_irq) \
		spin_unlock_irqrestore(&(workq)->lock_bh, (flags)); \
	else	\
		spin_unlock_bh(&(workq)->lock_bh); \
}

struct crm_workq_task *cam_req_mgr_workq_get_task(
	struct cam_req_mgr_core_workq *workq)
{
	struct crm_workq_task *task = NULL;
	unsigned long flags = 0;

	if (!workq)
		return NULL;

	WORKQ_ACQUIRE_LOCK(workq, flags);
	if (list_empty(&workq->task.empty_head))
		goto end;

	task = list_first_entry(&workq->task.empty_head,
		struct crm_workq_task, entry);
	if (task) {
		atomic_sub(1, &workq->task.free_cnt);
		list_del_init(&task->entry);
	}

end:
	WORKQ_RELEASE_LOCK(workq, flags);

	return task;
}

static void cam_req_mgr_workq_put_task(struct crm_workq_task *task)
{
	struct cam_req_mgr_core_workq *workq =
		(struct cam_req_mgr_core_workq *)task->parent;
	unsigned long flags = 0;

	list_del_init(&task->entry);
	task->cancel = 0;
	task->process_cb = NULL;
	task->priv = NULL;
	WORKQ_ACQUIRE_LOCK(workq, flags);
	list_add_tail(&task->entry,
		&workq->task.empty_head);
	atomic_add(1, &workq->task.free_cnt);
	WORKQ_RELEASE_LOCK(workq, flags);
}

/**
 * cam_req_mgr_process_task() - Process the enqueued task
 * @task: pointer to task workq thread shall process
 */
static int cam_req_mgr_process_task(struct crm_workq_task *task)
{
	struct cam_req_mgr_core_workq *workq = NULL;

	if (!task)
		return -EINVAL;

	workq = (struct cam_req_mgr_core_workq *)task->parent;
	if (task->process_cb)
		task->process_cb(task->priv, task->payload);
	else
		CAM_WARN(CAM_CRM, "FATAL:no task handler registered for workq");
	cam_req_mgr_workq_put_task(task);

	return 0;
}

/**
 * cam_req_mgr_process_workq() - main loop handling
 * @w: workqueue task pointer
 */
void cam_req_mgr_process_workq(struct work_struct *w)
{
	struct cam_req_mgr_core_workq *workq = NULL;
	struct crm_workq_task         *task;
	int32_t                        i = CRM_TASK_PRIORITY_0;
	unsigned long                  flags = 0;

	if (!w) {
		CAM_ERR(CAM_CRM, "NULL task pointer can not schedule");
		return;
	}
	workq = (struct cam_req_mgr_core_workq *)
		container_of(w, struct cam_req_mgr_core_workq, work);

	while (i < CRM_TASK_PRIORITY_MAX) {
		WORKQ_ACQUIRE_LOCK(workq, flags);
		while (!list_empty(&workq->task.process_head[i])) {
			task = list_first_entry(&workq->task.process_head[i],
				struct crm_workq_task, entry);
			atomic_sub(1, &workq->task.pending_cnt);
			list_del_init(&task->entry);
			WORKQ_RELEASE_LOCK(workq, flags);
			cam_req_mgr_process_task(task);
			CAM_DBG(CAM_CRM, "processed task %pK free_cnt %d",
				task, atomic_read(&workq->task.free_cnt));
			WORKQ_ACQUIRE_LOCK(workq, flags);
		}
		WORKQ_RELEASE_LOCK(workq, flags);
		i++;
	}
}

#ifdef __CAMERA_EDITOR__
/*Add by liubo.leoliu @ pico camera, 20220727, using kthread worker with prio to instead workqueue tasks*/
void cam_req_mgr_process_workq_usingkthread(struct kthread_work *w)
{
	struct cam_req_mgr_core_workq *workq = NULL;

	if (!w) {
		CAM_ERR(CAM_CRM, "NULL task pointer can not schedule");
		return;
	}
	workq = (struct cam_req_mgr_core_workq *)
		container_of(w, struct cam_req_mgr_core_workq, work_using_kthread);
    cam_req_mgr_process_workq(&workq->work);
}
#endif
int cam_req_mgr_workq_enqueue_task(struct crm_workq_task *task,
	void *priv, int32_t prio)
{
	int rc = 0;
	struct cam_req_mgr_core_workq *workq = NULL;
	unsigned long flags = 0;

	if (!task) {
		CAM_WARN(CAM_CRM, "NULL task pointer can not schedule");
		rc = -EINVAL;
		goto end;
	}
	workq = (struct cam_req_mgr_core_workq *)task->parent;
	if (!workq) {
		CAM_DBG(CAM_CRM, "NULL workq pointer suspect mem corruption");
		rc = -EINVAL;
		goto end;
	}

	if (task->cancel == 1) {
		cam_req_mgr_workq_put_task(task);
		CAM_WARN(CAM_CRM, "task aborted and queued back to pool");
		rc = 0;
		goto end;
	}
	task->priv = priv;
	task->priority =
		(prio < CRM_TASK_PRIORITY_MAX && prio >= CRM_TASK_PRIORITY_0)
		? prio : CRM_TASK_PRIORITY_0;

	WORKQ_ACQUIRE_LOCK(workq, flags);
#ifdef __CAMERA_EDITOR__
		/*Add by liubo.leoliu @ pico camera, 20220727, using kthread worker with prio to instead workqueue tasks*/
		if (!workq->job && !workq->is_using_kthread) {
#else
		if (!workq->job) {
#endif
			rc = -EINVAL;
			WORKQ_RELEASE_LOCK(workq, flags);
			goto end;
		}
#ifdef __CAMERA_EDITOR__
		/*Add by liubo.leoliu @ pico camera, 20220727, using kthread worker with prio to instead workqueue tasks*/
		if (!workq->job_using_kthread && workq->is_using_kthread) {
			rc = -EINVAL;
			WORKQ_RELEASE_LOCK(workq, flags);
			goto end;
		}
#endif

	list_add_tail(&task->entry,
		&workq->task.process_head[task->priority]);

	atomic_add(1, &workq->task.pending_cnt);
	CAM_DBG(CAM_CRM, "enq task %pK pending_cnt %d",
		task, atomic_read(&workq->task.pending_cnt));

#ifdef __CAMERA_EDITOR__
	/*Add by liubo.leoliu @ pico camera, 20220727, using kthread worker with prio to instead workqueue tasks*/
	if (workq->is_using_kthread) {
		kthread_queue_work(workq->job_using_kthread, &workq->work_using_kthread);
	} else {
		queue_work(workq->job, &workq->work);
	}
#else
	queue_work(workq->job, &workq->work);
#endif
	WORKQ_RELEASE_LOCK(workq, flags);
end:
	return rc;
}

#ifdef __CAMERA_EDITOR__
/* LiuBo@pico camera 20220721 , affinity cpu to 0-3 for workqueue
 * cpu 4-7 runing algos, prio is higher than kernel workqueue prio
 * */
void cam_req_mgr_workq_affinity(struct cam_req_mgr_core_workq *wq)
{
	struct workqueue_attrs *attrs = NULL;
	int ret = -1;
	attrs = alloc_workqueue_attrs(GFP_KERNEL);
	cpumask_clear_cpu(4, attrs->cpumask);
	ret = apply_workqueue_attrs(wq->job, attrs);
	if (ret != 0) {
		CAM_ERR(CAM_CRM, "apply workqueue attrs failed!!");
	}
	free_workqueue_attrs(attrs);
}

/*Add by liubo.leoliu @ pico camera, 20220727, using kthread worker with prio to instead workqueue tasks*/
int cam_req_mgr_workq_create_usingkthread(char *name, int32_t num_tasks,
	struct cam_req_mgr_core_workq **workq, enum crm_workq_context in_irq,
	int flags, bool is_static_payload, void (*func)(struct kthread_work *w))
{
	int32_t i, wq_flags = 0, max_active_tasks = 0;
	struct crm_workq_task  *task;
	struct cam_req_mgr_core_workq *crm_workq = NULL;
	char buf[128] = "crm_workq-";
#ifdef __CAMERA_EDITOR__
	struct sched_param param = { .sched_priority = 49 };
#endif

	if (!*workq) {
		crm_workq = kzalloc(sizeof(struct cam_req_mgr_core_workq),
			GFP_KERNEL);
		if (crm_workq == NULL)
			return -ENOMEM;

		wq_flags |= WQ_UNBOUND;
		if (flags & CAM_WORKQ_FLAG_HIGH_PRIORITY)
			wq_flags |= WQ_HIGHPRI;

		if (flags & CAM_WORKQ_FLAG_SERIAL)
			max_active_tasks = 1;
#ifdef __CAMERA_EDITOR__
		crm_workq->is_using_kthread = true;
#endif
		strlcat(buf, name, sizeof(buf));
		CAM_DBG(CAM_CRM, "create workque crm_workq-%s", name);
#ifdef __CAMERA_EDITOR__
		if (crm_workq->is_using_kthread) {
			/*liubo.leoliu@pico camera, 20220726 modified for set sched params*/
			crm_workq->job_using_kthread = kthread_create_worker(0, buf);
			if (!crm_workq->job_using_kthread) {
				kfree(crm_workq);
				return -ENOMEM;
			}
			sched_setscheduler(crm_workq->job_using_kthread->task, SCHED_FIFO, &param);
			kthread_init_work(&crm_workq->work_using_kthread, func);
		} else {
            CAM_ERR(CAM_CRM, "calling cam_req_mgr_workq_create_usingkthread, but flags is false");
            return -ENOMEM;
		}
#endif
		spin_lock_init(&crm_workq->lock_bh);
		CAM_DBG(CAM_CRM, "LOCK_DBG workq %s lock %pK",
			name, &crm_workq->lock_bh);

		/* Task attributes initialization */
		atomic_set(&crm_workq->task.pending_cnt, 0);
		atomic_set(&crm_workq->task.free_cnt, 0);
		for (i = CRM_TASK_PRIORITY_0; i < CRM_TASK_PRIORITY_MAX; i++)
			INIT_LIST_HEAD(&crm_workq->task.process_head[i]);
		INIT_LIST_HEAD(&crm_workq->task.empty_head);
		crm_workq->in_irq = in_irq;
		crm_workq->is_static_payload = is_static_payload;
		crm_workq->task.num_task = num_tasks;
		crm_workq->task.pool = kcalloc(crm_workq->task.num_task,
				sizeof(struct crm_workq_task), GFP_KERNEL);
		if (!crm_workq->task.pool) {
			CAM_WARN(CAM_CRM, "Insufficient memory %zu",
				sizeof(struct crm_workq_task) *
				crm_workq->task.num_task);
			kfree(crm_workq);
			return -ENOMEM;
		}

		for (i = 0; i < crm_workq->task.num_task; i++) {
			task = &crm_workq->task.pool[i];
			task->parent = (void *)crm_workq;
			/* Put all tasks in free pool */
			INIT_LIST_HEAD(&task->entry);
			cam_req_mgr_workq_put_task(task);
		}
		*workq = crm_workq;
		CAM_DBG(CAM_CRM, "free tasks %d",
			atomic_read(&crm_workq->task.free_cnt));
	}

	return 0;
}
#endif

int cam_req_mgr_workq_create(char *name, int32_t num_tasks,
	struct cam_req_mgr_core_workq **workq, enum crm_workq_context in_irq,
	int flags, bool is_static_payload, void (*func)(struct work_struct *w))
{
	int32_t i, wq_flags = 0, max_active_tasks = 0;
	struct crm_workq_task  *task;
	struct cam_req_mgr_core_workq *crm_workq = NULL;
	char buf[128] = "crm_workq-";

	if (!*workq) {
		crm_workq = kzalloc(sizeof(struct cam_req_mgr_core_workq),
			GFP_KERNEL);
		if (crm_workq == NULL)
			return -ENOMEM;

		wq_flags |= WQ_UNBOUND;
		if (flags & CAM_WORKQ_FLAG_HIGH_PRIORITY)
			wq_flags |= WQ_HIGHPRI;

		if (flags & CAM_WORKQ_FLAG_SERIAL)
			max_active_tasks = 1;

		strlcat(buf, name, sizeof(buf));
		CAM_DBG(CAM_CRM, "create workque crm_workq-%s", name);
		crm_workq->job = alloc_workqueue(buf,
			wq_flags, max_active_tasks, NULL);
		if (!crm_workq->job) {
			kfree(crm_workq);
			return -ENOMEM;
		}

		/* Workq attributes initialization */
		INIT_WORK(&crm_workq->work, func);
		spin_lock_init(&crm_workq->lock_bh);
		CAM_DBG(CAM_CRM, "LOCK_DBG workq %s lock %pK",
			name, &crm_workq->lock_bh);

		/* Task attributes initialization */
		atomic_set(&crm_workq->task.pending_cnt, 0);
		atomic_set(&crm_workq->task.free_cnt, 0);
		for (i = CRM_TASK_PRIORITY_0; i < CRM_TASK_PRIORITY_MAX; i++)
			INIT_LIST_HEAD(&crm_workq->task.process_head[i]);
		INIT_LIST_HEAD(&crm_workq->task.empty_head);
		crm_workq->in_irq = in_irq;
		crm_workq->is_static_payload = is_static_payload;
		crm_workq->task.num_task = num_tasks;
		crm_workq->task.pool = kcalloc(crm_workq->task.num_task,
				sizeof(struct crm_workq_task), GFP_KERNEL);
		if (!crm_workq->task.pool) {
			CAM_WARN(CAM_CRM, "Insufficient memory %zu",
				sizeof(struct crm_workq_task) *
				crm_workq->task.num_task);
			kfree(crm_workq);
			return -ENOMEM;
		}

		for (i = 0; i < crm_workq->task.num_task; i++) {
			task = &crm_workq->task.pool[i];
			task->parent = (void *)crm_workq;
			/* Put all tasks in free pool */
			INIT_LIST_HEAD(&task->entry);
			cam_req_mgr_workq_put_task(task);
		}
		*workq = crm_workq;
		CAM_DBG(CAM_CRM, "free tasks %d",
			atomic_read(&crm_workq->task.free_cnt));
	}

	return 0;
}

void cam_req_mgr_workq_destroy(struct cam_req_mgr_core_workq **crm_workq)
{
	unsigned long flags = 0;
	struct workqueue_struct   *job;
#ifdef __CAMERA_EDITOR__
	/*Add by liubo.leoliu @ pico camera, 20220727, using kthread worker with prio to instead workqueue tasks*/
	struct kthread_worker     *job_using_kthread;
#endif

	CAM_DBG(CAM_CRM, "destroy workque %pK", crm_workq);
	if (*crm_workq) {
		WORKQ_ACQUIRE_LOCK(*crm_workq, flags);
		if ((*crm_workq)->job) {
			job = (*crm_workq)->job;
			(*crm_workq)->job = NULL;
			WORKQ_RELEASE_LOCK(*crm_workq, flags);
			destroy_workqueue(job);
		} else {
			WORKQ_RELEASE_LOCK(*crm_workq, flags);
		}
#ifdef __CAMERA_EDITOR__
		/*Add by liubo.leoliu @ pico camera, 20220727, using kthread worker with prio to instead workqueue tasks*/
		WORKQ_ACQUIRE_LOCK(*crm_workq, flags);
		if ((*crm_workq)->job_using_kthread) {
			job_using_kthread = (*crm_workq)->job_using_kthread;
			(*crm_workq)->job_using_kthread = NULL;
			WORKQ_RELEASE_LOCK(*crm_workq, flags);
			kthread_destroy_worker(job_using_kthread);
		} else {
			WORKQ_RELEASE_LOCK(*crm_workq, flags);
		}
#endif
		/* Destroy workq payload data */
		if (!((*crm_workq)->is_static_payload)) {
			kfree((*crm_workq)->task.pool[0].payload);
			(*crm_workq)->task.pool[0].payload = NULL;
		}
		kfree((*crm_workq)->task.pool);
		kfree(*crm_workq);
		*crm_workq = NULL;
	}
}
