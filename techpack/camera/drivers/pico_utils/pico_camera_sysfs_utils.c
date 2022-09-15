// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022, Pico Immersive Pte. Ltd ("Pico"). All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/device.h>
#include "cam_debug_util.h"
#include "pico_camera_sysfs_utils.h"


static struct kobject *pico_camera_kobj = NULL;
static unsigned long camera_status = 0;


static ssize_t pico_camera_sixdof_show(struct device *device, struct device_attribute *attr, char *buf)
{
	static int sixdof_mask = (1<<SIXDOF_TL) | (1<<SIXDOF_TR) | (1<<SIXDOF_BL) | (1<<SIXDOF_BR);
	return sprintf(buf, "%d\n", (camera_status & sixdof_mask) == sixdof_mask);
}
static DEVICE_ATTR(sixdof, 0444, pico_camera_sixdof_show, NULL);

static struct attribute *pico_camera_attrs[] = {
	&dev_attr_sixdof.attr,
	NULL
};
ATTRIBUTE_GROUPS(pico_camera);

void pico_camera_set_camera_bit(enum camera_status_index which)
{
	set_bit(which, &camera_status);
}
EXPORT_SYMBOL(pico_camera_set_camera_bit);

static void reset_all(void)
{
	camera_status = 0;
}

void pico_camera_open_crm(void)
{
	int rc = 0;
	rc = sysfs_create_groups(pico_camera_kobj, pico_camera_groups);
	if (rc) {
		CAM_ERR(CAM_UTIL, "can not craete sysfs");
	}
	reset_all();
}
EXPORT_SYMBOL(pico_camera_open_crm);

void pico_camera_close_crm(void)
{
	sysfs_remove_groups(pico_camera_kobj, pico_camera_groups);
	reset_all();
}
EXPORT_SYMBOL(pico_camera_close_crm);


struct kobject * __maybe_unused get_pico_camera_root_kobj(void)
{
	if (pico_camera_kobj != NULL) {
		return pico_camera_kobj;
	}
	CAM_ERR(CAM_UTIL, "pico camera kobj is null , maybe not init ?");
	return NULL;
}
EXPORT_SYMBOL(get_pico_camera_root_kobj);

static int __init pico_camera_sysfs_init(void)
{
	pico_camera_kobj = kobject_create_and_add("pico_camera", NULL);
	if (pico_camera_kobj == NULL) {
		CAM_ERR(CAM_UTIL, "can not create kobject");
		return -1;
	}
	return 0;
}

static void __exit pico_camera_sysfs_deinit(void)
{
	if (pico_camera_kobj != NULL) {
		CAM_ERR(CAM_UTIL, "can not create kobject");
		kobject_del(pico_camera_kobj);
		pico_camera_kobj = NULL;
	}
}


module_init(pico_camera_sysfs_init);
module_exit(pico_camera_sysfs_deinit);
MODULE_DESCRIPTION("pic camera sysfs utils");
MODULE_LICENSE("GPL v2");
