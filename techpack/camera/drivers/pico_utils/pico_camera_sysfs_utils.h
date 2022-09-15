// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022, Pico Immersive Pte. Ltd ("Pico"). All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __PICO_CAMERA_SYSFS_UTILS__
#define __PICO_CAMERA_SYSFS_UTILS__

enum camera_status_index {
	SIXDOF_TL, // 6dof top left
	SIXDOF_TR, // 6dof top right
	SIXDOF_BL, // 6dof bottom left
	SIXDOF_BR, // 6dof bottom right

	EYS_ET_FT, // eye et-ft

};

void pico_camera_set_camera_bit(enum camera_status_index);
void pico_camera_open_crm(void);
void pico_camera_close_crm(void);

#endif
