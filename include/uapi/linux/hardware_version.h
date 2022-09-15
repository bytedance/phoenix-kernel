// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022, Pico Immersive Pte. Ltd ("Pico"). All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef HARDWARE_VERSION_H
#define HARDWARE_VERSION_H

#include <linux/types.h>
#include <linux/ioctl.h>

extern int hw_version_get(void);
#endif /* HARDWARE_VERSION_H */
