// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022, Pico Immersive Pte. Ltd ("Pico"). All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef KEYWORD

#define KEYWORD_ENUM
#define KEYWORD(symbol) symbol,

enum HWSTATE_E{
#endif

KEYWORD(fsa4480)

#ifdef KEYWORD_ENUM
KEYWORD(HWSTATE_MAX)
};

int factory_test_hwstate_set(char *hwstate_name, char *module_state);
#undef KEYWORD_ENUM
#undef KEYWORD
#endif
