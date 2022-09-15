// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022, Pico Immersive Pte. Ltd ("Pico"). All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __HWINFO_H_
#define __HWINFO_H_
#define USE_EMMC                0
#define USE_UFS                 1
#define USE_UFS2                0
#define USE_PRODUCT_VERSION     1
#define USE_NFC                 0
#define USE_LCD                 0
#define USE_TP                  0
#define USE_TP_VERSION          0 // not available now
#define USE_MEMINFO             0
#define CONFIG_APQ_GPIO_READ    1

#if USE_UFS2
int get_ufs2_support(void);
#endif
#if USE_NFC
int get_nfc_support(void);
#endif
#if USE_PRODUCT_VERSION
int get_flash_version(void);
#endif
#endif
#ifndef KEYWORD
#define KEYWORD_ENUM
#define KEYWORD(symbol) symbol,
enum HWINFO_E{
#endif
KEYWORD(emmc_sn)
#if USE_EMMC
KEYWORD(emmc_manufacturer)
KEYWORD(emmc_capacity)
#endif
#if USE_UFS
KEYWORD(ufs_manufacturer)
KEYWORD(ufs_capacity)
KEYWORD(ufs_version)
KEYWORD(ufs_model)
#endif
#if USE_UFS2
KEYWORD(ufs1_manufacturer)
KEYWORD(ufs1_capacity)
KEYWORD(ufs1_version)
#endif
KEYWORD(lpddr_manufacturer)
KEYWORD(lpddr_device_type)
KEYWORD(lpddr_capacity)
#if USE_LCD
KEYWORD(LCD_MFR)        //LCD manufacturer
#endif
#if USE_TP
KEYWORD(TP_MFR)          //Touch manufacturer
#endif
#if USE_TP_VERSION
KEYWORD(tp_version)
#endif
KEYWORD(battery_type)
#if USE_PRODUCT_VERSION
KEYWORD(version_id)
#endif
KEYWORD(secboot_version)
KEYWORD(pon_reason)
//KEYWORD(wipower)
KEYWORD(qchip_id)
KEYWORD(housing_color)
#ifdef KEYWORD_ENUM
KEYWORD(HWINFO_MAX)
KEYWORD(hw_version)
};
int bytedance_hwinfo_register(enum HWINFO_E e_hwinfo,char *hwinfo_name);
#undef KEYWORD_ENUM
#undef KEYWORD
#endif
