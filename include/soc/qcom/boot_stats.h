/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2013-2018, The Linux Foundation. All rights reserved.
 */

#ifdef CONFIG_MSM_BOOT_STATS

/*
 * Following structure defines all the cookies that have been placed
 * in boot's shared imem space.
 * The size of this struct must NOT exceed SHARED_IMEM_BOOT_SIZE
 */
struct boot_shared_imem_cookie_type
{
  /* Magic number which indicates boot shared imem has been initialized
     and the content is valid.*/
  uint32_t shared_imem_magic;

  /* Number to indicate what version of this structure is being used */
  uint32_t shared_imem_version;

  /* Pointer that points to etb ram dump buffer, should only be set by HLOS */
  uint64_t etb_buf_addr;

  /* Region where HLOS will write the l2 cache dump buffer start address */
  uint64_t l2_cache_dump_buff_addr;

  /* When SBL which is A32 allocates the 64bit pointer above it will only
     consume 4 bytes.  When HLOS running in A64 mode access this it will over
     flow into the member below it.  Adding this padding will ensure 8 bytes
     are consumed so A32 and A64 have the same view of the remaining members. */
  uint32_t a64_pointer_padding;

  /* Magic number for UEFI ram dump, if this cookie is set along with dload magic numbers,
     we don't enter dload mode but continue to boot. This cookie should only be set by UEFI*/
  uint32_t uefi_ram_dump_magic;

  uint32_t ddr_training_cookie;

  /* Abnormal reset cookie used by UEFI */
  uint32_t abnormal_reset_occurred;

  /* Reset Status Register */
  uint32_t reset_status_register;

  /* Cookie that will be used to sync with RPM */
  uint32_t rpm_sync_cookie;

  /* Debug config used by UEFI */
  uint32_t debug_config;

  /* Boot Log Location Pointer to be accessed in UEFI */
  uint64_t boot_log_addr;

  /* Boot Log Size */
  uint32_t boot_log_size;

  /*Boot failure count */
  uint32_t boot_fail_count;

  /*Error code delivery through EDL */
  uint32_t sbl1_error_type;

  /* cookie to detect if crash occurred in UEFI or HLOS */
  uint32_t uefi_image_magic;

  /*Boot Device option */
  uint32_t boot_device_type;

  /* Please add new cookie here, do NOT modify or rearrange the existing cookies*/
  uint32_t lpddr_vendor_id;
  uint32_t lpddr_device_type;
  uint32_t lpddr_capacity;
  uint32_t offline_dump_enable;
  uint64_t kernel_log_buf_addr;
  uint64_t log_first_idx_addr;
  uint64_t log_next_idx_addr;
  uint64_t pon_reason;
  uint64_t on_reason;
  uint64_t fault_reason;
  uint32_t is_enable_secure_boot;
  uint32_t offline_dump_happen;
  uint32_t limit_ddr;
  uint32_t if_has_ufs;
  uint32_t need_rendersplash;
  uint32_t hw_version;
  uint32_t slot_change;
  uint32_t lcd_magic;
/* Pico-BSP: supported to command force dump */
  uint32_t force_dump_enable;
  uint32_t bootcycle;
};


int boot_stats_init(void);
uint64_t get_boot_reason(void);
uint64_t get_on_reason(void);
uint64_t get_poff_fault_reason(void);
uint32_t get_secure_boot_value(void);
uint32_t get_lpddr_vendor_id(void);
uint32_t get_lpddr_device_type(void);
uint32_t get_lpddr_capacity(void);
const char * get_cpu_type(void);
uint32_t get_slot_change_status(void);
uint32_t get_lcd_magic(void);
/* Pico-BSP: supported to command force dump */
uint32_t get_force_dump_type(void);
uint32_t get_hw_version(void);
#else
static inline int boot_stats_init(void) { return 0; }
static inline uint64_t get_poff_fault_reason(void) { return 0; }
static inline uint64_t get_on_reason(void) { return 0; }
static inline uint64_t get_boot_reason(void) { return 0; }
static inline uint32_t get_secure_boot_value(void) { return 0; }
static inline uint32_t get_lpddr_vendor_id(void) { return 0; }
static inline uint32_t get_lpddr_device_type(void) { return 0; }
static inline uint32_t get_lpddr_capacity(void) { return 0; }
static inline uint32_t get_slot_change_status(void) { return 0; };
static uint32_t get_lcd_magic(void) { return 0; };
/* Pico-BSP: supported to command force dump */
static inline uint32_t get_force_dump_type(void) { return 0; };
static inline uint32_t get_lcd_magic(void) { return 0; };
static inline uint32_t get_hw_version(void) { return 0; };
#endif
