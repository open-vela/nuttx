/****************************************************************************
 * arch/arm/src/bk7258/hardware/bk7258_psram.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __VENDOR_BEKEN_CHIPS_BK7258_HARDWARE_BK7258_PSRAM_H
#define __VENDOR_BEKEN_CHIPS_BK7258_HARDWARE_BK7258_PSRAM_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BK7258_PSRAM_BASE                    0x60000000u
#define BK7258_PSRAM_SIZE                    0x01000000u
#define BK7258_PSRAM_END                     0x61000000u

#define BK7258_PSRAM_SLAB_USER_BASE          0x60000000u
#define BK7258_PSRAM_SLAB_USER_SIZE          0x00019000u
#define BK7258_PSRAM_SLAB_AUDIO_BASE         0x60019000u
#define BK7258_PSRAM_SLAB_AUDIO_SIZE         0x00019000u
#define BK7258_PSRAM_SLAB_ENCODE_BASE        0x60032000u
#define BK7258_PSRAM_SLAB_ENCODE_SIZE        0x0015e000u
#define BK7258_PSRAM_SLAB_DISPLAY_BASE       0x60190000u
#define BK7258_PSRAM_SLAB_DISPLAY_SIZE       0x00570000u
#define BK7258_CP_PSRAM_HEAP_BASE            0x60700000u
#define BK7258_CP_PSRAM_HEAP_SIZE            0x00020000u
#define BK7258_AP_PSRAM_HEAP_BASE            0x60720000u
#define BK7258_AP_PSRAM_HEAP_SIZE            0x002e0000u
#define BK7258_AP_PSRAM_SECTION_BASE         0x60a00000u
#define BK7258_AP_PSRAM_SECTION_SIZE         0x00600000u

#if BK7258_PSRAM_SLAB_USER_BASE != BK7258_PSRAM_BASE || \
    BK7258_PSRAM_SLAB_USER_BASE + BK7258_PSRAM_SLAB_USER_SIZE != \
      BK7258_PSRAM_SLAB_AUDIO_BASE || \
    BK7258_PSRAM_SLAB_AUDIO_BASE + BK7258_PSRAM_SLAB_AUDIO_SIZE != \
      BK7258_PSRAM_SLAB_ENCODE_BASE || \
    BK7258_PSRAM_SLAB_ENCODE_BASE + BK7258_PSRAM_SLAB_ENCODE_SIZE != \
      BK7258_PSRAM_SLAB_DISPLAY_BASE || \
    BK7258_PSRAM_SLAB_DISPLAY_BASE + BK7258_PSRAM_SLAB_DISPLAY_SIZE != \
      BK7258_CP_PSRAM_HEAP_BASE || \
    BK7258_CP_PSRAM_HEAP_BASE + BK7258_CP_PSRAM_HEAP_SIZE != \
      BK7258_AP_PSRAM_HEAP_BASE || \
    BK7258_AP_PSRAM_HEAP_BASE + BK7258_AP_PSRAM_HEAP_SIZE != \
      BK7258_AP_PSRAM_SECTION_BASE || \
    BK7258_AP_PSRAM_SECTION_BASE + BK7258_AP_PSRAM_SECTION_SIZE != \
      BK7258_PSRAM_END || \
    BK7258_PSRAM_BASE + BK7258_PSRAM_SIZE != BK7258_PSRAM_END
#  error "BK7258 PSRAM layout is not the validated 16 MB layout"
#endif

#endif
