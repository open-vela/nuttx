/****************************************************************************
 * arch/arm/src/bk7258/include/bk7258_psram.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef BK7258_PSRAM_H
#define BK7258_PSRAM_H
/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>
#include <stdint.h>
int bk7258_psram_initialize(void);
int bk7258_psram_shutdown(void);
void bk7258_psram_power_lost(void);
void bk7258_psram_dump(void);
uint32_t bk7258_psram_heap_used(void);
void *bk7258_psram_malloc(size_t size);
void *bk7258_psram_realloc(void *ptr, size_t size);
void bk7258_psram_free(void *ptr);
#endif
