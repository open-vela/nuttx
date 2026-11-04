/****************************************************************************
 * mm/iob/iob_initialize.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/nuttx.h>
#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* [V634] Shrink the IOB pool for this board.
 *
 * CONFIG_IOB_NBUFFERS is 160 here, where the NuttX default is 8.  With
 * CONFIG_IOB_BUFSIZE=512 every IOB costs 524 bytes, so this single static
 * pool holds 83,843 bytes of .dram0.bss - and on the ESP32-S3 .bss competes
 * byte for byte with the internal, DMA-capable region of the system heap the
 * WiFi driver keeps its static_rx=10 + dynamic_rx=32 + dynamic_tx=32 buffers
 * in (see the comment in esp32s3_wifi_adapter.c).
 *
 * Measured with the [V633] internal-DRAM probe: once the link is up the
 * largest block still allocatable from that region falls 1024 -> 512 -> 128
 * -> 0 bytes while 7.5 MB of PSRAM sits idle, and every data frame from the
 * 47th onward then returns ESP_ERR_NO_MEM.  64 buffers is still 8x the NuttX
 * default and covers the sockets this application uses, and it returns about
 * 50 KB to the radio.
 *
 * Both IOB_BUFFER_SIZE and the initialisation loop below are defined in this
 * file, so the override is local and cannot desynchronise another translation
 * unit: the only other consumers are iob_statistics (reports the number) and
 * iob_free (a <= assertion, which a smaller pool keeps trivially true).
 */

#undef  CONFIG_IOB_NBUFFERS
#define CONFIG_IOB_NBUFFERS 64

/* Fix the I/O Buffer size with specified alignment size */

#ifdef CONFIG_IOB_ALLOC
#  define IOB_ALIGN_SIZE  ALIGN_UP(sizeof(struct iob_s) + CONFIG_IOB_BUFSIZE, \
                                   IOB_ALIGNMENT)
#else
#  define IOB_ALIGN_SIZE  ALIGN_UP(sizeof(struct iob_s), IOB_ALIGNMENT)
#endif

#define IOB_BUFFER_SIZE   (IOB_ALIGN_SIZE * CONFIG_IOB_NBUFFERS + \
                           IOB_ALIGNMENT - 1)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Following raw buffer will be divided into iob_s instances, the initial
 * procedure will ensure that the member io_data of each iob_s is aligned
 * to the IOB_ALIGNMENT memory boundary.
 */

#ifdef IOB_SECTION
static uint8_t g_iob_buffer[IOB_BUFFER_SIZE]
  locate_data(CONFIG_IOB_SECTION);
#else
static uint8_t g_iob_buffer[IOB_BUFFER_SIZE];
#endif

#if CONFIG_IOB_NCHAINS > 0
/* This is a pool of pre-allocated iob_qentry_s buffers */

static struct iob_qentry_s g_iob_qpool[CONFIG_IOB_NCHAINS];
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* A list of all free, unallocated I/O buffers */

FAR struct iob_s *g_iob_freelist;

/* A list of I/O buffers that are committed for allocation */

FAR struct iob_s *g_iob_committed;

#if CONFIG_IOB_NCHAINS > 0
/* A list of all free, unallocated I/O buffer queue containers */

FAR struct iob_qentry_s *g_iob_freeqlist;

/* A list of I/O buffer queue containers that are committed for allocation */

FAR struct iob_qentry_s *g_iob_qcommitted;
#endif

#undef g_iob_sem
DEFINE_PER_CPU_BMP(sem_t, g_iob_sem) = SEM_INITIALIZER(0);
#define g_iob_sem this_cpu_var_bmp(g_iob_sem)

/* Counting that tracks the number of free IOBs/qentries */

int16_t g_iob_count = CONFIG_IOB_NBUFFERS;

#if CONFIG_IOB_THROTTLE > 0

#  undef g_throttle_sem
DEFINE_PER_CPU_BMP(sem_t, g_throttle_sem) = SEM_INITIALIZER(0);
#  define g_throttle_sem this_cpu_var_bmp(g_throttle_sem)

/* Wait Counts for throttle */

int16_t g_throttle_wait = 0;
#endif

#if CONFIG_IOB_NCHAINS > 0

#  undef g_qentry_sem
DEFINE_PER_CPU_BMP(sem_t, g_qentry_sem) = SEM_INITIALIZER(0);
#  define g_qentry_sem this_cpu_var_bmp(g_qentry_sem)

/* Wait Counts for qentry */

int16_t g_qentry_wait = 0;
#endif

volatile spinlock_t g_iob_lock = SP_UNLOCKED;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_initialize
 *
 * Description:
 *   Set up the I/O buffers for normal operations.
 *
 ****************************************************************************/

void iob_initialize(void)
{
  int i;
  uintptr_t buf;
  irqstate_t flags;
  size_t offset;

  flags = spin_lock_irqsave(&g_iob_lock);

  if (g_iob_freelist != NULL)
    {
      /* Already initialized */

      spin_unlock_irqrestore(&g_iob_lock, flags);
      return;
    }

  /* Get a start address which plus offsetof(struct iob_s, io_data) is
   * aligned to the IOB_ALIGNMENT memory boundary
   */

#ifdef CONFIG_IOB_ALLOC
  offset = sizeof(struct iob_s);
#else
  offset = offsetof(struct iob_s, io_data);
#endif
  buf = ALIGN_UP((uintptr_t)g_iob_buffer + offset, IOB_ALIGNMENT) - offset;

  /* Get I/O buffer instance from the start address and add each I/O buffer
   * to the free list
   */

  for (i = 0; i < CONFIG_IOB_NBUFFERS; i++)
    {
      FAR struct iob_s *iob = (FAR struct iob_s *)(buf + i * IOB_ALIGN_SIZE);

      /* Add the pre-allocate I/O buffer to the head of the free list */

      iob->io_flink   = g_iob_freelist;
#ifdef CONFIG_IOB_ALLOC
      iob->io_bufsize = CONFIG_IOB_BUFSIZE;
      iob->io_data    = (FAR uint8_t *)(iob + 1);
#endif
      g_iob_freelist  = iob;
    }

#if CONFIG_IOB_NCHAINS > 0
  /* Add each I/O buffer chain queue container to the free list */

  for (i = 0; i < CONFIG_IOB_NCHAINS; i++)
    {
      FAR struct iob_qentry_s *iobq = &g_iob_qpool[i];

      /* Add the pre-allocate buffer container to the head of the free
       * list
       */

      iobq->qe_flink  = g_iob_freeqlist;
      g_iob_freeqlist = iobq;
    }
#endif

  spin_unlock_irqrestore(&g_iob_lock, flags);
}
