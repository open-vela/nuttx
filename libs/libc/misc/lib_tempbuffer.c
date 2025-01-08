/****************************************************************************
 * libs/libc/misc/lib_tempbuffer.c
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
#include <nuttx/spinlock.h>
#include <nuttx/lib/lib.h>

#include <stdlib.h>
#include <string.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#if CONFIG_PATH_MAX > CONFIG_LINE_MAX
#  define TEMP_MAX_SIZE CONFIG_PATH_MAX
#else
#  define TEMP_MAX_SIZE CONFIG_LINE_MAX
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tempbuffer_s
{
  spinlock_t lock;           /* Lock for the buffer */
  unsigned long free_bitmap; /* Bitmap of free buffer */
  char buffer[CONFIG_LIBC_MAX_TEMPBUFFER][TEMP_MAX_SIZE];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct tempbuffer_s g_tempbuffer =
{
  SP_UNLOCKED,
  (1u << CONFIG_LIBC_MAX_TEMPBUFFER) - 1,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_get_tempbuffer
 *
 * Description:
 *   The lib_get_tempbuffer() function returns a pointer to a temporary
 *   buffer.  The buffer is allocated from a pool of pre-allocated buffers
 *   and if the pool is exhausted, a new buffer is allocated through
 *   kmm_malloc(). The size of the buffer is nbytes, and must freed by
 *   calling lib_put_tempbuffer().
 *
 * Returned Value:
 *   On success, lib_get_tempbuffer() returns a pointer to a temporary
 *   buffer.  On failure, NULL is returned.
 *
 ****************************************************************************/

FAR char *lib_get_tempbuffer(size_t nbytes)
{
  irqstate_t flags;
  int index;

  if (nbytes <= TEMP_MAX_SIZE)
    {
      /* Try to find a free buffer */

      flags = spin_lock_irqsave(&g_tempbuffer.lock);
      index = ffsl(g_tempbuffer.free_bitmap) - 1;
      if (index >= 0 && index < CONFIG_LIBC_MAX_TEMPBUFFER)
        {
          g_tempbuffer.free_bitmap &= ~(1u << index);
          spin_unlock_irqrestore(&g_tempbuffer.lock, flags);
          return g_tempbuffer.buffer[index];
        }

      spin_unlock_irqrestore(&g_tempbuffer.lock, flags);
    }

  /* If no free buffer is found, allocate a new one if
   * CONFIG_LIBC_TEMPBUFFER_MALLOC is enabled
   */

#ifdef CONFIG_LIBC_TEMPBUFFER_MALLOC
  return lib_malloc(nbytes);
#else
  return NULL;
#endif
}

/****************************************************************************
 * Name: lib_put_tempbuffer
 *
 * Description:
 *   The lib_put_tempbuffer() function frees a temporary buffer that was
 *   allocated by lib_get_tempbuffer(). If the buffer was allocated
 *   dynamically, it is freed by calling kmm_free(). Otherwise, the buffer
 *   is marked as free in the pool of pre-allocated buffers.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lib_put_tempbuffer(FAR char *buffer)
{
  irqstate_t flags;
  int index;

  index = (buffer - &g_tempbuffer.buffer[0][0]) / TEMP_MAX_SIZE;
  if (index >= 0 && index < CONFIG_LIBC_MAX_TEMPBUFFER)
    {
      /* Mark the corresponding bit as free */

      flags = spin_lock_irqsave(&g_tempbuffer.lock);
      g_tempbuffer.free_bitmap |= 1u << index;
      spin_unlock_irqrestore(&g_tempbuffer.lock, flags);
      return;
    }

  /* Free the buffer if it was dynamically allocated */

#ifdef CONFIG_LIBC_TEMPBUFFER_MALLOC
  lib_free(buffer);
#endif
}
