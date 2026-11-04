/****************************************************************************
 * libs/libc/audio/lib_buffer.c
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
#include <nuttx/cache.h>

#include <sys/types.h>
#include <sys/param.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/audio/audio.h>
#include <nuttx/usb/audio.h>
#include <nuttx/mutex.h>
#include <syslog.h>

#include "libc.h"

/* [V698] apb lifetime audit.
 *
 * lib_buffer.c's apb_free() used to free whenever `apb->crefs--` left a
 * value <= 1, so a second free of the same apb (crefs already 0) called
 * lib_ufree() again and pushed the same block onto the heap free list twice.
 * That is exactly the corruption mm_malloc() tripped over.  This block keeps
 * a small ring of the apb blocks that have really been returned to the heap,
 * so a repeat is visible, and lets apb_free() leak instead of corrupting.
 */

#define V698_RING 32

static FAR struct ap_buffer_s *g_v698_freed[V698_RING];
static unsigned int g_v698_freed_w;
static unsigned int g_v698_free_calls;
static unsigned int g_v698_double_n;

static bool v698_was_freed(FAR struct ap_buffer_s *apb)
{
  int i;

  for (i = 0; i < V698_RING; i++)
    {
      if (g_v698_freed[i] == apb)
        {
          return true;
        }
    }

  return false;
}

static void v698_mark_freed(FAR struct ap_buffer_s *apb)
{
  g_v698_freed[g_v698_freed_w] = apb;
  g_v698_freed_w = (g_v698_freed_w + 1) % V698_RING;
}

static void v698_mark_live(FAR struct ap_buffer_s *apb)
{
  int i;

  for (i = 0; i < V698_RING; i++)
    {
      if (g_v698_freed[i] == apb)
        {
          g_v698_freed[i] = NULL;
        }
    }
}

#if defined(CONFIG_AUDIO)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: apb_alloc
 *
 * Allocate an Audio Pipeline Buffer for use in the Audio sub-system.  This
 * will perform the actual allocate based on buffer data format, number of
 * channels, etc. and prepare the buffer for consumption.
 *
 ****************************************************************************/

int apb_alloc(FAR struct audio_buf_desc_s *bufdesc)
{
  int                    ret;
  FAR struct ap_buffer_s *apb;
  FAR uint8_t            *samp;

  DEBUGASSERT(bufdesc->u.pbuffer != NULL);

  /* Perform a user mode allocation */

  apb  = lib_umalloc(sizeof(struct ap_buffer_s));
  samp = lib_umemalign(MAX(up_get_dcache_linesize(), 1), bufdesc->numbytes);

  *bufdesc->u.pbuffer = apb;

  /* Test if the allocation was successful or not */

  if (apb == NULL || samp == NULL)
    {
      ret = -ENOMEM;
      lib_ufree(apb);
      lib_ufree(samp);
    }
  else
    {
      /* Populate the buffer contents */

      memset(apb, 0, sizeof(struct ap_buffer_s));
      memset(samp, 0, bufdesc->numbytes);
      apb->i.channels = 1;
      apb->crefs      = 1;
      apb->nmaxbytes  = bufdesc->numbytes;
      apb->nbytes     = 0;
      apb->flags      = 0;
      apb->samp       = samp;
#ifdef CONFIG_AUDIO_MULTI_SESSION
      apb->session    = bufdesc->session;
#endif

      nxmutex_init(&apb->lock);

      /* [V698] the block is live again */

      v698_mark_live(apb);
      /* [V878] see H1 */
      if (0) syslog(LOG_INFO, "[V698] apb new apb=%p samp=%p nb=%d\n",
             apb, samp, (int)bufdesc->numbytes);

      ret = sizeof(struct audio_buf_desc_s);
    }

  return ret;
}

/****************************************************************************
 * Name: apb_free
 *
 * Free's a previously allocated or referenced Audio Pipeline Buffer
 *
 ****************************************************************************/

void apb_free(FAR struct ap_buffer_s *apb)
{
  int refcount;
  int v698_dup;

  /* [V698] did we already hand this very block back to the heap? */

  v698_dup = v698_was_freed(apb) ? 1 : 0;

  /* Perform a reference count decrement and possibly release the memory */

  nxmutex_lock(&apb->lock);
  refcount = apb->crefs--;
  nxmutex_unlock(&apb->lock);

  /* [V698] Report the first 24 frees plus every 64th, and always report a
   * repeat, together with the calling address.
   */

  g_v698_free_calls++;
  /* [V878] Disabled.  Same panic class as [V877]: the board dies with
   * EXCCAUSE=001c inside vsyslog right after one of these lines.  This is
   * the audio buffer release path, so it runs under i2s_rx_worker(), and
   * it also passes __builtin_return_address(0) to %p -- peeling the return
   * address on Xtensa is fragile once frames are optimised.  Restore the
   * original condition to bring the audit back. */
  if (0)
    {
      syslog(LOG_INFO,
             "[V698] apb free apb=%p refcount=%d dup=%d caller=%p\n",
             apb, refcount, v698_dup, __builtin_return_address(0));
    }

  if (refcount == 1)
    {
      /* This was the last reference -- really release the memory. */

      audinfo("Freeing %p\n", apb);
      v698_mark_freed(apb);
      nxmutex_destroy(&apb->lock);
      lib_ufree(apb->samp);
      lib_ufree(apb);
    }
  else if (refcount <= 0)
    {
      /* [V698] A genuine double free: the reference count was already zero,
       * i.e. this apb has already been returned to the heap.  Freeing again
       * pushes the same block onto the heap free list a second time -- the
       * corruption that made mm_malloc() walk into a live apb payload and
       * panic (EXCCAUSE=001d, vaddr=0x8).  Leak instead of corrupting.
       */

      g_v698_double_n++;
      syslog(LOG_ERR,
             "[V698] *** APB DOUBLE-FREE apb=%p refcount=%d dup=%d "
             "n=%u caller=%p\n",
             apb, refcount, v698_dup, g_v698_double_n,
             __builtin_return_address(0));
    }
}

/****************************************************************************
 * Name: apb_reference
 *
 * Claim a reference to an Audio Pipeline Buffer.  Each call to apb_reference
 * will increment the reference count and must have a matching apb_free
 * call.  When the refcount decrements to zero, the buffer will be freed.
 *
 ****************************************************************************/

void apb_reference(FAR struct ap_buffer_s *apb)
{
  /* Do we need any thread protection here?  Almost certainly... */

  nxmutex_lock(&apb->lock);
  apb->crefs++;
  nxmutex_unlock(&apb->lock);
}

#endif /* CONFIG_AUDIO */
