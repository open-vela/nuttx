/****************************************************************************
 * boards/arm64/rk3588/evb7-amp/src/evb7_amp_cam.c
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

/* Consumer side of the camera path: Linux captures, this core reads.
 *
 * Why this direction uses shared memory at all, when the display stopped doing
 * so, is set out in evb7_amp_shm.h. Short version: the camera pipeline needs
 * Linux-side bring-up this core cannot do, and its output is already an image in
 * memory, so memory is the natural handover point. The display was the opposite -
 * this core can program the VOP window itself.
 *
 * What this file does not do is decide what the frame is for. It hands out the
 * newest complete frame and counts what went wrong; drawing it is the caller's
 * job. That split exists because the interesting failure modes here - a torn
 * frame, a descriptor that does not validate, notifications arriving faster than
 * they are consumed - are all about the transport, and mixing them with drawing
 * would make them harder to see.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <syslog.h>
#include <time.h>

#include <assert.h>
#include <fcntl.h>
#include <stddef.h>
#include <sys/ioctl.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>

#include <arch/barriers.h>
#include <arch/board/board.h>

#include "evb7_amp_shm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* How many times to re-read a frame that changed underneath us.
 *
 * One retry would almost always do: Linux publishes every 33ms at 30fps and the
 * copy here takes a couple of milliseconds, so losing the race twice in a row
 * needs the scheduler to stall this task mid-copy. Three attempts costs nothing
 * and turns "almost always" into "unless something is badly wrong", at which
 * point the caller gets -EAGAIN and the torn counter says why.
 */

#define CAM_COPY_ATTEMPTS 3

/* How many 500ms waits read() will sit through before returning -ETIMEDOUT.
 * Four is two seconds: long enough that a briefly idle producer does not
 * generate complaints, short enough that a persistent failure is on the console
 * while someone is still watching.
 */

#define CAM_READ_ROUNDS 4

/* The ioctl hands the caller's box array straight to evb7_amp_det_read() with a
 * cast, so the wire structure and the public one have to be the same bytes.
 *
 * Asserted rather than reasoned about. One is packed and the other is not, and
 * the natural layout happens to match the packed one - every field is already at
 * a multiple of its own size - but "happens to match" is exactly the kind of
 * claim that stops being true when somebody inserts a field, and it would then
 * fail as boxes drawn in the wrong places rather than as a build error.
 */

static_assert(sizeof(struct ampcam_box_s) == sizeof(struct amp_det_box_s),
              "public and wire box structures must be the same size");
static_assert(offsetof(struct ampcam_box_s, x) ==
              offsetof(struct amp_det_box_s, x), "box x moved");
static_assert(offsetof(struct ampcam_box_s, y) ==
              offsetof(struct amp_det_box_s, y), "box y moved");
static_assert(offsetof(struct ampcam_box_s, w) ==
              offsetof(struct amp_det_box_s, w), "box w moved");
static_assert(offsetof(struct ampcam_box_s, h) ==
              offsetof(struct amp_det_box_s, h), "box h moved");
static_assert(offsetof(struct ampcam_box_s, cls) ==
              offsetof(struct amp_det_box_s, cls), "box cls moved");
static_assert(offsetof(struct ampcam_box_s, score) ==
              offsetof(struct amp_det_box_s, score), "box score moved");

/* And the whole detection block has to fit the page reserved for it, or it runs
 * into the first frame slot.
 */

static_assert(sizeof(struct amp_det_desc_s) <= AMP_DET_DESC_SIZE,
              "detection descriptor does not fit its page");
static_assert(AMP_DET_DESC_OFFSET + AMP_DET_DESC_SIZE <= AMP_CAM_BUF0_OFFSET,
              "detection block overlaps the first frame slot");

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cam_copy_row
 *
 * Description:
 *   Copy one row out of the carveout, eight bytes at a time.
 *
 *   This exists because memcpy() here is a byte-at-a-time loop -
 *   libs/libc/string/lib_memcpy.c, taken because CONFIG_LIBC_ARCH_MEMCPY is
 *   unset and this architecture has no assembly version in the tree - and the
 *   source is MT_NORMAL_NC. That combination is pathological: every byte becomes
 *   its own uncached access to DRAM with no cache line to amortise it, so a
 *   589824-byte frame cost somewhere between fifteen and thirty milliseconds.
 *
 *   Which was not merely slow. Linux republishes every 33ms and there are two
 *   buffers, so a copy that long loses the sequence check every single time, all
 *   three attempts fail, and read() returns nothing while looking exactly like a
 *   camera that has not started. The visible symptom was an application that
 *   printed its geometry and then went quiet forever.
 *
 *   Both sides of the copy are at least four-byte aligned in practice - the
 *   slots are 1MB-aligned and the stride is a multiple of the pixel size - but
 *   that is not assumed here, because a future capture width could make the
 *   stride odd and the failure would be a silent unaligned access rather than
 *   anything obvious.
 *
 ****************************************************************************/

static void cam_copy_row(uint8_t *dst, const uint8_t *src, size_t len)
{
  /* Nothing clever for the misaligned case: fall back to the slow path rather
   * than shifting bytes into place, because it does not happen with any
   * geometry this publishes and pretending otherwise would be untested code on
   * a path that matters.
   */

  if ((((uintptr_t)dst | (uintptr_t)src) & 7) != 0)
    {
      memcpy(dst, src, len);
      return;
    }

  while (len >= 32)
    {
      /* Four loads before four stores, so the uncached reads can be in flight
       * together instead of one at a time. MT_NORMAL_NC permits gathering,
       * which is the whole reason this is worth doing.
       */

      uint64_t a = ((const uint64_t *)src)[0];
      uint64_t b = ((const uint64_t *)src)[1];
      uint64_t c = ((const uint64_t *)src)[2];
      uint64_t d = ((const uint64_t *)src)[3];

      ((uint64_t *)dst)[0] = a;
      ((uint64_t *)dst)[1] = b;
      ((uint64_t *)dst)[2] = c;
      ((uint64_t *)dst)[3] = d;

      src += 32;
      dst += 32;
      len -= 32;
    }

  while (len >= 8)
    {
      *(uint64_t *)dst = *(const uint64_t *)src;
      src += 8;
      dst += 8;
      len -= 8;
    }

  while (len-- > 0)
    {
      *dst++ = *src++;
    }
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The descriptor is a fixed address in the carveout. The mapping is flat, so a
 * physical address is also this core's pointer - the same reasoning as the
 * control block in evb7_amp_shm.c.
 *
 * volatile because Linux writes it. That is not a substitute for the barriers
 * below: volatile stops the compiler caching the value, it says nothing about
 * the order the hardware makes the accesses visible in.
 */

static volatile struct amp_cam_desc_s * const g_desc =
  (volatile struct amp_cam_desc_s *)
    (uintptr_t)(AMP_SHM_BASE + AMP_CAM_DESC_OFFSET);

static sem_t g_cam_sem;
static bool g_cam_ready;

/* Counters, all read by the stats call.
 *
 * They are separate rather than one "errors" number because they fail for
 * different reasons and want different fixes: notify without a matching copy is
 * this core being too slow, torn is the publication race, and bad is Linux
 * publishing something that does not validate - a bug on that side rather than
 * a timing problem.
 */

static uint32_t g_notify_cnt;
static uint32_t g_copy_cnt;
static uint32_t g_torn_cnt;
static uint32_t g_bad_cnt;
static uint32_t g_drop_cnt;
static uint32_t g_last_seq;

/* Detections live in their own block, written by Linux like the frames.
 *
 * Counted separately from the frame counters above for the same reason those are
 * separate from each other: detections arrive at a third of the frame rate, so
 * folding them into one set of numbers would make a healthy detector look like a
 * failing frame path.
 */

static volatile struct amp_det_desc_s * const g_det =
  (volatile struct amp_det_desc_s *)
    (uintptr_t)(AMP_SHM_BASE + AMP_DET_DESC_OFFSET);

static uint32_t g_det_notify_cnt;
static uint32_t g_det_read_cnt;
static uint32_t g_det_torn_cnt;
static uint32_t g_det_bad_cnt;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cam_validate
 *
 * Description:
 *   Check the descriptor before believing anything in it, and in particular
 *   before using stride and height to size a copy.
 *
 *   This is not defensive habit. The descriptor is written by another operating
 *   system on another core, and stride * height is about to become the length of
 *   a memcpy out of a 1MB slot. A wrong value there does not produce a bad
 *   picture, it reads past the end of the carveout - and the carveout is
 *   followed by memory that belongs to something else.
 *
 ****************************************************************************/

static int cam_validate(uint32_t ready, uint32_t *stride, uint32_t *width,
                        uint32_t *height)
{
  uint32_t nbuffers;
  uint32_t offset;
  uint32_t s;
  uint32_t w;
  uint32_t h;

  if (g_desc->magic != AMP_CAM_MAGIC)
    {
      return -ENODEV;                   /* Linux is not streaming yet */
    }

  if (g_desc->version != AMP_CAM_VERSION)
    {
      return -EPROTONOSUPPORT;
    }

  nbuffers = g_desc->nbuffers;
  if (nbuffers == 0 || nbuffers > AMP_CAM_NBUFFERS || ready >= nbuffers)
    {
      return -EINVAL;
    }

  /* Only XRGB8888 is handled, because that is what the framebuffer is and the
   * whole point of converting on the Linux side was to avoid having format
   * conversion on this one. Anything else is a mismatched pair of binaries.
   */

  if (g_desc->bpp != AMP_CAM_BPP)
    {
      return -EINVAL;
    }

  w = g_desc->width;
  h = g_desc->height;
  s = g_desc->stride;

  if (w == 0 || h == 0 || s < w * AMP_CAM_BPP)
    {
      return -EINVAL;
    }

  /* The bound that matters. Both factors come from another core, so check the
   * product against the slot rather than each part against something plausible.
   */

  if ((uint64_t)s * h > AMP_CAM_SLOT_SIZE)
    {
      return -E2BIG;
    }

  offset = g_desc->bufoffset[ready];
  if (offset != AMP_CAM_BUF0_OFFSET && offset != AMP_CAM_BUF1_OFFSET)
    {
      return -EINVAL;
    }

  *stride = s;
  *width  = w;
  *height = h;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: evb7_amp_cam_init
 ****************************************************************************/

int evb7_amp_cam_init(void)
{
  int ret;

  ret = sem_init(&g_cam_sem, 0, 0);
  if (ret < 0)
    {
      return -errno;
    }

  g_notify_cnt = 0;
  g_copy_cnt   = 0;
  g_torn_cnt   = 0;
  g_bad_cnt    = 0;
  g_drop_cnt   = 0;
  g_last_seq   = 0;
  g_cam_ready  = true;

  ret = evb7_amp_cam_register("/dev/amcam0");
  if (ret < 0)
    {
      /* The device is how anything reaches the frames, so losing it makes the
       * rest of this file unreachable - but it is still not worth failing the
       * whole shared-memory bring-up over, because touch shares that channel.
       */

      syslog(LOG_ERR, "[AMP] cam: /dev/amcam0 register failed %d\n", ret);
      g_cam_ready = false;
      return ret;
    }

  syslog(LOG_INFO,
         "[AMP] cam: /dev/amcam0 ready, descriptor at 0x%08lx, "
         "waiting for Linux\n",
         (unsigned long)(AMP_SHM_BASE + AMP_CAM_DESC_OFFSET));
  return OK;
}

/****************************************************************************
 * Name: evb7_amp_cam_notify
 *
 * Description:
 *   Called from the rpmsg endpoint callback. Only wakes a waiter; the frame
 *   itself is not touched here.
 *
 *   The index in the message is deliberately ignored - see the comment on
 *   amp_cam_msg_s. Doing anything with it would mean this core acting on a value
 *   that can be one frame stale, when the truth is one read away.
 *
 ****************************************************************************/

void evb7_amp_cam_notify(const struct amp_cam_msg_s *msg)
{
  int value = 0;

  UNUSED(msg);

  if (!g_cam_ready)
    {
      return;
    }

  g_notify_cnt++;

  /* At most one wakeup outstanding. Posting per notification would let the
   * semaphore count up to the number of frames missed while the consumer was
   * busy, and the consumer would then spin through that backlog reading the same
   * newest frame every time. One pending wakeup says exactly what is true:
   * there is something newer than what you last took.
   */

  if (sem_getvalue(&g_cam_sem, &value) == 0 && value > 0)
    {
      return;
    }

  sem_post(&g_cam_sem);
}

/****************************************************************************
 * Name: evb7_amp_cam_wait
 *
 * Description:
 *   Block until a frame notification arrives, or timeout_ms elapses. A negative
 *   timeout waits forever.
 *
 ****************************************************************************/

int evb7_amp_cam_wait(int timeout_ms)
{
  struct timespec ts;

  if (!g_cam_ready)
    {
      return -ENODEV;
    }

  if (timeout_ms < 0)
    {
      return sem_wait(&g_cam_sem) < 0 ? -errno : OK;
    }

  if (clock_gettime(CLOCK_REALTIME, &ts) < 0)
    {
      return -errno;
    }

  ts.tv_sec  += timeout_ms / 1000;
  ts.tv_nsec += (long)(timeout_ms % 1000) * 1000000;
  if (ts.tv_nsec >= 1000000000)
    {
      ts.tv_nsec -= 1000000000;
      ts.tv_sec++;
    }

  return sem_timedwait(&g_cam_sem, &ts) < 0 ? -errno : OK;
}

/****************************************************************************
 * Name: evb7_amp_cam_copy
 *
 * Description:
 *   Copy the newest complete frame into dst, one row at a time so the
 *   destination can have a different stride. Returns the geometry actually
 *   copied, clipped to maxw/maxh.
 *
 *   The seqlock is the whole substance of this function. With two buffers Linux
 *   writes into the one that is not published, so reading the published one is
 *   safe - unless Linux publishes twice while this copy is in progress, at which
 *   point it has come back around and started writing the buffer being read. The
 *   sequence number changing is exactly that condition, so re-reading it after
 *   the copy and retrying is what makes the result a whole frame rather than
 *   usually a whole frame.
 *
 ****************************************************************************/

int evb7_amp_cam_copy(void *dst, uint32_t dststride,
                      uint32_t maxw, uint32_t maxh,
                      uint32_t *width, uint32_t *height, uint32_t *seq)
{
  int attempt;
  int ret = -EAGAIN;

  if (dst == NULL || dststride == 0 || maxw == 0 || maxh == 0)
    {
      return -EINVAL;
    }

  if (!g_cam_ready)
    {
      return -ENODEV;
    }

  for (attempt = 0; attempt < CAM_COPY_ATTEMPTS; attempt++)
    {
      const uint8_t *src;
      uint8_t *out = dst;
      uint32_t seq0;
      uint32_t ready;
      uint32_t stride;
      uint32_t w;
      uint32_t h;
      uint32_t rowbytes;
      uint32_t row;

      seq0 = g_desc->seq;

      /* Everything below must be read after seq, or a stale ready could be
       * paired with a fresh sequence number and the check at the end would pass
       * on a frame that was never coherent.
       */

      UP_DMB();

      ready = g_desc->ready;

      ret = cam_validate(ready, &stride, &w, &h);
      if (ret < 0)
        {
          /* Not a race, so retrying will not help. -ENODEV just means Linux has
           * not started; anything else is a real disagreement and worth a
           * counter.
           */

          if (ret != -ENODEV)
            {
              g_bad_cnt++;
            }

          return ret;
        }

      if (w > maxw)
        {
          w = maxw;
        }

      if (h > maxh)
        {
          h = maxh;
        }

      rowbytes = w * AMP_CAM_BPP;
      if (rowbytes > dststride)
        {
          rowbytes = dststride;
        }

      src = (const uint8_t *)(uintptr_t)
            (AMP_SHM_BASE + g_desc->bufoffset[ready]);

      for (row = 0; row < h; row++)
        {
          cam_copy_row(out + (size_t)row * dststride,
                       src + (size_t)row * stride, rowbytes);
        }

      /* And the copy must finish before seq is read back, for the same reason in
       * the other direction.
       */

      UP_DMB();

      if (g_desc->seq == seq0)
        {
          /* Gaps are normal and not an error - they mean Linux produced frames
           * faster than this core asked for them, which at 30fps capture and a
           * display that redraws on demand is the expected state. Counting them
           * separately from torn frames keeps "we are behind" distinct from "we
           * read rubbish".
           */

          if (g_copy_cnt != 0 && seq0 > g_last_seq + 1)
            {
              g_drop_cnt += seq0 - g_last_seq - 1;
            }

          g_last_seq = seq0;
          g_copy_cnt++;

          if (width != NULL)
            {
              *width = w;
            }

          if (height != NULL)
            {
              *height = h;
            }

          if (seq != NULL)
            {
              *seq = seq0;
            }

          return OK;
        }

      g_torn_cnt++;
      ret = -EAGAIN;
    }

  return ret;
}

/****************************************************************************
 * Name: evb7_amp_cam_geom
 *
 * Description:
 *   Current capture geometry, or -ENODEV when Linux is not streaming. The
 *   stride reported is the packed one read() produces, not the stride in shared
 *   memory - a reader has no use for the latter.
 *
 ****************************************************************************/

int evb7_amp_cam_geom(uint32_t *width, uint32_t *height, uint32_t *seq)
{
  uint32_t stride;
  uint32_t w;
  uint32_t h;
  int ret;

  if (!g_cam_ready)
    {
      return -ENODEV;
    }

  ret = cam_validate(g_desc->ready, &stride, &w, &h);
  if (ret < 0)
    {
      return ret;
    }

  if (width != NULL)
    {
      *width = w;
    }

  if (height != NULL)
    {
      *height = h;
    }

  if (seq != NULL)
    {
      *seq = g_desc->seq;
    }

  return OK;
}

/****************************************************************************
 * Name: evb7_amp_det_notify
 ****************************************************************************/

void evb7_amp_det_notify(const struct amp_det_msg_s *msg)
{
  UNUSED(msg);

  /* Only a count. There is no semaphore to post because nothing waits for
   * detections: whoever draws them is already awake once per frame and takes
   * the newest set that exists. Blocking a drawing loop on a producer running
   * at a third of its rate would make the picture wait for the boxes, which is
   * the wrong way round.
   */

  g_det_notify_cnt++;
}

/****************************************************************************
 * Name: evb7_amp_det_read
 *
 * Description:
 *   The newest complete set of boxes, read under the same seqlock as the
 *   frames.
 *
 *   The all-or-nothing property is the whole point of reading it this way. A set
 *   of boxes mixing two frames is not a slightly stale picture, it is boxes that
 *   were never simultaneously true - two people drawn where one of them was and
 *   the other will be. That is worse than showing an older set, so a torn read
 *   is retried and then refused rather than returned.
 *
 ****************************************************************************/

int evb7_amp_det_read(struct amp_det_box_s *dst, unsigned int maxbox,
                      uint32_t *seq, uint32_t *width, uint32_t *height,
                      uint32_t *latency_us)
{
  int attempt;

  if (dst == NULL || maxbox == 0)
    {
      return -EINVAL;
    }

  if (!g_cam_ready)
    {
      return -ENODEV;
    }

  for (attempt = 0; attempt < CAM_COPY_ATTEMPTS; attempt++)
    {
      uint32_t seq0;
      uint32_t count;
      uint32_t w;
      uint32_t h;
      uint32_t lat;
      uint32_t i;

      if (g_det->magic != AMP_DET_MAGIC)
        {
          return -ENODEV;               /* Linux is not detecting yet */
        }

      if (g_det->version != AMP_DET_VERSION)
        {
          g_det_bad_cnt++;
          return -EPROTONOSUPPORT;
        }

      seq0 = g_det->seq;

      /* Odd means a write is in progress, and checking for it is what makes
       * this correct with a single block.
       *
       * The frames get away with a simpler scheme because there are two of
       * them: Linux fills the slot that is not published, so a reader never
       * looks at memory being written and the sequence number only has to
       * change once, at publication. There is one detection block, updated in
       * place, so that argument does not carry over - a reader could copy half
       * an old set and half a new one while the sequence number still held its
       * original value, and the check at the end would pass.
       *
       * So this is the textbook seqlock instead: the writer makes the count odd
       * before touching anything and even again afterwards. A torn set would be
       * worse than a stale one - boxes that were never simultaneously true, one
       * person drawn where they were and another where they will be.
       */

      if ((seq0 & 1u) != 0)
        {
          g_det_torn_cnt++;
          continue;
        }

      UP_DMB();

      count = g_det->count;
      w = g_det->width;
      h = g_det->height;
      lat = g_det->latency_us;

      /* Checked rather than trusted, because count is about to bound a loop
       * writing into the caller's array and it comes from another operating
       * system on another core.
       */

      if (count > AMP_DET_MAXBOX || w == 0 || h == 0)
        {
          g_det_bad_cnt++;
          return -EINVAL;
        }

      if (count > maxbox)
        {
          count = maxbox;
        }

      for (i = 0; i < count; i++)
        {
          dst[i].x        = g_det->box[i].x;
          dst[i].y        = g_det->box[i].y;
          dst[i].w        = g_det->box[i].w;
          dst[i].h        = g_det->box[i].h;
          dst[i].cls      = g_det->box[i].cls;
          dst[i].score    = g_det->box[i].score;
          dst[i].reserved = 0;
        }

      UP_DMB();

      if (g_det->seq == seq0)
        {
          g_det_read_cnt++;

          if (seq != NULL)
            {
              *seq = seq0;
            }

          if (width != NULL)
            {
              *width = w;
            }

          if (height != NULL)
            {
              *height = h;
            }

          if (latency_us != NULL)
            {
              *latency_us = lat;
            }

          return (int)count;
        }

      g_det_torn_cnt++;
    }

  return -EAGAIN;
}

/****************************************************************************
 * Name: evb7_amp_cam_stats
 ****************************************************************************/

void evb7_amp_cam_stats(uint32_t *notify, uint32_t *copied, uint32_t *torn,
                        uint32_t *bad, uint32_t *dropped)
{
  if (notify != NULL)
    {
      *notify = g_notify_cnt;
    }

  if (copied != NULL)
    {
      *copied = g_copy_cnt;
    }

  if (torn != NULL)
    {
      *torn = g_torn_cnt;
    }

  if (bad != NULL)
    {
      *bad = g_bad_cnt;
    }

  if (dropped != NULL)
    {
      *dropped = g_drop_cnt;
    }
}

/****************************************************************************
 * Name: evb7_amp_det_stats
 ****************************************************************************/

void evb7_amp_det_stats(uint32_t *notify, uint32_t *read, uint32_t *torn,
                        uint32_t *bad)
{
  if (notify != NULL)
    {
      *notify = g_det_notify_cnt;
    }

  if (read != NULL)
    {
      *read = g_det_read_cnt;
    }

  if (torn != NULL)
    {
      *torn = g_det_torn_cnt;
    }

  if (bad != NULL)
    {
      *bad = g_det_bad_cnt;
    }
}

/****************************************************************************
 * Character device
 ****************************************************************************/

/* /dev/amcam0 exists so that a consumer needs nothing from this file but the
 * ioctl numbers in <arch/board/board.h>.
 *
 * The alternative was a direct call into the functions above, which would have
 * meant an application including a header out of a board's src/ directory - and
 * then a second copy of the geometry structures when that turned out to be
 * awkward. A read() and two ioctls keep the transport on this side of the
 * boundary, and mean the eventual LVGL consumer is an ordinary file reader
 * rather than something wired to this board.
 */

/****************************************************************************
 * Name: cam_open
 ****************************************************************************/

static int cam_open(struct file *filep)
{
  /* f_priv carries the sequence number this handle last returned, stored in the
   * pointer itself rather than in an allocation. Per-handle rather than one
   * static, so two readers cannot silently starve each other of frames - and
   * zero is a safe start because Linux's first published frame is 1.
   */

  filep->f_priv = (void *)(uintptr_t)0;
  return OK;
}

/****************************************************************************
 * Name: cam_close
 ****************************************************************************/

static int cam_close(struct file *filep)
{
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: cam_read
 *
 * Description:
 *   One whole frame per call, packed at width * AMP_CAM_BPP, blocking until
 *   there is one newer than the last frame this handle returned.
 *
 *   Blocking rather than returning 0 when Linux has not started yet, because at
 *   boot that is the normal state: this core is up long before anything on the
 *   Linux side opens the sensor. A reader that does not want to wait can say so
 *   with O_NONBLOCK.
 *
 ****************************************************************************/

static ssize_t cam_read(struct file *filep, char *buffer, size_t buflen)
{
  uint32_t lastseq = (uint32_t)(uintptr_t)filep->f_priv;
  bool nonblock = (filep->f_oflags & O_NONBLOCK) != 0;
  unsigned int rounds = 0;

  if (buffer == NULL || buflen < AMP_CAM_BPP)
    {
      return -EINVAL;
    }

  for (; ; )
    {
      uint32_t w = 0;
      uint32_t h = 0;
      uint32_t seq = 0;
      uint32_t rows;
      int ret;

      ret = evb7_amp_cam_geom(&w, &h, &seq);
      if (ret == OK && seq != lastseq)
        {
          uint32_t packed = w * AMP_CAM_BPP;

          /* Whole rows only. Handing back a partial row would make the caller
           * work out where the image stops mid-line, and every plausible caller
           * is about to treat this as a rectangle.
           */

          rows = buflen / packed;
          if (rows == 0)
            {
              return -EINVAL;
            }

          if (rows > h)
            {
              rows = h;
            }

          ret = evb7_amp_cam_copy(buffer, packed, w, rows, &w, &h, &seq);
          if (ret == OK)
            {
              filep->f_priv = (void *)(uintptr_t)seq;
              return (ssize_t)(packed * h);
            }

          /* -EAGAIN here means Linux kept moving underneath the copy. Fall
           * through to the wait and try the next frame rather than returning it:
           * from the caller's point of view a frame it never saw was dropped,
           * which is not an error worth interrupting a video loop for.
           */

          if (ret != -EAGAIN)
            {
              return ret;
            }
        }
      else if (ret < 0 && ret != -ENODEV)
        {
          return ret;
        }

      if (nonblock)
        {
          return -EAGAIN;
        }

      /* A timeout rather than an indefinite wait, and then round the loop
       * again. Notifications can be missed - the endpoint callback collapses
       * them to one outstanding wakeup - so a reader that slept through the last
       * one would otherwise sit here with a frame already waiting in memory.
       */

      ret = evb7_amp_cam_wait(500);
      if (ret < 0 && ret != -ETIMEDOUT)
        {
          return ret;
        }

      /* Give up eventually, and report it, rather than blocking for ever.
       *
       * A read that never returns is the wrong shape for the failures that
       * actually happen here. If Linux is publishing faster than this side can
       * copy, every attempt loses the sequence check, and an unbounded loop
       * turns that into a program that prints nothing and looks identical to one
       * waiting for a camera that was never started. Coming back with
       * -ETIMEDOUT lets the caller say which of the two it is - the torn counter
       * distinguishes them immediately.
       */

      if (++rounds >= CAM_READ_ROUNDS)
        {
          return -ETIMEDOUT;
        }
    }
}

/****************************************************************************
 * Name: cam_ioctl
 ****************************************************************************/

static int cam_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  UNUSED(filep);

  switch (cmd)
    {
      case AMPCAMIOC_GETGEOM:
        {
          struct ampcam_geom_s *geom = (struct ampcam_geom_s *)arg;
          uint32_t w = 0;
          uint32_t h = 0;
          uint32_t seq = 0;
          int ret;

          if (geom == NULL)
            {
              return -EINVAL;
            }

          ret = evb7_amp_cam_geom(&w, &h, &seq);
          if (ret < 0)
            {
              /* Not streaming is reported as a zero-width frame rather than an
               * error, so a caller can poll geometry to find out when the
               * camera turns up without treating the normal case as a failure.
               */

              if (ret != -ENODEV)
                {
                  return ret;
                }

              memset(geom, 0, sizeof(*geom));
              return OK;
            }

          geom->width  = w;
          geom->height = h;
          geom->stride = w * AMP_CAM_BPP;
          geom->bpp    = AMP_CAM_BPP;
          geom->seq    = seq;
          return OK;
        }

      case AMPCAMIOC_GETSTATS:
        {
          struct ampcam_stats_s *st = (struct ampcam_stats_s *)arg;

          if (st == NULL)
            {
              return -EINVAL;
            }

          evb7_amp_cam_stats(&st->notify, &st->copied, &st->torn,
                             &st->bad, &st->dropped);
          evb7_amp_det_stats(&st->det_notify, &st->det_read, &st->det_torn,
                             &st->det_bad);
          return OK;
        }

      case AMPCAMIOC_GETDET:
        {
          struct ampcam_detect_s *d = (struct ampcam_detect_s *)arg;
          int n;

          if (d == NULL)
            {
              return -EINVAL;
            }

          memset(d, 0, sizeof(*d));

          /* The two structures are laid out identically on purpose - same
           * fields, same order, same widths - so this is a copy rather than a
           * field-by-field translation. They are still declared separately
           * because one is the wire format shared with Linux and the other is
           * this board's public interface, and letting an application include
           * the wire header is how the two stop being able to change
           * independently.
           */

          n = evb7_amp_det_read((struct amp_det_box_s *)d->box,
                                AMPCAM_MAXBOX, &d->seq, &d->width,
                                &d->height, &d->latency_us);
          if (n < 0)
            {
              /* Not producing is reported as count -1 rather than an error, so
               * a caller can redraw every frame without treating the normal
               * "no detector yet" state as a failure. A real disagreement -
               * a descriptor that does not validate - still comes back as an
               * error.
               */

              if (n == -ENODEV)
                {
                  d->count = -1;
                  return OK;
                }

              return n;
            }

          d->count = n;
          return OK;
        }

      default:
        return -ENOTTY;
    }
}

/* Designated rather than positional, because getting the order wrong here
 * compiles cleanly and then hands the ioctl to the mmap slot. The struct has a
 * dozen members and grows upstream; naming them means a future insertion cannot
 * silently shift these three.
 *
 * No poll method: the reader blocks in read() and that is all this needs. Adding
 * one would mean a second wakeup path to keep consistent with the semaphore for
 * no gain until something wants to wait on frames and something else at once.
 */

static const struct file_operations g_cam_fops =
{
  .open  = cam_open,
  .close = cam_close,
  .read  = cam_read,
  .ioctl = cam_ioctl,
};

/****************************************************************************
 * Name: evb7_amp_cam_register
 ****************************************************************************/

int evb7_amp_cam_register(const char *path)
{
  return register_driver(path, &g_cam_fops, 0444, NULL);
}
