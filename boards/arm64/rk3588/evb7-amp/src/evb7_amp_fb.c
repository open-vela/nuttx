/****************************************************************************
 * boards/arm64/rk3588/evb7-amp/src/evb7_amp_fb.c
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

/* A framebuffer whose memory is the shared area, so applications draw with
 * ordinary /dev/fb0 calls and Linux puts the result on the panel.
 *
 * There is no display controller behind this driver. This core cannot drive the
 * panel itself: the VOP's IOMMU is shared across all four of its video ports and
 * its interrupt is shared with that IOMMU's own, so neither can be handed to one
 * core alone while Linux owns the others. What the driver does instead is hand
 * out the shared buffer as framebuffer memory and, when an application says it
 * has finished a frame, ask the shared-frame layer to notify Linux.
 *
 * The consequence worth knowing: a flush here is not "the pixels are on the
 * screen", it is "the pixels are visible to whoever composites them". Nothing in
 * this driver can report vsync, which is why waitforvsync is deliberately absent
 * rather than stubbed out to return success.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <string.h>
#include <syslog.h>

#include <nuttx/clock.h>
#include <nuttx/video/fb.h>

#include "evb7_amp.h"
#include "evb7_amp_shm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Applications draw here, not into the shared area.
 *
 * The shared area has to be mapped non-cacheable, or the other core cannot see
 * what is written to it. That is fine for a buffer that gets filled once and
 * handed over, and ruinous for one that a graphics toolkit renders into: drawing
 * blends, which reads the destination pixel before writing it, and an uncached
 * read is a full trip to DRAM with no line fill and no prefetch to amortise it.
 *
 * Measured on this board with LVGL: rendering straight into the shared area took
 * about 255ms per full frame, against about 43ms for the host side to scale that
 * frame onto the panel. Roughly 85% of the frame time was this core stalling on
 * uncached accesses. A frame rate that low is not only a smoothness problem -
 * the toolkit polls the touch device from the same timer, so slow frames drop
 * input events and gestures stop working.
 *
 * So the framebuffer handed to applications is ordinary cacheable memory, and a
 * flush copies it across. The copy is sequential and write-combines, which is
 * the access pattern uncached memory is actually good at.
 *
 * The second benefit is smaller but real: the shared buffer is now written in one
 * short burst instead of being drawn into over hundreds of milliseconds, so the
 * window in which the host can read a half-drawn frame shrinks by the same
 * factor. It does not disappear - that needs a second buffer to flip between.
 */

#define AMP_FB_ALIGN 64

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_fbmem[AMP_SHM_BUFSIZE]
  __attribute__((aligned(AMP_FB_ALIGN)));

/* The other end of the copy. Flat mapping, so a physical address is a pointer. */

static uint8_t * const g_shmem =
  (uint8_t *)(uintptr_t)(AMP_SHM_BASE +
                         AMP_SHM_BUF_OFFSET(AMP_SHM_FB_INDEX));

static struct
{
  unsigned long flushes;
  unsigned long rows;
  unsigned long max_rows;
  clock_t       copy_ticks;
  clock_t       reported;
} g_stats;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int evb7_fb_getvideoinfo(struct fb_vtable_s *vtable,
                                struct fb_videoinfo_s *vinfo);
static int evb7_fb_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                                struct fb_planeinfo_s *pinfo);
static int evb7_fb_pandisplay(struct fb_vtable_s *vtable,
                              struct fb_planeinfo_s *pinfo);
#ifdef CONFIG_FB_UPDATE
static int evb7_fb_updatearea(struct fb_vtable_s *vtable,
                              const struct fb_area_s *area);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct fb_videoinfo_s g_videoinfo =
{
  /* FB_FMT_RGB32 is the framework's name for 32-bit colour with the unused
   * byte ignored, which is what the shared layout declares and what a Linux
   * XRGB8888 consumer expects. Alpha is not composited by anything here.
   */

  .fmt     = FB_FMT_RGB32,
  .xres    = AMP_SHM_WIDTH,
  .yres    = AMP_SHM_HEIGHT,
  .nplanes = 1,
};

static const struct fb_planeinfo_s g_planeinfo =
{
  .fbmem        = g_fbmem,
  .fblen        = AMP_SHM_BUFSIZE,
  .stride       = AMP_SHM_STRIDE,
  .display      = 0,
  .bpp          = AMP_SHM_BPP * 8,
  .xres_virtual = AMP_SHM_WIDTH,
  .yres_virtual = AMP_SHM_HEIGHT,
};

static struct fb_vtable_s g_fb_vtable =
{
  .getvideoinfo = evb7_fb_getvideoinfo,
  .getplaneinfo = evb7_fb_getplaneinfo,
  .pandisplay   = evb7_fb_pandisplay,
#ifdef CONFIG_FB_UPDATE
  .updatearea   = evb7_fb_updatearea,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int evb7_fb_getvideoinfo(struct fb_vtable_s *vtable,
                                struct fb_videoinfo_s *vinfo)
{
  if (vtable == NULL || vinfo == NULL)
    {
      return -EINVAL;
    }

  memcpy(vinfo, &g_videoinfo, sizeof(*vinfo));
  return OK;
}

static int evb7_fb_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                                struct fb_planeinfo_s *pinfo)
{
  if (vtable == NULL || pinfo == NULL || planeno != 0)
    {
      return -EINVAL;
    }

  memcpy(pinfo, &g_planeinfo, sizeof(*pinfo));
  return OK;
}

/****************************************************************************
 * Name: evb7_fb_pandisplay / evb7_fb_updatearea
 *
 * Description:
 *   Both mean "the application has finished drawing", and both do the same
 *   thing here.
 *
 *   Two hooks rather than one because applications use one or the other and
 *   there is no way to know which: the framebuffer example pans, while the NX
 *   and LVGL paths issue FBIO_UPDATE. Supporting only one of them would leave
 *   the other drawing into memory that is never announced - a display that
 *   quietly never updates, which is a poor failure to debug.
 *
 *   The area argument is ignored: the notification carries a whole buffer, so
 *   there is nothing useful to do with a partial rectangle until the protocol
 *   grows a damage region.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: evb7_fb_copyrows
 *
 * Description:
 *   Copy a band of rows into the shared area and announce the frame.
 *
 *   Rows rather than rectangles: a row is contiguous, so a band of them is a
 *   single memcpy, while a rectangle would be one short copy per row. Copying
 *   the few untouched pixels either side of the damaged area is cheaper than
 *   paying per-row call overhead for the privilege of skipping them.
 *
 *   Copying only part of the buffer is safe because the shared buffer keeps its
 *   previous contents - the rows outside the band still hold what was published
 *   last time, which is what the host is showing.
 *
 ****************************************************************************/

static void evb7_fb_copyrows(unsigned int y, unsigned int h)
{
  size_t offset;
  size_t len;
  clock_t start;

  if (y >= AMP_SHM_HEIGHT)
    {
      return;
    }

  if (h > AMP_SHM_HEIGHT - y)
    {
      h = AMP_SHM_HEIGHT - y;
    }

  offset = (size_t)y * AMP_SHM_STRIDE;
  len    = (size_t)h * AMP_SHM_STRIDE;

  start = clock_systime_ticks();

  memcpy(g_shmem + offset, g_fbmem + offset, len);

  /* How much is being redrawn, and what the handover costs.
   *
   * The host side can measure its own work and the gap between frames, but it
   * cannot see how much of the screen each frame actually touched - and that is
   * the number which separates "the toolkit is redrawing everything" from "the
   * toolkit is redrawing a little and taking a long time over it". Averaged over
   * a couple of seconds because the tick here is only a millisecond and a copy
   * is a few of them.
   */

  g_stats.flushes++;
  g_stats.rows += h;
  g_stats.copy_ticks += clock_systime_ticks() - start;

  if (h > g_stats.max_rows)
    {
      g_stats.max_rows = h;
    }

  if (clock_systime_ticks() - g_stats.reported >= MSEC2TICK(2000))
    {
      syslog(LOG_INFO,
             "[AMP] fb %lu flush(es): %lu rows avg (max %lu of %u), "
             "copy %lu ms total\n",
             (unsigned long)g_stats.flushes,
             (unsigned long)(g_stats.rows / g_stats.flushes),
             (unsigned long)g_stats.max_rows, AMP_SHM_HEIGHT,
             (unsigned long)TICK2MSEC(g_stats.copy_ticks));

      g_stats.reported   = clock_systime_ticks();
      g_stats.flushes    = 0;
      g_stats.rows       = 0;
      g_stats.max_rows   = 0;
      g_stats.copy_ticks = 0;
    }

  evb7_amp_shm_flush();
}

static int evb7_fb_pandisplay(struct fb_vtable_s *vtable,
                              struct fb_planeinfo_s *pinfo)
{
  /* No damage information, so the whole frame has to go. */

  evb7_fb_copyrows(0, AMP_SHM_HEIGHT);
  return OK;
}

#ifdef CONFIG_FB_UPDATE
static int evb7_fb_updatearea(struct fb_vtable_s *vtable,
                              const struct fb_area_s *area)
{
  if (area == NULL)
    {
      evb7_fb_copyrows(0, AMP_SHM_HEIGHT);
    }
  else
    {
      evb7_fb_copyrows(area->y, area->h);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fbinitialize
 *
 * Description:
 *   Initialise the framebuffer video hardware associated with the display.
 *   There is no hardware to initialise here - the memory is already mapped and
 *   the shared-frame layer is brought up before this - so this only has to be
 *   safe to call more than once, which the framework requires.
 *
 ****************************************************************************/

int up_fbinitialize(int display)
{
  return display == 0 ? OK : -EINVAL;
}

/****************************************************************************
 * Name: up_fbgetvplane
 ****************************************************************************/

struct fb_vtable_s *up_fbgetvplane(int display, int vplane)
{
  if (display != 0 || vplane != 0)
    {
      return NULL;
    }

  return &g_fb_vtable;
}

/****************************************************************************
 * Name: up_fbuninitialize
 ****************************************************************************/

void up_fbuninitialize(int display)
{
}

/****************************************************************************
 * Name: evb7_amp_fb_init
 *
 * Description:
 *   Register /dev/fb0 on top of the shared frame area.
 *
 ****************************************************************************/

int evb7_amp_fb_init(void)
{
  int ret = fb_register(0, 0);

  syslog(LOG_INFO,
         "[AMP] fb %s: %ux%u %ubpp, draw at %p -> share at %p\n",
         ret >= 0 ? "/dev/fb0 registered" : "registration FAILED",
         AMP_SHM_WIDTH, AMP_SHM_HEIGHT, AMP_SHM_BPP * 8,
         g_planeinfo.fbmem, g_shmem);

  return ret;
}
