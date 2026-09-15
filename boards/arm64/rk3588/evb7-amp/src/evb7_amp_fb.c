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

/* /dev/fb0 as two buffers in this core's own RAM, scanned out by the VOP.
 *
 * How this got here
 * ----------------------------------------------------------------------------
 * The first version put the framebuffer in the AMP shared area and asked Linux
 * to composite it. That area has to be mapped non-cacheable for the other core
 * to see writes to it, and rendering into non-cacheable memory is ruinous -
 * blending reads the destination pixel before writing it, and an uncached read
 * is a full trip to DRAM with no line fill and no prefetch. Measured with LVGL:
 * about 255ms per full frame against about 43ms for the host to scale it onto
 * the panel.
 *
 * The second version drew into ordinary cacheable memory and memcpy'd the dirty
 * rows across, which brought rendering back to sane numbers at the cost of a
 * 2MB copy per frame - about 4.7ms for a partial frame and 7.5ms for a large
 * one, measured on this board.
 *
 * That copy existed only because the destination had to be memory Linux could
 * read. It no longer does: the VOP's Esmart3 window is now programmed from this
 * core (see evb7_amp_vop.c), and it will read any physical address, including
 * this core's own RAM. So the framebuffer is here, cacheable, and the copy is
 * gone. What replaces it is a cache clean of the dirty rows - which writes the
 * same bytes to DRAM that the memcpy's store half did, without the load half
 * and without a second pass over the data.
 *
 * Why two buffers
 * ----------------------------------------------------------------------------
 * With one buffer the VOP is reading the same memory the application is drawing
 * into, so a frame can reach the panel half-drawn. No amount of care on this
 * side fixes that; it needs somewhere else to draw. Two buffers were not
 * affordable while the framebuffer lived in the 4MB shared area - two 540x960
 * ARGB8888 frames are 4.15MB - and are trivially affordable in the 16MB of RAM
 * this core has.
 *
 * The framework's double-buffer support is driven from yres_virtual: fb.c
 * computes fbcount as yres_virtual/yres and sizes the pan queue to match, and
 * LVGL's fbdev driver looks for yres_virtual == 2*yres to decide to allocate a
 * second draw buffer. Both are satisfied by reporting one contiguous region of
 * twice the height.
 *
 * The division of labour between the two hooks
 * ----------------------------------------------------------------------------
 * FBIO_UPDATE means "these rows are drawn". FBIOPAN_DISPLAY means "show this
 * buffer". With one buffer they collapse into the same thing, which is why the
 * earlier version treated them identically; with two they genuinely differ, and
 * conflating them would either publish half-drawn frames or never publish at
 * all.
 *
 * So: update cleans the dirty rows out of the cache, and pan points the window
 * at a buffer and waits for the hardware to actually switch. The wait is the
 * part that makes double buffering real rather than nominal - see
 * evb7_amp_vop_wait_latch().
 *
 * Applications that only ever issue FBIO_UPDATE still work. The rows they touch
 * are in the buffer already on screen, and that is exactly the case update
 * publishes immediately - including the very first one, where publishing means
 * bringing the window up rather than retargeting it.
 *
 * That last clause was missing for a while, in the comment and in the code. The
 * takeover lived only in the pan path, so this paragraph was true of every
 * caller that had ever existed - all of them double-buffered - and false for the
 * first single-buffered one, which drew a frame into memory nothing was scanning
 * and showed a black panel while reporting thirty frames a second.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/video/fb.h>

#include "evb7_amp.h"
#include "evb7_amp_shm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The geometry comes from the shared-area header even though no pixels go
 * through there any more, because Linux reads the same numbers out of the
 * control block to convert touch coordinates into this framebuffer's space. One
 * definition keeps the forward and inverse scaling from drifting apart.
 *
 * The buffer size is derived here rather than taken from that header: how many
 * buffers there are and how they are laid out is this driver's business now that
 * nothing else looks at them.
 */

#define AMP_FB_WIDTH    AMP_SHM_WIDTH
#define AMP_FB_HEIGHT   AMP_SHM_HEIGHT
#define AMP_FB_STRIDE   AMP_SHM_STRIDE
#define AMP_FB_BUFSIZE  (AMP_FB_STRIDE * AMP_FB_HEIGHT)
#define AMP_FB_NBUFFERS 2

/* A page for the region as a whole. The buffers are read by the VOP's AXI
 * master, and while a linear-mode window only documents a 4-byte requirement,
 * the version of this that was verified on hardware ran from a page-aligned
 * address in the shared area; keeping that property removes a variable from the
 * comparison and costs nothing.
 *
 * The second buffer does not inherit page alignment, because the buffer size is
 * not a multiple of a page: 540 * 4 * 960 is 2073600, which is 1024 * 2025. It
 * cannot be padded to one either - LVGL locates the second buffer at exactly
 * yres * stride, so any gap would point it at the wrong place. 1024 is well
 * past what the hardware asks for, and being a multiple of the 64-byte cache
 * line is what matters for the other reason to align: cleaning one buffer can
 * never write back a line belonging to the other.
 */

#define AMP_FB_ALIGN    4096

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Both buffers, contiguous, so they can be reported as a single region of twice
 * the height. Cacheable ordinary RAM - that is the entire point of moving them
 * here from the shared area.
 *
 * From the heap rather than .bss, which is not about where the memory ends up -
 * both are in the same MT_NORMAL bank and both are flat-mapped, so a pointer is
 * a physical address either way. It is about the size of nuttx.bin. The linker
 * script places .initstack after .bss, and objcopy -O binary spans from the
 * first section with contents to the last, so a 4MB .bss array in between
 * becomes 4MB of zeroes in the image - the image went from 880KB to 5.15MB when
 * these were static. Taking them from the heap instead shrinks .bss by the same
 * amount, which moves g_idle_topstack down and gives the heap back exactly what
 * it is about to hand out.
 */

static uint8_t *g_fbmem;

/* Which buffer the VOP is scanning. Only ever changed by pandisplay, and read
 * by updatearea to decide whether the rows being reported are on screen.
 */

static unsigned int g_active_buf;

/* Timings are taken with up_perf_gettime() rather than clock_systime_ticks().
 *
 * The tick here is a millisecond (CONFIG_USEC_PER_TICK=1000) and a cache clean
 * of a few dirty rows is a long way under that, so accumulating tick
 * differences added zero every time and reported a total of zero however long
 * it really took. That is worse than no measurement: it looks like an answer.
 *
 * The totals are 64-bit even though up_perf_gettime() returns clock_t, which is
 * 32 bits here (CONFIG_SYSTEM_TIME64 is not set). On this target the counter's
 * unit is a nanosecond - up_perf_getfreq() reports 1000000000 - so a 32-bit
 * accumulator wraps after 4.29 seconds. That is comfortably more than one frame
 * and comfortably less than one reporting period: a single stall of a few
 * seconds inside a period was enough to wrap the frame total and report 11ms
 * per frame in the middle of a run that was steady at 33ms.
 *
 * Individual intervals stay in clock_t on purpose - unsigned 32-bit subtraction
 * gives the right answer across a counter wrap, as long as the interval itself
 * is under 4.29 seconds.
 */

static struct
{
  unsigned long updates;
  unsigned long rows;
  unsigned long max_rows;
  unsigned long pans;

  /* Nanoseconds, converted only when reported. */

  uint64_t      clean;
  uint64_t      wait;
  uint64_t      lvgl;    /* pan returned -> next update entered  */
  uint64_t      frame;   /* pan to pan                           */

  clock_t       reported;
} g_stats;

/* End of the last pan, in performance-counter units. Splits each frame into the
 * part this driver is responsible for and the part it is not, which is the only
 * way to tell "the display path is slow" from "the toolkit had nothing to do".
 */

static clock_t g_last_pan;

/* Previous pan, kept separately because g_last_pan is cleared once consumed -
 * an application that pans twice without an update in between must not have the
 * gap counted as toolkit time twice.
 */

static clock_t g_prev_pan;

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
   * byte ignored, which is what the VOP is programmed for (ARGB8888 with the
   * alpha taken from the window's global value, so the byte is not read).
   */

  .fmt     = FB_FMT_RGB32,
  .xres    = AMP_FB_WIDTH,
  .yres    = AMP_FB_HEIGHT,
  .nplanes = 1,
};

/* Not const: fbmem is filled in by up_fbinitialize(), which the framework calls
 * before it reads any of this.
 */

static struct fb_planeinfo_s g_planeinfo =
{
  .fblen        = AMP_FB_NBUFFERS * AMP_FB_BUFSIZE,
  .stride       = AMP_FB_STRIDE,
  .display      = 0,
  .bpp          = 32,
  .xres_virtual = AMP_FB_WIDTH,

  /* Twice the visible height is how both fb.c and LVGL are told there are two
   * buffers. fb.c divides it by yres to size the pan queue; LVGL compares it
   * against 2*yres to decide to allocate a second draw buffer.
   */

  .yres_virtual = AMP_FB_NBUFFERS * AMP_FB_HEIGHT,
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

static inline uintptr_t evb7_fb_bufaddr(unsigned int buf)
{
  return (uintptr_t)g_fbmem + (uintptr_t)buf * AMP_FB_BUFSIZE;
}

/****************************************************************************
 * Name: evb7_fb_clean_rows
 *
 * Description:
 *   Push a band of rows out of the data cache so the VOP can read them.
 *
 *   Rows rather than rectangles because a row is contiguous: a band is one
 *   range, while a rectangle would be one range per row. Cleaning the few
 *   untouched pixels either side of the damage is cheaper than the per-row
 *   overhead of skipping them.
 *
 *   Cleaning an already-clean line costs a cache lookup and no bus traffic, so
 *   over-cleaning is close to free. That is what lets pandisplay clean a whole
 *   buffer without tracking what updatearea already did.
 *
 * Input Parameters:
 *   y - first row, in the doubled coordinate space the framework uses for a
 *       two-buffer region, so 0..2*yres-1.
 *   h - number of rows.
 *
 ****************************************************************************/

static void evb7_fb_clean_rows(unsigned int y, unsigned int h)
{
  uintptr_t start;
  uintptr_t end;
  clock_t   t0;

  if (y >= AMP_FB_NBUFFERS * AMP_FB_HEIGHT || h == 0)
    {
      return;
    }

  if (h > AMP_FB_NBUFFERS * AMP_FB_HEIGHT - y)
    {
      h = AMP_FB_NBUFFERS * AMP_FB_HEIGHT - y;
    }

  start = (uintptr_t)g_fbmem + (uintptr_t)y * AMP_FB_STRIDE;
  end   = start + (uintptr_t)h * AMP_FB_STRIDE;

  t0 = up_perf_gettime();
  up_clean_dcache(start, end);
  g_stats.clean += (clock_t)(up_perf_gettime() - t0);
}

/****************************************************************************
 * Name: evb7_fb_us
 *
 * Description:
 *   Performance-counter units to microseconds.
 *
 ****************************************************************************/

static unsigned long evb7_fb_us(uint64_t elapsed)
{
  unsigned long freq = up_perf_getfreq();

  if (freq == 0)
    {
      return 0;
    }

  return (unsigned long)((elapsed * 1000000ull) / freq);
}

/****************************************************************************
 * Name: evb7_fb_report
 *
 * Description:
 *   Four numbers every couple of seconds, chosen so that the two things that
 *   can go wrong are separable.
 *
 *   rows says how much of the screen each frame actually redraws, which is the
 *   difference between "the toolkit is redrawing everything" and "the toolkit
 *   is redrawing a little and taking a long time over it" - the host side can
 *   measure its own work but cannot see this.
 *
 *   wait says how long this core spent blocked on the panel. It should be most
 *   of the frame time once rendering is fast enough to be limited by the
 *   display, and near zero when it is not, so it tells you which side the
 *   bottleneck is on without any guessing.
 *
 *   timeouts should be zero. Anything else means frames are being committed
 *   that never reach the screen, so the pipeline is not synchronised and the
 *   second buffer is not actually preventing tearing.
 *
 ****************************************************************************/

static void evb7_fb_report(void)
{
  unsigned long n;

  if (clock_systime_ticks() - g_stats.reported < MSEC2TICK(2000))
    {
      return;
    }

  /* Intervals, not frames: n frames have n-1 gaps between them, and the
   * timestamps are cleared at the end of this function so nothing is carried
   * across a reporting boundary.
   *
   * Getting this wrong made the numbers actively misleading rather than merely
   * imprecise. Dividing a total that included one cross-boundary gap by the
   * frame count reported 894ms of toolkit time in a period whose own frame
   * interval was 20ms - a breakdown three times larger than the total it was
   * breaking down. Two rounds of diagnosis were spent chasing a rendering cost
   * that did not exist, until LVGL's own performance monitor put the same frame
   * at 11ms.
   */

  n = g_stats.pans > 1 ? g_stats.pans - 1 : 1;

  /* Per frame, and split so that frame == lvgl + clean + wait + rounding.
   *
   * lvgl is everything between this driver returning from one pan and being
   * entered for the next update: rendering, the toolkit's own timers, and the
   * sleep in its main loop. It is not this driver's time, and printing it next
   * to the parts that are is what makes the two separable - a frame interval of
   * hundreds of milliseconds with single-digit clean and wait means the display
   * path is idle and waiting for work, not slow.
   */

  syslog(LOG_INFO,
         "[AMP] fb %lu frame(s): %lu us/frame = lvgl %lu + clean %lu + "
         "wait %lu | %lu rows avg (max %lu of %u), %lu flip(s), "
         "%lu timeout(s)\n",
         g_stats.pans,
         evb7_fb_us(g_stats.frame) / n,
         evb7_fb_us(g_stats.lvgl) / n,
         evb7_fb_us(g_stats.clean) / n,
         evb7_fb_us(g_stats.wait) / n,
         g_stats.updates ? g_stats.rows / g_stats.updates : 0,
         g_stats.max_rows, AMP_FB_HEIGHT,
         evb7_amp_vop_flips(), evb7_amp_vop_latch_timeouts());

  g_stats.reported = clock_systime_ticks();
  g_stats.updates  = 0;
  g_stats.pans     = 0;
  g_stats.rows     = 0;
  g_stats.max_rows = 0;
  g_stats.clean    = 0;
  g_stats.wait     = 0;
  g_stats.lvgl     = 0;
  g_stats.frame    = 0;

  /* Start the next period's intervals from its own first frame. Without this
   * the first gap measured is the one spanning this report - which includes
   * however long the application was idle, or stalled, before it - and that one
   * outlier is enough to dominate an average of a few dozen frames.
   */

  g_prev_pan = 0;
  g_last_pan = 0;
}

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

/****************************************************************************
 * Name: evb7_fb_getplaneinfo
 *
 * Description:
 *   The same region whatever is asked for.
 *
 *   pinfo->display is not honoured, deliberately. LVGL locates the second
 *   buffer by asking for display+1 and comparing the address it gets back with
 *   the one for display 0: equal addresses mean "one contiguous region", and it
 *   then mmaps at yres*stride, which is where the second buffer is. Returning a
 *   different address here would make it treat the two as separate regions and
 *   mmap at offset zero - the same buffer twice.
 *
 ****************************************************************************/

static int evb7_fb_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                                struct fb_planeinfo_s *pinfo)
{
  uint8_t display;

  if (vtable == NULL || pinfo == NULL || planeno != 0)
    {
      return -EINVAL;
    }

  display = pinfo->display;
  memcpy(pinfo, &g_planeinfo, sizeof(*pinfo));
  pinfo->display = display;
  return OK;
}

/****************************************************************************
 * Name: evb7_fb_updatearea
 *
 * Description:
 *   "These rows are drawn." Get them out of the cache.
 *
 *   Publishing happens here only when the rows are in the buffer already on
 *   screen, which is what a single-buffered application does - it draws into
 *   the visible frame and expects it to appear. A double-buffered one draws
 *   into the other buffer and follows up with FBIOPAN_DISPLAY, and publishing
 *   at this point would put a half-drawn frame on the panel.
 *
 *   Distinguishing the two by which buffer was touched, rather than by a
 *   configuration flag, means both work with no knowledge of who is calling.
 *
 ****************************************************************************/

#ifdef CONFIG_FB_UPDATE
static int evb7_fb_updatearea(struct fb_vtable_s *vtable,
                              const struct fb_area_s *area)
{
  unsigned int y = 0;
  unsigned int h = AMP_FB_NBUFFERS * AMP_FB_HEIGHT;
  unsigned int buf;

  /* Time spent outside this driver since the last frame was published. */

  if (g_last_pan != 0)
    {
      g_stats.lvgl += (clock_t)(up_perf_gettime() - g_last_pan);
      g_last_pan = 0;
    }

  if (area != NULL)
    {
      y = area->y;
      h = area->h;
    }

  evb7_fb_clean_rows(y, h);

  g_stats.updates++;
  g_stats.rows += h;
  if (h > g_stats.max_rows)
    {
      g_stats.max_rows = h;
    }

  buf = y >= AMP_FB_HEIGHT ? 1 : 0;
  if (buf == g_active_buf)
    {
      if (evb7_amp_vop_active())
        {
          evb7_amp_vop_flip(evb7_fb_bufaddr(buf));
        }
      else
        {
          /* First update of all, from a caller that never pans.
           *
           * This branch is the difference between the paragraph above being
           * true and being aspirational. Until it existed the takeover lived
           * only in the pan path, so an application that drew into the visible
           * buffer and asked for it to be shown - exactly the case that comment
           * describes - got its rows cleaned out of the cache and nothing else:
           * the window had never been programmed, so there was nothing scanning
           * the memory it had just written. On screen that is indistinguishable
           * from a dead framebuffer, and it stayed hidden because everything
           * that had ever drawn here double-buffered and panned.
           *
           * Safe to do from here for the reason it is safe from the pan path:
           * the takeover has to happen after Linux's modeset disables every
           * window in its mask, and an application drawing is necessarily long
           * after that.
           */

          evb7_amp_vop_takeover(evb7_fb_bufaddr(buf));
        }
    }

  evb7_fb_report();
  return OK;
}
#endif

/****************************************************************************
 * Name: evb7_fb_pandisplay
 *
 * Description:
 *   "Show this buffer." Point the window at it and wait for the hardware to
 *   switch, then release one slot of the framework's pan queue.
 *
 *   The whole buffer is cleaned rather than just what updatearea reported,
 *   because an application is entitled to pan without ever calling
 *   FBIO_UPDATE - the framebuffer example does exactly that. Cleaning lines
 *   that are already clean costs a lookup and no bus traffic, so the safe
 *   version is also close to the cheap one.
 *
 *   The wait is what makes the second buffer worth having: it returns once the
 *   VOP is reading this buffer, which is the point at which the other one is
 *   free to draw into. Without it the caller would immediately start drawing
 *   over the frame still being scanned.
 *
 *   fb_remove_paninfo() releases the queue slot that fb.c is about to fill for
 *   this pan. Nothing else consumes that queue here - there is no vsync
 *   interrupt to hang it off - and a queue that fills up stops poll() reporting
 *   POLLOUT, which is what LVGL waits on before rendering. So skipping this
 *   would stall the display after two frames.
 *
 ****************************************************************************/

static int evb7_fb_pandisplay(struct fb_vtable_s *vtable,
                              struct fb_planeinfo_s *pinfo)
{
  unsigned int buf = 0;
  uintptr_t    phys;
  clock_t      t0;
  clock_t      now;

  if (pinfo != NULL && pinfo->yoffset >= AMP_FB_HEIGHT)
    {
      buf = 1;
    }

  phys = evb7_fb_bufaddr(buf);

  evb7_fb_clean_rows(buf * AMP_FB_HEIGHT, AMP_FB_HEIGHT);

  /* First frame of all: bring the window up rather than just retargeting it.
   * The takeover cannot happen at bringup, because Linux's modeset on this
   * video port disables every window in its mask and that happens long after
   * this core has booted. See evb7_amp_vop_takeover().
   */

  if (!evb7_amp_vop_active())
    {
      evb7_amp_vop_takeover(phys);
    }
  else
    {
      evb7_amp_vop_flip(phys);
    }

  t0 = up_perf_gettime();
  evb7_amp_vop_wait_latch(phys);
  now = up_perf_gettime();
  g_stats.wait += (clock_t)(now - t0);

  /* Frame interval, measured pan to pan - the only number here that is a frame
   * rate rather than a cost.
   */

  if (g_prev_pan != 0)
    {
      g_stats.frame += (clock_t)(now - g_prev_pan);
    }

  g_prev_pan   = now;
  g_last_pan   = now;
  g_active_buf = buf;
  g_stats.pans++;

  if (vtable != NULL)
    {
      fb_remove_paninfo(vtable, FB_NO_OVERLAY);
    }

  evb7_fb_report();
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fbinitialize
 *
 * Description:
 *   Allocate the two framebuffers.
 *
 *   No hardware is touched here. The window this core drives cannot be
 *   programmed yet: Linux has not brought the video port up at this point in
 *   the boot, and its modeset when it does would undo anything set now. That
 *   happens on the first frame instead - see evb7_fb_pandisplay().
 *
 *   The framework may call this more than once, so it has to be idempotent.
 *
 ****************************************************************************/

int up_fbinitialize(int display)
{
  if (display != 0)
    {
      return -EINVAL;
    }

  if (g_fbmem == NULL)
    {
      g_fbmem = kmm_memalign(AMP_FB_ALIGN,
                             AMP_FB_NBUFFERS * AMP_FB_BUFSIZE);
      if (g_fbmem == NULL)
        {
          syslog(LOG_ERR, "[AMP] fb: no memory for %u buffers of %u bytes\n",
                 AMP_FB_NBUFFERS, AMP_FB_BUFSIZE);
          return -ENOMEM;
        }

      g_planeinfo.fbmem = g_fbmem;
    }

  return OK;
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
 *   Register /dev/fb0 on top of this core's two framebuffers.
 *
 ****************************************************************************/

int evb7_amp_fb_init(void)
{
  int ret = fb_register(0, 0);

  syslog(LOG_INFO,
         "[AMP] fb %s: %ux%u %ubpp x%u at 0x%08lx/0x%08lx, scanned out here\n",
         ret >= 0 ? "/dev/fb0 registered" : "registration FAILED",
         AMP_FB_WIDTH, AMP_FB_HEIGHT, 32, AMP_FB_NBUFFERS,
         (unsigned long)evb7_fb_bufaddr(0),
         (unsigned long)evb7_fb_bufaddr(1));

  return ret;
}
