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

#include <nuttx/video/fb.h>

#include "evb7_amp.h"
#include "evb7_amp_shm.h"

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
  .fbmem        = (void *)(uintptr_t)(AMP_SHM_BASE +
                                      AMP_SHM_BUF_OFFSET(AMP_SHM_FB_INDEX)),
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

static int evb7_fb_pandisplay(struct fb_vtable_s *vtable,
                              struct fb_planeinfo_s *pinfo)
{
  evb7_amp_shm_flush();
  return OK;
}

#ifdef CONFIG_FB_UPDATE
static int evb7_fb_updatearea(struct fb_vtable_s *vtable,
                              const struct fb_area_s *area)
{
  evb7_amp_shm_flush();
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

  syslog(LOG_INFO, "[AMP] fb %s: %ux%u %ubpp at %p\n",
         ret >= 0 ? "/dev/fb0 registered" : "registration FAILED",
         AMP_SHM_WIDTH, AMP_SHM_HEIGHT, AMP_SHM_BPP * 8,
         g_planeinfo.fbmem);

  return ret;
}
