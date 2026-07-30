/****************************************************************************
 * boards/arm64/rk3588/evb7-amp/src/evb7_amp_shm.c
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

/* The rpmsg endpoint that carries touch, and the control block that tells Linux
 * what coordinate system to send it in.
 *
 * This started out as the shared-frame layer: bulk pixels through the carveout
 * with rpmsg carrying only the notice that a buffer was finished. That was the
 * right shape while Linux composited what this core drew, and it is not what
 * happens now - the VOP window is programmed from here and scans out of this
 * core's own RAM, so no part of the display path comes through this file.
 *
 * What is left is the one direction that still needs a channel. This core cannot
 * read the touch controller itself: it is on i2c5 with its interrupt in GPIO
 * bank 3, and that bank's single interrupt line is shared with the Type-C power
 * delivery controller's, so claiming it would break charging. Linux decodes the
 * contacts and forwards them here.
 *
 * The frame protocol was deleted rather than left dormant. Two plausible-looking
 * display paths with only one of them real is a worse state than one path.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <string.h>

#include <nuttx/rptun/rptun.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>

#include "evb7_amp.h"
#include "evb7_amp_shm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The first name-service announce is lost if Linux's rpmsg host is not online
 * yet, and there is no second attempt unless one is arranged. This core reaches
 * this code around half a second after reset while the host comes up at about
 * three and a half, so losing it is the normal case rather than the exception -
 * the console channel needed exactly the same treatment.
 *
 * Re-announcing is harmless once bound: the check below stops as soon as the
 * peer's address is known.
 */

#define AMP_SHM_REANNOUNCE_MAX     30
#define AMP_SHM_REANNOUNCE_PERIOD  MSEC2TICK(1000)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct amp_shm_dev_s
{
  struct rpmsg_endpoint    ept;
  struct work_s            announce_work;
  int                      announce_cnt;
  char                     cpuname[RPMSG_NAME_SIZE + 1];
  bool                     bound;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct amp_shm_dev_s g_amp_shm;

/* The control block is just a fixed address in the carveout. The mapping is
 * flat, so a physical address is also this core's pointer.
 */

static struct amp_shm_ctrl_s * const g_ctrl =
  (struct amp_shm_ctrl_s *)(uintptr_t)(AMP_SHM_BASE + AMP_SHM_HDR_OFFSET);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: amp_shm_ept_cb
 ****************************************************************************/

static int amp_shm_ept_cb(struct rpmsg_endpoint *ept, void *data,
                          size_t len, uint32_t src, void *priv)
{
  struct amp_shm_hdr_s *hdr = data;

  if (len < sizeof(*hdr))
    {
      syslog(LOG_WARNING, "[AMP] shm short message (%zu bytes)\n", len);
      return 0;
    }

  switch (hdr->cmd)
    {
      case AMP_SHM_CMD_HELLO:

        /* Only a trace that the channel came up. It used to matter for its side
         * effect - the rpmsg layer learns the peer's address from the first
         * inbound message, and without one this core could not send frame
         * notifications - but nothing is sent from here any more.
         */

        syslog(LOG_INFO, "[AMP] shm peer at 0x%08lx\n", (unsigned long)src);
        break;

#ifdef CONFIG_INPUT_TOUCHSCREEN
      case AMP_SHM_CMD_TOUCH:

        /* The same 16 bytes, seen in full rather than as just a header. */

        evb7_amp_touch_event((const struct amp_touch_msg_s *)data);
        break;
#endif

      case AMP_SHM_CMD_DETECT:

        /* Counted, not read. Same reasoning as the camera notification and more
         * so: a set of boxes is read by whoever is drawing, at the moment it
         * draws, and doing it here would put a copy on the rpmsg receive path
         * for a consumer that may not exist.
         */

        evb7_amp_det_notify((const struct amp_det_msg_s *)data);
        break;

      case AMP_SHM_CMD_CAMERA:

        /* Only a wakeup. The frame is not read here: this runs on the rpmsg
         * receive path, and copying half a megabyte on it would hold up every
         * other message on the channel - touch included - for as long as the copy
         * takes. The consumer reads the descriptor itself, which also means a
         * notification arriving with no consumer running costs nothing.
         */

        evb7_amp_cam_notify((const struct amp_cam_msg_s *)data);
        break;

      default:

        /* Includes the retired frame commands - 1, 2 and 3 - which is the point
         * of not reusing those numbers: a stale binary on the other side says so
         * instead of being misread as touch.
         */

        syslog(LOG_WARNING, "[AMP] shm unknown cmd %lu\n",
               (unsigned long)hdr->cmd);
        break;
    }

  return 0;
}

/****************************************************************************
 * Name: amp_shm_ns_bound
 ****************************************************************************/

static void amp_shm_ns_bound(struct rpmsg_endpoint *ept)
{
  struct amp_shm_dev_s *dev = ept->priv;

  dev->bound = true;
  syslog(LOG_INFO, "[AMP] shm endpoint bound by Linux\n");
}

/****************************************************************************
 * Name: amp_shm_announce_work
 ****************************************************************************/

static void amp_shm_announce_work(void *arg)
{
  struct amp_shm_dev_s *dev = arg;

  if (dev->ept.dest_addr != RPMSG_ADDR_ANY || dev->announce_cnt <= 0)
    {
      return;
    }

  dev->announce_cnt--;

  rpmsg_send_ns_message(&dev->ept, RPMSG_NS_CREATE);

  work_queue(HPWORK, &dev->announce_work, amp_shm_announce_work,
             dev, AMP_SHM_REANNOUNCE_PERIOD);
}

/****************************************************************************
 * Name: amp_shm_device_created
 ****************************************************************************/

static void amp_shm_device_created(struct rpmsg_device *rdev, void *priv)
{
  struct amp_shm_dev_s *dev = priv;

  if (strcmp(dev->cpuname, rpmsg_get_cpuname(rdev)) != 0)
    {
      return;
    }

  dev->ept.priv        = dev;
  dev->ept.ns_bound_cb = amp_shm_ns_bound;

  rpmsg_create_ept(&dev->ept, rdev, AMP_SHM_EPT_NAME,
                   RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                   amp_shm_ept_cb, NULL);

  dev->announce_cnt = AMP_SHM_REANNOUNCE_MAX;
  work_queue(HPWORK, &dev->announce_work, amp_shm_announce_work,
             dev, AMP_SHM_REANNOUNCE_PERIOD);
}

/****************************************************************************
 * Name: amp_shm_device_destroy
 ****************************************************************************/

static void amp_shm_device_destroy(struct rpmsg_device *rdev, void *priv)
{
  struct amp_shm_dev_s *dev = priv;

  if (strcmp(dev->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      work_cancel(HPWORK, &dev->announce_work);
      rpmsg_destroy_ept(&dev->ept);
      dev->bound = false;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: evb7_amp_shm_init
 ****************************************************************************/

int evb7_amp_shm_init(const char *cpuname)
{
  struct amp_shm_dev_s *dev = &g_amp_shm;
  int ret;

  /* Publish the geometry before the magic, so Linux cannot find the magic and
   * read fields that have not been written yet.
   */

  memset(g_ctrl, 0, sizeof(*g_ctrl));

  g_ctrl->version = AMP_SHM_VERSION;
  g_ctrl->width   = AMP_SHM_WIDTH;
  g_ctrl->height  = AMP_SHM_HEIGHT;
  g_ctrl->stride  = AMP_SHM_STRIDE;
  g_ctrl->bpp     = AMP_SHM_BPP;

  UP_DMB();

  g_ctrl->magic = AMP_SHM_MAGIC;

  UP_DMB();

  /* Arm the camera consumer before the channel exists, so a notification
   * arriving the instant Linux starts streaming has somewhere to go. Failure is
   * not fatal: touch is the older and more important half of this channel, and a
   * camera that will not start should not take it down too.
   */

  ret = evb7_amp_cam_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "[AMP] cam init failed %d - camera disabled\n", ret);
    }

  strlcpy(dev->cpuname, cpuname, sizeof(dev->cpuname));

  ret = rpmsg_register_callback(dev,
                               amp_shm_device_created,
                               amp_shm_device_destroy,
                               NULL,
                               NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "[AMP] shm rpmsg_register_callback failed %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO,
         "[AMP] shm ready: touch channel, geometry %ux%u x%u bpp, "
         "ctrl at 0x%08x\n",
         AMP_SHM_WIDTH, AMP_SHM_HEIGHT, AMP_SHM_BPP,
         (unsigned int)(AMP_SHM_BASE + AMP_SHM_HDR_OFFSET));

  /* Read the geometry back through the same mapping. The first attempt at this
   * had the control block at offset 0, where three of its words were being
   * overwritten by something else; the values looked plausible enough in a hex
   * dump that only comparing them against what was written made it obvious.
   */

  if (g_ctrl->height != AMP_SHM_HEIGHT || g_ctrl->stride != AMP_SHM_STRIDE ||
      g_ctrl->bpp != AMP_SHM_BPP || g_ctrl->width != AMP_SHM_WIDTH)
    {
      syslog(LOG_ERR,
             "[AMP] shm control block did not survive: %lux%lu x%lu, "
             "stride %lu - something else is writing here\n",
             (unsigned long)g_ctrl->width, (unsigned long)g_ctrl->height,
             (unsigned long)g_ctrl->bpp, (unsigned long)g_ctrl->stride);
    }

  return OK;
}
