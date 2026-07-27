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

/* Shared-frame ground work: draw into memory Linux can read, and say so over
 * rpmsg.
 *
 * This is the foundation under three separate things that all want the same
 * plumbing - putting a NuttX-drawn image on the panel, receiving camera frames
 * captured by Linux, and handing tensors to Linux for NPU inference. All three
 * are the same problem: bulk data through shared memory, with rpmsg carrying
 * only the notice. Building and proving that layer on its own keeps the
 * unverified parts down to one at a time, which is the opposite of how the
 * mailbox1 attempt went.
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

/* The control block and the buffers are just fixed addresses in the carveout.
 * The mapping is flat, so a physical address is also this core's pointer.
 */

static struct amp_shm_ctrl_s * const g_ctrl =
  (struct amp_shm_ctrl_s *)(uintptr_t)(AMP_SHM_BASE + AMP_SHM_HDR_OFFSET);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: amp_shm_buffer
 ****************************************************************************/

static uint32_t *amp_shm_buffer(unsigned int index)
{
  return (uint32_t *)(uintptr_t)(AMP_SHM_BASE + AMP_SHM_BUF_OFFSET(index));
}

/****************************************************************************
 * Name: amp_shm_render
 *
 * Description:
 *   Fill a buffer with a pattern derived from the frame number and return its
 *   checksum.
 *
 *   The pattern deliberately depends on x, y and the sequence number, so that a
 *   stale buffer, a buffer written at the wrong offset and a buffer read through
 *   the wrong mapping all produce a different sum rather than accidentally
 *   agreeing. Linux never needs to know the formula - it sums the same bytes
 *   through its own mapping, and the two sums either match or they do not.
 *
 ****************************************************************************/

static uint32_t amp_shm_render(unsigned int index, uint32_t seq)
{
  uint32_t *buf = amp_shm_buffer(index);
  uint32_t sum = 0;
  unsigned int x;
  unsigned int y;

  for (y = 0; y < AMP_SHM_HEIGHT; y++)
    {
      for (x = 0; x < AMP_SHM_WIDTH; x++)
        {
          uint32_t pix = 0xff000000u |
                         ((x + seq) & 0xff) << 16 |
                         ((y + seq) & 0xff) << 8 |
                         ((x ^ y) & 0xff);

          *buf++ = pix;
          sum += pix;
        }
    }

  return sum;
}

/****************************************************************************
 * Name: amp_shm_publish
 *
 * Description:
 *   Draw the next frame, describe it in the control block, and tell Linux.
 *
 ****************************************************************************/

static void amp_shm_publish(struct amp_shm_dev_s *dev)
{
  struct amp_shm_msg_s msg;
  uint32_t seq = g_ctrl->frame_seq + 1;
  unsigned int index = seq % AMP_SHM_NBUFFERS;
  uint32_t sum;

  sum = amp_shm_render(index, seq);

  /* Order matters here. The pixels have to be visible to the other core before
   * anything advertises them, otherwise Linux can be told about a frame it has
   * not fully received. The area is non-cacheable, so no flushing is needed,
   * but the stores still have to be ordered against the ones below.
   */

  UP_DMB();

  g_ctrl->ready_index = index;
  g_ctrl->ready_sum   = sum;
  g_ctrl->frame_seq   = seq;

  UP_DMB();

  msg.cmd   = AMP_SHM_CMD_READY;
  msg.seq   = seq;
  msg.index = index;
  msg.sum   = sum;

  rpmsg_send(&dev->ept, &msg, sizeof(msg));

  syslog(LOG_INFO, "[AMP] shm frame %lu -> buf%u sum 0x%08lx\n",
         (unsigned long)seq, index, (unsigned long)sum);
}

/****************************************************************************
 * Name: amp_shm_ept_cb
 ****************************************************************************/

static int amp_shm_ept_cb(struct rpmsg_endpoint *ept, void *data,
                          size_t len, uint32_t src, void *priv)
{
  struct amp_shm_dev_s *dev = ept->priv;
  struct amp_shm_msg_s *msg = data;

  if (len < sizeof(*msg))
    {
      syslog(LOG_WARNING, "[AMP] shm short message (%zu bytes)\n", len);
      return 0;
    }

  switch (msg->cmd)
    {
      case AMP_SHM_CMD_RENDER:
        amp_shm_publish(dev);
        break;

      case AMP_SHM_CMD_ACK:

        /* What Linux read back, checked against what was written. A mismatch
         * here is the interesting outcome: it means the two cores disagree
         * about the contents of the same physical memory, which points at the
         * mapping or at ordering rather than at anything above.
         */

        syslog(LOG_INFO,
               "[AMP] shm ack frame %lu buf%lu sum 0x%08lx -> %s\n",
               (unsigned long)msg->seq, (unsigned long)msg->index,
               (unsigned long)msg->sum,
               msg->sum == g_ctrl->ready_sum ? "MATCH" : "MISMATCH");
        break;

      default:
        syslog(LOG_WARNING, "[AMP] shm unknown cmd %lu\n",
               (unsigned long)msg->cmd);
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
  unsigned int i;
  int ret;

  /* Refuse to run if the geometry would not fit, rather than quietly writing
   * past the carveout into whatever follows it.
   */

  if (AMP_SHM_BUF_OFFSET(AMP_SHM_NBUFFERS) > AMP_SHM_SIZE)
    {
      syslog(LOG_ERR, "[AMP] shm geometry needs %u bytes, area is %u\n",
             (unsigned int)AMP_SHM_BUF_OFFSET(AMP_SHM_NBUFFERS),
             (unsigned int)AMP_SHM_SIZE);
      return -ENOSPC;
    }

  /* Publish the layout before the magic, so Linux cannot find the magic and
   * read geometry that has not been written yet.
   */

  memset(g_ctrl, 0, sizeof(*g_ctrl));

  g_ctrl->version  = AMP_SHM_VERSION;
  g_ctrl->width    = AMP_SHM_WIDTH;
  g_ctrl->height   = AMP_SHM_HEIGHT;
  g_ctrl->stride   = AMP_SHM_STRIDE;
  g_ctrl->bpp      = AMP_SHM_BPP;
  g_ctrl->nbuffers = AMP_SHM_NBUFFERS;
  g_ctrl->bufsize  = AMP_SHM_BUFSIZE;

  for (i = 0; i < AMP_SHM_NBUFFERS; i++)
    {
      g_ctrl->bufoffset[i] = AMP_SHM_BUF_OFFSET(i);
    }

  UP_DMB();

  g_ctrl->magic = AMP_SHM_MAGIC;

  UP_DMB();

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
         "[AMP] shm ready: %ux%u x%u bpp, %u buffers, ctrl at 0x%08x "
         "(%u KB used)\n",
         AMP_SHM_WIDTH, AMP_SHM_HEIGHT, AMP_SHM_BPP, AMP_SHM_NBUFFERS,
         (unsigned int)(AMP_SHM_BASE + AMP_SHM_HDR_OFFSET),
         (unsigned int)(AMP_SHM_BUF_OFFSET(AMP_SHM_NBUFFERS) / 1024));

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
