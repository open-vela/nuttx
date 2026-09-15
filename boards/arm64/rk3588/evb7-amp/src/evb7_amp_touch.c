/****************************************************************************
 * boards/arm64/rk3588/evb7-amp/src/evb7_amp_touch.c
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

/* /dev/input0 fed by touch events Linux forwards over rpmsg.
 *
 * The panel's touch controller sits on i2c5 and its interrupt is a pin in GPIO
 * bank 3. Neither can be taken over cleanly while Linux runs: the bank shares
 * one interrupt line across all of its pins, and the Type-C power delivery
 * controller's interrupt is on that same bank, so claiming it here would break
 * charging. Rather than take the hardware, this core consumes events that Linux
 * has already decoded.
 *
 * That makes this a lower half with no hardware underneath it - it exists to
 * turn messages into the shape the touchscreen framework expects, so that
 * ordinary applications, LVGL and NX included, work unmodified against
 * /dev/input0.
 *
 * "The shape the framework expects" turned out to include a size invariant that
 * is nowhere stated: maxpoint must equal the npoints every sample carries, or
 * readers silently lose events. See the comment on AMP_TOUCH_MAXPOINT - having no
 * hardware here does not exempt this driver from behaving like the ones that do.
 *
 * The consequence worth knowing: touch depends on Linux being alive. If Linux
 * stops forwarding, this device simply goes quiet, and there is nothing here
 * that can tell the difference between "nobody is touching the screen" and
 * "the other core stopped talking".
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <string.h>
#include <syslog.h>

#include <nuttx/input/touchscreen.h>

#include "evb7_amp.h"
#include "evb7_amp_shm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AMP_TOUCH_PATH     "/dev/input0"

/* One contact, and this has to equal what every sample actually carries.
 *
 * There is an unwritten contract in this framework: a reader may assume every
 * sample is SIZEOF_TOUCH_SAMPLE_S(maxpoint) bytes. The upper half queues
 * SIZEOF_TOUCH_SAMPLE_S(sample->npoints) bytes (touchscreen_upper.c:374), read()
 * returns whatever is available (:237), and LVGL's backend compares the byte
 * count against the maxpoint size for equality - discarding the data it has just
 * consumed when they differ (lv_nuttx_touchscreen.c:281).
 *
 * Declaring 5 while reporting 1 therefore lost events rather than merely wasting
 * space. A single-point sample is 32 bytes and maxpoint 5 asks for 128, so one,
 * two or three queued contacts all read short and were thrown away; only when a
 * fourth arrived did read() return 128 bytes, and then just the first sample in
 * that buffer was parsed and the other 96 bytes dropped. Three of every four
 * contacts never reached the toolkit. A tap survived because the press is first
 * in the buffer; a swipe of thirty-odd moves arrived as eight or nine widely
 * spaced points, which is not a swipe.
 *
 * Both in-tree drivers that set maxpoint keep the invariant: mouse_touch.c uses 1
 * for both, and goldfish_events.c assigns touchsample->npoints =
 * touchlower.maxpoint outright. Supporting more than one contact here means
 * raising this number *and* aggregating the per-contact messages Linux sends into
 * one multi-point sample - one without the other is what this comment exists to
 * prevent.
 */

#define AMP_TOUCH_MAXPOINT 1

/* Depth of the upper half's sample queue.
 *
 * Deep enough to hold a whole gesture, and that is not generosity. The upper half
 * discards the oldest sample when the queue is full (circbuf_overwrite,
 * touchscreen_upper.c:374), and the oldest sample is the press - the one event a
 * consumer cannot do without. A swipe on this panel is one press, thirty to forty
 * position updates and one release, all inside a few hundred milliseconds; with a
 * queue of eight, a reader that pauses for one render loses the press and keeps
 * the moves, so it sees a finger that was never put down. Nothing reports an
 * error, and gestures simply stop working.
 *
 * At one contact per sample this is 2KB of the fifteen megabytes free here.
 */

#define AMP_TOUCH_NBUFFER  64

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct touch_lowerhalf_s g_touch_lower;
static bool g_touch_ready;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: evb7_amp_touch_event
 *
 * Description:
 *   Turn one forwarded event into a touchscreen sample.
 *
 ****************************************************************************/

void evb7_amp_touch_event(const struct amp_touch_msg_s *msg)
{
  struct touch_sample_s sample;

  if (!g_touch_ready)
    {
      return;
    }

  memset(&sample, 0, sizeof(sample));

  sample.npoints        = 1;
  sample.point[0].id    = msg->id;
  sample.point[0].x     = (int16_t)msg->x;
  sample.point[0].y     = (int16_t)msg->y;
  sample.point[0].timestamp = touch_get_time();

  /* The position is meaningful on release too - an application that only learns
   * "a contact ended" without knowing where cannot implement a tap. So
   * TOUCH_POS_VALID is set on all three states, and the forwarder repeats the
   * last coordinates on the up event for that reason.
   */

  switch (msg->state)
    {
      case AMP_TOUCH_DOWN:
        sample.point[0].flags = TOUCH_DOWN;
        break;

      case AMP_TOUCH_MOVE:
        sample.point[0].flags = TOUCH_MOVE;
        break;

      case AMP_TOUCH_UP:
        sample.point[0].flags = TOUCH_UP;
        break;

      default:
        return;
    }

  sample.point[0].flags |= TOUCH_ID_VALID | TOUCH_POS_VALID;

  if (msg->pressure != 0)
    {
      sample.point[0].pressure = msg->pressure;
      sample.point[0].flags   |= TOUCH_PRESSURE_VALID;
    }

  touch_event(g_touch_lower.priv, &sample);
}

/****************************************************************************
 * Name: evb7_amp_touch_init
 *
 * Description:
 *   Register /dev/input0. Nothing to initialise in hardware; the events arrive
 *   through the shared-frame endpoint, so this only has to exist before the
 *   first message can be delivered.
 *
 ****************************************************************************/

int evb7_amp_touch_init(void)
{
  int ret;

  g_touch_lower.maxpoint = AMP_TOUCH_MAXPOINT;

  ret = touch_register(&g_touch_lower, AMP_TOUCH_PATH, AMP_TOUCH_NBUFFER);
  if (ret < 0)
    {
      syslog(LOG_ERR, "[AMP] touch registration FAILED: %d\n", ret);
      return ret;
    }

  g_touch_ready = true;

  syslog(LOG_INFO, "[AMP] touch %s registered, up to %u contact(s)\n",
         AMP_TOUCH_PATH, AMP_TOUCH_MAXPOINT);

  return OK;
}
