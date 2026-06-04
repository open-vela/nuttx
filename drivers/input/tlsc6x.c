/****************************************************************************
 * drivers/input/tlsc6x.c
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

#include <sys/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/atomic.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/tlsc6x.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TLSC6X_MAX_POINTS    5
#define TLSC6X_PACKET_REG    0x00
#define TLSC6X_PACKET_LEN    (3 + TLSC6X_MAX_POINTS * 6)  /* hdr + 5 * 6 */

/* Per-finger event field nibble values (vendor protocol) */

#define TLSC6X_EVT_DOWN      0
#define TLSC6X_EVT_UP        4
#define TLSC6X_EVT_MOVE      8

/* (buf[6i+3] & 0xC0) == 0xC0 marks a slot as invalid in the report. */

#define TLSC6X_INVALID_MASK  0xc0

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tlsc6x_dev_s
{
  struct touch_lowerhalf_s   lower;        /* Upper-half binding */
  FAR struct i2c_master_s   *i2c;          /* I2C bus */
  FAR const struct tlsc6x_config_s *cfg;   /* Per-board config */

  /* Resolved transform parameters (config overrides Kconfig defaults) */

  bool      exchange_xy;
  bool      revert_x;
  bool      revert_y;
  uint16_t  max_x;
  uint16_t  max_y;

  /* Worker plumbing.  The lock serialises the I2C transaction; the
   * worker-only fields below (active_mask, stat_*, rxbuf) are guarded
   * implicitly because tlsc6x_parse() runs only from the worker.  The
   * ISR touches int_pending only via atomic ops, never the lock.
   */

  struct work_s  work;                     /* LPWORK item */
  mutex_t        lock;                     /* Serialise I2C access */
  atomic_t       int_pending;              /* INT seen, worker queued */

  /* Bitmask of finger ids active in the previous frame.  Bits cleared
   * this frame produce TOUCH_UP events for ids the IC stopped reporting
   * without a 4-event (some firmware revisions skip UP entirely).
   * Worker-only.
   */

  uint8_t        active_mask;

#ifdef CONFIG_INPUT_TLSC6X_STATS
  /* Pollution statistics: separate "0xFF inside a valid slot" (real
   * corruption signal) from "whole 6-byte slot is 0xFF" (legitimate IC
   * INVALID_MASK encoding).  Worker-only; gated by Kconfig because the
   * periodic syslog dump is noisy on a healthy panel.
   */

  uint32_t       stat_total;
  uint32_t       stat_pollution;
  uint32_t       stat_invalid;
#endif

  uint8_t        rxbuf[TLSC6X_PACKET_LEN]; /* I2C RX scratch */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  tlsc6x_read_packet(FAR struct tlsc6x_dev_s *priv);
static void tlsc6x_parse(FAR struct tlsc6x_dev_s *priv);
static int  tlsc6x_isr(int irq, FAR void *context, FAR void *arg);
static void tlsc6x_worker(FAR void *arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Issue a write-then-read on the IC: write a single zero byte to set the
 * IC's internal pointer to 0x00, then read TLSC6X_PACKET_LEN bytes.
 */

static int tlsc6x_read_packet(FAR struct tlsc6x_dev_s *priv)
{
  uint8_t reg = TLSC6X_PACKET_REG;
  struct i2c_msg_s msgv[2] =
  {
    {
      .frequency = priv->cfg->frequency,
      .addr      = priv->cfg->address,
      .flags     = 0,
      .buffer    = &reg,
      .length    = 1,
    },
    {
      .frequency = priv->cfg->frequency,
      .addr      = priv->cfg->address,
      .flags     = I2C_M_READ,
      .buffer    = priv->rxbuf,
      .length    = TLSC6X_PACKET_LEN,
    }
  };

  return I2C_TRANSFER(priv->i2c, msgv, 2);
}

/* Parse a TLSC6X_PACKET_LEN-byte (33-byte) report, apply the resolved
 * transform, and push a single touch_sample_s with all valid points.
 * TOUCH_UP for fingers that disappeared since the last frame is
 * synthesised from the active_mask bookkeeping.
 */

static void tlsc6x_parse(FAR struct tlsc6x_dev_s *priv)
{
  FAR const uint8_t *buf = priv->rxbuf;
  uint8_t  prev_mask = priv->active_mask;
  uint8_t  this_mask = 0;
  uint8_t  count;
  FAR struct touch_sample_s *sample;
  uint8_t  storage[SIZEOF_TOUCH_SAMPLE_S(TLSC6X_MAX_POINTS * 2)];
  int      npoints = 0;
  int      i;
#ifdef CONFIG_INPUT_TLSC6X_STATS
  int      j;
  bool     frame_has_pollution;
  bool     frame_has_invalid;
#endif

  /* Header sanity: bits 7..3 of buf[2] are reserved-zero on a clean
   * report; bits 2..0 carry the touch count (0..5).  Bytes 0 and 1 are
   * status / soft-button bytes that must also stay zero on a panel with
   * no soft buttons.  When the IC is in transition (just woken, mid-
   * scan, NACKed) it returns garbage like "00 ff ff ..." — vendor BSP
   * dodges this by only reading slot 0, but we walk multiple slots so
   * we must reject the whole frame.
   */

  if (buf[0] != 0 || buf[1] != 0 || (buf[2] & 0xf8) != 0)
    {
      return;
    }

  count = buf[2] & 0x07;
  if (count > TLSC6X_MAX_POINTS)
    {
      return;
    }

  sample = (FAR struct touch_sample_s *)storage;
  memset(storage, 0, sizeof(storage));

#ifdef CONFIG_INPUT_TLSC6X_STATS
  /* Classify each frame as "pollution" (0xFF appearing inside a slot
   * that did NOT match INVALID_MASK) vs "invalid" (slot legitimately
   * marked empty by IC).  Periodic dump lets the user see real
   * pollution rate without spamming.
   */

  priv->stat_total++;
  frame_has_pollution = false;
  frame_has_invalid = false;

  for (i = 0; i < count; i++)
    {
      FAR const uint8_t *p = &buf[6 * i + 3];

      if ((p[0] & TLSC6X_INVALID_MASK) == TLSC6X_INVALID_MASK)
        {
          frame_has_invalid = true;
          continue;
        }

      for (j = 0; j < 6; j++)
        {
          if (p[j] == 0xff)
            {
              frame_has_pollution = true;
              break;
            }
        }
    }

  if (frame_has_pollution)
    {
      priv->stat_pollution++;
    }

  if (frame_has_invalid)
    {
      priv->stat_invalid++;
    }

  if ((priv->stat_total % 200) == 0)
    {
      syslog(LOG_INFO,
             "tlsc6x stat: total=%lu pollution=%lu invalid=%lu\n",
             (unsigned long)priv->stat_total,
             (unsigned long)priv->stat_pollution,
             (unsigned long)priv->stat_invalid);
    }
#endif

  /* Walk only the first 'count' slots — IC firmware on this panel
   * does not set the 0xc0 invalid marker on stale slots, so the
   * vendor-style "skip invalid" fallback would treat residual
   * register values as live fingers.
   */

  for (i = 0; i < count; i++)
    {
      FAR const uint8_t *p = &buf[6 * i + 3];
      uint16_t x;
      uint16_t y;
      uint8_t  evt;
      uint8_t  id;
      uint8_t  pressure;

      if ((p[0] & TLSC6X_INVALID_MASK) == TLSC6X_INVALID_MASK)
        {
          continue;
        }

      x        = ((uint16_t)(p[0] & 0x0f) << 8) | p[1];
      y        = ((uint16_t)(p[2] & 0x0f) << 8) | p[3];
      evt      = (p[0] >> 4) & 0x0f;
      id       = (p[2] >> 4) & 0x0f;
      pressure = p[4];

      /* The IC's id nibble is 4 bits wide but valid finger ids are
       * always < TLSC6X_MAX_POINTS.  Some firmware revisions put junk
       * in the upper bits during stale slots; drop those here so the
       * 1u<<id shift below stays in [0, MAX_POINTS).
       */

      if (id >= TLSC6X_MAX_POINTS)
        {
          continue;
        }

      if (priv->exchange_xy)
        {
          uint16_t tmp = x;
          x = y;
          y = tmp;
        }

      if (priv->revert_x)
        {
          x = priv->max_x - x;
        }

      if (priv->revert_y)
        {
          y = priv->max_y - y;
        }

      sample->point[npoints].id       = id;
      sample->point[npoints].x        = x;
      sample->point[npoints].y        = y;
      sample->point[npoints].pressure = pressure;
      sample->point[npoints].flags    = TOUCH_POS_VALID |
                                        TOUCH_PRESSURE_VALID |
                                        TOUCH_ID_VALID;

      if (evt == TLSC6X_EVT_UP)
        {
          sample->point[npoints].flags |= TOUCH_UP;
        }
      else if (evt == TLSC6X_EVT_MOVE)
        {
          sample->point[npoints].flags |= TOUCH_MOVE;
          this_mask |= (1u << id);
        }
      else
        {
          /* Treat any other event (DOWN=0, or vendor quirks) as DOWN. */

          sample->point[npoints].flags |= TOUCH_DOWN;
          this_mask |= (1u << id);
        }

      npoints++;
    }

  /* Synthesise UP for ids that vanished without a 4-event. */

  for (i = 0; i < TLSC6X_MAX_POINTS; i++)
    {
      uint8_t bit = 1u << i;

      if ((prev_mask & bit) && !(this_mask & bit))
        {
          sample->point[npoints].id    = i;
          sample->point[npoints].flags = TOUCH_UP | TOUCH_ID_VALID;
          npoints++;
        }
    }

  priv->active_mask = this_mask;

  if (npoints == 0)
    {
      return;
    }

  sample->npoints = npoints;
  touch_event(priv->lower.priv, sample);
}

static int tlsc6x_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct tlsc6x_dev_s *priv = (FAR struct tlsc6x_dev_s *)arg;

  UNUSED(irq);
  UNUSED(context);

  /* Coalesce edges that arrive while a worker is already queued — the
   * worker reads the IC state at run time, so one queued worker is
   * enough no matter how many edges come in.  atomic_xchg returns the
   * prior value, so we queue exactly when the flag was 0 before.
   */

  if (atomic_xchg(&priv->int_pending, 1) == 0)
    {
      work_queue(LPWORK, &priv->work, tlsc6x_worker, priv, 0);
    }

  return OK;
}

static void tlsc6x_worker(FAR void *arg)
{
  FAR struct tlsc6x_dev_s *priv = (FAR struct tlsc6x_dev_s *)arg;
  int ret;

  /* Clear the pending flag *before* the I2C read so that an edge which
   * arrives while we are reading still gets a follow-up worker queued.
   */

  atomic_set(&priv->int_pending, 0);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return;
    }

  ret = tlsc6x_read_packet(priv);
  if (ret < 0)
    {
      ierr("tlsc6x: I2C read failed: %d\n", ret);
      goto out;
    }

  tlsc6x_parse(priv);

out:
  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int tlsc6x_register(FAR const char *devpath,
                    FAR struct i2c_master_s *i2c,
                    FAR const struct tlsc6x_config_s *config)
{
  FAR struct tlsc6x_dev_s *priv;
  int ret;

  DEBUGASSERT(devpath != NULL && i2c != NULL && config != NULL);
  DEBUGASSERT(config->attach != NULL && config->enable != NULL &&
              config->nreset != NULL);

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->cfg = config;
  nxmutex_init(&priv->lock);

  /* Resolve transform: bool fields can only be RAISED — when the
   * Kconfig default is "y" the transform is forced on regardless of
   * board override; when the Kconfig default is "n" the board's bool
   * is honoured.  Disabling a Kconfig default-on transform must be
   * done via Kconfig (set the global default to n) — the bool field
   * cannot distinguish "user set false" from "left zero".  uint16_t
   * fields use 0 as a sentinel meaning "fall back to Kconfig default".
   */

#ifdef CONFIG_INPUT_TLSC6X_DEFAULT_EXCHANGE_XY
  priv->exchange_xy = true;
#else
  priv->exchange_xy = config->exchange_xy;
#endif

#ifdef CONFIG_INPUT_TLSC6X_DEFAULT_REVERT_X
  priv->revert_x = true;
#else
  priv->revert_x = config->revert_x;
#endif

#ifdef CONFIG_INPUT_TLSC6X_DEFAULT_REVERT_Y
  priv->revert_y = true;
#else
  priv->revert_y = config->revert_y;
#endif

  priv->max_x = config->max_x ? config->max_x
                              : CONFIG_INPUT_TLSC6X_DEFAULT_MAX_X;
  priv->max_y = config->max_y ? config->max_y
                              : CONFIG_INPUT_TLSC6X_DEFAULT_MAX_Y;

  /* Lower-half setup.  Upper half fills priv->lower.priv when
   * touch_register succeeds, which the worker then uses with
   * touch_event().
   */

  priv->lower.maxpoint = TLSC6X_MAX_POINTS;

  ret = config->attach(config, tlsc6x_isr, priv);
  if (ret < 0)
    {
      ierr("tlsc6x: attach failed: %d\n", ret);
      goto err_attach;
    }

  ret = touch_register(&priv->lower, devpath, 1);
  if (ret < 0)
    {
      ierr("tlsc6x: touch_register failed: %d\n", ret);
      goto err_register;
    }

  config->enable(config, true);

  iinfo("tlsc6x: %s registered (addr=0x%02x, %ux%u, xchg=%d rx=%d ry=%d)\n",
        devpath, config->address, priv->max_x, priv->max_y,
        priv->exchange_xy, priv->revert_x, priv->revert_y);

  return OK;

err_register:
  config->attach(config, NULL, NULL);
err_attach:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}
