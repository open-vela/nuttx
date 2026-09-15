/****************************************************************************
 * drivers/serial/uart_rpmsg.c
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

#include <debug.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/serial/serial.h>
#include <nuttx/serial/uart_rpmsg.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define UART_RPMSG_DEV_CONSOLE          "/dev/console"
#define UART_RPMSG_DEV_PREFIX           "/dev/tty"
#define UART_RPMSG_EPT_PREFIX           "rpmsg-tty"
#define UART_RPMSG_NAME_SIZE            (RPMSG_NAME_SIZE - 10)

#define UART_RPMSG_TTY_WRITE            0
#define UART_RPMSG_TTY_WAKEUP           1

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct uart_rpmsg_header_s
{
  uint32_t command : 31;
  uint32_t response : 1;
  int32_t  result;
  uint64_t cookie;
} end_packed_struct;

begin_packed_struct struct uart_rpmsg_write_s
{
  struct uart_rpmsg_header_s header;
  uint32_t                   count;
  uint32_t                   resolved;
  char                       data[0];
} end_packed_struct;

begin_packed_struct struct uart_rpmsg_wakeup_s
{
  struct uart_rpmsg_header_s header;
} end_packed_struct;

struct uart_rpmsg_priv_s
{
  struct rpmsg_endpoint ept;
  mutex_t               lock;
  char                  devname[UART_RPMSG_NAME_SIZE];
  char                  cpuname[UART_RPMSG_NAME_SIZE];
  FAR void              *recv_data;
  atomic_t              last_upper;
  struct work_s         announce_work;   /* deferred NS re-announce */
  int                   announce_cnt;    /* re-announce attempts left */
};

/* Some masters (e.g. Linux) bring their rpmsg host online well after this
 * remote pins DRIVER_OK, so the single name-service announce emitted by
 * rpmsg_create_ept() during device-created can be lost (no vring buffer yet).
 * Re-announce periodically until the master binds our endpoint (dest_addr
 * learned) or we run out of attempts. Harmless when the master is early: the
 * first check already sees the endpoint bound and the work stops.
 */

#define UART_RPMSG_REANNOUNCE_MAX      30
#define UART_RPMSG_REANNOUNCE_PERIOD   MSEC2TICK(1000)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  uart_rpmsg_setup(FAR struct uart_dev_s *dev);
static void uart_rpmsg_shutdown(FAR struct uart_dev_s *dev);
static int  uart_rpmsg_attach(FAR struct uart_dev_s *dev);
static void uart_rpmsg_detach(FAR struct uart_dev_s *dev);
static void uart_rpmsg_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool uart_rpmsg_rxflowcontrol(FAR struct uart_dev_s *dev,
                                     unsigned int nbuffered, bool upper);
static void uart_rpmsg_dmasend(FAR struct uart_dev_s *dev);
static void uart_rpmsg_dmareceive(FAR struct uart_dev_s *dev);
static void uart_rpmsg_dmarxfree(FAR struct uart_dev_s *dev);
static void uart_rpmsg_dmatxavail(FAR struct uart_dev_s *dev);
static void uart_rpmsg_send(FAR struct uart_dev_s *dev, int ch);
static void uart_rpmsg_txint(FAR struct uart_dev_s *dev, bool enable);
static bool uart_rpmsg_txready(FAR struct uart_dev_s *dev);
static bool uart_rpmsg_txempty(FAR struct uart_dev_s *dev);
static void uart_rpmsg_device_created(FAR struct rpmsg_device *rdev,
                                      FAR void *priv_);
static void uart_rpmsg_device_destroy(FAR struct rpmsg_device *rdev,
                                      FAR void *priv_);
static int  uart_rpmsg_ept_cb(struct rpmsg_endpoint *ept, FAR void *data,
                              size_t len, uint32_t src, FAR void *priv_);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_rpmsg_ops =
{
  .setup         = uart_rpmsg_setup,
  .shutdown      = uart_rpmsg_shutdown,
  .attach        = uart_rpmsg_attach,
  .detach        = uart_rpmsg_detach,
  .rxint         = uart_rpmsg_rxint,
  .rxflowcontrol = uart_rpmsg_rxflowcontrol,
  .dmasend       = uart_rpmsg_dmasend,
  .dmareceive    = uart_rpmsg_dmareceive,
  .dmarxfree     = uart_rpmsg_dmarxfree,
  .dmatxavail    = uart_rpmsg_dmatxavail,
  .send          = uart_rpmsg_send,
  .txint         = uart_rpmsg_txint,
  .txready       = uart_rpmsg_txready,
  .txempty       = uart_rpmsg_txempty,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int uart_rpmsg_setup(FAR struct uart_dev_s *dev)
{
  return OK;
}

static void uart_rpmsg_shutdown(FAR struct uart_dev_s *dev)
{
}

static int uart_rpmsg_attach(FAR struct uart_dev_s *dev)
{
  return OK;
}

static void uart_rpmsg_detach(FAR struct uart_dev_s *dev)
{
}

static void uart_rpmsg_rxint(FAR struct uart_dev_s *dev, bool enable)
{
}

static bool uart_rpmsg_rxflowcontrol(FAR struct uart_dev_s *dev,
                                     unsigned int nbuffered, bool upper)
{
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;
  FAR struct uart_rpmsg_wakeup_s msg;

  if (atomic_xchg(&priv->last_upper, upper) != upper && !upper)
    {
      memset(&msg, 0, sizeof(msg));

      msg.header.command = UART_RPMSG_TTY_WAKEUP;
      if (is_rpmsg_ept_ready(&priv->ept))
        {
          /* Non-blocking: this runs in the RX path; a full TX ring must not
           * wedge it (see dmasend). Dropping a wakeup only briefly delays
           * flow-control resumption.
           */

          rpmsg_trysend(&priv->ept, &msg, sizeof(msg));
        }
    }

  return false;
}

static void uart_rpmsg_dmasend(FAR struct uart_dev_s *dev)
{
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;
  FAR struct uart_dmaxfer_s *xfer = &dev->dmatx;
  FAR struct uart_rpmsg_write_s *msg;
  size_t len = xfer->length + xfer->nlength;
  uint32_t space;

  /* Non-blocking (wait=false). The blocking form deadlocks on the console
   * use case: if the peer (Linux) stops consuming -- e.g. the tty is closed
   * on a picocom disconnect -- the TX ring fills and this call, running in the
   * NuttX sender path, would wait forever. Instead just drop this batch (the
   * data stays in dev->xmit and is retried on the next dmatxavail), so the
   * sender never wedges across a disconnect/reconnect.
   */

  msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, false);
  if (!msg)
    {
      dev->dmatx.length = 0;
      return;
    }

  memset(msg, 0, sizeof(*msg));

  space = space - sizeof(*msg);

  if (len > space)
    {
      len = space;
    }

  if (len > xfer->length)
    {
      memcpy(msg->data, xfer->buffer, xfer->length);
      memcpy(msg->data + xfer->length, xfer->nbuffer, len - xfer->length);
    }
  else
    {
      memcpy(msg->data, xfer->buffer, len);
    }

  msg->count          = len;
  msg->header.command = UART_RPMSG_TTY_WRITE;
  msg->header.result  = -ENXIO;
  msg->header.cookie  = (uintptr_t)dev;

  if (rpmsg_send_nocopy(&priv->ept, msg, sizeof(*msg) + len) < 0)
    {
      rpmsg_release_tx_buffer(&priv->ept, msg);
    }
}

static void uart_rpmsg_dmareceive(FAR struct uart_dev_s *dev)
{
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;
  FAR struct uart_dmaxfer_s *xfer = &dev->dmarx;
  FAR struct uart_rpmsg_write_s *msg = priv->recv_data;
  uint32_t len = msg->count;
  size_t space = xfer->length + xfer->nlength;

  if (len > space)
    {
      len = space;
    }

  if (len > xfer->length)
    {
      memcpy(xfer->buffer, msg->data, xfer->length);
      memcpy(xfer->nbuffer, msg->data + xfer->length, len - xfer->length);
    }
  else
    {
      memcpy(xfer->buffer, msg->data, len);
    }

  xfer->nbytes = len;
  uart_recvchars_done(dev);

  msg->header.result = len;

  if (len != msg->count)
    {
      uart_rpmsg_rxflowcontrol(dev, 0, true);
    }
}

static void uart_rpmsg_dmarxfree(FAR struct uart_dev_s *dev)
{
}

static void uart_rpmsg_dmatxavail(FAR struct uart_dev_s *dev)
{
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;

  nxmutex_lock(&priv->lock);

  if (is_rpmsg_ept_ready(&priv->ept) && dev->dmatx.length == 0)
    {
      uart_xmitchars_dma(dev);
    }

  nxmutex_unlock(&priv->lock);
}

static void uart_rpmsg_send(FAR struct uart_dev_s *dev, int ch)
{
  int nexthead;

  nexthead = dev->xmit.head + 1;
  if (nexthead >= dev->xmit.size)
    {
      nexthead = 0;
    }

  if (nexthead != dev->xmit.tail)
    {
      /* No.. not full.  Add the character to the TX buffer and return. */

      dev->xmit.buffer[dev->xmit.head] = ch;
      dev->xmit.head = nexthead;
    }

  uart_rpmsg_dmatxavail(dev);
}

static void uart_rpmsg_txint(FAR struct uart_dev_s *dev, bool enable)
{
}

static bool uart_rpmsg_txready(FAR struct uart_dev_s *dev)
{
  int nexthead;

  nexthead = dev->xmit.head + 1;
  if (nexthead >= dev->xmit.size)
    {
      nexthead = 0;
    }

  return nexthead != dev->xmit.tail;
}

static bool uart_rpmsg_txempty(FAR struct uart_dev_s *dev)
{
  return true;
}

static void uart_rpmsg_ns_bound(struct rpmsg_endpoint *ept)
{
  FAR struct uart_dev_s *dev = ept->priv;

  uart_rpmsg_dmatxavail(dev);
}

static void uart_rpmsg_announce_work(FAR void *arg)
{
  FAR struct uart_rpmsg_priv_s *priv = arg;

  /* Stop once the master has bound our endpoint (dest_addr learned) or the
   * attempt budget is exhausted.
   */

  if (priv->ept.dest_addr != RPMSG_ADDR_ANY || priv->announce_cnt <= 0)
    {
      return;
    }

  priv->announce_cnt--;

  /* Re-send the NS announce UNCONDITIONALLY (as a healthy remote does). This
   * also kicks the mailbox doorbell, which is what makes the master (Linux)
   * process the announce sitting in the vring and create the channel. Gating
   * this on is_rpmsg_ept_ready() was a chicken-and-egg bug: dest_addr stays
   * RPMSG_ADDR_ANY until the master binds, and the master never binds without
   * receiving this doorbell -> the channel was never created.
   */

  rpmsg_send_ns_message(&priv->ept, RPMSG_NS_CREATE);

  work_queue(HPWORK, &priv->announce_work, uart_rpmsg_announce_work,
             priv, UART_RPMSG_REANNOUNCE_PERIOD);
}

static void uart_rpmsg_device_created(FAR struct rpmsg_device *rdev,
                                      FAR void *priv_)
{
  FAR struct uart_dev_s *dev = priv_;
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;
  char eptname[RPMSG_NAME_SIZE];

  if (strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      priv->ept.priv = dev;
      priv->ept.ns_bound_cb = uart_rpmsg_ns_bound;
      snprintf(eptname, sizeof(eptname), "%s%s",
               UART_RPMSG_EPT_PREFIX, priv->devname);
      rpmsg_create_ept(&priv->ept, rdev, eptname,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       uart_rpmsg_ept_cb, NULL);

      /* The first name-service announce above may be lost if the master's
       * rpmsg host is not online yet. Re-announce periodically until it binds.
       */

      priv->announce_cnt = UART_RPMSG_REANNOUNCE_MAX;
      work_queue(HPWORK, &priv->announce_work, uart_rpmsg_announce_work,
                 priv, UART_RPMSG_REANNOUNCE_PERIOD);
    }
}

static void uart_rpmsg_device_destroy(FAR struct rpmsg_device *rdev,
                                      FAR void *priv_)
{
  FAR struct uart_dev_s *dev = priv_;
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;

  if (priv->ept.priv != NULL &&
      strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      work_cancel(HPWORK, &priv->announce_work);
      rpmsg_destroy_ept(&priv->ept);
    }

  dev->dmatx.nbytes = dev->dmatx.length + dev->dmatx.nlength;
  uart_xmitchars_done(dev);
}

static int uart_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept, FAR void *data,
                             size_t len, uint32_t src, FAR void *priv_)
{
  FAR struct uart_dev_s *dev = priv_;
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;
  FAR struct uart_rpmsg_header_s *header = data;
  FAR struct uart_rpmsg_write_s *msg = data;

  if (header->response)
    {
      /* Get write-cmd response, this tell how many data have sent */

      nxmutex_lock(&priv->lock);

      dev->dmatx.nbytes = header->result;
      if (header->result < 0)
        {
          dev->dmatx.nbytes = 0;
        }

      uart_xmitchars_done(dev);

      nxmutex_unlock(&priv->lock);

      /* If have sent some data succeed, then continue send */

      if (msg->count == header->result)
        {
          uart_rpmsg_dmatxavail(dev);
        }
    }
  else if (header->command == UART_RPMSG_TTY_WRITE)
    {
      irqstate_t flags;

      /* Get write-cmd, there are some data, we need receive them */

      flags = uart_spinlock(dev, true);
      priv->recv_data = data;
      uart_recvchars_dma(dev);
      priv->recv_data = NULL;
      uart_spinunlock(dev, true, flags);

      header->response = 1;
      rpmsg_send(ept, msg, sizeof(*msg));
    }
  else if (header->command == UART_RPMSG_TTY_WAKEUP)
    {
      /* Remote core have space, then wakeup current core, continue send */

      uart_rpmsg_dmatxavail(dev);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int uart_rpmsg_init(FAR const char *cpuname, FAR const char *devname,
                    int buf_size, bool isconsole)
{
  FAR struct uart_rpmsg_priv_s *priv;
  FAR struct uart_dev_s *dev;
  char dev_name[32];
  int ret = -ENOMEM;

  dev = kmm_zalloc(sizeof(struct uart_dev_s));
  if (!dev)
    {
      return ret;
    }

  dev->ops       = &g_uart_rpmsg_ops;
  dev->isconsole = isconsole;
  dev->recv.size = buf_size;
  dev->xmit.size = buf_size;

  dev->recv.buffer = kmm_malloc(dev->recv.size);
  if (!dev->recv.buffer)
    {
      goto fail;
    }

  dev->xmit.buffer = kmm_malloc(dev->xmit.size);
  if (!dev->xmit.buffer)
    {
      goto fail;
    }

  priv = kmm_zalloc(sizeof(struct uart_rpmsg_priv_s));
  if (!priv)
    {
      goto fail;
    }

  nxmutex_init(&priv->lock);
  strlcpy(priv->cpuname, cpuname, sizeof(priv->cpuname));
  strlcpy(priv->devname, devname, sizeof(priv->devname));

  dev->priv = priv;

  ret = rpmsg_register_callback(dev,
                                uart_rpmsg_device_created,
                                uart_rpmsg_device_destroy,
                                NULL,
                                NULL);
  if (ret < 0)
    {
      nxmutex_destroy(&priv->lock);
      goto fail;
    }

  snprintf(dev_name, sizeof(dev_name), "%s%s",
           UART_RPMSG_DEV_PREFIX, devname);
  ret = uart_register(dev_name, dev);
  if (ret < 0)
    {
      rpmsgerr("uart register failed, ret=%d\n", ret);
      goto fail_with_rpmsg;
    }

  if (dev->isconsole)
    {
      ret = uart_register(UART_RPMSG_DEV_CONSOLE, dev);
      if (ret < 0)
        {
          rpmsgerr("uart register console failed, ret=%d\n", ret);
          goto fail_with_rpmsg;
        }
    }

  return OK;

fail_with_rpmsg:
  nxmutex_destroy(&priv->lock);
  rpmsg_unregister_callback(dev,
                            uart_rpmsg_device_created,
                            uart_rpmsg_device_destroy,
                            NULL,
                            NULL);
fail:
  kmm_free(dev->recv.buffer);
  kmm_free(dev->xmit.buffer);
  kmm_free(dev->priv);
  kmm_free(dev);

  return ret;
}
