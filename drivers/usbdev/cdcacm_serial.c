/****************************************************************************
 * drivers/usbdev/cdcacm_serial.c
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
 * Description
 *
 * Thin uart_dev_s adapter on top of the cdcacm USB CDC ACM core.
 * Registers /dev/ttyACMn and a cdcacm_user_ops_s callback table that
 * translates ACM class events to NuttX uart semantics.
 *
 * This file holds the entire uart_dev_s adapter (the cdcuart_*
 * implementations, the g_uartops table and cdcacm_initialize /
 * cdcacm_uninitialize); cdcacm.c retains only the USB protocol core.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <sys/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/queue.h>
#include <nuttx/semaphore.h>
#include <nuttx/serial/serial.h>
#include <nuttx/spinlock.h>
#include <nuttx/wdog.h>

#include <nuttx/usb/cdc.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include "cdcacm.h"
#include "cdcacm_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Per-instance state owned by the uart adapter.  cdcacm_set_user_ops
 * stashes a pointer to this struct as the user_priv blob; ops bodies and
 * the moved cdcuart_* functions recover it via dev->priv.
 */

struct cdcacm_serial_priv_s
{
  struct uart_dev_s        serdev;     /* uart upper half (registered) */
  struct cdc_linecoding_s  linecoding; /* current ACM line coding */
  uint8_t                  ctrlline;   /* DTR/RTS state, bits 0/1 */
  cdcacm_callback_t        callback;   /* legacy event callback */

  /* Pointer back to the cdcacm USB protocol core.  Set by
   * cdcacm_initialize after cdcacm_register returns; used by the
   * moved cdcuart_* functions to reach EPs / wrcontainer pool.
   */

  FAR struct cdcacm_dev_s *cdc_dev;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* ACM class request ops */

static int  cdcacm_serial_set_line_coding(FAR struct cdcacm_dev_s *dev,
                                          FAR const uint8_t *coding,
                                          size_t len);
static int  cdcacm_serial_get_line_coding(FAR struct cdcacm_dev_s *dev,
                                          FAR uint8_t *out, size_t cap);
static int  cdcacm_serial_set_ctrl_line_state(FAR struct cdcacm_dev_s *dev,
                                              uint16_t state);
static int  cdcacm_serial_send_break(FAR struct cdcacm_dev_s *dev,
                                     uint16_t duration);

/* USB lifecycle / data plane ops */

static void cdcacm_serial_on_connect(FAR struct cdcacm_dev_s *dev,
                                     bool connected);
static void cdcacm_serial_on_suspend(FAR struct cdcacm_dev_s *dev,
                                     bool suspended);
static void cdcacm_serial_on_rx(FAR struct cdcacm_dev_s *dev,
                                FAR const uint8_t *buf, size_t len);
static int  cdcacm_serial_pull_tx(FAR struct cdcacm_dev_s *dev,
                                  FAR uint8_t *dst, size_t cap);

/* uart_ops_s implementation */

static int     cdcuart_setup(FAR struct uart_dev_s *dev);
static void    cdcuart_shutdown(FAR struct uart_dev_s *dev);
static int     cdcuart_attach(FAR struct uart_dev_s *dev);
static void    cdcuart_detach(FAR struct uart_dev_s *dev);
static int     cdcuart_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);
static void    cdcuart_rxint(FAR struct uart_dev_s *dev, bool enable);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool    cdcuart_rxflowcontrol(FAR struct uart_dev_s *dev,
                 unsigned int nbuffered, bool upper);
#endif
static void    cdcuart_txint(FAR struct uart_dev_s *dev, bool enable);
static bool    cdcuart_txempty(FAR struct uart_dev_s *dev);
static int     cdcuart_release(FAR struct uart_dev_s *dev);
static bool    cdcuart_rxavailable(FAR struct uart_dev_s *dev);
static ssize_t cdcuart_recvbuf(FAR struct uart_dev_s *dev,
                               FAR void *buf, size_t len);
static bool    cdcuart_txready(FAR struct uart_dev_s *dev);
static ssize_t cdcuart_sendbuf(FAR struct uart_dev_s *dev,
                               FAR const void *buf, size_t len);
#ifndef CONFIG_CDCACM_DISABLE_TXBUF
static void    cdcuart_dmasend(FAR struct uart_dev_s *dev);
#endif
#ifndef CONFIG_CDCACM_DISABLE_RXBUF
static void    cdcuart_dmareceive(FAR struct uart_dev_s *dev);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cdcacm_serial_set_line_coding
 *
 * Description:
 *   ACM SET_LINE_CODING handler.  Saves the host-supplied dwDTERate /
 *   bCharFormat / bParityType / bDataBits into the private state and
 *   notifies the legacy event callback (if any).  Mirrors the fallback
 *   path that cdcacm.c used before the ops dispatch was introduced.
 *
 ****************************************************************************/

static int cdcacm_serial_set_line_coding(FAR struct cdcacm_dev_s *dev,
                                         FAR const uint8_t *coding,
                                         size_t len)
{
  FAR struct cdcacm_serial_priv_s *sp = cdcacm_get_user_priv(dev);

  if (sp == NULL || coding == NULL)
    {
      return -EINVAL;
    }

  if (len > sizeof(sp->linecoding))
    {
      len = sizeof(sp->linecoding);
    }

  memcpy(&sp->linecoding, coding, len);

  if (sp->callback != NULL)
    {
      sp->callback(CDCACM_EVENT_LINECODING);
    }

  return 0;
}

/****************************************************************************
 * Name: cdcacm_serial_get_line_coding
 *
 * Description:
 *   ACM GET_LINE_CODING handler.  Copies the current line coding into the
 *   caller-supplied buffer.  Returns the number of bytes written.
 *
 ****************************************************************************/

static int cdcacm_serial_get_line_coding(FAR struct cdcacm_dev_s *dev,
                                         FAR uint8_t *out, size_t cap)
{
  FAR struct cdcacm_serial_priv_s *sp = cdcacm_get_user_priv(dev);
  size_t n;

  if (sp == NULL || out == NULL)
    {
      return -EINVAL;
    }

  n = MIN(cap, sizeof(sp->linecoding));
  memcpy(out, &sp->linecoding, n);
  return (int)n;
}

/****************************************************************************
 * Name: cdcacm_serial_set_ctrl_line_state
 *
 * Description:
 *   ACM SET_CONTROL_LINE_STATE handler.  Records DTR (bit 0) / RTS (bit 1)
 *   and notifies the legacy event callback.
 *
 ****************************************************************************/

static int cdcacm_serial_set_ctrl_line_state(FAR struct cdcacm_dev_s *dev,
                                             uint16_t state)
{
  FAR struct cdcacm_serial_priv_s *sp = cdcacm_get_user_priv(dev);

  if (sp == NULL)
    {
      return -EINVAL;
    }

  sp->ctrlline = (uint8_t)(state & 3);

  if (sp->callback != NULL)
    {
      sp->callback(CDCACM_EVENT_CTRLLINE);
    }

  return 0;
}

/****************************************************************************
 * Name: cdcacm_serial_send_break
 *
 * Description:
 *   ACM SEND_BREAK handler.  The legacy fallback path simply notifies the
 *   event callback and lets the registrant decide what to do with the
 *   duration; we preserve that behaviour.
 *
 ****************************************************************************/

static int cdcacm_serial_send_break(FAR struct cdcacm_dev_s *dev,
                                    uint16_t duration)
{
  FAR struct cdcacm_serial_priv_s *sp = cdcacm_get_user_priv(dev);

  UNUSED(duration);

  if (sp == NULL)
    {
      return -EINVAL;
    }

  if (sp->callback != NULL)
    {
      sp->callback(CDCACM_EVENT_SENDBREAK);
    }

  return 0;
}

/****************************************************************************
 * Name: cdcacm_serial_on_connect
 *
 * Description:
 *   USB connect/disconnect notification.  The cdcacm core dispatches both
 *   SET_CONFIGURATION up-transitions and unbind/disconnect down-transitions
 *   through this callback.  We forward the edge to the uart upper half so
 *   pending I/O can be unblocked when the cable is yanked.
 *
 ****************************************************************************/

static void cdcacm_serial_on_connect(FAR struct cdcacm_dev_s *dev,
                                     bool connected)
{
  FAR struct cdcacm_serial_priv_s *sp = cdcacm_get_user_priv(dev);

  if (sp == NULL)
    {
      return;
    }

#ifdef CONFIG_SERIAL_REMOVABLE
  uart_connected(&sp->serdev, connected);
#else
  UNUSED(connected);
#endif
}

/****************************************************************************
 * Name: cdcacm_serial_on_suspend
 *
 * Description:
 *   USB bus suspend/resume notification.  Suspend is treated as a
 *   transient disconnect and resume as a reconnect for the upper half.
 *
 ****************************************************************************/

static void cdcacm_serial_on_suspend(FAR struct cdcacm_dev_s *dev,
                                     bool suspended)
{
  FAR struct cdcacm_serial_priv_s *sp = cdcacm_get_user_priv(dev);

  if (sp == NULL)
    {
      return;
    }

#ifdef CONFIG_SERIAL_REMOVABLE
  uart_connected(&sp->serdev, !suspended);
#else
  UNUSED(suspended);
#endif
}

/****************************************************************************
 * Name: cdcacm_serial_on_rx
 *
 * Description:
 *   OUT-endpoint completion notification.  cdcacm core has pushed the bytes
 *   into priv->rxpending; drive the serial framework to copy them into the
 *   uart_dev_s receive ring (the buffer uart_read() actually drains), then
 *   wake any blocked reader.  buf/len are informational -- the data is taken
 *   from rxpending via the recvbuf / dmareceive op.
 *
 ****************************************************************************/

static void cdcacm_serial_on_rx(FAR struct cdcacm_dev_s *dev,
                                FAR const uint8_t *buf, size_t len)
{
  FAR struct cdcacm_serial_priv_s *sp = cdcacm_get_user_priv(dev);

  UNUSED(buf);
  UNUSED(len);

  if (sp == NULL)
    {
      return;
    }

#if defined(CONFIG_SERIAL_RXDMA) && !defined(CONFIG_CDCACM_DISABLE_RXBUF)
  /* DMA-capable recv ring: uart_recvchars_dma sets up the transfer window
   * and invokes cdcuart_dmareceive, which copies rxpending into dev->recv
   * and calls uart_recvchars_done (which wakes the reader).  Guard on
   * rxpending so cdcuart_dmareceive's non-empty assertion always holds.
   */

  if (!sq_empty(&sp->cdc_dev->rxpending))
    {
      uart_recvchars_dma(&sp->serdev);
    }
#else
  /* Non-DMA recv ring: uart_recvchars loops while cdcuart_rxavailable() is
   * true, pulling each rxpending packet through cdcuart_recvbuf into
   * dev->recv, and wakes the reader at the end.
   */

  uart_recvchars(&sp->serdev);
#endif

  /* Wake any reader/poll waiter even when no bytes were moved (e.g. the
   * window was momentarily full); harmless and keeps the wakeup guaranteed.
   */

  uart_datareceived(&sp->serdev);
}

/****************************************************************************
 * Name: cdcacm_serial_pull_tx
 *
 * Description:
 *   IN-endpoint refill request.  Drain up to cap bytes from the uart xmit
 *   ring into dst, advance the tail, and wake any writer blocked on free
 *   space.  Returns the number of bytes copied (0 means "ring empty,
 *   nothing to send").
 *
 *   This is the manual translation of the legacy uart_xmitchars_dma path:
 *   the real uart_xmitchars_dma() takes only a uart_dev_t and pulls from
 *   dev->dmatx, so we cannot delegate directly with the (dst, cap)
 *   arguments cdcacm_setup hands us.  Instead we replicate the ring math
 *   inline.
 *
 ****************************************************************************/

static int cdcacm_serial_pull_tx(FAR struct cdcacm_dev_s *dev,
                                 FAR uint8_t *dst, size_t cap)
{
  FAR struct cdcacm_serial_priv_s *sp = cdcacm_get_user_priv(dev);
  FAR struct uart_buffer_s *xmit;
  irqstate_t flags;
  size_t copied = 0;
  size_t chunk;

  if (sp == NULL || dst == NULL || cap == 0)
    {
      return 0;
    }

  xmit = &sp->serdev.xmit;

  /* Read head/tail and advance the tail under dev->lock.  This is the
   * same lock the cdcacm core holds around txfree and that cdcuart_dmasend
   * also takes.  Without it the wrcomplete-IRQ pull_tx path and the
   * cdcuart_txint task path race on the xmit ring -- the same window gets
   * copied into two wrreqs and the tail is advanced twice, causing data
   * duplication and corruption.  Drop the lock before uart_datasent(),
   * which may reschedule.
   */

  flags = spin_lock_irqsave(&dev->lock);

  if (xmit->head == xmit->tail)
    {
      spin_unlock_irqrestore(&dev->lock, flags);
      return 0;
    }

  if (xmit->tail < xmit->head)
    {
      /* Contiguous: [tail, head) */

      chunk = MIN(cap, (size_t)(xmit->head - xmit->tail));
      memcpy(dst, &xmit->buffer[xmit->tail], chunk);
      xmit->tail = (xmit->tail + chunk) % xmit->size;
      copied = chunk;
    }
  else
    {
      /* Wrapped: [tail, size) then [0, head) */

      chunk = MIN(cap, (size_t)(xmit->size - xmit->tail));
      memcpy(dst, &xmit->buffer[xmit->tail], chunk);
      xmit->tail = (xmit->tail + chunk) % xmit->size;
      copied = chunk;

      if (copied < cap && xmit->head > 0)
        {
          chunk = MIN(cap - copied, (size_t)xmit->head);
          memcpy(dst + copied, xmit->buffer, chunk);
          xmit->tail = (xmit->tail + chunk) % xmit->size;
          copied += chunk;
        }
    }

  spin_unlock_irqrestore(&dev->lock, flags);

  if (copied > 0)
    {
      uart_datasent(&sp->serdev);
    }

  return (int)copied;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The DISABLE_TXBUF zero-copy alias ops (claim_xmit_buf / release_xmit_buf)
 * are intentionally left NULL.  The adapter has no xmit buffer of its own
 * to lend; cdcacm.c's dispatch path detects the NULL fields and falls
 * back to its internal wrcontainer->req->buf alias, which is byte-for-byte
 * the same behaviour as before this restructure.
 */

static const struct cdcacm_user_ops_s g_cdcacm_user_ops =
{
  .set_line_coding     = cdcacm_serial_set_line_coding,
  .get_line_coding     = cdcacm_serial_get_line_coding,
  .set_ctrl_line_state = cdcacm_serial_set_ctrl_line_state,
  .send_break          = cdcacm_serial_send_break,
  .on_connect          = cdcacm_serial_on_connect,
  .on_suspend          = cdcacm_serial_on_suspend,
  .on_rx               = cdcacm_serial_on_rx,
  .pull_tx             = cdcacm_serial_pull_tx,
  .claim_xmit_buf      = NULL,    /* wrcontainer alias */
  .release_xmit_buf    = NULL,    /* wrcontainer alias */
};

/* uart_ops_s implementation table.  The member layout is positional
 * because uart_ops_s membership is conditional on a tangle of
 * SERIAL_TX/RXDMA macros and IFLOWCONTROL.
 */

static const struct uart_ops_s g_uartops =
{
  cdcuart_setup,         /* setup */
  cdcuart_shutdown,      /* shutdown */
  cdcuart_attach,        /* attach */
  cdcuart_detach,        /* detach */
  cdcuart_ioctl,         /* ioctl */
  NULL,                  /* receive */
  cdcuart_rxint,         /* rxinit */
  cdcuart_rxavailable,   /* rxavailable */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  cdcuart_rxflowcontrol, /* rxflowcontrol */
#endif
#ifdef CONFIG_SERIAL_TXDMA
#ifndef CONFIG_CDCACM_DISABLE_TXBUF
  cdcuart_dmasend,       /* dmasend */
#else
  NULL,                  /* dmasend */
#endif
#endif
#ifdef CONFIG_SERIAL_RXDMA
#ifndef CONFIG_CDCACM_DISABLE_RXBUF
  cdcuart_dmareceive,    /* dmareceive */
#else
  NULL,                  /* dmareceive */
#endif
  NULL,                  /* dmarxfree */
#endif
#ifdef CONFIG_SERIAL_TXDMA
  NULL,                  /* dmatxavail */
#endif
  NULL,                  /* send */
  cdcuart_txint,         /* txint */
  cdcuart_txready,       /* txready */
  cdcuart_txempty,       /* txempty */
  cdcuart_release,       /* release */
  cdcuart_recvbuf,       /* recvbuf */
  cdcuart_sendbuf        /* sendbuf */
};

/****************************************************************************
 * Private Functions: cdcuart_*
 ****************************************************************************/

/****************************************************************************
 * Name: cdcuart_setup
 *
 * Description:
 *   This method is called the first time that the serial port is opened.
 *
 ****************************************************************************/

static int cdcuart_setup(FAR struct uart_dev_s *dev)
{
  FAR struct cdcacm_serial_priv_s *sp;
  FAR struct cdcacm_dev_s *priv;

  usbtrace(CDCACM_CLASSAPI_SETUP, 0);

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev || !dev->priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EINVAL;
    }
#endif

  /* Extract reference to private data */

  sp   = (FAR struct cdcacm_serial_priv_s *)dev->priv;
  priv = sp->cdc_dev;

  /* Check if we have been configured */

  if (priv->config == CDCACM_CONFIGIDNONE)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SETUPNOTCONNECTED), 0);
      return -ENOTCONN;
    }

  return OK;
}

/****************************************************************************
 * Name: cdcuart_shutdown
 ****************************************************************************/

static void cdcuart_shutdown(FAR struct uart_dev_s *dev)
{
  usbtrace(CDCACM_CLASSAPI_SHUTDOWN, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev || !dev->priv)
    {
       usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
    }
#endif
}

/****************************************************************************
 * Name: cdcuart_attach
 ****************************************************************************/

static int cdcuart_attach(FAR struct uart_dev_s *dev)
{
  UNUSED(dev);
  usbtrace(CDCACM_CLASSAPI_ATTACH, 0);
  return OK;
}

/****************************************************************************
 * Name: cdcuart_detach
 ****************************************************************************/

static void cdcuart_detach(FAR struct uart_dev_s *dev)
{
  UNUSED(dev);
  usbtrace(CDCACM_CLASSAPI_DETACH, 0);
}

/****************************************************************************
 * Name: cdcuart_ioctl
 ****************************************************************************/

static int cdcuart_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode               *inode  = filep->f_inode;
  FAR struct cdcacm_serial_priv_s *sp    = inode->i_private;
  FAR struct cdcacm_dev_s        *priv   = sp->cdc_dev;
#if defined(CONFIG_CDCACM_DISABLE_TXBUF) || defined(CONFIG_CDCACM_DISABLE_RXBUF)
  FAR struct uart_dev_s          *serdev = &sp->serdev;
#endif
  int                             ret    = OK;

  UNUSED(priv);

  switch (cmd)
    {
#ifdef CONFIG_CDCACM_DISABLE_RXBUF
    case FIONREAD:
      {
        FAR struct cdcacm_rdreq_s *rdcontainer;
        FAR sq_entry_t *entry;
        int count;

        irqstate_t flags = spin_lock_irqsave(&priv->lock);

        count = serdev->recv.head - serdev->recv.tail;

        sq_for_every(&priv->rxpending, entry)
          {
            rdcontainer = (FAR struct cdcacm_rdreq_s *)entry;
            count += rdcontainer->req->xfrd;
          }

        spin_unlock_irqrestore(&priv->lock, flags);

        *(FAR int *)((uintptr_t)arg) = count;
      }
      break;
#endif

#ifdef CONFIG_CDCACM_DISABLE_TXBUF
    case FIONWRITE:
      {
        FAR struct cdcacm_wrreq_s *wrcontainer;
        FAR sq_entry_t *entry;
        int count;
        int i;

        irqstate_t flags = spin_lock_irqsave(&priv->lock);

        count = serdev->xmit.head - serdev->xmit.tail;

        if (priv->nwrq < (CONFIG_CDCACM_NWRREQS - 1))
          {
            for (i = 0; i < CONFIG_CDCACM_NWRREQS; i++)
              {
                sq_for_every(&priv->txfree, entry)
                  {
                    wrcontainer = (FAR struct cdcacm_wrreq_s *)entry;
                    if (&priv->wrreqs[i] == wrcontainer)
                      {
                        continue;
                      }
                    else if (&priv->wrreqs[i] != priv->wrcontainer)
                      {
                        count += priv->wrreqs[i].req->len;
                      }
                  }
              }
          }

        spin_unlock_irqrestore(&priv->lock, flags);

        *(FAR int *)((uintptr_t)arg) = count;
      }
      break;

    case FIONSPACE:
      {
        FAR sq_entry_t *entry;
        int count = 0;

        irqstate_t flags = spin_lock_irqsave(&priv->lock);

        if (serdev->xmit.head == 0)
          {
            count = serdev->xmit.size - 1;
          }

        sq_for_every(&priv->txfree, entry)
          {
            count += serdev->xmit.size - 1;
          }

        spin_unlock_irqrestore(&priv->lock, flags);

        *(FAR int *)((uintptr_t)arg) = count;
      }
      break;
#endif

    case TCFLSH:
      {
        ret = -ENOTTY;

#ifdef CONFIG_CDCACM_DISABLE_RXBUF
        if (arg == TCIFLUSH || arg == TCIOFLUSH)
          {
            FAR struct cdcacm_rdreq_s *rdcontainer;
            ret = OK;

            irqstate_t flags = spin_lock_irqsave(&priv->lock);

            if (priv->rdcontainer)
              {
                sq_addlast((FAR sq_entry_t *)priv->rdcontainer,
                           &priv->rxpending);
                priv->rdcontainer = NULL;
              }

            while (!sq_empty(&priv->rxpending))
              {
                 rdcontainer = (FAR struct cdcacm_rdreq_s *)
                               sq_remfirst(&priv->rxpending);
                 ret = cdcacm_requeue_rdrequest(priv, rdcontainer);
              }

            serdev->recv.head = 0;
            serdev->recv.tail = 0;

            spin_unlock_irqrestore(&priv->lock, flags);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
            uart_rxflowcontrol(serdev, 0, false);
#endif
          }
#endif

#ifdef CONFIG_CDCACM_DISABLE_TXBUF
        if (arg == TCOFLUSH || arg == TCIOFLUSH)
          {
            irqstate_t flags = spin_lock_irqsave_nopreempt(&priv->lock);
            ret = OK;

            if (priv->wrcontainer)
              {
                serdev->xmit.head = 0;
                serdev->xmit.tail = 0;

                uart_datasent(serdev);
              }
            else if (priv->nwrq > 0)
              {
                priv->wrcontainer = (FAR struct cdcacm_wrreq_s *)
                                    sq_remfirst(&priv->txfree);
                serdev->xmit.buffer =
                         (FAR char *)priv->wrcontainer->req->buf;
                priv->nwrq--;
                serdev->xmit.head = 0;
                serdev->xmit.tail = 0;

                uart_datasent(serdev);
              }
            else
              {
                ret = -EBUSY;
              }

            spin_unlock_irqrestore_nopreempt(&priv->lock, flags);
          }
#endif
      }
      break;

    case CAIOC_REGISTERCB:
      {
        sp->callback = (cdcacm_callback_t)((uintptr_t)arg);
      }
      break;

    case CAIOC_GETLINECODING:
      {
        FAR struct cdc_linecoding_s *ptr =
          (FAR struct cdc_linecoding_s *)((uintptr_t)arg);
        if (ptr != NULL)
          {
            memcpy(ptr, &sp->linecoding, sizeof(struct cdc_linecoding_s));
          }
        else
          {
            ret = -EINVAL;
          }
      }
      break;

    case CAIOC_GETCTRLLINE:
      {
        FAR int *ptr = (FAR int *)((uintptr_t)arg);
        if (ptr != NULL)
          {
            *ptr = sp->ctrlline;
          }
        else
          {
            ret = -EINVAL;
          }
      }
      break;

#ifdef CONFIG_CDCACM_IFLOWCONTROL
    case CAIOC_NOTIFY:
      {
        DEBUGASSERT(arg < UINT8_MAX);

        priv->serialstate = (uint8_t)arg;
        ret = cdcacm_serialstate(priv);
      }
      break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios *termiosp = (FAR struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        termiosp->c_cflag =
            ((sp->linecoding.parity != CDC_PARITY_NONE) ? PARENB : 0) |
            ((sp->linecoding.parity == CDC_PARITY_ODD) ? PARODD : 0) |
            ((sp->linecoding.stop == CDC_CHFMT_STOP2) ? CSTOPB : 0) |
            CS8;

#ifdef CONFIG_CDCACM_OFLOWCONTROL
#  warning Missing logic
#endif
#ifdef CONFIG_CDCACM_IFLOWCONTROL
        termiosp->c_cflag |= (priv->iflow) ? CRTS_IFLOW : 0;
#endif
      cfsetispeed(termiosp, (speed_t)sp->linecoding.baud[3] << 24 |
                            (speed_t)sp->linecoding.baud[2] << 16 |
                            (speed_t)sp->linecoding.baud[1] << 8  |
                            (speed_t)sp->linecoding.baud[0]);
      }
      break;

    case TCSETS:
      {
        struct termios *termiosp = (FAR struct termios *)arg;
#ifdef CONFIG_CDCACM_IFLOWCONTROL
        bool iflow;
#endif

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

#ifdef CONFIG_CDCACM_OFLOWCONTROL
#  warning Missing logic
#endif

#ifdef CONFIG_CDCACM_IFLOWCONTROL
        iflow = ((termiosp->c_cflag & CRTS_IFLOW) != 0);
        if (iflow != priv->iflow)
          {
            if (!iflow)
              {
                if ((priv->serialstate & CDCACM_UART_DSR) == 0)
                  {
                    priv->serialstate |= (CDCACM_UART_DSR | CDCACM_UART_DCD);
                    ret = cdcacm_serialstate(priv);
                  }

                priv->iflow   = false;
                priv->iactive = false;

                cdcacm_release_rxpending(priv);
              }
            else
              {
                priv->iflow        = true;
                priv->iactive      = false;

                if (priv->upper)
                  {
                    priv->serialstate &= ~CDCACM_UART_DSR;
                    priv->serialstate |= CDCACM_UART_DCD;
                    ret = cdcacm_serialstate(priv);

                    priv->iactive      = true;
                  }
              }

            priv->rxenabled = true;
          }
#endif
      }
      break;
#endif

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: cdcuart_rxint
 ****************************************************************************/

static void cdcuart_rxint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct cdcacm_serial_priv_s *sp;
  FAR struct cdcacm_dev_s *priv;
  irqstate_t flags;

  usbtrace(CDCACM_CLASSAPI_RXINT, (uint16_t)enable);

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev || !dev->priv)
    {
       usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
       return;
    }
#endif

  sp   = (FAR struct cdcacm_serial_priv_s *)dev->priv;
  priv = sp->cdc_dev;

  if (enable)
    {
      flags = spin_lock_irqsave(&priv->lock);
      if (!priv->rxenabled)
        {
          priv->rxenabled = true;
        }

      spin_unlock_irqrestore(&priv->lock, flags);

      cdcacm_release_rxpending(priv);
    }
  else
    {
      flags = spin_lock_irqsave(&priv->lock);
      priv->rxenabled = false;
      spin_unlock_irqrestore(&priv->lock, flags);
    }
}

/****************************************************************************
 * Name: cdcuart_rxflowcontrol
 ****************************************************************************/

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool cdcuart_rxflowcontrol(FAR struct uart_dev_s *dev,
                                  unsigned int nbuffered, bool upper)
{
#ifdef CONFIG_CDCACM_IFLOWCONTROL
  FAR struct cdcacm_serial_priv_s *sp;
  FAR struct cdcacm_dev_s *priv;

  UNUSED(nbuffered);

#ifdef CONFIG_DEBUG_FEATURES
  if (dev == NULL || dev->priv == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return false;
    }
#endif

  sp   = (FAR struct cdcacm_serial_priv_s *)dev->priv;
  priv = sp->cdc_dev;

  priv->upper = upper;
  if (priv->iflow)
    {
      if (upper)
        {
          if ((priv->serialstate & CDCACM_UART_DSR) != 0)
            {
              priv->serialstate &= ~CDCACM_UART_DSR;
              priv->serialstate |= CDCACM_UART_DCD;

              cdcacm_serialstate(priv);
            }

          priv->iactive = true;
        }
      else
        {
          priv->iactive = false;

          if ((priv->serialstate & CDCACM_UART_DSR) == 0)
            {
              priv->serialstate |= (CDCACM_UART_DSR | CDCACM_UART_DCD);

              cdcacm_serialstate(priv);
            }

          cdcacm_release_rxpending(priv);
        }
    }
  else
    {
      if ((priv->serialstate & CDCACM_UART_DSR) == 0)
        {
          priv->serialstate |= (CDCACM_UART_DSR | CDCACM_UART_DCD);

          cdcacm_serialstate(priv);

          priv->iactive = false;
        }
    }

  return priv->iactive;
#else
  UNUSED(dev);
  UNUSED(nbuffered);
  UNUSED(upper);
  return false;
#endif
}
#endif

/****************************************************************************
 * Name: cdcuart_txint
 ****************************************************************************/

static void cdcuart_txint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct cdcacm_serial_priv_s *sp;
  FAR struct cdcacm_dev_s *priv;

  usbtrace(CDCACM_CLASSAPI_TXINT, (uint16_t)enable);

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev || !dev->priv)
    {
       usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
       return;
    }
#endif

  sp   = (FAR struct cdcacm_serial_priv_s *)dev->priv;
  priv = sp->cdc_dev;

  uinfo("enable=%d head=%d tail=%d\n",
        enable, sp->serdev.xmit.head, sp->serdev.xmit.tail);

  if (enable && sp->serdev.xmit.head != sp->serdev.xmit.tail)
    {
      cdcacm_sndpacket(priv);
    }
}

/****************************************************************************
 * Name: cdcuart_txready
 ****************************************************************************/

static bool cdcuart_txready(FAR struct uart_dev_s *dev)
{
  FAR struct cdcacm_serial_priv_s *sp = dev->priv;
  FAR struct cdcacm_dev_s *priv = sp->cdc_dev;
  FAR struct usbdev_ep_s *ep = priv->epbulkin;

  if (sq_empty(&priv->txfree))
    {
      priv->ispolling = true;
      EP_POLL(ep);
      priv->ispolling = false;
    }

  return !sq_empty(&priv->txfree);
}

/****************************************************************************
 * Name: cdcuart_txempty
 ****************************************************************************/

static bool cdcuart_txempty(FAR struct uart_dev_s *dev)
{
  FAR struct cdcacm_serial_priv_s *sp =
    (FAR struct cdcacm_serial_priv_s *)dev->priv;
  FAR struct cdcacm_dev_s *priv;
  FAR struct usbdev_ep_s *ep;
  irqstate_t flags;
  bool empty;

  usbtrace(CDCACM_CLASSAPI_TXEMPTY, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!sp)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return true;
    }
#endif

  priv = sp->cdc_dev;
  ep   = priv->epbulkin;

  priv->ispolling = true;
  EP_POLL(ep);
  priv->ispolling = false;

  flags = spin_lock_irqsave(&priv->lock);

#ifdef CONFIG_CDCACM_DISABLE_TXBUF
  empty = priv->nwrq >= (CONFIG_CDCACM_NWRREQS - 1);
#else
  empty = priv->nwrq >= CONFIG_CDCACM_NWRREQS;
#endif
  spin_unlock_irqrestore(&priv->lock, flags);

  return empty;
}

/****************************************************************************
 * Name: cdcuart_release
 *
 * Description:
 *   Free per-instance state when the uart upper half drops its last
 *   reference.  The cdcacm_dev_s teardown stays in cdcacm_uninitialize
 *   (called from board logic); here we only free the adapter struct and
 *   its xmit/recv buffers.
 *
 ****************************************************************************/

static int cdcuart_release(FAR struct uart_dev_s *dev)
{
  FAR struct cdcacm_serial_priv_s *sp =
    (FAR struct cdcacm_serial_priv_s *)dev->priv;

  usbtrace(CDCACM_CLASSAPI_RELEASE, 0);

  if (sp != NULL)
    {
#ifndef CONFIG_CDCACM_DISABLE_TXBUF
      kmm_free(sp->serdev.xmit.buffer);
#endif
#ifndef CONFIG_CDCACM_DISABLE_RXBUF
      kmm_free(sp->serdev.recv.buffer);
#endif
      kmm_free(sp);
    }

  return OK;
}

/****************************************************************************
 * Name: cdcuart_rxavailable
 ****************************************************************************/

static bool cdcuart_rxavailable(FAR struct uart_dev_s *dev)
{
  FAR struct cdcacm_serial_priv_s *sp = dev->priv;
  FAR struct cdcacm_dev_s *priv = sp->cdc_dev;
  FAR struct usbdev_ep_s *ep = priv->epbulkout;

  if (sq_empty(&priv->rxpending))
    {
      priv->ispolling = true;
      EP_POLL(ep);
      priv->ispolling = false;
    }

  return !sq_empty(&priv->rxpending);
}

/****************************************************************************
 * Name: cdcuart_recvbuf
 ****************************************************************************/

static ssize_t cdcuart_recvbuf(FAR struct uart_dev_s *dev,
                               FAR void *buf, size_t len)
{
  FAR struct cdcacm_serial_priv_s *sp = dev->priv;
  FAR struct cdcacm_dev_s *priv = sp->cdc_dev;
  FAR struct cdcacm_rdreq_s *rdcontainer;
  FAR struct usbdev_req_s *req;
  FAR uint8_t *reqbuf;
  size_t reqlen;
  size_t nbytes;
  int ret;

  rdcontainer = (FAR struct cdcacm_rdreq_s *)sq_peek(&priv->rxpending);
  DEBUGASSERT(rdcontainer != NULL);

  req = rdcontainer->req;
  DEBUGASSERT(req != NULL);

  reqbuf = &req->buf[rdcontainer->offset];
  reqlen = req->xfrd - rdcontainer->offset;

  nbytes = MIN(reqlen, len);
  memcpy(buf, reqbuf, nbytes);
  rdcontainer->offset += nbytes;

  if (rdcontainer->offset >= req->xfrd)
    {
      sq_remfirst(&priv->rxpending);
      ret = cdcacm_requeue_rdrequest(priv, rdcontainer);
      if (ret < 0)
        {
          return ret;
        }
    }

  return nbytes;
}

/****************************************************************************
 * Name: cdcuart_sendbuf
 ****************************************************************************/

static ssize_t cdcuart_sendbuf(FAR struct uart_dev_s *dev,
                               FAR const void *buf, size_t len)
{
  FAR struct cdcacm_serial_priv_s *sp = dev->priv;
  FAR struct cdcacm_dev_s *priv = sp->cdc_dev;
  FAR struct usbdev_ep_s *ep = priv->epbulkin;
  FAR struct cdcacm_wrreq_s *wrcontainer;
  FAR struct usbdev_req_s *req;
  irqstate_t flags;
  size_t reqlen;
  size_t nbytes;
  int ret;

  /* Use full request buffer; USB stack chunks into maxpacket-sized
   * packets internally (same fix as cdcuart_dmasend).
   */

  reqlen = CONFIG_CDCACM_BULKIN_REQLEN;

  flags = spin_lock_irqsave(&priv->lock);
  wrcontainer = (FAR struct cdcacm_wrreq_s *)sq_remfirst(&priv->txfree);
  req = wrcontainer->req;
  priv->nwrq--;
  spin_unlock_irqrestore(&priv->lock, flags);

  nbytes = MIN(reqlen, len);
  memcpy(req->buf, buf, nbytes);

  req->len   = nbytes;
  req->priv  = wrcontainer;
  req->flags = USBDEV_REQFLAGS_NULLPKT;
  priv->ispolling = true;
  ret        = EP_SUBMIT(ep, req);
  priv->ispolling = false;
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SUBMITFAIL),
               (uint16_t)-ret);
      return ret;
    }

  return nbytes;
}

#ifndef CONFIG_CDCACM_DISABLE_TXBUF

/****************************************************************************
 * Name: cdcuart_dmasend
 ****************************************************************************/

static void cdcuart_dmasend(FAR struct uart_dev_s *dev)
{
  FAR struct cdcacm_serial_priv_s *sp = dev->priv;
  FAR struct cdcacm_dev_s *priv = sp->cdc_dev;
  FAR struct usbdev_ep_s *ep = priv->epbulkin;
  FAR struct cdcacm_wrreq_s *wrcontainer;
  FAR struct usbdev_req_s *req;
  FAR struct uart_buffer_s *txbuf = &dev->xmit;
  irqstate_t flags;
  size_t reqlen;
  size_t nbytes;
  size_t cp1;
  size_t cp2;
  int ret;

  /* Use full request buffer; USB stack chunks into maxpacket-sized
   * packets internally.  Capping reqlen at maxpacket is the legacy
   * cdcacm.c bug -- it forces one EP_SUBMIT per USB packet and halves
   * sustained ACM IN throughput.
   */

  reqlen = CONFIG_CDCACM_BULKIN_REQLEN;

  /* Race fix: do not trust dev->dmatx (the xfer struct set by
   * uart_xmitchars_dma).  Two concurrent callers (wrcomplete IRQ via
   * cdcacm_sndpacket/pull_tx + cdcuart_txint task) both go through
   * uart_xmitchars_dma, each overwriting dev->dmatx before either
   * reads it here -- the same xmit window gets copied into two
   * wrreqs and the tail advanced twice (duplication and corruption).
   * Instead claim a wrcontainer + read the dev->xmit ring head/tail +
   * memcpy + advance tail ALL under priv->lock (== sp->cdc_dev->lock,
   * the same lock guarding txfree and pull_tx).  EP_SUBMIT runs
   * outside the lock to avoid AB/BA with the controller driver lock.
   */

  flags = spin_lock_irqsave(&priv->lock);

  if (txbuf->head == txbuf->tail)
    {
      /* Concurrent caller already drained the ring. */

      spin_unlock_irqrestore(&priv->lock, flags);
      return;
    }

  wrcontainer = (FAR struct cdcacm_wrreq_s *)sq_remfirst(&priv->txfree);
  if (wrcontainer == NULL)
    {
      spin_unlock_irqrestore(&priv->lock, flags);
      return;
    }

  req = wrcontainer->req;
  priv->nwrq--;

  /* Compute wrap-around copy spans from the current head/tail. */

  if (txbuf->tail < txbuf->head)
    {
      cp1 = MIN(reqlen, (size_t)(txbuf->head - txbuf->tail));
      cp2 = 0;
    }
  else
    {
      cp1 = MIN(reqlen, (size_t)(txbuf->size - txbuf->tail));
      cp2 = MIN(reqlen - cp1, (size_t)txbuf->head);
    }

  memcpy(req->buf, &txbuf->buffer[txbuf->tail], cp1);
  if (cp2)
    {
      memcpy(req->buf + cp1, txbuf->buffer, cp2);
    }

  req->len = cp1 + cp2;
  nbytes   = req->len;

  txbuf->tail = (txbuf->tail + nbytes) % txbuf->size;

  spin_unlock_irqrestore(&priv->lock, flags);

  /* Wake any writer blocked on free space (outside the lock). */

  if (nbytes)
    {
      uart_datasent(dev);
    }

  req->priv  = wrcontainer;
  req->flags = USBDEV_REQFLAGS_NULLPKT;
  ret        = EP_SUBMIT(ep, req);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SUBMITFAIL),
               (uint16_t)-ret);
    }
}
#endif

#ifndef CONFIG_CDCACM_DISABLE_RXBUF

/****************************************************************************
 * Name: cdcuart_dmareceive
 ****************************************************************************/

static void cdcuart_dmareceive(FAR struct uart_dev_s *dev)
{
  FAR struct uart_dmaxfer_s *xfer = &dev->dmarx;
  FAR struct cdcacm_serial_priv_s *sp = dev->priv;
  FAR struct cdcacm_dev_s *priv = sp->cdc_dev;
  FAR struct cdcacm_rdreq_s *rdcontainer;
  FAR struct usbdev_req_s *req;
  FAR uint8_t *reqbuf;
  size_t nbytes = 0;
  size_t reqlen;

  rdcontainer = (FAR struct cdcacm_rdreq_s *)
    sq_peek(&priv->rxpending);
  DEBUGASSERT(rdcontainer != NULL);

  req = rdcontainer->req;
  DEBUGASSERT(req != NULL);

  reqbuf = &req->buf[rdcontainer->offset];
  reqlen = req->xfrd - rdcontainer->offset;

  nbytes = MIN(reqlen, xfer->length);
  memcpy(xfer->buffer, reqbuf, nbytes);
  rdcontainer->offset += nbytes;
  xfer->nbytes = nbytes;

  if (xfer->nbuffer)
    {
      nbytes = MIN(reqlen - nbytes, xfer->nlength);
      memcpy(xfer->nbuffer, reqbuf + xfer->nbytes, nbytes);
      rdcontainer->offset += nbytes;
      xfer->nbytes += nbytes;
    }

  uart_recvchars_done(dev);

  if (rdcontainer->offset >= rdcontainer->req->xfrd)
    {
      sq_remfirst(&priv->rxpending);
      cdcacm_requeue_rdrequest(priv, rdcontainer);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cdcacm_initialize
 *
 * Description:
 *   Allocate the uart adapter, register the underlying cdcacm USB
 *   device, install the user_ops table that bridges ACM events into
 *   uart semantics, and finally register /dev/ttyACMn (and /dev/console
 *   when CDCACM_CONSOLE selects this instance).
 *
 *   Signature is preserved for backward compatibility with all existing
 *   board callers; the returned handle is the cdcacm_dev_s handle, the
 *   same value that cdcacm_uninitialize accepts.
 *
 ****************************************************************************/

#ifndef CONFIG_CDCACM_COMPOSITE
int cdcacm_initialize(int minor, FAR void **handle)
{
  FAR struct cdcacm_serial_priv_s *sp;
  struct usbdev_devinfo_s devinfo;
  char devname[CDCACM_DEVNAME_SIZE];
  int ret;

  /* Allocate adapter state.  Includes the embedded uart_dev_s */

  sp = (FAR struct cdcacm_serial_priv_s *)kmm_zalloc(sizeof(*sp));
  if (sp == NULL)
    {
      return -ENOMEM;
    }

  /* Init linecoding defaults: 115200/8/N/1. */

  sp->linecoding.baud[0] = (115200) & 0xff;
  sp->linecoding.baud[1] = (115200 >> 8) & 0xff;
  sp->linecoding.baud[2] = (115200 >> 16) & 0xff;
  sp->linecoding.baud[3] = (115200 >> 24) & 0xff;
  sp->linecoding.stop    = CDC_CHFMT_STOP1;
  sp->linecoding.parity  = CDC_PARITY_NONE;
  sp->linecoding.nbits   = 8;

  /* Initialise the uart upper-half struct */

#ifdef CONFIG_SERIAL_REMOVABLE
  sp->serdev.disconnected = true;
#endif

#ifndef CONFIG_CDCACM_DISABLE_RXBUF
  sp->serdev.recv.size   = CONFIG_CDCACM_RXBUFSIZE;
  sp->serdev.recv.buffer = (FAR char *)kmm_zalloc(CONFIG_CDCACM_RXBUFSIZE);
  if (sp->serdev.recv.buffer == NULL)
    {
      ret = -ENOMEM;
      goto errout_free_sp;
    }
#endif

#ifndef CONFIG_CDCACM_DISABLE_TXBUF
  sp->serdev.xmit.size   = CONFIG_CDCACM_TXBUFSIZE;
  sp->serdev.xmit.buffer = (FAR char *)kmm_zalloc(CONFIG_CDCACM_TXBUFSIZE);
  if (sp->serdev.xmit.buffer == NULL)
    {
      ret = -ENOMEM;
      goto errout_free_recv;
    }
#endif

  sp->serdev.ops  = &g_uartops;
  sp->serdev.priv = sp;

  /* Allocate and register the cdcacm USB protocol core.  cdcacm_register
   * handles the alloc + ops binding + usbdev_register in one shot.
   */

  memset(&devinfo, 0, sizeof(devinfo));
  devinfo.ninterfaces = CDCACM_NINTERFACES;
  devinfo.nstrings    = CDCACM_NSTRIDS;
  devinfo.nendpoints  = CDCACM_NUM_EPS;
  devinfo.epno[CDCACM_EP_INTIN_IDX]   = CONFIG_CDCACM_EPINTIN;
  devinfo.epno[CDCACM_EP_BULKIN_IDX]  = CONFIG_CDCACM_EPBULKIN;
  devinfo.epno[CDCACM_EP_BULKOUT_IDX] = CONFIG_CDCACM_EPBULKOUT;

  ret = cdcacm_register(minor, &g_cdcacm_user_ops, sp, &devinfo,
                        &sp->cdc_dev);
  if (ret < 0)
    {
      goto errout_free_xmit;
    }

#ifdef CONFIG_CDCACM_CONSOLE
  if (minor == 0)
    {
      sp->serdev.isconsole = true;

      ret = uart_register("/dev/console", &sp->serdev);
      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONSOLEREGISTER),
                   (uint16_t)-ret);
          goto errout_unreg_cdc;
        }
    }
#endif

  snprintf(devname, sizeof(devname), CDCACM_DEVNAME_FORMAT, minor);
  ret = uart_register(devname, &sp->serdev);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UARTREGISTER),
               (uint16_t)-ret);
      goto errout_unreg_console;
    }

  if (handle != NULL)
    {
      /* Return the usbdevclass_driver_s pointer to mirror the legacy
       * cdcacm_initialize contract that boards still rely on -- it is
       * the same value that cdcacm_uninitialize accepts.
       */

      *handle = &((FAR struct cdcacm_alloc_s *)sp->cdc_dev)->drvr.drvr;
    }

  return OK;

errout_unreg_console:
#ifdef CONFIG_CDCACM_CONSOLE
  if (minor == 0)
    {
      unregister_driver("/dev/console");
    }

errout_unreg_cdc:
#endif
  cdcacm_unregister(sp->cdc_dev);
  sp->cdc_dev = NULL;

errout_free_xmit:
#ifndef CONFIG_CDCACM_DISABLE_TXBUF
  kmm_free(sp->serdev.xmit.buffer);
errout_free_recv:
#endif
#ifndef CONFIG_CDCACM_DISABLE_RXBUF
  kmm_free(sp->serdev.recv.buffer);
errout_free_sp:
#endif
  kmm_free(sp);
  return ret;
}
#endif

/****************************************************************************
 * Name: cdcacm_uninitialize
 *
 * Description:
 *   Tear down a cdcacm_initialize-created instance, OR the composite
 *   USB framework's per-class teardown hook.  Order:
 *
 *     1. Unregister /dev/ttyACMn -- this drops the uart upper half's
 *        last reference, which fires cdcuart_release to free sp and its
 *        rx/xmit buffers.  After this call, no more user_ops dispatch
 *        can happen against the freed sp.
 *     2. Standalone: cdcacm_unregister tears down the USB class driver
 *        (cancels EPs, frees ctrlreq / rd / wr request pools, drains
 *        refcount, frees the cdc_dev).
 *        Composite: the composite framework already owns
 *        usbdev_unregister; we just free the cdc_dev resources via
 *        cdcacm_unregister which is safe because closing=true gates
 *        further ops dispatch.
 *
 ****************************************************************************/

void cdcacm_uninitialize(FAR struct usbdevclass_driver_s *classdev)
{
  FAR struct cdcacm_driver_s *cdcdrvr =
    (FAR struct cdcacm_driver_s *)classdev;
  FAR struct cdcacm_dev_s    *priv    = cdcdrvr->dev;
  char devname[CDCACM_DEVNAME_SIZE];
  int minor = priv->minor;
  int ret;

  /* Snapshot the minor (above) before tearing down the core:
   * cdcacm_unregister frees priv, so priv->minor must not be read after.
   *
   * Tear down the USB protocol core FIRST.  cdcacm_unregister detaches
   * user_ops (so no callback can dispatch into the adapter state), then
   * usbdev_unregister cancels the endpoints, unbinds, drains the refcount
   * and frees priv.  Crucially this happens while the adapter state (sp,
   * registered as the tty's dev->priv) is still alive, so the on_connect /
   * on_rx dispatched during unbind are safe.  Only after the core can no
   * longer dispatch do we unregister the TTY, which frees sp via
   * cdcuart_release.  (The reverse order frees sp while the bulk-OUT
   * endpoint is still armed, so an in-flight completion would dispatch
   * on_rx into freed sp -- a use-after-free.)
   */

  cdcacm_unregister(priv);

  snprintf(devname, sizeof(devname), CDCACM_DEVNAME_FORMAT, minor);
  ret = unregister_driver(devname);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UARTUNREGISTER),
               (uint16_t)-ret);
    }

#ifdef CONFIG_CDCACM_CONSOLE
  if (minor == 0)
    {
      unregister_driver("/dev/console");
    }
#endif
}
