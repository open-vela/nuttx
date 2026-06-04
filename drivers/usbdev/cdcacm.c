/****************************************************************************
 * drivers/usbdev/cdcacm.c
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
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <fcntl.h>
#include <poll.h>

#include <nuttx/spinlock.h>
#include <nuttx/mutex.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/queue.h>
#include <nuttx/semaphore.h>
#include <nuttx/wdog.h>
#include <nuttx/arch.h>
#include <nuttx/atomic.h>
#include <nuttx/fs/fs.h>
#include <nuttx/serial/serial.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/cdc.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/usbdev_trace.h>

#include "cdcacm.h"
#include "cdcacm_internal.h"

#ifdef CONFIG_CDCACM_COMPOSITE
#  include <nuttx/usb/composite.h>
#  include "composite.h"
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Transfer helpers *********************************************************/

#ifdef CONFIG_CDCACM_DISABLE_RXBUF
static void    cdcacm_rcvpacket(FAR struct cdcacm_dev_s *priv);
#endif
#ifndef CONFIG_CDCACM_DISABLE_RXBUF
static void    cdcacm_rxtimeout(wdparm_t arg);
#endif

/* user_ops borrow helper ***************************************************/

static FAR const struct cdcacm_user_ops_s *
                cdcacm_borrow_ops(FAR struct cdcacm_dev_s *priv);

/* Configuration ************************************************************/

static void    cdcacm_resetconfig(FAR struct cdcacm_dev_s *priv);
static int     cdcacm_epconfigure(FAR struct usbdev_ep_s *ep,
                 enum cdcacm_epdesc_e epid, bool last,
                 FAR struct usbdev_devinfo_s *devinfo,
                 uint8_t speed);
static int     cdcacm_setconfig(FAR struct cdcacm_dev_s *priv,
                 uint8_t config);

/* Completion event handlers ************************************************/

static void    cdcacm_ep0incomplete(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);
static void    cdcacm_rdcomplete(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);
static void    cdcacm_wrcomplete(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);

/* USB class device *********************************************************/

static int     cdcacm_bind(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static void    cdcacm_unbind(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static int     cdcacm_setup(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev,
                 FAR const struct usb_ctrlreq_s *ctrl, FAR uint8_t *dataout,
                 size_t outlen);
static void    cdcacm_disconnect(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
#ifdef CONFIG_SERIAL_REMOVABLE
static void    cdcacm_suspend(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static void    cdcacm_resume(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
#endif

/* /dev/cdcacmN chardev *****************************************************/

static int     cdcacm_chardev_open(FAR struct file *filep);
static int     cdcacm_chardev_close(FAR struct file *filep);
static ssize_t cdcacm_chardev_read(FAR struct file *filep, FAR char *buf,
                 size_t len);
static ssize_t cdcacm_chardev_write(FAR struct file *filep,
                 FAR const char *buf, size_t len);
static int     cdcacm_chardev_poll(FAR struct file *filep,
                 FAR struct pollfd *fds, bool setup);

/* Shared TX submit helper (used by chardev_write and outstream_puts) */

static ssize_t cdcacm_internal_submit(FAR struct cdcacm_dev_s *priv,
                 FAR const void *buf, size_t len, bool nonblock);

/* user_ops install/detach, internal to register/unregister */

static int     cdcacm_set_user_ops(FAR struct cdcacm_dev_s *dev,
                 FAR const struct cdcacm_user_ops_s *ops,
                 FAR void *user_priv);
static void    cdcacm_clear_user_ops(FAR struct cdcacm_dev_s *dev);

/* lib_outstream_s methods for cdcacm_outstream_s ***************************/

static void    cdcacm_outstream_putc(FAR struct lib_outstream_s *self,
                 int ch);
static ssize_t cdcacm_outstream_puts(FAR struct lib_outstream_s *self,
                 FAR const void *buf, size_t len);
static int     cdcacm_outstream_flush(FAR struct lib_outstream_s *self);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_CDCACM
static FAR struct cdcacm_dev_s *g_syslog_cdcacm;
#endif

/* USB class device *********************************************************/

static const struct usbdevclass_driverops_s g_driverops =
{
  cdcacm_bind,           /* bind */
  cdcacm_unbind,         /* unbind */
  cdcacm_setup,          /* setup */
  cdcacm_disconnect,     /* disconnect */
#ifdef CONFIG_SERIAL_REMOVABLE
  cdcacm_suspend,        /* suspend */
  cdcacm_resume,         /* resume */
#else
  NULL,                  /* suspend */
  NULL,                  /* resume */
#endif
};

/* /dev/cdcacmN file_operations *********************************************/

static const struct file_operations g_cdcacm_chardev_fops =
{
  cdcacm_chardev_open,   /* open */
  cdcacm_chardev_close,  /* close */
  cdcacm_chardev_read,   /* read */
  cdcacm_chardev_write,  /* write */
  NULL,                  /* seek */
  NULL,                  /* ioctl */
  NULL,                  /* truncate */
  cdcacm_chardev_poll    /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cdcacm_sndpacket
 *
 * Description:
 *   This function obtains write requests, transfers the TX data into the
 *   request, and submits the requests to the USB controller.  This
 *   continues until either (1) there are no further packets available, or
 *   (2) there is no further data to send.
 *
 ****************************************************************************/

int cdcacm_sndpacket(FAR struct cdcacm_dev_s *priv)
{
  FAR const struct cdcacm_user_ops_s *ops;

#ifdef CONFIG_DEBUG_FEATURES
  if (priv == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EINVAL;
    }
#endif

  if (priv->ispolling)
    {
      return OK;
    }

  uinfo("nwrq=%d empty=%d\n", priv->nwrq, sq_empty(&priv->txfree));

  ops = cdcacm_borrow_ops(priv);

  /* Without a user_ops adapter installed (chardev / outstream consumers)
   * cdcacm_sndpacket is a no-op: cdcacm core has no buffer of its own to
   * drain.  Those consumers submit IN data directly via cdcacm_chardev_write
   * and the outstream puts path, each of which pops a wrcontainer and
   * EP_SUBMITs without routing through cdcacm_sndpacket.
   *
   * With an adapter, the pull_tx callback is responsible for filling the
   * IN packet from the adapter's xmit ring.  In DISABLE_TXBUF=y mode the
   * adapter aliases the xmit ring onto wrcontainer->req->buf (handled
   * once at bind time via claim_xmit_buf, and rotated on each
   * wrcomplete) -- pull_tx still copies bytes if any are pending.
   */

  if (ops == NULL || ops->pull_tx == NULL)
    {
      return OK;
    }

  if (!sq_empty(&priv->txfree))
    {
      FAR struct usbdev_ep_s *ep = priv->epbulkin;
      FAR struct cdcacm_wrreq_s *wrc;
      irqstate_t flags;
      size_t reqlen;
      int len;
      int ret;

      /* USB stack splits reqlen into maxpacket-sized USB packets
       * internally; capping reqlen at maxpacket here would force
       * one EP_SUBMIT per packet and shred IN throughput.
       */

      reqlen = CONFIG_CDCACM_BULKIN_REQLEN;

      flags = spin_lock_irqsave(&priv->lock);
      wrc = (FAR struct cdcacm_wrreq_s *)sq_remfirst(&priv->txfree);
      priv->nwrq--;
      spin_unlock_irqrestore(&priv->lock, flags);

      len = ops->pull_tx(priv, wrc->req->buf, reqlen);
      if (len > 0)
        {
          wrc->req->len   = len;
          wrc->req->priv  = wrc;
          wrc->req->flags = USBDEV_REQFLAGS_NULLPKT;
          ret = EP_SUBMIT(ep, wrc->req);
          if (ret < 0)
            {
              usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SUBMITFAIL),
                       (uint16_t)-ret);
              flags = spin_lock_irqsave(&priv->lock);
              sq_addfirst((FAR sq_entry_t *)wrc, &priv->txfree);
              priv->nwrq++;
              spin_unlock_irqrestore(&priv->lock, flags);
            }
        }
      else
        {
          flags = spin_lock_irqsave(&priv->lock);
          sq_addfirst((FAR sq_entry_t *)wrc, &priv->txfree);
          priv->nwrq++;
          spin_unlock_irqrestore(&priv->lock, flags);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: cdcacm_requeue_rdrequest
 *
 * Description:
 *   Add any pending RX packets to the upper half serial drivers RX buffer.
 *
 ****************************************************************************/

int cdcacm_requeue_rdrequest(FAR struct cdcacm_dev_s *priv,
                             FAR struct cdcacm_rdreq_s *rdcontainer)
{
  FAR struct usbdev_req_s *req;
  FAR struct usbdev_ep_s *ep;
  int ret;

  DEBUGASSERT(priv != NULL && rdcontainer != NULL);
  rdcontainer->offset = 0;

  req      = rdcontainer->req;
  DEBUGASSERT(req != NULL);

  /* Requeue the read request */

  ep       = priv->epbulkout;
  req->len = MIN(CONFIG_CDCACM_BULKOUT_REQLEN, ep->maxpacket);
  ret      = EP_SUBMIT(ep, req);
  if (ret != OK)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSUBMIT),
                              (uint16_t)-req->result);
    }

  return ret;
}

/****************************************************************************
 * Name: cdcacm_release_rxpending
 *
 * Description:
 *   Add any pending RX packets to the upper half serial drivers RX buffer.
 *
 ****************************************************************************/

int cdcacm_release_rxpending(FAR struct cdcacm_dev_s *priv)
{
  FAR const struct cdcacm_user_ops_s *ops;
  irqstate_t flags;
  int ret = -EBUSY;

  /* Note that the priv->rxpending queue, priv->rxenabled, priv->iactive
   * may be modified by interrupt level processing and, hence, interrupts
   * must be disabled throughout the following.
   */

  flags = spin_lock_irqsave_nopreempt(&priv->lock);

  if (priv->ispolling)
    {
      goto out;
    }

  /* Cancel any pending failsafe timer */

#ifndef CONFIG_CDCACM_DISABLE_RXBUF
  wd_cancel(&priv->rxfailsafe);
#endif

  /* If RX "interrupts" are enabled and if input flow control is not in
   * effect, then pass the packet at the head of the pending RX packet list
   * to the upper serial layer.  Otherwise, let the packet continue to pend
   * the priv->rxpending list until the upper serial layer is able to buffer
   * it.
   */

#ifdef CONFIG_CDCACM_IFLOWCONTROL
  if (priv->rxenabled && !priv->iactive)
#else
  if (priv->rxenabled)
#endif
    {
      /* Process pending RX packets while the queue is not empty and while
       * no errors occur.  NOTE that the priv->rxpending queue is accessed
       * from interrupt level processing and, hence, interrupts must be
       * disabled throughout the following.
       */

      ret = OK;

      /* Data lives in priv->rxpending.  Notification flow depends on the
       * installed consumer:
       *   - user_ops adapter present: dispatch on_rx (uart adapter wakes
       *     its read waiter).  In DISABLE_RXBUF=y mode cdcacm_rcvpacket
       *     also rotates the head packet into priv->rdcontainer so the
       *     adapter's pull_rx path can alias it.
       *   - no user_ops (chardev / outstream consumers): post rx_waitsem
       *     so any chardev reader blocked in cdcacm_chardev_read wakes
       *     and drains rxpending.
       */

      ops = cdcacm_borrow_ops(priv);

      if (ops != NULL && ops->on_rx != NULL)
        {
#ifdef CONFIG_CDCACM_DISABLE_RXBUF
          cdcacm_rcvpacket(priv);
#else
          if (!sq_empty(&priv->rxpending))
            {
              ops->on_rx(priv, NULL, 0);
            }
#endif
        }
      else if (!sq_empty(&priv->rxpending))
        {
          nxsem_post(&priv->rx_waitsem);
        }
    }

  /* Restart the RX failsafe timer if there are RX packets in
   * priv->rxpending.  This could happen if either RX "interrupts" are
   * disable, RX flow control is in effect of if the upper serial drivers
   * RX buffer is full and cannot accept additional data.
   *
   * If/when the timer expires, cdcacm_release_rxpending() will be called
   * the timer handler (at interrupt level).
   *
   * The timer may not be necessary, but it is a failsafe to be certain
   * that data cannot stall in priv->rxpending.
   */

#ifndef CONFIG_CDCACM_DISABLE_RXBUF
  if (!sq_empty(&priv->rxpending))
    {
      wd_start(&priv->rxfailsafe, CDCACM_RXDELAY,
               cdcacm_rxtimeout, (wdparm_t)priv);
    }
#endif

out:
  spin_unlock_irqrestore_nopreempt(&priv->lock, flags);
  return ret;
}

#ifndef CONFIG_CDCACM_DISABLE_RXBUF

/****************************************************************************
 * Name: cdcacm_rxtimeout
 *
 * Description:
 *   Timer expiration handler.  Whenever cdcacm_release_rxpending()
 *   terminates with  pending RX data in priv->rxpending, it will set a
 *   timer to recheck the queued RX data can be processed later.  This
 *   failsafe timer may not be necessary, but this reduces my  paranoia
 *   about stalls in the RX pending FIFO .
 *
 ****************************************************************************/

static void cdcacm_rxtimeout(wdparm_t arg)
{
  FAR struct cdcacm_dev_s *priv = (FAR struct cdcacm_dev_s *)arg;

  DEBUGASSERT(priv != NULL);
  cdcacm_release_rxpending(priv);
}

#endif

/****************************************************************************
 * Name: cdcacm_serialstate
 *
 * Description:
 *   Send the serial state message.
 *
 * 1. Format and send a request header with:
 *
 *   bmRequestType:
 *    USB_REQ_DIR_IN | USB_REQ_TYPE_CLASS |
 *    USB_REQ_RECIPIENT_INTERFACE
 *   bRequest: ACM_SERIAL_STATE
 *   wValue: 0
 *   wIndex: 0
 *   wLength: Length of data = 2
 *
 * 2. Followed by the notification data
 *
 ****************************************************************************/

#ifdef CONFIG_CDCACM_IFLOWCONTROL
int cdcacm_serialstate(FAR struct cdcacm_dev_s *priv)
{
  FAR struct usbdev_ep_s *ep;
  FAR struct usbdev_req_s *req;
  FAR struct cdcacm_wrreq_s *wrcontainer;
  FAR struct cdc_notification_s *notify;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv != NULL && priv->epintin != NULL);
#ifdef CONFIG_DEBUG_FEATURES
  if (priv == NULL || priv->epintin == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EINVAL;
    }
#endif

  usbtrace(CDCACM_CLASSAPI_FLOWCONTROL, (uint16_t)priv->serialstate);

  /* Use our interrupt IN endpoint for the transfer */

  ep = priv->epintin;

  /* Remove the next container from the request list */

  flags = spin_lock_irqsave(&priv->lock);

  wrcontainer = (FAR struct cdcacm_wrreq_s *)sq_remfirst(&priv->txfree);
  if (wrcontainer == NULL)
    {
      ret = -ENOMEM;
      spin_unlock_irqrestore(&priv->lock, flags);
      goto errout_with_flags;
    }

  /* Decrement the count of write requests */

  priv->nwrq--;
  spin_unlock_irqrestore(&priv->lock, flags);

  /* Format the SerialState notification */

  DEBUGASSERT(wrcontainer->req != NULL);
  req                  = wrcontainer->req;

  DEBUGASSERT(req->buf != NULL);
  notify               = (FAR struct cdc_notification_s *)req->buf;

  notify->type         = (USB_REQ_DIR_IN | USB_REQ_TYPE_CLASS |
                          USB_REQ_RECIPIENT_INTERFACE);
  notify->notification = ACM_SERIAL_STATE;
  notify->value[0]     = 0;
  notify->value[1]     = 0;
  notify->index[0]     = 0;
  notify->index[1]     = 0;
  notify->len[0]       = 2;
  notify->len[1]       = 0;
  notify->data[0]      = priv->serialstate;
  notify->data[1]      = 0;

  /* Then submit the request to the endpoint */

  req->len             = SIZEOF_NOTIFICATION_S(2);
  req->priv            = wrcontainer;
  req->flags           = USBDEV_REQFLAGS_NULLPKT;
  ret                  = EP_SUBMIT(ep, req);

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SUBMITFAIL), (uint16_t)-ret);
    }

errout_with_flags:

  /* Reset all of the "irregular" notification */

  priv->serialstate &= CDC_UART_CONSISTENT;

  return ret;
}
#endif

/****************************************************************************
 * Name: cdcacm_resetconfig
 *
 * Description:
 *   Mark the device as not configured and disable all endpoints.
 *
 ****************************************************************************/

static void cdcacm_resetconfig(FAR struct cdcacm_dev_s *priv)
{
  FAR const struct cdcacm_user_ops_s *ops;

  /* When the USB is pulled out, if there is an unprocessed buffer,
   * it needs to be push them to upper half serial drivers RX buffer.
   */

  if (priv->nrdq != 0)
    {
      cdcacm_release_rxpending(priv);
      priv->nrdq = 0;
    }

  /* Are we configured? */

  if (priv->config != CDCACM_CONFIGIDNONE)
    {
      /* Yes.. but not anymore */

      priv->config = CDCACM_CONFIGIDNONE;

      /* Inform the "upper half" driver that there is no (functional) USB
       * connection.  When no user_ops adapter is installed (chardev /
       * outstream consumers) this is a no-op -- such consumers neither
       * need nor want SERIAL_REMOVABLE-style notifications.
       */

      ops = cdcacm_borrow_ops(priv);
      if (ops != NULL && ops->on_connect != NULL)
        {
          ops->on_connect(priv, false);
        }

      /* Disable endpoints.  This should force completion of all pending
       * transfers.
       */

#ifdef CONFIG_CDCACM_HAVE_EPINTIN
      EP_DISABLE(priv->epintin);
#endif
      EP_DISABLE(priv->epbulkin);
      EP_DISABLE(priv->epbulkout);
    }
}

/****************************************************************************
 * Name: cdcacm_epconfigure
 *
 * Description:
 *   Configure one endpoint.
 *
 ****************************************************************************/

static int cdcacm_epconfigure(FAR struct usbdev_ep_s *ep,
                              enum cdcacm_epdesc_e epid, bool last,
                              FAR struct usbdev_devinfo_s *devinfo,
                              uint8_t speed)
{
  struct usb_ss_epdesc_s epdesc;
  cdcacm_copy_epdesc(epid, &epdesc.epdesc, devinfo, speed);
  return EP_CONFIGURE(ep, &epdesc.epdesc, last);
}

/****************************************************************************
 * Name: cdcacm_setconfig
 *
 * Description:
 *   Set the device configuration by allocating and configuring endpoints and
 *   by allocating and queue read and write requests.
 *
 ****************************************************************************/

static int cdcacm_setconfig(FAR struct cdcacm_dev_s *priv, uint8_t config)
{
  FAR const struct cdcacm_user_ops_s *ops;
  FAR struct usbdev_req_s *req;
  int i;
  int ret = 0;

#ifdef CONFIG_DEBUG_FEATURES
  if (priv == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EINVAL;
    }
#endif

  if (config == priv->config)
    {
      /* Already configured -- Do nothing */

      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALREADYCONFIGURED), 0);
      return 0;
    }

  /* Discard the previous configuration data */

  cdcacm_resetconfig(priv);

  /* Was this a request to simply discard the current configuration? */

  if (config == CDCACM_CONFIGIDNONE)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGNONE), 0);
      return 0;
    }

  /* We only accept one configuration */

  if (config != CDCACM_CONFIGID)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGIDBAD), 0);
      return -EINVAL;
    }

#ifdef CONFIG_CDCACM_HAVE_EPINTIN
  /* Configure the IN interrupt endpoint */

  ret = cdcacm_epconfigure(priv->epintin, CDCACM_EPINTIN, false,
                           &priv->devinfo, priv->usbdev->speed);

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTINCONFIGFAIL), 0);
      goto errout;
    }

  priv->epintin->priv = priv;
#endif

  /* Configure the IN bulk endpoint */

  ret = cdcacm_epconfigure(priv->epbulkin, CDCACM_EPBULKIN, false,
                           &priv->devinfo, priv->usbdev->speed);

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKINCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkin->priv = priv;

  /* Configure the OUT bulk endpoint */

  ret = cdcacm_epconfigure(priv->epbulkout, CDCACM_EPBULKOUT, true,
                           &priv->devinfo, priv->usbdev->speed);

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKOUTCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkout->priv = priv;

  /* Queue read requests in the bulk OUT endpoint */

  DEBUGASSERT(priv->nrdq == 0);
  for (i = 0; i < CONFIG_CDCACM_NRDREQS; i++)
    {
      req           = priv->rdreqs[i].req;
      req->callback = cdcacm_rdcomplete;

      /* Submit with len capped to one maxpacket, identical to
       * cdcacm_requeue_rdrequest.  The buffer is allocated larger
       * (BULKOUT_REQLEN) but if the submitted len is a multiple of
       * maxpacket, a host OUT transfer of exactly N*maxpacket bytes with
       * no terminating ZLP never satisfies the "short packet or buffer
       * full" completion rule -- the rdreq stays open and a reader blocks
       * forever.  Capping len to maxpacket makes every received packet
       * complete its request, matching the requeue path and avoiding the
       * initial-vs-requeued asymmetry.
       */

      req->len      = MIN(CONFIG_CDCACM_BULKOUT_REQLEN,
                          priv->epbulkout->maxpacket);
      ret           = EP_SUBMIT(priv->epbulkout, req);
      if (ret != OK)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSUBMIT),
                   (uint16_t)-ret);
          goto errout;
        }

      priv->nrdq++;
    }

  /* We are successfully configured */

  priv->config = config;

  /* Inform the "upper half" driver that we are "open for business".
   * When no user_ops adapter is installed (chardev / outstream consumers)
   * this is a no-op.
   */

  ops = cdcacm_borrow_ops(priv);
  if (ops != NULL && ops->on_connect != NULL)
    {
      ops->on_connect(priv, true);
    }

  return OK;

errout:
  cdcacm_resetconfig(priv);
  return ret;
}

/****************************************************************************
 * Name: cdcacm_ep0incomplete
 *
 * Description:
 *   Handle completion of EP0 control operations
 *
 ****************************************************************************/

static void cdcacm_ep0incomplete(FAR struct usbdev_ep_s *ep,
                                 FAR struct usbdev_req_s *req)
{
  if (req->result || req->xfrd != req->len)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_REQRESULT),
               (uint16_t)-req->result);
    }
}

/****************************************************************************
 * Name: cdcacm_rdcomplete
 *
 * Description:
 *   Handle completion of read request on the bulk OUT endpoint.  This
 *   is handled like the receipt of serial data on the "UART"
 *
 ****************************************************************************/

static void cdcacm_rdcomplete(FAR struct usbdev_ep_s *ep,
                              FAR struct usbdev_req_s *req)
{
  FAR struct cdcacm_rdreq_s *rdcontainer;
  FAR struct cdcacm_dev_s *priv;
  irqstate_t flags;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !ep->priv || !req)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract references to private data */

  priv = (FAR struct cdcacm_dev_s *)ep->priv;

  /* Get the container of the read request */

  rdcontainer = (FAR struct cdcacm_rdreq_s *)req->priv;
  DEBUGASSERT(rdcontainer != NULL);

  /* Process the received data unless this is some unusual condition */

  switch (req->result)
    {
    case 0: /* Normal completion */
      {
        usbtrace(TRACE_CLASSRDCOMPLETE, priv->nrdq);

        /* Place the incoming packet at the end of pending RX packet list. */

        flags = spin_lock_irqsave(&priv->lock);
        rdcontainer->offset = 0;
        sq_addlast((FAR sq_entry_t *)rdcontainer, &priv->rxpending);
        spin_unlock_irqrestore(&priv->lock, flags);

        /* Then process all pending RX packet starting at the head of the
         * list
         */

        cdcacm_release_rxpending(priv);
      }
      break;

    case -ESHUTDOWN: /* Disconnection */
      {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSHUTDOWN), 0);
        flags = spin_lock_irqsave(&priv->lock);
        if (priv->nrdq != 0)
          {
            priv->nrdq--;
          }

        spin_unlock_irqrestore(&priv->lock, flags);
      }
      break;

    default: /* Some other error occurred */
      {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDUNEXPECTED),
                                (uint16_t)-req->result);
        cdcacm_requeue_rdrequest(priv, rdcontainer);
        break;
      }
    }
}

/****************************************************************************
 * Name: cdcacm_wrcomplete
 *
 * Description:
 *   Handle completion of write request.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static void cdcacm_wrcomplete(FAR struct usbdev_ep_s *ep,
                              FAR struct usbdev_req_s *req)
{
  FAR struct cdcacm_dev_s *priv;
  FAR struct cdcacm_wrreq_s *wrcontainer;
#ifdef CONFIG_CDCACM_DISABLE_TXBUF
  FAR const struct cdcacm_user_ops_s *ops;
#endif
  irqstate_t flags;
  int sval = 0;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !ep->priv || !req || !req->priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract references to our private data */

  priv        = (FAR struct cdcacm_dev_s *)ep->priv;
  wrcontainer = (FAR struct cdcacm_wrreq_s *)req->priv;

  /* Return the write request to the free list */

  flags = spin_lock_irqsave(&priv->lock);
  sq_addlast((FAR sq_entry_t *)wrcontainer, &priv->txfree);
  priv->nwrq++;
  spin_unlock_irqrestore(&priv->lock, flags);

  /* Wake any TX submitter blocked on a wrcontainer.  Both chardev_write
   * and cdcacm_outstream_puts share cdcacm_internal_submit and may be
   * parked on tx_waitsem; gating only on chardev_open_count would leave
   * outstream-only waiters stranded (no chardev fd ever opens, yet a
   * stream consumer in task context is sleeping here).  Always post,
   * but clamp at one outstanding credit via sval <= 0 so a quiet system
   * with no waiter does not accumulate phantom credits -- the next
   * submitter consumes the credit before sleeping.
   */

  if (nxsem_get_value(&priv->tx_waitsem, &sval) == OK && sval <= 0)
    {
      nxsem_post(&priv->tx_waitsem);
    }

#ifdef CONFIG_CDCACM_DISABLE_TXBUF
  ops = cdcacm_borrow_ops(priv);
  if (ops != NULL && ops->release_xmit_buf != NULL)
    {
      ops->release_xmit_buf(priv, req->xfrd);
    }
#endif

  /* Send the next packet unless this was some unusual termination
   * condition
   */

  switch (req->result)
    {
    case OK: /* Normal completion */
      {
        usbtrace(TRACE_CLASSWRCOMPLETE, priv->nwrq);
#ifdef CONFIG_CDCACM_DISABLE_TXBUF
        if (priv->wrcontainer == NULL)
#endif
          {
            cdcacm_sndpacket(priv);
          }
      }
      break;

    case -ESHUTDOWN: /* Disconnection */
      {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRSHUTDOWN), priv->nwrq);
      }
      break;

    default: /* Some other error occurred */
      {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRUNEXPECTED),
                 (uint16_t)-req->result);
      }
      break;
    }
}

/****************************************************************************
 * USB Class Driver Methods
 ****************************************************************************/

/****************************************************************************
 * Name: cdcacm_bind
 *
 * Description:
 *   Invoked when the driver is bound to a USB device driver
 *
 ****************************************************************************/

static int cdcacm_bind(FAR struct usbdevclass_driver_s *driver,
                       FAR struct usbdev_s *dev)
{
  FAR struct cdcacm_dev_s *priv =
    ((FAR struct cdcacm_driver_s *)driver)->dev;
  FAR struct cdcacm_wrreq_s *wrcontainer;
  FAR struct cdcacm_rdreq_s *rdcontainer;
#ifdef CONFIG_CDCACM_DISABLE_TXBUF
  FAR const struct cdcacm_user_ops_s *ops;
#endif
  irqstate_t flags;
  size_t reqlen;
  int ret;
  int i;

  usbtrace(TRACE_CLASSBIND, 0);

  /* Bind the structures */

  priv->usbdev   = dev;

  /* Save the reference to our private data structure in EP0 so that it
   * can be recovered in ep0 completion events (Unless we are part of
   * a composite device and, in that case, the composite device owns
   * EP0).
   */

#ifndef CONFIG_CDCACM_COMPOSITE
  dev->ep0->priv = priv;
#endif

  /* Preallocate control request */

  priv->ctrlreq = usbdev_allocreq(dev->ep0, CDCACM_MXDESCLEN);
  if (priv->ctrlreq == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCCTRLREQ), 0);
      ret = -ENOMEM;
      goto errout;
    }

  priv->ctrlreq->callback = cdcacm_ep0incomplete;

  /* Pre-allocate all endpoints... the endpoints will not be functional
   * until the SET CONFIGURATION request is processed in cdcacm_setconfig.
   * This is done here because there may be calls to kmm_malloc and the SET
   * CONFIGURATION processing probably occurs within interrupt handling
   * logic where kmm_malloc calls will fail.
   */

#ifdef CONFIG_CDCACM_HAVE_EPINTIN
  /* Pre-allocate the IN interrupt endpoint */

  priv->epintin = DEV_ALLOCEP(dev, CDCACM_MKEPINTIN(&priv->devinfo),
                              true, USB_EP_ATTR_XFER_INT);
  if (!priv->epintin)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTINALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

  priv->epintin->priv = priv;
#endif

  /* Pre-allocate the IN bulk endpoint */

  priv->epbulkin = DEV_ALLOCEP(dev, CDCACM_MKEPBULKIN(&priv->devinfo),
                               true, USB_EP_ATTR_XFER_BULK);
  if (!priv->epbulkin)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKINALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

  priv->epbulkin->priv = priv;

  /* Pre-allocate the OUT bulk endpoint */

  priv->epbulkout = DEV_ALLOCEP(dev, CDCACM_MKEPBULKOUT(&priv->devinfo),
                                false, USB_EP_ATTR_XFER_BULK);
  if (!priv->epbulkout)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKOUTALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

  priv->epbulkout->priv = priv;

  /* Pre-allocate read requests.  The buffer size is one full packet. */
#if defined(CONFIG_USBDEV_SUPERSPEED)
  if (dev->speed == USB_SPEED_SUPER ||
      dev->speed == USB_SPEED_SUPER_PLUS)
    {
      if (CONFIG_CDCACM_EPBULKOUT_MAXBURST < USB_SS_BULK_EP_MAXBURST)
        {
          reqlen = CONFIG_CDCACM_EPBULKOUT_SSSIZE *
                   (CONFIG_CDCACM_EPBULKOUT_MAXBURST + 1);
        }
      else
        {
          reqlen = CONFIG_CDCACM_EPBULKOUT_SSSIZE *
                   USB_SS_BULK_EP_MAXBURST;
        }
    }
  else
#endif
#if defined(CONFIG_USBDEV_DUALSPEED)
  if (dev->speed == USB_SPEED_HIGH)
    {
      reqlen = CONFIG_CDCACM_EPBULKOUT_HSSIZE;
    }
  else
#endif
    {
      reqlen = CONFIG_CDCACM_EPBULKOUT_FSSIZE;
    }

  if (CONFIG_CDCACM_BULKOUT_REQLEN > reqlen)
    {
      reqlen = CONFIG_CDCACM_BULKOUT_REQLEN;
    }

  for (i = 0; i < CONFIG_CDCACM_NRDREQS; i++)
    {
      rdcontainer      = &priv->rdreqs[i];
      rdcontainer->req = usbdev_allocreq(priv->epbulkout, reqlen);
      if (rdcontainer->req == NULL)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDALLOCREQ), -ENOMEM);
          ret = -ENOMEM;
          goto errout;
        }

      rdcontainer->offset        = 0;
      rdcontainer->req->priv     = rdcontainer;
      rdcontainer->req->callback = cdcacm_rdcomplete;
    }

  /* Pre-allocate write request containers and put in a free list.  The
   * buffer size should be larger than a full build IN packet.  Otherwise,
   * we will send a bogus null packet at the end of each packet.
   *
   * Pick the larger of the max packet size and the configured request size.
   *
   * NOTE: These write requests are sized for the bulk IN endpoint but are
   * shared with interrupt IN endpoint which does not need a large buffer.
   */

#if defined(CONFIG_USBDEV_SUPERSPEED)
  if (dev->speed == USB_SPEED_SUPER ||
      dev->speed == USB_SPEED_SUPER_PLUS)
    {
      if (CONFIG_CDCACM_EPBULKIN_MAXBURST < USB_SS_BULK_EP_MAXBURST)
        {
          reqlen = CONFIG_CDCACM_EPBULKOUT_SSSIZE *
                   (CONFIG_CDCACM_EPBULKIN_MAXBURST + 1);
        }
      else
        {
          reqlen = CONFIG_CDCACM_EPBULKOUT_SSSIZE *
                   USB_SS_BULK_EP_MAXBURST;
        }
    }
  else
#endif
#if defined(CONFIG_USBDEV_DUALSPEED)
  if  (dev->speed == USB_SPEED_HIGH)
    {
      reqlen = CONFIG_CDCACM_EPBULKIN_HSSIZE;
    }
  else
#endif
    {
      reqlen = CONFIG_CDCACM_EPBULKIN_FSSIZE;
    }

  if (CONFIG_CDCACM_BULKIN_REQLEN > reqlen)
    {
      reqlen = CONFIG_CDCACM_BULKIN_REQLEN;
    }

  for (i = 0; i < CONFIG_CDCACM_NWRREQS; i++)
    {
      wrcontainer      = &priv->wrreqs[i];
      wrcontainer->req = usbdev_allocreq(priv->epbulkin, reqlen);
      if (wrcontainer->req == NULL)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRALLOCREQ), -ENOMEM);
          ret = -ENOMEM;
          goto errout;
        }

      wrcontainer->req->priv     = wrcontainer;
      wrcontainer->req->callback = cdcacm_wrcomplete;

      flags = spin_lock_irqsave(&priv->lock);
      sq_addlast((FAR sq_entry_t *)wrcontainer, &priv->txfree);
      priv->nwrq++;     /* Count of write requests available */
      spin_unlock_irqrestore(&priv->lock, flags);
    }

#ifdef CONFIG_CDCACM_DISABLE_TXBUF
  /* DISABLE_TXBUF: borrow one wrcontainer up front for the zero-copy alias.
   * The adapter (when registered) will alias its xmit ring at
   * priv->wrcontainer->req->buf via claim_xmit_buf; the size to alias is
   * reqlen + 1 (NULLPKT slack).  cdcacm core does not touch the alias
   * buffer itself -- it only owns the wrcontainer slot rotation.
   */

  flags = spin_lock_irqsave(&priv->lock);
  priv->wrcontainer = (FAR struct cdcacm_wrreq_s *)
                      sq_remfirst(&priv->txfree);
  priv->nwrq--;
  spin_unlock_irqrestore(&priv->lock, flags);

  ops = cdcacm_borrow_ops(priv);
  if (ops != NULL && ops->claim_xmit_buf != NULL)
    {
      FAR uint8_t *abuf = NULL;
      size_t       acap = 0;

      ops->claim_xmit_buf(priv, &abuf, &acap);
      UNUSED(abuf);
      UNUSED(acap);
    }
#endif

  /* Report if we are selfpowered (unless we are part of a
   * composite device)
   */

#ifndef CONFIG_CDCACM_COMPOSITE
#ifdef CONFIG_USBDEV_SELFPOWERED
  DEV_SETSELFPOWERED(dev);
#endif

  /* And pull-up the data line for the soft connect function (unless we are
   * part of a composite device)
   */

  DEV_CONNECT(dev);
#endif
  return OK;

errout:
  cdcacm_unbind(driver, dev);
  return ret;
}

/****************************************************************************
 * Name: cdcacm_unbind
 *
 * Description:
 *    Invoked when the driver is unbound from a USB device driver
 *
 ****************************************************************************/

static void cdcacm_unbind(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev)
{
  FAR struct cdcacm_dev_s *priv;
  FAR struct cdcacm_wrreq_s *wrcontainer;
  FAR struct cdcacm_rdreq_s *rdcontainer;
  irqstate_t flags;
  int i;

  usbtrace(TRACE_CLASSUNBIND, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract reference to private data */

  priv = ((FAR struct cdcacm_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return;
    }
#endif

  /* Make sure that we are not already unbound */

  if (priv != NULL)
    {
      /* Make sure that the endpoints have been unconfigured.  If
       * we were terminated gracefully, then the configuration should
       * already have been reset.  If not, then calling cdcacm_resetconfig
       * should cause the endpoints to immediately terminate all
       * transfers and return the requests to us (with result == -ESHUTDOWN)
       */

      cdcacm_resetconfig(priv);

      /* Free the pre-allocated control request */

      if (priv->ctrlreq != NULL)
        {
          usbdev_freereq(dev->ep0, priv->ctrlreq);
          priv->ctrlreq = NULL;
        }

      /* Free pre-allocated read requests (which should all have
       * been returned to the free list at this time -- we don't check)
       */

      DEBUGASSERT(priv->nrdq == 0);
      for (i = 0; i < CONFIG_CDCACM_NRDREQS; i++)
        {
          rdcontainer = &priv->rdreqs[i];
          if (rdcontainer->req)
            {
              usbdev_freereq(priv->epbulkout, rdcontainer->req);
              rdcontainer->req = NULL;
            }
        }

      /* Free write requests that are not in use (which should be all
       * of them)
       */

      flags = spin_lock_irqsave(&priv->lock);

#ifdef CONFIG_CDCACM_DISABLE_TXBUF
      DEBUGASSERT(priv->nwrq >= CONFIG_CDCACM_NWRREQS - 1);
      if (priv->wrcontainer)
        {
          sq_addlast((FAR sq_entry_t *)priv->wrcontainer, &priv->txfree);
          priv->wrcontainer = NULL;
          priv->nwrq++;
        }
#else
      DEBUGASSERT(priv->nwrq == CONFIG_CDCACM_NWRREQS);
#endif

      while (!sq_empty(&priv->txfree))
        {
          wrcontainer = (struct cdcacm_wrreq_s *)sq_remfirst(&priv->txfree);
          if (wrcontainer->req != NULL)
            {
              usbdev_freereq(priv->epbulkin, wrcontainer->req);
              priv->nwrq--;     /* Number of write requests queued */
            }
        }

      DEBUGASSERT(priv->nwrq == 0);
      spin_unlock_irqrestore(&priv->lock, flags);

#ifdef CONFIG_CDCACM_HAVE_EPINTIN
      /* Free the interrupt IN endpoint */

      if (priv->epintin)
        {
          DEV_FREEEP(dev, priv->epintin);
          priv->epintin = NULL;
        }
#endif

      /* Free the bulk OUT endpoint */

      if (priv->epbulkout)
        {
          DEV_FREEEP(dev, priv->epbulkout);
          priv->epbulkout = NULL;
        }

      /* Free the bulk IN endpoint */

      if (priv->epbulkin)
        {
          DEV_FREEEP(dev, priv->epbulkin);
          priv->epbulkin = NULL;
        }
    }
}

/****************************************************************************
 * Name: cdcacm_borrow_ops
 *
 * Description:
 *   Snapshot priv->user_ops under ops_lock and return it.  Uses a spinlock
 *   with IRQ save/restore so this helper is safe to call from any context,
 *   including the USB controller's request-complete IRQ (e.g. from
 *   cdcacm_wrcomplete -> cdcacm_sndpacket).  The returned pointer is valid
 *   for use by the caller in this single dispatch because
 *   cdcacm_set_user_ops is one-shot and cdcacm_clear_user_ops only runs
 *   after the adapter has unregistered itself, which by construction is
 *   sequential w.r.t. callbacks from the USB stack.
 *
 ****************************************************************************/

static FAR const struct cdcacm_user_ops_s *
cdcacm_borrow_ops(FAR struct cdcacm_dev_s *priv)
{
  FAR const struct cdcacm_user_ops_s *ops;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->ops_lock);
  ops = priv->user_ops;
  spin_unlock_irqrestore(&priv->ops_lock, flags);
  return ops;
}

/****************************************************************************
 * Name: cdcacm_setup
 *
 * Description:
 *   Invoked for ep0 control requests.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static int cdcacm_setup(FAR struct usbdevclass_driver_s *driver,
                        FAR struct usbdev_s *dev,
                        FAR const struct usb_ctrlreq_s *ctrl,
                        FAR uint8_t *dataout, size_t outlen)
{
  FAR struct cdcacm_dev_s *priv;
  FAR struct usbdev_req_s *ctrlreq;
  uint16_t value;
  uint16_t index;
  uint16_t len;
  int ret = -EOPNOTSUPP;

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev || !ctrl)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EINVAL;
    }
#endif

  /* Extract reference to private data */

  usbtrace(TRACE_CLASSSETUP, ctrl->req);
  priv = ((FAR struct cdcacm_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv || !priv->ctrlreq)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return -ENODEV;
    }
#endif

  ctrlreq = priv->ctrlreq;

  /* Extract the little-endian 16-bit values to host order */

  value = GETUINT16(ctrl->value);
  index = GETUINT16(ctrl->index);
  len   = GETUINT16(ctrl->len);

  uinfo("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        ctrl->type, ctrl->req, value, index, len);

  if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD)
    {
      /**********************************************************************
       * Standard Requests
       **********************************************************************/

      switch (ctrl->req)
        {
        case USB_REQ_GETDESCRIPTOR:
          {
            /* The value field specifies the descriptor type in the MS byte
             * and the descriptor index in the LS byte (order is little
             * endian)
             */

            switch (ctrl->value[1])
              {
              /* If the serial device is used in as part of a composite
               * device, then the device descriptor is provided by logic in
               * the composite device implementation.
               */

#ifndef CONFIG_CDCACM_COMPOSITE
              case USB_DESC_TYPE_DEVICE:
                {
                  ret = usbdev_copy_devdesc(ctrlreq->buf,
                                            cdcacm_getdevdesc(),
                                            dev->speed);
                }
                break;
#endif

              /* If the serial device is used in as part of a composite
               * device, then the device qualifier descriptor is provided by
               * logic in the composite device implementation.
               */

#if !defined(CONFIG_CDCACM_COMPOSITE) && defined(CONFIG_USBDEV_DUALSPEED)
              case USB_DESC_TYPE_DEVICEQUALIFIER:
                {
                  ret = USB_SIZEOF_QUALDESC;
                  memcpy(ctrlreq->buf, cdcacm_getqualdesc(), ret);
                }
                break;

              case USB_DESC_TYPE_OTHERSPEEDCONFIG:
#endif

              /* If the serial device is used in as part of a composite
               * device, then the configuration descriptor is provided by
               * logic in the composite device implementation.
               */

#ifndef CONFIG_CDCACM_COMPOSITE
              case USB_DESC_TYPE_CONFIG:
                {
                  ret = cdcacm_mkcfgdesc(ctrlreq->buf, &priv->devinfo,
                                         dev->speed, ctrl->value[1]);
                }
                break;
#endif

              /* If the serial device is used in as part of a composite
               * device, then the language string descriptor is provided by
               * logic in the composite device implementation.
               */

#ifndef CONFIG_CDCACM_COMPOSITE
              case USB_DESC_TYPE_STRING:
                {
                  /* index == language code. */

                  ret =
                  cdcacm_mkstrdesc(ctrl->value[0],
                                  (FAR struct usb_strdesc_s *)
                                    ctrlreq->buf);
                }
                break;
#endif

              default:
                {
                  usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_GETUNKNOWNDESC),
                           value);
                }
                break;
              }
          }
          break;

        case USB_REQ_SETCONFIGURATION:
          {
            if (ctrl->type == 0)
              {
                ret = cdcacm_setconfig(priv, value);
              }
          }
          break;

        /* If the serial device is used in as part of a composite device,
         * then the overall composite class configuration is managed by logic
         * in the composite device implementation.
         */

#ifndef CONFIG_CDCACM_COMPOSITE
        case USB_REQ_GETCONFIGURATION:
          {
            if (ctrl->type == USB_DIR_IN)
              {
                *(FAR uint8_t *)ctrlreq->buf = priv->config;
                ret = 1;
              }
          }
          break;
#endif

        case USB_REQ_SETINTERFACE:
          {
            if (ctrl->type == USB_REQ_RECIPIENT_INTERFACE &&
                priv->config == CDCACM_CONFIGID)
              {
                  if ((index == priv->devinfo.ifnobase &&
                       value == CDCACM_NOTALTIFID) ||
                      (index == (priv->devinfo.ifnobase + 1) &&
                       value == CDCACM_DATAALTIFID))
                  {
                    cdcacm_resetconfig(priv);
                    cdcacm_setconfig(priv, CDCACM_CONFIGID);
                    ret = 0;
                  }
              }
          }
          break;

        case USB_REQ_GETINTERFACE:
          {
            if (ctrl->type == (USB_DIR_IN | USB_REQ_RECIPIENT_INTERFACE) &&
                priv->config == CDCACM_CONFIGIDNONE)
              {
                  if ((index == priv->devinfo.ifnobase &&
                       value == CDCACM_NOTALTIFID) ||
                      (index == (priv->devinfo.ifnobase + 1) &&
                       value == CDCACM_DATAALTIFID))
                   {
                    *(FAR uint8_t *) ctrlreq->buf = value;
                    ret = 1;
                  }
                else
                  {
                    ret = -EDOM;
                  }
              }
           }
           break;

        default:
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDSTDREQ),
                   ctrl->req);
          break;
        }
    }

  else if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_CLASS)
    {
      /**********************************************************************
       * CDC ACM-Specific Requests
       **********************************************************************/

      switch (ctrl->req)
        {
        /* ACM_GET_LINE_CODING requests current DTE rate, stop-bits, parity,
         * and number-of-character bits. (Optional)
         */

        case ACM_GET_LINE_CODING:
          {
            if (ctrl->type == (USB_DIR_IN | USB_REQ_TYPE_CLASS |
                               USB_REQ_RECIPIENT_INTERFACE) &&
                index == priv->devinfo.ifnobase)
              {
                FAR const struct cdcacm_user_ops_s *ops =
                  cdcacm_borrow_ops(priv);

                if (ops != NULL && ops->get_line_coding != NULL)
                  {
                    ret = ops->get_line_coding(priv, ctrlreq->buf,
                                               SIZEOF_CDC_LINECODING);
                  }
                else
                  {
                    /* No adapter: respond with a sane default
                     * (115200/8/N/1) so the host's enumeration succeeds.
                     */

                    FAR struct cdc_linecoding_s *lc =
                      (FAR struct cdc_linecoding_s *)ctrlreq->buf;

                    memset(lc, 0, SIZEOF_CDC_LINECODING);
                    lc->baud[0] = (115200) & 0xff;
                    lc->baud[1] = (115200 >> 8) & 0xff;
                    lc->baud[2] = (115200 >> 16) & 0xff;
                    lc->baud[3] = (115200 >> 24) & 0xff;
                    lc->stop    = CDC_CHFMT_STOP1;
                    lc->parity  = CDC_PARITY_NONE;
                    lc->nbits   = 8;
                    ret = SIZEOF_CDC_LINECODING;
                  }
              }
            else
              {
                usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCLASSREQ),
                        ctrl->type);
              }
          }
          break;

        /* ACM_SET_LINE_CODING configures DTE rate, stop-bits, parity, and
         * number-of-character bits. (Optional)
         */

        case ACM_SET_LINE_CODING:
          {
            if (ctrl->type == (USB_DIR_OUT | USB_REQ_TYPE_CLASS |
                               USB_REQ_RECIPIENT_INTERFACE) &&
                len == SIZEOF_CDC_LINECODING && /* dataout && len == outlen && */
                index == priv->devinfo.ifnobase)
              {
                FAR const struct cdcacm_user_ops_s *ops =
                  cdcacm_borrow_ops(priv);

                if (ops != NULL && ops->set_line_coding != NULL)
                  {
                    ret = ops->set_line_coding(priv, dataout, outlen);
                  }
                else
                  {
                    /* No adapter: ACK the request (host-side line
                     * coding is meaningless for chardev / outstream
                     * consumers; the cdcacm core does not retain it).
                     */

                    ret = 0;
                  }
              }
            else
              {
                usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCLASSREQ),
                         ctrl->type);
              }
          }
          break;

        /* ACM_SET_CTRL_LINE_STATE: RS-232 signal used to tell the DCE
         * device the DTE device is now present. (Optional)
         */

        case ACM_SET_CTRL_LINE_STATE:
          {
            if (ctrl->type == (USB_DIR_OUT | USB_REQ_TYPE_CLASS |
                               USB_REQ_RECIPIENT_INTERFACE) &&
                index == priv->devinfo.ifnobase)
              {
                FAR const struct cdcacm_user_ops_s *ops =
                  cdcacm_borrow_ops(priv);

                /* Always update priv->ctrlline -- cdcacm_write reads it
                 * to gate output on the DTE_PRESENT bit, regardless of
                 * whether an adapter is installed.
                 */

                priv->ctrlline = value & 3;

                if (ops != NULL && ops->set_ctrl_line_state != NULL)
                  {
                    ret = ops->set_ctrl_line_state(priv, value);
                  }
                else
                  {
                    /* No adapter: ACK with a zero-length packet. */

                    ret = 0;
                  }
              }
            else
              {
                usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCLASSREQ),
                         ctrl->type);
              }
          }
          break;

        /*  Sends special carrier */

        case ACM_SEND_BREAK:
          {
            if (ctrl->type == (USB_DIR_OUT | USB_REQ_TYPE_CLASS |
                               USB_REQ_RECIPIENT_INTERFACE) &&
                index == priv->devinfo.ifnobase)
              {
                FAR const struct cdcacm_user_ops_s *ops =
                  cdcacm_borrow_ops(priv);

                if (ops != NULL && ops->send_break != NULL)
                  {
                    ret = ops->send_break(priv, value);
                  }
                else
                  {
                    /* No adapter: ACK the request with a zero-length
                     * packet -- chardev / outstream consumers do not
                     * have a hardware UART line to drive the break.
                     */

                    ret = 0;
                  }
              }
            else
              {
                usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCLASSREQ),
                         ctrl->type);
              }
          }
          break;

        default:
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCLASSREQ),
                   ctrl->req);
          break;
        }
    }
  else
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDTYPE), ctrl->type);
    }

  /* Respond to the setup command if data was returned.  On an error return
   * value (ret < 0), the USB driver will stall.
   */

  if (ret >= 0)
    {
      /* Configure the response */

      ctrlreq->len   = MIN(len, ret);
      ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;

      /* Send the response -- either directly to the USB controller or
       * indirectly in the case where this class is a member of a composite
       * device.
       */

#ifndef CONFIG_CDCACM_COMPOSITE
      ret = EP_SUBMIT(dev->ep0, ctrlreq);
#else
      ret = composite_ep0submit(driver, dev, ctrlreq, ctrl);
#endif
      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPRESPQ), (uint16_t)-ret);
          ctrlreq->result = OK;
          cdcacm_ep0incomplete(dev->ep0, ctrlreq);
        }
    }

  /* Returning a negative value will cause a STALL */

  return ret;
}

/****************************************************************************
 * Name: cdcacm_disconnect
 *
 * Description:
 *   Invoked after all transfers have been stopped, when the host is
 *   disconnected.  This function is probably called from the context of an
 *   interrupt handler.
 *
 ****************************************************************************/

static void cdcacm_disconnect(FAR struct usbdevclass_driver_s *driver,
                              FAR struct usbdev_s *dev)
{
  FAR struct cdcacm_dev_s *priv;
  FAR const struct cdcacm_user_ops_s *ops;

  usbtrace(TRACE_CLASSDISCONNECT, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev || !dev->ep0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract reference to private data */

  priv = ((FAR struct cdcacm_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return;
    }
#endif

  /* Inform the upper-half adapter that we have lost the USB serial
   * connection.  When no user_ops is installed (chardev / outstream
   * consumers) this is a no-op.
   */

  ops = cdcacm_borrow_ops(priv);
  if (ops != NULL && ops->on_connect != NULL)
    {
      ops->on_connect(priv, false);
    }

  /* Reset the configuration */

  cdcacm_resetconfig(priv);

  /* The cdcacm core holds no xmit ring of its own.  When a uart adapter is
   * installed the live xmit ring lives in cdcacm_serial_priv_s, and the
   * adapter's on_connect(false) callback is the proper place to reset its
   * own ring indices.
   */

  /* Perform the soft connect function so that we will we can be
   * re-enumerated (unless we are part of a composite device)
   */

#ifndef CONFIG_CDCACM_COMPOSITE
  DEV_CONNECT(dev);
#endif
}

/****************************************************************************
 * Name: cdcacm_suspend
 *
 * Description:
 *   Handle the USB suspend event.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_REMOVABLE
static void cdcacm_suspend(FAR struct usbdevclass_driver_s *driver,
                           FAR struct usbdev_s *dev)
{
  FAR struct cdcacm_dev_s *priv;
  FAR const struct cdcacm_user_ops_s *ops;

  usbtrace(TRACE_CLASSSUSPEND, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract reference to private data */

  priv = ((FAR struct cdcacm_driver_s *)driver)->dev;

  /* And let the upper-half adapter know that we are suspended.  When
   * no user_ops is installed (chardev / outstream consumers) this is
   * a no-op.
   */

  ops = cdcacm_borrow_ops(priv);
  if (ops != NULL && ops->on_suspend != NULL)
    {
      ops->on_suspend(priv, true);
    }
}
#endif

/****************************************************************************
 * Name: cdcacm_resume
 *
 * Description:
 *   Handle the USB resume event.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_REMOVABLE
static void cdcacm_resume(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev)
{
  FAR struct cdcacm_dev_s *priv;

  usbtrace(TRACE_CLASSRESUME, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract reference to private data */

  priv = ((FAR struct cdcacm_driver_s *)driver)->dev;

  /* Are we still configured? */

  if (priv->config != CDCACM_CONFIGIDNONE)
    {
      /* Yes.. let the upper-half adapter know we have resumed.  When
       * no user_ops is installed this is a no-op.
       */

      FAR const struct cdcacm_user_ops_s *ops = cdcacm_borrow_ops(priv);
      if (ops != NULL && ops->on_suspend != NULL)
        {
          ops->on_suspend(priv, false);
        }
    }
}
#endif

/****************************************************************************
 * Serial Device Methods (the uart_dev_s adapter lives in cdcacm_serial.c)
 ****************************************************************************/

#ifdef CONFIG_CDCACM_DISABLE_RXBUF

/****************************************************************************
 * Name: cdcacm_rcvpacket
 *
 * Description:
 *   Set up to receive bytes into the RX container.
 *
 ****************************************************************************/

static void cdcacm_rcvpacket(FAR struct cdcacm_dev_s *priv)
{
  FAR struct cdcacm_rdreq_s *rdcontainer;
  FAR const struct cdcacm_user_ops_s *ops;

  /* If a previous rdcontainer is still claimed by an adapter (it has
   * not finished consuming the bytes yet) then there is nothing to do
   * for now -- the adapter will release the container by calling back
   * into cdcacm_release_rxpending once it has drained.
   */

  if (priv->rdcontainer != NULL)
    {
      /* The entire packet has been processed and requeue the req.
       * If there is a pending req, cdcacm_rdcomplete may be called at
       * requeue time, which causes this function to be called again,
       * so priv->rdcontainer must be set to NULL before requeue.
       */

      rdcontainer = priv->rdcontainer;
      priv->rdcontainer = NULL;
      cdcacm_requeue_rdrequest(priv, rdcontainer);
    }

  if (priv->rdcontainer == NULL && !sq_empty(&priv->rxpending))
    {
      priv->rdcontainer = (FAR struct cdcacm_rdreq_s *)
                          sq_remfirst(&priv->rxpending);

      ops = cdcacm_borrow_ops(priv);
      if (ops != NULL && ops->on_rx != NULL)
        {
          ops->on_rx(priv,
                     (FAR const uint8_t *)priv->rdcontainer->req->buf,
                     priv->rdcontainer->req->xfrd);
        }

      /* When no adapter is installed (chardev / outstream consumers) the
       * rdcontainer stays parked here; cdcacm_chardev_read drains rxpending
       * directly, copying the bytes out and requeueing the request.
       */
    }
}

#endif

/****************************************************************************
 * Name: cdcacm_chardev_open
 *
 * Description:
 *   Open handler for /dev/cdcacmN.  Bumps the cdcacm refcount so the
 *   instance cannot be torn down while a fd is held.
 *
 *   When a uart adapter is installed (priv->user_ops != NULL) the chardev
 *   would race with cdcuart_recvbuf for rxpending, and with cdcacm_sndpacket
 *   for the wrcontainer pool.  To avoid that, a chardev open is rejected
 *   with -EBUSY whenever a user_ops adapter is in place.
 *
 ****************************************************************************/

static int cdcacm_chardev_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct cdcacm_dev_s *priv = inode->i_private;
  irqstate_t flags;
  bool want_rx;
  bool want_tx;

  if (priv == NULL)
    {
      return -ENXIO;
    }

  if (priv->closing)
    {
      return -ENXIO;
    }

  /* The data plane is exclusive per direction: at most one read-capable
   * (O_RDONLY / O_RDWR) opener may hold the RX slot and at most one
   * write-capable (O_WRONLY / O_RDWR) opener may hold the TX slot.  This
   * prevents two readers from interleaving a single RX packet (or two
   * writers a single xmit window) -- each direction has a single owner.
   * A reader and a writer still coexist (full duplex); only same-direction
   * sharing is rejected with -EBUSY.
   */

  want_rx = (filep->f_oflags & O_RDOK) != 0;
  want_tx = (filep->f_oflags & O_WROK) != 0;

  flags = spin_lock_irqsave(&priv->ops_lock);

  if (priv->user_ops != NULL)
    {
      spin_unlock_irqrestore(&priv->ops_lock, flags);
      return -EBUSY;
    }

  if ((want_rx && priv->rxreader_open) ||
      (want_tx && priv->txwriter_open))
    {
      spin_unlock_irqrestore(&priv->ops_lock, flags);
      return -EBUSY;
    }

  /* Take the lifecycle reference under the same lock that admits this open,
   * so admission and refcount move atomically.  If the instance is closing
   * the acquire fails and the open is rejected -- this keeps acquire/release
   * symmetric (a slot reserved here would otherwise pair with an unmatched
   * release at close, underflowing the refcount).  Done before any side
   * effect so a failed acquire needs no unwind.
   */

  if (!cdcacm_acquire(priv))
    {
      spin_unlock_irqrestore(&priv->ops_lock, flags);
      return -ENXIO;
    }

  if (want_rx)
    {
      priv->rxreader_open = true;
    }

  if (want_tx)
    {
      priv->txwriter_open = true;
    }

  priv->chardev_open_count++;
  spin_unlock_irqrestore(&priv->ops_lock, flags);

  /* In chardev-only mode there is no uart adapter to call rxint and enable
   * RX notification, so priv->rxenabled would stay false and
   * cdcacm_release_rxpending would never post rx_waitsem -- a chardev reader
   * would block forever even though bulk-OUT data is landing in rxpending.
   * Enable RX for the reader and drain whatever arrived before this open.
   * A write-only opener leaves RX disabled (no one would drain rxpending).
   */

  if (want_rx)
    {
      flags = spin_lock_irqsave(&priv->lock);
      priv->rxenabled = true;
      spin_unlock_irqrestore(&priv->lock, flags);

      cdcacm_release_rxpending(priv);
    }

  return OK;
}

/****************************************************************************
 * Name: cdcacm_chardev_close
 *
 * Description:
 *   Close handler for /dev/cdcacmN.  Drops the refcount taken by open and
 *   decrements the chardev_open_count gate.
 *
 ****************************************************************************/

static int cdcacm_chardev_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct cdcacm_dev_s *priv = inode->i_private;
  irqstate_t flags;
  bool released_rx;

  if (priv == NULL)
    {
      return OK;
    }

  flags = spin_lock_irqsave(&priv->ops_lock);
  if (priv->chardev_open_count > 0)
    {
      priv->chardev_open_count--;
    }

  /* Release the per-direction slots this fd held (mirrors open). */

  released_rx = (filep->f_oflags & O_RDOK) != 0 && priv->rxreader_open;
  if (released_rx)
    {
      priv->rxreader_open = false;
    }

  if ((filep->f_oflags & O_WROK) != 0)
    {
      priv->txwriter_open = false;
    }

  spin_unlock_irqrestore(&priv->ops_lock, flags);

  /* When the reader closes, disable RX again so incoming packets simply
   * pend (matching the uart adapter's rxint(false) behavior) instead of
   * posting a semaphore nobody waits on.
   */

  if (released_rx)
    {
      flags = spin_lock_irqsave(&priv->lock);
      priv->rxenabled = false;
      spin_unlock_irqrestore(&priv->lock, flags);
    }

  cdcacm_release(priv);
  return OK;
}

/****************************************************************************
 * Name: cdcacm_chardev_read
 *
 * Description:
 *   Drain priv->rxpending into the user buffer.  One memcpy per call,
 *   covering as many packets as fit (we requeue the rdcontainer once we
 *   have copied its full payload, which lets the host send more).
 *
 *   Blocks on rx_waitsem when the queue is empty unless O_NONBLOCK is set.
 *
 ****************************************************************************/

static ssize_t cdcacm_chardev_read(FAR struct file *filep, FAR char *buf,
                                   size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct cdcacm_dev_s *priv = inode->i_private;
  FAR struct cdcacm_rdreq_s *rdcontainer;
  FAR struct usbdev_req_s *req;
  irqstate_t flags;
  size_t copied = 0;
  size_t offset;
  size_t avail;
  size_t want;
  size_t nb;
  bool drained;
  int ret;

  if (priv == NULL)
    {
      return -ENXIO;
    }

  if (buf == NULL || len == 0)
    {
      return 0;
    }

  while (copied < len)
    {
      flags = spin_lock_irqsave(&priv->lock);

      rdcontainer = (FAR struct cdcacm_rdreq_s *)sq_peek(&priv->rxpending);
      if (rdcontainer == NULL)
        {
          spin_unlock_irqrestore(&priv->lock, flags);

          if (priv->closing)
            {
              return copied > 0 ? (ssize_t)copied : -ENXIO;
            }

          if (copied > 0)
            {
              break;
            }

          if ((filep->f_oflags & O_NONBLOCK) != 0)
            {
              return -EAGAIN;
            }

          ret = nxsem_wait(&priv->rx_waitsem);
          if (ret < 0)
            {
              return ret;
            }

          continue;
        }

      req = rdcontainer->req;

      /* Snapshot the unread span of the head packet, then release the lock
       * BEFORE the memcpy.  The single-reader exclusion enforced at open
       * time guarantees this reader is the sole consumer of the head
       * packet; the RX IRQ (cdcacm_rdcomplete) only ever appends to the
       * tail of rxpending and never touches the head's offset, so the
       * snapshot stays valid across the unlocked copy.  Copying under the
       * spinlock would keep interrupts disabled for the whole transfer and
       * hurt latency on large reads.
       */

      offset  = rdcontainer->offset;
      avail   = req->xfrd > offset ? (size_t)(req->xfrd - offset) : 0;
      want    = len - copied;
      nb      = MIN(avail, want);
      drained = (offset + nb) >= req->xfrd;

      if (drained)
        {
          /* This read consumes the rest of the head packet: dequeue it
           * under the lock so a concurrent requeue path cannot observe a
           * half-consumed head, then copy + requeue unlocked.
           */

          sq_remfirst(&priv->rxpending);
          spin_unlock_irqrestore(&priv->lock, flags);

          if (nb > 0)
            {
              memcpy(buf + copied, &req->buf[offset], nb);
              copied += nb;
            }

          cdcacm_requeue_rdrequest(priv, rdcontainer);
          continue;
        }

      /* Partial consume: advance offset under the lock (so the next loop
       * iteration / read sees the new head position), then copy unlocked.
       */

      rdcontainer->offset = offset + nb;
      spin_unlock_irqrestore(&priv->lock, flags);

      if (nb > 0)
        {
          memcpy(buf + copied, &req->buf[offset], nb);
          copied += nb;
        }

      /* Caller's buffer is now full (partial consume implies want < avail);
       * the head packet still has data left for the next read.
       */

      break;
    }

  return (ssize_t)copied;
}

/****************************************************************************
 * Name: cdcacm_chardev_write
 *
 * Description:
 *   Submit one or more bulk-IN requests carrying buf.  Always-copy path
 *   (the spec leaves zero-copy as a future optimisation): pop a wrcontainer,
 *   memcpy up to MIN(len, reqlen) bytes, EP_SUBMIT, repeat until len has
 *   been consumed or until txfree is empty and we're non-blocking.
 *
 ****************************************************************************/

static ssize_t cdcacm_chardev_write(FAR struct file *filep,
                                    FAR const char *buf, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct cdcacm_dev_s *priv = inode->i_private;
  bool nonblock;

  if (priv == NULL)
    {
      return -ENXIO;
    }

  if (buf == NULL || len == 0)
    {
      return 0;
    }

  /* IRQ context cannot sleep on tx_waitsem; treat O_NONBLOCK identically. */

  nonblock = (filep->f_oflags & O_NONBLOCK) != 0 ||
             up_interrupt_context();

  return cdcacm_internal_submit(priv, buf, len, nonblock);
}

/****************************************************************************
 * Name: cdcacm_internal_submit
 *
 * Description:
 *   Shared TX submit helper -- always-copy path used by both
 *   cdcacm_chardev_write() and cdcacm_outstream_puts().  Pop a wrcontainer
 *   off the txfree pool, memcpy up to MIN(len, pktcap) bytes into the
 *   pre-allocated request buffer, then EP_SUBMIT.  Repeat until len has
 *   been consumed or txfree is exhausted in a non-blocking caller.
 *
 *   Honors the lib_outstream_s contract that the caller buffer is reusable
 *   on return (we copy before the call returns; we never retain a pointer
 *   to caller memory across submit).  Zero-copy submit is left as a future
 *   optimization -- same rationale as the chardev write path: per-call sem
 *   bookkeeping + custom callback dispatch interacts non-trivially with
 *   cdcacm_wrcomplete's existing tx_waitsem post / nwrq accounting and is
 *   not justified by current consumers where the bulk-IN reqlen already
 *   exceeds typical packet sizes.
 *
 * Input Parameters:
 *   priv     - cdcacm device handle
 *   buf      - source buffer (must remain valid for the duration of the
 *              call only)
 *   len      - number of bytes to submit
 *   nonblock - true: do not sleep on tx_waitsem; return -EAGAIN / partial
 *              write when txfree is empty.  Must be true if invoked from
 *              IRQ context.
 *
 * Returned Value:
 *   Number of bytes accepted (0 < ret <= len) on success, or a negated
 *   errno on failure when no bytes were submitted.
 *
 ****************************************************************************/

static ssize_t cdcacm_internal_submit(FAR struct cdcacm_dev_s *priv,
                                      FAR const void *buf, size_t len,
                                      bool nonblock)
{
  FAR const uint8_t *src = (FAR const uint8_t *)buf;
  FAR struct usbdev_ep_s *ep;
  FAR struct cdcacm_wrreq_s *wrcontainer;
  FAR struct usbdev_req_s *req;
  irqstate_t flags;
  size_t pktcap;
  size_t sent = 0;
  size_t want;
  int ret;

  if (priv->closing)
    {
      return -ENXIO;
    }

  if (priv->config != CDCACM_CONFIGID)
    {
      return -ENOTCONN;
    }

  ep = priv->epbulkin;
  if (ep == NULL)
    {
      return -ENOTCONN;
    }

  /* Use the full request buffer; USB stack chunks into maxpacket
   * packets internally.  Capping at maxpacket here would force one
   * EP_SUBMIT per packet and starve sustained throughput.
   */

  pktcap = (size_t)CONFIG_CDCACM_BULKIN_REQLEN;
  if (pktcap == 0)
    {
      return -EINVAL;
    }

  while (sent < len)
    {
      /* Acquire a free wrcontainer.  Non-blocking callers (IRQ ctx,
       * O_NONBLOCK) bail out with -EAGAIN once txfree is empty.
       */

      flags = spin_lock_irqsave(&priv->lock);
      while (sq_empty(&priv->txfree))
        {
          spin_unlock_irqrestore(&priv->lock, flags);

          if (priv->closing)
            {
              return sent > 0 ? (ssize_t)sent : -ENXIO;
            }

          if (nonblock)
            {
              return sent > 0 ? (ssize_t)sent : -EAGAIN;
            }

          ret = nxsem_wait(&priv->tx_waitsem);
          if (ret < 0)
            {
              return sent > 0 ? (ssize_t)sent : ret;
            }

          flags = spin_lock_irqsave(&priv->lock);
        }

      wrcontainer = (FAR struct cdcacm_wrreq_s *)sq_remfirst(&priv->txfree);
      priv->nwrq--;
      spin_unlock_irqrestore(&priv->lock, flags);

      req = wrcontainer->req;

      want = MIN(pktcap, len - sent);
      memcpy(req->buf, src + sent, want);
      req->len   = want;
      req->priv  = wrcontainer;
      req->flags = USBDEV_REQFLAGS_NULLPKT;

      ret = EP_SUBMIT(ep, req);
      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SUBMITFAIL),
                   (uint16_t)-ret);

          /* Submit failed -- return the wrcontainer to the free list */

          flags = spin_lock_irqsave(&priv->lock);
          sq_addfirst((FAR sq_entry_t *)wrcontainer, &priv->txfree);
          priv->nwrq++;
          spin_unlock_irqrestore(&priv->lock, flags);

          return sent > 0 ? (ssize_t)sent : ret;
        }

      sent += want;
    }

  return (ssize_t)sent;
}

/****************************************************************************
 * Name: cdcacm_chardev_poll
 *
 * Description:
 *   Trivial level-triggered poll: POLLIN if rxpending has data, POLLOUT if
 *   txfree has at least one wrcontainer, POLLHUP when the device is gone.
 *
 *   This path does not register pollfd slots for later notify -- the
 *   chardev_read / chardev_write semaphore wake-ups already cover the
 *   hand-off, and most callers do blocking I/O rather than poll().
 *   poll(setup) just samples the queues and reports the level.
 *   poll(teardown) is a no-op.
 *
 ****************************************************************************/

static int cdcacm_chardev_poll(FAR struct file *filep,
                               FAR struct pollfd *fds, bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct cdcacm_dev_s *priv = inode->i_private;
  pollevent_t eventset;
  irqstate_t flags;

  if (priv == NULL || fds == NULL)
    {
      return -EINVAL;
    }

  if (!setup)
    {
      return OK;
    }

  eventset = 0;

  flags = spin_lock_irqsave(&priv->lock);

  if (!sq_empty(&priv->rxpending))
    {
      eventset |= POLLIN | POLLRDNORM;
    }

  if (!sq_empty(&priv->txfree))
    {
      eventset |= POLLOUT | POLLWRNORM;
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  if (priv->closing || priv->config != CDCACM_CONFIGID)
    {
      eventset |= POLLHUP;
    }

  poll_notify(&fds, 1, eventset);
  return OK;
}

/****************************************************************************
 * Name: cdcacm_set_user_ops
 *
 * Description:
 *   Install the user ops table on a freshly allocated instance.  Called once
 *   by cdcacm_register (before usbdev_register) when an adapter registers
 *   with a non-NULL ops table; takes a refcount that cdcacm_clear_user_ops
 *   releases.
 *
 * Input Parameters:
 *   dev       - Device handle returned by cdcacm_register().
 *   ops       - New user callback table.  Any field may be NULL.
 *   user_priv - Opaque pointer stored in the instance.
 *
 * Returned Value:
 *   Zero (OK) on success, a negated errno value on failure.
 *
 ****************************************************************************/

static int cdcacm_set_user_ops(FAR struct cdcacm_dev_s *dev,
                               FAR const struct cdcacm_user_ops_s *ops,
                               FAR void *user_priv)
{
  irqstate_t flags;

  if (dev == NULL || dev->closing)
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&dev->ops_lock);
  if (dev->user_ops != NULL)
    {
      spin_unlock_irqrestore(&dev->ops_lock, flags);
      return -EBUSY;  /* one user_ops at a time */
    }

  if (dev->chardev_open_count > 0)
    {
      spin_unlock_irqrestore(&dev->ops_lock, flags);
      return -EBUSY;  /* chardev consumer already attached */
    }

  dev->user_ops  = ops;
  dev->user_priv = user_priv;
  atomic_add(&dev->refcount, 1);  /* ops holder */
  spin_unlock_irqrestore(&dev->ops_lock, flags);
  return 0;
}

/****************************************************************************
 * Name: cdcacm_clear_user_ops
 *
 * Description:
 *   Detach the current user ops table.  After this call all dispatched
 *   callbacks become no-ops until a new ops table is installed.
 *
 * Input Parameters:
 *   dev - Device handle returned by cdcacm_register().
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void cdcacm_clear_user_ops(FAR struct cdcacm_dev_s *dev)
{
  irqstate_t flags;

  if (dev == NULL)
    {
      return;
    }

  flags = spin_lock_irqsave(&dev->ops_lock);
  if (dev->user_ops != NULL)
    {
      dev->user_ops  = NULL;
      dev->user_priv = NULL;
      spin_unlock_irqrestore(&dev->ops_lock, flags);
      cdcacm_release(dev);  /* match the acquire in set_user_ops */
      return;
    }

  spin_unlock_irqrestore(&dev->ops_lock, flags);
}

/****************************************************************************
 * Name: cdcacm_outstream_putc
 *
 * Description:
 *   lib_outstream_s.putc method -- emit a single byte through the cdcacm
 *   bulk-IN endpoint.  Discards EOF (0xff sentinel from <stdio.h>) like
 *   the other lib_*outstream implementations.
 *
 ****************************************************************************/

static void cdcacm_outstream_putc(FAR struct lib_outstream_s *self, int ch)
{
  uint8_t b;

  if (ch == EOF)
    {
      return;
    }

  b = (uint8_t)ch;
  cdcacm_outstream_puts(self, &b, 1);
}

/****************************************************************************
 * Name: cdcacm_outstream_puts
 *
 * Description:
 *   lib_outstream_s.puts method -- enqueue len bytes from buf for transmit.
 *   The lib_outstream_s contract requires buf be reusable on return; this
 *   is satisfied because cdcacm_internal_submit() copies into the
 *   pre-allocated wrcontainer buffer before EP_SUBMIT (see helper comment
 *   for the always-copy rationale).
 *
 *   Updates self->nput on partial / full success so callers using
 *   lib_outstream macros see the running counter.
 *
 *   May be invoked from IRQ context (e.g. note trace flush from a syslog
 *   path).  cdcacm_internal_submit handles that by short-circuiting the
 *   tx_waitsem wait.
 *
 ****************************************************************************/

static ssize_t cdcacm_outstream_puts(FAR struct lib_outstream_s *self,
                                     FAR const void *buf, size_t len)
{
  FAR struct cdcacm_outstream_s *cos =
    (FAR struct cdcacm_outstream_s *)self;
  ssize_t ret;

  if (cos == NULL || cos->dev == NULL || cos->dev->closing)
    {
      return -EBADF;
    }

  if (buf == NULL || len == 0)
    {
      return 0;
    }

  /* IRQ context (and by extension any caller that cannot sleep) is
   * indistinguishable from O_NONBLOCK at the helper level: bail out with
   * -EAGAIN once the wrcontainer pool is empty rather than block on
   * tx_waitsem.
   */

  ret = cdcacm_internal_submit(cos->dev, buf, len,
                               up_interrupt_context());
  if (ret > 0)
    {
      self->nput += (off_t)ret;
    }

  return ret;
}

/****************************************************************************
 * Name: cdcacm_outstream_flush
 *
 * Description:
 *   lib_outstream_s.flush method.  cdcacm_internal_submit performs
 *   EP_SUBMIT on every puts() call; there is no buffering layer to drain.
 *   This is a no-op (return OK) -- consistent with other unbuffered
 *   outstreams (lib_lowoutstream, lib_memoutstream).
 *
 ****************************************************************************/

static int cdcacm_outstream_flush(FAR struct lib_outstream_s *self)
{
  UNUSED(self);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_CDCACM
/****************************************************************************
 * Name: cdcacm_write
 *
 * Description:
 *   This provides a cdcacm write method for syslog devices that support
 *   multiple byte writes
 *
 * Input Parameters:
 *   buffer - The buffer containing the data to be output
 *   buflen - The number of bytes in the buffer
 *
 * Returned Value:
 *   On success, the number of characters written is returned.  A negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

ssize_t cdcacm_write(FAR const char *buffer, size_t buflen)
{
  FAR struct cdcacm_dev_s *priv = g_syslog_cdcacm;
  size_t len = 0;

  while (len < buflen)
    {
      FAR struct usbdev_ep_s *ep;
      FAR struct cdcacm_wrreq_s *wrcontainer;
      FAR struct usbdev_req_s *req;
      irqstate_t flags;
      size_t reqlen;
      size_t nbytes;
      int ret;

      if (!priv || !(priv->ctrlline & CDC_DTE_PRESENT))
        {
          return -EINVAL;
        }

      ep = priv->epbulkin;

      /* Equivalent to the legacy cdcuart_txready: poll the EP, then test
       * if a wrcontainer is available.
       */

      if (sq_empty(&priv->txfree))
        {
          priv->ispolling = true;
          EP_POLL(ep);
          priv->ispolling = false;
        }

      if (sq_empty(&priv->txfree))
        {
          continue;
        }

      /* Equivalent to the legacy cdcuart_sendbuf: pop a wrcontainer, copy
       * payload, submit the request.
       */

      reqlen = MIN(CONFIG_CDCACM_BULKIN_REQLEN, ep->maxpacket);

      flags = spin_lock_irqsave(&priv->lock);
      wrcontainer = (FAR struct cdcacm_wrreq_s *)sq_remfirst(&priv->txfree);
      req = wrcontainer->req;
      priv->nwrq--;
      spin_unlock_irqrestore(&priv->lock, flags);

      nbytes = MIN(reqlen, buflen - len);
      memcpy(req->buf, buffer + len, nbytes);

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

      len += nbytes;
    }

  return buflen;
}

/****************************************************************************
 * Name: cdcacm_disable_syslog
 *
 * Description:
 *   Disable CDCACM syslog channel by clearing the globle pointer.
 *   This function is used in specific situation, such as must disable
 *   cdcacm log printing when usb re-enumeration.
 *
 ****************************************************************************/

void cdcacm_disable_syslog(void)
{
  g_syslog_cdcacm = NULL;
}
#endif

/****************************************************************************
 * Name: cdcacm_alloc_dev
 *
 * Description:
 *   Allocate and initialise a cdcacm_alloc_s blob (cdcacm_dev_s + the
 *   driving usbdevclass_driver_s wrapper).  Returns the contained
 *   cdcacm_dev_s on success, or NULL on allocation failure.
 *
 *   Shared by cdcacm_register (standalone API) and cdcacm_classobject
 *   (composite-USB legacy entry point).
 *
 ****************************************************************************/

static FAR struct cdcacm_dev_s *
cdcacm_alloc_dev(int minor, FAR struct usbdev_devinfo_s *devinfo)
{
  FAR struct cdcacm_alloc_s *alloc;
  FAR struct cdcacm_dev_s *priv;
  FAR struct cdcacm_driver_s *drvr;

  alloc = (FAR struct cdcacm_alloc_s *)
    kmm_malloc(sizeof(struct cdcacm_alloc_s));
  if (alloc == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCDEVSTRUCT), 0);
      return NULL;
    }

  priv = &alloc->dev;
  drvr = &alloc->drvr;

  memset(priv, 0, sizeof(struct cdcacm_dev_s));
  sq_init(&priv->txfree);
  sq_init(&priv->rxpending);
  spin_lock_init(&priv->lock);

  priv->minor = minor;

  /* Refcount-based lifecycle: the registering caller holds the initial
   * reference; cdcacm_unregister drops it.
   */

  atomic_set(&priv->refcount, 1);
  priv->closing   = false;
  nxsem_init(&priv->close_done, 0, 0);
  spin_lock_init(&priv->ops_lock);
  priv->user_ops  = NULL;
  priv->user_priv = NULL;

  /* Chardev-path waitsems.  rx_waitsem is posted by the rx data-arrival
   * site when no user_ops is installed (chardev mode); tx_waitsem is
   * posted when wrcomplete returns a wrcontainer to txfree.  Both are
   * counting semaphores so multiple waiters can be woken without races.
   */

  priv->chardev_open_count = 0;
  nxsem_init(&priv->rx_waitsem, 0, 0);
  nxsem_init(&priv->tx_waitsem, 0, 0);

  if (devinfo != NULL)
    {
      memcpy(&priv->devinfo, devinfo, sizeof(struct usbdev_devinfo_s));
    }

#ifdef CONFIG_CDCACM_IFLOWCONTROL
  priv->serialstate = (CDCACM_UART_DCD | CDCACM_UART_DSR);
#endif

  /* Initialize the USB class driver structure */

#if defined(CONFIG_USBDEV_SUPERSPEED)
  drvr->drvr.speed = USB_SPEED_SUPER;
#elif defined(CONFIG_USBDEV_DUALSPEED)
  drvr->drvr.speed = USB_SPEED_HIGH;
#else
  drvr->drvr.speed = USB_SPEED_FULL;
#endif
  drvr->drvr.ops = &g_driverops;
  drvr->dev      = priv;

#ifdef CONFIG_SYSLOG_CDCACM
  if (minor == CONFIG_SYSLOG_CDCACM_MINOR)
    {
      g_syslog_cdcacm = priv;
    }
#endif

  return priv;
}

/****************************************************************************
 * Name: cdcacm_classobject
 *
 * Description:
 *   Composite-USB entry point.  Allocates a cdcacm instance and returns
 *   its usbdevclass_driver_s pointer; composite owns usbdev_register and
 *   any user_ops binding.  Preserved as a thin wrapper so the existing
 *   board *_composite.c call sites remain unchanged.
 *
 ****************************************************************************/

int cdcacm_classobject(int minor, FAR struct usbdev_devinfo_s *devinfo,
                       FAR struct usbdevclass_driver_s **classdev)
{
  FAR struct cdcacm_dev_s *priv;

  priv = cdcacm_alloc_dev(minor, devinfo);
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  *classdev = &((FAR struct cdcacm_alloc_s *)priv)->drvr.drvr;
  return OK;
}

/* cdcacm.c carries the USB protocol core only.  The uart_dev_s adapter,
 * including cdcacm_initialize and cdcacm_uninitialize, lives in
 * cdcacm_serial.c and is built only when CONFIG_CDCACM_SERIAL is enabled.
 */

/****************************************************************************
 * Name: cdcacm_get_composite_devdesc
 *
 * Description:
 *   Helper function to fill in some constants into the composite
 *   configuration struct.
 *
 * Input Parameters:
 *     dev - Pointer to the configuration struct we should fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_CDCACM_COMPOSITE)
void cdcacm_get_composite_devdesc(struct composite_devdesc_s *dev)
{
  memset(dev, 0, sizeof(struct composite_devdesc_s));

  /* The callback functions for the CDC/ACM class.
   *
   * classobject() and uninitialize() must be provided by board-specific
   * logic
   */

  dev->mkconfdesc   = cdcacm_mkcfgdesc;
  dev->mkstrdesc    = cdcacm_mkstrdesc;

  dev->nconfigs     = CDCACM_NCONFIGS;           /* Number of configurations supported */
  dev->configid     = CDCACM_CONFIGID;           /* The only supported configuration ID */

  /* Let the construction function calculate the size of config descriptor */

  dev->cfgdescsize  = cdcacm_mkcfgdesc(NULL, NULL, USB_SPEED_UNKNOWN, 0);

  /* Board-specific logic must provide the device minor */

  /* Interfaces.
   *
   * ifnobase must be provided by board-specific logic
   */

  dev->devinfo.ninterfaces = CDCACM_NINTERFACES; /* Number of interfaces in the configuration */

  /* Strings.
   *
   * strbase must be provided by board-specific logic
   */

  dev->devinfo.nstrings    = CDCACM_NSTRIDS;     /* Number of Strings */

  /* Endpoints.
   *
   * Endpoint numbers must be provided by board-specific logic.
   */

  dev->devinfo.nendpoints  = CDCACM_NUM_EPS;
}
#endif

/****************************************************************************
 * Name: cdcacm_register
 *
 * Description:
 *   Allocate a cdcacm instance, install the supplied user ops table, and
 *   register the underlying USB device class driver.  Returns the opaque
 *   device handle through dev_out for use with the rest of the layered
 *   API.
 *
 * Input Parameters:
 *   minor     - Device minor number; same meaning as cdcacm_initialize().
 *   ops       - User callback table.  Any field may be NULL.
 *   user_priv - Opaque pointer stored in the instance and returned from
 *               cdcacm_get_user_priv().
 *   devinfo   - Composite endpoint/interface descriptor info, or NULL
 *               when running standalone.
 *   dev_out   - Location to receive the new device handle on success.
 *
 * Returned Value:
 *   Zero (OK) on success, a negated errno value on failure.
 *
 ****************************************************************************/

int cdcacm_register(int minor,
                    FAR const struct cdcacm_user_ops_s *ops,
                    FAR void *user_priv,
                    FAR struct usbdev_devinfo_s *devinfo,
                    FAR struct cdcacm_dev_s **dev_out)
{
  FAR struct cdcacm_dev_s *priv;
  FAR struct cdcacm_alloc_s *alloc;
  char devname[16];
  int ret;

  if (dev_out == NULL)
    {
      return -EINVAL;
    }

  priv = cdcacm_alloc_dev(minor, devinfo);
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  alloc = (FAR struct cdcacm_alloc_s *)priv;

  /* Install the user_ops table BEFORE usbdev_register so any control
   * requests that arrive immediately after registration find the ops in
   * place.  cdcacm_set_user_ops takes its own refcount.
   */

  if (ops != NULL)
    {
      ret = cdcacm_set_user_ops(priv, ops, user_priv);
      if (ret < 0)
        {
          goto errout_free;
        }
    }

  /* Hand the driver to the USB stack.  bind() will allocate ctrlreq,
   * EPs and read/write request pools.  Composite-mode callers go via
   * cdcacm_classobject + the composite framework's own usbdev_register;
   * skip our own registration in that case to avoid double-bind.
   */

#ifndef CONFIG_CDCACM_COMPOSITE
  ret = usbdev_register(&alloc->drvr.drvr);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_DEVREGISTER),
               (uint16_t)-ret);
      goto errout_clear_ops;
    }
#endif

  /* Register the /dev/cdcacmN chardev so non-uart consumers (note trace,
   * bugreport, generic open/read/write callers) can talk to this instance.
   * The uart adapter, when installed, gets its own /dev/ttyACMn registration
   * inside cdcacm_serial.c.
   */

  snprintf(devname, sizeof(devname), "/dev/cdcacm%d", minor);
  ret = register_driver(devname, &g_cdcacm_chardev_fops, 0666, priv);
  if (ret < 0)
    {
      goto errout_unregister_usb;
    }

  *dev_out = priv;
  return OK;

#ifndef CONFIG_CDCACM_COMPOSITE
errout_unregister_usb:
  usbdev_unregister(&alloc->drvr.drvr);
errout_clear_ops:
  if (ops != NULL)
    {
      cdcacm_clear_user_ops(priv);
    }
#else
errout_unregister_usb:
  if (ops != NULL)
    {
      cdcacm_clear_user_ops(priv);
    }
#endif

errout_free:
#ifdef CONFIG_SYSLOG_CDCACM
  if (g_syslog_cdcacm == priv)
    {
      g_syslog_cdcacm = NULL;
    }
#endif

  nxsem_destroy(&priv->close_done);
  nxsem_destroy(&priv->rx_waitsem);
  nxsem_destroy(&priv->tx_waitsem);
  kmm_free(alloc);
  return ret;
}

/****************************************************************************
 * Name: cdcacm_unregister
 *
 * Description:
 *   Tear down a cdcacm instance previously created by cdcacm_register().
 *   Once all in-flight users have released the device, the underlying USB
 *   device class driver is unregistered and the instance is freed.
 *
 * Input Parameters:
 *   dev - Device handle returned by cdcacm_register().
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void cdcacm_unregister(FAR struct cdcacm_dev_s *dev)
{
  FAR struct cdcacm_alloc_s *alloc;
  char devname[16];
  int sval;

  if (dev == NULL)
    {
      return;
    }

  alloc = (FAR struct cdcacm_alloc_s *)dev;

  /* Mark the instance closing so cdcacm_acquire fails for newcomers.
   * Existing holders complete on their own and call cdcacm_release;
   * once refcount drops to 1 (the registering caller's), the matching
   * release below will post close_done.
   */

  dev->closing = true;

  /* Wake every reader/writer blocked on the waitsems so they observe
   * priv->closing and unwind via close (which drops refcount).  Posting
   * only one credit each would leave additional waiters parked, holding
   * refcounts, and cdcacm_release/close_done below would hang.
   *
   * Sizing this against chardev_open_count alone is wrong: outstream
   * consumers also block on tx_waitsem via cdcacm_internal_submit but
   * never bump that counter (they hold their own ref through
   * cdcacm_outstream_open's cdcacm_acquire).  Instead, post each sem
   * until its value goes positive -- semaphore value reflects the
   * outstanding waiter count when negative, so this drains exactly the
   * waiters present and leaves at most one phantom credit (consumed by
   * the next caller's pre-wait re-check).
   *
   * Race-free because closing=true was published above: cdcacm_chardev_open
   * and cdcacm_outstream_open both reject new arrivals once closing is
   * set, and the chardev/outstream submit/read paths re-check closing
   * after wake before returning to wait.  Waiter count only decreases
   * during this loop.
   */

  do
    {
      nxsem_post(&dev->rx_waitsem);
    }
  while (nxsem_get_value(&dev->rx_waitsem, &sval) == OK && sval <= 0);

  do
    {
      nxsem_post(&dev->tx_waitsem);
    }
  while (nxsem_get_value(&dev->tx_waitsem, &sval) == OK && sval <= 0);

  /* Pull /dev/cdcacmN out of the namespace so new opens fail immediately.
   * Existing fds keep priv alive via the refcount taken in chardev_open;
   * close drops them.
   */

  snprintf(devname, sizeof(devname), "/dev/cdcacm%d", dev->minor);
  unregister_driver(devname);

  /* Detach any installed user_ops table.  This drops the user_ops ref
   * that set_user_ops took, so the only remaining holder is the
   * registering caller.
   */

  cdcacm_clear_user_ops(dev);

  /* usbdev_unregister synchronously calls our cdcacm_unbind which frees
   * EPs, ctrlreq and the rd/wr request pools.  Composite-mode callers
   * have already had this done by the composite framework before they
   * reach cdcacm_uninitialize -> cdcacm_unregister; calling it twice
   * would be a use-after-free, so skip it.
   */

#ifndef CONFIG_CDCACM_COMPOSITE
  usbdev_unregister(&alloc->drvr.drvr);
#endif

#ifdef CONFIG_SYSLOG_CDCACM
  if (g_syslog_cdcacm == dev)
    {
      g_syslog_cdcacm = NULL;
    }
#endif

  /* Drop the registering caller's reference; if no other holders remain
   * the matching cdcacm_release posts close_done and returns
   * immediately.  Otherwise we wait for them to drain.
   */

  cdcacm_release(dev);
  while (nxsem_wait(&dev->close_done) == -EINTR);

  nxsem_destroy(&dev->close_done);
  nxsem_destroy(&dev->rx_waitsem);
  nxsem_destroy(&dev->tx_waitsem);
  kmm_free(alloc);
}

/****************************************************************************
 * Name: cdcacm_acquire
 *
 * Description:
 *   Increment the cdcacm instance refcount, preventing teardown while the
 *   caller holds a reference.
 *
 * Input Parameters:
 *   dev - Device handle returned by cdcacm_register().
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

bool cdcacm_acquire(FAR struct cdcacm_dev_s *dev)
{
  /* Take a reference only while the instance is open for business.  Once
   * unregister has published closing=true the instance is draining toward
   * teardown and must not be revived; report failure so the caller can
   * abort its open instead of taking an unmatched release later (which
   * would underflow the refcount and free the instance out from under a
   * legitimate holder).
   */

  if (dev != NULL && !dev->closing)
    {
      atomic_add(&dev->refcount, 1);
      return true;
    }

  return false;
}

/****************************************************************************
 * Name: cdcacm_release
 *
 * Description:
 *   Drop a reference previously taken by cdcacm_acquire().  When the
 *   refcount reaches zero and the instance is in the closing state, the
 *   instance is freed.
 *
 * Input Parameters:
 *   dev - Device handle returned by cdcacm_register().
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void cdcacm_release(FAR struct cdcacm_dev_s *dev)
{
  if (dev == NULL)
    {
      return;
    }

  if (atomic_sub(&dev->refcount, 1) == 1)
    {
      nxsem_post(&dev->close_done);
    }
}

/****************************************************************************
 * Name: cdcacm_get_user_priv
 *
 * Description:
 *   Retrieve the opaque user_priv pointer that the adapter passed to
 *   cdcacm_register().
 *
 * Input Parameters:
 *   dev - Device handle returned by cdcacm_register().
 *
 * Returned Value:
 *   The stored user_priv pointer, or NULL if none was set.
 *
 ****************************************************************************/

FAR void *cdcacm_get_user_priv(FAR struct cdcacm_dev_s *dev)
{
  return dev != NULL ? dev->user_priv : NULL;
}

/****************************************************************************
 * Name: cdcacm_send_serial_state
 *
 * Description:
 *   Intended to send a SERIAL_STATE notification on the interrupt IN
 *   endpoint when the modem-status lines (DCD/DSR/RI/CTS) change.
 *
 *   Currently a stub that returns -ENOSYS.  The live notification path is
 *   cdcacm_serialstate(), reached through the uart adapter's CAIOC_NOTIFY
 *   ioctl when CONFIG_CDCACM_IFLOWCONTROL is enabled.
 *
 * Input Parameters:
 *   dev   - Device handle returned by cdcacm_register().
 *   state - Bitmap of UART state bits, see "Table 69: UART State Bitmap
 *           Values" in the CDC PSTN spec and CDC_UART_definitions in
 *           include/nuttx/usb/cdc.h.
 *
 * Returned Value:
 *   -ENOSYS (not implemented).
 *
 ****************************************************************************/

int cdcacm_send_serial_state(FAR struct cdcacm_dev_s *dev, uint16_t state)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: cdcacm_outstream_open
 *
 * Description:
 *   Initialize a cdcacm_outstream_s so it can be used with the standard
 *   lib_outstream_s puts/putc/flush operations.  Naming aligns with
 *   lib_blkoutstream_open() / lib_mtdoutstream_open().
 *
 * Input Parameters:
 *   stream - Caller-allocated stream object to initialize.
 *   dev    - Device handle returned by cdcacm_register().
 *
 * Returned Value:
 *   Zero (OK) on success, a negated errno value on failure.
 *
 ****************************************************************************/

int cdcacm_outstream_open(FAR struct cdcacm_outstream_s *stream,
                          FAR struct cdcacm_dev_s *dev)
{
  if (stream == NULL || dev == NULL)
    {
      return -EINVAL;
    }

  /* Hold a refcount for the lifetime of the stream so the cdcacm instance
   * cannot be torn down while a holder of stream still calls puts().  Take
   * it first: if the instance is closing the acquire fails and we must not
   * bind the stream (a later close would release a reference we never took).
   */

  if (!cdcacm_acquire(dev))
    {
      return -ENXIO;
    }

  stream->common.nput  = 0;
  stream->common.putc  = cdcacm_outstream_putc;
  stream->common.puts  = cdcacm_outstream_puts;
  stream->common.flush = cdcacm_outstream_flush;
  stream->common.none  = NULL;
  stream->dev          = dev;

  return OK;
}

/****************************************************************************
 * Name: cdcacm_outstream_close
 *
 * Description:
 *   Release any resources owned by stream and detach it from the cdcacm
 *   instance.  After this call the stream object must not be used.
 *
 * Input Parameters:
 *   stream - Stream object previously initialized by
 *            cdcacm_outstream_open().
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void cdcacm_outstream_close(FAR struct cdcacm_outstream_s *stream)
{
  FAR struct cdcacm_dev_s *dev;

  if (stream == NULL)
    {
      return;
    }

  dev = stream->dev;
  if (dev != NULL)
    {
      stream->dev = NULL;        /* prevent reuse */
      cdcacm_release(dev);       /* match cdcacm_outstream_open's acquire */
    }
}
