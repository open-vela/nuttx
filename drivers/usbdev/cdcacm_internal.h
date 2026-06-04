/****************************************************************************
 * drivers/usbdev/cdcacm_internal.h
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

#ifndef __DRIVERS_USBDEV_CDCACM_INTERNAL_H
#define __DRIVERS_USBDEV_CDCACM_INTERNAL_H

/****************************************************************************
 * Description
 *
 * Private types and helper prototypes shared between cdcacm.c (the USB
 * protocol core) and cdcacm_serial.c (the uart_dev_s adapter).  Not part of
 * the public API; do NOT include from outside drivers/usbdev/.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/atomic.h>
#include <nuttx/queue.h>
#include <nuttx/semaphore.h>
#include <nuttx/serial/serial.h>
#include <nuttx/spinlock.h>
#include <nuttx/wdog.h>

#include <nuttx/usb/cdc.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/usbdev.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RX poll delay = 200 milliseconds. CLK_TCK is the number of clock ticks per
 * second
 */

#define CDCACM_RXDELAY   (CLK_TCK / 5)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Container to support a list of requests */

struct cdcacm_wrreq_s
{
  FAR struct cdcacm_wrreq_s *flink;    /* Implements a singly linked list */
  FAR struct usbdev_req_s *req;        /* The contained request */
};

struct cdcacm_rdreq_s
{
  FAR struct cdcacm_rdreq_s *flink;    /* Implements a singly linked list */
  FAR struct usbdev_req_s *req;        /* The contained request */
  uint16_t offset;                     /* Offset to valid data in the RX request */
};

/* This structure describes the internal state of the driver */

struct cdcacm_dev_s
{
  FAR struct usbdev_s *usbdev;         /* usbdev driver pointer */

  uint8_t config;                      /* Configuration number */
  uint8_t nwrq;                        /* Number of queue write requests (in txfree) */
  uint8_t nrdq;                        /* Number of queue read requests (in epbulkout) */
  uint8_t minor;                       /* The device minor number */
  uint8_t ctrlline;                    /* Host-side control line state (DTR/RTS bits);
                                        * read by cdcacm_write for the DTE_PRESENT
                                        * gate.  Maintained by cdcacm core regardless
                                        * of whether a user_ops adapter is installed.
                                        */
#ifdef CONFIG_CDCACM_IFLOWCONTROL
  uint8_t serialstate;                 /* State of the DSR/DCD */
  bool iflow;                          /* True: input flow control is enabled */
  bool iactive;                        /* True: input flow control is active */
  bool upper;                          /* True: RX buffer is (nearly) full */
#endif
  bool rxenabled;                      /* true: UART RX "interrupts" enabled */
  bool ispolling;
  spinlock_t lock;

  FAR struct usbdev_ep_s *epintin;     /* Interrupt IN endpoint structure */
  FAR struct usbdev_ep_s *epbulkin;    /* Bulk IN endpoint structure */
  FAR struct usbdev_ep_s *epbulkout;   /* Bulk OUT endpoint structure */
  FAR struct usbdev_req_s *ctrlreq;    /* Allocated control request */
#ifndef CONFIG_CDCACM_DISABLE_RXBUF
  struct wdog_s rxfailsafe;            /* Failsafe timer to prevent RX stalls */
#endif
  struct sq_queue_s txfree;            /* Available write request containers */
  struct sq_queue_s rxpending;         /* Pending read request containers */

  struct usbdev_devinfo_s devinfo;

  /* Pre-allocated write request containers.  The write requests will
   * be linked in a free list (txfree), and used to send requests to
   * EPBULKIN; Read requests will be queued in the EBULKOUT.
   */

  struct cdcacm_wrreq_s wrreqs[CONFIG_CDCACM_NWRREQS];
  struct cdcacm_rdreq_s rdreqs[CONFIG_CDCACM_NRDREQS];

  /* Serial I/O req container (only the zero-copy alias paths still need
   * a dedicated rd/wr-container slot in the cdcacm core; the legacy uart
   * inline rxbuffer/txbuffer arrays are gone -- cdcacm_serial.c owns
   * those buffers when running as the uart adapter.
   */

#ifdef CONFIG_CDCACM_DISABLE_RXBUF
  FAR struct cdcacm_rdreq_s *rdcontainer;
#endif
#ifdef CONFIG_CDCACM_DISABLE_TXBUF
  FAR struct cdcacm_wrreq_s *wrcontainer;
#endif

  /* === Refcount-based lifecycle === */

  atomic_t                         refcount;     /* total holders */
  bool                             closing;      /* unregister in progress */
  sem_t                            close_done;   /* drain notification */
  spinlock_t                       ops_lock;     /* user_ops update spinlock (IRQ-safe) */
  FAR const struct cdcacm_user_ops_s *user_ops;  /* may be NULL */
  FAR void                        *user_priv;

  /* === /dev/cdcacmN chardev path === */

  uint8_t                          chardev_open_count; /* open() refs */
  bool                             rxreader_open;      /* read fd holds RX */
  bool                             txwriter_open;      /* write fd holds TX */
  sem_t                            rx_waitsem;         /* rxpending grew */
  sem_t                            tx_waitsem;         /* txfree grew */
};

/* The internal version of the class driver */

struct cdcacm_driver_s
{
  struct usbdevclass_driver_s drvr;
  FAR struct cdcacm_dev_s     *dev;
};

/* This is what is allocated */

struct cdcacm_alloc_s
{
  struct cdcacm_dev_s    dev;
  struct cdcacm_driver_s drvr;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Implemented in cdcacm.c, called from cdcacm_serial.c */

int  cdcacm_sndpacket(FAR struct cdcacm_dev_s *priv);
int  cdcacm_release_rxpending(FAR struct cdcacm_dev_s *priv);
int  cdcacm_requeue_rdrequest(FAR struct cdcacm_dev_s *priv,
                              FAR struct cdcacm_rdreq_s *rdcontainer);
#ifdef CONFIG_CDCACM_IFLOWCONTROL
int  cdcacm_serialstate(FAR struct cdcacm_dev_s *priv);
#endif

#endif /* __DRIVERS_USBDEV_CDCACM_INTERNAL_H */
