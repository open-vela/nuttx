/****************************************************************************
 * arch/arm/src/t113/t113_usbdev.c
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
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/trace.h>
#include <nuttx/spinlock.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/trace.h>

#include "arm_internal.h"
#include <arch/barriers.h>
#include "hardware/t113_usb.h"

#ifdef CONFIG_T113_DMA
#  include <nuttx/cache.h>
#  include "hardware/t113_dma.h"
#  include "t113_dma.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USB OTG register base */

#define MUSB_BASE            T113_USB_OTG_BASE
#define PHY_BASE             T113_USB_PHY_BASE

/* Number of physical endpoints (EP0-EP4) */

#define T113_NPHYSEP         T113_USB_NEPS
#define T113_NLOGEP          (T113_NPHYSEP * 2)  /* IN + OUT for each */

/* EP0 max packet size */

#define EP0_MAXPACKET        64

/* Endpoint bitmask helpers */

#define T113_EPALLSET        0x1ff  /* EP0-EP4, both directions */
#define T113_EPCTRLSET       0x003  /* EP0 IN + OUT */

#ifdef CONFIG_T113_DMA

/* TX DMA: use MUSB internal DMA (OTG+0x500/0x540) - truly internal to
 * the MUSB controller, no MBUS arbitration window.  Functional on T113
 * when VEND0 BUS_SEL=1 is programmed per transaction.
 */

#  define T113_TX_DMA_THRESHOLD   1024

/* RX DMA threshold (minimum request-remaining bytes to arm DMA).
 * Since AUTOCLEAR moves whole maxpacket packets, the realistic floor
 * is 1 maxpacket (512).  Anything smaller is a short packet that
 * must be drained via PIO anyway.
 */

#  define T113_RX_DMA_THRESHOLD   512

/* DRQ port for a given EP number (1-based): Port 30 = EP1, etc. */

#  define T113_USB_DRQ(epno)   (DRQ_USB0_EP1 + (epno) - 1)
#endif

/* Debug tracing */

#ifdef CONFIG_DEBUG_USB_INFO
#  define usb_trace_info(fmt, ...) uinfo(fmt, ##__VA_ARGS__)
#else
#  define usb_trace_info(fmt, ...)
#endif

#ifdef CONFIG_DEBUG_USB_ERROR
#  define usb_trace_err(fmt, ...)  uerr(fmt, ##__VA_ARGS__)
#else
#  define usb_trace_err(fmt, ...)
#endif

/* Logical EP address encoding (matches NuttX convention) */

#define T113_EPPHYIN(n)      ((n) * 2)
#define T113_EPPHYOUT(n)     ((n) * 2 + 1)
#define T113_EP0_IN          T113_EPPHYIN(0)
#define T113_EP0_OUT         T113_EPPHYOUT(0)

/* Convert physical index to logical address */

#define PHYIN2LOG(n)         ((n) | USB_DIR_IN)
#define PHYOUT2LOG(n)        ((n) | USB_DIR_OUT)

/* Register access helpers.
 *
 * Two sets of accessors:
 *   musb_writeb / musb_readb / musb_putreg16 / musb_getreg16
 *     - with ISB/DSB barriers, used on slow paths (init,
 *       configure, EP0).
 *   Barrier-free "fast" variants
 *     - used in ISR data-path hot loops.  ARM Device-memory
 *       semantics guarantee ordering for accesses to the
 *       same peripheral, so explicit barriers are not needed.
 */

#define phy_getreg32(off)    getreg32(PHY_BASE + (off))
#define phy_putreg32(v, off) putreg32((v), PHY_BASE + (off))

/* T113 MUSB register access with ISB/DSB barriers matching vendor HAL.
 * The vendor hal_writeb/hal_readb use ISB before and DSB after every
 * register access.  Without these barriers, writes may be silently
 * dropped by the CPU pipeline.
 */

static inline void musb_writeb(uint8_t val, uint32_t off)
{
  UP_ISB();
  putreg8(val, MUSB_BASE + off);
  UP_DSB();
}

static inline uint8_t musb_readb(uint32_t off)
{
  UP_ISB();
  uint8_t v = getreg8(MUSB_BASE + off);
  UP_DSB();
  return v;
}

static inline void musb_putreg16(uint16_t val, uint32_t off)
{
  UP_ISB();
  putreg16(val, MUSB_BASE + off);
  UP_DSB();
}

static inline uint16_t musb_getreg16(uint32_t off)
{
  UP_ISB();
  uint16_t v = getreg16(MUSB_BASE + off);
  UP_DSB();
  return v;
}

/* Barrier-free accessors for data-path hot loops (TXCSR, RXCSR,
 * RXCOUNT, EP select, FIFO).  MUSB CSR and FIFO share the same AHB
 * slave so peripheral-side ordering is guaranteed by the bus.
 * volatile semantics prevent compiler reordering.
 */

static inline uint16_t musb_getreg16_fast(uint32_t off)
{
  return getreg16(MUSB_BASE + off);
}

static inline void musb_putreg16_fast(uint16_t val, uint32_t off)
{
  putreg16(val, MUSB_BASE + off);
}

#define musb_setbits16_fast(off, bits) \
  musb_putreg16_fast(musb_getreg16_fast(off) | (bits), (off))
#define musb_clrbits16_fast(off, bits) \
  musb_putreg16_fast(musb_getreg16_fast(off) & ~(bits), (off))

/* Modify helpers (with barriers, for control/setup paths) */

#define musb_setbits16(off, bits) \
  musb_putreg16(musb_getreg16(off) | (bits), (off))
#define musb_clrbits16(off, bits) \
  musb_putreg16(musb_getreg16(off) & ~(bits), (off))

/* RXCSR-specific RMW helpers: RXPKTRDY (bit 0) in device mode is W0C
 * (writing 0 explicitly clears, writing 1 is a no-op).  A plain RMW
 * that reads csr when RXPKTRDY=0, then writes back bit 0 = 0, will
 * clobber RXPKTRDY if hardware set it between read and write (new
 * packet arrival).  These helpers OR RXPKTRDY=1 into the write so the
 * bit is preserved regardless of the hw state at write time.  Use for
 * any RMW on RXCSR that may run while the EP can still accept new
 * packets.  See t113_musb_dma_rx_done() for the original discovery.
 */

#define musb_rxcsr_clrbits_safe(bits)                                \
  musb_putreg16_fast(                                                \
    (musb_getreg16_fast(MUSB_RXCSR) & ~(bits)) |                     \
    MUSB_RXCSR_RXPKTRDY, MUSB_RXCSR)

#define musb_rxcsr_setbits_safe(bits)                                \
  musb_putreg16_fast(                                                \
    musb_getreg16_fast(MUSB_RXCSR) | (bits) | MUSB_RXCSR_RXPKTRDY,   \
    MUSB_RXCSR)

/* Barriered variants for control/setup paths */

#define musb_rxcsr_clrbits_safe_bar(bits)                            \
  musb_putreg16(                                                     \
    (musb_getreg16(MUSB_RXCSR) & ~(bits)) |                          \
    MUSB_RXCSR_RXPKTRDY, MUSB_RXCSR)

#define musb_rxcsr_setbits_safe_bar(bits)                            \
  musb_putreg16(                                                     \
    musb_getreg16(MUSB_RXCSR) | (bits) | MUSB_RXCSR_RXPKTRDY,        \
    MUSB_RXCSR)

#define phy_setbits32(off, bits) \
  phy_putreg32(phy_getreg32(off) | (bits), (off))
#define phy_clrbits32(off, bits) \
  phy_putreg32(phy_getreg32(off) & ~(bits), (off))

/* FIFO address is in units of 8 bytes */

#define FIFO_ADDR(bytes)     ((bytes) >> 3)

/* ISCR change detect bits (not exposed in t113_usb.h register map) */

#define USB_ISCR_VBUS_CHANGE_DETECT   0x0040
#define USB_ISCR_ID_CHANGE_DETECT     0x0020
#define USB_ISCR_DPDM_CHANGE_DETECT   0x0010

/* Debug trace variables - read via JLink mem32 */

/* EP0 state machine */

#define EP0STATE_IDLE             0   /* Idle, waiting for SETUP */
#define EP0STATE_SETUP_OUT        1   /* OUT SETUP received (no data) */
#define EP0STATE_SETUP_IN         2   /* IN data requested by host */
#define EP0STATE_DATA_IN          3   /* Sending IN data */
#define EP0STATE_DATA_OUT         4   /* Receiving OUT data */
#define EP0STATE_SHORTWRITE       5   /* Short write, no req queued */
#define EP0STATE_WAIT_STATUS_IN   6   /* Waiting for IN status phase */
#define EP0STATE_WAIT_STATUS_OUT  7   /* Waiting for OUT status phase */
#define EP0STATE_STALL            8   /* Stalled */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* FIFO configuration for each endpoint direction */

struct t113_fifoconfig_s
{
  uint8_t  epno;        /* Physical EP number */
  bool     is_in;       /* true = TX/IN, false = RX/OUT */
  uint16_t addr;        /* FIFO start address (bytes) */
  uint16_t size;        /* FIFO size (bytes) */
  uint8_t  fifosz;      /* MUSB size encoding */
  bool     dpb;         /* Double-packet buffer */
};

/* Request wrapper */

struct t113_req_s
{
  struct usbdev_req_s  req;      /* Standard USB request (must be first) */
  struct t113_req_s   *flink;    /* Singly-linked list next pointer */
  struct t113_ep_s    *complep;  /* EP that completed this req */
};

/* Endpoint state */

struct t113_ep_s
{
  struct usbdev_ep_s       ep;       /* Standard EP (must be first) */
  struct t113_usbdev_s    *dev;      /* Back-pointer to device */
  struct t113_req_s       *head;     /* Request queue head */
  struct t113_req_s       *tail;     /* Request queue tail */
  uint8_t                  epphy;    /* Physical EP number (0-4) */
  bool                     is_in;    /* Direction: true=IN/TX */
  uint8_t                  eptype;   /* Transfer type */
  bool                     stalled;  /* Endpoint stalled */
  uint16_t                 fifosz;   /* FIFO size */
#ifdef CONFIG_T113_DMA
  bool                     dma_busy;       /* DMA transfer in progress */
  uint32_t                 dma_len;        /* Bytes programmed for DMA */
  int8_t                   musb_dma_ch;    /* MUSB internal DMA ch (TX) */
  int8_t                   musb_dma_ch_rx; /* MUSB internal DMA ch (RX) */
#endif
};

/* Device state */

struct t113_usbdev_s
{
  struct usbdev_s              usbdev;    /* Standard device (must be first) */
  struct usbdevclass_driver_s *driver;    /* Bound class driver */

  /* SMP lock for request queue and hardware register access */

  spinlock_t lock;

  /* EP0 control transfer state */

  uint8_t  ep0state;                      /* EP0 state machine */
  uint8_t  ep0buf[EP0_MAXPACKET];         /* EP0 data buffer */
  uint16_t ep0datlen;                     /* EP0 data length */
  uint16_t ep0reqlen;                     /* EP0 request wLength */

  /* Device state */

  uint8_t  paddr;                         /* USB address */
  bool     paddrset;                      /* Address pending */
  bool     selfpowered;                   /* Self-powered flag */
  bool     attached;                      /* Host attached */
  bool     suspended;                     /* Suspended */

  /* Setup request buffer */

  struct usb_ctrlreq_s  ep0ctrl;          /* Last SETUP packet */

  /* Available endpoints bitmap */

  uint32_t  epavail;

  /* All endpoint structures: EP0 IN/OUT + EP1-4 IN + EP1-4 OUT = 10 */

  struct t113_ep_s  eplist[T113_NLOGEP];

  /* Deferred completion queue - filled under lock by
   * t113_reqcomplete(), drained outside lock at every exit point.
   *
   * Ordering guarantee: when multiple lock-release cycles happen
   * (e.g. nested IRQ during drain), only the outermost drain fires
   * callbacks.  Inner unlock_and_drain detects 'draining == true'
   * and skips - the outer loop will re-acquire the lock and pick up
   * any new completions.  This ensures callback order matches the
   * order completions were queued (FIFO order).
   */

  struct t113_req_s *done_head;
  struct t113_req_s *done_tail;
  bool               draining;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Request queue management */

static bool t113_rqempty(struct t113_ep_s *privep);
static struct t113_req_s *t113_rqpeek(struct t113_ep_s *privep);
static struct t113_req_s *t113_rqdequeue(struct t113_ep_s *privep);
static bool t113_rqenqueue(struct t113_ep_s *privep,
                           struct t113_req_s *privreq);

/* Low-level MUSB register access */

static void t113_ep_select(uint8_t epno);
static void t113_fifo_read(uint8_t epno, uint8_t *buf, uint16_t len);
static void t113_fifo_write(uint8_t epno, const uint8_t *buf, uint16_t len);

/* Request processing */

static void t113_reqcomplete(struct t113_ep_s *privep,
                             struct t113_req_s *privreq,
                             int16_t result);
static void t113_drain_done(struct t113_req_s *done);
static void t113_cancelrequests(struct t113_ep_s *privep,
                                int16_t status);

/* EP0 handling */

static void t113_ep0_setup(struct t113_usbdev_s *priv);
static void t113_ep0_indone(struct t113_usbdev_s *priv);
static void t113_ep0_outdone(struct t113_usbdev_s *priv);
static void t113_ep0_dispatch(struct t113_usbdev_s *priv);
static void t113_ep0_transmit(struct t113_usbdev_s *priv,
                              const uint8_t *buf, uint16_t len);

/* EPn handling */

static void t113_epn_txdone(struct t113_usbdev_s *priv, uint8_t epno);
static void t113_epn_rxready(struct t113_usbdev_s *priv, uint8_t epno);
static void t113_epn_txstart(struct t113_ep_s *privep);

/* Interrupt handling */

static int  t113_usbdev_interrupt(int irq, void *context, void *arg);

/* Hardware init / deinit */

static void t113_ccu_init(void);
static void t113_phy_init(void);
static void t113_musb_init(struct t113_usbdev_s *priv);
static void t113_musb_enable(void);
static void t113_musb_reset(struct t113_usbdev_s *priv);

/* Endpoint operations (usbdev_epops_s) */

static int  t113_epconfigure(struct usbdev_ep_s *ep,
                             const struct usb_epdesc_s *desc, bool last);
static int  t113_epdisable(struct usbdev_ep_s *ep);
static struct usbdev_req_s *t113_epallocreq(struct usbdev_ep_s *ep);
static void t113_epfreereq(struct usbdev_ep_s *ep,
                           struct usbdev_req_s *req);
static int  t113_epsubmit(struct usbdev_ep_s *ep,
                          struct usbdev_req_s *req);
static int  t113_epcancel(struct usbdev_ep_s *ep,
                          struct usbdev_req_s *req);
static int  t113_epstall(struct usbdev_ep_s *ep, bool resume);

/* Device operations (usbdev_ops_s) */

static struct usbdev_ep_s *t113_allocep(struct usbdev_s *dev,
                                        uint8_t epphy, bool in,
                                        uint8_t eptype);
static void t113_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep);
static int  t113_getframe(struct usbdev_s *dev);
static int  t113_wakeup(struct usbdev_s *dev);
static int  t113_selfpowered(struct usbdev_s *dev, bool selfpowered);
static int  t113_pullup(struct usbdev_s *dev, bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Endpoint operations */

static const struct usbdev_epops_s g_epops =
{
  .configure  = t113_epconfigure,
  .disable    = t113_epdisable,
  .allocreq   = t113_epallocreq,
  .freereq    = t113_epfreereq,
  .submit     = t113_epsubmit,
  .cancel     = t113_epcancel,
  .stall      = t113_epstall,
};

/* Device operations */

static const struct usbdev_ops_s g_devops =
{
  .allocep      = t113_allocep,
  .freeep       = t113_freeep,
  .getframe     = t113_getframe,
  .wakeup       = t113_wakeup,
  .selfpowered  = t113_selfpowered,
  .pullup       = t113_pullup,
};

/* Single device instance */

static struct t113_usbdev_s g_usbdev;

/* FIFO layout for T113 (8KB SRAM, EP0-EP4):
 *
 * All bulk EPs use 512B + DPB (double-packet buffering) for full-speed
 * ping-pong transfer. 512B matches USB 2.0 HS bulk maxpacket exactly.
 * DPB costs 2x SRAM (1024B per direction) but eliminates NAK stalls.
 *
 * EP1-EP3: 512B DPB, covers MSC, CDC-ACM, Composite (up to 3 functions)
 * EP4:     512B single, rarely used, only for 4+ function composite
 *
 * SRAM budget: 64 + 6*1024 + 2*512 = 7232 / 8192 (960B spare)
 */

/* Fields: ep, is_in, addr, size, fifosz, dpb.  Tail comment shows SRAM. */

static const struct t113_fifoconfig_s g_fifoconfig[] =
{
  {0, true,     0,   64, MUSB_FIFOSZ_64,   false}, /*   64  0..63     */
  {1, true,    64,  512, MUSB_FIFOSZ_512,   true}, /* 1024  64..1087  */
  {1, false, 1088,  512, MUSB_FIFOSZ_512,   true}, /* 1024  1088..2111 */
  {2, true,  2112,  512, MUSB_FIFOSZ_512,   true}, /* 1024  2112..3135 */
  {2, false, 3136,  512, MUSB_FIFOSZ_512,   true}, /* 1024  3136..4159 */
  {3, true,  4160,  512, MUSB_FIFOSZ_512,   true}, /* 1024  4160..5183 */
  {3, false, 5184,  512, MUSB_FIFOSZ_512,   true}, /* 1024  5184..6207 */
  {4, true,  6208,  512, MUSB_FIFOSZ_512,  false}, /*  512  6208..6719 */
  {4, false, 6720,  512, MUSB_FIFOSZ_512,  false}, /*  512  6720..7231 */
};

#define NFIFOCONFIGS (sizeof(g_fifoconfig) / sizeof(g_fifoconfig[0]))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_rqempty
 *
 * Description:
 *   Check if the request queue for an endpoint is empty.
 *
 ****************************************************************************/

static bool t113_rqempty(struct t113_ep_s *privep)
{
  return privep->head == NULL;
}

/****************************************************************************
 * Name: t113_rqpeek
 *
 * Description:
 *   Return the head request without removing it.
 *
 ****************************************************************************/

static struct t113_req_s *t113_rqpeek(struct t113_ep_s *privep)
{
  return privep->head;
}

/****************************************************************************
 * Name: t113_rqdequeue
 *
 * Description:
 *   Dequeue the head request.
 *
 ****************************************************************************/

static struct t113_req_s *t113_rqdequeue(struct t113_ep_s *privep)
{
  struct t113_req_s *ret = privep->head;

  if (ret != NULL)
    {
      privep->head = ret->flink;
      if (privep->head == NULL)
        {
          privep->tail = NULL;
        }

      ret->flink = NULL;
    }

  return ret;
}

/****************************************************************************
 * Name: t113_rqenqueue
 *
 * Description:
 *   Enqueue a request.  Returns true if the queue was empty before.
 *
 ****************************************************************************/

static bool t113_rqenqueue(struct t113_ep_s *privep,
                           struct t113_req_s *privreq)
{
  bool was_empty = (privep->head == NULL);

  privreq->flink = NULL;
  if (was_empty)
    {
      privep->head = privreq;
      privep->tail = privreq;
    }
  else
    {
      privep->tail->flink = privreq;
      privep->tail = privreq;
    }

  return was_empty;
}

/****************************************************************************
 * Name: t113_ep_select
 *
 * Description:
 *   Select an endpoint via the MUSB INDEX register.
 *
 ****************************************************************************/

static void t113_ep_select(uint8_t epno)
{
  musb_writeb(epno, MUSB_INDEX);
}

static inline void t113_ep_select_fast(uint8_t epno)
{
  putreg8(epno, MUSB_BASE + MUSB_INDEX);
}

static void t113_fifo_read_fast(uint8_t epno, uint8_t *buf, uint16_t len)
{
  volatile uint32_t *fifo =
    (volatile uint32_t *)(MUSB_BASE + MUSB_FIFO(epno));
  volatile uint8_t *fifo8 =
    (volatile uint8_t *)(MUSB_BASE + MUSB_FIFO(epno));
  uint32_t count32 = len >> 2;
  uint16_t i;

  for (i = 0; i < count32; i++)
    {
      uint32_t tmp = *fifo;
      memcpy(buf, &tmp, 4);
      buf += 4;
    }

  len &= 3;
  for (i = 0; i < len; i++)
    {
      *buf++ = *fifo8;
    }
}

/****************************************************************************
 * Name: t113_fifo_read
 *
 * Description:
 *   Read data from the MUSB FIFO for the given endpoint.
 *
 ****************************************************************************/

static void t113_fifo_read(uint8_t epno, uint8_t *buf, uint16_t len)
{
  uint32_t fifo_addr = MUSB_BASE + MUSB_FIFO(epno);
  uint32_t count32;
  uint16_t i;

  /* Read 32-bit words first for efficiency */

  count32 = len >> 2;
  for (i = 0; i < count32; i++)
    {
      uint32_t tmp = getreg32(fifo_addr);
      memcpy(buf, &tmp, 4);
      buf += 4;
    }

  /* Read remaining bytes */

  len &= 3;
  for (i = 0; i < len; i++)
    {
      *buf++ = getreg8(fifo_addr);
    }
}

/****************************************************************************
 * Name: t113_fifo_write
 *
 * Description:
 *   Write data to the MUSB FIFO for the given endpoint.
 *
 ****************************************************************************/

static void t113_fifo_write(uint8_t epno, const uint8_t *buf, uint16_t len)
{
  uint32_t fifo_addr = MUSB_BASE + MUSB_FIFO(epno);
  uint32_t count32;
  uint16_t i;

  /* Write 32-bit words first for efficiency */

  count32 = len >> 2;
  for (i = 0; i < count32; i++)
    {
      uint32_t tmp;
      memcpy(&tmp, buf, 4);
      putreg32(tmp, fifo_addr);
      buf += 4;
    }

  /* Write remaining bytes */

  len &= 3;
  for (i = 0; i < len; i++)
    {
      putreg8(*buf++, fifo_addr);
    }
}

/****************************************************************************
 * Name: t113_fifo_write_fast
 *
 * Description:
 *   Barrier-free FIFO write for the EPn TX data path.
 *   Uses volatile pointer access instead of putreg32/putreg8.
 *
 ****************************************************************************/

static void t113_fifo_write_fast(uint8_t epno,
                                 const uint8_t *buf,
                                 uint16_t len)
{
  volatile uint32_t *fifo =
    (volatile uint32_t *)(MUSB_BASE + MUSB_FIFO(epno));
  volatile uint8_t *fifo8 =
    (volatile uint8_t *)(MUSB_BASE + MUSB_FIFO(epno));
  uint32_t count32 = len >> 2;
  uint32_t tmp;
  uint16_t i;

  for (i = 0; i < count32; i++)
    {
      memcpy(&tmp, buf, 4);
      *fifo = tmp;
      buf += 4;
    }

  len &= 3;
  for (i = 0; i < len; i++)
    {
      *fifo8 = *buf++;
    }
}

/****************************************************************************
 * Name: t113_reqcomplete
 *
 * Description:
 *   Queue a completed request for deferred callback.  The actual
 *   callback is invoked later outside the spinlock by
 *   t113_drain_done().  Caller must hold priv->lock.
 *
 ****************************************************************************/

static void t113_reqcomplete(struct t113_ep_s *privep,
                             struct t113_req_s *privreq,
                             int16_t result)
{
  struct t113_usbdev_s *priv = privep->dev;

  privreq->req.result = result;
  privreq->complep    = privep;
  privreq->flink      = NULL;

  usb_trace_info("EP%d %s complete: len=%d xfrd=%d result=%d\n",
                 privep->epphy, privep->is_in ? "IN" : "OUT",
                 privreq->req.len, privreq->req.xfrd, result);

  /* Append to done queue (caller holds lock) */

  if (priv->done_tail != NULL)
    {
      priv->done_tail->flink = privreq;
    }
  else
    {
      priv->done_head = privreq;
    }

  priv->done_tail = privreq;
}

/****************************************************************************
 * Name: t113_drain_done
 *
 * Description:
 *   Invoke completion callbacks for a detached done-list.
 *   Called outside the controller lock with a local head pointer
 *   that was snapshot'd under the lock.  No shared state is
 *   accessed, so no locking is needed.
 *
 ****************************************************************************/

static void t113_drain_done(struct t113_req_s *done)
{
  while (done != NULL)
    {
      struct t113_req_s *cur = done;
      struct t113_ep_s *ep   = cur->complep;

      done = cur->flink;
      cur->flink = NULL;
      cur->req.callback(&ep->ep, &cur->req);
    }
}

/****************************************************************************
 * Name: t113_unlock_and_drain
 *
 * Description:
 *   Snapshot the deferred completion queue under the controller
 *   lock, release the lock, then invoke all pending callbacks.
 *   Every lock-exit point that may have called t113_reqcomplete()
 *   must use this instead of a bare spin_unlock_irqrestore().
 *
 ****************************************************************************/

static inline void t113_unlock_and_drain(
    struct t113_usbdev_s *priv, irqstate_t flags)
{
  struct t113_req_s *done;

  /* If another context is already draining (e.g. we are a nested IRQ
   * that preempted the outer drain between spin_unlock and callback
   * invocation), just release the lock.  The outer drain loop will
   * re-acquire the lock and pick up our completions, preserving the
   * global FIFO ordering of callbacks.
   */

  if (priv->draining)
    {
      spin_unlock_irqrestore(&priv->lock, flags);
      return;
    }

  priv->draining = true;

  /* Drain loop: snapshot the done list, release the lock, fire
   * callbacks, then re-check.  Any completions added by an IRQ
   * that fires between spin_unlock and the re-lock are picked up
   * by the next iteration - in strict FIFO order.
   */

  do
    {
      done = priv->done_head;
      priv->done_head = NULL;
      priv->done_tail = NULL;
      spin_unlock_irqrestore(&priv->lock, flags);

      t113_drain_done(done);

      flags = spin_lock_irqsave(&priv->lock);
    }
  while (priv->done_head != NULL);

  priv->draining = false;
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: t113_cancelrequests
 *
 * Description:
 *   Cancel all queued requests on an endpoint.
 *
 ****************************************************************************/

static void t113_cancelrequests(struct t113_ep_s *privep,
                                int16_t status)
{
  while (!t113_rqempty(privep))
    {
      struct t113_req_s *privreq = t113_rqdequeue(privep);
      t113_reqcomplete(privep, privreq, status);
    }
}

/****************************************************************************
 * Name: t113_ep0_transmit
 *
 * Description:
 *   Start an EP0 IN data transfer.
 *
 ****************************************************************************/

static void t113_ep0_transmit(struct t113_usbdev_s *priv,
                              const uint8_t *buf, uint16_t len)
{
  uint16_t xfrlen;
  uint16_t csr0;

  t113_ep_select(0);

  /* Limit to max packet and requested length */

  xfrlen = len;
  if (xfrlen > EP0_MAXPACKET)
    {
      xfrlen = EP0_MAXPACKET;
    }

  if (xfrlen > priv->ep0reqlen)
    {
      xfrlen = priv->ep0reqlen;
    }

  /* Write data to EP0 FIFO */

  if (xfrlen > 0)
    {
      t113_fifo_write(0, buf, xfrlen);
    }

  priv->ep0datlen -= xfrlen;
  priv->ep0reqlen -= xfrlen;

  /* Set TXPKTRDY. If this is the last packet, set DATAEND too. */

  csr0 = MUSB_CSR0_TXPKTRDY;
  if (xfrlen < EP0_MAXPACKET || priv->ep0datlen == 0 ||
      priv->ep0reqlen == 0)
    {
      /* This is the last packet */

      csr0 |= MUSB_CSR0_DATAEND;
      priv->ep0state = EP0STATE_WAIT_STATUS_OUT;
    }
  else
    {
      priv->ep0state = EP0STATE_DATA_IN;
    }

  musb_putreg16(csr0, MUSB_CSR0);
}

/****************************************************************************
 * Name: t113_ep0_dispatch
 *
 * Description:
 *   Dispatch a SETUP packet to the class driver.
 *
 ****************************************************************************/

static void t113_ep0_dispatch(struct t113_usbdev_s *priv)
{
  uint8_t saved_index;
  int ret;

  if (priv->driver == NULL)
    {
      return;
    }

  saved_index = musb_readb(MUSB_INDEX);
  spin_unlock(&priv->lock);
  ret = CLASS_SETUP(priv->driver, &priv->usbdev,
                    &priv->ep0ctrl, priv->ep0buf,
                    priv->ep0datlen);
  spin_lock(&priv->lock);
  musb_writeb(saved_index, MUSB_INDEX);
  if (ret < 0)
    {
      /* Stall EP0 on error */

      usb_trace_err("CLASS_SETUP failed: %d\n", ret);
      priv->ep0state = EP0STATE_STALL;
      t113_ep_select(0);
      musb_putreg16(MUSB_CSR0_SENDSTALL, MUSB_CSR0);
    }
  else if (ret == 0 && USB_REQ_ISOUT(priv->ep0ctrl.type) &&
           GETUINT16(priv->ep0ctrl.len) == 0)
    {
      /* No-data OUT control transfer completed */

      t113_ep_select(0);
      musb_putreg16(MUSB_CSR0_SVDRXPKTRDY | MUSB_CSR0_DATAEND, MUSB_CSR0);
      priv->ep0state = EP0STATE_IDLE;
    }
}

/****************************************************************************
 * Name: t113_ep0_stdrequest
 *
 * Description:
 *   Handle standard EP0 control requests (SET_ADDRESS etc).
 *   Returns true if handled, false if the class driver should handle it.
 *
 ****************************************************************************/

static bool t113_ep0_stdrequest(struct t113_usbdev_s *priv,
                                const struct usb_ctrlreq_s *ctrl)
{
  uint16_t value  = GETUINT16(ctrl->value);
  uint16_t len    = GETUINT16(ctrl->len);
  bool handled    = false;

  /* Only handle standard device requests */

  if ((ctrl->type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      return false;
    }

  switch (ctrl->req)
    {
      case USB_REQ_SETADDRESS:
        {
          /* Save address, will be set after status phase */

          priv->paddr = (uint8_t)(value & 0x7f);
          priv->paddrset = true;

          /* Send zero-length status */

          t113_ep_select(0);
          musb_putreg16(MUSB_CSR0_SVDRXPKTRDY | MUSB_CSR0_DATAEND,
                        MUSB_CSR0);
          priv->ep0state = EP0STATE_WAIT_STATUS_IN;
          handled = true;

          usb_trace_info("SET_ADDRESS: %d\n", priv->paddr);
        }
        break;

      case USB_REQ_GETSTATUS:
        {
          uint8_t recipient = ctrl->type & USB_REQ_RECIPIENT_MASK;
          uint16_t status_val = 0;

          if (recipient == USB_REQ_RECIPIENT_DEVICE)
            {
              status_val = priv->selfpowered ?
                           USB_FEATURE_SELFPOWERED : 0;
            }
          else if (recipient == USB_REQ_RECIPIENT_ENDPOINT)
            {
              /* Check stall status of the addressed endpoint */

              uint8_t epno = USB_EPNO(GETUINT16(ctrl->index));

              if (epno < T113_NPHYSEP)
                {
                  /* Check both IN and OUT for this EP number */

                  int idx = USB_ISEPIN(GETUINT16(ctrl->index)) ?
                            T113_EPPHYIN(epno) : T113_EPPHYOUT(epno);
                  if (priv->eplist[idx].stalled)
                    {
                      status_val = 1;
                    }
                }
            }

          priv->ep0buf[0] = (uint8_t)(status_val & 0xff);
          priv->ep0buf[1] = (uint8_t)((status_val >> 8) & 0xff);

          t113_ep_select(0);
          musb_putreg16(MUSB_CSR0_SVDRXPKTRDY, MUSB_CSR0);

          priv->ep0datlen = 2;
          priv->ep0reqlen = len;
          t113_ep0_transmit(priv, priv->ep0buf, 2);
          handled = true;
        }
        break;

      case USB_REQ_CLEARFEATURE:
        {
          if ((ctrl->type & USB_REQ_RECIPIENT_MASK) ==
              USB_REQ_RECIPIENT_ENDPOINT &&
              value == USB_FEATURE_ENDPOINTHALT)
            {
              uint8_t epno = USB_EPNO(GETUINT16(ctrl->index));
              bool is_in   = USB_ISEPIN(GETUINT16(ctrl->index));

              if (epno > 0 && epno < T113_NPHYSEP)
                {
                  int idx = is_in ? T113_EPPHYIN(epno) :
                                    T113_EPPHYOUT(epno);
                  struct t113_ep_s *privep = &priv->eplist[idx];

                  /* Clear the stall */

                  privep->stalled = false;
                  t113_ep_select(epno);

                  if (is_in)
                    {
                      musb_clrbits16(MUSB_TXCSR,
                                     MUSB_TXCSR_SENDSTALL |
                                     MUSB_TXCSR_SENTSTALL);
                      musb_setbits16(MUSB_TXCSR,
                                     MUSB_TXCSR_CLRDATATOG);
                    }
                  else
                    {
                      /* Unstall race: host may send first post-unstall
                       * packet anytime; preserve RXPKTRDY across RMW.
                       */

                      musb_rxcsr_clrbits_safe_bar(
                                     MUSB_RXCSR_SENDSTALL |
                                     MUSB_RXCSR_SENTSTALL);
                      musb_rxcsr_setbits_safe_bar(
                                     MUSB_RXCSR_CLRDATATOG);
                    }
                }

              t113_ep_select(0);
              musb_putreg16(MUSB_CSR0_SVDRXPKTRDY | MUSB_CSR0_DATAEND,
                            MUSB_CSR0);
              priv->ep0state = EP0STATE_IDLE;
              handled = true;
            }
        }
        break;

      case USB_REQ_SETFEATURE:
        {
          if ((ctrl->type & USB_REQ_RECIPIENT_MASK) ==
              USB_REQ_RECIPIENT_ENDPOINT &&
              value == USB_FEATURE_ENDPOINTHALT)
            {
              uint8_t epno = USB_EPNO(GETUINT16(ctrl->index));
              bool is_in   = USB_ISEPIN(GETUINT16(ctrl->index));

              if (epno > 0 && epno < T113_NPHYSEP)
                {
                  int idx = is_in ? T113_EPPHYIN(epno) :
                                    T113_EPPHYOUT(epno);
                  struct t113_ep_s *privep = &priv->eplist[idx];

                  privep->stalled = true;
                  t113_ep_select(epno);

                  if (is_in)
                    {
                      musb_setbits16(MUSB_TXCSR,
                                     MUSB_TXCSR_SENDSTALL);
                    }
                  else
                    {
                      musb_setbits16(MUSB_RXCSR,
                                     MUSB_RXCSR_SENDSTALL);
                    }
                }

              t113_ep_select(0);
              musb_putreg16(MUSB_CSR0_SVDRXPKTRDY | MUSB_CSR0_DATAEND,
                            MUSB_CSR0);
              priv->ep0state = EP0STATE_IDLE;
              handled = true;
            }
        }
        break;

      default:
        break;
    }

  return handled;
}

/****************************************************************************
 * Name: t113_ep0_setup
 *
 * Description:
 *   Handle EP0 SETUP packet reception.
 *
 ****************************************************************************/

static void t113_ep0_setup(struct t113_usbdev_s *priv)
{
  struct usb_ctrlreq_s ctrl;
  uint16_t csr0;

  t113_ep_select(0);
  csr0 = musb_getreg16(MUSB_CSR0);
  (void)musb_getreg16(MUSB_RXCOUNT);

  if (csr0 & MUSB_CSR0_SENTSTALL)
    {
      musb_putreg16(0, MUSB_CSR0);
      priv->ep0state = EP0STATE_IDLE;
      return;
    }

  if (csr0 & MUSB_CSR0_SETUPEND)
    {
      musb_putreg16(MUSB_CSR0_SVDSETUPEND, MUSB_CSR0);
      priv->ep0state = EP0STATE_IDLE;
    }

  if (!(csr0 & MUSB_CSR0_RXPKTRDY))
    {
      /* No SETUP packet, handle ongoing transfers */

      switch (priv->ep0state)
        {
          case EP0STATE_DATA_IN:
            t113_ep0_indone(priv);
            break;

          case EP0STATE_DATA_OUT:
            t113_ep0_outdone(priv);
            break;

          case EP0STATE_WAIT_STATUS_IN:

            /* Status phase complete, apply pending address if needed */

            if (priv->paddrset)
              {
                musb_writeb(priv->paddr, MUSB_FADDR);
                priv->paddrset = false;
                usb_trace_info("FADDR set to %d\n", priv->paddr);
              }

            priv->ep0state = EP0STATE_IDLE;
            break;

          case EP0STATE_WAIT_STATUS_OUT:
            priv->ep0state = EP0STATE_IDLE;
            break;

          default:
            break;
        }

      return;
    }

  t113_fifo_read(0, (uint8_t *)&ctrl, USB_SIZEOF_CTRLREQ);

  memcpy(&priv->ep0ctrl, &ctrl, USB_SIZEOF_CTRLREQ);

  priv->ep0datlen = 0;
  priv->ep0reqlen = GETUINT16(ctrl.len);

  /* Try standard request handling first */

  if (t113_ep0_stdrequest(priv, &ctrl))
    {
      return;
    }

  /* Service RXPKTRDY for class driver dispatch */

  musb_putreg16(MUSB_CSR0_SVDRXPKTRDY, MUSB_CSR0);

  /* Not a standard request, dispatch to class driver */

  if (USB_REQ_ISOUT(ctrl.type) && priv->ep0reqlen > 0)
    {
      /* OUT data phase expected */

      priv->ep0state = EP0STATE_DATA_OUT;
    }
  else if (USB_REQ_ISIN(ctrl.type))
    {
      /* IN data phase expected */

      priv->ep0state = EP0STATE_SETUP_IN;
    }
  else
    {
      /* No data phase */

      priv->ep0state = EP0STATE_SETUP_OUT;
    }

  t113_ep0_dispatch(priv);
}

/****************************************************************************
 * Name: t113_ep0_indone
 *
 * Description:
 *   EP0 IN transfer completed (previous TXPKTRDY was sent).
 *   Continue sending data if more is available.
 *
 ****************************************************************************/

static void t113_ep0_indone(struct t113_usbdev_s *priv)
{
  struct t113_ep_s *ep0in = &priv->eplist[T113_EP0_IN];
  struct t113_req_s *privreq;

  /* Check if there's a queued request */

  privreq = t113_rqpeek(ep0in);
  if (privreq != NULL)
    {
      uint16_t remaining = privreq->req.len - privreq->req.xfrd;

      if (remaining > 0 && priv->ep0reqlen > 0)
        {
          uint16_t xfrlen = remaining;
          if (xfrlen > EP0_MAXPACKET)
            {
              xfrlen = EP0_MAXPACKET;
            }

          if (xfrlen > priv->ep0reqlen)
            {
              xfrlen = priv->ep0reqlen;
            }

          t113_ep_select(0);
          t113_fifo_write(0,
                          privreq->req.buf + privreq->req.xfrd,
                          xfrlen);
          privreq->req.xfrd += xfrlen;
          priv->ep0reqlen -= xfrlen;

          if (xfrlen < EP0_MAXPACKET || privreq->req.xfrd >=
              privreq->req.len || priv->ep0reqlen == 0)
            {
              musb_putreg16(MUSB_CSR0_TXPKTRDY | MUSB_CSR0_DATAEND,
                            MUSB_CSR0);
              priv->ep0state = EP0STATE_WAIT_STATUS_OUT;

              /* Complete the request */

              privreq = t113_rqdequeue(ep0in);
              if (privreq != NULL)
                {
                  t113_reqcomplete(ep0in, privreq, OK);
                }
            }
          else
            {
              musb_putreg16(MUSB_CSR0_TXPKTRDY, MUSB_CSR0);
            }
        }
      else
        {
          /* No more data, complete the request */

          t113_ep_select(0);
          musb_putreg16(MUSB_CSR0_TXPKTRDY | MUSB_CSR0_DATAEND,
                        MUSB_CSR0);
          priv->ep0state = EP0STATE_WAIT_STATUS_OUT;

          privreq = t113_rqdequeue(ep0in);
          if (privreq != NULL)
            {
              t113_reqcomplete(ep0in, privreq, OK);
            }
        }
    }
  else
    {
      /* No request queued, short write from ep0buf completed */

      if (priv->ep0datlen > 0 && priv->ep0reqlen > 0)
        {
          t113_ep0_transmit(priv,
                            priv->ep0buf + (EP0_MAXPACKET -
                            priv->ep0datlen),
                            priv->ep0datlen);
        }
      else
        {
          priv->ep0state = EP0STATE_WAIT_STATUS_OUT;
        }
    }
}

/****************************************************************************
 * Name: t113_ep0_outdone
 *
 * Description:
 *   EP0 OUT data received.
 *
 ****************************************************************************/

static void t113_ep0_outdone(struct t113_usbdev_s *priv)
{
  struct t113_ep_s *ep0out = &priv->eplist[T113_EP0_OUT];
  struct t113_req_s *privreq;
  uint16_t rxcount;

  t113_ep_select(0);
  rxcount = musb_getreg16(MUSB_RXCOUNT);

  privreq = t113_rqpeek(ep0out);
  if (privreq != NULL)
    {
      uint16_t xfrlen = rxcount;
      uint16_t remaining = privreq->req.len - privreq->req.xfrd;

      if (xfrlen > remaining)
        {
          xfrlen = remaining;
        }

      t113_fifo_read(0, privreq->req.buf + privreq->req.xfrd, xfrlen);
      privreq->req.xfrd += xfrlen;

      if (privreq->req.xfrd >= privreq->req.len ||
          xfrlen < EP0_MAXPACKET)
        {
          /* Transfer complete */

          musb_putreg16(MUSB_CSR0_SVDRXPKTRDY | MUSB_CSR0_DATAEND,
                        MUSB_CSR0);
          priv->ep0state = EP0STATE_WAIT_STATUS_IN;

          privreq = t113_rqdequeue(ep0out);
          if (privreq != NULL)
            {
              t113_reqcomplete(ep0out, privreq, OK);
            }
        }
      else
        {
          musb_putreg16(MUSB_CSR0_SVDRXPKTRDY, MUSB_CSR0);
        }
    }
  else
    {
      /* No request, read into ep0buf */

      uint16_t xfrlen = rxcount;
      if (xfrlen > EP0_MAXPACKET)
        {
          xfrlen = EP0_MAXPACKET;
        }

      t113_fifo_read(0, priv->ep0buf, xfrlen);
      priv->ep0datlen = xfrlen;

      musb_putreg16(MUSB_CSR0_SVDRXPKTRDY | MUSB_CSR0_DATAEND,
                    MUSB_CSR0);
      priv->ep0state = EP0STATE_WAIT_STATUS_IN;

      /* Re-dispatch with data */

      t113_ep0_dispatch(priv);
    }
}

#ifdef CONFIG_T113_DMA

/****************************************************************************
 * MUSB internal DMA (Allwinner block at OTG+0x500/0x540) - TX only.
 *
 * 8 channels, 1:1 to EPs.  Each channel programmed by writing ADDR,
 * COUNT, then CFG|START.  Completion raises bit n in MUSB_DMA_INTR_STATUS
 * (W1C) and asserts the shared USB IRQ line.  VEND0 BUS_SEL=1 routes
 * the target EP's FIFO to the DMA bus for the duration of the transfer;
 * PIO access requires BUS_SEL=0.
 ****************************************************************************/

struct t113_musb_dma_ch_s
{
  struct t113_ep_s *owner;
};

static struct t113_musb_dma_ch_s g_musb_dma_chan[MUSB_DMA_AW_NCHAN];

/* Per-direction count of internal-DMA transfers currently in flight.
 * VEND0 is a single shared register (BUS_SEL + DRQ_SEL field).  Today
 * only one direction at a time uses internal DMA (bulk MSC alternates
 * IN / OUT on the host side), so tx+rx totals stay <= 1.  Split
 * counters let the TX and RX done paths decrement independently, and
 * an assertion enforces the single-in-flight invariant.
 */

static uint8_t g_musb_dma_tx_inflight;

static uint8_t g_musb_dma_rx_inflight;

static int t113_musb_dma_request(struct t113_ep_s *privep)
{
  int i;
  for (i = 0; i < MUSB_DMA_AW_NCHAN; i++)
    {
      if (g_musb_dma_chan[i].owner == NULL)
        {
          g_musb_dma_chan[i].owner = privep;
          return i;
        }
    }

  return -1;
}

static void t113_musb_dma_release(int ch)
{
  if (ch >= 0 && ch < MUSB_DMA_AW_NCHAN)
    {
      putreg32(0, MUSB_BASE + MUSB_DMA_CFG_AW(ch));
      modifyreg32(MUSB_BASE + MUSB_DMA_INTR_EN, 1u << ch, 0);
      putreg32(1u << ch, MUSB_BASE + MUSB_DMA_INTR_STATUS);
      g_musb_dma_chan[ch].owner = NULL;
    }
}

/* VEND0 programming helpers.
 *
 * BUS_SEL (bit0) is a GLOBAL enable: it routes the EP FIFO subsystem to the
 * internal-DMA bus (1) vs the CPU/PIO bus (0).  It is held =1 for the whole
 * time internal DMA is in use and is NOT a per-direction selector -- the
 * actual per-transfer routing is done by each DMA channel's own CFG word
 * (DIR bit4 + EP bits3:0 at 0x540+ch*0x10), so a TX channel and an RX
 * channel self-route independently and run concurrently (full duplex).
 * This is the canonical Mentor HSDMA model (per-channel CONTROL, no global
 * serializer).
 *
 * DRQ_SEL (bits1+) is a single legacy index, redundant for routing once the
 * per-channel CFG carries DIR+EP.  We still OR it in at arm for parity with
 * the vendor formula, but the completion path NEVER clears BUS_SEL while DMA
 * is in use -- clearing it to PIO would starve a concurrent sibling
 * direction's DMA (the root cause of the earlier "DMA moves zero bytes"
 * regression was exactly BUS_SEL getting cleared).
 *
 *     TX EP n: DRQ_SEL = (n-1) * 2
 *     RX EP n: DRQ_SEL = (n-1) * 2 + 1
 */

static inline uint8_t t113_vend0_drq_tx(uint8_t epphy)
{
  return (uint8_t)(((epphy - 1) * 2) << MUSB_VEND0_DRQ_SHIFT);
}

static inline uint8_t t113_vend0_drq_rx(uint8_t epphy)
{
  return (uint8_t)((((epphy - 1) * 2) + 1) << MUSB_VEND0_DRQ_SHIFT);
}

static uint8_t t113_musb_vend0_tx(uint8_t epphy)
{
  uint8_t v;

  DEBUGASSERT(epphy >= 1 && epphy <= T113_NPHYSEP);
  v  = getreg8(MUSB_BASE + MUSB_VEND0);
  v |= t113_vend0_drq_tx(epphy) | MUSB_VEND0_BUS_SEL;
  musb_writeb(v, MUSB_VEND0);
  return v;
}

static uint8_t t113_musb_vend0_rx(uint8_t epphy)
{
  uint8_t v;

  DEBUGASSERT(epphy >= 1 && epphy <= T113_NPHYSEP);
  v  = getreg8(MUSB_BASE + MUSB_VEND0);
  v |= t113_vend0_drq_rx(epphy) | MUSB_VEND0_BUS_SEL;
  musb_writeb(v, MUSB_VEND0);
  return v;
}

static void t113_musb_vend0_clear_tx(uint8_t epphy)
{
  uint8_t v;

  DEBUGASSERT(epphy >= 1 && epphy <= T113_NPHYSEP);

  /* Clear only this EP's TX DRQ index bits and KEEP BUS_SEL set.  Never drop
   * to PIO here: a concurrent RX DMA may be in flight, and clearing BUS_SEL
   * would starve the whole internal-DMA bus (zero-byte transfers).  BUS_SEL
   * is released only on full teardown (t113_musb_vend0_pio).
   */

  v  = getreg8(MUSB_BASE + MUSB_VEND0);
  v &= ~t113_vend0_drq_tx(epphy);
  v |= MUSB_VEND0_BUS_SEL;
  musb_writeb(v, MUSB_VEND0);
}

static void t113_musb_vend0_clear_rx(uint8_t epphy)
{
  uint8_t v;

  DEBUGASSERT(epphy >= 1 && epphy <= T113_NPHYSEP);

  /* Clear only this EP's RX DRQ index bits and KEEP BUS_SEL set (see
   * t113_musb_vend0_clear_tx).
   */

  v  = getreg8(MUSB_BASE + MUSB_VEND0);
  v &= ~t113_vend0_drq_rx(epphy);
  v |= MUSB_VEND0_BUS_SEL;
  musb_writeb(v, MUSB_VEND0);
}

/* Full teardown: return the FIFO subsystem to the CPU/PIO bus (BUS_SEL=0).
 * Only called on reset / ep-disable when no DMA is or will be in flight.
 */

static void t113_musb_vend0_pio(void)
{
  musb_writeb(0, MUSB_VEND0);
}

/* Kick a TX DMA from memory `buf` to the EP FIFO for `len` bytes.
 * Caller must already have programmed TXCSR with AUTOSET|MODE|
 * DMAENAB|DMAMODE and selected the correct EP index, and must hold
 * the USB spinlock.
 */

static void t113_musb_dma_tx_submit(struct t113_ep_s *privep,
                                    uintptr_t buf, uint32_t len)
{
  int ch = privep->musb_dma_ch;
  uint32_t cfg;

  /* Full duplex: TX and RX use independent DMA channels that self-route via
   * their own CFG word (DIR+EP), so a TX DMA and an RX DMA run concurrently.
   * BUS_SEL is held globally; only assert this direction is not already
   * mid-transfer on its own channel.
   */

  DEBUGASSERT(g_musb_dma_tx_inflight == 0);
  g_musb_dma_tx_inflight++;

  /* OR this EP's TX DRQ index in and ensure BUS_SEL stays set (global
   * FIFO->DMA-bus enable; never cleared while DMA is in use).
   */

  t113_musb_vend0_tx(privep->epphy);

  /* Clear stale channel IRQ + enable. */

  putreg32(1u << ch, MUSB_BASE + MUSB_DMA_INTR_STATUS);
  modifyreg32(MUSB_BASE + MUSB_DMA_INTR_EN, 0, 1u << ch);

  /* Program address, count, and kick off. */

  putreg32((uint32_t)buf, MUSB_BASE + MUSB_DMA_ADDR_AW(ch));
  putreg32(len, MUSB_BASE + MUSB_DMA_COUNT_AW(ch));

  cfg = ((privep->ep.maxpacket & 0x7ffu) << MUSB_DMA_CFG_AW_BURST_SHIFT)
      | MUSB_DMA_CFG_AW_DIR_TX
      | (privep->epphy & MUSB_DMA_CFG_AW_EP_MASK);

  putreg32(cfg | MUSB_DMA_CFG_AW_START, MUSB_BASE + MUSB_DMA_CFG_AW(ch));
}

/* Kick an RX DMA from the EP FIFO to memory `buf` for `len` bytes.
 * Caller must already have programmed RXCSR with AUTOCLEAR|DMAENAB|
 * DMAMODE (vendor 4-step sequence) and selected the correct EP
 * index, and must hold the USB spinlock.  The caller is also
 * responsible for cache maintenance on `buf` before calling.
 */

static void t113_musb_dma_rx_submit(struct t113_ep_s *privep,
                                    uintptr_t buf, uint32_t len)
{
  int ch = privep->musb_dma_ch_rx;
  uint32_t cfg;

  /* Full duplex: independent RX channel, concurrent with a TX DMA.  Assert
   * only this direction is not already mid-transfer on its own channel.
   */

  DEBUGASSERT(g_musb_dma_rx_inflight == 0);
  g_musb_dma_rx_inflight++;

  /* Mask this EP's INTRRX while DMA runs.  AUTOCLEAR drains packets
   * automatically, but each packet still pulses RXPKTRDY and fires
   * INTRRX - without this mask the ISR re-enters t113_epn_rxready
   * mid-DMA, hits the dma_busy branch, and aborts the channel.
   * Vendor `usbc_ep_intr_disable(ep, 0)`: clear INTRRXE + W1C INTRRX.
   * Use barriered writes so the INTRRXE update is globally visible
   * before this ISR returns (next IRQ would otherwise see stale mask).
   */

  musb_clrbits16(MUSB_INTRRXE, 1u << privep->epphy);
  musb_putreg16(1u << privep->epphy, MUSB_INTRRX);

  /* Route FIFO to the DMA bus for this transaction. */

  t113_musb_vend0_rx(privep->epphy);

  /* Clear stale channel IRQ + enable. */

  putreg32(1u << ch, MUSB_BASE + MUSB_DMA_INTR_STATUS);
  modifyreg32(MUSB_BASE + MUSB_DMA_INTR_EN, 0, 1u << ch);

  /* Program address, count, and kick off. */

  putreg32((uint32_t)buf, MUSB_BASE + MUSB_DMA_ADDR_AW(ch));
  putreg32(len, MUSB_BASE + MUSB_DMA_COUNT_AW(ch));

  cfg = ((privep->ep.maxpacket & 0x7ffu) << MUSB_DMA_CFG_AW_BURST_SHIFT)
      | MUSB_DMA_CFG_AW_DIR_RX
      | (privep->epphy & MUSB_DMA_CFG_AW_EP_MASK);

  putreg32(cfg | MUSB_DMA_CFG_AW_START, MUSB_BASE + MUSB_DMA_CFG_AW(ch));
}

/* Forward decl - defined below. */

static void t113_musb_dma_tx_done(struct t113_usbdev_s *priv,
                                  struct t113_ep_s *privep);
static void t113_musb_dma_rx_done(struct t113_usbdev_s *priv,
                                  struct t113_ep_s *privep);

/* Service all pending MUSB DMA channel IRQs; called under priv->lock
 * from the main USB ISR.  Returns non-zero if any channel serviced.
 */

static int t113_musb_dma_irq(struct t113_usbdev_s *priv)
{
  uint32_t status;
  int ch;
  int serviced = 0;

  status = getreg32(MUSB_BASE + MUSB_DMA_INTR_STATUS);
  status &= getreg32(MUSB_BASE + MUSB_DMA_INTR_EN);
  if (status == 0)
    {
      return 0;
    }

  for (ch = 0; ch < MUSB_DMA_AW_NCHAN; ch++)
    {
      struct t113_ep_s *privep;

      if ((status & (1u << ch)) == 0)
        {
          continue;
        }

      /* W1C this channel's IRQ bit. */

      putreg32(1u << ch, MUSB_BASE + MUSB_DMA_INTR_STATUS);
      serviced |= (1 << ch);

      privep = g_musb_dma_chan[ch].owner;
      if (privep == NULL || !privep->dma_busy)
        {
          continue;
        }

      /* RX path: no FIFO drain needed (DMA already pulled from FIFO).
       * Just stop the channel, hand off to rx_done for bookkeeping.
       */

      if (!privep->is_in)
        {
          putreg32(0, MUSB_BASE + MUSB_DMA_CFG_AW(ch));
          modifyreg32(MUSB_BASE + MUSB_DMA_INTR_EN, 1u << ch, 0);
          t113_musb_dma_rx_done(priv, privep);
          continue;
        }

      /* Wait for the FIFO to drain the last packet on the wire.
       * 1us yield between TXCSR reads avoids contending with the
       * controller's indexed-register port while it processes the
       * draining packet (empirically gives ~30% throughput lift over
       * a tight while-loop spin on this MUSB IP).  No timeout abort:
       * the FIFO always drains in bounded time at HS line rate.
       */

      t113_ep_select_fast(privep->epphy);
      while (musb_getreg16_fast(MUSB_TXCSR) &
             (MUSB_TXCSR_TXPKTRDY | MUSB_TXCSR_FIFONOTEMPTY))
        {
          up_udelay(1);
        }

      putreg32(0, MUSB_BASE + MUSB_DMA_CFG_AW(ch));
      modifyreg32(MUSB_BASE + MUSB_DMA_INTR_EN, 1u << ch, 0);

      t113_musb_dma_tx_done(priv, privep);
      continue;
    }

  return serviced;
}

/* Completion path after the FIFO has drained.  Equivalent to the
 * external-DMAC tx_callback: updates xfrd, clears CSR DMA bits,
 * restores VEND0=0 for concurrent PIO/external-DMAC FIFO access
 * (RX path), and advances the queue.
 */

static void t113_musb_dma_tx_done(struct t113_usbdev_s *priv,
                                  struct t113_ep_s *privep)
{
  struct t113_req_s *privreq;
  uint16_t csr;

  usb_trace_endex("dma_active");

  privreq = t113_rqpeek(privep);
  if (privreq != NULL)
    {
      privreq->req.xfrd += privep->dma_len;
    }

  privep->dma_len  = 0;
  privep->dma_busy = false;

  t113_ep_select_fast(privep->epphy);
  csr = musb_getreg16_fast(MUSB_TXCSR);
  csr &= ~(MUSB_TXCSR_AUTOSET | MUSB_TXCSR_DMAENAB);
  musb_putreg16_fast(csr, MUSB_TXCSR);
  csr &= ~MUSB_TXCSR_DMAMODE;
  musb_putreg16_fast(csr, MUSB_TXCSR);

  /* Restore FIFO to PIO bus so the RX external-DMAC path and any
   * PIO FIFO access work correctly.  The next TX DMA submit will
   * re-program VEND0 as needed.
   */

  t113_musb_vend0_clear_tx(privep->epphy);
  DEBUGASSERT(g_musb_dma_tx_inflight > 0);
  g_musb_dma_tx_inflight--;

  if (privreq == NULL)
    {
      return;
    }

  if (privreq->req.xfrd < privreq->req.len)
    {
      /* Residual bytes - kick txstart for PIO tail or next DMA */

      t113_epn_txstart(privep);
    }
  else
    {
      /* Full request done - check ZLP */

      bool need_zlp =
        (privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) &&
        (privreq->req.len > 0) &&
        (privreq->req.len % privep->ep.maxpacket == 0);

      if (need_zlp)
        {
          privreq->req.flags &= ~USBDEV_REQFLAGS_NULLPKT;
          musb_putreg16_fast(MUSB_TXCSR_MODE | MUSB_TXCSR_TXPKTRDY,
                             MUSB_TXCSR);
        }
      else
        {
          usb_trace_beginex("reqcomplete");
          privreq = t113_rqdequeue(privep);
          if (privreq != NULL)
            {
              t113_reqcomplete(privep, privreq, OK);
            }

          usb_trace_endex("reqcomplete");

          /* Start next queued request */

          t113_epn_txstart(privep);
        }
    }
}

/* Completion path for an RX MUSB internal-DMA transfer.  Called from
 * the USB ISR after the channel's DMA_INTR_STATUS bit fired.  The
 * DMA has already pulled `dma_len` bytes (all of which are
 * maxpacket-aligned full packets; short packets were filtered out
 * at submit time).  AUTOCLEAR cleared RXPKTRDY for every full packet
 * so no manual drain is required.  Remaining work:
 *   - clear DMA bits in RXCSR
 *   - clear VEND0, decrement the in-flight counter
 *   - invalidate dcache for the written region
 *   - advance req->xfrd, complete the request if it's full
 *   - if RXPKTRDY is now set (short packet arrived while DMA was
 *     running), re-enter t113_epn_rxready so the PIO fallback
 *     drains it on the next ISR pass
 */

static void t113_musb_dma_rx_done(struct t113_usbdev_s *priv,
                                  struct t113_ep_s *privep)
{
  struct t113_req_s *privreq;
  uint16_t csr;
  uint32_t dma_len;

  t113_ep_select_fast(privep->epphy);
  csr = musb_getreg16_fast(MUSB_RXCSR);
  csr &= ~(MUSB_RXCSR_DMAENAB | MUSB_RXCSR_AUTOCLEAR |
           MUSB_RXCSR_DMAMODE);

  /* RXPKTRDY is W0C in device mode: writing 1 has no effect, writing 0
   * explicitly clears it.  Between the read above and the write below a
   * new packet may have arrived and set RXPKTRDY in hardware - writing
   * the stale read value back with bit 0 = 0 would clobber it, leaving
   * the packet stranded in the FIFO with no further INTRRX edge.  OR it
   * back to 1 so the write is a no-op for that bit regardless of the hw
   * state at write time.  Same protection applied to other status bits
   * below that could race with hardware updates during the RMW window.
   */

  csr |= MUSB_RXCSR_RXPKTRDY;
  musb_putreg16_fast(csr, MUSB_RXCSR);

  t113_musb_vend0_clear_rx(privep->epphy);
  DEBUGASSERT(g_musb_dma_rx_inflight > 0);
  g_musb_dma_rx_inflight--;

  /* Re-enable INTRRX for this EP - vendor pattern, mirrors the
   * mask applied in t113_musb_dma_rx_submit.  Any RXPKTRDY pending
   * after DMA done (short packet) will now fire INTRRX for PIO.
   */

  musb_setbits16(MUSB_INTRRXE, 1u << privep->epphy);

  dma_len = privep->dma_len;
  privep->dma_len  = 0;
  privep->dma_busy = false;

  privreq = t113_rqpeek(privep);
  if (privreq != NULL && dma_len > 0)
    {
      uintptr_t start = (uintptr_t)(privreq->req.buf + privreq->req.xfrd);
      up_invalidate_dcache(start, start + dma_len);
      privreq->req.xfrd += dma_len;
    }

  /* If the full request is satisfied, complete it. */

  if (privreq != NULL && privreq->req.xfrd >= privreq->req.len)
    {
      privreq = t113_rqdequeue(privep);
      if (privreq != NULL)
        {
          t113_reqcomplete(privep, privreq, OK);
        }
    }

  /* If RXPKTRDY is set (next packet already in FIFO - typically a
   * CBW between BOT data phases), drain it here.  The rxintr loop
   * above us in the same ISR already W1C-cleared this EP's INTRRX
   * bit when the packet was latched; no future edge will fire for
   * it, so re-enter t113_epn_rxready now to avoid a deadlock.
   */

  csr = musb_getreg16_fast(MUSB_RXCSR);
  if (csr & MUSB_RXCSR_RXPKTRDY)
    {
      t113_epn_rxready(priv, privep->epphy);
    }
}

#endif /* CONFIG_T113_DMA */

/****************************************************************************
 * Name: t113_epn_txstart
 *
 * Description:
 *   Start a TX transfer on a bulk/interrupt IN endpoint.
 *   Uses external DMAC for large transfers, PIO for small ones.
 *
 ****************************************************************************/

static void t113_epn_txstart(struct t113_ep_s *privep)
{
  struct t113_req_s *privreq;
  uint32_t xfrlen;
  uint16_t pkt;

  privreq = t113_rqpeek(privep);
  if (privreq == NULL)
    {
      return;
    }

  xfrlen = privreq->req.len - privreq->req.xfrd;

#ifdef CONFIG_T113_DMA
  /* Use MUSB internal DMA (OTG+0x500/0x540) for large IN transfers.  TX and
   * RX have independent self-routing DMA channels, so a TX DMA may run while
   * an RX DMA is in flight (full duplex).  Only guard against re-arming this
   * same EP's channel while it is still busy.
   */

  if (xfrlen >= T113_TX_DMA_THRESHOLD && privep->musb_dma_ch >= 0 &&
      !privep->dma_busy && g_musb_dma_tx_inflight == 0)
    {
      uintptr_t buf = (uintptr_t)(privreq->req.buf + privreq->req.xfrd);

      /* Mode 1 + AUTOSET only autoloads TXPKTRDY on full maxpacket
       * boundaries.  A short tail (xfrlen % maxpacket != 0) leaves
       * the residual bytes stuck in the FIFO with FIFONOTEMPTY=1 and
       * no TXPKTRDY, hangs dma_irq's drain spin and wedges the EP.
       * Floor the DMA length to a maxpacket multiple; the residual
       * is picked up by PIO on the next t113_epn_txstart call from
       * dma_tx_done -> txstart (xfrd < req.len path).
       */

      uint32_t dma_len = (xfrlen / privep->ep.maxpacket) *
                         privep->ep.maxpacket;
      if (dma_len < T113_TX_DMA_THRESHOLD)
        {
          /* Residual after flooring is below threshold - fall through
           * to PIO so we don't waste a DMA setup on a sub-1KB run.
           */

          goto pio_fallback;
        }

      t113_ep_select_fast(privep->epphy);

      /* Wait for previous packet to be sent before arming DMA */

      if (musb_getreg16_fast(MUSB_TXCSR) & MUSB_TXCSR_TXPKTRDY)
        {
          return;
        }

      usb_trace_beginex("dma_cflush");
      up_flush_dcache(buf, buf + dma_len);
      usb_trace_endex("dma_cflush");

      /* Set TXCSR: MODE + DMAENAB + AUTOSET + DMAMODE.
       * AUTOSET autoloads TXPKTRDY on each full packet; DMAMODE=1
       * suppresses per-packet TX IRQ for the internal-DMA path.
       */

      musb_putreg16_fast(MUSB_TXCSR_MODE | MUSB_TXCSR_DMAENAB |
                         MUSB_TXCSR_AUTOSET | MUSB_TXCSR_DMAMODE,
                         MUSB_TXCSR);

      privep->dma_busy = true;
      privep->dma_len  = dma_len;

      usb_trace_beginex("dma_active");
      t113_musb_dma_tx_submit(privep, buf, dma_len);
      return;
    }

pio_fallback:
  ;
#endif /* CONFIG_T113_DMA */

  /* PIO fallback for small transfers or when DMA is busy */

  pkt = xfrlen;
  if (pkt > privep->ep.maxpacket)
    {
      pkt = privep->ep.maxpacket;
    }

  t113_ep_select_fast(privep->epphy);

  if (musb_getreg16_fast(MUSB_TXCSR) & MUSB_TXCSR_TXPKTRDY)
    {
      return;
    }

  usb_trace_beginex("fifo_write");
  t113_fifo_write_fast(privep->epphy,
                       privreq->req.buf + privreq->req.xfrd,
                       pkt);
  usb_trace_endex("fifo_write");
  privreq->req.xfrd += pkt;

  musb_putreg16_fast(MUSB_TXCSR_MODE | MUSB_TXCSR_TXPKTRDY,
                     MUSB_TXCSR);
}

/****************************************************************************
 * Name: t113_epn_txdone
 *
 * Description:
 *   Handle TX complete interrupt for EPn.
 *
 ****************************************************************************/

static void t113_epn_txdone(struct t113_usbdev_s *priv, uint8_t epno)
{
  struct t113_ep_s *privep = &priv->eplist[T113_EPPHYIN(epno)];
  struct t113_req_s *privreq;
  uint16_t txcsr;

#ifdef CONFIG_T113_DMA
  /* If DMA is driving this EP, ignore TX interrupts -
   * AUTOSET handles TXPKTRDY, DMA callback handles completion.
   */

  if (privep->dma_busy)
    {
      return;
    }
#endif

  usb_trace_beginex("txdone");

  t113_ep_select_fast(epno);
  txcsr = musb_getreg16_fast(MUSB_TXCSR);

  if (txcsr & MUSB_TXCSR_SENTSTALL)
    {
      musb_clrbits16_fast(MUSB_TXCSR, MUSB_TXCSR_SENTSTALL |
                                       MUSB_TXCSR_SENDSTALL);
      musb_setbits16_fast(MUSB_TXCSR, MUSB_TXCSR_CLRDATATOG);
      usb_trace_endex("txdone");
      return;
    }

  if (txcsr & MUSB_TXCSR_UNDERRUN)
    {
      musb_clrbits16_fast(MUSB_TXCSR, MUSB_TXCSR_UNDERRUN);
    }

  privreq = t113_rqpeek(privep);
  if (privreq == NULL)
    {
      usb_trace_endex("txdone");
      return;
    }

  if (privreq->req.xfrd >= privreq->req.len)
    {
      bool need_zlp = (privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) &&
                      (privreq->req.len > 0) &&
                      (privreq->req.len % privep->ep.maxpacket == 0);

      if (need_zlp)
        {
          privreq->req.flags &= ~USBDEV_REQFLAGS_NULLPKT;
          musb_putreg16_fast(MUSB_TXCSR_MODE | MUSB_TXCSR_TXPKTRDY,
                             MUSB_TXCSR);
          usb_trace_endex("txdone");
          return;
        }

      usb_trace_beginex("reqcomplete");
      privreq = t113_rqdequeue(privep);
      if (privreq != NULL)
        {
          t113_reqcomplete(privep, privreq, OK);
        }

      usb_trace_endex("reqcomplete");
    }

  /* Send next packet immediately - avoid extra interrupt round-trip */

  t113_epn_txstart(privep);
  usb_trace_endex("txdone");
}

/****************************************************************************
 * Name: t113_epn_rxready
 *
 * Description:
 *   Handle RX ready interrupt for EPn.
 *
 ****************************************************************************/

static void t113_epn_rxready(struct t113_usbdev_s *priv, uint8_t epno)
{
  struct t113_ep_s *privep = &priv->eplist[T113_EPPHYOUT(epno)];
  struct t113_req_s *privreq;
  uint16_t rxcsr;
  uint16_t rxcount;
  int drain_budget = 64;

#ifdef CONFIG_T113_DMA
  if (privep->dma_busy)
    {
      /* DMA is still running on this EP.  INTRRXE is masked for the
       * EP during DMA, so this path is only reachable when INTRRX
       * was already snapshotted by the ISR in the same pass as the
       * DMA-done status bit.  Defer: the DMA IRQ handler runs after
       * the rxintr loop in this same ISR and will call rx_done
       * (which may recursively drain RXPKTRDY via PIO).  Nothing to
       * do here.
       */

      return;
    }
#endif

  t113_ep_select_fast(epno);

  rxcsr = musb_getreg16_fast(MUSB_RXCSR);

  if (rxcsr & MUSB_RXCSR_SENTSTALL)
    {
      /* Use rxcsr_*_safe so a new packet arriving during the RMW
       * window does not get its RXPKTRDY clobbered - same race that
       * drove the fix in t113_musb_dma_rx_done().
       */

      musb_rxcsr_clrbits_safe(MUSB_RXCSR_SENTSTALL |
                               MUSB_RXCSR_SENDSTALL);
      musb_rxcsr_setbits_safe(MUSB_RXCSR_CLRDATATOG);
      return;
    }

  if (rxcsr & MUSB_RXCSR_OVERRUN)
    {
      usb_trace_beginex("rx_overrun");
      usb_trace_endex("rx_overrun");
      musb_rxcsr_clrbits_safe(MUSB_RXCSR_OVERRUN);
    }

#ifdef CONFIG_T113_DMA
  /* Mode 1 RX DMA (MUSB internal DMA).  Program DMAC for a
   * maxpacket-aligned transfer.  AUTOCLEAR auto-clears RXPKTRDY for
   * full packets; short packets stay in the FIFO and trigger a
   * subsequent INTRRX (handled by the dma_busy early-abort branch
   * above).
   */

  if ((rxcsr & MUSB_RXCSR_RXPKTRDY) &&
      privep->musb_dma_ch_rx >= 0 && !privep->dma_busy &&
      g_musb_dma_rx_inflight == 0)
    {
      rxcount = musb_getreg16_fast(MUSB_RXCOUNT);
      privreq = t113_rqpeek(privep);

      /* Eligible for DMA only when the FIFO holds exactly one full
       * maxpacket (more packets to come) AND remaining request size
       * >= T113_RX_DMA_THRESHOLD.  The AUTOCLEAR mechanism transfers
       * full packets autonomously; short packets are left for PIO.
       */

      if (privreq != NULL &&
          rxcount == privep->ep.maxpacket &&
          (privreq->req.len - privreq->req.xfrd) >=
              T113_RX_DMA_THRESHOLD)
        {
          uintptr_t buf = (uintptr_t)(privreq->req.buf +
                                      privreq->req.xfrd);
          uint32_t dma_len = privep->ep.maxpacket;
          uint16_t csr;

          if (dma_len == 0)
            {
              goto pio_fallback;
            }

          /* 4-step RXCSR sequence for MUSB internal DMA:
           *   1. Set DMAMODE     2. Set AUTOCLEAR | DMAENAB
           *   3. Clear DMAMODE   4. Set DMAMODE
           * Final: AUTOCLEAR=1, DMAENAB=1, DMAMODE=1
           */

          csr  = musb_getreg16_fast(MUSB_RXCSR);
          csr |= MUSB_RXCSR_DMAMODE;
          musb_putreg16_fast(csr, MUSB_RXCSR);

          csr |= (MUSB_RXCSR_AUTOCLEAR | MUSB_RXCSR_DMAENAB);
          musb_putreg16_fast(csr, MUSB_RXCSR);

          csr &= ~MUSB_RXCSR_DMAMODE;
          musb_putreg16_fast(csr, MUSB_RXCSR);

          csr |= MUSB_RXCSR_DMAMODE;
          musb_putreg16_fast(csr, MUSB_RXCSR);

          /* Ensure the destination buffer is flushed/invalidated
           * before DMA writes into it.  The clean step pushes any
           * dirty lines out; after DMA completion we invalidate.
           */

          up_clean_dcache(buf, buf + dma_len);

          privep->dma_busy = true;
          privep->dma_len  = dma_len;
          t113_musb_dma_rx_submit(privep, buf, dma_len);
          return;
        }
    }
#endif

pio_fallback:
  while ((rxcsr & MUSB_RXCSR_RXPKTRDY) && drain_budget-- > 0)
    {
      rxcount = musb_getreg16_fast(MUSB_RXCOUNT);

      privreq = t113_rqpeek(privep);
      if (privreq == NULL)
        {
          /* No request queued - leave RXPKTRDY set and let the next
           * EP_SUBMIT drain the FIFO.  Clearing RXPKTRDY here silently
           * drops a real bulk-OUT packet when the class driver is between
           * requests (for example while MSC is writing a slow backend),
           * leaving BOT waiting forever for data that the host already
           * sent.  t113_epsubmit() explicitly checks RXPKTRDY after it
           * queues an OUT request, so preserving the packet provides USB
           * back-pressure instead of data loss.
           */

          usb_trace_beginex("rx_noreq");
          usb_trace_endex("rx_noreq");
          return;
        }

      uint16_t remaining = privreq->req.len - privreq->req.xfrd;
      uint16_t xfrlen = rxcount;

      if (xfrlen > remaining)
        {
          xfrlen = remaining;
        }

      usb_trace_beginex("fifo_read");
      t113_fifo_read_fast(epno,
                           privreq->req.buf + privreq->req.xfrd,
                           xfrlen);
      usb_trace_endex("fifo_read");
      privreq->req.xfrd += xfrlen;

      musb_clrbits16_fast(MUSB_RXCSR, MUSB_RXCSR_RXPKTRDY);

      if (xfrlen < privep->ep.maxpacket ||
          privreq->req.xfrd >= privreq->req.len)
        {
          privreq = t113_rqdequeue(privep);
          if (privreq != NULL)
            {
              t113_reqcomplete(privep, privreq, OK);
            }
        }

      rxcsr = musb_getreg16_fast(MUSB_RXCSR);
    }
}

/****************************************************************************
 * Name: t113_musb_reset
 *
 * Description:
 *   Handle USB bus reset.
 *
 ****************************************************************************/

static void t113_musb_reset(struct t113_usbdev_s *priv)
{
  int i;

  t113_ep_select(0);
  musb_writeb(0, MUSB_FADDR);
  musb_putreg16(MUSB_CSR0_FLUSHFIFO, MUSB_CSR0);
  musb_putreg16(MUSB_CSR0_SVDSETUPEND | MUSB_CSR0_SVDRXPKTRDY,
                MUSB_CSR0);

  priv->paddr = 0;
  priv->paddrset = false;
  priv->ep0state = EP0STATE_IDLE;
  priv->ep0datlen = 0;
  priv->ep0reqlen = 0;

  for (i = 0; i < T113_NLOGEP; i++)
    {
#ifdef CONFIG_T113_DMA
      /* Abort any MUSB internal DMA in flight on this EP. */

      if (priv->eplist[i].dma_busy)
        {
          struct t113_ep_s *ep = &priv->eplist[i];
          int ch = ep->is_in ? ep->musb_dma_ch : ep->musb_dma_ch_rx;
          if (ch >= 0)
            {
              putreg32(0, MUSB_BASE + MUSB_DMA_CFG_AW(ch));
              modifyreg32(MUSB_BASE + MUSB_DMA_INTR_EN, 1u << ch, 0);
              putreg32(1u << ch, MUSB_BASE + MUSB_DMA_INTR_STATUS);
            }

          if (ep->is_in && g_musb_dma_tx_inflight > 0)
            {
              g_musb_dma_tx_inflight--;
            }

          if (!ep->is_in && g_musb_dma_rx_inflight > 0)
            {
              g_musb_dma_rx_inflight--;
            }

          ep->dma_busy = false;
          ep->dma_len  = 0;
          t113_musb_vend0_pio();
        }

      /* Always clear lingering DMA bits in RXCSR/TXCSR and flush the
       * EP FIFO on reset.  Without this, a RX DMA that was mid-flight
       * when the host issued the reset leaves RXCSR with
       * DMAENAB|DMAMODE|AUTOCLEAR set.  After re-enumeration the first
       * packet into that FIFO confuses the controller (packets
       * drained autonomously but we never arm a new DMA, so xfrd
       * never advances and the class driver stalls waiting for a
       * callback that never fires).  This matches the observed
       * "second bulk-OUT transaction hangs after first USB reset"
       * pattern.
       */

      if (priv->eplist[i].epphy != 0)
        {
          t113_ep_select(priv->eplist[i].epphy);
          if (priv->eplist[i].is_in)
            {
              uint16_t txcsr = musb_getreg16(MUSB_TXCSR);
              txcsr &= ~(MUSB_TXCSR_DMAENAB | MUSB_TXCSR_AUTOSET |
                         MUSB_TXCSR_DMAMODE);
              musb_putreg16(txcsr | MUSB_TXCSR_FLUSHFIFO, MUSB_TXCSR);
              musb_putreg16(txcsr | MUSB_TXCSR_FLUSHFIFO, MUSB_TXCSR);
            }
          else
            {
              uint16_t rxcsr = musb_getreg16(MUSB_RXCSR);
              rxcsr &= ~(MUSB_RXCSR_DMAENAB | MUSB_RXCSR_AUTOCLEAR |
                         MUSB_RXCSR_DMAMODE);
              musb_putreg16(rxcsr | MUSB_RXCSR_FLUSHFIFO, MUSB_RXCSR);
              musb_putreg16(rxcsr | MUSB_RXCSR_FLUSHFIFO, MUSB_RXCSR);
            }
        }
#endif

#ifndef CONFIG_T113_DMA
      if (priv->eplist[i].epphy != 0)
        {
          t113_ep_select(priv->eplist[i].epphy);
          if (priv->eplist[i].is_in)
            {
              musb_putreg16(MUSB_TXCSR_FLUSHFIFO, MUSB_TXCSR);
              musb_putreg16(MUSB_TXCSR_FLUSHFIFO, MUSB_TXCSR);
            }
          else
            {
              musb_putreg16(MUSB_RXCSR_FLUSHFIFO, MUSB_RXCSR);
              musb_putreg16(MUSB_RXCSR_FLUSHFIFO, MUSB_RXCSR);
            }
        }
#endif

      t113_cancelrequests(&priv->eplist[i], -ECONNRESET);
    }

  if (musb_readb(MUSB_POWER) & MUSB_POWER_HSMODE)
    {
      priv->usbdev.speed = USB_SPEED_HIGH;
    }
  else
    {
      priv->usbdev.speed = USB_SPEED_FULL;
    }

  musb_putreg16(1, MUSB_INTRTXE);
  musb_writeb(MUSB_INTR_SUSPEND | MUSB_INTR_RESUME | MUSB_INTR_RESET,
              MUSB_INTRUSBE);

  if (priv->driver != NULL)
    {
      spin_unlock(&priv->lock);
      CLASS_DISCONNECT(priv->driver, &priv->usbdev);
      spin_lock(&priv->lock);
    }
}

/****************************************************************************
 * Name: t113_usbdev_interrupt
 *
 * Description:
 *   USB interrupt handler.
 *
 ****************************************************************************/

static int t113_usbdev_interrupt(int irq, void *context, void *arg)
{
  struct t113_usbdev_s *priv = (struct t113_usbdev_s *)arg;
  irqstate_t flags;
  uint8_t  usbintr;
  uint16_t txintr;
  uint16_t rxintr;
  uint16_t csr0;
  uint8_t  old_index;
  int      i;

  UNUSED(irq);
  UNUSED(context);

  /* Acquire the controller lock before any register access.
   * On SMP another core may be in EP_SUBMIT / epconfigure
   * concurrently modifying INDEX and enable-mask registers.
   *
   * All class-driver callbacks are invoked with the lock
   * temporarily dropped to avoid ABBA deadlocks with
   * class-driver locks (e.g. cdcacm.lock).
   */

  flags = spin_lock_irqsave(&priv->lock);

  /* Save current EP index (now protected by lock) */

  old_index = musb_readb(MUSB_INDEX);

  usbintr = musb_readb(MUSB_INTRUSB);
  txintr  = musb_getreg16(MUSB_INTRTX);
  rxintr  = musb_getreg16(MUSB_INTRRX);

  if (usbintr)
    {
      musb_writeb(usbintr, MUSB_INTRUSB);
    }

  usbintr &= musb_readb(MUSB_INTRUSBE);
  txintr  &= musb_getreg16(MUSB_INTRTXE);
  rxintr  &= musb_getreg16(MUSB_INTRRXE);

  /* Handle bus reset (highest priority) */

  if (usbintr & MUSB_INTR_RESET)
    {
      t113_musb_reset(priv);
    }

  /* Handle suspend */

  if (usbintr & MUSB_INTR_SUSPEND)
    {
      usb_trace_info("USB suspend\n");
      priv->suspended = true;
      if (priv->driver != NULL)
        {
          spin_unlock(&priv->lock);
          CLASS_SUSPEND(priv->driver, &priv->usbdev);
          spin_lock(&priv->lock);
        }
    }

  /* Handle resume */

  if (usbintr & MUSB_INTR_RESUME)
    {
      usb_trace_info("USB resume\n");
      priv->suspended = false;
      if (priv->driver != NULL)
        {
          spin_unlock(&priv->lock);
          CLASS_RESUME(priv->driver, &priv->usbdev);
          spin_lock(&priv->lock);
        }
    }

  /* Handle EP0 */

  t113_ep_select(0);
  csr0 = musb_getreg16(MUSB_CSR0);

  if ((csr0 & (MUSB_CSR0_RXPKTRDY | MUSB_CSR0_SENTSTALL |
               MUSB_CSR0_SETUPEND)) ||
      (priv->ep0state != EP0STATE_IDLE))
    {
      t113_ep0_setup(priv);
    }

  if (txintr & 1)
    {
      musb_putreg16(1, MUSB_INTRTX);
    }

  for (i = 1; i < T113_NPHYSEP; i++)
    {
      if (txintr & (1 << i))
        {
          musb_putreg16(1 << i, MUSB_INTRTX);
          t113_epn_txdone(priv, i);
        }
    }

  for (i = 1; i < T113_NPHYSEP; i++)
    {
      if (rxintr & (1 << i))
        {
          musb_putreg16(1 << i, MUSB_INTRRX);
          t113_epn_rxready(priv, i);
        }
    }

#ifdef CONFIG_T113_DMA
  /* MUSB internal DMA completion - shares the USB IRQ line. */

  t113_musb_dma_irq(priv);
#endif

  musb_writeb(old_index, MUSB_INDEX);

  t113_unlock_and_drain(priv, flags);
  return OK;
}

/****************************************************************************
 * Name: t113_ccu_init
 *
 * Description:
 *   Initialize USB clocks and deassert resets.
 *
 ****************************************************************************/

static void t113_ccu_init(void)
{
  uint32_t reg;

  reg = getreg32(T113_CCU_USB_BGR);
  reg &= ~(USB_BGR_OTG0_RST | USB_BGR_EHCI0_RST |
            USB_BGR_OHCI0_RST | USB_BGR_OTG0_GATING |
            USB_BGR_EHCI0_GATING | USB_BGR_OHCI0_GATING);
  putreg32(reg, T113_CCU_USB_BGR);

  up_mdelay(5);

  reg = getreg32(T113_CCU_USB0_CLK);
  reg |= USB0_CLK_PHYRST_DEASSERT;
  putreg32(reg, T113_CCU_USB0_CLK);

  reg = getreg32(T113_CCU_USB_BGR);
  reg |= USB_BGR_OTG0_RST | USB_BGR_OTG0_GATING;
  putreg32(reg, T113_CCU_USB_BGR);

  up_mdelay(2);
}

static int phy_vc_bit_offset(uint32_t mask)
{
  int i;

  for (i = 0; i < 32; i++)
    {
      if (mask & (1u << i))
        {
          return i;
        }
    }

  return 0;
}

static void phy_vc_write(int addr, int data, int len)
{
  uint32_t phyctl;
  int j;

  phyctl = phy_getreg32(USBPHY_PHYCTL28NM);
  phyctl |= USB_PHYCTL28NM_VC_EN;
  phy_putreg32(phyctl, USBPHY_PHYCTL28NM);

  for (j = 0; j < len; j++)
    {
      phyctl = phy_getreg32(USBPHY_PHYCTL28NM);
      phyctl &= ~USB_PHYCTL28NM_VC_CLK;
      phy_putreg32(phyctl, USBPHY_PHYCTL28NM);

      phyctl = phy_getreg32(USBPHY_PHYCTL28NM);
      phyctl &= ~USB_PHYCTL28NM_VC_ADDR;
      phyctl |= ((addr + j) << 8);
      phy_putreg32(phyctl, USBPHY_PHYCTL28NM);

      phyctl = phy_getreg32(USBPHY_PHYCTL28NM);
      phyctl &= ~USB_PHYCTL28NM_VC_DI;
      phyctl |= ((data & 0x01) << 7);
      phy_putreg32(phyctl, USBPHY_PHYCTL28NM);

      phyctl |= USB_PHYCTL28NM_VC_CLK;
      phy_putreg32(phyctl, USBPHY_PHYCTL28NM);

      phyctl &= ~USB_PHYCTL28NM_VC_CLK;
      phy_putreg32(phyctl, USBPHY_PHYCTL28NM);

      data >>= 1;
    }

  phyctl = phy_getreg32(USBPHY_PHYCTL28NM);
  phyctl &= ~USB_PHYCTL28NM_VC_EN;
  phy_putreg32(phyctl, USBPHY_PHYCTL28NM);
}

static void t113_phy_efuse_calibrate(void)
{
  uint32_t efuse;
  int mode;
  int res;
  int val;

  efuse = getreg32(USB_PHY_EFUSE_ADDR);
  if (!(efuse & USB_PHY_EFUSE_ADJUST))
    {
      return;
    }

  mode = (efuse & USB_PHY_EFUSE_MODE) ?
         USB_VCPHY_IREF_MODE : USB_VCPHY_VERF_MODE;
  phy_vc_write(USB_VCPHY_MODE, mode, 1);

  res = (efuse & USB_PHY_EFUSE_RES) >>
        phy_vc_bit_offset(USB_PHY_EFUSE_RES);
  phy_vc_write(0x43, 0x0, 1);
  phy_vc_write(0x41, 0x0, 1);
  phy_vc_write(0x40, 0x0, 1);
  phy_vc_write(USB_VCPHY_TRAN_SOFT_RES, res, 4);
  phy_vc_write(0x43, 0x1, 1);

  val = (efuse & USB_PHY_EFUSE_VERF_COMMON) >>
        phy_vc_bit_offset(USB_PHY_EFUSE_VERF_COMMON);
  if (mode == USB_VCPHY_VERF_MODE)
    {
      phy_vc_write(USB_VCPHY_COMM_VREF_RISE, val, 3);
    }
  else
    {
      phy_vc_write(USB_VCPHY_TRAN_IREF_RISE, val, 3);
    }

  phy_putreg32(USB_PHYCTL28NM_VBUSVLDEXT, USBPHY_PHYCTL28NM);
}

/****************************************************************************
 * Name: t113_phy_init
 *
 * Description:
 *   Initialize the USB PHY for device mode.
 *
 ****************************************************************************/

static void t113_phy_init(void)
{
  uint32_t reg;

  /* Clear change detect bits first */

  phy_clrbits32(USBPHY_ISCR, USB_ISCR_VBUS_CHANGE_DETECT |
                              USB_ISCR_ID_CHANGE_DETECT |
                              USB_ISCR_DPDM_CHANGE_DETECT);

  /* Force ID high (device mode) */

  reg = phy_getreg32(USBPHY_ISCR);
  reg &= ~USB_ISCR_FORCE_ID_MASK;
  reg |= USB_ISCR_FORCE_ID_HIGH;
  phy_putreg32(reg, USBPHY_ISCR);

  /* Force VBUS valid (always assume connected) */

  reg = phy_getreg32(USBPHY_ISCR);
  reg &= ~USB_ISCR_FORCE_VBUS_MASK;
  reg |= USB_ISCR_FORCE_VBUS_HIGH;
  phy_putreg32(reg, USBPHY_ISCR);

  phy_setbits32(USBPHY_ISCR,
                USB_ISCR_DPDM_PULLUP_EN | USB_ISCR_ID_PULLUP_EN);

  /* Clear change detect again */

  phy_clrbits32(USBPHY_ISCR, USB_ISCR_VBUS_CHANGE_DETECT |
                              USB_ISCR_ID_CHANGE_DETECT |
                              USB_ISCR_DPDM_CHANGE_DETECT);

  /* 28nm PHY: Clear SIDDQ to exit power-down */

  phy_clrbits32(USBPHY_PHYCTL28NM, USB_PHYCTL28NM_SIDDQ);

  /* Select OTG mode */

  phy_setbits32(USBPHY_PHYSEL, USB_PHYSEL_OTG_SEL);

  /* Initialise VEND0 to PIO bus mode (BUS_SEL=0).  The legacy DMA block
   * at offset 0x200+ is non-functional on T113 (writes dropped, reads
   * zero), but the OTG+0x500/0x540 internal-DMA block IS functional: the
   * TX/RX DMA submit paths set VEND0 BUS_SEL per transaction to route the
   * EP FIFO onto the internal-DMA bus, and restore PIO on teardown.  PIO
   * is the correct power-on default before any DMA is armed.
   */

  musb_writeb(0, MUSB_VEND0);

  t113_phy_efuse_calibrate();

  up_mdelay(1);
}

/****************************************************************************
 * Name: t113_musb_init
 *
 * Description:
 *   Initialize the MUSB controller registers.
 *
 ****************************************************************************/

static void t113_musb_init(struct t113_usbdev_s *priv)
{
  int i;

  musb_writeb(0, MUSB_INTRUSBE);
  musb_putreg16(0, MUSB_INTRTXE);
  musb_putreg16(0, MUSB_INTRRXE);
  musb_writeb(0xff, MUSB_INTRUSB);
  musb_putreg16(0xffff, MUSB_INTRTX);
  musb_putreg16(0xffff, MUSB_INTRRX);
  musb_writeb(musb_readb(MUSB_POWER) &
              ~(MUSB_POWER_SOFTCONN | MUSB_POWER_HSENAB), MUSB_POWER);

  musb_writeb(0, MUSB_FADDR);

  for (i = 0; i < (int)NFIFOCONFIGS; i++)
    {
      const struct t113_fifoconfig_s *cfg = &g_fifoconfig[i];
      uint8_t fifosz;

      t113_ep_select(cfg->epno);

      if (cfg->epno == 0)
        {
          continue;
        }

      fifosz = cfg->fifosz;
      if (cfg->dpb)
        {
          fifosz |= 0x10;
        }

      if (cfg->is_in)
        {
          musb_putreg16(MUSB_TXCSR_FLUSHFIFO | MUSB_TXCSR_CLRDATATOG,
                        MUSB_TXCSR);
          musb_putreg16(MUSB_TXCSR_FLUSHFIFO | MUSB_TXCSR_CLRDATATOG,
                        MUSB_TXCSR);
          musb_putreg16(cfg->size, MUSB_TXMAXP);
          musb_writeb(fifosz, MUSB_TXFIFOSZ);
          musb_putreg16(FIFO_ADDR(cfg->addr), MUSB_TXFIFOADD);
        }
      else
        {
          musb_putreg16(MUSB_RXCSR_FLUSHFIFO | MUSB_RXCSR_CLRDATATOG,
                        MUSB_RXCSR);
          musb_putreg16(MUSB_RXCSR_FLUSHFIFO | MUSB_RXCSR_CLRDATATOG,
                        MUSB_RXCSR);
          musb_putreg16(cfg->size, MUSB_RXMAXP);
          musb_writeb(fifosz, MUSB_RXFIFOSZ);
          musb_putreg16(FIFO_ADDR(cfg->addr), MUSB_RXFIFOADD);
        }
    }

  t113_ep_select(0);

  priv->ep0state = EP0STATE_IDLE;
  priv->attached = true;
}

static void t113_musb_enable(void)
{
  uint8_t power;

  power = musb_readb(MUSB_POWER);
  power &= ~MUSB_POWER_ISOUPDATE;
  power |= MUSB_POWER_HSENAB;
  musb_writeb(power, MUSB_POWER);

  musb_writeb(MUSB_INTR_SUSPEND | MUSB_INTR_RESUME | MUSB_INTR_RESET,
              MUSB_INTRUSBE);
  musb_putreg16(1, MUSB_INTRTXE);
}

/****************************************************************************
 * Endpoint Operations
 ****************************************************************************/

/****************************************************************************
 * Name: t113_epconfigure
 *
 * Description:
 *   Configure an endpoint with the given descriptor.
 *
 ****************************************************************************/

static int t113_epconfigure(struct usbdev_ep_s *ep,
                            const struct usb_epdesc_s *desc,
                            bool last)
{
  struct t113_ep_s *privep = (struct t113_ep_s *)ep;
  uint16_t maxpacket;
  uint8_t  epno;
  bool     is_in;
  irqstate_t flags;

  DEBUGASSERT(ep != NULL && desc != NULL);

  epno = USB_EPNO(desc->addr);
  is_in = USB_ISEPIN(desc->addr);
  maxpacket = GETUINT16(desc->mxpacketsize);

  usb_trace_info("EP%d %s configure: maxpkt=%d type=%d\n",
                 epno, is_in ? "IN" : "OUT", maxpacket,
                 desc->attr & USB_EP_ATTR_XFERTYPE_MASK);

  flags = spin_lock_irqsave(&privep->dev->lock);

  ep->maxpacket = maxpacket;
  privep->eptype = desc->attr & USB_EP_ATTR_XFERTYPE_MASK;
  privep->stalled = false;

#ifdef CONFIG_T113_DMA
  privep->dma_busy = false;

  /* Allocate a MUSB internal-DMA channel for every bulk EP.
   * Direction is encoded in DRQ_SEL at submit time.
   */

  if (epno > 0 && privep->eptype == USB_EP_ATTR_XFER_BULK)
    {
      if (is_in && privep->musb_dma_ch < 0)
        {
          int ch = t113_musb_dma_request(privep);
          if (ch >= 0)
            {
              privep->musb_dma_ch = (int8_t)ch;
            }
        }

      if (!is_in && privep->musb_dma_ch_rx < 0)
        {
          int ch = t113_musb_dma_request(privep);
          if (ch >= 0)
            {
              privep->musb_dma_ch_rx = (int8_t)ch;
            }
        }
    }
#endif

  /* Configure the hardware endpoint */

  t113_ep_select(epno);

  if (is_in)
    {
      /* Set TX maxpacket */

      musb_putreg16(maxpacket, MUSB_TXMAXP);

      /* Clear and flush TX FIFO */

      musb_putreg16(MUSB_TXCSR_MODE | MUSB_TXCSR_CLRDATATOG |
                    MUSB_TXCSR_FLUSHFIFO, MUSB_TXCSR);
      musb_putreg16(MUSB_TXCSR_MODE | MUSB_TXCSR_CLRDATATOG |
                    MUSB_TXCSR_FLUSHFIFO, MUSB_TXCSR);

      /* Enable TX interrupt for this EP */

      musb_setbits16(MUSB_INTRTXE, (1 << epno));
    }
  else
    {
      /* Set RX maxpacket */

      musb_putreg16(maxpacket, MUSB_RXMAXP);

      /* Clear and flush RX FIFO */

      musb_putreg16(MUSB_RXCSR_CLRDATATOG | MUSB_RXCSR_FLUSHFIFO,
                    MUSB_RXCSR);
      musb_putreg16(MUSB_RXCSR_CLRDATATOG | MUSB_RXCSR_FLUSHFIFO,
                    MUSB_RXCSR);

      /* Enable RX interrupt for this EP */

      musb_setbits16(MUSB_INTRRXE, (1 << epno));
    }

  spin_unlock_irqrestore(&privep->dev->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: t113_epdisable
 *
 * Description:
 *   Disable an endpoint.
 *
 ****************************************************************************/

static int t113_epdisable(struct usbdev_ep_s *ep)
{
  struct t113_ep_s *privep = (struct t113_ep_s *)ep;
  irqstate_t flags;

  DEBUGASSERT(ep != NULL);

  usb_trace_info("EP%d %s disable\n",
                 privep->epphy, privep->is_in ? "IN" : "OUT");

  flags = spin_lock_irqsave(&privep->dev->lock);

  privep->stalled = true;

#ifdef CONFIG_T113_DMA
  /* Release MUSB internal DMA channels (both directions).  If a DMA
   * was in flight on this EP when the class driver tore it down, we
   * must also decrement the matching global inflight counter and
   * restore VEND0 to PIO once both counters hit zero - otherwise a
   * subsequent submit trips the single-in-flight DEBUGASSERT.  Mirror
   * the cleanup path in t113_musb_reset().
   */

  if (privep->dma_busy)
    {
      if (privep->is_in)
        {
          if (g_musb_dma_tx_inflight > 0)
            {
              g_musb_dma_tx_inflight--;
            }
        }
      else
        {
          if (g_musb_dma_rx_inflight > 0)
            {
              g_musb_dma_rx_inflight--;
            }
        }

      if (g_musb_dma_tx_inflight == 0 && g_musb_dma_rx_inflight == 0)
        {
          t113_musb_vend0_pio();
        }

      privep->dma_busy = false;
      privep->dma_len  = 0;
    }

  if (privep->musb_dma_ch >= 0)
    {
      t113_musb_dma_release(privep->musb_dma_ch);
      privep->musb_dma_ch = -1;
    }

  if (privep->musb_dma_ch_rx >= 0)
    {
      t113_musb_dma_release(privep->musb_dma_ch_rx);
      privep->musb_dma_ch_rx = -1;
    }
#endif

  /* Disable endpoint interrupt */

  t113_ep_select(privep->epphy);

  if (privep->is_in)
    {
      musb_clrbits16(MUSB_INTRTXE, (1 << privep->epphy));
      musb_putreg16(MUSB_TXCSR_FLUSHFIFO, MUSB_TXCSR);
      musb_putreg16(MUSB_TXCSR_FLUSHFIFO, MUSB_TXCSR);
    }
  else
    {
      musb_clrbits16(MUSB_INTRRXE, (1 << privep->epphy));
      musb_putreg16(MUSB_RXCSR_FLUSHFIFO, MUSB_RXCSR);
      musb_putreg16(MUSB_RXCSR_FLUSHFIFO, MUSB_RXCSR);
    }

  /* Cancel all pending requests */

  t113_cancelrequests(privep, -ESHUTDOWN);

  t113_unlock_and_drain(privep->dev, flags);
  return OK;
}

/****************************************************************************
 * Name: t113_epallocreq
 *
 * Description:
 *   Allocate a USB request.
 *
 ****************************************************************************/

static struct usbdev_req_s *t113_epallocreq(struct usbdev_ep_s *ep)
{
  struct t113_req_s *privreq;

  DEBUGASSERT(ep != NULL);

  privreq = kmm_zalloc(sizeof(struct t113_req_s));
  if (privreq == NULL)
    {
      usb_trace_err("epallocreq: out of memory\n");
      return NULL;
    }

  return &privreq->req;
}

/****************************************************************************
 * Name: t113_epfreereq
 *
 * Description:
 *   Free a USB request.
 *
 ****************************************************************************/

static void t113_epfreereq(struct usbdev_ep_s *ep,
                           struct usbdev_req_s *req)
{
  struct t113_req_s *privreq = (struct t113_req_s *)req;

  DEBUGASSERT(ep != NULL && req != NULL);
  kmm_free(privreq);
}

/****************************************************************************
 * Name: t113_epsubmit
 *
 * Description:
 *   Submit a USB request for transfer.
 *
 ****************************************************************************/

static int t113_epsubmit(struct usbdev_ep_s *ep,
                         struct usbdev_req_s *req)
{
  struct t113_ep_s  *privep  = (struct t113_ep_s *)ep;
  struct t113_req_s *privreq = (struct t113_req_s *)req;
  struct t113_usbdev_s *priv;
  irqstate_t flags;
  bool was_empty;

  DEBUGASSERT(ep != NULL && req != NULL && req->callback != NULL &&
              req->buf != NULL);

  priv = privep->dev;

  req->result = -EINPROGRESS;
  req->xfrd   = 0;

  flags = spin_lock_irqsave(&priv->lock);

  /* Handle EP0 specially */

  if (privep->epphy == 0)
    {
      /* EP0 IN or OUT based on current state */

      if (privep->is_in)
        {
          /* EP0 IN: send data */

          t113_rqenqueue(privep, privreq);

          if (priv->ep0state == EP0STATE_SETUP_IN ||
              priv->ep0state == EP0STATE_DATA_IN)
            {
              uint16_t xfrlen = req->len;

              if (xfrlen > EP0_MAXPACKET)
                {
                  xfrlen = EP0_MAXPACKET;
                }

              if (xfrlen > priv->ep0reqlen)
                {
                  xfrlen = priv->ep0reqlen;
                }

              t113_ep_select(0);
              t113_fifo_write(0, req->buf, xfrlen);
              req->xfrd = xfrlen;
              priv->ep0reqlen -= xfrlen;

              if (xfrlen < EP0_MAXPACKET || req->xfrd >= req->len ||
                  priv->ep0reqlen == 0)
                {
                  musb_putreg16(MUSB_CSR0_TXPKTRDY |
                                MUSB_CSR0_DATAEND, MUSB_CSR0);
                  priv->ep0state = EP0STATE_WAIT_STATUS_OUT;

                  privreq = t113_rqdequeue(privep);
                  if (privreq != NULL)
                    {
                      t113_reqcomplete(privep, privreq, OK);
                    }
                }
              else
                {
                  musb_putreg16(MUSB_CSR0_TXPKTRDY, MUSB_CSR0);
                  priv->ep0state = EP0STATE_DATA_IN;
                }
            }
        }
      else
        {
          /* EP0 OUT: receive data */

          t113_rqenqueue(privep, privreq);
        }

      t113_unlock_and_drain(priv, flags);
      return OK;
    }

  /* EPn: add to queue */

  if (privep->stalled)
    {
      t113_unlock_and_drain(priv, flags);
      return -EPERM;
    }

  was_empty = t113_rqenqueue(privep, privreq);

  if (privep->is_in)
    {
      if (was_empty)
        {
          /* Start TX transfer */

          t113_epn_txstart(privep);
        }
    }
  else
    {
      /* If RX data is already in the FIFO, process it now.
       * t113_reqcomplete() only queues the completion - the
       * callback fires outside the lock via t113_drain_done().
       * t113_unlock_and_drain fires callbacks with IRQs masked
       * to prevent an ISR-driven completion from jumping ahead
       * in the class driver's receive queue.
       */

      t113_ep_select(privep->epphy);
      if (musb_getreg16(MUSB_RXCSR) & MUSB_RXCSR_RXPKTRDY)
        {
          t113_epn_rxready(priv, privep->epphy);
        }
    }

  t113_unlock_and_drain(priv, flags);
  return OK;
}

/****************************************************************************
 * Name: t113_epcancel
 *
 * Description:
 *   Cancel a pending USB request.
 *
 ****************************************************************************/

static int t113_epcancel(struct usbdev_ep_s *ep,
                         struct usbdev_req_s *req)
{
  struct t113_ep_s *privep = (struct t113_ep_s *)ep;
  struct t113_req_s *privreq;
  struct t113_req_s *prev;
  irqstate_t flags;

  DEBUGASSERT(ep != NULL && req != NULL);

  flags = spin_lock_irqsave(&privep->dev->lock);

  /* Walk the queue to find and remove only the matching req */

  prev = NULL;
  for (privreq = privep->head;
       privreq != NULL;
       prev = privreq, privreq = privreq->flink)
    {
      if (&privreq->req == req)
        {
          /* Unlink from queue */

          if (prev != NULL)
            {
              prev->flink = privreq->flink;
            }
          else
            {
              privep->head = privreq->flink;
            }

          if (privreq == privep->tail)
            {
              privep->tail = prev;
            }

          privreq->flink = NULL;
          t113_reqcomplete(privep, privreq,
                           -ECONNRESET);
          break;
        }
    }

  t113_unlock_and_drain(privep->dev, flags);
  return OK;
}

/****************************************************************************
 * Name: t113_epstall
 *
 * Description:
 *   Stall or resume an endpoint.
 *
 ****************************************************************************/

static int t113_epstall(struct usbdev_ep_s *ep, bool resume)
{
  struct t113_ep_s *privep = (struct t113_ep_s *)ep;
  irqstate_t flags;

  DEBUGASSERT(ep != NULL);

  usb_trace_info("EP%d %s %s\n", privep->epphy,
                 privep->is_in ? "IN" : "OUT",
                 resume ? "resume" : "stall");

  flags = spin_lock_irqsave(&privep->dev->lock);

  t113_ep_select(privep->epphy);

  if (privep->epphy == 0)
    {
      /* EP0 stall */

      if (!resume)
        {
          privep->stalled = true;
          musb_putreg16(MUSB_CSR0_SENDSTALL |
                        MUSB_CSR0_SVDRXPKTRDY, MUSB_CSR0);
          privep->dev->ep0state = EP0STATE_STALL;
        }
      else
        {
          privep->stalled = false;
          privep->dev->ep0state = EP0STATE_IDLE;
        }
    }
  else if (privep->is_in)
    {
      if (resume)
        {
          /* Clear stall */

          privep->stalled = false;
          musb_clrbits16(MUSB_TXCSR, MUSB_TXCSR_SENDSTALL |
                                      MUSB_TXCSR_SENTSTALL);
          musb_setbits16(MUSB_TXCSR, MUSB_TXCSR_CLRDATATOG);
        }
      else
        {
          privep->stalled = true;
          musb_setbits16(MUSB_TXCSR, MUSB_TXCSR_SENDSTALL);
        }
    }
  else
    {
      if (resume)
        {
          /* Clear stall */

          privep->stalled = false;
          musb_clrbits16(MUSB_RXCSR, MUSB_RXCSR_SENDSTALL |
                                      MUSB_RXCSR_SENTSTALL);
          musb_setbits16(MUSB_RXCSR, MUSB_RXCSR_CLRDATATOG);
        }
      else
        {
          privep->stalled = true;
          musb_setbits16(MUSB_RXCSR, MUSB_RXCSR_SENDSTALL);
        }
    }

  spin_unlock_irqrestore(&privep->dev->lock, flags);
  return OK;
}

/****************************************************************************
 * Device Operations
 ****************************************************************************/

/****************************************************************************
 * Name: t113_allocep
 *
 * Description:
 *   Allocate an endpoint.
 *
 ****************************************************************************/

static struct usbdev_ep_s *t113_allocep(struct usbdev_s *dev,
                                        uint8_t epphy, bool in,
                                        uint8_t eptype)
{
  struct t113_usbdev_s *priv = (struct t113_usbdev_s *)dev;
  struct t113_ep_s *privep;
  irqstate_t flags;
  int idx;
  int epno;

  DEBUGASSERT(dev != NULL);

  epphy &= USB_EPNO_MASK;

  flags = spin_lock_irqsave(&priv->lock);

  if (epphy == 0)
    {
      for (epno = 1; epno < T113_NPHYSEP; epno++)
        {
          idx = in ? T113_EPPHYIN(epno) : T113_EPPHYOUT(epno);
          if ((priv->epavail & (1 << idx)) != 0)
            {
              priv->epavail &= ~(1 << idx);
              privep = &priv->eplist[idx];
              spin_unlock_irqrestore(&priv->lock, flags);

              usb_trace_info("allocep: EP%d %s\n",
                             epno, in ? "IN" : "OUT");
              return &privep->ep;
            }
        }
    }
  else
    {
      /* Allocate the specific EP */

      epno = epphy;
      if (epno < T113_NPHYSEP)
        {
          idx = in ? T113_EPPHYIN(epno) : T113_EPPHYOUT(epno);
          if ((priv->epavail & (1 << idx)) != 0)
            {
              priv->epavail &= ~(1 << idx);
              privep = &priv->eplist[idx];
              spin_unlock_irqrestore(&priv->lock, flags);

              usb_trace_info("allocep: EP%d %s (specific)\n",
                             epno, in ? "IN" : "OUT");
              return &privep->ep;
            }
        }
    }

  spin_unlock_irqrestore(&priv->lock, flags);
  usb_trace_err("allocep: no EP available\n");
  return NULL;
}

/****************************************************************************
 * Name: t113_freeep
 *
 * Description:
 *   Free an endpoint.
 *
 ****************************************************************************/

static void t113_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep)
{
  struct t113_usbdev_s *priv = (struct t113_usbdev_s *)dev;
  struct t113_ep_s *privep = (struct t113_ep_s *)ep;
  irqstate_t flags;
  int idx;

  DEBUGASSERT(dev != NULL && ep != NULL);

  idx = privep->is_in ? T113_EPPHYIN(privep->epphy) :
                         T113_EPPHYOUT(privep->epphy);

  flags = spin_lock_irqsave(&priv->lock);
  priv->epavail |= (1 << idx);
  spin_unlock_irqrestore(&priv->lock, flags);

  usb_trace_info("freeep: EP%d %s\n",
                 privep->epphy, privep->is_in ? "IN" : "OUT");
}

/****************************************************************************
 * Name: t113_getframe
 *
 * Description:
 *   Get the current frame number.
 *
 ****************************************************************************/

static int t113_getframe(struct usbdev_s *dev)
{
  UNUSED(dev);
  return (int)musb_getreg16(MUSB_FRAME);
}

/****************************************************************************
 * Name: t113_wakeup
 *
 * Description:
 *   Send remote wakeup signal.
 *
 ****************************************************************************/

static int t113_wakeup(struct usbdev_s *dev)
{
  UNUSED(dev);
  musb_writeb(musb_readb(MUSB_POWER) | MUSB_POWER_RESUME, MUSB_POWER);
  up_mdelay(10);
  musb_writeb(musb_readb(MUSB_POWER) & ~MUSB_POWER_RESUME, MUSB_POWER);
  return OK;
}

/****************************************************************************
 * Name: t113_selfpowered
 *
 * Description:
 *   Set the self-powered status.
 *
 ****************************************************************************/

static int t113_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  struct t113_usbdev_s *priv = (struct t113_usbdev_s *)dev;
  priv->selfpowered = selfpowered;
  return OK;
}

/****************************************************************************
 * Name: t113_pullup
 *
 * Description:
 *   Control the USB pull-up resistor (connect/disconnect from host).
 *
 ****************************************************************************/

static int t113_pullup(struct usbdev_s *dev, bool enable)
{
  UNUSED(dev);

  if (enable)
    {
      musb_writeb(musb_readb(MUSB_POWER) | MUSB_POWER_SOFTCONN,
                  MUSB_POWER);
    }
  else
    {
      musb_writeb(musb_readb(MUSB_POWER) & ~MUSB_POWER_SOFTCONN,
                  MUSB_POWER);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_usbinitialize
 *
 * Description:
 *   Initialize USB hardware (CCU, PHY, MUSB).
 *   Must be called from arm_boot (early init) because T113 MUSB
 *   POWER.SOFTCONN can only be written before scheduler starts.
 *
 ****************************************************************************/

void arm_usbinitialize(void)
{
  struct t113_usbdev_s *priv = &g_usbdev;
  int i;

  usb_trace_info("T113 MUSB USB device initialization\n");

  /* Zero out the device state */

  memset(priv, 0, sizeof(struct t113_usbdev_s));

  /* Set up the standard device structure */

  priv->usbdev.ops = &g_devops;
  priv->usbdev.ep0 = &priv->eplist[T113_EP0_IN].ep;
  priv->usbdev.speed = USB_SPEED_FULL;
  priv->usbdev.dualspeed = 1;

  /* Initialize all endpoint structures */

  for (i = 0; i < T113_NLOGEP; i++)
    {
      struct t113_ep_s *privep = &priv->eplist[i];

      privep->ep.ops = &g_epops;
      privep->dev = priv;
      privep->head = NULL;
      privep->tail = NULL;
      privep->stalled = false;

      /* Determine physical EP number and direction */

      privep->epphy = i / 2;
      privep->is_in = (i % 2) == 0;

      if (privep->epphy == 0)
        {
          /* EP0 */

          privep->ep.eplog = 0;
          privep->ep.maxpacket = EP0_MAXPACKET;
        }
      else
        {
          /* EPn */

          privep->ep.eplog = privep->is_in ?
                             PHYIN2LOG(privep->epphy) :
                             PHYOUT2LOG(privep->epphy);

          /* Set default maxpacket from FIFO config */

          if (privep->epphy <= 2)
            {
              privep->ep.maxpacket = 1024;
            }
          else
            {
              privep->ep.maxpacket = 512;
            }
        }

      /* FIFO size lookup */

      privep->fifosz = privep->ep.maxpacket;

#ifdef CONFIG_T113_DMA
      privep->musb_dma_ch    = -1;
      privep->musb_dma_ch_rx = -1;
#endif
    }

  /* Available endpoints (all except EP0) */

  priv->epavail = T113_EPALLSET & ~T113_EPCTRLSET;

  t113_ccu_init();
  t113_phy_init();
  t113_musb_init(priv);

  irq_attach(T113_IRQ_USB0_DEVICE, t113_usbdev_interrupt, priv);
  up_enable_irq(T113_IRQ_USB0_DEVICE);

  t113_musb_enable();
}

/****************************************************************************
 * Name: arm_usbuninitialize
 *
 * Description:
 *   Uninitialize USB device hardware.
 *
 ****************************************************************************/

void arm_usbuninitialize(void)
{
  struct t113_usbdev_s *priv = &g_usbdev;
  irqstate_t flags;
  int i;

  /* Disable interrupts */

  up_disable_irq(T113_IRQ_USB0_DEVICE);

  /* Disconnect from host */

  musb_writeb(musb_readb(MUSB_POWER) & ~MUSB_POWER_SOFTCONN, MUSB_POWER);

  /* Disable all interrupt sources */

  musb_writeb(0, MUSB_INTRUSBE);
  musb_putreg16(0, MUSB_INTRTXE);
  musb_putreg16(0, MUSB_INTRRXE);

  /* Detach interrupt handler */

  irq_detach(T113_IRQ_USB0_DEVICE);

  /* Cancel all pending requests (lock required by t113_reqcomplete) */

  flags = spin_lock_irqsave(&priv->lock);
  for (i = 0; i < T113_NLOGEP; i++)
    {
      t113_cancelrequests(&priv->eplist[i], -ESHUTDOWN);
    }

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: usbdev_register
 *
 * Description:
 *   Register a USB device class driver.  The class driver's bind() method
 *   will be called to bind it to a USB device driver.
 *
 ****************************************************************************/

int usbdev_register(struct usbdevclass_driver_s *driver)
{
  struct t113_usbdev_s *priv = &g_usbdev;
  int ret;

  DEBUGASSERT(driver != NULL && driver->ops->bind != NULL &&
              driver->ops->unbind != NULL &&
              driver->ops->setup != NULL &&
              driver->ops->disconnect != NULL);

  if (priv->driver != NULL)
    {
      usb_trace_err("usbdev_register: already bound\n");
      return -EBUSY;
    }

  /* Bind the class driver */

  priv->driver = driver;
  ret = CLASS_BIND(driver, &priv->usbdev);
  if (ret < 0)
    {
      usb_trace_err("usbdev_register: bind failed: %d\n", ret);
      priv->driver = NULL;
      return ret;
    }

  usb_trace_info("usbdev_register: class driver bound\n");

  musb_writeb(musb_readb(MUSB_POWER) & ~MUSB_POWER_SOFTCONN, MUSB_POWER);
  up_mdelay(200);
  musb_writeb(musb_readb(MUSB_POWER) | MUSB_POWER_SOFTCONN, MUSB_POWER);

  return OK;
}

/****************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Unregister a USB device class driver.
 *
 ****************************************************************************/

int usbdev_unregister(struct usbdevclass_driver_s *driver)
{
  struct t113_usbdev_s *priv = &g_usbdev;

  DEBUGASSERT(driver != NULL);

  if (priv->driver != driver)
    {
      return -EINVAL;
    }

  /* Disconnect from host */

  musb_writeb(musb_readb(MUSB_POWER) & ~MUSB_POWER_SOFTCONN, MUSB_POWER);

  /* Unbind the class driver */

  CLASS_DISCONNECT(driver, &priv->usbdev);
  CLASS_UNBIND(driver, &priv->usbdev);

  priv->driver = NULL;

  /* Reconnect to allow new class driver binding */

  musb_writeb(musb_readb(MUSB_POWER) | MUSB_POWER_SOFTCONN, MUSB_POWER);

  usb_trace_info("usbdev_unregister: class driver unbound\n");
  return OK;
}
