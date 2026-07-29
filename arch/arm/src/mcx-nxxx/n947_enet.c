/****************************************************************************
 * arch/arm/src/mcx-nxxx/n947_enet.c
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

/* Copyright 2022 NXP */

/* DWC EMAC Ethernet driver for MCXN947.
 * Adapted from arch/arm/src/s32k3xx/s32k3xx_emac.c.
 * External PHY identity, address and status decoding are supplied by the
 * board layer through BOARD_PHY_* definitions.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <sys/param.h>
#include <arpa/inet.h>

#include <nuttx/wdog.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/phy.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include "arm_internal.h"
#include "chip.h"
#include "hardware/n947/n947_enet.h"
#include "hardware/nxxx_memorymap.h"
#include "hardware/nxxx_clock.h"
#include "nxxx_port.h"
#include "n947_enet.h"

#include <arch/board/board.h>

#ifdef CONFIG_N947_ENET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required
#else
#  define ETHWORK LPWORK
#endif

#if CONFIG_N947_ENET_NETHIFS != 1
#  error "CONFIG_N947_ENET_NETHIFS must be 1"
#endif

#if CONFIG_N947_ENET_NTXBUFFERS < 1
#  error "Need at least one TX buffer"
#endif

#if CONFIG_N947_ENET_NRXBUFFERS < 1
#  error "Need at least one RX buffer"
#endif

#define NENET_NBUFFERS \
  (CONFIG_N947_ENET_NTXBUFFERS + CONFIG_N947_ENET_NRXBUFFERS)

/* Round up to 16-byte alignment for DMA */

#define OPTIMAL_ETH_BUFSIZE  ((CONFIG_NET_ETH_PKTSIZE + 4 + 15) & ~15)

#ifdef CONFIG_N947_ENET_BUFSIZE
#  define ETH_BUFSIZE CONFIG_N947_ENET_BUFSIZE
#else
#  define ETH_BUFSIZE OPTIMAL_ETH_BUFSIZE
#endif

#if ETH_BUFSIZE > N947_ENET_TDES2_B1L_MASK
#  error "ETH_BUFSIZE is too large"
#endif

#if (ETH_BUFSIZE & 15) != 0
#  error "ETH_BUFSIZE must be 16-byte aligned"
#endif

/* One extra free buffer beyond TX count */

#define N947_ENET_NFREEBUFFERS (CONFIG_N947_ENET_NTXBUFFERS + 1)

/* Cache-line alignment for descriptors and buffers.
 * CM33 without D-cache: any alignment works, but 16 bytes keeps descriptors
 * naturally aligned to descriptor size.
 */

#define DMA_BUFFER_MASK    (15)
#define DMA_ALIGN_UP(n)    (((n) + DMA_BUFFER_MASK) & ~DMA_BUFFER_MASK)

#define DESC_SIZE           N947_ENET_DESC_SIZE
#define DESC_PADSIZE        DMA_ALIGN_UP(DESC_SIZE)
#define ALIGNED_BUFSIZE     DMA_ALIGN_UP(ETH_BUFSIZE)

#define RXTABLE_SIZE        (CONFIG_N947_ENET_NRXBUFFERS)
#define TXTABLE_SIZE        (CONFIG_N947_ENET_NTXBUFFERS)

#define RXBUFFER_SIZE       (CONFIG_N947_ENET_NRXBUFFERS * ALIGNED_BUFSIZE)
#define TXBUFFER_SIZE       (N947_ENET_NFREEBUFFERS * ALIGNED_BUFSIZE)

/* TX timeout: 1 minute */

#define N947_TXTIMEOUT     (60 * CLK_TCK)
#define MII_MAXPOLLS       (0x1ffff)
#define LINK_WAITUS        (500 * 1000)
#define LINK_NLOOPS        (4)

/* MDC clock range: 150-250 MHz covers 150MHz system clock */

#define N947_MII_CR_VALUE   N947_ENET_MDIO_CR_150_250MHZ

#define BUF ((struct eth_hdr_s *)priv->dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* DMA descriptor: 4 x uint32_t, padded to 16 bytes */

struct eth_desc_s
{
  uint32_t des0;
  uint32_t des1;
  uint32_t des2;
  uint32_t des3;
};

union n947_desc_u
{
  uint8_t           pad[DESC_PADSIZE];
  struct eth_desc_s desc;
};

struct n947_driver_s
{
  bool bifup;
  uint8_t phyaddr;
  struct wdog_s txtimeout;
  struct work_s irqwork;
  struct work_s pollwork;
  struct work_s linkwork;

  struct net_driver_s dev;

  struct eth_desc_s *txhead;
  struct eth_desc_s *rxhead;
  struct eth_desc_s *txchbase;
  struct eth_desc_s *rxchbase;
  struct eth_desc_s *txtail;
  uint16_t inflight;
  sq_queue_t freeb;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct n947_driver_s g_enet[CONFIG_N947_ENET_NETHIFS];

static union n947_desc_u g_rxtable[RXTABLE_SIZE]
aligned_data(16);
static union n947_desc_u g_txtable[TXTABLE_SIZE]
aligned_data(16);

static uint8_t g_rxbuffer[RXBUFFER_SIZE]
aligned_data(16);
static uint8_t g_txbuffer[TXBUFFER_SIZE]
aligned_data(16);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void n947_initbuffer(struct n947_driver_s *priv, uint8_t *txbuffer);
static inline uint8_t *n947_allocbuffer(struct n947_driver_s *priv);
static inline void n947_freebuffer(struct n947_driver_s *priv,
                                   uint8_t *buffer);
static inline bool n947_isfreebuffer(struct n947_driver_s *priv);

static int  n947_transmit(struct n947_driver_s *priv);
static int  n947_txpoll(struct net_driver_s *dev);

static void n947_enableint(struct n947_driver_s *priv, uint32_t ierbit);
static void n947_disableint(struct n947_driver_s *priv, uint32_t ierbit);

static void n947_freesegment(struct n947_driver_s *priv,
                             struct eth_desc_s *rxfirst, int segments);
static int  n947_recvframe(struct n947_driver_s *priv);
static void n947_receive(struct n947_driver_s *priv);
static void n947_freeframe(struct n947_driver_s *priv);
static void n947_txdone(struct n947_driver_s *priv);

static void n947_interrupt_work(void *arg);
static int  n947_enet_interrupt(int irq, void *context, void *arg);

static void n947_txtimeout_work(void *arg);
static void n947_txtimeout_expiry(wdparm_t arg);

static int  n947_ifup_action(struct net_driver_s *dev, bool resetphy);
static int  n947_ifup(struct net_driver_s *dev);
static int  n947_ifdown(struct net_driver_s *dev);

static void n947_txavail_work(void *arg);
static int  n947_txavail(struct net_driver_s *dev);

#ifdef CONFIG_NET_MCASTGROUP
static int  n947_addmac(struct net_driver_s *dev, const uint8_t *mac);
static int  n947_rmmac(struct net_driver_s *dev, const uint8_t *mac);
#endif

#ifdef CONFIG_NETDEV_IOCTL
static int  n947_ioctl(struct net_driver_s *dev, int cmd,
                       unsigned long arg);
#endif

static int  n947_writemii(struct n947_driver_s *priv, uint8_t phyaddr,
                          uint8_t regaddr, uint16_t data);
static int  n947_readmii(struct n947_driver_s *priv, uint8_t phyaddr,
                         uint8_t regaddr, uint16_t *data);
static int  n947_initphy(struct n947_driver_s *priv, bool renogphy);
static void n947_linkpoll_work(void *arg);

static void n947_initbuffers(struct n947_driver_s *priv,
                             union n947_desc_u *txtable,
                             union n947_desc_u *rxtable,
                             uint8_t *rxbuffer);
static void n947_initdma(struct n947_driver_s *priv);
static void n947_initmtl(struct n947_driver_s *priv);
static uint32_t n947_reset(struct n947_driver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void n947_initbuffer(struct n947_driver_s *priv, uint8_t *txbuffer)
{
  uint8_t *buffer;
  int i;

  sq_init(&priv->freeb);

  for (i = 0, buffer = txbuffer;
       i < N947_ENET_NFREEBUFFERS;
       i++, buffer += ALIGNED_BUFSIZE)
    {
      sq_addlast((sq_entry_t *)buffer, &priv->freeb);
    }
}

static inline uint8_t *n947_allocbuffer(struct n947_driver_s *priv)
{
  return (uint8_t *)sq_remfirst(&priv->freeb);
}

static inline void n947_freebuffer(struct n947_driver_s *priv,
                                   uint8_t *buffer)
{
  sq_addlast((sq_entry_t *)buffer, &priv->freeb);
}

static inline bool n947_isfreebuffer(struct n947_driver_s *priv)
{
  return !sq_empty(&priv->freeb);
}

/****************************************************************************
 * Function: n947_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 ****************************************************************************/

static int n947_transmit(struct n947_driver_s *priv)
{
  struct eth_desc_s *txdesc;
  struct eth_desc_s *txfirst;
  uintptr_t txtail;

  txdesc  = priv->txhead;
  txfirst = txdesc;

  ninfo("d_len: %d inflight: %d\n", priv->dev.d_len, priv->inflight);

  /* Never let software lap the DMA ring.  There is one extra packet buffer
   * beyond the descriptor count, so testing only d_buf (or testing
   * inflight > NTXBUFFERS after enqueue) permits a 13th packet to overwrite
   * descriptor zero while the DMA still owns it.  Under repeated network
   * transactions that corrupts the TX ring and eventually presents as
   * application traffic failures followed by the TX watchdog.
   */

  if (txdesc == NULL ||
      priv->inflight >= TXTABLE_SIZE ||
      (txdesc->des3 & N947_ENET_TDES3_OWN) != 0)
    {
      return -EBUSY;
    }

  n947_enableint(priv, N947_ENET_DMA_CH0_INTERRUPT_ENABLE_TIE);

  /* Flush the contents of the TX buffer into physical memory */

  txdesc->des0 = (uint32_t)(uintptr_t)priv->dev.d_buf;
  txdesc->des1 = 0;
  txdesc->des2 = N947_ENET_TDES2_B1L(priv->dev.d_len) |
                 N947_ENET_TDES2_IOC;
  txdesc->des3 = N947_ENET_TDES3_FL(priv->dev.d_len) |
                 N947_ENET_TDES3_FD | N947_ENET_TDES3_LD;

  /* Give descriptor to DMA */

  txdesc->des3 |= N947_ENET_TDES3_OWN;

  /* Advance TX head pointer */

  if (++priv->txhead >= priv->txchbase + TXTABLE_SIZE)
    {
      priv->txhead = priv->txchbase;
    }

  /* Ensure descriptor writes reach SRAM before the DMA is triggered.
   * Without this barrier the tail-pointer write (Device memory) can
   * overtake the descriptor writes (Normal memory) and the DMA fetches
   * a descriptor without OWN set, suspending the TX engine.
   */

  __asm__ volatile ("dsb" ::: "memory");

  /* Update the TX tail pointer to trigger DMA */

  if (txdesc + 1 >= priv->txchbase + TXTABLE_SIZE)
    {
      txtail = (uintptr_t)priv->txchbase;
    }
  else
    {
      txtail = (uintptr_t)(txdesc + 1);
    }

  putreg32(txtail, N947_ENET_DMA_CH0_TXDESC_TAIL_POINTER);

  /* Track as in-flight */

  if (priv->txtail == NULL)
    {
      priv->txtail = txfirst;
    }

  priv->inflight++;

  /* Start TX timeout watchdog */

  wd_start(&priv->txtimeout, N947_TXTIMEOUT, n947_txtimeout_expiry,
           (wdparm_t)priv);

  n947_disableint(priv, N947_ENET_DMA_CH0_INTERRUPT_ENABLE_TXSE);
  priv->dev.d_buf  = n947_allocbuffer(priv);
  priv->dev.d_len  = 0;

  if (priv->dev.d_buf != NULL)
    {
      return OK;
    }
  else
    {
      return -ENOMEM;
    }
}

/****************************************************************************
 * Function: n947_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *
 ****************************************************************************/

static int n947_txpoll(struct net_driver_s *dev)
{
  struct n947_driver_s *priv = (struct n947_driver_s *)dev->d_private;

  DEBUGASSERT(priv->dev.d_buf != NULL);

  if (priv->dev.d_len > 0)
    {
      int ret = n947_transmit(priv);

      if (ret < 0)
        {
          return ret;
        }

      if (priv->dev.d_buf == NULL ||
          priv->inflight >= CONFIG_N947_ENET_NTXBUFFERS)
        {
          return -EBUSY;
        }
    }

  return OK;
}

/****************************************************************************
 * Function: n947_enableint / n947_disableint
 ****************************************************************************/

static void n947_enableint(struct n947_driver_s *priv, uint32_t ierbit)
{
  uint32_t regval = getreg32(N947_ENET_DMA_CH0_INTERRUPT_ENABLE);
  regval |= (N947_ENET_DMA_CH0_INTERRUPT_ENABLE_NIE | ierbit);
  putreg32(regval, N947_ENET_DMA_CH0_INTERRUPT_ENABLE);
}

static void n947_disableint(struct n947_driver_s *priv, uint32_t ierbit)
{
  uint32_t regval = getreg32(N947_ENET_DMA_CH0_INTERRUPT_ENABLE);
  regval &= ~ierbit;
  if ((regval & N947_ENET_DMAINT_NORMAL) == 0)
    {
      regval &= ~N947_ENET_DMA_CH0_INTERRUPT_ENABLE_NIE;
    }

  putreg32(regval, N947_ENET_DMA_CH0_INTERRUPT_ENABLE);
}

/****************************************************************************
 * Function: n947_freesegment
 *
 * Description:
 *   The function is called when a frame is received using the DMA receive
 *   interrupt.  It scans the RX descriptors to the the received frame.
 *
 ****************************************************************************/

/* Re-arm one RX descriptor with its dedicated buffer and hand it to DMA.
 * RX buffers are statically assigned per descriptor index; the DMA
 * write-back clobbers des0, so the address must be recomputed here.
 * Never give the network stack's d_buf to RX DMA (this driver copies).
 */

static void n947_rearm_rxdesc(struct n947_driver_s *priv,
                              struct eth_desc_s *rxdesc)
{
  rxdesc->des0 = (uint32_t)(uintptr_t)
                 (g_rxbuffer +
                  (rxdesc - priv->rxchbase) * ALIGNED_BUFSIZE);
  rxdesc->des1 = 0;
  rxdesc->des2 = 0;
  rxdesc->des3 = N947_ENET_RDES3_BUF1V | N947_ENET_RDES3_IOC |
                 N947_ENET_RDES3_OWN;
}

static void n947_freesegment(struct n947_driver_s *priv,
                             struct eth_desc_s *rxfirst, int segments)
{
  struct eth_desc_s *rxdesc;
  int i;

  ninfo("rxfirst: %p segments: %d\n", rxfirst, segments);

  rxdesc = rxfirst;

  for (i = 0; i < segments; i++)
    {
      n947_rearm_rxdesc(priv, rxdesc);

      /* Advance pointer */

      if (++rxdesc >= priv->rxchbase + RXTABLE_SIZE)
        {
          rxdesc = priv->rxchbase;
        }
    }

  /* Descriptor writes must reach SRAM before the tail-pointer update */

  __asm__ volatile ("dsb" ::: "memory");

  /* Ring the RX doorbell.  The tail pointer must be an address the DMA
   * current pointer can never equal: a moving tail that wraps to the
   * ring base stops RX for good after the first lap (current==tail ->
   * suspend), and base+N can match the pre-wrap "linear next" address
   * on some EQOS revisions.  base+N+1 matches neither the in-ring
   * addresses nor the linear-next address; the DMA never dereferences
   * the tail value, it only compares it.  Flow control is done by the
   * OWN bits; this write only wakes a suspended RX DMA.
   */

  putreg32((uint32_t)(uintptr_t)(priv->rxchbase + RXTABLE_SIZE + 1),
           N947_ENET_DMA_CH0_RXDESC_TAIL_POINTER);
}

/****************************************************************************
 * Function: n947_recvframe
 *
 * Description:
 *   The function is called when a frame is received using the DMA receive
 *   interrupt.  It scans the RX descriptors to the the received frame.
 *
 ****************************************************************************/

static int n947_recvframe(struct n947_driver_s *priv)
{
  struct eth_desc_s *rxdesc;
  struct eth_desc_s *rxcurr;
  uint8_t *buffer;
  int segments;
  uint32_t des3;

  ninfo("rxhead: %p\n", priv->rxhead);

  rxdesc   = priv->rxhead;
  rxcurr   = NULL;
  segments = 0;

  while (1)
    {
      des3 = rxdesc->des3;

      /* DMA still owns this? */

      if ((des3 & N947_ENET_RDES3_OWN) != 0)
        {
          if (rxcurr != NULL)
            {
              /* Partial frame without all segments -- error */

              n947_freesegment(priv, rxcurr, segments);
            }

          return -EAGAIN;
        }

      /* Is this a context descriptor? */

      if ((des3 & N947_ENET_RDES3_CTXT) != 0)
        {
          /* Put it back */

          n947_rearm_rxdesc(priv, rxdesc);
          rxdesc = (++rxdesc >= priv->rxchbase + RXTABLE_SIZE) ?
                   priv->rxchbase : rxdesc;
          continue;
        }

      if (rxcurr == NULL)
        {
          /* First segment of frame */

          if ((des3 & N947_ENET_RDES3_FD) != 0)
            {
              rxcurr = rxdesc;
              segments = 1;
            }
          else
            {
              /* Not first - stray descriptor, reclaim */

              n947_rearm_rxdesc(priv, rxdesc);
            }
        }
      else
        {
          segments++;
        }

      /* Last segment? */

      if ((des3 & N947_ENET_RDES3_LD) != 0)
        {
          priv->rxhead = (++rxdesc >= priv->rxchbase + RXTABLE_SIZE) ?
                         priv->rxchbase : rxdesc;

          /* Error? */

          if ((des3 & N947_ENET_RDES3_ES) != 0)
            {
              n947_freesegment(priv, rxcurr, segments);
              return -EIO;
            }

          /* Single-buffer frame only for simplicity */

          DEBUGASSERT(segments == 1);

          /* Copy to d_buf.  The DMA write-back clobbers des0, so the
           * buffer address must be computed from the descriptor index.
           */

          buffer = g_rxbuffer +
                   (rxcurr - priv->rxchbase) * ALIGNED_BUFSIZE;
          priv->dev.d_len = (des3 & N947_ENET_RDES3_PL_MASK);

          if (priv->dev.d_len > CONFIG_NET_ETH_PKTSIZE)
            {
              n947_freesegment(priv, rxcurr, segments);
              return -E2BIG;
            }

          memcpy(priv->dev.d_buf, buffer, priv->dev.d_len);

          /* Reclaim descriptor */

          n947_freesegment(priv, rxcurr, segments);
          return OK;
        }

      /* Not last segment yet */

      rxdesc = (++rxdesc >= priv->rxchbase + RXTABLE_SIZE) ?
               priv->rxchbase : rxdesc;
    }
}

/****************************************************************************
 * Function: n947_receive
 ****************************************************************************/

static void n947_receive(struct n947_driver_s *priv)
{
  do
    {
      /* Make sure we have a working buffer (it may have been handed to
       * the TX ring and the pool exhausted on a previous pass).
       */

      if (priv->dev.d_buf == NULL)
        {
          priv->dev.d_buf = n947_allocbuffer(priv);
          if (priv->dev.d_buf == NULL)
            {
              break;
            }
        }

      if (n947_recvframe(priv) < 0)
        {
          break;
        }

#ifdef CONFIG_NET_PKT
      if (IFF_IS_PROMISC(priv->dev.d_flags))
        {
          pkt_input(&priv->dev);
        }
#endif

#ifdef CONFIG_NET_IPv4
      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");
          NETDEV_RXIPV4(&priv->dev);
          ipv4_input(&priv->dev);

          if (priv->dev.d_len > 0)
            {
              if (n947_transmit(priv) < 0)
                {
                  break;  /* TX buffer pool exhausted, d_buf is NULL */
                }
            }
        }
#ifdef CONFIG_NET_ARP
      else if (BUF->type == HTONS(ETHTYPE_ARP))
        {
          ninfo("ARP frame\n");
          NETDEV_RXARP(&priv->dev);
          arp_input(&priv->dev);

          if (priv->dev.d_len > 0)
            {
              if (n947_transmit(priv) < 0)
                {
                  break;
                }
            }
        }
#endif
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(ETHTYPE_IP6))
        {
          ninfo("IPv6 frame\n");
          NETDEV_RXIPV6(&priv->dev);
          ipv6_input(&priv->dev);

          if (priv->dev.d_len > 0)
            {
              if (n947_transmit(priv) < 0)
                {
                  break;
                }
            }
        }
      else
#endif
        {
          NETDEV_RXDROPPED(&priv->dev);
        }
    }
  while (true);
}

/****************************************************************************
 * Function: n947_freeframe
 *
 * Description:
 *   Scan the TX descriptor ring and reclaim any completed buffers.
 *
 ****************************************************************************/

static void n947_freeframe(struct n947_driver_s *priv)
{
  struct eth_desc_s *txdesc;
  int i;

  ninfo("txtail: %p\n", priv->txtail);

  if (priv->txtail == NULL)
    {
      return;
    }

  txdesc = priv->txtail;

  for (i = 0; (txdesc->des3 & N947_ENET_TDES3_OWN) == 0 &&
       priv->inflight > 0; i++)
    {
      /* Free the buffer used by this descriptor */

      if (txdesc->des0 != 0)
        {
          n947_freebuffer(priv, (uint8_t *)(uintptr_t)txdesc->des0);
          txdesc->des0 = 0;
        }

      priv->inflight--;
      NETDEV_TXDONE(&priv->dev);

      if (++txdesc >= priv->txchbase + TXTABLE_SIZE)
        {
          txdesc = priv->txchbase;
        }

      if (txdesc == priv->txhead)
        {
          /* Caught up to head */

          priv->txtail = NULL;
          return;
        }
    }

  priv->txtail = txdesc;
}

/****************************************************************************
 * Function: n947_txdone
 ****************************************************************************/

static void n947_txdone(struct n947_driver_s *priv)
{
  DEBUGASSERT(priv->txtail != NULL);

  n947_freeframe(priv);

  if (priv->inflight <= 0)
    {
      /* All TX completed - stop the TX timeout watchdog, otherwise it
       * fires 60 s after a successful transmit and needlessly restarts
       * the interface (PHY reset drops the link for seconds).
       */

      wd_cancel(&priv->txtimeout);
      n947_disableint(priv, N947_ENET_DMA_CH0_INTERRUPT_ENABLE_TIE);
    }

  /* Poll for new TX data */

  if (priv->dev.d_buf == NULL)
    {
      priv->dev.d_buf = n947_allocbuffer(priv);
    }

  if (priv->dev.d_buf != NULL)
    {
      devif_poll(&priv->dev, n947_txpoll);
    }
}

/****************************************************************************
 * Function: n947_interrupt_work
 ****************************************************************************/

static void n947_interrupt_work(void *arg)
{
  struct n947_driver_s *priv = (struct n947_driver_s *)arg;
  uint32_t dmasr;

  net_lock();

  dmasr = getreg32(N947_ENET_DMA_CH0_STATUS);

  /* Handle Abnormal interrupts */

  if ((dmasr & N947_ENET_DMA_CH0_STATUS_AIS) != 0)
    {
      syslog(LOG_ERR, "ENET: DMA abnormal status 0x%08lx\n",
             (unsigned long)dmasr);
      NETDEV_ERRORS(&priv->dev);

      if ((dmasr & N947_ENET_DMA_CH0_STATUS_RPS) != 0)
        {
          nerr("ERROR: RX stopped\n");
          NETDEV_RXERRORS(&priv->dev);
        }

      if ((dmasr & N947_ENET_DMA_CH0_STATUS_FBE) != 0)
        {
          nerr("ERROR: fatal bus error\n");
          n947_ifdown(&priv->dev);
          n947_ifup_action(&priv->dev, false);
        }
      else
        {
          /* Buffer-unavailable (RBU/TBU) SUSPENDS the DMA; it only resumes
           * when the tail-pointer doorbell is rung again.  The normal RI/TI
           * path re-arms the tails, but RBU/TBU can land in an interrupt
           * without RI/TI, leaving the engine suspended and the transfer
           * dead-locked (seen as a hang during a large TLS upload).  Ring
           * the doorbells explicitly here so a suspended DMA always resumes.
           */

          if ((dmasr & N947_ENET_DMA_CH0_STATUS_RBU) != 0)
            {
              __asm__ volatile ("dsb" ::: "memory");
              putreg32((uint32_t)(uintptr_t)
                       (priv->rxchbase + RXTABLE_SIZE + 1),
                       N947_ENET_DMA_CH0_RXDESC_TAIL_POINTER);
            }

          if ((dmasr & N947_ENET_DMA_CH0_STATUS_TBU) != 0)
            {
              __asm__ volatile ("dsb" ::: "memory");
              putreg32((uint32_t)(uintptr_t)priv->txhead,
                       N947_ENET_DMA_CH0_TXDESC_TAIL_POINTER);
            }
        }

      putreg32(N947_ENET_DMA_CH0_STATUS_AIS |
               N947_ENET_DMAINT_ABNORMAL,
               N947_ENET_DMA_CH0_STATUS);
    }

  /* Handle Normal interrupts */

  if ((dmasr & N947_ENET_DMA_CH0_STATUS_NIS) != 0)
    {
      /* RX interrupt */

      if ((dmasr & N947_ENET_DMA_CH0_STATUS_RI) != 0)
        {
          putreg32(N947_ENET_DMA_CH0_STATUS_RI, N947_ENET_DMA_CH0_STATUS);
          NETDEV_RXPACKETS(&priv->dev);
          n947_receive(priv);
        }

      /* TX interrupt */

      if ((dmasr & N947_ENET_DMA_CH0_STATUS_TI) != 0)
        {
          putreg32(N947_ENET_DMA_CH0_STATUS_TI, N947_ENET_DMA_CH0_STATUS);
          NETDEV_TXPACKETS(&priv->dev);
          n947_txdone(priv);
        }

      putreg32(N947_ENET_DMA_CH0_STATUS_NIS, N947_ENET_DMA_CH0_STATUS);
    }

  /* Re-enable the Ethernet interrupt */

  up_enable_irq(NXXX_IRQ_ETHERNET);
  net_unlock();
}

/****************************************************************************
 * Function: n947_enet_interrupt
 ****************************************************************************/

static int n947_enet_interrupt(int irq, void *context, void *arg)
{
  struct n947_driver_s *priv = &g_enet[0];

  /* Disable further ENET interrupts while we handle it in the work queue */

  up_disable_irq(NXXX_IRQ_ETHERNET);

  work_queue(ETHWORK, &priv->irqwork, n947_interrupt_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Function: n947_txtimeout_work
 ****************************************************************************/

static void n947_txtimeout_work(void *arg)
{
  struct n947_driver_s *priv = (struct n947_driver_s *)arg;

  /* Dump the TX engine state so the hang location is visible:
   * DMA_DEBUG_STATUS0[15:12] = TX DMA state, MTL_TXQ0_DEBUG = queue
   * state, MAC_DEBUG = MAC transmitter state, des3 OWN bit = whether
   * the DMA ever fetched the descriptor.
   */

  syslog(LOG_ERR, "ENET: TX timeout! DMASTAT=0x%08lx DBG0=0x%08lx "
         "MTLDBG=0x%08lx MACDBG=0x%08lx\n",
         (unsigned long)getreg32(N947_ENET_DMA_CH0_STATUS),
         (unsigned long)getreg32(N947_ENET_DMA_DEBUG_STATUS0),
         (unsigned long)getreg32(N947_ENET_MTL_TXQ0_DEBUG),
         (unsigned long)getreg32(N947_ENET_MAC_DEBUG));
  syslog(LOG_ERR, "ENET: txtail=%p des3=0x%08lx inflight=%d\n",
         priv->txtail,
         priv->txtail != NULL ? (unsigned long)priv->txtail->des3 : 0ul,
         priv->inflight);

  net_lock();
  NETDEV_TXTIMEOUTS(&priv->dev);

  n947_ifdown(&priv->dev);
  n947_ifup_action(&priv->dev, true);

  net_unlock();
}

/****************************************************************************
 * Function: n947_txtimeout_expiry
 ****************************************************************************/

static void n947_txtimeout_expiry(wdparm_t arg)
{
  struct n947_driver_s *priv = (struct n947_driver_s *)arg;

  work_queue(ETHWORK, &priv->irqwork, n947_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Function: n947_writemii
 *
 * Description:
 *   Write a 16-bit value to a PHY register via the MDIO interface.
 *
 ****************************************************************************/

static int n947_writemii(struct n947_driver_s *priv, uint8_t phyaddr,
                         uint8_t regaddr, uint16_t data)
{
  uint32_t regval;
  int timeout;

  /* Wait for not busy */

  for (timeout = 0; timeout < MII_MAXPOLLS; timeout++)
    {
      if ((getreg32(N947_ENET_MAC_MDIO_ADDRESS) &
           N947_ENET_MAC_MDIO_ADDRESS_GB) == 0)
        {
          break;
        }
    }

  if (timeout >= MII_MAXPOLLS)
    {
      return -ETIMEDOUT;
    }

  putreg32(data & N947_ENET_MAC_MDIO_DATA_GD_MASK, N947_ENET_MAC_MDIO_DATA);

  regval = N947_ENET_MAC_MDIO_ADDRESS_GB |
           N947_ENET_MAC_MDIO_ADDRESS_GOC_WRITE |
           N947_ENET_MAC_MDIO_ADDRESS_CR(N947_MII_CR_VALUE) |
           N947_ENET_MAC_MDIO_ADDRESS_RDA(regaddr) |
           N947_ENET_MAC_MDIO_ADDRESS_PA(phyaddr);

  putreg32(regval, N947_ENET_MAC_MDIO_ADDRESS);

  /* Wait for completion */

  for (timeout = 0; timeout < MII_MAXPOLLS; timeout++)
    {
      if ((getreg32(N947_ENET_MAC_MDIO_ADDRESS) &
           N947_ENET_MAC_MDIO_ADDRESS_GB) == 0)
        {
          return OK;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Function: n947_readmii
 *
 * Description:
 *   Read a 16-bit value from a PHY register via the MDIO interface.
 *
 ****************************************************************************/

static int n947_readmii(struct n947_driver_s *priv, uint8_t phyaddr,
                        uint8_t regaddr, uint16_t *data)
{
  uint32_t regval;
  int timeout;

  /* Wait for not busy */

  for (timeout = 0; timeout < MII_MAXPOLLS; timeout++)
    {
      if ((getreg32(N947_ENET_MAC_MDIO_ADDRESS) &
           N947_ENET_MAC_MDIO_ADDRESS_GB) == 0)
        {
          break;
        }
    }

  if (timeout >= MII_MAXPOLLS)
    {
      return -ETIMEDOUT;
    }

  regval = N947_ENET_MAC_MDIO_ADDRESS_GB |
           N947_ENET_MAC_MDIO_ADDRESS_GOC_READ |
           N947_ENET_MAC_MDIO_ADDRESS_CR(N947_MII_CR_VALUE) |
           N947_ENET_MAC_MDIO_ADDRESS_RDA(regaddr) |
           N947_ENET_MAC_MDIO_ADDRESS_PA(phyaddr);

  putreg32(regval, N947_ENET_MAC_MDIO_ADDRESS);

  /* Wait for completion */

  for (timeout = 0; timeout < MII_MAXPOLLS; timeout++)
    {
      if ((getreg32(N947_ENET_MAC_MDIO_ADDRESS) &
           N947_ENET_MAC_MDIO_ADDRESS_GB) == 0)
        {
          *data = (uint16_t)(getreg32(N947_ENET_MAC_MDIO_DATA) &
                             N947_ENET_MAC_MDIO_DATA_GD_MASK);
          return OK;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Function: n947_initphy
 *
 * Description:
 *   Configure the board-selected PHY and wait for link.
 *
 ****************************************************************************/

static int n947_initphy(struct n947_driver_s *priv, bool renogphy)
{
  uint16_t phyid1;
  uint16_t phyid2;
  uint16_t phydata;
  uint16_t phystatus;
  uint8_t scan;
  int retries;
  int ret;

  /* Set up MDIO clock */

  putreg32(N947_ENET_MAC_MDIO_ADDRESS_CR(N947_MII_CR_VALUE),
           N947_ENET_MAC_MDIO_ADDRESS);

  /* Scan PHY addresses 0-7 to find where the selected PHY responds.
   * Board PHY address strapping may place it at a non-zero address.
   */

  for (scan = 0; scan < 8; scan++)
    {
      uint16_t id1 = 0;
      uint16_t id2 = 0;

      n947_readmii(priv, scan, MII_PHYID1, &id1);
      n947_readmii(priv, scan, MII_PHYID2, &id2);
      syslog(LOG_INFO, "ENET: PHY scan addr=%d ID1=0x%04x ID2=0x%04x\n",
             scan, id1, id2);
      if (id1 != 0x0000 && id1 != 0xffff &&
          id1 == BOARD_PHYID1 &&
          (id2 & 0xfff0) == (BOARD_PHYID2 & 0xfff0))
        {
          syslog(LOG_INFO, "ENET: " BOARD_PHY_NAME " found at addr=%d\n",
                 scan);
          priv->phyaddr = scan;
          break;
        }
    }

  /* Read PHY ID at confirmed address */

  ret = n947_readmii(priv, priv->phyaddr, MII_PHYID1, &phyid1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ENET: MDIO timeout reading PHYID1 (ret=%d)\n", ret);
      return ret;
    }

  ret = n947_readmii(priv, priv->phyaddr, MII_PHYID2, &phyid2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ENET: MDIO timeout reading PHYID2 (ret=%d)\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "ENET: PHY ID1=0x%04x ID2=0x%04x\n", phyid1, phyid2);

  if (phyid1 != BOARD_PHYID1 ||
      (phyid2 & 0xfff0) != (BOARD_PHYID2 & 0xfff0))
    {
      syslog(LOG_ERR,
             "ENET: PHY ID mismatch: got %04x/%04x expected %04x/%04x\n",
             phyid1, phyid2, BOARD_PHYID1, BOARD_PHYID2);
      return -ENODEV;
    }

  syslog(LOG_INFO, "ENET: PHY " BOARD_PHY_NAME " detected\n");

  if (renogphy)
    {
      /* Reset PHY */

      ret = n947_writemii(priv, priv->phyaddr, MII_MCR, MII_MCR_RESET);
      if (ret < 0)
        {
          return ret;
        }

      /* Wait for reset completion */

      for (retries = 0; retries < 1000; retries++)
        {
          ret = n947_readmii(priv, priv->phyaddr, MII_MCR, &phydata);
          if (ret < 0)
            {
              return ret;
            }

          if ((phydata & MII_MCR_RESET) == 0)
            {
              break;
            }

          up_udelay(100);
        }

      /* Enable auto-negotiation */

      ret = n947_writemii(priv, priv->phyaddr, MII_MCR,
                          MII_MCR_ANENABLE | MII_MCR_ANRESTART);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Wait for link - not finding link is not fatal; ifup still succeeds.
   * MII_MSR LINKSTATUS (bit2) is latching-low: read twice to get the
   * current state.  Wait up to LINK_NLOOPS * LINK_WAITUS for autoneg.
   */

  phydata = 0;
  for (retries = 0; retries < LINK_NLOOPS; retries++)
    {
      /* First read clears any latched-low link status */

      n947_readmii(priv, priv->phyaddr, MII_MSR, &phydata);
      ret = n947_readmii(priv, priv->phyaddr, MII_MSR, &phydata);
      if (ret < 0)
        {
          return ret;
        }

      if ((phydata & MII_MSR_LINKSTATUS) != 0)
        {
          break;
        }

      up_udelay(LINK_WAITUS);
    }

  if ((phydata & MII_MSR_LINKSTATUS) == 0)
    {
      uint16_t bmcr = 0;
      uint16_t scsr = 0;
      n947_readmii(priv, priv->phyaddr, MII_MCR, &bmcr);
      n947_readmii(priv, priv->phyaddr, BOARD_PHY_STATUS, &scsr);
      syslog(LOG_WARNING,
             "ENET: No link - MSR=0x%04x BMCR=0x%04x SCSR=0x%04x "
             "(ANEG_done=%d link=%d)\n",
             phydata, bmcr, scsr,
             (phydata & MII_MSR_ANEGCOMPLETE) ? 1 : 0,
             (phydata & MII_MSR_LINKSTATUS) ? 1 : 0);

      /* Keep default 100M full-duplex settings from MAC_CONFIGURATION init */

      return OK;
    }

  syslog(LOG_INFO, "ENET: Link established\n");

  /* Read PHY status for speed/duplex */

  ret = n947_readmii(priv, priv->phyaddr, BOARD_PHY_STATUS, &phystatus);
  if (ret < 0)
    {
      return ret;
    }

  uint32_t maccfg = getreg32(N947_ENET_MAC_CONFIGURATION);
  maccfg &= ~(N947_ENET_MAC_CONFIGURATION_DM |
              N947_ENET_MAC_CONFIGURATION_FES);

  if (BOARD_PHY_100BASET(phystatus))
    {
      maccfg |= N947_ENET_MAC_CONFIGURATION_FES;
      syslog(LOG_INFO, "ENET: 100Base-T\n");
    }
  else
    {
      syslog(LOG_INFO, "ENET: 10Base-T\n");
    }

  if (BOARD_PHY_ISDUPLEX(phystatus))
    {
      maccfg |= N947_ENET_MAC_CONFIGURATION_DM;
      syslog(LOG_INFO, "ENET: Full duplex\n");
    }
  else
    {
      syslog(LOG_INFO, "ENET: Half duplex\n");
    }

  putreg32(maccfg, N947_ENET_MAC_CONFIGURATION);

  /* Mark the carrier as up: IFF_RUNNING is required by the IPv4 routing
   * lookup (netdev_findby_ripv4addr); without it every outbound packet
   * fails with ENETUNREACH even though the interface is UP.
   */

  netdev_carrier_on(&priv->dev);
  return OK;
}

/****************************************************************************
 * Function: n947_linkpoll_work
 *
 * Description:
 *   Periodic PHY link supervision.  Reads the PHY link status and toggles
 *   the interface carrier (IFF_RUNNING) accordingly, so that cable
 *   plug/unplug after ifup is detected.  Reschedules itself every second
 *   while the interface is up.
 *
 ****************************************************************************/

static void n947_linkpoll_work(void *arg)
{
  struct n947_driver_s *priv = (struct n947_driver_s *)arg;
  uint16_t msr = 0;
  bool link;
  bool carrier;

  net_lock();

  if (!priv->bifup)
    {
      net_unlock();
      return;
    }

  /* LINKSTATUS is latching-low: read twice for the current state */

  n947_readmii(priv, priv->phyaddr, MII_MSR, &msr);
  n947_readmii(priv, priv->phyaddr, MII_MSR, &msr);

  link    = (msr & MII_MSR_LINKSTATUS) != 0;
  carrier = IFF_IS_RUNNING(priv->dev.d_flags);

  if (link && !carrier)
    {
      uint16_t scsr = 0;
      uint32_t maccfg;

      /* Sync MAC speed/duplex with the autonegotiation result; ifup may
       * have configured defaults before the link was established.
       */

      n947_readmii(priv, priv->phyaddr, BOARD_PHY_STATUS, &scsr);

      maccfg = getreg32(N947_ENET_MAC_CONFIGURATION);
      maccfg &= ~(N947_ENET_MAC_CONFIGURATION_FES |
                  N947_ENET_MAC_CONFIGURATION_DM);

      if (BOARD_PHY_100BASET(scsr))
        {
          maccfg |= N947_ENET_MAC_CONFIGURATION_FES;
        }

      if (BOARD_PHY_ISDUPLEX(scsr))
        {
          maccfg |= N947_ENET_MAC_CONFIGURATION_DM;
        }

      putreg32(maccfg, N947_ENET_MAC_CONFIGURATION);

      syslog(LOG_INFO, "ENET: Link UP (%s %s, SCSR=0x%04x)\n",
             BOARD_PHY_100BASET(scsr) ? "100M" : "10M",
             BOARD_PHY_ISDUPLEX(scsr) ? "FDX" : "HDX",
             scsr);
      netdev_carrier_on(&priv->dev);
    }
  else if (!link && carrier)
    {
      syslog(LOG_INFO, "ENET: Link DOWN\n");
      netdev_carrier_off(&priv->dev);
    }

  net_unlock();

  work_queue(ETHWORK, &priv->linkwork, n947_linkpoll_work, priv,
             SEC2TICK(1));
}

/****************************************************************************
 * Function: n947_initbuffers
 *
 * Description:
 *   Initialize TX and RX descriptor rings and point them to buffers.
 *
 ****************************************************************************/

static void n947_initbuffers(struct n947_driver_s *priv,
                             union n947_desc_u *txtable,
                             union n947_desc_u *rxtable,
                             uint8_t *rxbuffer)
{
  struct eth_desc_s *txdesc;
  struct eth_desc_s *rxdesc;
  int i;

  /* TX descriptors: all start as owned by software (empty) */

  priv->txchbase = (struct eth_desc_s *)txtable;
  priv->txhead   = priv->txchbase;
  priv->txtail   = NULL;
  priv->inflight = 0;

  for (i = 0, txdesc = priv->txchbase; i < TXTABLE_SIZE; i++, txdesc++)
    {
      txdesc->des0 = 0;
      txdesc->des1 = 0;
      txdesc->des2 = 0;
      txdesc->des3 = 0;  /* Software owns */
    }

  /* RX descriptors: all start as owned by DMA */

  priv->rxchbase = (struct eth_desc_s *)rxtable;
  priv->rxhead   = priv->rxchbase;

  for (i = 0, rxdesc = priv->rxchbase;
       i < RXTABLE_SIZE;
       i++, rxdesc++, rxbuffer += ALIGNED_BUFSIZE)
    {
      rxdesc->des0 = (uint32_t)(uintptr_t)rxbuffer;
      rxdesc->des1 = 0;
      rxdesc->des2 = 0;
      rxdesc->des3 = N947_ENET_RDES3_BUF1V | N947_ENET_RDES3_IOC |
                     N947_ENET_RDES3_OWN;
    }
}

/****************************************************************************
 * Function: n947_initdma
 *
 * Description:
 *   Initialize the DMA engine.
 *
 ****************************************************************************/

static void n947_initdma(struct n947_driver_s *priv)
{
  uint32_t regval;

  /* Descriptor rings were just written by the CPU - make sure they have
   * reached SRAM before the DMA engine is configured to fetch them.
   */

  __asm__ volatile ("dsb" ::: "memory");

  /* DMA sysbus mode: fixed-burst, address-aligned */

  regval = N947_ENET_DMA_SYSBUS_MODE_AAL | N947_ENET_DMA_SYSBUS_MODE_FB;
  putreg32(regval, N947_ENET_DMA_SYSBUS_MODE);

  /* Channel 0 control: descriptor skip = 0, no PBLx8 */

  putreg32(0, N947_ENET_DMA_CH0_CONTROL);

  /* TX: PBL=32, start (will be started after descriptor init) */

  putreg32(N947_ENET_DMA_CH0_TX_CONTROL_TXPBL(32),
           N947_ENET_DMA_CH0_TX_CONTROL);

  /* RX: PBL=32, buffer size */

  /* RBSZ takes the buffer size in bytes (the macro shifts it into
   * bits [14:1]).  Passing size>>1 halves the usable RX buffer and
   * makes the DMA split frames larger than 760 bytes.
   */

  putreg32(N947_ENET_DMA_CH0_RX_CONTROL_RXPBL(32) |
           N947_ENET_DMA_CH0_RX_CONTROL_RBSZ(ALIGNED_BUFSIZE),
           N947_ENET_DMA_CH0_RX_CONTROL);

  /* TX descriptor ring */

  putreg32((uint32_t)(uintptr_t)priv->txchbase,
           N947_ENET_DMA_CH0_TXDESC_LIST_ADDRESS);
  putreg32(TXTABLE_SIZE - 1, N947_ENET_DMA_CH0_TXDESC_RING_LENGTH);

  /* RX descriptor ring */

  putreg32((uint32_t)(uintptr_t)priv->rxchbase,
           N947_ENET_DMA_CH0_RXDESC_LIST_ADDRESS);
  putreg32(RXTABLE_SIZE - 1, N947_ENET_DMA_CH0_RXDESC_RING_LENGTH);

  /* Tail pointers */

  putreg32((uint32_t)(uintptr_t)priv->txchbase,
           N947_ENET_DMA_CH0_TXDESC_TAIL_POINTER);

  /* RX tail: see n947_freesegment - must never equal the DMA current
   * pointer, including the pre-wrap linear-next address (base + N).
   */

  putreg32((uint32_t)(uintptr_t)(priv->rxchbase + RXTABLE_SIZE + 1),
           N947_ENET_DMA_CH0_RXDESC_TAIL_POINTER);

  /* Enable interrupts */

  regval = N947_ENET_DMAINT_NORMAL | N947_ENET_DMAINT_ABNORMAL |
           N947_ENET_DMA_CH0_INTERRUPT_ENABLE_NIE |
           N947_ENET_DMA_CH0_INTERRUPT_ENABLE_AIE;
  putreg32(regval, N947_ENET_DMA_CH0_INTERRUPT_ENABLE);
}

/****************************************************************************
 * Function: n947_initmtl
 *
 * Description:
 *   Initialize the MTL (MAC Transaction Layer).
 *
 ****************************************************************************/

static void n947_initmtl(struct n947_driver_s *priv)
{
  uint32_t regval;

  /* TX queue 0: store-and-forward, TX queue size = 2048 bytes
   * (256 words-1).
   */

  regval = N947_ENET_MTL_TXQ0_OPERATION_MODE_TSF |
           N947_ENET_MTL_TXQ0_OPERATION_MODE_TXQEN |
           N947_ENET_MTL_TXQ0_OPERATION_MODE_TQS(7);  /* (7+1)*256=2048 */
  putreg32(regval, N947_ENET_MTL_TXQ0_OPERATION_MODE);

  /* RX queue 0: store-and-forward, RX queue size = 2048 bytes */

  regval = N947_ENET_MTL_RXQ0_OPERATION_MODE_RSF |
           N947_ENET_MTL_RXQ0_OPERATION_MODE_RQS(7);
  putreg32(regval, N947_ENET_MTL_RXQ0_OPERATION_MODE);
}

/****************************************************************************
 * Function: n947_reset
 *
 * Description:
 *   Perform a software reset of the DWC EMAC.
 *
 ****************************************************************************/

static uint32_t n947_reset(struct n947_driver_s *priv)
{
  uint32_t timeout;

  /* DWC EMAC SWR waits for all clock domains (AHB + RMII) to complete.
   * Give up to 100 ms for the RMII 50 MHz from the PHY to stabilise.
   */

  putreg32(N947_ENET_DMA_MODE_SWR, N947_ENET_DMA_MODE);

  for (timeout = 0; timeout < 10000; timeout++)
    {
      if ((getreg32(N947_ENET_DMA_MODE) & N947_ENET_DMA_MODE_SWR) == 0)
        {
          break;
        }

      up_udelay(10);
    }

  return timeout;
}

/****************************************************************************
 * Function: n947_macaddress
 *
 * Description:
 *   Program the MAC address into hardware.
 *
 ****************************************************************************/

static void n947_macaddress(struct n947_driver_s *priv)
{
  struct net_driver_s *dev = &priv->dev;
  uint32_t addr;

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        dev->d_mac.ether.ether_addr_octet[0],
        dev->d_mac.ether.ether_addr_octet[1],
        dev->d_mac.ether.ether_addr_octet[2],
        dev->d_mac.ether.ether_addr_octet[3],
        dev->d_mac.ether.ether_addr_octet[4],
        dev->d_mac.ether.ether_addr_octet[5]);

  /* Write HIGH first, then LOW: the MAC address registers are
   * double-synchronized into the receive (MII) clock domain and the
   * synchronization is triggered ONLY by the LOW-register write (see
   * the note in MCUXpresso fsl_enet_qos.h ENET_QOS_SetMacAddr).
   * Writing LOW first latches the reset-value HIGH (FF:FF) into the
   * receive filter: every unicast frame is silently dropped while CSR
   * reads still return the correct address.
   */

  addr = ((uint32_t)dev->d_mac.ether.ether_addr_octet[5] << 8) |
          (uint32_t)dev->d_mac.ether.ether_addr_octet[4];
  putreg32(addr | (1u << 31), N947_ENET_MAC_ADDRESS0_HIGH);

  addr = ((uint32_t)dev->d_mac.ether.ether_addr_octet[3] << 24) |
         ((uint32_t)dev->d_mac.ether.ether_addr_octet[2] << 16) |
         ((uint32_t)dev->d_mac.ether.ether_addr_octet[1] << 8)  |
          (uint32_t)dev->d_mac.ether.ether_addr_octet[0];
  putreg32(addr, N947_ENET_MAC_ADDRESS0_LOW);
}

/****************************************************************************
 * Function: n947_ifup_action
 *
 * Description:
 *   Internal ifup that can optionally skip PHY reset (used after TX timeout
 *   to restart without full renegotiation).
 *
 ****************************************************************************/

static int n947_ifup_action(struct net_driver_s *dev, bool resetphy)
{
  struct n947_driver_s *priv = (struct n947_driver_s *)dev->d_private;
  uint32_t regval;
  uint32_t rst;
  int ret;

  syslog(LOG_INFO, "ENET: ifup_action called (drv v11)\n");

  /* Re-select RMII before the MAC soft reset: the MAC samples the
   * phy_intf_sel input (HW_FEAT0.ACTPHYSEL) when SWR completes.
   */

  putreg32(N947_ENET_PHY_INTF_RMII, N947_SYSCON0_ENETPHYINTFSEL);

  /* Reset MAC.  SWR needs RMII 50 MHz from PHY; warn but continue on
   * timeout.
   */

  rst = n947_reset(priv);
  syslog(LOG_INFO, "ENET: DMA reset returned %lu\n", (unsigned long)rst);

  /* HW_FEAT0[30:28] = ACTPHYSEL, sampled at SWR: 100b = RMII (0x4e..),
   * 000b = MII (0x0e.. - interface select wrong, RX/TX dead).
   */

  syslog(LOG_INFO, "ENET: MAC_VERSION=0x%08lx HW_FEAT0=0x%08lx\n",
         (unsigned long)getreg32(N947_ENET_MAC_VERSION),
         (unsigned long)getreg32(N947_ENET_MAC_HW_FEATURE0));
  if (rst >= 10000)
    {
      syslog(LOG_WARNING,
             "ENET: DMA reset timeout (RMII clock may not be running)\n");

      /* Continue anyway - MDIO and register access may still work */
    }

  /* Disable MMC interrupts */

  putreg32(0xffffffff, N947_ENET_MMC_TX_INTERRUPT_MASK);
  putreg32(0xffffffff, N947_ENET_MMC_RX_INTERRUPT_MASK);

  /* Initialize MTL */

  n947_initmtl(priv);

  /* Configure MAC */

  /* Note: no IPC (RX checksum offload) - the software stack verifies
   * checksums; hardware offload can cause IP frames to be flagged or
   * dropped while non-IP (ARP) traffic passes, which is hard to debug.
   */

  /* Select MII/RMII, 100 Mbps, full duplex and automatic pad/CRC stripping.
   * The speed and duplex settings are adjusted after PHY initialization.
   */

  regval = N947_ENET_MAC_CONFIGURATION_PS |
           N947_ENET_MAC_CONFIGURATION_FES |
           N947_ENET_MAC_CONFIGURATION_DM |
           N947_ENET_MAC_CONFIGURATION_ACS;
  putreg32(regval, N947_ENET_MAC_CONFIGURATION);

  /* Enable RX queue 0 */

  putreg32(N947_ENET_MAC_RXQ_CTRL0_RXQ0EN_DCB, N947_ENET_MAC_RXQ_CTRL0);

  /* Set MAC address */

  n947_macaddress(priv);

  /* Initialize descriptor rings */

  n947_initbuffers(priv, g_txtable, g_rxtable, g_rxbuffer);

  /* Initialize DMA */

  n947_initdma(priv);

  /* Initialize PHY and get link speed/duplex */

  ret = n947_initphy(priv, resetphy);
  if (ret < 0)
    {
      nerr("ERROR: n947_initphy failed: %d\n", ret);
      return ret;
    }

  /* Enable TX and RX in MAC */

  regval  = getreg32(N947_ENET_MAC_CONFIGURATION);
  regval |= N947_ENET_MAC_CONFIGURATION_TE | N947_ENET_MAC_CONFIGURATION_RE;
  putreg32(regval, N947_ENET_MAC_CONFIGURATION);

  /* Start DMA TX and RX */

  regval  = getreg32(N947_ENET_DMA_CH0_TX_CONTROL);
  regval |= N947_ENET_DMA_CH0_TX_CONTROL_ST;
  putreg32(regval, N947_ENET_DMA_CH0_TX_CONTROL);

  regval  = getreg32(N947_ENET_DMA_CH0_RX_CONTROL);
  regval |= N947_ENET_DMA_CH0_RX_CONTROL_SR;
  putreg32(regval, N947_ENET_DMA_CH0_RX_CONTROL);

  priv->bifup = true;

  /* Attach and enable interrupt */

  irq_attach(NXXX_IRQ_ETHERNET, n947_enet_interrupt, NULL);
  up_enable_irq(NXXX_IRQ_ETHERNET);

  /* Start periodic PHY link supervision (sets/clears IFF_RUNNING) */

  work_queue(ETHWORK, &priv->linkwork, n947_linkpoll_work, priv,
             SEC2TICK(1));

  return OK;
}

/****************************************************************************
 * Function: n947_ifup
 ****************************************************************************/

static int n947_ifup(struct net_driver_s *dev)
{
  return n947_ifup_action(dev, true);
}

/****************************************************************************
 * Function: n947_ifdown
 ****************************************************************************/

static int n947_ifdown(struct net_driver_s *dev)
{
  struct n947_driver_s *priv = (struct n947_driver_s *)dev->d_private;
  irqstate_t flags;

  flags = enter_critical_section();

  up_disable_irq(NXXX_IRQ_ETHERNET);
  irq_detach(NXXX_IRQ_ETHERNET);
  wd_cancel(&priv->txtimeout);
  work_cancel(ETHWORK, &priv->linkwork);
  netdev_carrier_off(&priv->dev);

  /* Stop DMA */

  uint32_t regval;
  regval  = getreg32(N947_ENET_DMA_CH0_TX_CONTROL);
  regval &= ~N947_ENET_DMA_CH0_TX_CONTROL_ST;
  putreg32(regval, N947_ENET_DMA_CH0_TX_CONTROL);

  regval  = getreg32(N947_ENET_DMA_CH0_RX_CONTROL);
  regval &= ~N947_ENET_DMA_CH0_RX_CONTROL_SR;
  putreg32(regval, N947_ENET_DMA_CH0_RX_CONTROL);

  /* Disable TX/RX in MAC */

  regval  = getreg32(N947_ENET_MAC_CONFIGURATION);
  regval &= ~(N947_ENET_MAC_CONFIGURATION_TE |
              N947_ENET_MAC_CONFIGURATION_RE);
  putreg32(regval, N947_ENET_MAC_CONFIGURATION);

  priv->bifup = false;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Function: n947_txavail_work
 ****************************************************************************/

static void n947_txavail_work(void *arg)
{
  struct n947_driver_s *priv = (struct n947_driver_s *)arg;

  net_lock();

  if (priv->bifup)
    {
      if (priv->dev.d_buf == NULL)
        {
          priv->dev.d_buf = n947_allocbuffer(priv);
        }

      if (priv->dev.d_buf != NULL)
        {
          devif_poll(&priv->dev, n947_txpoll);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Function: n947_txavail
 ****************************************************************************/

static int n947_txavail(struct net_driver_s *dev)
{
  struct n947_driver_s *priv = (struct n947_driver_s *)dev->d_private;

  if (work_available(&priv->pollwork))
    {
      work_queue(ETHWORK, &priv->pollwork, n947_txavail_work, priv, 0);
    }

  return OK;
}

#ifdef CONFIG_NET_MCASTGROUP
/****************************************************************************
 * Function: n947_addmac / n947_rmmac
 *
 * Description:
 *   Add/remove a multicast MAC address.  Uses the hash filter approach.
 *
 ****************************************************************************/

static int n947_addmac(struct net_driver_s *dev, const uint8_t *mac)
{
  /* Set promiscuous mode to accept all multicast */

  uint32_t regval = getreg32(N947_ENET_MAC_PACKET_FILTER);
  regval |= N947_ENET_MAC_PACKET_FILTER_PM;
  putreg32(regval, N947_ENET_MAC_PACKET_FILTER);
  return OK;
}

static int n947_rmmac(struct net_driver_s *dev, const uint8_t *mac)
{
  return OK;
}
#endif

#ifdef CONFIG_NETDEV_IOCTL
/****************************************************************************
 * Function: n947_ioctl
 ****************************************************************************/

static int n947_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg)
{
  int ret = -EINVAL;

#ifdef CONFIG_NETDEV_PHY_IOCTL
  struct mii_ioctl_data_s *req = (struct mii_ioctl_data_s *)((uintptr_t)arg);
  struct n947_driver_s *priv = (struct n947_driver_s *)dev->d_private;

  switch (cmd)
    {
      case SIOCGMIIPHY:
        req->phy_id = priv->phyaddr;
        ret = OK;
        break;

      case SIOCGMIIREG:
        {
          uint16_t phydata;
          ret = n947_readmii(priv, req->phy_id, req->reg_num, &phydata);
          req->val_out = phydata;
        }
        break;

      case SIOCSMIIREG:
        ret = n947_writemii(priv, req->phy_id, req->reg_num, req->val_in);
        break;

      default:
        ret = -ENOTTY;
        break;
    }
#endif

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: n947_netinitialize
 *
 * Description:
 *   Initialize the ENET driver.  This function should be called once during
 *   board initialization.  A random MAC address will be used unless
 *   board.h defines BOARD_ENET_MACADDR.
 *
 ****************************************************************************/

int n947_netinitialize(int intf)
{
  struct n947_driver_s *priv;

  DEBUGASSERT(intf < CONFIG_N947_ENET_NETHIFS);
  priv = &g_enet[intf];

  /* Enable ENET peripheral clock gate (AHB_CLK_CTRL2, bit 2) */

  putreg32(N947_ENET_AHBCLK_BIT, AHB_CLK_CTRL_SET2);

  /* Select the external RMII reference clock supplied by the board PHY. */

  putreg32(N947_ENETRMII_CLK_NONE, N947_SYSCON0_ENETRMIICLKSEL);

  /* Assert then deassert ENET peripheral reset.
   * PRESETCTRL2 bit=1 -> peripheral in reset; bit=0 -> released.
   * PRESETCTRLSET2 sets bit to 1 (assert); PRESETCTRLCLR2 clears bit to 0
   * (deassert).
   */

  /* Assert reset. */

  putreg32(N947_ENET_RST_MASK, N947_SYSCON0_PRESETCTRLSET2);
  up_udelay(10);

  /* Deassert reset. */

  putreg32(N947_ENET_RST_MASK, N947_SYSCON0_PRESETCTRLCLR2);
  up_udelay(10);

  /* Select RMII interface type (ENET_PHY_INTF_SEL = 1).
   * Boards using this driver provide an RMII-connected external PHY.
   */

  putreg32(N947_ENET_PHY_INTF_RMII, N947_SYSCON0_ENETPHYINTFSEL);

  /* Configure ENET port pins (all PORT1, ALT9) */

  nxxx_port_configure(PORT_ENET_TX_CLK);
  nxxx_port_configure(PORT_ENET_TXEN);
  nxxx_port_configure(PORT_ENET_TXD0);
  nxxx_port_configure(PORT_ENET_TXD1);
  nxxx_port_configure(PORT_ENET_RXDV);
  nxxx_port_configure(PORT_ENET_RXD0);
  nxxx_port_configure(PORT_ENET_RXD1);
  nxxx_port_configure(PORT_ENET_MDC);
  nxxx_port_configure(PORT_ENET_MDIO);

  priv->phyaddr = BOARD_PHY_ADDR;

  /* Set up a default MAC address */

  priv->dev.d_mac.ether.ether_addr_octet[0] = 0x02;
  priv->dev.d_mac.ether.ether_addr_octet[1] = 0x00;
  priv->dev.d_mac.ether.ether_addr_octet[2] = 0x00;
  priv->dev.d_mac.ether.ether_addr_octet[3] = 0x00;
  priv->dev.d_mac.ether.ether_addr_octet[4] = 0x00;
  priv->dev.d_mac.ether.ether_addr_octet[5] = intf + 1;

#ifdef BOARD_ENET_MACADDR
  memcpy(priv->dev.d_mac.ether.ether_addr_octet, BOARD_ENET_MACADDR, 6);
#endif

  /* Initialize TX buffer pool */

  n947_initbuffer(priv, g_txbuffer);
  priv->dev.d_buf = n947_allocbuffer(priv);

  /* Set up driver callbacks */

  priv->dev.d_ifup    = n947_ifup;
  priv->dev.d_ifdown  = n947_ifdown;
  priv->dev.d_txavail = n947_txavail;
#ifdef CONFIG_NET_MCASTGROUP
  priv->dev.d_addmac  = n947_addmac;
  priv->dev.d_rmmac   = n947_rmmac;
#endif
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = n947_ioctl;
#endif
  priv->dev.d_private = priv;

  /* Register the network device */

  return netdev_register(&priv->dev, NET_LL_ETHERNET);
}

/****************************************************************************
 * Name: arm_netinitialize
 *
 * Description:
 *   Called by the OS during early initialization to set up the Ethernet
 *   interface.  When CONFIG_NETDEV_LATEINIT is set, the board bringup
 *   code calls n947_netinitialize() directly instead.
 *
 ****************************************************************************/

#if CONFIG_N947_ENET_NETHIFS == 1 && !defined(CONFIG_NETDEV_LATEINIT)
void arm_netinitialize(void)
{
  n947_netinitialize(0);
}
#endif

#endif /* CONFIG_N947_ENET */
