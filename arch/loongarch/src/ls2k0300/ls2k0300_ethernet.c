/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_ethernet.c
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
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>

#include <arch/board/board.h>

#include "loongarch_internal.h"
#include "hardware/ls2k0300_gmac.h"
#include "hardware/ls2k0300_memorymap.h"
#include <arch/barriers.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GMAC Base Addresses (UNCACHED for DMA) */

#define LS2K0300_GMAC0_MAC_BASE    PHYS_TO_UNCACHED(LS2K0300_GMAC0_BASE + 0x0000)
#define LS2K0300_GMAC0_DMA_BASE    PHYS_TO_UNCACHED(LS2K0300_GMAC0_BASE + 0x1000)

/* IRQ Number - GMAC0 interrupt
 *
 * GMAC0 sits on ICU line 49 (the legacy I/O interrupt controller that
 * cascades to CPU IP2).  Use the canonical symbol from
 * arch/loongarch/include/ls2k0300/irq.h rather than a raw number: the
 * NuttX IRQ space is NOT laid out as (32 + LIO_line) like RT-Thread's;
 * EXCCODE_INT_START=64, LOONGARCH_MAX_IRQ=76, LS2K0300_IRQ_ICU_START=78,
 * so GMAC0 resolves to 78 + 49 = 127.  A previous version of this file
 * hardcoded 65, which is LOONGARCH_IRQ_SIP1 (a CPU software interrupt
 * that never fires for GMAC), causing up_enable_irq() to be a no-op and
 * the RX path to never run.
 */

#define LS2K0300_GMAC0_IRQ         LS2K0300_IRQ_GMAC0

/* Default MAC address */

#define LS2K0300_DEFAULT_MAC_ADDR  {0x00, 0x11, 0x22, 0x33, 0x44, 0x55}

/* Work queue support */

#if defined(CONFIG_SCHED_WORKQUEUE)
#  define ls2k0300_workqueue       LPWORK
#else
#  define ls2k0300_workqueue       SYS_WORK
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* DMA Descriptor Structure */

struct ls2k0300_dma_desc_s
{
  uint32_t status;
  uint32_t length;
  uint32_t buffer1;
  uint32_t buffer2;
};

/* LS2K0300 Ethernet Driver Private Data */

struct ls2k0300_driver_s
{
  /* Device structure */

  struct net_driver_s dev;

  /* TX/RX descriptors */

  struct ls2k0300_dma_desc_s tx_desc[LS2K0300_TX_DESC_NUM]
    __attribute__((aligned(16)));
  struct ls2k0300_dma_desc_s rx_desc[LS2K0300_RX_DESC_NUM]
    __attribute__((aligned(16)));

  /* TX/RX buffers */

  uint8_t tx_buffer[LS2K0300_TX_DESC_NUM][LS2K0300_TX_BUF_SIZE]
    __attribute__((aligned(16)));
  uint8_t rx_buffer[LS2K0300_RX_DESC_NUM][LS2K0300_RX_BUF_SIZE]
    __attribute__((aligned(16)));

  /* Descriptor indices */

  uint32_t tx_next;    /* Next TX descriptor to use */
  uint32_t tx_done;    /* TX descriptor that DMA has completed */
  uint32_t rx_idx;     /* Current RX descriptor index */

  /* PHY status */

  uint32_t phy_addr;
  bool link_up;
  uint32_t speed;
  bool duplex;

  /* Interrupt mask */

  uint32_t irq_mask;

  /* Work structure for interrupt handling */

  struct work_s irq_work;

  /* Work structure for deferred TX polling */

  struct work_s tx_pollwork;

  /* Watchdog timer for PHY polling */

  struct wdog_s tx_timeout;

  /* Statistics */

  uint64_t tx_packets;
  uint64_t rx_packets;
  uint64_t tx_errors;
  uint64_t rx_errors;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Driver instance */

static struct ls2k0300_driver_s g_ls2k0300;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register Access */

static inline uint32_t gmac_read(uint64_t base, uint32_t offset);
static inline void gmac_write(uint64_t base, uint32_t offset, uint32_t value);
static inline void gmac_set_bits(uint64_t base, uint32_t offset, uint32_t bits);
static inline void gmac_clear_bits(uint64_t base, uint32_t offset, uint32_t bits);

/* MDIO Operations */

static uint16_t gmac_mdio_read(uint32_t phy_addr, uint32_t reg_addr);
static void gmac_mdio_write(uint32_t phy_addr, uint32_t reg_addr, uint16_t data);

/* DMA Cache Coherency */

/* Assembly function in ls2k0300_dma_cache.S */
extern void ls2k0300_dma_flush(FAR void *addr, size_t len);

/* PHY Operations */

static void ls2k0300_phy_init(struct ls2k0300_driver_s *priv);
static void ls2k0300_phy_poll(struct ls2k0300_driver_s *priv);

/* DMA Operations */

static void ls2k0300_dma_reset(uint64_t dma_base);
static void ls2k0300_tx_desc_init(struct ls2k0300_driver_s *priv);
static void ls2k0300_rx_desc_init(struct ls2k0300_driver_s *priv);

/* Network Driver Interface */

static int ls2k0300_ifup(struct net_driver_s *dev);
static int ls2k0300_ifdown(struct net_driver_s *dev);
static int ls2k0300_txavail(struct net_driver_s *dev);
static int ls2k0300_transmit(struct ls2k0300_driver_s *priv);
static int ls2k0300_txpoll(struct net_driver_s *dev);
static void ls2k0300_txavail_work(void *arg);
#ifdef CONFIG_NET_MCASTGROUP
static int ls2k0300_addmac(struct net_driver_s *dev, const uint8_t *mac);
static int ls2k0300_rmmac(struct net_driver_s *dev, const uint8_t *mac);
#endif

/* Interrupt Handling */

static int ls2k0300_interrupt(int irq, void *context, void *arg);
static void ls2k0300_irq_work_handler(void *arg);
static void ls2k0300_receive_packet(struct ls2k0300_driver_s *priv);
static void ls2k0300_transmit_complete(struct ls2k0300_driver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gmac_read
 *
 * Description:
 *   Read a GMAC register
 *
 ****************************************************************************/

static inline uint32_t gmac_read(uint64_t base, uint32_t offset)
{
  return getreg32(base + offset);
}

/****************************************************************************
 * Name: gmac_write
 *
 * Description:
 *   Write a GMAC register
 *
 ****************************************************************************/

static inline void gmac_write(uint64_t base, uint32_t offset, uint32_t value)
{
  putreg32(value, base + offset);
}

/****************************************************************************
 * Name: gmac_set_bits
 *
 * Description:
 *   Set bits in a GMAC register
 *
 ****************************************************************************/

static inline void gmac_set_bits(uint64_t base, uint32_t offset, uint32_t bits)
{
  uint32_t value = gmac_read(base, offset);
  gmac_write(base, offset, value | bits);
}

/****************************************************************************
 * Name: gmac_clear_bits
 *
 * Description:
 *   Clear bits in a GMAC register
 *
 ****************************************************************************/

static inline void gmac_clear_bits(uint64_t base, uint32_t offset, uint32_t bits)
{
  uint32_t value = gmac_read(base, offset);
  gmac_write(base, offset, value & ~bits);
}

/****************************************************************************
 * Name: gmac_mdio_read
 *
 * Description:
 *   Read a PHY register via MDIO
 *
 ****************************************************************************/

static uint16_t gmac_mdio_read(uint32_t phy_addr, uint32_t reg_addr)
{
  uint64_t mac_base = LS2K0300_GMAC0_MAC_BASE;
  uint32_t addr;
  uint32_t timeout = 10000;

  /* Build the GMII address register value */

  addr = ((phy_addr << GMAC_GMII_ADDR_PA_SHIFT) & GMAC_GMII_ADDR_PA_MASK) |
         ((reg_addr << GMAC_GMII_ADDR_GR_SHIFT) & GMAC_GMII_ADDR_GR_MASK) |
         GMAC_GMII_ADDR_CR_150_250 |
         GMAC_GMII_ADDR_GB;

  /* Write the address register to initiate read */

  gmac_write(mac_base, GMAC_GMII_ADDR, addr);

  /* Wait for the operation to complete */

  while (timeout > 0)
    {
      if ((gmac_read(mac_base, GMAC_GMII_ADDR) & GMAC_GMII_ADDR_GB) == 0)
        {
          break;
        }

      timeout--;
    }

  if (timeout == 0)
    {
      nerr("ERROR: MDIO read timeout\n");
      return 0;
    }

  /* Read the data */

  return (uint16_t)(gmac_read(mac_base, GMAC_GMII_DATA) & 0xFFFF);
}

/****************************************************************************
 * Name: gmac_mdio_write
 *
 * Description:
 *   Write a PHY register via MDIO
 *
 ****************************************************************************/

static void gmac_mdio_write(uint32_t phy_addr, uint32_t reg_addr, uint16_t data)
{
  uint64_t mac_base = LS2K0300_GMAC0_MAC_BASE;
  uint32_t addr;
  uint32_t timeout = 10000;

  /* Write the data first */

  gmac_write(mac_base, GMAC_GMII_DATA, data);

  /* Build the GMII address register value with write bit */

  addr = ((phy_addr << GMAC_GMII_ADDR_PA_SHIFT) & GMAC_GMII_ADDR_PA_MASK) |
         ((reg_addr << GMAC_GMII_ADDR_GR_SHIFT) & GMAC_GMII_ADDR_GR_MASK) |
         GMAC_GMII_ADDR_CR_150_250 |
         GMAC_GMII_ADDR_GW |
         GMAC_GMII_ADDR_GB;

  /* Write the address register to initiate write */

  gmac_write(mac_base, GMAC_GMII_ADDR, addr);

  /* Wait for the operation to complete */

  while (timeout > 0)
    {
      if ((gmac_read(mac_base, GMAC_GMII_ADDR) & GMAC_GMII_ADDR_GB) == 0)
        {
          break;
        }

      timeout--;
    }

  if (timeout == 0)
    {
      nerr("ERROR: MDIO write timeout\n");
    }
}

/****************************************************************************
 * Name: ls2k0300_phy_init
 *
 * Description:
 *   Initialize the PHY
 *
 ****************************************************************************/

static void ls2k0300_phy_init(struct ls2k0300_driver_s *priv)
{
  uint16_t phy_id1;
  uint16_t phy_id2;
  int i;
  uint16_t bmsr;

  /* Scan for PHY at all possible addresses */

  for (i = 0; i < 32; i++)
    {
      phy_id1 = gmac_mdio_read(i, PHY_ID1);
      if (phy_id1 != 0 && phy_id1 != 0xffff)
        {
          phy_id2 = gmac_mdio_read(i, PHY_ID2);
          priv->phy_addr = i;
          break;
        }
    }

  /* Read final PHY ID */

  phy_id1 = gmac_mdio_read(priv->phy_addr, PHY_ID1);
  phy_id2 = gmac_mdio_read(priv->phy_addr, PHY_ID2);
  (void)phy_id1;
  (void)phy_id2;


  /* Check if bootloader already established the link */

  bmsr = gmac_mdio_read(priv->phy_addr, PHY_BMSR);

  if (bmsr & (1 << 2))  /* Bit 2: Link Status */
    {
      //_alert("PHY: link already UP from bootloader, skipping reset\n");
      return;
    }

  /* Reset PHY */

  _alert("PHY: link DOWN, performing full initialization\n");

  gmac_mdio_write(priv->phy_addr, PHY_BMCR, PHY_BMCR_RESET);

  /* Wait for reset to complete */

  up_mdelay(100);

  /* Enable auto-negotiation */

  gmac_mdio_write(priv->phy_addr, PHY_BMCR,
                  PHY_BMCR_AN_ENABLE | PHY_BMCR_RESTART_AN);
}

/****************************************************************************
 * Name: ls2k0300_phy_poll
 *
 * Description:
 *   Poll PHY link status
 *
 ****************************************************************************/

static void ls2k0300_phy_poll(struct ls2k0300_driver_s *priv)
{
  uint32_t rgsmii_status;
  bool link_up;
  uint32_t speed;
  bool duplex;

  /* Read RGMII status register */

  rgsmii_status = gmac_read(LS2K0300_GMAC0_MAC_BASE, GMAC_RGSMII_STATUS);

  link_up = (rgsmii_status & GMAC_RGSMII_LINK) ? true : false;

  /* Check for link state change */

  if (link_up != priv->link_up)
    {
      priv->link_up = link_up;

      if (link_up)
        {
          /* Get speed and duplex */

          speed = (rgsmii_status & GMAC_RGSMII_SPEED_MASK);
          duplex = (rgsmii_status & GMAC_RGSMII_DUPLEX) ? true : false;

          priv->speed = speed;
          priv->duplex = duplex;

          ninfo("Link is UP - Speed: %s, Duplex: %s\n",
                speed == GMAC_RGSMII_SPEED_125 ? "1000Mbps" :
                speed == GMAC_RGSMII_SPEED_25 ? "100Mbps" : "10Mbps",
                duplex ? "Full" : "Half");

          /* Mark the device as running so routing lookups find it */

          priv->dev.d_flags |= IFF_RUNNING;

          /* Update MAC configuration */

          uint32_t config = gmac_read(LS2K0300_GMAC0_MAC_BASE, GMAC_CONFIG);

          /* Set duplex mode */

          if (duplex)
            {
              config |= GMAC_CONFIG_DM;
            }
          else
            {
              config &= ~GMAC_CONFIG_DM;
            }

          /* Set speed */

          if (speed == GMAC_RGSMII_SPEED_25)
            {
              config |= GMAC_CONFIG_FES;
            }
          else
            {
              config &= ~GMAC_CONFIG_FES;
            }

          gmac_write(LS2K0300_GMAC0_MAC_BASE, GMAC_CONFIG, config);
        }
      else
        {
          ninfo("Link is DOWN\n");

          /* Clear the running flag so routing lookups skip this device */

          priv->dev.d_flags &= ~IFF_RUNNING;
        }
    }
}

/****************************************************************************
 * Name: ls2k0300_dma_reset
 *
 * Description:
 *   Reset the DMA engine
 *
 ****************************************************************************/

static void ls2k0300_dma_reset(uint64_t dma_base)
{
  uint32_t timeout = 10000;

  /* Issue software reset */

  gmac_write(dma_base, DMA_BUS_MODE, DMA_BUS_MODE_SWR);

  /* Wait for reset to complete */

  while (timeout > 0)
    {
      if ((gmac_read(dma_base, DMA_BUS_MODE) & DMA_BUS_MODE_SWR) == 0)
        {
          break;
        }

      timeout--;
    }

  if (timeout == 0)
    {
      nerr("ERROR: DMA reset timeout\n");
    }
}

/****************************************************************************
 * Name: ls2k0300_tx_desc_init
 *
 * Description:
 *   Initialize TX descriptors
 *
 ****************************************************************************/

static void ls2k0300_tx_desc_init(struct ls2k0300_driver_s *priv)
{
  int i;

  priv->tx_next = 0;
  priv->tx_done = 0;

  for (i = 0; i < LS2K0300_TX_DESC_NUM; i++)
    {
      priv->tx_desc[i].status = 0;
      priv->tx_desc[i].length = 0;
      priv->tx_desc[i].buffer1 = (uint32_t)(uintptr_t)&priv->tx_buffer[i][0];
      priv->tx_desc[i].buffer2 = 0;

      /* Mark last descriptor with end of ring */

      if (i == LS2K0300_TX_DESC_NUM - 1)
        {
          priv->tx_desc[i].status |= DMA_DESC_TER;
        }
    }

  /* Set TX descriptor base address */

  gmac_write(LS2K0300_GMAC0_DMA_BASE, DMA_TX_BASE_ADDR,
             (uint32_t)(uintptr_t)priv->tx_desc);

  /* Flush descriptors from cache so DMA sees the initialized values */

  ls2k0300_dma_flush(priv->tx_desc,
                    sizeof(priv->tx_desc[0]) * LS2K0300_TX_DESC_NUM);
}

/****************************************************************************
 * Name: ls2k0300_rx_desc_init
 *
 * Description:
 *   Initialize RX descriptors
 *
 ****************************************************************************/

static void ls2k0300_rx_desc_init(struct ls2k0300_driver_s *priv)
{
  int i;

  priv->rx_idx = 0;

  for (i = 0; i < LS2K0300_RX_DESC_NUM; i++)
    {
      priv->rx_desc[i].status = DMA_DESC_OWN;  /* Owned by DMA */
      priv->rx_desc[i].length = LS2K0300_RX_BUF_SIZE;
      priv->rx_desc[i].buffer1 = (uint32_t)(uintptr_t)&priv->rx_buffer[i][0];
      priv->rx_desc[i].buffer2 = 0;

      /* Mark last descriptor with end of ring.  In the Synopsys DWMAC
       * enhanced descriptor format the RX End-of-Ring bit lives in the
       * LENGTH word (bit 15); bit 15 of the STATUS word is the Error
       * Summary, so setting it there (as the previous code did) both
       * leaves the ring without a valid wrap marker and falsely flags
       * desc[N-1] as errored.
       */

      if (i == LS2K0300_RX_DESC_NUM - 1)
        {
          priv->rx_desc[i].length |= DMA_DESC_RER;
        }
    }

  /* Set RX descriptor base address */

  gmac_write(LS2K0300_GMAC0_DMA_BASE, DMA_RX_BASE_ADDR,
             (uint32_t)(uintptr_t)priv->rx_desc);

  /* Flush descriptors from cache so DMA sees the initialized values */

  ls2k0300_dma_flush(priv->rx_desc,
                    sizeof(priv->rx_desc[0]) * LS2K0300_RX_DESC_NUM);
}

/****************************************************************************
 * Name: ls2k0300_receive_packet
 *
 * Description:
 *   Process received packets
 *
 ****************************************************************************/

static void ls2k0300_receive_packet(struct ls2k0300_driver_s *priv)
{
  struct ls2k0300_dma_desc_s *desc;
  uint32_t status;
  uint32_t length;
  struct ether_header *eth;

  /* Process all received packets */

  while (1)
    {
      desc = &priv->rx_desc[priv->rx_idx];

      /* Check if DMA owns this descriptor */

      if (desc->status & DMA_DESC_OWN)
        {
          break;
        }

      /* Get packet status */

      status = desc->status;
      length = (status >> 16) & 0x3FFF;

      /* Check for errors */

      if (status & (1 << 15))  /* Error summary */
        {
          nwarn("WARNING: RX error, status=0x%08x\n", status);
          priv->rx_errors++;
        }
      else
        {
          /* Copy packet to network buffer */

          if (length <= priv->dev.d_pktsize)
            {
              memcpy(priv->dev.d_buf, &priv->rx_buffer[priv->rx_idx][0], length);
              priv->dev.d_len = length;

              /* Get Ethernet header */

              eth = (struct ether_header *)priv->dev.d_buf;

#ifdef CONFIG_NET_PKT
              /* When packet sockets are enabled, feed the frame into the tap */

              pkt_input(&priv->dev);
#endif

              /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
              if (eth->ether_type == HTONS(ETHTYPE_IP))
                {
                  ninfo("IPv4 frame\n");

                  /* Receive an IPv4 packet from the network device */

                  ipv4_input(&priv->dev);

                  /* If the above function invocation resulted in data that
                   * should be sent out on the network, the field d_len will
                   * set to a value > 0.
                   */

                  if (priv->dev.d_len > 0)
                    {
                      ls2k0300_transmit(priv);
                    }
                }
              else
#endif
#ifdef CONFIG_NET_IPv6
              if (eth->ether_type == HTONS(ETHTYPE_IP6))
                {
                  ninfo("IPv6 frame\n");

                  /* Give the IPv6 packet to the network layer */

                  ipv6_input(&priv->dev);

                  /* If the above function invocation resulted in data that
                   * should be sent out on the network, the field d_len will
                   * set to a value > 0.
                   */

                  if (priv->dev.d_len > 0)
                    {
                      ls2k0300_transmit(priv);
                    }
                }
              else
#endif
#ifdef CONFIG_NET_ARP
              if (eth->ether_type == HTONS(ETHTYPE_ARP))
                {
                  arp_input(&priv->dev);

                  /* If the above function invocation resulted in data that
                   * should be sent out on the network, the field d_len will
                   * set to a value > 0.
                   */

                  if (priv->dev.d_len > 0)
                    {
                      ls2k0300_transmit(priv);
                    }
                }
#endif
              priv->rx_packets++;
            }
        }

      /* Return descriptor to DMA */

      desc->status = DMA_DESC_OWN;
      desc->length = LS2K0300_RX_BUF_SIZE;

      /* Mark last descriptor with end of ring (in the length word, see
       * ls2k0300_rx_desc_init for the descriptor-format rationale).
       */

      if (priv->rx_idx == LS2K0300_RX_DESC_NUM - 1)
        {
          desc->length |= DMA_DESC_RER;
        }

      /* Move to next descriptor */

      priv->rx_idx = (priv->rx_idx + 1) % LS2K0300_RX_DESC_NUM;
    }
}

/****************************************************************************
 * Name: ls2k0300_transmit_complete
 *
 * Description:
 *   Process completed transmissions
 *
 ****************************************************************************/

static void ls2k0300_transmit_complete(struct ls2k0300_driver_s *priv)
{
  struct ls2k0300_dma_desc_s *desc;

  /* Process all completed transmissions */

  while (priv->tx_done != priv->tx_next)
    {
      desc = &priv->tx_desc[priv->tx_done];

      /* Check if DMA still owns this descriptor */

      if (desc->status & DMA_DESC_OWN)
        {
          break;
        }

      /* Check for errors */

      if (desc->status & (1 << 15))  /* Error summary */
        {
          nwarn("WARNING: TX error, status=0x%08x\n", desc->status);
          priv->tx_errors++;
        }
      else
        {
          priv->tx_packets++;
        }

      /* Clear descriptor */

      desc->status = 0;
      desc->length = 0;

      /* Mark last descriptor with end of ring */

      if (priv->tx_done == LS2K0300_TX_DESC_NUM - 1)
        {
          desc->status = DMA_DESC_TER;
        }

      /* Move to next descriptor */

      priv->tx_done = (priv->tx_done + 1) % LS2K0300_TX_DESC_NUM;
    }

  /* Now that descriptors have been reclaimed, poll the network stack for
   * any packets that were queued while the ring was full.  The network
   * lock is held by the caller (ls2k0300_irq_work_handler).
   */

  devif_poll(&priv->dev, ls2k0300_txpoll);
}

/****************************************************************************
 * Name: ls2k0300_interrupt
 *
 * Description:
 *   GMAC interrupt handler
 *
 ****************************************************************************/

static int ls2k0300_interrupt(int irq, void *context, void *arg)
{
  struct ls2k0300_driver_s *priv = &g_ls2k0300;
  uint32_t status;

  /* Read interrupt status */

  status = gmac_read(LS2K0300_GMAC0_DMA_BASE, DMA_STATUS);

  if (status == 0)
    {
      return OK;
    }

  /* Clear interrupts */

  gmac_write(LS2K0300_GMAC0_DMA_BASE, DMA_STATUS, status);

  /* Disable interrupts until work is processed */

  gmac_write(LS2K0300_GMAC0_DMA_BASE, DMA_INTERRUPT, 0);

  /* Schedule work to handle interrupts */

  work_queue(ls2k0300_workqueue, &priv->irq_work,
             ls2k0300_irq_work_handler, priv, 0);

  return OK;
}

/****************************************************************************
 * Name: ls2k0300_irq_work_handler
 *
 * Description:
 *   Handle interrupts in work queue context
 *
 ****************************************************************************/

static void ls2k0300_irq_work_handler(void *arg)
{
  struct ls2k0300_driver_s *priv = (struct ls2k0300_driver_s *)arg;

  /* The global network lock must be held for RX processing (ipv4_input /
   * arp_input) and for the devif_poll() call inside transmit_complete.
   */

  net_lock();

  /* Process completed transmissions */

  ls2k0300_transmit_complete(priv);

  /* Process received packets */

  ls2k0300_receive_packet(priv);

  net_unlock();

  /* Re-enable interrupts */

  gmac_write(LS2K0300_GMAC0_DMA_BASE, DMA_INTERRUPT, priv->irq_mask);
}

/****************************************************************************
 * Name: ls2k0300_ifup
 *
 * Description:
 *   Enable the network interface
 *
 ****************************************************************************/

static int ls2k0300_ifup(struct net_driver_s *dev)
{
  struct ls2k0300_driver_s *priv = (struct ls2k0300_driver_s *)dev->d_private;
  uint64_t mac_base = LS2K0300_GMAC0_MAC_BASE;
  uint64_t dma_base = LS2K0300_GMAC0_DMA_BASE;

  ninfo("Bringing up interface\n");

  /* Probe hardware: if the DMA bus mode register reads back 0xFFFFFFFF
   * or 0x00000000, the GMAC hardware is not accessible (e.g., running
   * under QEMU without GMAC emulation).  In that case, skip hardware
   * initialization and just mark the interface as UP.
   */

  uint32_t bus_mode = gmac_read(dma_base, DMA_BUS_MODE);
  if (bus_mode == 0xffffffff || bus_mode == 0x00000000)
    {
      nwarn("WARNING: GMAC hardware not accessible, "
            "using virtual interface mode\n");
      goto skip_hw_init;
    }

  /* Reset DMA */

  ls2k0300_dma_reset(dma_base);

  /* Configure DMA bus mode */

  gmac_write(dma_base, DMA_BUS_MODE,
             DMA_BUS_MODE_PBL_32 | DMA_BUS_MODE_FB | DMA_BUS_MODE_MB);

  /* Configure DMA operation mode */

  gmac_write(dma_base, DMA_CONTROL,
             DMA_CONTROL_OSF | DMA_CONTROL_TSF | DMA_CONTROL_RSF);

  /* Initialize descriptors */

  ls2k0300_tx_desc_init(priv);
  ls2k0300_rx_desc_init(priv);

  /* Set MAC address */

  uint32_t high = ((uint32_t)dev->d_mac.ether.ether_addr_octet[5] << 8) |
                  (uint32_t)dev->d_mac.ether.ether_addr_octet[4];
  uint32_t low = ((uint32_t)dev->d_mac.ether.ether_addr_octet[3] << 24) |
                 ((uint32_t)dev->d_mac.ether.ether_addr_octet[2] << 16) |
                 ((uint32_t)dev->d_mac.ether.ether_addr_octet[1] << 8) |
                 (uint32_t)dev->d_mac.ether.ether_addr_octet[0];

  gmac_write(mac_base, GMAC_ADDR0_HIGH, high);
  gmac_write(mac_base, GMAC_ADDR0_LOW, low);

  /* Configure MAC */

  gmac_write(mac_base, GMAC_CONFIG,
             GMAC_CONFIG_JD | GMAC_CONFIG_WD);

  /* Initialize PHY */

  ls2k0300_phy_init(priv);

  /* Wait for link to come up (auto-negotiation takes 1-3 seconds) */

  {
    int link_timeout = 40;  /* 40 × 100ms = 4 seconds max */

    while (link_timeout > 0)
      {
        up_mdelay(100);
        ls2k0300_phy_poll(priv);
        if (priv->link_up)
          {
            break;
          }

        link_timeout--;
      }

    if (priv->link_up)
      {
        _alert("PHY: link UP after %d00ms\n", 40 - link_timeout);
      }
    else
      {
        _alert("PHY: link still DOWN after 4s timeout\n");
      }
  }

  /* Enable MAC transmitter and receiver */

  gmac_set_bits(mac_base, GMAC_CONFIG, GMAC_CONFIG_TE | GMAC_CONFIG_RE);

  /* Enable DMA transmitter and receiver */

  gmac_set_bits(dma_base, DMA_CONTROL, DMA_CONTROL_ST | DMA_CONTROL_SR);

  /* Set interrupt mask */

  priv->irq_mask = DMA_INT_EN_NIE | DMA_INT_EN_AIE |
                   DMA_INT_EN_RIE | DMA_INT_EN_TIE |
                   DMA_INT_EN_UNE | DMA_INT_EN_RUE;

  /* Enable interrupts */

  gmac_write(dma_base, DMA_INTERRUPT, priv->irq_mask);

  /* Attach interrupt handler */

  irq_attach(LS2K0300_GMAC0_IRQ, ls2k0300_interrupt, priv);
  up_enable_irq(LS2K0300_GMAC0_IRQ);

  /* Poll initial PHY link status */

  ls2k0300_phy_poll(priv);

skip_hw_init:
  ninfo("Interface is up\n");

  return OK;
}

/****************************************************************************
 * Name: ls2k0300_ifdown
 *
 * Description:
 *   Disable the network interface
 *
 ****************************************************************************/

static int ls2k0300_ifdown(struct net_driver_s *dev)
{
  struct ls2k0300_driver_s *priv = (struct ls2k0300_driver_s *)dev->d_private;
  uint64_t mac_base = LS2K0300_GMAC0_MAC_BASE;
  uint64_t dma_base = LS2K0300_GMAC0_DMA_BASE;

  ninfo("Bringing down interface\n");

  /* Disable interrupts */

  gmac_write(dma_base, DMA_INTERRUPT, 0);
  up_disable_irq(LS2K0300_GMAC0_IRQ);

  /* Disable DMA transmitter and receiver */

  gmac_clear_bits(dma_base, DMA_CONTROL, DMA_CONTROL_ST | DMA_CONTROL_SR);

  /* Disable MAC transmitter and receiver */

  gmac_clear_bits(mac_base, GMAC_CONFIG, GMAC_CONFIG_TE | GMAC_CONFIG_RE);

  /* Cancel any pending work */

  work_cancel(ls2k0300_workqueue, &priv->irq_work);
  work_cancel(ls2k0300_workqueue, &priv->tx_pollwork);

  ninfo("Interface is down\n");

  return OK;
}

/****************************************************************************
 * Name: ls2k0300_transmit
 *
 * Description:
 *   Send one packet from dev->d_buf / dev->d_len to the hardware.  Called
 *   both from the devif_poll() callback (ls2k0300_txpoll) for stack-
 *   originated TX, and directly from the RX path for response packets
 *   (where d_buf is already filled by the input handler).
 *
 ****************************************************************************/

static int ls2k0300_transmit(struct ls2k0300_driver_s *priv)
{
  struct net_driver_s *dev = &priv->dev;
  struct ls2k0300_dma_desc_s *desc;

  /* Check if link is up */

  if (!priv->link_up)
    {
      return -ENETUNREACH;
    }

  /* Check if TX descriptor is available */

  desc = &priv->tx_desc[priv->tx_next];

  if (desc->status & DMA_DESC_OWN)
    {
      return -EBUSY;
    }

  /* Copy packet to TX buffer */

  memcpy(&priv->tx_buffer[priv->tx_next][0], dev->d_buf, dev->d_len);

  ninfo("TXMIT: d_len=%d tx_next=%d link_up=%d own=%d\n", dev->d_len,
        priv->tx_next, priv->link_up, (desc->status & DMA_DESC_OWN));

  /* Set up descriptor */

  desc->length = dev->d_len;
  desc->status = DMA_DESC_OWN | DMA_DESC_FS | DMA_DESC_LS | DMA_DESC_IC;

  /* Mark last descriptor with end of ring */

  if (priv->tx_next == LS2K0300_TX_DESC_NUM - 1)
    {
      desc->status |= DMA_DESC_TER;
    }

  /* Ensure descriptor and buffer writes are visible to DMA by flushing the
   * CPU caches.  On LoongArch with write-back caches, the DMA engine reads
   * physical memory which may have stale data if cache lines haven't been
   * written back.  This MUST happen before tx_next is advanced, otherwise
   * we would flush the wrong (next) buffer slot.
   */

  ls2k0300_dma_flush(desc, sizeof(*desc));
  ls2k0300_dma_flush(&priv->tx_buffer[priv->tx_next][0], dev->d_len);

  /* Move to next descriptor */

  priv->tx_next = (priv->tx_next + 1) % LS2K0300_TX_DESC_NUM;

  /* Poll TX demand */

  gmac_write(LS2K0300_GMAC0_DMA_BASE, DMA_TX_POLL_DEMAND, 1);

  return OK;
}

/****************************************************************************
 * Name: ls2k0300_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *
 ****************************************************************************/

static int ls2k0300_txpoll(struct net_driver_s *dev)
{
  struct ls2k0300_driver_s *priv = (struct ls2k0300_driver_s *)dev->d_private;
  int ret = ls2k0300_transmit(priv);

  ninfo("TXPOLL: transmit ret=%d d_len=%d\n", ret, dev->d_len);

  /* Return 0 to continue polling; non-zero to stop (ring full or link down).
   * devif_poll() also stops automatically when no connection stages a
   * packet (d_len == 0).
   */

  return (ret == OK) ? 0 : 1;
}

/****************************************************************************
 * Name: ls2k0300_txavail_work
 *
 * Description:
 *   Perform TX polling on the worker thread.  Takes the global network
 *   lock (required by devif_poll and the stack's connection walk) and
 *   drains any queued packets via devif_poll().
 *
 ****************************************************************************/

static void ls2k0300_txavail_work(void *arg)
{
  struct ls2k0300_driver_s *priv = (struct ls2k0300_driver_s *)arg;

  ninfo("TXAVAIL_WORK: enter link_up=%d\n", priv->link_up);

  net_lock();

  if (priv->link_up)
    {
      devif_poll(&priv->dev, ls2k0300_txpoll);
    }

  net_unlock();

  ninfo("TXAVAIL_WORK: exit\n");
}

/****************************************************************************
 * Name: ls2k0300_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus to perform an out-of-cycle poll and, thereby, reduce TX
 *   latency.  The actual devif_poll() runs on a worker thread so it can
 *   acquire the network lock without deadlocking against the caller
 *   (e.g. arp_send, which holds netdev_lock and releases it while waiting
 *   on its completion semaphore).
 *
 ****************************************************************************/

static int ls2k0300_txavail(struct net_driver_s *dev)
{
  struct ls2k0300_driver_s *priv = (struct ls2k0300_driver_s *)dev->d_private;

  ninfo("TXAVAIL: enter link_up=%d avail=%d\n", priv->link_up,
        work_available(&priv->tx_pollwork));

  if (work_available(&priv->tx_pollwork))
    {
      work_queue(ls2k0300_workqueue, &priv->tx_pollwork,
                 ls2k0300_txavail_work, priv, 0);
    }

  return OK;
}

#ifdef CONFIG_NET_MCASTGROUP
/****************************************************************************
 * Name: ls2k0300_addmac
 *
 * Description:
 *   Add a multicast MAC address
 *
 ****************************************************************************/

static int ls2k0300_addmac(struct net_driver_s *dev, const uint8_t *mac)
{
  /* TODO: Implement multicast address filtering */

  return OK;
}

/****************************************************************************
 * Name: ls2k0300_rmmac
 *
 * Description:
 *   Remove a multicast MAC address
 *
 ****************************************************************************/

static int ls2k0300_rmmac(struct net_driver_s *dev, const uint8_t *mac)
{
  /* TODO: Implement multicast address filtering */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ls2k0300_ethernet_initialize
 *
 * Description:
 *   Initialize the LS2K0300 Ethernet driver
 *
 ****************************************************************************/

int ls2k0300_ethernet_initialize(void)
{
  struct ls2k0300_driver_s *priv = &g_ls2k0300;
  static const uint8_t default_mac[6] = LS2K0300_DEFAULT_MAC_ADDR;

  ninfo("Initializing LS2K0300 Ethernet driver\n");

  /* Initialize driver structure */

  memset(priv, 0, sizeof(struct ls2k0300_driver_s));

  /* Set default MAC address */

  memcpy(priv->dev.d_mac.ether.ether_addr_octet, default_mac, 6);

  /* Set PHY address */

  priv->phy_addr = LS2K0300_DEFAULT_PHY_ADDR;

  /* Set device parameters */

  priv->dev.d_pktsize = LS2K0300_TX_BUF_SIZE;
  priv->dev.d_lltype = NET_LL_ETHERNET;

  /* Allocate device buffer for RX packet processing */

  priv->dev.d_buf = kmm_malloc(LS2K0300_TX_BUF_SIZE + CONFIG_NET_GUARDSIZE);
  if (priv->dev.d_buf == NULL)
    {
      serr("ERROR: Failed to allocate device buffer\n");
      return -ENOMEM;
    }

  /* Set driver callbacks */

  priv->dev.d_ifup = ls2k0300_ifup;
  priv->dev.d_ifdown = ls2k0300_ifdown;
  priv->dev.d_txavail = ls2k0300_txavail;
#ifdef CONFIG_NET_MCASTGROUP
  priv->dev.d_addmac = ls2k0300_addmac;
  priv->dev.d_rmmac = ls2k0300_rmmac;
#endif

  priv->dev.d_private = priv;

  /* Register the network device */

  netdev_register(&priv->dev, NET_LL_ETHERNET);

  ninfo("LS2K0300 Ethernet driver initialized\n");

  return OK;
}

/****************************************************************************
 * Name: loongarch_netinitialize
 *
 * Description:
 *   Initialize the network device for LoongArch architecture
 *
 ****************************************************************************/

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void loongarch_netinitialize(void)
{
  ls2k0300_ethernet_initialize();
}
#endif
