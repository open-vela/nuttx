/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_eth.c
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
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "arm64_internal.h"
#include "hardware/rk3576_eth.h"
#include "rk3576_eth.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Ethernet timeout (microseconds) */

#define ETH_TIMEOUT_US              100000
#define ETH_RESET_TIMEOUT_US        10000
#define ETH_MDIO_TIMEOUT            10000

/* MDIO read/write bits */

#define MDIO_GMII_BUSY              (1 << 31)
#define MDIO_GMII_WRITE             (1 << 26)
#define MDIO_GMII_READ              (0 << 26)
#define MDIO_GMII_CLK_DIV           (15 << 0)

/* Default MAC address */

static const uint8_t g_default_mac[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Ethernet controller base addresses */

const uint32_t g_eth_base[RK3576_ETH_COUNT] =
{
  RK3576_ETH0_ADDR,
  RK3576_ETH1_ADDR,
};

/* Per-controller state */

struct rk3576_eth_dev_s
{
  int iface;                    /* Interface type (MII/RMII/RGMII) */
  int speed;                    /* Speed (10/100/1000 Mbps) */
  int duplex;                   /* Half/Full duplex */
  uint8_t mac[6];              /* MAC address */
  bool link_up;                 /* Link status */
  bool initialized;             /* Initialized flag */
};

static struct rk3576_eth_dev_s g_eth_dev[RK3576_ETH_COUNT];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_eth_validate
 *
 * Description:
 *   Validate Ethernet port number.
 *
 ****************************************************************************/

static inline int rk3576_eth_validate(int port)
{
  if (port < 0 || port >= RK3576_ETH_COUNT)
    {
      nerr("ETH: invalid port %d\n", port);
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: rk3576_eth_wait_reset
 *
 * Description:
 *   Wait for DMA reset to complete.
 *
 ****************************************************************************/

static int rk3576_eth_wait_reset(int port)
{
  uint32_t base = g_eth_base[port];
  int timeout = ETH_RESET_TIMEOUT_US;

  while (timeout--)
    {
      if (!(getreg32(base + ETH_DMA_BUS_MODE) & ETH_DMA_BUS_MODE_SFT_RESET))
        {
          return OK;
        }

      up_udelay(1);
    }

  nerr("ETH%d: reset timeout\n", port);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_eth_mdio_wait_busy
 *
 * Description:
 *   Wait for MDIO (GMII) to become idle.
 *
 ****************************************************************************/

static int rk3576_eth_mdio_wait_busy(int port)
{
  uint32_t base = g_eth_base[port];
  int timeout = ETH_MDIO_TIMEOUT;

  while (timeout--)
    {
      if (!(getreg32(base + ETH_GMII_ADDR) & MDIO_GMII_BUSY))
        {
          return OK;
        }

      up_udelay(1);
    }

  nerr("ETH%d: MDIO busy timeout\n", port);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_eth_set_mac_filter
 *
 * Description:
 *   Configure MAC address filter.
 *
 ****************************************************************************/

static void rk3576_eth_set_mac_filter(int port)
{
  uint32_t base = g_eth_base[port];

  /* Set MAC address into MAC address filter registers (DWC offset 0x400) */

  putreg32((uint32_t)g_eth_dev[port].mac[0] |
           ((uint32_t)g_eth_dev[port].mac[1] << 8) |
           ((uint32_t)g_eth_dev[port].mac[2] << 16) |
           ((uint32_t)g_eth_dev[port].mac[3] << 24),
           base + 0x400);

  putreg32((uint32_t)g_eth_dev[port].mac[4] |
           ((uint32_t)g_eth_dev[port].mac[5] << 8),
           base + 0x404);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_eth_init
 *
 * Description:
 *   Initialize Ethernet controller.
 *
 ****************************************************************************/

void rk3576_eth_init(int port)
{
  uint32_t base;
  int ret;

  if (rk3576_eth_validate(port) < 0)
    {
      return;
    }

  base = g_eth_base[port];

  /* Initialize state */

  memset(&g_eth_dev[port], 0, sizeof(struct rk3576_eth_dev_s));
  memcpy(g_eth_dev[port].mac, g_default_mac, 6);
  g_eth_dev[port].iface = RK3576_ETH_IF_RGMII;
  g_eth_dev[port].speed = RK3576_ETH_SPEED_100M;
  g_eth_dev[port].duplex = RK3576_ETH_FULL_DUPLEX;

  /* Disable all interrupts */

  putreg32(0, base + ETH_DMA_INT_ENABLE);
  putreg32(0xffffffff, base + ETH_DMA_INT_STATUS);

  /* Software reset */

  putreg32(ETH_DMA_BUS_MODE_SFT_RESET, base + ETH_DMA_BUS_MODE);

  ret = rk3576_eth_wait_reset(port);
  if (ret < 0)
    {
      nerr("ETH%d: init failed\n", port);
      return;
    }

  /* Configure DMA bus mode:
   * - PBL = 32
   * - Descriptor skip length = 0
   */

  putreg32(ETH_DMA_BUS_MODE_PBL_32, base + ETH_DMA_BUS_MODE);

  /* Configure MAC:
   * - Interface mode (MII/RMII/RGMII)
   * - Speed and duplex
   */

  {
    uint32_t mac_config = 0;

    /* Set interface mode */

    switch (g_eth_dev[port].iface)
      {
        case RK3576_ETH_IF_MII:
          mac_config |= ETH_MAC_CONFIG_MODE_MII;
          break;
        case RK3576_ETH_IF_RMII:
          mac_config |= ETH_MAC_CONFIG_MODE_RMII;
          break;
        case RK3576_ETH_IF_RGMII:
          mac_config |= ETH_MAC_CONFIG_MODE_RGMII;
          break;
      }

    /* Set speed */

    if (g_eth_dev[port].speed == RK3576_ETH_SPEED_100M ||
        g_eth_dev[port].speed == RK3576_ETH_SPEED_1000M)
      {
        mac_config |= ETH_MAC_CONFIG_FES;
      }

    /* Set duplex */

    if (g_eth_dev[port].duplex == RK3576_ETH_FULL_DUPLEX)
      {
        mac_config |= ETH_MAC_CONFIG_DM;
      }

    putreg32(mac_config, base + ETH_MAC_CONFIG);
  }

  /* Set MAC address filter */

  rk3576_eth_set_mac_filter(port);

  /* Clear all interrupts */

  putreg32(0xffffffff, base + ETH_INT_STATUS);
  putreg32(0xffffffff, base + ETH_DMA_INT_STATUS);

  /* Enable DMA interrupts */

  putreg32(ETH_DMA_INT_TI | ETH_DMA_INT_RI | ETH_DMA_INT_NIS,
           base + ETH_DMA_INT_ENABLE);

  g_eth_dev[port].initialized = true;

  ninfo("ETH%d: initialized (MAC: %02x:%02x:%02x:%02x:%02x:%02x)\n",
        port,
        g_eth_dev[port].mac[0], g_eth_dev[port].mac[1],
        g_eth_dev[port].mac[2], g_eth_dev[port].mac[3],
        g_eth_dev[port].mac[4], g_eth_dev[port].mac[5]);
}

/****************************************************************************
 * Name: rk3576_eth_set_interface
 *
 * Description:
 *   Set Ethernet interface type (MII/RMII/RGMII).
 *
 ****************************************************************************/

int rk3576_eth_set_interface(int port, int iface)
{
  int ret;

  ret = rk3576_eth_validate(port);
  if (ret < 0)
    {
      return ret;
    }

  g_eth_dev[port].iface = iface;

  /* Update MAC config */

  {
    uint32_t base = g_eth_base[port];
    uint32_t mac_config = getreg32(base + ETH_MAC_CONFIG);
    mac_config &= ~ETH_MAC_CONFIG_MODE_MASK;

    switch (iface)
      {
        case RK3576_ETH_IF_MII:
          mac_config |= ETH_MAC_CONFIG_MODE_MII;
          break;
        case RK3576_ETH_IF_RMII:
          mac_config |= ETH_MAC_CONFIG_MODE_RMII;
          break;
        case RK3576_ETH_IF_RGMII:
          mac_config |= ETH_MAC_CONFIG_MODE_RGMII;
          break;
        default:
          return -EINVAL;
      }

    putreg32(mac_config, base + ETH_MAC_CONFIG);
  }

  ninfo("ETH%d: interface set to %d\n", port, iface);
  return OK;
}

/****************************************************************************
 * Name: rk3576_eth_set_speed
 *
 * Description:
 *   Set Ethernet speed (10/100/1000 Mbps).
 *
 ****************************************************************************/

int rk3576_eth_set_speed(int port, int speed)
{
  int ret;

  ret = rk3576_eth_validate(port);
  if (ret < 0)
    {
      return ret;
    }

  g_eth_dev[port].speed = speed;

  /* Update MAC config */

  {
    uint32_t base = g_eth_base[port];
    uint32_t mac_config = getreg32(base + ETH_MAC_CONFIG);
    mac_config &= ~ETH_MAC_CONFIG_FES;

    if (speed == RK3576_ETH_SPEED_100M ||
        speed == RK3576_ETH_SPEED_1000M)
      {
        mac_config |= ETH_MAC_CONFIG_FES;
      }

    putreg32(mac_config, base + ETH_MAC_CONFIG);
  }

  ninfo("ETH%d: speed set to %d\n", port, speed);
  return OK;
}

/****************************************************************************
 * Name: rk3576_eth_set_duplex
 *
 * Description:
 *   Set Ethernet duplex mode (half/full).
 *
 ****************************************************************************/

int rk3576_eth_set_duplex(int port, int duplex)
{
  int ret;

  ret = rk3576_eth_validate(port);
  if (ret < 0)
    {
      return ret;
    }

  g_eth_dev[port].duplex = duplex;

  /* Update MAC config */

  {
    uint32_t base = g_eth_base[port];
    uint32_t mac_config = getreg32(base + ETH_MAC_CONFIG);
    mac_config &= ~ETH_MAC_CONFIG_DM;

    if (duplex == RK3576_ETH_FULL_DUPLEX)
      {
        mac_config |= ETH_MAC_CONFIG_DM;
      }

    putreg32(mac_config, base + ETH_MAC_CONFIG);
  }

  ninfo("ETH%d: duplex set to %s\n", port,
        duplex == 0 ? "half" : "full");
  return OK;
}

/****************************************************************************
 * Name: rk3576_eth_set_macaddr
 *
 * Description:
 *   Set Ethernet MAC address.
 *
 ****************************************************************************/

int rk3576_eth_set_macaddr(int port, const uint8_t *mac)
{
  int ret;

  ret = rk3576_eth_validate(port);
  if (ret < 0)
    {
      return ret;
    }

  if (mac == NULL)
    {
      return -EINVAL;
    }

  memcpy(g_eth_dev[port].mac, mac, 6);
  rk3576_eth_set_mac_filter(port);

  ninfo("ETH%d: MAC set to %02x:%02x:%02x:%02x:%02x:%02x\n",
        port, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return OK;
}

/****************************************************************************
 * Name: rk3576_eth_mdio_read
 *
 * Description:
 *   Read a PHY register via MDIO.
 *
 ****************************************************************************/

int rk3576_eth_mdio_read(int port, int phy_addr, int reg_addr)
{
  uint32_t base;
  uint32_t gmii_addr;
  int ret;
  int timeout;

  ret = rk3576_eth_validate(port);
  if (ret < 0)
    {
      return ret;
    }

  base = g_eth_base[port];

  /* Wait for MDIO to be idle */

  ret = rk3576_eth_mdio_wait_busy(port);
  if (ret < 0)
    {
      return ret;
    }

  /* Set up GMII address register:
   * - PHY address
   * - Register address
   * - Read operation
   * - Clock divider
   * - Busy bit
   */

  gmii_addr = MDIO_GMII_BUSY | MDIO_GMII_READ | MDIO_GMII_CLK_DIV |
              ((phy_addr << 21) & (0x1f << 21)) |
              ((reg_addr << 16) & (0x1f << 16));

  putreg32(gmii_addr, base + ETH_GMII_ADDR);

  /* Wait for read to complete */

  timeout = ETH_MDIO_TIMEOUT;
  while (timeout--)
    {
      if (!(getreg32(base + ETH_GMII_ADDR) & MDIO_GMII_BUSY))
        {
          return (int)(getreg32(base + ETH_GMII_DATA) & 0xffff);
        }

      up_udelay(1);
    }

  nerr("ETH%d: MDIO read timeout\n", port);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_eth_mdio_write
 *
 * Description:
 *   Write a PHY register via MDIO.
 *
 ****************************************************************************/

int rk3576_eth_mdio_write(int port, int phy_addr, int reg_addr, uint16_t val)
{
  uint32_t base;
  uint32_t gmii_addr;
  int ret;
  int timeout;

  ret = rk3576_eth_validate(port);
  if (ret < 0)
    {
      return ret;
    }

  base = g_eth_base[port];

  /* Wait for MDIO to be idle */

  ret = rk3576_eth_mdio_wait_busy(port);
  if (ret < 0)
    {
      return ret;
    }

  /* Write data */

  putreg32(val, base + ETH_GMII_DATA);

  /* Set up GMII address register:
   * - PHY address
   * - Register address
   * - Write operation
   * - Clock divider
   * - Busy bit
   */

  gmii_addr = MDIO_GMII_BUSY | MDIO_GMII_WRITE | MDIO_GMII_CLK_DIV |
              ((phy_addr << 21) & (0x1f << 21)) |
              ((reg_addr << 16) & (0x1f << 16));

  putreg32(gmii_addr, base + ETH_GMII_ADDR);

  /* Wait for write to complete */

  timeout = ETH_MDIO_TIMEOUT;
  while (timeout--)
    {
      if (!(getreg32(base + ETH_GMII_ADDR) & MDIO_GMII_BUSY))
        {
          return OK;
        }

      up_udelay(1);
    }

  nerr("ETH%d: MDIO write timeout\n", port);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_eth_start
 *
 * Description:
 *   Start Ethernet transmission and reception.
 *
 ****************************************************************************/

void rk3576_eth_start(int port)
{
  uint32_t base;
  uint32_t mac_config;

  if (rk3576_eth_validate(port) < 0)
    {
      return;
    }

  base = g_eth_base[port];

  /* Enable MAC transmitter and receiver */

  mac_config = getreg32(base + ETH_MAC_CONFIG);
  mac_config |= ETH_MAC_CONFIG_TE | ETH_MAC_CONFIG_RE;
  putreg32(mac_config, base + ETH_MAC_CONFIG);

  /* Enable DMA transmission and reception */

  putreg32(ETH_DMA_CONTROL_ST | ETH_DMA_CONTROL_SR,
           base + ETH_DMA_CONTROL);

  ninfo("ETH%d: started\n", port);
}

/****************************************************************************
 * Name: rk3576_eth_stop
 *
 * Description:
 *   Stop Ethernet transmission and reception.
 *
 ****************************************************************************/

void rk3576_eth_stop(int port)
{
  uint32_t base;
  uint32_t mac_config;

  if (rk3576_eth_validate(port) < 0)
    {
      return;
    }

  base = g_eth_base[port];

  /* Disable DMA transmission and reception */

  putreg32(0, base + ETH_DMA_CONTROL);

  /* Disable MAC transmitter and receiver */

  mac_config = getreg32(base + ETH_MAC_CONFIG);
  mac_config &= ~(ETH_MAC_CONFIG_TE | ETH_MAC_CONFIG_RE);
  putreg32(mac_config, base + ETH_MAC_CONFIG);

  ninfo("ETH%d: stopped\n", port);
}

/****************************************************************************
 * Name: rk3576_eth_is_link_up
 *
 * Description:
 *   Check if Ethernet link is up.
 *
 ****************************************************************************/

int rk3576_eth_is_link_up(int port)
{
  if (rk3576_eth_validate(port) < 0)
    {
      return -EINVAL;
    }

  return g_eth_dev[port].link_up ? 1 : 0;
}

/****************************************************************************
 * Name: rk3576_eth_get_speed
 *
 * Description:
 *   Get current Ethernet speed.
 *
 ****************************************************************************/

int rk3576_eth_get_speed(int port)
{
  if (rk3576_eth_validate(port) < 0)
    {
      return -EINVAL;
    }

  return g_eth_dev[port].speed;
}

/****************************************************************************
 * Name: rk3576_eth_get_duplex
 *
 * Description:
 *   Get current Ethernet duplex mode.
 *
 ****************************************************************************/

int rk3576_eth_get_duplex(int port)
{
  if (rk3576_eth_validate(port) < 0)
    {
      return -EINVAL;
    }

  return g_eth_dev[port].duplex;
}
