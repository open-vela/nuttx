/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_gmac.c
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
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "loongarch_internal.h"
#include "hardware/ls2k0300_gmac.h"
#include "hardware/ls2k0300_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GMAC Base Addresses (UNCACHED for DMA) */

#define LS2K0300_GMAC0_MAC_BASE    PHYS_TO_UNCACHED(LS2K0300_GMAC0_BASE + 0x0000)
#define LS2K0300_GMAC0_DMA_BASE    PHYS_TO_UNCACHED(LS2K0300_GMAC0_BASE + 0x1000)

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

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register Access */

static inline uint32_t gmac_read(uint64_t base, uint32_t offset);
static inline void gmac_write(uint64_t base, uint32_t offset,
                              uint32_t value);
static inline void gmac_set_bits(uint64_t base, uint32_t offset,
                                 uint32_t bits);
static inline void gmac_clear_bits(uint64_t base, uint32_t offset,
                                    uint32_t bits);

/* MDIO Operations */

static uint16_t gmac_mdio_read(uint64_t mac_base, uint32_t phy_addr,
                               uint32_t reg_addr);
static void gmac_mdio_write(uint64_t mac_base, uint32_t phy_addr,
                            uint32_t reg_addr, uint16_t data);

/* MAC Operations */

static void gmac_mac_reset(uint64_t mac_base);
static void gmac_mac_set_address(uint64_t mac_base, const uint8_t *mac_addr);
static void gmac_mac_enable_tx(uint64_t mac_base);
static void gmac_mac_enable_rx(uint64_t mac_base);

/* DMA Operations */

static void gmac_dma_reset(uint64_t dma_base);
static void gmac_dma_enable_tx(uint64_t dma_base);
static void gmac_dma_enable_rx(uint64_t dma_base);

/* Interrupt Operations */

static void gmac_irq_clear(uint64_t dma_base, uint32_t mask);
static uint32_t gmac_irq_status(uint64_t dma_base);

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

static inline void gmac_set_bits(uint64_t base, uint32_t offset,
                                 uint32_t bits)
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

static inline void gmac_clear_bits(uint64_t base, uint32_t offset,
                                    uint32_t bits)
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

static uint16_t gmac_mdio_read(uint64_t mac_base, uint32_t phy_addr,
                               uint32_t reg_addr)
{
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

  return (uint16_t)(gmac_read(mac_base, GMAC_GMII_DATA) & 0xffff);
}

/****************************************************************************
 * Name: gmac_mdio_write
 *
 * Description:
 *   Write a PHY register via MDIO
 *
 ****************************************************************************/

static void gmac_mdio_write(uint64_t mac_base, uint32_t phy_addr,
                            uint32_t reg_addr, uint16_t data)
{
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
 * Name: gmac_mac_reset
 *
 * Description:
 *   Reset the MAC
 *
 ****************************************************************************/

static void gmac_mac_reset(uint64_t mac_base)
{
  /* Reset transmitter */

  gmac_clear_bits(mac_base, GMAC_CONFIG, GMAC_CONFIG_TE);

  /* Reset receiver */

  gmac_clear_bits(mac_base, GMAC_CONFIG, GMAC_CONFIG_RE);

  /* Wait for reset to complete */

  up_mdelay(10);
}

/****************************************************************************
 * Name: gmac_mac_set_address
 *
 * Description:
 *   Set the MAC address
 *
 ****************************************************************************/

static void gmac_mac_set_address(uint64_t mac_base, const uint8_t *mac_addr)
{
  uint32_t high;
  uint32_t low;

  /* Build the high register (bytes 4-5) */

  high = ((uint32_t)mac_addr[5] << 8) | (uint32_t)mac_addr[4];

  /* Build the low register (bytes 0-3) */

  low = ((uint32_t)mac_addr[3] << 24) |
        ((uint32_t)mac_addr[2] << 16) |
        ((uint32_t)mac_addr[1] << 8) |
        (uint32_t)mac_addr[0];

  /* Write the address registers */

  gmac_write(mac_base, GMAC_ADDR0_HIGH, high);
  gmac_write(mac_base, GMAC_ADDR0_LOW, low);
}

/****************************************************************************
 * Name: gmac_mac_enable_tx
 *
 * Description:
 *   Enable MAC transmitter
 *
 ****************************************************************************/

static void gmac_mac_enable_tx(uint64_t mac_base)
{
  gmac_set_bits(mac_base, GMAC_CONFIG, GMAC_CONFIG_TE);
}

/****************************************************************************
 * Name: gmac_mac_enable_rx
 *
 * Description:
 *   Enable MAC receiver
 *
 ****************************************************************************/

static void gmac_mac_enable_rx(uint64_t mac_base)
{
  gmac_set_bits(mac_base, GMAC_CONFIG, GMAC_CONFIG_RE);
}

/****************************************************************************
 * Name: gmac_dma_reset
 *
 * Description:
 *   Reset the DMA engine
 *
 ****************************************************************************/

static void gmac_dma_reset(uint64_t dma_base)
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
 * Name: gmac_dma_enable_tx
 *
 * Description:
 *   Enable DMA transmitter
 *
 ****************************************************************************/

static void gmac_dma_enable_tx(uint64_t dma_base)
{
  gmac_set_bits(dma_base, DMA_CONTROL, DMA_CONTROL_ST);
}

/****************************************************************************
 * Name: gmac_dma_enable_rx
 *
 * Description:
 *   Enable DMA receiver
 *
 ****************************************************************************/

static void gmac_dma_enable_rx(uint64_t dma_base)
{
  gmac_set_bits(dma_base, DMA_CONTROL, DMA_CONTROL_SR);
}

/****************************************************************************
 * Name: gmac_irq_clear
 *
 * Description:
 *   Clear GMAC interrupts
 *
 ****************************************************************************/

static void gmac_irq_clear(uint64_t dma_base, uint32_t mask)
{
  gmac_write(dma_base, DMA_STATUS, mask);
}

/****************************************************************************
 * Name: gmac_irq_status
 *
 * Description:
 *   Get GMAC interrupt status
 *
 ****************************************************************************/

static uint32_t gmac_irq_status(uint64_t dma_base)
{
  return gmac_read(dma_base, DMA_STATUS);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ls2k0300_gmac_initialize
 *
 * Description:
 *   Initialize the GMAC hardware
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int ls2k0300_gmac_initialize(void)
{
  uint64_t mac_base = LS2K0300_GMAC0_MAC_BASE;
  uint64_t dma_base = LS2K0300_GMAC0_DMA_BASE;

  ninfo("Initializing LS2K0300 GMAC\n");

  /* Reset the MAC */

  gmac_mac_reset(mac_base);

  /* Reset the DMA */

  gmac_dma_reset(dma_base);

  /* Configure DMA bus mode */

  gmac_write(dma_base, DMA_BUS_MODE,
             DMA_BUS_MODE_PBL_32 | DMA_BUS_MODE_FB | DMA_BUS_MODE_MB);

  /* Configure DMA operation mode */

  gmac_write(dma_base, DMA_CONTROL,
             DMA_CONTROL_OSF | DMA_CONTROL_TSF | DMA_CONTROL_RSF);

  /* Enable MAC transmitter and receiver */

  gmac_mac_enable_tx(mac_base);
  gmac_mac_enable_rx(mac_base);

  /* Enable DMA transmitter and receiver */

  gmac_dma_enable_tx(dma_base);
  gmac_dma_enable_rx(dma_base);

  ninfo("LS2K0300 GMAC initialized\n");

  return OK;
}

/****************************************************************************
 * Name: ls2k0300_gmac_set_mac_address
 *
 * Description:
 *   Set the MAC address
 *
 * Input Parameters:
 *   mac_addr - Pointer to 6-byte MAC address
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ls2k0300_gmac_set_mac_address(const uint8_t *mac_addr)
{
  gmac_mac_set_address(LS2K0300_GMAC0_MAC_BASE, mac_addr);
}

/****************************************************************************
 * Name: ls2k0300_gmac_phy_read
 *
 * Description:
 *   Read a PHY register
 *
 * Input Parameters:
 *   phy_addr - PHY address
 *   reg_addr - Register address
 *
 * Returned Value:
 *   Register value
 *
 ****************************************************************************/

uint16_t ls2k0300_gmac_phy_read(uint32_t phy_addr, uint32_t reg_addr)
{
  return gmac_mdio_read(LS2K0300_GMAC0_MAC_BASE, phy_addr, reg_addr);
}

/****************************************************************************
 * Name: ls2k0300_gmac_phy_write
 *
 * Description:
 *   Write a PHY register
 *
 * Input Parameters:
 *   phy_addr - PHY address
 *   reg_addr - Register address
 *   data     - Data to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ls2k0300_gmac_phy_write(uint32_t phy_addr, uint32_t reg_addr,
                            uint16_t data)
{
  gmac_mdio_write(LS2K0300_GMAC0_MAC_BASE, phy_addr, reg_addr, data);
}

/****************************************************************************
 * Name: ls2k0300_gmac_get_link_status
 *
 * Description:
 *   Get the link status from RGMII status register
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Link status (1 = up, 0 = down)
 *
 ****************************************************************************/

int ls2k0300_gmac_get_link_status(void)
{
  uint32_t status;

  status = gmac_read(LS2K0300_GMAC0_MAC_BASE, GMAC_RGSMII_STATUS);

  return (status & GMAC_RGSMII_LINK) ? 1 : 0;
}

/****************************************************************************
 * Name: ls2k0300_gmac_irq_handler
 *
 * Description:
 *   GMAC interrupt handler
 *
 * Input Parameters:
 *   irq     - Interrupt number
 *   context - Interrupt context
 *   arg     - Argument
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

int ls2k0300_gmac_irq_handler(int irq, void *context, void *arg)
{
  uint64_t dma_base = LS2K0300_GMAC0_DMA_BASE;
  uint32_t status;

  /* Read interrupt status */

  status = gmac_irq_status(dma_base);

  if (status == 0)
    {
      return OK;
    }

  /* Clear interrupts */

  gmac_irq_clear(dma_base, status);

  /* Handle transmit interrupt */

  if (status & DMA_STATUS_TI)
    {
      ninfo("TX interrupt\n");
    }

  /* Handle receive interrupt */

  if (status & DMA_STATUS_RI)
    {
      ninfo("RX interrupt\n");
    }

  /* Handle error interrupts */

  if (status & DMA_STATUS_FBI)
    {
      nerr("ERROR: Fatal bus error\n");
    }

  if (status & DMA_STATUS_UNF)
    {
      nerr("ERROR: Transmit underflow\n");
    }

  if (status & DMA_STATUS_OVF)
    {
      nerr("ERROR: Receive overflow\n");
    }

  return OK;
}
