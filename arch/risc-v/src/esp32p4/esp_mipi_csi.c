/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_mipi_csi.c
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>

#include "esp_mipi_csi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register base addresses */

#define MIPI_CSI_HOST_BASE    0x500d0000
#define MIPI_CSI_BRIDGE_BASE  0x500d1000
#define HP_SYS_CLKRST_BASE   0x500e6000

/* PMU LDO register for MIPI PHY power */

#define PMU_BASE              0x50115000
#define PMU_HP_LDO_CTRL0_REG (PMU_BASE + 0x01c0)

/* LDO channel 3 (unit 2) register addresses.
 * index_array[4] = {0, 3, 1, 4}, unit 2 → index 1 → ext_ldo[1] (P0_0P2A)
 */

#define PMU_EXT_LDO_P0_0P2A_CTRL  (PMU_BASE + 0x1c0)
#define PMU_EXT_LDO_P0_0P2A_ANA   (PMU_BASE + 0x1c4)

/* Bit positions in ext_ldo control register */

#define EXT_LDO_FORCE_TIEH_SEL_BIT  (1 << 7)
#define EXT_LDO_XPD_BIT             (1 << 8)
#define EXT_LDO_TIEH_SEL_SHIFT      9
#define EXT_LDO_TIEH_SEL_MASK       (0x7 << EXT_LDO_TIEH_SEL_SHIFT)
#define EXT_LDO_TIEH_BIT            (1 << 14)

/* Bit positions in ext_ldo ANA register */

#define EXT_LDO_ANA_MUL_SHIFT       23
#define EXT_LDO_ANA_MUL_MASK        (0x7 << EXT_LDO_ANA_MUL_SHIFT)
#define EXT_LDO_ANA_DREF_SHIFT      28
#define EXT_LDO_ANA_DREF_MASK       (0xFU << EXT_LDO_ANA_DREF_SHIFT)

/* Voltage parameters for 2500mV (computed from ldo_ll algorithm) */

#define MIPI_PHY_LDO_DREF           13
#define MIPI_PHY_LDO_MUL            3

/* Simplified register access macros */

#define REG_WRITE(addr, val)  (*((volatile uint32_t *)(addr)) = (val))
#define REG_READ(addr)        (*((volatile uint32_t *)(addr)))
#define REG_SET_BIT(addr, b)  REG_WRITE((addr), REG_READ(addr) | (b))
#define REG_CLR_BIT(addr, b)  REG_WRITE((addr), REG_READ(addr) & ~(b))

/* HP_SYS_CLKRST offsets */

#define SOC_CLK_CTRL1_OFF     0x0010
#define HP_RST_EN0_OFF        0x0030
#define PERI_CLK_CTRL03_OFF   0x0058

/* Bits in SOC_CLK_CTRL1 */

#define CSI_HOST_SYS_CLK_EN  (1 << 26)
#define CSI_BRG_SYS_CLK_EN   (1 << 27)

/* Bits in HP_RST_EN0 */

#define RST_EN_CSI_HOST       (1 << 16)
#define RST_EN_CSI_BRG        (1 << 17)

/* Bits in PERI_CLK_CTRL03 */

#define CSI_DPHY_CLK_SRC_MASK (3 << 0)
#define CSI_DPHY_CFG_CLK_EN   (1 << 2)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp_csi_dev_s g_csi_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_csi_ldo_enable
 *
 * Description:
 *   Enable LDO channel 3 for MIPI PHY at 2.5V.
 *   Note: This is a simplified implementation. The actual LDO config
 *   depends on the PMU register layout.
 ****************************************************************************/

static void esp_csi_ldo_enable(void)
{
  volatile uint32_t *ldo_ctrl =
      (volatile uint32_t *)PMU_EXT_LDO_P0_0P2A_CTRL;
  volatile uint32_t *ldo_ana =
      (volatile uint32_t *)PMU_EXT_LDO_P0_0P2A_ANA;
  uint32_t reg;
  volatile uint32_t count;
  uint32_t i;

  /* Set owner to software (force_tieh_sel = 1, tieh_sel = 0) */

  reg = *ldo_ctrl;
  reg |= EXT_LDO_FORCE_TIEH_SEL_BIT;
  reg &= ~EXT_LDO_TIEH_SEL_MASK;
  *ldo_ctrl = reg;

  /* Set voltage parameters (dref and mul) */

  reg = *ldo_ana;
  reg &= ~EXT_LDO_ANA_DREF_MASK;
  reg |= ((uint32_t)MIPI_PHY_LDO_DREF << EXT_LDO_ANA_DREF_SHIFT);
  reg &= ~EXT_LDO_ANA_MUL_MASK;
  reg |= ((uint32_t)MIPI_PHY_LDO_MUL << EXT_LDO_ANA_MUL_SHIFT);
  *ldo_ana = reg;

  /* Set tieh = 0 (use Vref*Mul, not rail voltage) */

  reg = *ldo_ctrl;
  reg &= ~EXT_LDO_TIEH_BIT;
  *ldo_ctrl = reg;

  /* Enable the LDO (xpd = 1) */

  reg = *ldo_ctrl;
  reg |= EXT_LDO_XPD_BIT;
  *ldo_ctrl = reg;

  /* Wait 5ms for voltage to stabilize */

  for (i = 0; i < 5; i++)
    {
      for (count = 0; count < 40000; count++)
        {
          __asm__ volatile ("nop");
        }
    }

  syslog(LOG_INFO, "CSI: LDO channel %d enabled at %dmV "
         "(dref=%d, mul=%d)\n",
         ESP_CSI_PHY_LDO_CHAN, ESP_CSI_PHY_LDO_MV,
         MIPI_PHY_LDO_DREF, MIPI_PHY_LDO_MUL);
}

/****************************************************************************
 * Name: esp_csi_clock_enable
 *
 * Description:
 *   Enable clocks for CSI Host and Bridge, configure PHY clock source.
 *   Steps 2-5 from the initialization sequence.
 ****************************************************************************/

static void esp_csi_clock_enable(void)
{
  uint32_t reg;

  /* Step 2: Enable CSI Host bus clock */

  reg = REG_READ(HP_SYS_CLKRST_BASE + SOC_CLK_CTRL1_OFF);
  reg |= CSI_HOST_SYS_CLK_EN | CSI_BRG_SYS_CLK_EN;
  REG_WRITE(HP_SYS_CLKRST_BASE + SOC_CLK_CTRL1_OFF, reg);

  /* Step 3: Reset CSI Host */

  REG_SET_BIT(HP_SYS_CLKRST_BASE + HP_RST_EN0_OFF, RST_EN_CSI_HOST);
  REG_CLR_BIT(HP_SYS_CLKRST_BASE + HP_RST_EN0_OFF, RST_EN_CSI_HOST);

  /* Reset CSI Bridge */

  REG_SET_BIT(HP_SYS_CLKRST_BASE + HP_RST_EN0_OFF, RST_EN_CSI_BRG);
  REG_CLR_BIT(HP_SYS_CLKRST_BASE + HP_RST_EN0_OFF, RST_EN_CSI_BRG);

  /* Step 4: Set PHY clock source = PLL_F20M (value 0) */

  reg = REG_READ(HP_SYS_CLKRST_BASE + PERI_CLK_CTRL03_OFF);
  reg &= ~CSI_DPHY_CLK_SRC_MASK;  /* Clear src sel bits = PLL_F20M */
  REG_WRITE(HP_SYS_CLKRST_BASE + PERI_CLK_CTRL03_OFF, reg);

  /* Step 5: Enable PHY configuration clock */

  REG_SET_BIT(HP_SYS_CLKRST_BASE + PERI_CLK_CTRL03_OFF, CSI_DPHY_CFG_CLK_EN);

  syslog(LOG_INFO, "CSI: Clocks enabled\n");
}

/****************************************************************************
 * Name: esp_csi_phy_init
 *
 * Description:
 *   Initialize the CSI D-PHY (Steps 6-8).
 *   - Assert PHY reset
 *   - Configure PLL frequency via test interface
 *   - Release PHY reset
 ****************************************************************************/

static void esp_csi_phy_init(void)
{
  /* CSI Host PHY control registers (offsets from CSI Host base) */

  #define CSI_PHY_SHUTDOWNZ_OFF  0x0040
  #define CSI_DPHY_RSTZ_OFF      0x0044
  #define CSI_CSI2_RESETN_OFF    0x0048
  #define CSI_PHY_TEST_CTRL0_OFF 0x0050
  #define CSI_PHY_TEST_CTRL1_OFF 0x0054

  /* Step 6: Assert resets (active low signals - write 0 to assert) */

  REG_WRITE(MIPI_CSI_HOST_BASE + CSI_PHY_SHUTDOWNZ_OFF, 0);  /* phy_shutdownz = 0 */
  REG_WRITE(MIPI_CSI_HOST_BASE + CSI_DPHY_RSTZ_OFF, 0);      /* dphy_rstz = 0 */
  REG_WRITE(MIPI_CSI_HOST_BASE + CSI_CSI2_RESETN_OFF, 0);    /* csi2_resetn = 0 */

  /* Clear PHY test interface */

  REG_WRITE(MIPI_CSI_HOST_BASE + CSI_PHY_TEST_CTRL0_OFF, 0x01); /* clear */
  REG_WRITE(MIPI_CSI_HOST_BASE + CSI_PHY_TEST_CTRL0_OFF, 0x00);

  /* Step 7: Configure PHY PLL frequency range via test interface.
   * Write register 0x44 with hs_freq_sel << 1.
   * Sequence: set addr → clock↑ → clock↓ → set val → clock↑ → clock↓
   */

  /* Write address 0x44 */

  REG_WRITE(MIPI_CSI_HOST_BASE + CSI_PHY_TEST_CTRL1_OFF,
            (1 << 16) | 0x44);                           /* testen=1, addr=0x44 */
  REG_WRITE(MIPI_CSI_HOST_BASE + CSI_PHY_TEST_CTRL0_OFF, 0x02); /* clock=1 */
  REG_WRITE(MIPI_CSI_HOST_BASE + CSI_PHY_TEST_CTRL0_OFF, 0x00); /* clock=0 */

  /* Write value: hs_freq_sel << 1 for 200 Mbps */

  REG_WRITE(MIPI_CSI_HOST_BASE + CSI_PHY_TEST_CTRL1_OFF,
            (ESP_CSI_PHY_HS_FREQ_SEL << 1) & 0xFF);     /* testen=0, data */
  REG_WRITE(MIPI_CSI_HOST_BASE + CSI_PHY_TEST_CTRL0_OFF, 0x02); /* clock=1 */
  REG_WRITE(MIPI_CSI_HOST_BASE + CSI_PHY_TEST_CTRL0_OFF, 0x00); /* clock=0 */

  /* Step 8: Release resets */

  REG_WRITE(MIPI_CSI_HOST_BASE + CSI_PHY_SHUTDOWNZ_OFF, 1);  /* phy_shutdownz = 1 */
  REG_WRITE(MIPI_CSI_HOST_BASE + CSI_DPHY_RSTZ_OFF, 1);      /* dphy_rstz = 1 */
  REG_WRITE(MIPI_CSI_HOST_BASE + CSI_CSI2_RESETN_OFF, 1);    /* csi2_resetn = 1 */

  syslog(LOG_INFO, "CSI: D-PHY initialized (hs_freq_sel=0x%02x)\n",
         ESP_CSI_PHY_HS_FREQ_SEL);
}

/****************************************************************************
 * Name: esp_csi_host_init
 *
 * Description:
 *   Configure CSI Host Controller (Steps 9-12).
 *   - Set active lane count
 *   - Disable virtual channel extension
 *   - Disable scrambling
 ****************************************************************************/

static void esp_csi_host_init(void)
{
  /* Host register offsets */

  #define CSI_N_LANES_OFF       0x0004
  #define CSI_VC_EXTENSION_OFF  0x000c
  #define CSI_SCRAMBLING_OFF    0x0020

  /* Step 10: Set active lanes = 2 (register value = lanes - 1 = 1) */

  REG_WRITE(MIPI_CSI_HOST_BASE + CSI_N_LANES_OFF, ESP_CSI_LANE_NUM - 1);

  /* Step 11: Disable virtual channel extension (write 1 to disable) */

  REG_WRITE(MIPI_CSI_HOST_BASE + CSI_VC_EXTENSION_OFF, 1);

  /* Step 12: Disable scrambling */

  REG_WRITE(MIPI_CSI_HOST_BASE + CSI_SCRAMBLING_OFF, 0);

  syslog(LOG_INFO, "CSI: Host configured (%d lanes)\n", ESP_CSI_LANE_NUM);
}

/****************************************************************************
 * Name: esp_csi_bridge_init
 *
 * Description:
 *   Configure CSI Bridge (Steps 13-17).
 *   - Set frame size
 *   - Set FIFO thresholds
 *   - Set data type filter range
 *   - Set DMA burst length
 *   - Configure RAW8 → RGB565 color conversion
 ****************************************************************************/

static void esp_csi_bridge_init(void)
{
  /* Bridge register offsets */

  #define CSI_BRG_EN_OFF          0x0000
  #define CSI_BRG_HOST_CTRL_OFF   0x0004
  #define CSI_BRG_FRAME_CFG_OFF   0x0010
  #define CSI_BRG_DATA_TYPE_OFF   0x0014
  #define CSI_BRG_BUF_FLOW_OFF    0x0018
  #define CSI_BRG_DMA_REQ_OFF     0x001c
  #define CSI_BRG_ENDIAN_OFF      0x0024
  #define CSI_BRG_CM_CTRL_OFF     0x0080

  uint32_t reg;

  /* Step 13: Set frame size (h_pixel and v_row) */

  reg = (ESP_CSI_HRES & 0xFFF) | ((ESP_CSI_VRES & 0xFFF) << 12);
  REG_WRITE(MIPI_CSI_BRIDGE_BASE + CSI_BRG_FRAME_CFG_OFF, reg);

  /* Step 14: Set FIFO almost-full threshold */

  REG_WRITE(MIPI_CSI_BRIDGE_BASE + CSI_BRG_BUF_FLOW_OFF,
            ESP_CSI_BRG_AFULL_THRD);

  /* Step 15: Set data type filter range [0x12, 0x2F] */

  reg = (ESP_CSI_BRG_DT_MIN & 0x3F) | ((ESP_CSI_BRG_DT_MAX & 0x3F) << 8);
  REG_WRITE(MIPI_CSI_BRIDGE_BASE + CSI_BRG_DATA_TYPE_OFF, reg);

  /* Step 16: Set DMA burst length */

  REG_WRITE(MIPI_CSI_BRIDGE_BASE + CSI_BRG_DMA_REQ_OFF,
            ESP_CSI_BRG_BURST_LEN);

  /* Step 17: Configure color format conversion (RAW8 → RGB565)
   * Enable color conversion, set input=RAW8, output=RGB565, bypass=false.
   * CM_CTRL register layout (approximate):
   *   bit[0]    : cm_en (enable)
   *   bit[1]    : cm_bypass
   *   bit[3:2]  : cm_rx (input format: 0=RGB888, 1=RGB565, 2=YUV422, 3=YUV420)
   *   bit[5:4]  : cm_tx (output format: same encoding)
   *   For RAW8 input with color conversion, we rely on the Bridge's
   *   internal demosaic treating RAW8 as a special input mode.
   *
   * NOTE: The exact register layout depends on chip revision.
   * This is a simplified version that sets known-good values.
   */

  reg = 0;
  reg |= (1 << 0);  /* cm_en = 1 (enable conversion) */
  reg &= ~(1 << 1); /* cm_bypass = 0 (don't bypass) */
  /* Input: RAW8 maps to a special mode, output: RGB565 = 1 */
  reg |= (1 << 4);  /* cm_tx = 1 (RGB565 output) */
  REG_WRITE(MIPI_CSI_BRIDGE_BASE + CSI_BRG_CM_CTRL_OFF, reg);

  /* Enable Bridge clock */

  REG_SET_BIT(MIPI_CSI_BRIDGE_BASE + CSI_BRG_HOST_CTRL_OFF, (1 << 0));

  syslog(LOG_INFO, "CSI: Bridge configured (%dx%d, RAW8->RGB565)\n",
         ESP_CSI_HRES, ESP_CSI_VRES);
}

/****************************************************************************
 * Name: esp_csi_dma_init
 *
 * Description:
 *   Initialize DMA for CSI frame transfer (Steps 18-19).
 *   This is a simplified stub - full DMA setup with DW-GDMA is complex.
 *   For now, we allocate frame buffers and prepare for manual polling
 *   or future interrupt-driven DMA.
 ****************************************************************************/

static int esp_csi_dma_init(void)
{
  /* Allocate double frame buffers from heap (PSRAM-backed on ESP32-P4).
   * In a production driver, use heap_caps_aligned_alloc with SPIRAM cap.
   * For NuttX, we use kmm_memalign for cache-aligned allocation.
   */

  g_csi_dev.frame_buffer[0] = (uint8_t *)kmm_memalign(64,
                                                       ESP_CSI_FRAME_SIZE);
  if (g_csi_dev.frame_buffer[0] == NULL)
    {
      syslog(LOG_ERR, "CSI: Failed to allocate frame buffer 0\n");
      return -ENOMEM;
    }

  g_csi_dev.frame_buffer[1] = (uint8_t *)kmm_memalign(64,
                                                       ESP_CSI_FRAME_SIZE);
  if (g_csi_dev.frame_buffer[1] == NULL)
    {
      kmm_free(g_csi_dev.frame_buffer[0]);
      g_csi_dev.frame_buffer[0] = NULL;
      syslog(LOG_ERR, "CSI: Failed to allocate frame buffer 1\n");
      return -ENOMEM;
    }

  /* Clear buffers */

  memset(g_csi_dev.frame_buffer[0], 0, ESP_CSI_FRAME_SIZE);
  memset(g_csi_dev.frame_buffer[1], 0, ESP_CSI_FRAME_SIZE);

  g_csi_dev.active_buf = 0;

  /* TODO: Configure DW-GDMA channel with:
   *   Source: CSI Bridge FIFO (ESP_CSI_BRG_MEM_BASE), FIXED address
   *   Dest: frame_buffer[active_buf], INCREMENT address
   *   Transfer size: ESP_CSI_FRAME_SIZE / 8 (64-bit words)
   *   Flow controller: source (CSI Bridge)
   *   Callback: frame complete → switch buffer, post semaphore
   *
   * For initial bring-up, DMA is not fully configured.
   * The frame buffer will contain test pattern or zeros until DMA is done.
   */

  syslog(LOG_INFO, "CSI: DMA buffers allocated (2 x %d bytes)\n",
         ESP_CSI_FRAME_SIZE);
  syslog(LOG_INFO, "CSI: buf[0]=%p buf[1]=%p\n",
         g_csi_dev.frame_buffer[0], g_csi_dev.frame_buffer[1]);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_csi_init
 ****************************************************************************/

int esp_csi_init(void)
{
  int ret;

  if (g_csi_dev.initialized)
    {
      return OK;
    }

  syslog(LOG_INFO, "CSI: Initializing MIPI-CSI controller\n");

  memset(&g_csi_dev, 0, sizeof(g_csi_dev));

  /* Step 1: Enable MIPI PHY LDO */

  esp_csi_ldo_enable();

  /* Steps 2-5: Enable clocks */

  esp_csi_clock_enable();

  /* Steps 6-8: Initialize D-PHY */

  esp_csi_phy_init();

  /* Steps 9-12: Configure Host Controller */

  esp_csi_host_init();

  /* Steps 13-17: Configure Bridge */

  esp_csi_bridge_init();

  /* Steps 18-19: Initialize DMA */

  ret = esp_csi_dma_init();
  if (ret < 0)
    {
      return ret;
    }

  g_csi_dev.h_res = ESP_CSI_HRES;
  g_csi_dev.v_res = ESP_CSI_VRES;
  g_csi_dev.initialized = true;

  syslog(LOG_INFO, "CSI: Initialization complete\n");
  return OK;
}

/****************************************************************************
 * Name: esp_csi_start
 ****************************************************************************/

int esp_csi_start(void)
{
  if (!g_csi_dev.initialized)
    {
      return -EINVAL;
    }

  if (g_csi_dev.streaming)
    {
      return OK;
    }

  /* Enable CSI Bridge to start receiving data */

  REG_SET_BIT(MIPI_CSI_BRIDGE_BASE + 0x0000, (1 << 0));

  g_csi_dev.streaming = true;
  syslog(LOG_INFO, "CSI: Streaming started\n");
  return OK;
}

/****************************************************************************
 * Name: esp_csi_stop
 ****************************************************************************/

int esp_csi_stop(void)
{
  if (!g_csi_dev.initialized)
    {
      return -EINVAL;
    }

  if (!g_csi_dev.streaming)
    {
      return OK;
    }

  /* Disable CSI Bridge */

  REG_CLR_BIT(MIPI_CSI_BRIDGE_BASE + 0x0000, (1 << 0));

  g_csi_dev.streaming = false;
  syslog(LOG_INFO, "CSI: Streaming stopped\n");
  return OK;
}

/****************************************************************************
 * Name: esp_csi_get_frame_buffer
 ****************************************************************************/

uint8_t *esp_csi_get_frame_buffer(void)
{
  if (!g_csi_dev.initialized)
    {
      return NULL;
    }

  /* Return the buffer that is NOT currently being written to by DMA */

  return g_csi_dev.frame_buffer[g_csi_dev.active_buf ^ 1];
}
