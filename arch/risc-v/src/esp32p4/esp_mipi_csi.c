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
#include <sys/time.h>

#include <nuttx/kmalloc.h>
#include <nuttx/irq.h>
#include <nuttx/cache.h>

#include "esp_irq.h"
#include "riscv_internal.h"

#include "hal/dw_gdma_ll.h"
#include "hal/dw_gdma_types.h"
#include "hal/mipi_csi_ll.h"
#include "hal/mipi_csi_phy_ll.h"
#include "hal/mipi_csi_host_ll.h"
#include "hal/mipi_csi_brg_ll.h"
#include "soc/interrupts.h"

#include "esp_mipi_csi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register base addresses */

#define MIPI_CSI_HOST_BASE    0x500d0000
#define MIPI_CSI_BRIDGE_BASE  0x500d1000
#define HP_SYS_CLKRST_BASE   0x500e6000

/* ISP register base (DR_REG_HPPERIPH0_BASE + 0xA1000) */

#define ISP_BASE              0x500a1000
#define ISP_CLK_EN_REG        (ISP_BASE + 0x04)
#define ISP_CNTL_REG          (ISP_BASE + 0x08)
#define ISP_FRAME_CFG_REG     (ISP_BASE + 0x10)
#define ISP_INT_CLR_REG       (ISP_BASE + 0x70)

/* ISP_CNTL_REG bit definitions */

#define ISP_MIPI_DATA_EN      (1 << 0)   /* MIPI data input enable (gate) */
#define ISP_EN                (1 << 1)   /* ISP global enable */
#define ISP_DEMOSAIC_EN       (1 << 6)   /* Demosaic enable */
#define ISP_RGB2YUV_EN        (1 << 10)  /* RGB2YUV enable */
#define ISP_YUV2RGB_EN        (1 << 13)  /* YUV2RGB enable */

/* ISP_CNTL_REG field positions */

#define ISP_DATA_TYPE_SHIFT   25   /* 2 bits: 0=RAW8, 1=RAW10, 2=RAW12 */
#define ISP_IN_SRC_SHIFT      27   /* 2 bits: 0=CSI, 1=CAM/DVP, 2=DMA */
#define ISP_OUT_TYPE_SHIFT    29   /* 3 bits: 0=RAW8, 1=YUV422, 2=RGB888 */

/* ISP_FRAME_CFG_REG field positions */

#define ISP_VADR_NUM_SHIFT    0    /* 12 bits: vertical rows - 1 */
#define ISP_HADR_NUM_SHIFT    12   /* 12 bits: horizontal pixels - 1 */
#define ISP_HSYNC_START_BIT   (1 << 29)
#define ISP_HSYNC_END_BIT     (1 << 30)

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
  int __DECLARE_RCC_ATOMIC_ENV;
  (void)__DECLARE_RCC_ATOMIC_ENV;

  /* Enable CSI host bus clock and CSI bridge module clock, release resets */

  mipi_csi_ll_enable_host_bus_clock(0, true);
  mipi_csi_ll_reset_host_clock(0);
  mipi_csi_ll_enable_brg_module_clock(0, true);
  mipi_csi_ll_reset_brg_module_clock(0);

  /* PHY config clock source + enable (reset pulse) */

  mipi_csi_ll_set_phy_clock_source(0, MIPI_CSI_PHY_CLK_SRC_DEFAULT);
  mipi_csi_ll_enable_phy_config_clock(0, false);
  mipi_csi_ll_enable_phy_config_clock(0, true);

  syslog(LOG_INFO, "CSI: Clocks enabled (LL)\n");
}

/****************************************************************************
 * Name: esp_csi_isp_init
 *
 * Description:
 *   Initialize the ISP as a passthrough (RAW8 → RAW8) to open the MIPI
 *   data gate. On ESP32-P4 rev3, the CSI→bridge data path shares the ISP
 *   pipeline; the ISP's mipi_data_en bit is the master gate for MIPI data
 *   entering the CSI/ISP/bridge shared bus. Without enabling ISP and
 *   setting mipi_data_en=1, no pixel data reaches the bridge/DMA.
 *
 *   We configure ISP in bypass mode:
 *   - Input source = CSI (isp_in_src = 0)
 *   - Data type = RAW8
 *   - Output type = RAW8
 *   - All processing blocks disabled (demosaic, rgb2yuv, etc.)
 *   - mipi_data_en = 1, isp_en = 1
 *   - Frame dimensions match sensor output
 ****************************************************************************/

static void esp_csi_isp_init(void)
{
  uint32_t reg;

  /* Step 1: Enable ISP module clock via HP_SYS_CLKRST struct.
   * The struct is already accessible from the mipi_csi_ll includes.
   * peri_clk_ctrl25.reg_isp_clk_en = 1
   * peri_clk_ctrl25.reg_isp_clk_src_sel = 0 (XTAL 40MHz)
   */

  HP_SYS_CLKRST.peri_clk_ctrl25.reg_isp_clk_en = 1;
  HP_SYS_CLKRST.peri_clk_ctrl25.reg_isp_clk_src_sel = 0;

  /* Step 2: Reset ISP module via HP_SYS_CLKRST.hp_rst_en0.reg_rst_en_isp */

  HP_SYS_CLKRST.hp_rst_en0.reg_rst_en_isp = 1;

  /* Brief delay for reset */

  volatile uint32_t i;
  for (i = 0; i < 100; i++)
    {
      __asm__ volatile("nop");
    }

  HP_SYS_CLKRST.hp_rst_en0.reg_rst_en_isp = 0;

  /* Step 3: Configure ISP registers for bypass (RAW8 passthrough) */

  /* Clear ISP_CNTL: disable everything first */

  REG_WRITE(ISP_CNTL_REG, 0);

  /* Enable ISP register clock (ISP_CLK_EN bit0) */

  REG_WRITE(ISP_CLK_EN_REG, 1);

  /* Clear all pending interrupts */

  REG_WRITE(ISP_INT_CLR_REG, 0xFFFFFFFF);

  /* Configure ISP_CNTL:
   *   bit 0: mipi_data_en = 1 (open the MIPI data gate!)
   *   bit 1: isp_en = 1 (ISP global enable)
   *   bit 6: demosaic_en = 1 (RAW Bayer → RGB)
   *   bit 10: rgb2yuv_en = 1 (needed for output pipeline)
   *   bit 13: yuv2rgb_en = 1 (YUV → RGB output)
   *   bits [26:25]: isp_data_type = 0 (RAW8)
   *   bits [28:27]: isp_in_src = 0 (CSI host)
   *   bits [31:29]: isp_out_type = 4 (RGB565)
   */

  reg = ISP_MIPI_DATA_EN | ISP_EN |
        ISP_DEMOSAIC_EN | ISP_RGB2YUV_EN | ISP_YUV2RGB_EN |
        (4u << ISP_OUT_TYPE_SHIFT);  /* isp_out_type = 4 = RGB565 */
  REG_WRITE(ISP_CNTL_REG, reg);

  /* Step 4: Configure frame dimensions
   *
   * ISP_FRAME_CFG:
   *   vadr_num[11:0]  = VRES - 1 (vertical rows)
   *   hadr_num[23:12] = HRES - 1 (horizontal pixels)
   *   hsync_start_exist[29] = 1
   *   hsync_end_exist[30] = 1
   */

  reg = ((ESP_CSI_VRES - 1) << ISP_VADR_NUM_SHIFT) |
        ((ESP_CSI_HRES - 1) << ISP_HADR_NUM_SHIFT) |
        ISP_HSYNC_START_BIT |
        ISP_HSYNC_END_BIT;
  REG_WRITE(ISP_FRAME_CFG_REG, reg);

  syslog(LOG_INFO, "CSI: ISP initialized as RAW8→RGB565 "
         "(mipi_data_en=1, demosaic+rgb2yuv+yuv2rgb, %dx%d)\n",
         ESP_CSI_HRES, ESP_CSI_VRES);
  syslog(LOG_INFO, "CSI: ISP_CNTL=0x%08lx ISP_FRAME_CFG=0x%08lx\n",
         (unsigned long)REG_READ(ISP_CNTL_REG),
         (unsigned long)REG_READ(ISP_FRAME_CFG_REG));
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

static void esp_csi_phy_write_reg(csi_host_dev_t *host, uint8_t addr,
                                  uint8_t val)
{
  /* DW MIPI D-PHY test interface write sequence (matches HAL) */

  mipi_csi_phy_ll_write_clock(host, 0, false);
  mipi_csi_phy_ll_write_reg_addr(host, addr);
  mipi_csi_phy_ll_write_clock(host, 1, false);
  mipi_csi_phy_ll_write_clock(host, 0, false);
  mipi_csi_phy_ll_write_reg_val(host, val);
  mipi_csi_phy_ll_write_clock(host, 1, false);
  mipi_csi_phy_ll_write_clock(host, 0, false);
}

static void esp_csi_phy_init(void)
{
  csi_host_dev_t *host = MIPI_CSI_HOST_LL_GET_HW(0);

  /* Assert PHY shutdown + resets (matches mipi_csi_hal_init) */

  mipi_csi_phy_ll_enable_shutdown_input(host, true);
  mipi_csi_phy_ll_enable_reset_output(host, true);
  mipi_csi_host_ll_enable_reset_output(host, true);

  /* Reset PHY test reg addr/val to defaults */

  mipi_csi_phy_ll_write_reg_addr(host, 0x0);
  mipi_csi_phy_ll_write_clock(host, 0, true);
  mipi_csi_phy_ll_write_clock(host, 0, false);

  /* Program PHY PLL HS frequency range for the configured lane rate */

  esp_csi_phy_write_reg(host, 0x44, ESP_CSI_PHY_HS_FREQ_SEL << 1);

  /* Release PHY shutdown + resets */

  mipi_csi_phy_ll_enable_shutdown_input(host, false);
  mipi_csi_phy_ll_enable_reset_output(host, false);
  mipi_csi_host_ll_enable_reset_output(host, false);

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
  csi_host_dev_t *host = MIPI_CSI_HOST_LL_GET_HW(0);

  /* Active lanes, disable VC extension and scrambling (matches HAL) */

  mipi_csi_host_ll_set_active_lanes_num(host, ESP_CSI_LANE_NUM);
  mipi_csi_host_ll_enable_virtual_channel_extension(host, false);
  mipi_csi_host_ll_enable_scrambling(host, false);

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
  csi_brg_dev_t *brg = MIPI_CSI_BRG_LL_GET_HW(0);

  /* Enable the bridge PHY/host clock */

  mipi_csi_brg_ll_enable_clock(brg, true);

  /* Frame geometry: H pixels and V rows.
   * NOTE: mipi_csi_hal_init passes frame_height as the H pixel count and
   * frame_width as the V row count for this sensor orientation.
   */

  mipi_csi_brg_ll_set_intput_data_h_pixel_num(brg, ESP_CSI_HRES);
  mipi_csi_brg_ll_set_intput_data_v_row_num(brg, ESP_CSI_VRES);



  /* FIFO almost-full threshold and RAW8 data-type filter [0x12, 0x2f] */

  mipi_csi_brg_ll_set_flow_ctl_buf_afull_thrd(brg, ESP_CSI_BRG_AFULL_THRD);
  mipi_csi_brg_ll_set_data_type_min(brg, ESP_CSI_BRG_DT_MIN);
  mipi_csi_brg_ll_set_data_type_max(brg, ESP_CSI_BRG_DT_MAX);
  mipi_csi_brg_ll_set_output_byte_endian(brg, false);

  /* DMA burst length (64-bit words per burst), bridge is flow controller */

  mipi_csi_brg_ll_set_burst_len(brg, ESP_CSI_BRG_BURST_LEN);

  /* Color-mode conversion: RAW8 in == RAW8 out, so enable the color-mode
   * block in bypass (passthrough). This matches esp_cam_ctlr_csi's
   * s_csi_ctlr_format_conversion for the src==dst case and is required
   * for the bridge to forward pixel data on rev >= 3.0.
   */

  mipi_csi_brg_ll_enable_color_conversion(brg, true);
  mipi_csi_brg_ll_set_color_mode_bypass(brg, true);

  syslog(LOG_INFO, "CSI: Bridge configured (%dx%d RAW8, burst=%d)\n",
         ESP_CSI_HRES, ESP_CSI_VRES, ESP_CSI_BRG_BURST_LEN);
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

/****************************************************************************
 * Name: esp_csi_dma_arm
 *
 * Description:
 *   Program the DW-GDMA channel transfer (source = CSI bridge FIFO,
 *   destination = dst) and enable the channel. Must be called with the
 *   destination buffer already validated (non-NULL, large enough).
 ****************************************************************************/

static void esp_csi_dma_arm(FAR uint8_t *dst)
{
  dw_gdma_dev_t *dev = DW_GDMA_LL_GET_HW(0);
  const uint8_t ch = ESP_CSI_DMA_CHANNEL;

  /* Source: CSI bridge FIFO, fixed address, 64-bit, burst 512/16 */

  dw_gdma_ll_channel_set_src_addr(dev, ch, ESP_CSI_BRG_MEM_BASE);
  dw_gdma_ll_channel_set_src_burst_mode(dev, ch, DW_GDMA_BURST_MODE_FIXED);
  dw_gdma_ll_channel_set_src_trans_width(dev, ch, DW_GDMA_TRANS_WIDTH_64);
  dw_gdma_ll_channel_set_src_burst_items(dev, ch, DW_GDMA_BURST_ITEMS_512);
  dw_gdma_ll_channel_set_src_burst_len(dev, ch, 16);
  dw_gdma_ll_channel_set_src_master_port(dev, ch, ESP_CSI_BRG_MEM_BASE);

  /* Destination: frame buffer, incrementing address, 64-bit */

  dw_gdma_ll_channel_set_dst_addr(dev, ch, (uint32_t)(uintptr_t)dst);
  dw_gdma_ll_channel_set_dst_burst_mode(dev, ch, DW_GDMA_BURST_MODE_INCREMENT);
  dw_gdma_ll_channel_set_dst_trans_width(dev, ch, DW_GDMA_TRANS_WIDTH_64);
  dw_gdma_ll_channel_set_dst_burst_items(dev, ch, DW_GDMA_BURST_ITEMS_512);
  dw_gdma_ll_channel_set_dst_burst_len(dev, ch, 16);
  dw_gdma_ll_channel_set_dst_master_port(dev, ch, (uintptr_t)dst);

  /* One RAW8 frame worth of 64-bit items */

  dw_gdma_ll_channel_set_trans_block_size(dev, ch, ESP_CSI_DMA_XFER_ITEMS);

  /* Contiguous single-block transfer, source is flow controller (CSI) */

  dw_gdma_ll_channel_set_src_multi_block_type(dev, ch,
                                     DW_GDMA_BLOCK_TRANSFER_CONTIGUOUS);
  dw_gdma_ll_channel_set_dst_multi_block_type(dev, ch,
                                     DW_GDMA_BLOCK_TRANSFER_CONTIGUOUS);
  dw_gdma_ll_channel_set_trans_flow(dev, ch, DW_GDMA_ROLE_PERIPH_CSI,
                                    DW_GDMA_ROLE_MEM, DW_GDMA_FLOW_CTRL_SRC);

  /* Hardware handshake, CSI as source peripheral */

  dw_gdma_ll_channel_set_src_handshake_interface(dev, ch,
                                                 DW_GDMA_HANDSHAKE_HW);
  dw_gdma_ll_channel_set_dst_handshake_interface(dev, ch,
                                                 DW_GDMA_HANDSHAKE_HW);
  dw_gdma_ll_channel_set_src_handshake_periph(dev, ch,
                                              DW_GDMA_ROLE_PERIPH_CSI);
  dw_gdma_ll_channel_set_src_outstanding_limit(dev, ch, 5);
  dw_gdma_ll_channel_set_dst_outstanding_limit(dev, ch, 5);
  dw_gdma_ll_channel_set_priority(dev, ch, 1);

  /* Enable block-transfer-done interrupt for this single block */

  dw_gdma_ll_channel_set_block_markers(dev, ch, true, true, true);
  dw_gdma_ll_channel_clear_intr(dev, ch, UINT32_MAX);
  dw_gdma_ll_channel_enable_intr_generation(dev, ch,
                             DW_GDMA_LL_CHANNEL_EVENT_BLOCK_TFR_DONE, true);
  dw_gdma_ll_channel_enable_intr_propagation(dev, ch,
                             DW_GDMA_LL_CHANNEL_EVENT_BLOCK_TFR_DONE, true);

  /* Enable the channel */

  dw_gdma_ll_channel_enable(dev, ch, true);
}

/****************************************************************************
 * Name: esp_csi_dma_isr
 *
 * Description:
 *   DW-GDMA interrupt handler. On block-transfer-done for the CSI channel,
 *   invalidate the completed buffer's cache, notify the upper layer, and
 *   re-arm the DMA with the next buffer.
 ****************************************************************************/

static int esp_csi_dma_isr(int irq, void *context, void *arg)
{
  dw_gdma_dev_t *dev = DW_GDMA_LL_GET_HW(0);
  const uint8_t ch = ESP_CSI_DMA_CHANNEL;
  uint32_t status;

  /* --- Handle DSI channel 1 first (if it fired) --- */

  {
    uint32_t dsi_st = dev->ch[1].int_st0.val;
    if (dsi_st)
      {
        extern void esp_dsi_dma_isr_handler(void);
        esp_dsi_dma_isr_handler();
      }
  }

  /* --- Handle CSI channel 0 --- */

  status = dev->ch[ch].int_st0.val;

  if ((status & DW_GDMA_LL_CHANNEL_EVENT_BLOCK_TFR_DONE) == 0)
    {
      /* Not our event - clear whatever fired and return */

      dw_gdma_ll_channel_clear_intr(dev, ch, status);
      return OK;
    }

  dw_gdma_ll_channel_clear_intr(dev, ch, status);

  g_csi_dev.frame_count++;

  /* Invalidate cache for the just-filled buffer so the CPU sees DMA data */

  if (g_csi_dev.dma_dst != NULL)
    {
      up_invalidate_dcache((uintptr_t)g_csi_dev.dma_dst,
                           (uintptr_t)g_csi_dev.dma_dst +
                           g_csi_dev.dma_dst_size);
    }

  /* Notify upper layer and obtain the next destination buffer */

  if (g_csi_dev.frame_cb != NULL)
    {
      FAR uint8_t *next = NULL;
      uint32_t next_size = 0;

      g_csi_dev.frame_cb(g_csi_dev.frame_cb_arg, &next, &next_size);

      if (next != NULL && next_size >= ESP_CSI_FRAME_SIZE)
        {
          g_csi_dev.dma_dst = next;
          g_csi_dev.dma_dst_size = next_size;
          esp_csi_dma_arm(next);
        }
      else
        {
          /* No buffer available: leave DMA disarmed until next start */

          g_csi_dev.dma_dst = NULL;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: esp_csi_dma_init
 *
 * Description:
 *   Initialize the DW-GDMA controller and allocate the CSI DMA channel
 *   interrupt. Also allocates internal backup frame buffers.
 ****************************************************************************/

static int esp_csi_dma_init(void)
{
  dw_gdma_dev_t *dev = DW_GDMA_LL_GET_HW(0);
  int __DECLARE_RCC_ATOMIC_ENV;
  (void)__DECLARE_RCC_ATOMIC_ENV;

  /* Allocate internal backup frame buffers (used when no v4l2 buffer is
   * queued). RAW8 sized, 64-byte (cache line) aligned.
   */

  g_csi_dev.frame_buffer[0] = (uint8_t *)kmm_memalign(64,
                                                      ESP_CSI_FRAME_SIZE);
  g_csi_dev.frame_buffer[1] = (uint8_t *)kmm_memalign(64,
                                                      ESP_CSI_FRAME_SIZE);
  if (g_csi_dev.frame_buffer[0] == NULL || g_csi_dev.frame_buffer[1] == NULL)
    {
      syslog(LOG_ERR, "CSI: Failed to allocate frame buffers\n");
      return -ENOMEM;
    }

  memset(g_csi_dev.frame_buffer[0], 0, ESP_CSI_FRAME_SIZE);
  memset(g_csi_dev.frame_buffer[1], 0, ESP_CSI_FRAME_SIZE);
  g_csi_dev.active_buf = 0;

  /* Enable DW-GDMA bus clock and release reset */

  dw_gdma_ll_enable_bus_clock(0, true);
  dw_gdma_ll_reset_register(0);

  /* Reset and enable the controller, enable global interrupt */

  dw_gdma_ll_reset(dev);
  dw_gdma_ll_enable_controller(dev, true);
  dw_gdma_ll_enable_intr_global(dev, true);

  /* Hook the DW-GDMA interrupt */

  g_csi_dev.cpuint = esp_setup_irq(ETS_DW_GDMA_INTR_SOURCE,
                                   ESP_IRQ_PRIORITY_DEFAULT,
                                   ESP_IRQ_TRIGGER_LEVEL,
                                   esp_csi_dma_isr, NULL);
  if (g_csi_dev.cpuint < 0)
    {
      syslog(LOG_ERR, "CSI: Failed to setup DW-GDMA IRQ: %d\n",
             g_csi_dev.cpuint);
      return g_csi_dev.cpuint;
    }

  up_enable_irq(ESP_SOURCE2IRQ(ETS_DW_GDMA_INTR_SOURCE));

  syslog(LOG_INFO, "CSI: DW-GDMA initialized (xfer_items=%d, frame=%d bytes)\n",
         ESP_CSI_DMA_XFER_ITEMS, ESP_CSI_FRAME_SIZE);
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

  /* ISP initialization: open the MIPI data gate (must be before bridge) */

  esp_csi_isp_init();

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
  FAR uint8_t *dst;
  uint32_t size;

  if (!g_csi_dev.initialized)
    {
      return -EINVAL;
    }

  if (g_csi_dev.streaming)
    {
      return OK;
    }

  /* Choose the initial DMA destination: a buffer set via esp_csi_set_buffer
   * if available, otherwise the internal backup buffer.
   */

  if (g_csi_dev.dma_dst != NULL &&
      g_csi_dev.dma_dst_size >= ESP_CSI_FRAME_SIZE)
    {
      dst  = g_csi_dev.dma_dst;
      size = g_csi_dev.dma_dst_size;
    }
  else
    {
      dst  = g_csi_dev.frame_buffer[0];
      size = ESP_CSI_FRAME_SIZE;
    }

  g_csi_dev.dma_dst = dst;
  g_csi_dev.dma_dst_size = size;

  /* Arm the DMA before enabling the bridge so the first frame is captured */

  esp_csi_dma_arm(dst);

  /* Enable CSI Bridge (csi_en register) to start receiving data */

  mipi_csi_brg_ll_enable(MIPI_CSI_BRG_LL_GET_HW(0), true);

  g_csi_dev.streaming = true;
  syslog(LOG_INFO, "CSI: Streaming started (dst=%p)\n", dst);
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

  /* Disable CSI Bridge (csi_en register) */

  mipi_csi_brg_ll_enable(MIPI_CSI_BRG_LL_GET_HW(0), false);

  /* Disable the DMA channel */

  dw_gdma_ll_channel_enable(DW_GDMA_LL_GET_HW(0), ESP_CSI_DMA_CHANNEL, false);

  g_csi_dev.streaming = false;
  syslog(LOG_INFO, "CSI: Streaming stopped (%lu frames)\n",
         (unsigned long)g_csi_dev.frame_count);
  return OK;
}

/****************************************************************************
 * Name: esp_csi_register_frame_cb
 ****************************************************************************/

int esp_csi_register_frame_cb(esp_csi_frame_cb_t cb, FAR void *arg)
{
  g_csi_dev.frame_cb = cb;
  g_csi_dev.frame_cb_arg = arg;
  return OK;
}

/****************************************************************************
 * Name: esp_csi_set_buffer
 ****************************************************************************/

int esp_csi_set_buffer(FAR uint8_t *buf, uint32_t size)
{
  if (buf == NULL || size < ESP_CSI_FRAME_SIZE)
    {
      return -EINVAL;
    }

  g_csi_dev.dma_dst = buf;
  g_csi_dev.dma_dst_size = size;
  return OK;
}

/****************************************************************************
 * Name: esp_csi_dump_status
 *
 * Description:
 *   Diagnostic: dump DW-GDMA channel and CSI bridge state to isolate where
 *   the capture pipeline stalls. DAR advancing past the buffer start means
 *   the DMA moved data (bridge produced DMA requests).
 ****************************************************************************/

void esp_csi_dump_status(void)
{
  dw_gdma_dev_t *dev = DW_GDMA_LL_GET_HW(0);
  const uint8_t ch = ESP_CSI_DMA_CHANNEL;
  csi_brg_dev_t *brg = MIPI_CSI_BRG_LL_GET_HW(0);
  csi_host_dev_t *host = MIPI_CSI_HOST_LL_GET_HW(0);
  int i;

  /* Sample repeatedly: the D-PHY returns to stop-state between frames,
   * so a single sample is unreliable. Watch DAR advance and PHY activity
   * over several samples.
   */

  for (i = 0; i < 8; i++)
    {
      syslog(LOG_INFO, "CSI-DIAG[%d]: dar=0x%08lx chen=0x%lx dma_ist=0x%lx | "
             "brg_en=%lu brg_iraw=0x%lx | phy_ss=0x%lx phy_rx=0x%lx "
             "ist_main=0x%lx\n",
             i,
             (unsigned long)dev->ch[ch].dar0.val,
             (unsigned long)dev->chen0.val,
             (unsigned long)dev->ch[ch].int_st0.val,
             (unsigned long)brg->csi_en.val,
             (unsigned long)brg->int_raw.val,
             (unsigned long)host->phy_stopstate.val,
             (unsigned long)host->phy_rx.val,
             (unsigned long)host->int_st_main.val);
      up_udelay(2000);   /* 2ms between samples */
    }

  /* Decode which CSI-2 frame errors are latched */

  syslog(LOG_INFO, "CSI-DIAG: bndry_fatal=0x%lx seq_fatal=0x%lx "
         "phy_fatal=0x%lx pkt_fatal=0x%lx\n",
         (unsigned long)host->int_st_bndry_frame_fatal.val,
         (unsigned long)host->int_st_seq_frame_fatal.val,
         (unsigned long)host->int_st_phy_fatal.val,
         (unsigned long)host->int_st_pkt_fatal.val);

  /* ISP status: verify mipi_data_en is set */

  syslog(LOG_INFO, "CSI-DIAG: ISP_CNTL=0x%08lx ISP_FRAME_CFG=0x%08lx "
         "ISP_CLK_EN=0x%08lx\n",
         (unsigned long)REG_READ(ISP_CNTL_REG),
         (unsigned long)REG_READ(ISP_FRAME_CFG_REG),
         (unsigned long)REG_READ(ISP_CLK_EN_REG));
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
