/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_mipi_dsi.c
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
#include <math.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/cache.h>
#include <nuttx/kthread.h>
#include <nuttx/irq.h>
#include <string.h>
#include <unistd.h>

#include "esp_mipi_dsi.h"
#include "esp_dw_gdma_idf.h"
#include "esp_cache.h"
#include "soc/soc.h"       /* SOC_NON_CACHEABLE_OFFSET_SRAM */
#include "hal/cache_ll.h"  /* CACHE_LL_L2MEM_NON_CACHE_ADDR */

/* HAL includes */

#include "hal/mipi_dsi_hal.h"
#include "hal/mipi_dsi_ll.h"
#include "hal/mipi_dsi_host_ll.h"
#include "hal/mipi_dsi_phy_ll.h"
#include "hal/mipi_dsi_brg_ll.h"
#include "hal/dw_gdma_ll.h"
#include "hal/dw_gdma_types.h"

/****************************************************************************
 * MIPI PHY LDO Configuration (Direct Register Access)
 *
 * LDO channel 3 → unit = LDO_ID2UNIT(3) = 2
 * index_array[4] = {0, 3, 1, 4} → index_array[2] = 1
 * ext_ldo[1] corresponds to P0_0P2A registers:
 *   Control reg: PMU_BASE + 0x1c0
 *   ANA reg:     PMU_BASE + 0x1c4
 *
 * pmu_ext_ldo_reg_t bit fields:
 *   bit[7]    : force_tieh_sel (0=hw, 1=sw)
 *   bit[8]    : xpd (enable)
 *   bit[9:11] : tieh_sel
 *   bit[14]   : tieh (0=Vref*Mul, 1=3.3V)
 *
 * pmu_ext_ldo_ana_reg_t bit fields:
 *   bit[23:25]: mul (3 bits)
 *   bit[28:31]: dref (4 bits)
 *
 * For 2500mV without eFuse calibration: dref=13, mul=3
 ****************************************************************************/

#define PMU_BASE_ADDR                   0x50115000
#define PMU_EXT_LDO_P0_0P2A_REG_ADDR   (PMU_BASE_ADDR + 0x1c0)
#define PMU_EXT_LDO_P0_0P2A_ANA_ADDR   (PMU_BASE_ADDR + 0x1c4)

/* Bit positions in ext_ldo control register */

#define EXT_LDO_FORCE_TIEH_SEL_BIT      (1 << 7)
#define EXT_LDO_XPD_BIT                 (1 << 8)
#define EXT_LDO_TIEH_SEL_SHIFT          9
#define EXT_LDO_TIEH_SEL_MASK           (0x7 << EXT_LDO_TIEH_SEL_SHIFT)
#define EXT_LDO_TIEH_BIT                (1 << 14)

/* Bit positions in ext_ldo ANA register */

#define EXT_LDO_ANA_MUL_SHIFT           23
#define EXT_LDO_ANA_MUL_MASK            (0x7 << EXT_LDO_ANA_MUL_SHIFT)
#define EXT_LDO_ANA_DREF_SHIFT          28
#define EXT_LDO_ANA_DREF_MASK           (0xF << EXT_LDO_ANA_DREF_SHIFT)

/* Voltage parameters for 2500mV (computed from ldo_ll algorithm) */

#define MIPI_PHY_LDO_DREF               13
#define MIPI_PHY_LDO_MUL                3

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DSI_BUS_ID           ESP_DSI_BUS_ID
#define DSI_NUM_LANES        ESP_DSI_NUM_DATA_LANES
#define DSI_LANE_RATE_MBPS   ESP_DSI_LANE_BITRATE_MBPS
#define DSI_DPI_CLK_MHZ      ESP_DSI_DPI_CLK_MHZ
#define DSI_DPI_SRC_MHZ      ESP_DSI_DPI_CLK_SRC_MHZ
#define DSI_PHY_CLK_FREQ     ESP_DSI_PHY_CLK_SRC_FREQ

/* Panel timing */

#define PANEL_HRES           ESP_DSI_HRES
#define PANEL_VRES           ESP_DSI_VRES
#define PANEL_HSYNC          ESP_DSI_HSYNC
#define PANEL_HBP            ESP_DSI_HBP
#define PANEL_HFP            ESP_DSI_HFP
#define PANEL_VSYNC          ESP_DSI_VSYNC
#define PANEL_VBP            ESP_DSI_VBP
#define PANEL_VFP            ESP_DSI_VFP

/* Bridge FIFO memory base address for DMA writes.
 * MUST match HAL's MIPI_DSI_BRG_MEM_BASE (0x50105000) - the DMA master
 * port selection logic compares against this exact value to route the
 * transfer to the DSI bridge instead of generic memory.
 */

#define MIPI_DSI_BRG_MEM_BASE  0x50105000

/****************************************************************************
 * GPIO Configuration for Panel Reset and Backlight
 *
 * ESP32-P4 HP GPIO:
 *   DR_REG_GPIO_BASE  = 0x500E0000
 *   DR_REG_IO_MUX_BASE = 0x500E1000
 *
 * GPIO 0-31 use low registers, GPIO 32-55 use high registers.
 * Panel Reset = GPIO 27, Backlight = GPIO 26 (both in low range).
 *
 * IO MUX register layout (per pin):
 *   Bits [14:12] = MCU_SEL (function select, 1 = GPIO function)
 *
 * GPIO output registers (offsets from GPIO_BASE):
 *   OUT_W1TS = +0x08 (set output high)
 *   OUT_W1TC = +0x0C (set output low)
 *   ENABLE_W1TS = +0x24 (enable output)
 ****************************************************************************/

#define DSI_GPIO_BASE           0x500E0000
#define DSI_IO_MUX_BASE         0x500E1000

/* IO MUX register addresses for GPIO 26 and 27 */

#define DSI_IO_MUX_GPIO26_ADDR  (DSI_IO_MUX_BASE + 0x6C)
#define DSI_IO_MUX_GPIO27_ADDR  (DSI_IO_MUX_BASE + 0x70)

/* GPIO register offsets */

#define DSI_GPIO_OUT_W1TS_OFF   0x08
#define DSI_GPIO_OUT_W1TC_OFF   0x0C
#define DSI_GPIO_ENABLE_W1TS_OFF 0x24

/* MCU_SEL field in IO MUX: bits [14:12], GPIO function = 1 */

#define DSI_MCU_SEL_MASK        (0x7 << 12)
#define DSI_MCU_SEL_GPIO        (0x1 << 12)

/* Panel reset and backlight GPIO numbers */

#define DSI_PANEL_RESET_GPIO    27
#define DSI_BACKLIGHT_GPIO      26

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* EK79007 panel init command entry */

struct panel_cmd_s
{
  uint8_t cmd;
  uint8_t data;
  uint8_t data_len;  /* 0 = no data byte, 1 = has one data byte */
  uint16_t delay_ms; /* delay after command in ms */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* HAL context */

static mipi_dsi_hal_context_t g_dsi_hal;

/* Display buffers (allocated from PSRAM).
 *
 * g_fb_count is the number of buffers actually available: 2 when the
 * second allocation succeeded, 1 otherwise. g_fb_front is the index of
 * the buffer the display DMA is currently reading, and therefore also
 * the buffer the single-buffer users (fb0, demo thread) draw into.
 *
 * When g_fb_contiguous is true both buffers come from one single
 * allocation, g_framebuffer[1] == g_framebuffer[0] + ESP_DSI_FB_SIZE, and
 * userspace can therefore cover both with a single mmap of 2 *
 * ESP_DSI_FB_SIZE. g_fb_base keeps the allocation base so the region can
 * be released as one block later on.
 */

static uint8_t *g_framebuffer[2];
static uint8_t *g_fb_base;
static int g_fb_count;
static int g_fb_front;
static bool g_fb_contiguous;

/* DW-GDMA handles for DSI DMA refresh (high-level driver) */

static dw_gdma_channel_handle_t g_dsi_dma_chan;
static dw_gdma_link_list_handle_t g_dsi_link_list;

/* Transfer configuration for DMA re-arm in ISR callback */

static dw_gdma_block_transfer_config_t g_dsi_xfer_config;

/* EK79007 panel initialization sequence */

static const struct panel_cmd_s g_ek79007_init[] =
{
  { 0xb2, 0x10, 1, 0   },  /* PAD_CONTROL: 2-lane mode */
  { 0x80, 0x8b, 1, 0   },  /* Vendor specific */
  { 0x81, 0x78, 1, 0   },
  { 0x82, 0x84, 1, 0   },
  { 0x83, 0x88, 1, 0   },
  { 0x84, 0xa8, 1, 0   },
  { 0x85, 0xe3, 1, 0   },
  { 0x86, 0x88, 1, 0   },
  { 0x11, 0x00, 0, 120 },  /* Sleep Out, wait 120ms */
  { 0x29, 0x00, 0, 50  },  /* Display On, wait 50ms */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dsi_delay_ms
 *
 * Description:
 *   Simple busy-wait delay in milliseconds.
 *
 ****************************************************************************/

static void dsi_delay_ms(uint32_t ms)
{
  volatile uint32_t count;
  /* Approximate delay - ~400MHz CPU, ~10 cycles per iteration */

  for (uint32_t i = 0; i < ms; i++)
    {
      for (count = 0; count < 40000; count++)
        {
          __asm__ volatile ("nop");
        }
    }
}

/****************************************************************************
 * Name: dsi_configure_gpio_output
 *
 * Description:
 *   Configure a GPIO pin as output (set IO MUX to GPIO function, enable
 *   output driver).
 *
 ****************************************************************************/

static void dsi_configure_gpio_output(uint32_t gpio_num)
{
  volatile uint32_t *iomux = (volatile uint32_t *)(uintptr_t)
      (DSI_IO_MUX_BASE + 0x04 + gpio_num * 4);
  volatile uint32_t *enable_w1ts =
      (volatile uint32_t *)(DSI_GPIO_BASE + DSI_GPIO_ENABLE_W1TS_OFF);
  uint32_t reg;

  /* Set IO MUX function to GPIO (MCU_SEL = 1) */

  reg = *iomux;
  reg &= ~DSI_MCU_SEL_MASK;
  reg |= DSI_MCU_SEL_GPIO;
  *iomux = reg;

  /* Enable output */

  *enable_w1ts = (1 << gpio_num);
}

/****************************************************************************
 * Name: dsi_gpio_set_level
 *
 * Description:
 *   Set a GPIO output level (high or low).
 *
 ****************************************************************************/

static void dsi_gpio_set_level(uint32_t gpio_num, bool level)
{
  if (level)
    {
      volatile uint32_t *out_w1ts =
          (volatile uint32_t *)(DSI_GPIO_BASE + DSI_GPIO_OUT_W1TS_OFF);
      *out_w1ts = (1 << gpio_num);
    }
  else
    {
      volatile uint32_t *out_w1tc =
          (volatile uint32_t *)(DSI_GPIO_BASE + DSI_GPIO_OUT_W1TC_OFF);
      *out_w1tc = (1 << gpio_num);
    }
}

/****************************************************************************
 * Name: dsi_reset_panel
 *
 * Description:
 *   Reset the LCD panel via GPIO 27 (active low reset).
 *   Sequence: configure as output → pull low → wait 10ms → pull high →
 *   wait 10ms.
 *
 ****************************************************************************/

static void dsi_reset_panel(void)
{
  /* Configure GPIO 27 as output */

  dsi_configure_gpio_output(DSI_PANEL_RESET_GPIO);

  /* Pull low (assert reset, active low) - hold 20ms */

  dsi_gpio_set_level(DSI_PANEL_RESET_GPIO, false);
  dsi_delay_ms(20);

  /* Pull high (release reset) */

  dsi_gpio_set_level(DSI_PANEL_RESET_GPIO, true);

  /* EK79007AD datasheet Power On Sequence:
   * After GRB release: wait 30ms
   * Then LP11 must be maintained for 55ms minimum
   * Total wait: 100ms before sending any DCS command
   */

  dsi_delay_ms(100);
}

/****************************************************************************
 * Name: dsi_enable_backlight
 *
 * Description:
 *   Enable LCD backlight by pulling GPIO 26 high.
 *   Simplified implementation (full brightness, no PWM).
 *
 ****************************************************************************/

static void dsi_enable_backlight(void)
{
  /* Configure GPIO 26 as output */

  dsi_configure_gpio_output(DSI_BACKLIGHT_GPIO);

  /* Pull high (turn on backlight) */

  dsi_gpio_set_level(DSI_BACKLIGHT_GPIO, true);

  lcdinfo("Backlight enabled (GPIO %d)\n", DSI_BACKLIGHT_GPIO);
}

/****************************************************************************
 * Name: dsi_enable_phy_ldo
 *
 * Description:
 *   Enable LDO channel 3 at 2.5V for MIPI PHY power supply.
 *   Uses direct register access to PMU ext_ldo registers.
 *   LDO channel 3 = unit 2, index_array[2] = 1 → ext_ldo[1] (P0_0P2A).
 *
 ****************************************************************************/

static void dsi_enable_phy_ldo(void)
{
  volatile uint32_t *ldo_ctrl =
      (volatile uint32_t *)PMU_EXT_LDO_P0_0P2A_REG_ADDR;
  volatile uint32_t *ldo_ana =
      (volatile uint32_t *)PMU_EXT_LDO_P0_0P2A_ANA_ADDR;
  uint32_t reg;

  /* Step 1: Set owner to software (force_tieh_sel = 1, tieh_sel = 0) */

  reg = *ldo_ctrl;
  reg |= EXT_LDO_FORCE_TIEH_SEL_BIT;          /* force_tieh_sel = 1 (SW) */
  reg &= ~EXT_LDO_TIEH_SEL_MASK;              /* tieh_sel = 0 */
  *ldo_ctrl = reg;

  /* Step 2: Set voltage parameters (dref and mul) in ANA register */

  reg = *ldo_ana;
  reg &= ~EXT_LDO_ANA_DREF_MASK;
  reg |= ((uint32_t)MIPI_PHY_LDO_DREF << EXT_LDO_ANA_DREF_SHIFT);
  reg &= ~EXT_LDO_ANA_MUL_MASK;
  reg |= ((uint32_t)MIPI_PHY_LDO_MUL << EXT_LDO_ANA_MUL_SHIFT);
  *ldo_ana = reg;

  /* Step 3: Set tieh = 0 (use Vref*Mul, not rail voltage) */

  reg = *ldo_ctrl;
  reg &= ~EXT_LDO_TIEH_BIT;                   /* tieh = 0 */
  *ldo_ctrl = reg;

  /* Step 4: Enable the LDO (xpd = 1) */

  reg = *ldo_ctrl;
  reg |= EXT_LDO_XPD_BIT;                     /* xpd = 1 */
  *ldo_ctrl = reg;

  /* Step 5: Wait for voltage to stabilize */

  dsi_delay_ms(5);

  lcdinfo("DSI PHY LDO enabled: chan=%d, voltage=%dmV (dref=%d, mul=%d)\n",
          ESP_DSI_PHY_LDO_CHAN, ESP_DSI_PHY_LDO_MV,
          MIPI_PHY_LDO_DREF, MIPI_PHY_LDO_MUL);
}

/****************************************************************************
 * Name: dsi_enable_clocks
 *
 * Description:
 *   Enable DSI bus clocks, PHY config clock, and PHY PLL reference clock.
 *   Corresponds to Steps 1-3 of the initialization sequence.
 *
 ****************************************************************************/

static void dsi_enable_clocks(void)
{
  /* Declare RCC atomic env variable required by LL macros */

  int __DECLARE_RCC_ATOMIC_ENV;
  (void)__DECLARE_RCC_ATOMIC_ENV;

  /* Step 1: Enable DSI APB bus clock and reset Bridge */

  mipi_dsi_ll_enable_bus_clock(DSI_BUS_ID, true);
  mipi_dsi_ll_reset_register(DSI_BUS_ID);

  /* Step 2: Enable PHY configuration clock */

  mipi_dsi_ll_set_phy_config_clock_source(DSI_BUS_ID,
      MIPI_DSI_PHY_CFG_CLK_SRC_PLL_F20M);
  mipi_dsi_ll_enable_phy_config_clock(DSI_BUS_ID, true);

  /* Step 3: Enable PHY PLL reference clock (XTAL 40MHz) */

  mipi_dsi_ll_set_phy_pllref_clock_source(DSI_BUS_ID,
      MIPI_DSI_PHY_PLLREF_CLK_SRC_XTAL);
  mipi_dsi_ll_set_phy_pll_ref_clock_div(DSI_BUS_ID, 1);
  mipi_dsi_ll_enable_phy_pllref_clock(DSI_BUS_ID, true);
}

/****************************************************************************
 * Name: dsi_init_phy
 *
 * Description:
 *   Initialize PHY: set lanes, power on, configure PLL, wait for lock.
 *   Corresponds to Steps 4-6.
 *
 ****************************************************************************/

static int dsi_init_phy(void)
{
  mipi_dsi_hal_config_t hal_cfg;
  int timeout;

  /* Step 4: HAL init - sets lane number, powers on Host+PHY, resets PHY,
   * enables clock lane, forces PLL, resets bridge.
   */

  hal_cfg.bus_id = DSI_BUS_ID;
  hal_cfg.lane_bit_rate_mbps = (float)DSI_LANE_RATE_MBPS;
  hal_cfg.num_data_lanes = DSI_NUM_LANES;

  mipi_dsi_hal_init(&g_dsi_hal, &hal_cfg);

  /* Step 5: Configure PHY PLL.
   *
   * ESP-IDF does NOT re-assert PHY shutdown/reset before PLL config.
   * It relies on mipi_dsi_hal_init having already done the correct
   * power-up sequence (shutdownz→rstz→enableclk→forcepll).
   * Previous NuttX workaround re-asserted reset here, which may leave
   * PHY in an incorrect state (PHY_STATUS bits 2,4,7 extra vs ESP-IDF).
   */

  /* Write PLL M/N/HS-freq-range via test interface */

  mipi_dsi_hal_configure_phy_pll(&g_dsi_hal, DSI_PHY_CLK_FREQ,
                                 (float)DSI_LANE_RATE_MBPS);

  /* Step 6: Wait for PLL lock */

  timeout = 100000;
  while (!mipi_dsi_phy_ll_is_pll_locked(g_dsi_hal.host) && timeout > 0)
    {
      timeout--;
    }

  if (timeout <= 0)
    {
      syslog(LOG_ERR, "[DSI] ERROR: PHY PLL failed to lock!\n");
    }
  else
    {
      syslog(LOG_INFO, "[DSI] PHY PLL locked (timeout remaining=%d)\n",
             timeout);
    }

  /* Wait for lanes to reach stop state */

  timeout = 100000;
  while (!mipi_dsi_phy_ll_are_lanes_stopped(g_dsi_hal.host,
         DSI_NUM_LANES) && timeout > 0)
    {
      timeout--;
    }

  syslog(LOG_INFO, "[DSI] PHY init done, PLL %s, lanes %s\n",
         mipi_dsi_phy_ll_is_pll_locked(g_dsi_hal.host) ? "LOCKED" : "UNLOCKED",
         (timeout > 0) ? "stopped" : "NOT stopped");
  return OK;
}

/****************************************************************************
 * Name: dsi_configure_host
 *
 * Description:
 *   Configure Host controller for command mode: clock lane state,
 *   HS/LP switch time, CRC/ECC, timeout and escape clock.
 *   Corresponds to Steps 7-12.
 *
 ****************************************************************************/

static void dsi_configure_host(void)
{
  mipi_dsi_host_soc_handle_t host = g_dsi_hal.host;
  uint32_t lane_byte_clk_div;

  /* Step 7: Enter command mode (for panel init commands) */

  mipi_dsi_host_ll_enable_video_mode(host, false);

  /* Step 8: Clock lane auto mode */

  mipi_dsi_host_ll_set_clock_lane_state(host,
      MIPI_DSI_LL_CLOCK_LANE_STATE_AUTO);

  /* Step 9: HS/LP switch timing */

  mipi_dsi_phy_ll_set_switch_time(host, 50, 104, 46, 128);

  /* Step 10: Enable CRC, ECC, EoTp */

  mipi_dsi_host_ll_enable_rx_crc(host, true);
  mipi_dsi_host_ll_enable_rx_ecc(host, true);
  mipi_dsi_host_ll_enable_tx_eotp(host, true, false);

  /* Step 11: Timeout clock division and Escape clock division
   * TO clock = lane_byte_clk / div
   * lane_byte_clk = lane_bit_rate / 8 = 1000/8 = 125 MHz
   * TO div = 125/10 = 12 (targeting ~10 MHz)
   * ESC div = 125/18 ≈ 7 (targeting ~7 MHz, must be 2-20 MHz)
   */

  lane_byte_clk_div = (uint32_t)roundf(
      (float)DSI_LANE_RATE_MBPS / 8.0f / 10.0f);
  mipi_dsi_host_ll_set_timeout_clock_division(host, lane_byte_clk_div);

  lane_byte_clk_div = (uint32_t)roundf(
      (float)DSI_LANE_RATE_MBPS / 8.0f / 18.0f);
  mipi_dsi_host_ll_set_escape_clock_division(host, lane_byte_clk_div);

  /* Step 12: Disable all timeout counters (set to 0) */

  mipi_dsi_host_ll_set_timeout_count(host, 0, 0, 0, 0, 0, 0, 0);
  mipi_dsi_phy_ll_set_max_read_time(host, 6000);
  mipi_dsi_phy_ll_set_stop_wait_time(host, 0x3f);

  /* Step 13: Command-mode (DBI) configuration — CMD_MODE_CFG register.
   *
   * Ported from ESP-IDF esp_lcd_new_panel_io_dbi(). All DCS/generic
   * command transmissions must go out in LOW POWER (escape) mode; the
   * EK79007 panel does not accept init commands sent in high-speed mode.
   * Without this the panel never executes Sleep Out and stays blank.
   *
   * Produces CMD_MODE_CFG (host offset 0x68) == 0x010f7f02, matching
   * the ESP-IDF reference implementation.
   */

  mipi_dsi_host_ll_enable_te_ack(host, false);
  mipi_dsi_host_ll_enable_cmd_ack(host, true);

  mipi_dsi_host_ll_set_gen_short_wr_speed_mode(host, 0,
      MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_gen_short_wr_speed_mode(host, 1,
      MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_gen_short_wr_speed_mode(host, 2,
      MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_gen_long_wr_speed_mode(host,
      MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_gen_short_rd_speed_mode(host, 0,
      MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_gen_short_rd_speed_mode(host, 1,
      MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_gen_short_rd_speed_mode(host, 2,
      MIPI_DSI_LL_TRANS_SPEED_LP);

  mipi_dsi_host_ll_set_dcs_short_wr_speed_mode(host, 0,
      MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_dcs_short_wr_speed_mode(host, 1,
      MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_dcs_long_wr_speed_mode(host,
      MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_dcs_short_rd_speed_mode(host, 0,
      MIPI_DSI_LL_TRANS_SPEED_LP);

  mipi_dsi_host_ll_set_mrps_speed_mode(host,
      MIPI_DSI_LL_TRANS_SPEED_LP);
}

/****************************************************************************
 * Name: dsi_send_panel_commands
 *
 * Description:
 *   Send EK79007 panel initialization commands via DCS.
 *   Corresponds to Step 13.
 *
 ****************************************************************************/

static void dsi_send_panel_commands(void)
{
  int i;
  int num_cmds = sizeof(g_ek79007_init) / sizeof(g_ek79007_init[0]);

  for (i = 0; i < num_cmds; i++)
    {
      const struct panel_cmd_s *cmd = &g_ek79007_init[i];

      if (cmd->data_len > 0)
        {
          /* DCS command with 1 byte parameter */

          mipi_dsi_hal_host_gen_write_dcs_command(&g_dsi_hal, 0,
              cmd->cmd, 1, &cmd->data, cmd->data_len);
        }
      else
        {
          /* DCS command with no parameter (e.g., Sleep Out) */

          mipi_dsi_hal_host_gen_write_dcs_command(&g_dsi_hal, 0,
              cmd->cmd, 1, NULL, 0);
        }

      if (cmd->delay_ms > 0)
        {
          dsi_delay_ms(cmd->delay_ms);
        }
    }

  lcdinfo("Panel init commands sent (%d commands)\n", num_cmds);
}

/****************************************************************************
 * Name: dsi_configure_dpi
 *
 * Description:
 *   Configure DPI clock, Host DPI interface (video mode parameters),
 *   and Bridge.
 *   Corresponds to Steps 14-17.
 *
 ****************************************************************************/

static void dsi_configure_dpi(void)
{
  mipi_dsi_host_soc_handle_t host = g_dsi_hal.host;
  mipi_dsi_bridge_soc_handle_t bridge = g_dsi_hal.bridge;
  uint32_t dpi_div;

  /* Declare RCC atomic env variable required by LL macros */

  int __DECLARE_RCC_ATOMIC_ENV;
  (void)__DECLARE_RCC_ATOMIC_ENV;

  /* Step 14: Configure DPI clock
   * Source: PLL_F240M (240 MHz), divide by 5 → 48 MHz
   */

  dpi_div = mipi_dsi_hal_host_dpi_calculate_divider(&g_dsi_hal,
      (float)DSI_DPI_SRC_MHZ, (float)DSI_DPI_CLK_MHZ);

  mipi_dsi_ll_set_dpi_clock_source(DSI_BUS_ID,
      MIPI_DSI_DPI_CLK_SRC_PLL_F240M);
  mipi_dsi_ll_set_dpi_clock_div(DSI_BUS_ID, dpi_div);
  mipi_dsi_ll_enable_dpi_clock(DSI_BUS_ID, true);

  /* Step 15: Configure Host DPI interface */

  mipi_dsi_host_ll_dpi_set_vcid(host, 0);
  /* RGB565 with sub_config 0 selects 16-bit configuration 1, which is
   * what ESP-IDF's esp_lcd_new_panel_dpi() passes. This programs
   * DPI_COLOR_CODING (host offset 0x10) to 0.
   */

  mipi_dsi_host_ll_dpi_set_color_coding(host,
      LCD_COLOR_FMT_RGB565, 0);

  /* All signals active high */

  mipi_dsi_host_ll_dpi_set_timing_polarity(host,
      false, false, false, false, false);

  /* Allow LP during blanking periods */

  mipi_dsi_host_ll_dpi_enable_lp_horizontal_timing(host, true, true);
  mipi_dsi_host_ll_dpi_enable_lp_vertical_timing(host,
      true, true, true, true);
  mipi_dsi_host_ll_dpi_enable_lp_command(host, true);
  mipi_dsi_host_ll_dpi_enable_frame_ack(host, true);

  /* Burst mode with sync pulses */

  mipi_dsi_host_ll_dpi_set_video_burst_type(host,
      MIPI_DSI_LL_VIDEO_BURST_WITH_SYNC_PULSES);
  mipi_dsi_host_ll_dpi_set_video_packet_pixel_num(host, PANEL_HRES);
  mipi_dsi_host_ll_dpi_set_trunks_num(host, 0);
  mipi_dsi_host_ll_dpi_set_null_packet_size(host, 0);

  /* Step 16: Set horizontal and vertical timing */

  mipi_dsi_hal_host_dpi_set_horizontal_timing(&g_dsi_hal,
      PANEL_HSYNC, PANEL_HBP, PANEL_HRES, PANEL_HFP);
  mipi_dsi_hal_host_dpi_set_vertical_timing(&g_dsi_hal,
      PANEL_VSYNC, PANEL_VBP, PANEL_VRES, PANEL_VFP);

  /* Step 17: Configure DSI Bridge
   * RGB565 input → RGB565 output (no color conversion)
   */

  mipi_dsi_brg_ll_set_num_pixel_bits(bridge,
      PANEL_HRES * PANEL_VRES * 16);  /* RGB565 = 16 bits per pixel */
  mipi_dsi_brg_ll_set_underrun_discard_count(bridge, PANEL_HRES);
  mipi_dsi_brg_ll_set_input_color_format(bridge,
      LCD_COLOR_FMT_RGB565);  /* Input: RGB565 from framebuffer */
  mipi_dsi_brg_ll_set_output_color_format(bridge,
      LCD_COLOR_FMT_RGB565, 0);  /* Output: RGB565 (no conversion) */
  mipi_dsi_brg_ll_set_flow_controller(bridge,
      MIPI_DSI_LL_FLOW_CONTROLLER_DMA);
  mipi_dsi_brg_ll_set_multi_block_number(bridge, 1);
  mipi_dsi_brg_ll_set_burst_len(bridge, 256);
  mipi_dsi_brg_ll_set_empty_threshold(bridge, 1024 - 256);

  /* NOTE: Bridge enable and DPI config update are deferred to
   * esp_mipi_dsi_start_refresh() — they must happen AFTER panel
   * DCS init (sleep out) and AFTER DMA is ready to feed the FIFO.
   * Enabling bridge too early causes underrun with sleeping panel.
   */

  lcdinfo("DPI configured: %dx%d @ %d MHz\n",
          PANEL_HRES, PANEL_VRES, DSI_DPI_CLK_MHZ);
}

/****************************************************************************
 * Name: dsi_alloc_framebuffer
 *
 * Description:
 *   Allocate the display buffers from the heap (PSRAM).
 *
 *   Both buffers are taken from ONE 2 * ESP_DSI_FB_SIZE allocation so that
 *   they form a single contiguous region. That is what lets /dev/fb0
 *   publish them as one mmap area, which in turn lets a producer such as
 *   the camera DMA fill them directly with no copy at all.
 *
 *   Fallbacks, in order: one contiguous double-size block, a single
 *   ESP_DSI_FB_SIZE block (single buffered), a 4KB test block.
 *
 ****************************************************************************/

static int dsi_alloc_framebuffer(void)
{
  uint16_t *fb16;
  int i;
  int n;

  g_fb_count      = 0;
  g_fb_front      = 0;
  g_fb_contiguous = false;
  g_fb_base       = NULL;

  /* Preferred layout: one 64-byte aligned allocation holding both buffers
   * back to back. ESP_DSI_FB_SIZE is HRES * VRES * 2 = 1228800, a multiple
   * of 64, so g_framebuffer[1] = base + ESP_DSI_FB_SIZE is guaranteed to
   * land on a cache-line boundary as well.
   */

  DEBUGASSERT((ESP_DSI_FB_SIZE % 64) == 0);

  g_fb_base = (uint8_t *)kmm_memalign(64, ESP_DSI_FB_SIZE * 2);
  if (g_fb_base != NULL)
    {
      g_framebuffer[0] = g_fb_base;
      g_framebuffer[1] = g_fb_base + ESP_DSI_FB_SIZE;
      g_fb_count       = 2;
      g_fb_contiguous  = true;
      goto fill;
    }

  syslog(LOG_WARNING,
         "[DSI] Contiguous double FB alloc failed (%d bytes), "
         "falling back to single buffer\n",
         ESP_DSI_FB_SIZE * 2);

  /* Fallback: a single full-size framebuffer, no page flipping. */

  g_fb_base = (uint8_t *)kmm_memalign(64, ESP_DSI_FB_SIZE);
  if (g_fb_base != NULL)
    {
      g_framebuffer[0] = g_fb_base;
      g_framebuffer[1] = NULL;
      g_fb_count       = 1;
      goto fill;
    }

  /* Last resort: a 4KB test buffer, enough to prove the pipeline runs. */

  syslog(LOG_WARNING,
         "Full FB alloc failed (%d bytes), using small test buffer\n",
         ESP_DSI_FB_SIZE);

  g_fb_base        = (uint8_t *)kmm_memalign(64, 4096);
  g_framebuffer[0] = g_fb_base;
  if (g_framebuffer[0] == NULL)
    {
      lcderr("ERROR: Even small FB alloc failed\n");
      return -ENOMEM;
    }

  /* Fill with test pattern (red in RGB565 = 0xF800) */

  fb16 = (uint16_t *)g_framebuffer[0];
  for (i = 0; i < 2048; i++)
    {
      fb16[i] = 0xf800;  /* Red */
    }

  g_fb_count = 1;
  syslog(LOG_INFO, "Small test framebuffer allocated: %p (4096 bytes)\n",
         g_framebuffer[0]);
  return OK;

fill:

  /* Fill every live buffer with RED (RGB565: 0xF800) so that switching the
   * display to the back buffer before the first camera frame arrives can
   * never show uninitialised memory.
   */

  for (n = 0; n < g_fb_count; n++)
    {
      fb16 = (uint16_t *)g_framebuffer[n];
      for (i = 0; i < ESP_DSI_HRES * ESP_DSI_VRES; i++)
        {
          fb16[i] = 0xf800;
        }
    }

  DEBUGASSERT(g_fb_count < 2 ||
              ((uintptr_t)g_framebuffer[1] % 64) == 0);

  syslog(LOG_INFO,
         "Framebuffers: count=%d contiguous=%d [0]=%p [1]=%p size=%d "
         "(filled RED)\n",
         g_fb_count, (int)g_fb_contiguous, g_framebuffer[0],
         g_framebuffer[1], ESP_DSI_FB_SIZE);
  return OK;
}

/****************************************************************************
 * Name: dsi_start_video
 *
 * Description:
 *   Switch to video mode and enable DPI output.
 *   For this initial implementation, we do NOT set up continuous DMA.
 *   The display will show whatever is in the Bridge FIFO / last written.
 *
 *   TODO: Implement DW-GDMA continuous refresh for production use.
 *
 *   Corresponds to Steps 18-21.
 *
 ****************************************************************************/

static void dsi_start_video(void)
{
  /* Video mode and DPI output are NOT enabled here.
   * Per ESP-IDF reference, the correct sequence is:
   *   1. Start DMA first (so bridge has data to send)
   *   2. THEN enable video mode + DPI output
   * This is done in esp_mipi_dsi_start_refresh() after DMA channel enable.
   */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_mipi_dsi_initialize
 *
 * Description:
 *   Full DSI initialization sequence for EK79007 panel.
 *
 ****************************************************************************/

int esp_mipi_dsi_initialize(void)
{
  int ret;

  syslog(LOG_INFO, "[DSI] === MIPI-DSI init START ===\n");

  /* Phase 1: Enable PHY LDO and clocks */

  dsi_enable_phy_ldo();
  dsi_enable_clocks();

  /* Phase 2: Initialize PHY (Steps 4-6) */

  ret = dsi_init_phy();
  if (ret < 0)
    {
      return ret;
    }


  /* Phase 3: Configure Host for command mode (Steps 7-12) */

  dsi_configure_host();

  /* Phase 4: Reset panel and send DCS commands */

  dsi_reset_panel();
  dsi_send_panel_commands();

  /* Phase 5: Configure DPI / Video mode (Steps 14-17) */

  dsi_configure_dpi();

  /* Phase 6: Allocate framebuffer */

  ret = dsi_alloc_framebuffer();
  if (ret < 0)
    {
      return ret;
    }


  /* Phase 7: Start video output (Steps 20-21) */

  dsi_start_video();

  /* Write initial framebuffer content to bridge */


  /* Start continuous framebuffer refresh via DMA.
   * EK79007AD requires: HS Video Pattern started BEFORE backlight.
   * DMA will continuously feed bridge FIFO from framebuffer.
   */

  /* DMA refresh disabled (causes hang during bringup).
   * Use CPU refresh thread instead.
   */

  /* Wait 200ms for AVDD to stabilize before enabling backlight
   * (per EK79007AD datasheet power-on sequence)
   */

  dsi_delay_ms(200);

  /* Enable backlight */

  dsi_enable_backlight();

  syslog(LOG_INFO, "[DSI] === MIPI-DSI init COMPLETE ===\n");

  /* Note: Continuous refresh not started here.
   * The fb0 pandisplay ioctl will call esp_mipi_dsi_flush_fb_n()
   * when nxcamera/apps write to the framebuffer.
   */

  return OK;
}

/****************************************************************************
 * Name: esp_mipi_dsi_get_fb
 ****************************************************************************/

uint8_t *esp_mipi_dsi_get_fb(void)
{
  /* Return the buffer the display DMA is reading right now. With no
   * buffer switching going on this is always g_framebuffer[0], i.e. the
   * behaviour the single-buffer users (fb0, demo thread) already rely on.
   */

  return g_framebuffer[g_fb_front];
}

/****************************************************************************
 * Name: esp_mipi_dsi_get_fb_count
 ****************************************************************************/

int esp_mipi_dsi_get_fb_count(void)
{
  return g_fb_count;
}

/****************************************************************************
 * Name: esp_mipi_dsi_get_fb_n
 ****************************************************************************/

uint8_t *esp_mipi_dsi_get_fb_n(int index)
{
  if (index < 0 || index >= g_fb_count)
    {
      return NULL;
    }

  return g_framebuffer[index];
}

/****************************************************************************
 * Name: esp_mipi_dsi_fb_is_contiguous
 *
 * Description:
 *   True when the display buffers form one contiguous region, so they can
 *   be mapped as a single mmap area.
 *
 ****************************************************************************/

bool esp_mipi_dsi_fb_is_contiguous(void)
{
  return g_fb_contiguous;
}

/****************************************************************************
 * Name: esp_mipi_dsi_set_front_fb
 *
 * Description:
 *   Point the display DMA at buffer 'index'. The caller is responsible for
 *   having written that buffer back to memory first.
 *
 *   The transfer-done callback reprograms the link list item from
 *   g_dsi_xfer_config after every completed frame, so updating the source
 *   address here is enough: the switch takes effect on the next frame
 *   boundary. The running channel and the link list item are deliberately
 *   left untouched, they belong to the ISR.
 *
 ****************************************************************************/

int esp_mipi_dsi_set_front_fb(int index)
{
  irqstate_t flags;

  if (index < 0 || index >= g_fb_count || g_framebuffer[index] == NULL)
    {
      return -EINVAL;
    }

  /* g_dsi_xfer_config is read by the DMA ISR. Keep the critical section
   * down to the two field updates so a torn 32-bit source address can
   * never be observed.
   */

  flags = enter_critical_section();
  g_dsi_xfer_config.src.addr = (uint32_t)(uintptr_t)g_framebuffer[index];
  g_fb_front = index;
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: esp_mipi_dsi_flush_fb
 *
 * Description:
 *   Write the framebuffer back from the CPU data cache to physical memory
 *   so that the DW-GDMA engine feeding the DSI bridge sees fresh pixels.
 *
 *   The bridge is fed continuously by DMA, which re-reads the framebuffer
 *   through its normal (cached) address. Producers therefore write the
 *   framebuffer through that same cached address and call this function
 *   afterwards; without the writeback the DMA keeps sending stale lines.
 *
 ****************************************************************************/

void esp_mipi_dsi_flush_fb(void)
{
  /* Write back the very buffer esp_mipi_dsi_get_fb() handed out, so the
   * two stay consistent for the single-buffer users.
   */

  esp_mipi_dsi_flush_fb_n(g_fb_front);
}

/****************************************************************************
 * Name: esp_mipi_dsi_flush_fb_n
 *
 * Description:
 *   Same as esp_mipi_dsi_flush_fb() but for an explicitly named buffer.
 *
 *   A page-flipping producer draws into the BACK buffer and then makes it
 *   front, so it has to write back the buffer it is about to publish, not
 *   the one currently on screen. Out-of-range indices are ignored.
 *
 ****************************************************************************/

void esp_mipi_dsi_flush_fb_n(int index)
{
  if (index < 0 || index >= g_fb_count || g_framebuffer[index] == NULL)
    {
      return;
    }

  esp_cache_msync(g_framebuffer[index], ESP_DSI_FB_SIZE,
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);
}

/****************************************************************************
 * Name: esp_mipi_dsi_start_refresh
 *
 * Description:
 *   Start continuous DW-GDMA transfer from framebuffer to DSI bridge FIFO
 *   using the high-level DW-GDMA driver with ISR-based re-arm.
 *
 *   Mirrors ESP-IDF esp_lcd_panel_dpi.c: MEM→PERIPH_DSI, flow=SELF,
 *   HW handshake, LIST block transfer mode, trans_done callback re-arm.
 *
 *   MUST be called after DW-GDMA controller is enabled (camera init).
 ****************************************************************************/

/****************************************************************************
 * Name: dsi_dma_trans_done_cb
 *
 * Description:
 *   DMA transfer-done callback. Re-arms the DMA for continuous refresh.
 *   This matches ESP-IDF's mipi_dsi_dma_trans_done_cb exactly.
 ****************************************************************************/

static bool dsi_dma_trans_done_cb(dw_gdma_channel_handle_t chan,
    const dw_gdma_trans_done_event_data_t *event_data, void *user_data)
{
  (void)event_data;
  (void)user_data;

  /* Re-configure LLI and re-enable channel */

  dw_gdma_lli_config_transfer(
      dw_gdma_link_list_get_item(g_dsi_link_list, 0),
      &g_dsi_xfer_config);

  dw_gdma_block_markers_t m = {
      .is_valid = true,
      .is_last = true,
      .en_trans_done_intr = true,
  };

  dw_gdma_lli_set_block_markers(
      dw_gdma_link_list_get_item(g_dsi_link_list, 0), m);

  dw_gdma_channel_use_link_list(chan, g_dsi_link_list);
  dw_gdma_channel_enable_ctrl(chan, true);
  return false;
}

/****************************************************************************
 * Name: dsi_dma_start_linked_list
 *
 * Description:
 *   Configure DMA using the high-level DW-GDMA driver (matching ESP-IDF
 *   esp_lcd_panel_dpi.c exactly) and start continuous refresh.
 ****************************************************************************/

static void dsi_dma_start_linked_list(void)
{
  esp_err_t err;

  /* 1. Allocate DMA channel (matches ESP-IDF DSI panel config) */

  dw_gdma_channel_alloc_config_t dma_alloc_config = {
      .src = {
          .block_transfer_type = DW_GDMA_BLOCK_TRANSFER_LIST,
          .role = DW_GDMA_ROLE_MEM,
          .handshake_type = DW_GDMA_HANDSHAKE_HW,
          .num_outstanding_requests = 5,
      },
      .dst = {
          .block_transfer_type = DW_GDMA_BLOCK_TRANSFER_LIST,
          .role = DW_GDMA_ROLE_PERIPH_DSI,
          .handshake_type = DW_GDMA_HANDSHAKE_HW,
          .num_outstanding_requests = 2,
      },
      .flow_controller = DW_GDMA_FLOW_CTRL_SELF,
      .chan_priority = 1,
  };

  err = dw_gdma_new_channel(&dma_alloc_config, &g_dsi_dma_chan);
  if (err != ESP_OK)
    {
      syslog(LOG_ERR, "[DSI] Failed to allocate DMA channel: 0x%x\n", err);
      return;
    }

  /* 2. Create link list (1 item, singly linked) */

  dw_gdma_link_list_config_t link_list_config = {
      .num_items = 1,
      .link_type = DW_GDMA_LINKED_LIST_TYPE_SINGLY,
  };

  err = dw_gdma_new_link_list(&link_list_config, &g_dsi_link_list);
  if (err != ESP_OK)
    {
      syslog(LOG_ERR, "[DSI] Failed to create link list: 0x%x\n", err);
      return;
    }

  /* 3. Configure transfer parameters */

  g_dsi_xfer_config.src.addr =
      (uint32_t)(uintptr_t)g_framebuffer[g_fb_front];
  g_dsi_xfer_config.src.burst_mode = DW_GDMA_BURST_MODE_INCREMENT;
  g_dsi_xfer_config.src.burst_items = DW_GDMA_BURST_ITEMS_512;
  g_dsi_xfer_config.src.burst_len = 16;
  g_dsi_xfer_config.src.width = DW_GDMA_TRANS_WIDTH_64;
  g_dsi_xfer_config.dst.addr = MIPI_DSI_BRG_MEM_BASE;
  g_dsi_xfer_config.dst.burst_mode = DW_GDMA_BURST_MODE_FIXED;
  g_dsi_xfer_config.dst.burst_items = DW_GDMA_BURST_ITEMS_256;
  g_dsi_xfer_config.dst.burst_len = 16;
  g_dsi_xfer_config.dst.width = DW_GDMA_TRANS_WIDTH_64;
  g_dsi_xfer_config.size = ESP_DSI_FB_SIZE * 8 / 64;

  dw_gdma_lli_config_transfer(
      dw_gdma_link_list_get_item(g_dsi_link_list, 0),
      &g_dsi_xfer_config);

  /* 4. Set block markers */

  dw_gdma_block_markers_t markers = {
      .is_valid = true,
      .is_last = true,
      .en_trans_done_intr = true,
  };

  dw_gdma_lli_set_block_markers(
      dw_gdma_link_list_get_item(g_dsi_link_list, 0), markers);

  /* 5. Register trans_done callback for ISR re-arm */

  dw_gdma_event_callbacks_t cbs = {
      .on_full_trans_done = dsi_dma_trans_done_cb,
  };

  err = dw_gdma_channel_register_event_callbacks(
            g_dsi_dma_chan, &cbs, NULL);
  if (err != ESP_OK)
    {
      syslog(LOG_ERR, "[DSI] Failed to register DMA callbacks: 0x%x\n",
             err);
      return;
    }

  /* 6. Apply link list and enable channel */

  dw_gdma_channel_use_link_list(g_dsi_dma_chan, g_dsi_link_list);
  dw_gdma_channel_enable_ctrl(g_dsi_dma_chan, true);

  syslog(LOG_INFO, "[DSI] DW-GDMA high-level driver started\n");
}

void esp_mipi_dsi_start_refresh(void)
{
  uint8_t *fb = g_framebuffer[g_fb_front];

  if (fb == NULL)
    {
      return;
    }

  /* Write back framebuffer to physical PSRAM for DMA access.
   * On ESP32-P4, PSRAM at 0x48xxxxxx has non-cached alias at 0x88xxxxxx.
   * Belt-and-suspenders: do cache writeback AND write through non-cached.
   */

  esp_cache_msync((void *)fb, ESP_DSI_FB_SIZE,
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);

  /* Fill red through the cached address, then write back. Never use the
   * non-cached alias here: the DMA source is the cached address, so mixing
   * the two views leaves the DMA reading stale cache lines.
   */

  {
    uint16_t *fb16 = (uint16_t *)fb;
    uint32_t j;

    for (j = 0; j < ESP_DSI_HRES * ESP_DSI_VRES; j++)
      {
        fb16[j] = 0xf800;  /* Red */
      }

    esp_cache_msync(fb, ESP_DSI_FB_SIZE,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  }

  /* Re-send Sleep Out and Display On in command mode (belt-and-suspenders).
   * This ensures the panel is awake even if initial DCS commands
   * during initialization didn't reach it properly.
   */

  syslog(LOG_INFO, "[DSI] Re-sending Sleep Out + Display On...\n");
  mipi_dsi_hal_host_gen_write_dcs_command(&g_dsi_hal, 0, 0x11, 1, NULL, 0);
  dsi_delay_ms(120);
  mipi_dsi_hal_host_gen_write_dcs_command(&g_dsi_hal, 0, 0x29, 1, NULL, 0);
  dsi_delay_ms(50);

  /* Enable the bridge (must happen after panel DCS init, before DMA) */

  syslog(LOG_INFO, "[DSI] Enabling bridge...\n");
  mipi_dsi_brg_ll_enable(g_dsi_hal.bridge, true);
  mipi_dsi_brg_ll_update_dpi_config(g_dsi_hal.bridge);

  /* Start linked-list DMA — begins pushing FB to bridge FIFO */

  syslog(LOG_INFO, "[DSI] Starting DMA linked-list...\n");
  dsi_dma_start_linked_list();

  /* Wait for DMA to fill some data into bridge FIFO before enabling
   * video mode. Without data in FIFO, Host won't have anything to send.
   */

  dsi_delay_ms(50);

  /* NOW enable video mode + DPI output (bridge has data to send) */

  syslog(LOG_INFO, "[DSI] Enabling video mode + DPI output...\n");
  mipi_dsi_host_ll_enable_video_mode(g_dsi_hal.host, true);
  dsi_delay_ms(10);
  mipi_dsi_brg_ll_enable_dpi_output(g_dsi_hal.bridge, true);
  mipi_dsi_brg_ll_update_dpi_config(g_dsi_hal.bridge);

  syslog(LOG_INFO, "[DSI] Linked-list DMA refresh started\n");
}

/****************************************************************************
 * Name: dsi_demo_thread
 *
 * Description:
 *   Polling DMA re-arm + color demo thread.
 *   Checks if DMA channel has stopped (CHEN bit cleared) and re-arms it.
 *   Also alternates red/blue every 2 seconds.
 ****************************************************************************/

static int dsi_demo_thread(int argc, FAR char *argv[])
{
  uint16_t *fb16;
  bool show_red = true;
  uint32_t j;

  (void)argc;
  (void)argv;

  if (g_framebuffer[0] == NULL)
    {
      return -EINVAL;
    }

  /* Write through the normal (cached) framebuffer address and write back
   * with esp_cache_msync afterwards. This is the ESP-IDF pattern used by
   * esp_lcd_dpi_panel_draw_bitmap(). Do NOT mix non-cached alias writes
   * with a cached framebuffer: the DMA reads the cached address and would
   * see stale data.
   */

  while (1)
    {
      /* Alternate colors every 2 seconds */

      usleep(2000000);

      /* Always draw into the buffer the display is reading, re-read every
       * iteration in case something switched the front buffer.
       */

      fb16 = (uint16_t *)esp_mipi_dsi_get_fb();

      show_red = !show_red;
      if (show_red)
        {
          for (j = 0; j < ESP_DSI_HRES * ESP_DSI_VRES; j++)
            {
              fb16[j] = 0xf800;  /* Red */
            }
        }
      else
        {
          for (j = 0; j < ESP_DSI_HRES * ESP_DSI_VRES; j++)
            {
              fb16[j] = 0x001f;  /* Blue */
            }
        }

      esp_cache_msync((uint8_t *)fb16, ESP_DSI_FB_SIZE,
                      ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    }

  return 0;
}

/****************************************************************************
 * Name: esp_mipi_dsi_start_demo
 *
 * Description:
 *   Start the red/blue alternating demo thread.
 *   Call after esp_mipi_dsi_start_refresh().
 ****************************************************************************/

static int g_demo_pid = -1;

void esp_mipi_dsi_start_demo(void)
{
  if (g_demo_pid > 0)
    {
      return;
    }

  g_demo_pid = kthread_create("dsi_demo", 100,
                              CONFIG_DEFAULT_TASK_STACKSIZE,
                              dsi_demo_thread, NULL);
  if (g_demo_pid < 0)
    {
      syslog(LOG_ERR, "[DSI] Demo thread failed: %d\n", g_demo_pid);
    }
  else
    {
      syslog(LOG_INFO, "[DSI] Demo thread started (pid=%d)\n", g_demo_pid);
    }
}
