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

#include "esp_mipi_dsi.h"

/* HAL includes */

#include "hal/mipi_dsi_hal.h"
#include "hal/mipi_dsi_ll.h"
#include "hal/mipi_dsi_host_ll.h"
#include "hal/mipi_dsi_phy_ll.h"
#include "hal/mipi_dsi_brg_ll.h"

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

/* Bridge FIFO memory base address for DMA writes */

#define MIPI_DSI_BRG_MEM_BASE  0x50108000

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

/* Framebuffer pointer (allocated from PSRAM) */

static uint8_t *g_framebuffer;

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

  /* Pull low (assert reset, active low) */

  dsi_gpio_set_level(DSI_PANEL_RESET_GPIO, false);
  dsi_delay_ms(10);

  /* Pull high (release reset) */

  dsi_gpio_set_level(DSI_PANEL_RESET_GPIO, true);
  dsi_delay_ms(10);

  lcdinfo("Panel reset complete (GPIO %d)\n", DSI_PANEL_RESET_GPIO);
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

  /* Step 5: Configure PHY PLL (calculates M/N, writes PHY registers) */

  mipi_dsi_hal_configure_phy_pll(&g_dsi_hal, DSI_PHY_CLK_FREQ,
                                 (float)DSI_LANE_RATE_MBPS);

  /* Step 6: Wait for PLL lock and lanes stopped */

  timeout = 10000;
  while (!mipi_dsi_phy_ll_is_pll_locked(g_dsi_hal.host) && timeout > 0)
    {
      timeout--;
    }

  if (timeout <= 0)
    {
      lcdwarn("WARNING: DSI PHY PLL failed to lock (continuing anyway)\n");
    }

  timeout = 10000;
  while (!mipi_dsi_phy_ll_are_lanes_stopped(g_dsi_hal.host,
         DSI_NUM_LANES) && timeout > 0)
    {
      timeout--;
    }

  if (timeout <= 0)
    {
      lcdwarn("WARNING: DSI lanes failed to enter stop state "
              "(continuing anyway)\n");
    }

  lcdinfo("DSI PHY initialized, PLL %s\n",
          mipi_dsi_phy_ll_is_pll_locked(g_dsi_hal.host) ?
          "locked" : "NOT locked (soft fail)");
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
  mipi_dsi_brg_ll_enable(bridge, true);
  mipi_dsi_brg_ll_update_dpi_config(bridge);

  lcdinfo("DPI configured: %dx%d @ %d MHz\n",
          PANEL_HRES, PANEL_VRES, DSI_DPI_CLK_MHZ);
}

/****************************************************************************
 * Name: dsi_alloc_framebuffer
 *
 * Description:
 *   Allocate framebuffer from heap (ideally PSRAM).
 *   For now use kmm_memalign for cache-line aligned allocation.
 *
 ****************************************************************************/

static int dsi_alloc_framebuffer(void)
{
  /* Allocate framebuffer aligned to 64 bytes (cache line) */

  g_framebuffer = (uint8_t *)kmm_memalign(64, ESP_DSI_FB_SIZE);
  if (g_framebuffer == NULL)
    {
      /* Full framebuffer allocation failed, try minimal test buffer */

      syslog(LOG_WARNING,
             "Full FB alloc failed (%d bytes), using small test buffer\n",
             ESP_DSI_FB_SIZE);
      g_framebuffer = (uint8_t *)kmm_memalign(64, 4096);
      if (g_framebuffer == NULL)
        {
          lcderr("ERROR: Even small FB alloc failed\n");
          return -ENOMEM;
        }

      /* Fill with test pattern (red in RGB565 = 0xF800) */

      uint16_t *fb16 = (uint16_t *)g_framebuffer;
      int i;
      for (i = 0; i < 2048; i++)
        {
          fb16[i] = 0xf800;  /* Red */
        }

      syslog(LOG_INFO, "Small test framebuffer allocated: %p (4096 bytes)\n",
             g_framebuffer);
      return OK;
    }

  /* Fill full framebuffer with blue (0x001F in RGB565) */

  uint16_t *fb16 = (uint16_t *)g_framebuffer;
  int i;
  for (i = 0; i < (ESP_DSI_FB_SIZE / 2); i++)
    {
      fb16[i] = 0x001f;  /* Blue */
    }

  syslog(LOG_INFO, "Framebuffer allocated: %p, size=%d bytes\n",
         g_framebuffer, ESP_DSI_FB_SIZE);
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
  mipi_dsi_host_soc_handle_t host = g_dsi_hal.host;
  mipi_dsi_bridge_soc_handle_t bridge = g_dsi_hal.bridge;

  /* Step 20: Switch to video mode */

  mipi_dsi_host_ll_enable_video_mode(host, true);

  /* Enable DPI output from bridge */

  mipi_dsi_brg_ll_enable_dpi_output(bridge, true);
  mipi_dsi_brg_ll_update_dpi_config(bridge);

  lcdinfo("DSI video mode started\n");
}

/****************************************************************************
 * Name: dsi_write_fb_to_bridge
 *
 * Description:
 *   CPU-based framebuffer write to Bridge FIFO.
 *   This is a simplified approach for initial bring-up.
 *   In production, DW-GDMA would handle this continuously.
 *
 ****************************************************************************/

static void dsi_write_fb_to_bridge(void)
{
  volatile uint32_t *brg_mem = (volatile uint32_t *)MIPI_DSI_BRG_MEM_BASE;
  uint32_t *fb32 = (uint32_t *)g_framebuffer;
  uint32_t fb_size;
  uint32_t words;
  uint32_t i;

  /* Determine actual buffer size based on whether full alloc succeeded */

  if (g_framebuffer == NULL)
    {
      return;
    }

  /* Check if we got a small buffer by testing allocation */

  fb_size = ESP_DSI_FB_SIZE;

  /* Try to detect small buffer: if the first pixel at offset 0 is red
   * (0xF800) we are using the small test buffer.
   */

  uint16_t *fb16 = (uint16_t *)g_framebuffer;
  if (fb16[0] == 0xf800)
    {
      fb_size = 4096;
      syslog(LOG_INFO, "Writing small test buffer to bridge (%lu bytes)\n",
             (unsigned long)fb_size);
    }

  words = fb_size / 4;
  for (i = 0; i < words; i++)
    {
      *brg_mem = fb32[i];
    }
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
  syslog(LOG_INFO, "[DSI] === NOOP TEST - just return OK ===\n");
  return OK;

#if 0 /* Full DSI init - disabled for crash debugging */
  /* Phase 2: Initialize PHY (Steps 4-6) */

  syslog(LOG_INFO, "[DSI] Phase 2: PHY init...\n");
  ret = dsi_init_phy();
  if (ret < 0)
    {
      return ret;
    }

  syslog(LOG_INFO, "[DSI] Phase 2: done\n");

  /* Phase 3: Configure Host for command mode (Steps 7-12) */

  syslog(LOG_INFO, "[DSI] Phase 3: host config...\n");
  dsi_configure_host();
  syslog(LOG_INFO, "[DSI] Phase 3: done\n");

  /* Phase 4: Send panel DCS commands (Step 13) */

  syslog(LOG_INFO, "[DSI] Phase 4: panel cmds...\n");
  dsi_send_panel_commands();
  syslog(LOG_INFO, "[DSI] Phase 4: done\n");

  /* Phase 5: Configure DPI / Video mode (Steps 14-17) */

  syslog(LOG_INFO, "[DSI] Phase 5: DPI config...\n");
  dsi_configure_dpi();
  syslog(LOG_INFO, "[DSI] Phase 5: done\n");

  /* Phase 6: Allocate framebuffer */

  syslog(LOG_INFO, "[DSI] Phase 6: FB alloc...\n");
  ret = dsi_alloc_framebuffer();
  if (ret < 0)
    {
      syslog(LOG_ERR, "[DSI] Phase 6: FAILED %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "[DSI] Phase 6: done\n");

  /* Phase 7: Start video output (Steps 20-21) */

  syslog(LOG_INFO, "[DSI] Phase 7: video start...\n");
  dsi_start_video();
  syslog(LOG_INFO, "[DSI] Phase 7: done\n");

  /* Write initial framebuffer content to bridge */

  syslog(LOG_INFO, "[DSI] Phase 7.5: FB write to bridge...\n");
  dsi_write_fb_to_bridge();
  syslog(LOG_INFO, "[DSI] Phase 7.5: done\n");

  /* Phase 8: Enable backlight (GPIO 26) */

  syslog(LOG_INFO, "[DSI] Phase 8: backlight...\n");
  dsi_enable_backlight();
  syslog(LOG_INFO, "[DSI] === MIPI-DSI init COMPLETE ===\n");
  return OK;
#endif
}

/****************************************************************************
 * Name: esp_mipi_dsi_get_fb
 ****************************************************************************/

uint8_t *esp_mipi_dsi_get_fb(void)
{
  return g_framebuffer;
}
