/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_dsi_fb.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <syslog.h>

#include "esp_dsi_fb.h"
#include "esp_lcd_dsi_bus_nuttx.h"
#include "esp_lcd_panel_dpi_nuttx.h"
#include "hal/mipi_dsi_hal.h"
#include "esp_cache.h"

/* GPIO register-direct access (proven working from previous code) */

#define GPIO_BASE         0x500E0000
#define IO_MUX_BASE       0x500E1000
#define GPIO_OUT_W1TS     (*(volatile uint32_t *)(GPIO_BASE + 0x08))
#define GPIO_OUT_W1TC     (*(volatile uint32_t *)(GPIO_BASE + 0x0C))
#define GPIO_ENABLE_W1TS  (*(volatile uint32_t *)(GPIO_BASE + 0x24))

/* PMU LDO for DSI PHY (2.5V, proven working) */

#define PMU_EXT_LDO_CTRL  (*(volatile uint32_t *)0x501151C0)
#define PMU_EXT_LDO_ANA   (*(volatile uint32_t *)0x501151C4)

static esp_lcd_dsi_bus_handle_t g_dsi_bus;
static esp_lcd_dpi_panel_handle_t g_dpi_panel;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void dsi_phy_ldo_enable(void)
{
  /* LDO channel 3 -> 2.5V: dref=13, mul=3 */

  PMU_EXT_LDO_ANA = (13u << 28) | (3u << 23);
  PMU_EXT_LDO_CTRL = (1u << 7) | (1u << 8); /* force_tieh_sel + xpd */
  usleep(5000); /* Wait for LDO to stabilize */
}

static void gpio_set_output(uint32_t pin)
{
  volatile uint32_t *iomux = (volatile uint32_t *)
      (IO_MUX_BASE + 0x04 + pin * 4);
  *iomux = (1u << 12); /* func_sel=1 (GPIO function) */
  GPIO_ENABLE_W1TS = (1u << pin);
}

static void panel_reset(void)
{
  gpio_set_output(27);

  /* Reset pulse: low 10ms -> high 85ms (meets EK79007 timing) */

  GPIO_OUT_W1TC = (1u << 27);
  usleep(10000);
  GPIO_OUT_W1TS = (1u << 27);
  usleep(85000);
}

static void panel_send_dcs(mipi_dsi_hal_context_t *hal)
{
  /* Sleep Out (0x11) - EK79007 only needs this one command */

  mipi_dsi_hal_host_gen_write_dcs_command(hal, 0, 0x11, 1, NULL, 0);
  usleep(120000); /* 120ms required by spec */
}

static void backlight_enable(void)
{
  gpio_set_output(26);
  GPIO_OUT_W1TS = (1u << 26);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int esp_dsi_fb_initialize(void)
{
  int ret;

  syslog(LOG_INFO, "[DSI-FB] Starting initialization\n");

  /* 1. PHY LDO (2.5V) */

  dsi_phy_ldo_enable();
  syslog(LOG_INFO, "[DSI-FB] PHY LDO enabled\n");

  /* 2. DSI Bus (PHY + Host) */

  esp_lcd_dsi_bus_config_t bus_cfg;
  memset(&bus_cfg, 0, sizeof(bus_cfg));
  bus_cfg.bus_id = 0;
  bus_cfg.num_data_lanes = 2;
  bus_cfg.lane_bit_rate_mbps = 1000.0f;
  bus_cfg.phy_clk_src = 0; /* default XTAL */
  bus_cfg.flags.clock_lane_force_hs = false;

  ret = esp_lcd_new_dsi_bus(&bus_cfg, &g_dsi_bus);
  if (ret != 0)
    {
      syslog(LOG_ERR, "[DSI-FB] Bus init failed: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "[DSI-FB] Bus init OK\n");

  /* 3. Panel reset + DCS commands (while in command mode) */

  panel_reset();
  panel_send_dcs(&g_dsi_bus->hal);
  syslog(LOG_INFO, "[DSI-FB] Panel reset + Sleep Out done\n");

  /* 4. DPI Panel (Bridge + DMA link) */

  esp_lcd_dpi_panel_config_t dpi_cfg;
  memset(&dpi_cfg, 0, sizeof(dpi_cfg));
  dpi_cfg.virtual_channel = 0;
  dpi_cfg.dpi_clock_freq_mhz = 52.0f;
  dpi_cfg.video_timing.h_size = 1024;
  dpi_cfg.video_timing.v_size = 600;
  dpi_cfg.video_timing.hsync_pulse_width = 10;
  dpi_cfg.video_timing.hsync_back_porch = 120;
  dpi_cfg.video_timing.hsync_front_porch = 120;
  dpi_cfg.video_timing.vsync_pulse_width = 1;
  dpi_cfg.video_timing.vsync_back_porch = 20;
  dpi_cfg.video_timing.vsync_front_porch = 10;

  ret = esp_lcd_new_panel_dpi(g_dsi_bus, &dpi_cfg, &g_dpi_panel);
  if (ret != 0)
    {
      syslog(LOG_ERR, "[DSI-FB] DPI panel create failed: %d\n", ret);
      return ret;
    }

  /* 5. DMA start + video mode will be called AFTER Camera init
   * (Camera's esp_csi_dma_init resets GDMA, which would erase our config)
   */

  syslog(LOG_INFO, "[DSI-FB] Phase 1 done (bus+panel created). "
         "Call esp_dsi_fb_start_refresh() after camera init.\n");

  return 0;
}

/****************************************************************************
 * Name: esp_dsi_fb_start_refresh
 *
 * Description:
 *   Start DMA refresh loop + video mode + backlight.
 *   MUST be called AFTER Camera CSI init (which resets GDMA).
 ****************************************************************************/

int esp_dsi_fb_start_refresh(void)
{
  int ret;

  if (g_dpi_panel == NULL)
    {
      return -EINVAL;
    }

  /* Fill framebuffer with white for visual verification */

  {
    uint8_t *fb = esp_lcd_dpi_panel_get_fb(g_dpi_panel);
    size_t fb_size = esp_lcd_dpi_panel_get_fb_size(g_dpi_panel);
    if (fb && fb_size > 0)
      {
        memset(fb, 0xFF, fb_size);  /* White = 0xFF for all RGB888 bytes */
        esp_cache_msync(fb, fb_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        syslog(LOG_INFO, "[DSI-FB] FB filled white (%zu bytes)\n", fb_size);
      }
  }

  /* Start DMA -> video mode -> DPI output */

  ret = esp_lcd_dpi_panel_init(g_dpi_panel);
  if (ret != 0)
    {
      syslog(LOG_ERR, "[DSI-FB] DPI panel init failed: %d\n", ret);
      return ret;
    }

  /* Wait 200ms for stable video, then backlight */

  usleep(200000);
  backlight_enable();
  syslog(LOG_INFO, "[DSI-FB] Backlight ON, display ready\n");

  return 0;
}

uint8_t *esp_dsi_fb_get_buffer(void)
{
  if (g_dpi_panel == NULL)
    {
      return NULL;
    }

  return esp_lcd_dpi_panel_get_fb(g_dpi_panel);
}
