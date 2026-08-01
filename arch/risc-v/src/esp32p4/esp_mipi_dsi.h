/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_mipi_dsi.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_ESP32P4_ESP_MIPI_DSI_H
#define __ARCH_RISCV_SRC_ESP32P4_ESP_MIPI_DSI_H

#include <nuttx/config.h>
#include <stdint.h>

/* Panel parameters (EK79007) */

#define ESP_DSI_HRES              1024
#define ESP_DSI_VRES              600
#define ESP_DSI_FB_BPP            24   /* RGB888 */
#define ESP_DSI_FB_SIZE           (ESP_DSI_HRES * ESP_DSI_VRES * 3)

/* Panel timing (EK79007) */

#define ESP_DSI_HSYNC             10
#define ESP_DSI_HBP               120
#define ESP_DSI_HFP               120
#define ESP_DSI_VSYNC             1
#define ESP_DSI_VBP               20
#define ESP_DSI_VFP               10

/* DSI Configuration */

#define ESP_DSI_BUS_ID            0
#define ESP_DSI_NUM_DATA_LANES    2
#define ESP_DSI_LANE_BITRATE_MBPS 1000
#define ESP_DSI_DPI_CLK_MHZ       48
#define ESP_DSI_DPI_CLK_SRC_MHZ   240
#define ESP_DSI_PHY_CLK_SRC_FREQ  40000000

/* PHY LDO Configuration */

#define ESP_DSI_PHY_LDO_CHAN      3
#define ESP_DSI_PHY_LDO_MV        2500

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: esp_mipi_dsi_initialize
 *
 * Description:
 *   Initialize DSI subsystem: PHY LDO, clocks, PHY PLL, host config,
 *   panel reset, DCS commands, DPI config, framebuffer allocation.
 *   Does NOT start video output (call esp_mipi_dsi_start_refresh).
 ****************************************************************************/

int esp_mipi_dsi_initialize(void);

/****************************************************************************
 * Name: esp_mipi_dsi_start_refresh
 *
 * Description:
 *   Start continuous DMA-driven DSI video output.
 *   Must be called after esp_mipi_dsi_initialize().
 ****************************************************************************/

void esp_mipi_dsi_start_refresh(void);

/****************************************************************************
 * Name: esp_mipi_dsi_get_fb
 *
 * Description:
 *   Get pointer to the RGB888 framebuffer.
 ****************************************************************************/

uint8_t *esp_mipi_dsi_get_fb(void);

/****************************************************************************
 * Name: esp_mipi_dsi_flush_fb
 *
 * Description:
 *   Write the framebuffer back from the CPU data cache to physical memory
 *   so the DMA feeding the DSI bridge sees fresh pixels. Call after every
 *   framebuffer update, which must be done via the normal cached address.
 ****************************************************************************/

void esp_mipi_dsi_flush_fb(void);

/****************************************************************************
 * Name: esp_mipi_dsi_start_demo
 *
 * Description:
 *   Start red/blue alternating demo thread (for display verification).
 ****************************************************************************/

void esp_mipi_dsi_start_demo(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_ESP32P4_ESP_MIPI_DSI_H */
