/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_mipi_dsi.h
 *
 * Compatibility redirect — legacy API maps to new esp_dsi_fb.
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_ESP32P4_ESP_MIPI_DSI_H
#define __ARCH_RISCV_SRC_ESP32P4_ESP_MIPI_DSI_H

#include "esp_dsi_fb.h"

/* Legacy API mappings */

#define esp_mipi_dsi_initialize()    esp_dsi_fb_initialize()
#define esp_mipi_dsi_get_fb()        esp_dsi_fb_get_buffer()
#define esp_mipi_dsi_flush_fb()
#define esp_mipi_dsi_start_refresh()

#endif /* __ARCH_RISCV_SRC_ESP32P4_ESP_MIPI_DSI_H */
