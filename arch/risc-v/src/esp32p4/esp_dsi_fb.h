/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_dsi_fb.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_ESP32P4_ESP_DSI_FB_H
#define __ARCH_RISCV_SRC_ESP32P4_ESP_DSI_FB_H

#include <nuttx/config.h>
#include <stdint.h>

/* Panel parameters (EK79007) */

#define ESP_DSI_HRES              1024
#define ESP_DSI_VRES              600
#define ESP_DSI_FB_BPP            24   /* RGB888 */
#define ESP_DSI_FB_SIZE           (ESP_DSI_HRES * ESP_DSI_VRES * 3)

#ifdef __cplusplus
extern "C"
{
#endif

int esp_dsi_fb_initialize(void);
int esp_dsi_fb_start_refresh(void);
uint8_t *esp_dsi_fb_get_buffer(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_ESP32P4_ESP_DSI_FB_H */
