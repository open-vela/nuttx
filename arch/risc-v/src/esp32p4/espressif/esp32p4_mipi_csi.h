/****************************************************************************
 * arch/risc-v/src/esp32p4/espressif/esp32p4_mipi_csi.h
 *
 * Native NuttX MIPI-CSI driver (no FreeRTOS dependency).
 * Frame capture reuses the in-tree esp-hal-3rdparty mipi_csi_hal
 * and dw_gdma for continuous streaming.
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_ESP32P4_ESPRESSIF_ESP32P4_MIPI_CSI_H
#define __ARCH_RISCV_SRC_ESP32P4_ESPRESSIF_ESP32P4_MIPI_CSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stddef.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct esp32p4_mipi_csi_config_s
{
  uint32_t h_res;              /* active pixels per line */
  uint32_t v_res;              /* active lines */
  uint32_t lanes_num;          /* MIPI data lanes (1 or 2) */
  uint32_t lane_bit_rate_mbps; /* bitrate per lane */
  uint32_t in_bpp;             /* input depth (RAW10 = 10) */
};

/* Frame callback (ISR context: flag/semaphore ops only) */

typedef void (*esp32p4_mipi_csi_frame_cb_t)(void *buf, size_t len,
                                            void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int esp32p4_mipi_csi_initialize(const struct esp32p4_mipi_csi_config_s *cfg);
int esp32p4_mipi_csi_start(esp32p4_mipi_csi_frame_cb_t frame_cb, void *arg);
int esp32p4_mipi_csi_stop(void);

/* Latest completed frame (rotates with the DMA buffer) */

void *esp32p4_mipi_csi_get_frame(void);
uint32_t esp32p4_mipi_csi_frame_count(void);
size_t esp32p4_mipi_csi_framelen(void);

#endif /* __ARCH_RISCV_SRC_ESP32P4_ESPRESSIF_ESP32P4_MIPI_CSI_H */
