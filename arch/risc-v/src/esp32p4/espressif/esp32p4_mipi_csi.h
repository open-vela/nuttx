/****************************************************************************
 * arch/risc-v/src/esp32p4/espressif/esp32p4_mipi_csi.h
 *
 * 原生 NuttX MIPI-CSI 驱动（无 FreeRTOS 依赖）。
 * 复用 in-tree esp-hal-3rdparty 的 mipi_csi_hal 与 dw_gdma 实现
 * 连续取帧。
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
  uint32_t h_res;              /* 每行有效像素 */
  uint32_t v_res;              /* 有效行数 */
  uint32_t lanes_num;          /* MIPI 数据 lane 数（1 或 2） */
  uint32_t lane_bit_rate_mbps; /* 每 lane 位速率 */
  uint32_t in_bpp;             /* 输入位深（RAW10=10） */
};

/* 取帧回调（ISR 上下文，只应做轻量标志/信号量操作） */

typedef void (*esp32p4_mipi_csi_frame_cb_t)(void *buf, size_t len,
                                            void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int esp32p4_mipi_csi_initialize(const struct esp32p4_mipi_csi_config_s *cfg);
int esp32p4_mipi_csi_start(esp32p4_mipi_csi_frame_cb_t frame_cb, void *arg);
int esp32p4_mipi_csi_stop(void);

/* 最新已完成帧缓冲（与 DMA 写入缓冲轮换，可直接读） */

void *esp32p4_mipi_csi_get_frame(void);
uint32_t esp32p4_mipi_csi_frame_count(void);
size_t esp32p4_mipi_csi_framelen(void);

#endif /* __ARCH_RISCV_SRC_ESP32P4_ESPRESSIF_ESP32P4_MIPI_CSI_H */
