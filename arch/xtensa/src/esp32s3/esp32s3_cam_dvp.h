/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_cam_dvp.h
 *
 * Public interface for the ESP32-S3 DVP camera capture driver
 * (LCD_CAM CAM input mode + GDMA RX, per-frame ISR re-arm).
 *
 * [DIAG] Camera bring-up. Keep until capture is fully working.
 ****************************************************************************/

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_CAM_DVP_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_CAM_DVP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stddef.h>
#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_CAM_DVP

/* Initialize DVP capture hardware (GPIO + LCD_CAM CAM mode + GDMA RX). */

int esp32s3_cam_dvp_init(void);

/* Arm DMA RX chain and start CAM capture. */

int esp32s3_cam_dvp_start(void);

/* Stop CAM capture. */

int esp32s3_cam_dvp_stop(void);

/* Capture one complete RGB565 frame (vs_eof=1, one aligned frame per VSYNC;
 * sink is called once with CAM_FRAME_SIZE bytes). Returns CAM_FRAME_SIZE on
 * success, or negated errno.
 */

int esp32s3_cam_dvp_capture(void (*sink)(const uint8_t *data, size_t len,
                                         void *arg),
                            void *sink_arg,
                            volatile bool *eoi_out);

/* Layered hardware diagnosis: print raw register state of every link in the
 * capture chain (camera GPIO / CAM ctrl / clock / int / GDMA).
 */

void esp32s3_cam_dvp_diag(void);

/* Return the internal static DMA RX buffer (first CAM_FRAME_SIZE bytes hold
 * the latest captured frame). Avoids a second frame buffer copy.
 */

uint8_t *esp32s3_cam_dvp_get_frame(void);

/* Runtime frame geometry switching (preview 160x120 / capture 640x480). DMA
 * chains and buffers are sized for the maximum Frame allocation, call only
 * when capture stops; after switch AEC/AWB will re-run warmup.
 */

int esp32s3_cam_dvp_set_framesize(uint32_t w, uint32_t h);
void esp32s3_cam_dvp_get_framesize(uint32_t *w, uint32_t *h);

/* Runtime toggle for the software RGB565 byte swap (default ON). */

void esp32s3_cam_dvp_set_swap(bool on);
bool esp32s3_cam_dvp_get_swap(void);

/* Silent mode: disable per-frame capture logging (for preview thread) */

void esp32s3_cam_dvp_set_quiet(bool quiet);

/* Preview stream mode (rolling snapshot): capture does not freeze DMA, only
 * waits for one frame period per frame.
 */

void esp32s3_cam_dvp_set_stream(bool on);
bool esp32s3_cam_dvp_get_stream(void);

#endif /* CONFIG_ESP32S3_CAM_DVP */

#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_CAM_DVP_H */
