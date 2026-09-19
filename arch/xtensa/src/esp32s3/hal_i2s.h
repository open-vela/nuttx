/****************************************************************************
 * arch/xtensa/src/esp32s3/hal_i2s.h
 *
 * Minimal I2S HAL layer: direct operation of ESP32-S3 I2S0 peripheral
 * registers
 * (bypassing NuttX's esp32s3_i2s driver, to avoid RX DMA clock issue)
 *
 * Key design points:
 * - Master mode, TX and RX share BCK/WS
 * - Start sequence: enable TX (silent) first, then RX to ensure BCK always
 * outputs
 * - Both TX/RX use DMA (reusing NuttX esp32s3_dma API, which has verified TX
 * functionality)
 * - RX EOF interrupt is triggered by I2S hardware (DMA receives
 * I2S_RXEOF_NUM bytes)
 *
 * Clock: PLL_F160M(160MHz) / mclk_div=13 = 12.288MHz MCLK
 * bclk = rate * 2ch * 16bit = 768kHz (24kHz stereo)
 *
 * GPIO pin configuration (verified on ESP32-S3-BOX-3):
 * MCLK=GPIO2 BCLK=GPIO17 WS=GPIO45 DOUT=GPIO15 DIN=GPIO16
 * PA_CTRL=GPIO46 (high level enables speaker amplifier)
 ****************************************************************************/

#ifndef __APPS_PLANT_COMPANION_HAL_HAL_I2S_H
#define __APPS_PLANT_COMPANION_HAL_HAL_I2S_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Pin definitions (board-level hardware) */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HAL_I2S_PIN_MCLK   2
#define HAL_I2S_PIN_BCLK  17
#define HAL_I2S_PIN_WS    45
#define HAL_I2S_PIN_DOUT  15
#define HAL_I2S_PIN_DIN   16
#define HAL_PA_PIN        46

/* Sampling parameters */

#define HAL_I2S_RATE       24000
#define HAL_I2S_BITS       16
#define HAL_I2S_CHANNELS   2   /* stereo */

/****************************************************************************
 * @brief Initialize I2S0 peripheral + GPIO matrix + PA (start TX for
 * continuous BCK output)
 * @return 0 success
 ****************************************************************************/

int hal_i2s_init(void);

/****************************************************************************
 * @brief Start TX channel (silent data, only used to generate BCK/WS clock)
 * @return 0 success
 ****************************************************************************/

int hal_i2s_start_tx_clock(void);

/****************************************************************************
 * @brief DMA receives RX data (blocking, no interrupt dependency; for
 * scenarios with active BCK)
 * @param buf receive buffer (int16_t *), size >= bytes
 * @param bytes Number of bytes to receive (integer multiple of samples)
 * @return actual bytes received, <0 failure
 ****************************************************************************/

int hal_i2s_read(void *buf, uint32_t bytes);

/****************************************************************************
 * @brief Single-slot RX read (recording only)—receives only designated
 * slot (MIC1/MIC2)
 * @param buf receive buffer (int16_t *), size >= bytes
 * @param bytes Number of bytes to receive (integer multiple of samples)
 * @param slot slot number (0=left/MIC1, 1=right/MIC2)
 * @return Actual bytes received, <0 on failure
 ****************************************************************************/

int hal_i2s_read_slot(void *buf, uint32_t bytes, int slot);

/****************************************************************************
 * @brief DMA sends TX data (blocking, waits for all to complete)
 * @param buf Send buffer
 * @param bytes number of bytes
 * @return Actual bytes sent, <0 indicates failure
 ****************************************************************************/

int hal_i2s_write(const void *buf, uint32_t bytes);

/****************************************************************************
 * @brief DMA sends TX data (asynchronous: returns after queuing, without
 * waiting for completion).
 * Windowed continuous queuing (≤3 blocks in transit) + seamless chain
 * continuation with official driver ISR, eliminating
 * Inter-block DMA gap during block-by-block completion (root cause fix for
 * pure tone 'squeak' 2026-09-02).
 * ⚠️ Driver copies data via memcpy when queued, caller may reuse buffer
 * after return.
 * @param buf send buffer.
 * @param bytes Number of bytes
 * @return Actual queued byte count, <0 failure
 ****************************************************************************/

int hal_i2s_write_async(const void *buf, uint32_t bytes);

/****************************************************************************
 * @brief Wait for all queued TX data to complete (must call before reusing
 * buf / changing direction)
 * @return 0 success; -ETIMEDOUT driver exception (status cleared).
 ****************************************************************************/

int hal_i2s_write_flush(void);

/****************************************************************************
 * @brief Simultaneous send + receive (for loopback test): start RX DMA
 * first, then start TX DMA
 * @param tx_buf Send buffer
 * @param tx_bytes Number of TX bytes
 * @param rx_buf receive buffer.
 * @param rx_bytes RX byte count
 * @return 0 success, <0 failure/timeout
 ****************************************************************************/

int hal_i2s_write_read_sync(const void *tx_buf, uint32_t tx_bytes,
                            void *rx_buf, uint32_t rx_bytes);

/****************************************************************************
 * @brief Select RX sampling slot (for diagnostics/self-test)
 * @param slot 0 = left channel (MIC1), 1 = right channel (MIC2).
 * @return 0 success
 ****************************************************************************/

int hal_i2s_rx_set_channel(int slot);

/****************************************************************************
 * @brief Diagnostics: print I2S TX status (TX_CONF/TX_CONF1/STATE/TX_CLKM)
 * Silent troubleshooting—when playback shows "complete" but silent, check
 * TX_START/STOP_EN/IDLE
 ****************************************************************************/

void hal_i2s_dump_tx(void);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_PLANT_COMPANION_HAL_HAL_I2S_H */
