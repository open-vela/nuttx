/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_sai.h
 *
 * RK3576 SAI (Serial Audio Interface) capture driver for the AMP slave.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_SAI_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_SAI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SAI2 is free on the KickPi K7 (SAI1 belongs to Linux/ES8388).  Its
 * sai2m0 pins are the only I2S group fully exposed on the 40-pin header:
 *   SCLK  GPIO1_D1  (header pin 18)
 *   LRCK  GPIO1_D2  (header pin 26)
 *   SDI   GPIO1_D3  (header pin 28)
 * MCLK (GPIO1_D4) is NOT on the header, which is fine: I2S MEMS mics such
 * as the INMP441 derive everything from SCLK.
 */

#define RK3576_SAI2_BASE        0x2a620000
#define RK3576_SAI2_IRQ         221         /* GIC_SPI 189 + 32 */

/* Capture format produced by rk3576_sai_read(): 16-bit signed mono.
 * The wire format is 2 slots x 32 bit (standard I2S frame for a MEMS mic);
 * the driver keeps the left slot and returns its most significant 16 bits.
 */

#define RK3576_SAI_SAMPLE_BITS  16
#define RK3576_SAI_CHANNELS     1

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Name: rk3576_sai_capture_init
 *
 * Description:
 *   Bring up SAI2 as an I2S master and start capturing from SDI.  Programs
 *   the clocks, pin mux and the SAI itself; after this call the SCLK/LRCK
 *   pins are toggling and the RX FIFO is filling.
 *
 * Returned Value:
 *   The actual sample rate in Hz (it is derived from the 24 MHz crystal, so
 *   it is close to but not exactly the nominal rate), or a negated errno.
 *
 ****************************************************************************/

int rk3576_sai_capture_init(void);

/****************************************************************************
 * Name: rk3576_sai_read
 *
 * Description:
 *   Drain whatever the RX FIFO holds into buf as 16-bit mono samples.
 *   Non-blocking: returns 0 when the FIFO is empty.
 *
 * Input Parameters:
 *   buf      - destination for int16_t samples
 *   nsamples - capacity of buf, in samples
 *
 * Returned Value:
 *   Number of samples written.
 *
 ****************************************************************************/

size_t rk3576_sai_read(int16_t *buf, size_t nsamples);

/****************************************************************************
 * Name: rk3576_sai_start / rk3576_sai_stop
 *
 * Description:
 *   Run or halt the receiver.  Capture is left stopped by
 *   rk3576_sai_capture_init(): while nothing drains the FIFO the controller
 *   would report a continuous overrun.
 *
 ****************************************************************************/

void rk3576_sai_start(void);
void rk3576_sai_stop(void);

/****************************************************************************
 * Name: rk3576_sai_overruns
 *
 * Description:
 *   Number of RX overruns seen since init (FIFO not drained fast enough).
 *
 ****************************************************************************/

uint32_t rk3576_sai_overruns(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3576_RK3576_SAI_H */
