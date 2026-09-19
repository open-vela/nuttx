/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_pdm.h
 *
 * RK3576 PDM (digital microphone) capture driver for the AMP slave.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_PDM_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_PDM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PDM1 is the only PDM controller with pins on the K7 40-pin header, and
 * only through its "m1" mux group:
 *
 *   pdm1m1_clk0  GPIO4_A6   shared with uart6m0_xfer (RX)
 *   pdm1m1_clk1  GPIO4_B0   shared with spi4m2_pins
 *   pdm1m1_sdi1  GPIO4_B2   shared with spi4m2_pins
 *   pdm1m1_sdi2  GPIO4_B1   shared with spi4m2_pins
 *   pdm1m1_sdi3  GPIO4_A4   shared with uart6m0_xfer (TX)
 *   pdm1m1_sdi0  GPIO4_B3   NOT routed to the header
 *
 * So claiming a PDM mic costs Linux both uart6 and spi4 (see the AMP dtsi,
 * which disables them).  We drive BOTH clock pins: the two are electrically
 * the same PDM clock, and offering the user a choice of header pin is free
 * once uart6 is gone anyway.
 *
 * Data is taken from sdi1; PDM_CLK_CTRL remaps it onto internal path 0.
 */

#define RK3576_PDM1_BASE        0x2a6e0000
#define RK3576_PDM1_IRQ         235         /* GIC_SPI 203 + 32 (unused) */

/* RK3576's PDM is the 2024 "v2" IP, not the classic Rockchip one.  Only
 * sound/soc/rockchip/rockchip_pdm_v2.c matches "rockchip,rk3576-pdm"; its
 * register map shares nothing past offset 0x0 with rockchip_pdm.h.  The
 * version register below is how rk3576_pdm_capture_init() proves it is
 * talking to the block it thinks it is.
 */

/* One PDM data line always carries two channels: the mic that ties L/R low
 * drives the samples clocked on one edge, a second mic sharing the line
 * drives the other.  The driver hands out one of them as mono.
 */

#define RK3576_PDM_SAMPLE_BITS  16
#define RK3576_PDM_CHANNELS     1

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
 * Name: rk3576_pdm_capture_init
 *
 * Description:
 *   Program PDM1's clocks, pins and registers.  Leaves the block stopped
 *   (no clock on the pin, receiver halted) — call rk3576_pdm_start().
 *
 * Returned Value:
 *   The sample rate in Hz, or a negated errno.
 *
 ****************************************************************************/

int rk3576_pdm_capture_init(void);

/****************************************************************************
 * Name: rk3576_pdm_start / rk3576_pdm_stop
 *
 * Description:
 *   Run or halt capture.  rk3576_pdm_start() busy-waits ~40 ms in total (the
 *   microphone needs time to wake after the clock appears, and the decimation
 *   filter needs time to settle), so call it from a thread, not an ISR.
 *
 ****************************************************************************/

void rk3576_pdm_start(void);
void rk3576_pdm_stop(void);

/****************************************************************************
 * Name: rk3576_pdm_read
 *
 * Description:
 *   Drain the RX FIFO into buf as 16-bit mono samples, keeping only the
 *   channel selected by rk3576_pdm_set_channel().  Non-blocking.
 *
 * Returned Value:
 *   Number of samples written.
 *
 ****************************************************************************/

size_t rk3576_pdm_read(int16_t *buf, size_t nsamples);

/****************************************************************************
 * Name: rk3576_pdm_set_channel
 *
 * Description:
 *   Choose which of the data line's two channels rk3576_pdm_read() returns
 *   (0 or 1).  Which one carries the microphone depends on how its L/R
 *   select pin is strapped; rk3576_pdm_peaks() tells you which is live.
 *
 ****************************************************************************/

void rk3576_pdm_set_channel(int ch);

/****************************************************************************
 * Name: rk3576_pdm_peaks
 *
 * Description:
 *   Peak absolute sample seen on each channel since the last call, and reset
 *   both.  Used by the MIC_INFO reply so the host can tell which channel the
 *   microphone is actually on.
 *
 ****************************************************************************/

void rk3576_pdm_peaks(uint16_t *ch0, uint16_t *ch1);

/****************************************************************************
 * Name: rk3576_pdm_overruns
 *
 * Description:
 *   Number of reads that found the RX FIFO at or above its high-water mark,
 *   i.e. samples were probably lost.
 *
 ****************************************************************************/

uint32_t rk3576_pdm_overruns(void);

/****************************************************************************
 * Name: rk3576_pdm_maxfifo
 *
 * Description:
 *   Highest FIFO level observed.  Diagnostic: the FIFO depth is not
 *   documented in the Linux driver, so this is how we learn it.
 *
 ****************************************************************************/

uint32_t rk3576_pdm_maxfifo(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3576_RK3576_PDM_H */
