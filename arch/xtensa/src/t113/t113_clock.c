/****************************************************************************
 * arch/xtensa/src/t113/t113_clock.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * UART2 / DSP_INTC clock + pin mux bring-up.
 *
 * Currently a no-op.  AP-side boards/arm/t113/t113-evb/src/t113_dsp_uart.c
 * configures PIO PE2/PE3 (mux 3) and the CCU UART2_BGR clock gate before
 * the AP triggers DSP execution, so by the time this function runs the
 * UART2 IP is already clocked and pin-muxed.  Future work: relocate that
 * setup DSP-side once a 9pfs / shmem channel makes it cheap to do so.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void t113_clock_init(void)
{
}
