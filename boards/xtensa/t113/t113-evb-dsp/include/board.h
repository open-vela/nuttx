/****************************************************************************
 * boards/xtensa/t113/t113-evb-dsp/include/board.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __BOARDS_XTENSA_T113_T113_EVB_DSP_INCLUDE_BOARD_H
#define __BOARDS_XTENSA_T113_T113_EVB_DSP_INCLUDE_BOARD_H

/* APB1 = PLL_PERI0_1X / 2 / 3 = 100 MHz (set by AP-side).
 * UART2 baud divisor = APB1 / (16 * baud)
 *                    = 100_000_000 / (16 * 115200) = 54.
 */

#define BOARD_APB1_FREQUENCY       100000000

/* DSP core clock - drives CCOUNT and the system tick.  Reset default is
 * 600 MHz; the AP-side may reprogram before DSP release.  Override via
 * board.h on a per-board basis if needed.
 *
 * BOARD_CLOCK_FREQUENCY is the name the upstream xtensa_timer.h expects
 * for the CCOUNT/CCOMPARE tick base; alias it to the DSP core clock.
 */

#define BOARD_DSP_CLOCK_FREQUENCY  600000000
#define BOARD_CLOCK_FREQUENCY      BOARD_DSP_CLOCK_FREQUENCY

#endif /* __BOARDS_XTENSA_T113_T113_EVB_DSP_INCLUDE_BOARD_H */
