/****************************************************************************
 * boards/arm/rk3588-m0/evb7-m0/include/board.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_RK3588_M0_EVB7_M0_INCLUDE_BOARD_H
#define __BOARDS_ARM_RK3588_M0_EVB7_M0_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking.
 *
 * The M0 core clock is not programmed by this port - it inherits whatever the
 * boot firmware selected, and SysTick is derived from it. On RK3588 EVB7 V11
 * that is 200MHz: FCLK_PMU_CM0_CORE comes off hclk_pmu_cm0_root, and
 * PMU_CLKSEL_CON1 (physical 0xFD7F0304) reads 0x00000400, so bits[11:10]=0b01
 * select clk_pmu1_200m_src. The mux also offers 400/100MHz and xin24m.
 */

#define BOARD_MCU_FREQUENCY CONFIG_RK3588M0_MCU_FREQUENCY

/* Heartbeat markers.
 *
 * The bare-metal bring-up firmware published a magic word plus a counter near
 * the base of the reserved region so the Linux side could confirm the core was
 * alive with /dev/mem. This port keeps the same convention and the same
 * physical addresses, so the existing checks still work:
 *
 *   busybox devmem 0x07a00800 32   -> magic
 *   busybox devmem 0x07a00804 32   -> counter, increments
 *
 * The addresses below are the M0's view; physical = 0x07a00000 + offset.
 */

#define BOARD_HEARTBEAT_MAGIC_ADDR 0x00000800
#define BOARD_HEARTBEAT_COUNT_ADDR 0x00000804
#define BOARD_HEARTBEAT_MAGIC      0x414d5030 /* "AMP0" */

/* Shared-memory window check.
 *
 * The off-chip DDR window (relocated by soc_con11 from the FIT "exsram_start"
 * property) is where rpmsg vrings will live, so it is worth proving before
 * anything depends on it. Rather than carve out new memory - and therefore
 * touch the Linux device tree - the test aims the window at an unused part of
 * the region this core already owns:
 *
 *   exsram_start   = 0x07b00000   (1MB into mcu_reserved; the image is ~23KB)
 *   M0 sees it at    0x60000000   via the DDR window
 *   and also at      0x00100000   via the code window (same physical bytes)
 *
 * Writing through one view and reading back through the other confirms the
 * window lands where intended, and Linux can check the same words at physical
 * 0x07b00000 with /dev/mem.
 *
 * Use the 0x60000000 view, not 0x80000000. Per TRM Table 9-3 both DDR rows
 * subtract 0x60000000, so the write-through alias at 0x80000000 reaches
 * base + 0x20000000, not base - which is exactly how the first attempt wrote to
 * 0x27b00000 and left the intended location untouched.
 *
 * Cache coherency is handled by the uncache range instead: uc_start/uc_end in
 * the FIT already cover this region, so accesses bypass the cache and Linux
 * sees the stores without any maintenance.
 */

#define BOARD_SHMEM_BASE           0x60000000 /* DDR window == exsram_start  */
#define BOARD_SHMEM_CODE_VIEW      0x00100000 /* same bytes via code window  */
#define BOARD_SHMEM_MAGIC          0x30535845 /* "EXS0" */

#endif /* __BOARDS_ARM_RK3588_M0_EVB7_M0_INCLUDE_BOARD_H */
