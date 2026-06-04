/****************************************************************************
 * boards/arm/t113/t113-evb/src/t113-evb.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __BOARDS_ARM_T113_T113_EVB_SRC_T113_EVB_H
#define __BOARDS_ARM_T113_T113_EVB_SRC_T113_EVB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

int t113_bringup(void);

#ifdef CONFIG_T113_USBHOST
int board_usbhost_initialize(void);
#endif

#ifdef CONFIG_RPTUN_BMP
/* Provided by t113_rptun_slave.c (CONFIG_T113_RPTUN_SLAVE) or
 * t113_rptun_bmp.c (BMP runtime role); the two are build-time exclusive.
 */

int t113_rptun_init(void);
#endif

#ifdef CONFIG_T113_RPTUN_DSP
void t113_dsp_uart_prepare(void);
#endif

#ifdef CONFIG_T113_RPTUN_MASTER
int t113_rptun_master_init(void);
#endif

#ifdef CONFIG_LCD_GC9503CV_DSI
int t113_lcd_initialize(void);
#endif

#ifdef CONFIG_BMP
/* Declared under CONFIG_SMP in include/nuttx/arch.h; BMP needs it too
 * but doesn't select CONFIG_SMP.  Strong definition lives in
 * arch/arm/src/t113/t113_cpuboot.c.
 */

int up_cpu_start(int cpu);
#endif

#ifdef CONFIG_BOARDCTL_BOOT_IMAGE
struct mtd_dev_s;
FAR struct mtd_dev_s *t113_get_mtd_part(int mtdn);
#endif

/****************************************************************************
 * Name: t113_boot_jump
 *
 * Description:
 *   Hand off control to a loaded image.  Never returns.  <entry> is
 *   the branch target.  <boot_arg> is shoveled into r2 verbatim and
 *   interpreted by the chained image (typically a DTB physical address
 *   under the ARM Linux DT calling convention; pass 0 for the legacy
 *   NuttX raw-boot convention).
 *
 ****************************************************************************/

void t113_boot_jump(uintptr_t entry, uintptr_t boot_arg) noreturn_function;

#ifdef CONFIG_T113_CTP
int t113_ctp_initialize(const char *devpath);
#endif

#endif /* __BOARDS_ARM_T113_T113_EVB_SRC_T113_EVB_H */
