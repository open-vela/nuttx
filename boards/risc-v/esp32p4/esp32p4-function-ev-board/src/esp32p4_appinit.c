/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/esp32p4_appinit.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>

#include "esp32p4-function-ev-board.h"

/* RAM debug marker survives warm resets at this fixed SRAM address; print the
 * previous run's last marker at every boot to aid post-hang diagnosis. */

#define DBG_MARK_ADDR ((volatile uint32_t *)0x4ffae000)

#ifdef CONFIG_BOARDCTL

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
  uint32_t m = *DBG_MARK_ADDR;
  static const char hex[] = "0123456789abcdef";

  /* Polled output via up_putc: printf at this boot stage is dropped. */

  up_putc('M');                       /* 'M' = marker dump follows */
  up_putc(hex[(m >> 28) & 0xF]);
  up_putc(hex[(m >> 24) & 0xF]);
  up_putc(hex[(m >> 20) & 0xF]);
  up_putc(hex[(m >> 16) & 0xF]);
  up_putc(hex[(m >> 12) & 0xF]);
  up_putc(hex[(m >>  8) & 0xF]);
  up_putc(hex[(m >>  4) & 0xF]);
  up_putc(hex[m & 0xF]);
  up_putc('\n');
  return OK;
}

#endif /* CONFIG_BOARDCTL */
