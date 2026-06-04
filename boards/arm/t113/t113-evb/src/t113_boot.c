/****************************************************************************
 * boards/arm/t113/t113-evb/src/t113_boot.c
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
#include <nuttx/board.h>
#include <debug.h>
#include "t113-evb.h"
#include "t113_fel.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void t113_boardinitialize(void)
{
#ifdef CONFIG_T113_BOOT0
  /* FEL-rescue guard owned by the boot0 SPL: boot0 is the earliest
   * stage in the chain (BROM -> boot0 -> AP) and the smallest /
   * most stable image, so the counter sits at the most reliable
   * point to catch crash loops - including ones where the AP image
   * fails to start at all.  AP build leaves this disabled
   * (CONFIG_T113_FEL_RESCUE_THRESHOLD=0 by Kconfig default) so
   * boot0+AP do not double-count on every panic+boot cycle.
   *
   * Compiles to a no-op when CONFIG_T113_FEL_RESCUE_THRESHOLD = 0.
   */

  t113_fel_rescue();
#endif
}

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  t113_bringup();
}
#endif
