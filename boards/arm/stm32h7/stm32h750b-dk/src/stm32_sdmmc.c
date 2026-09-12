/****************************************************************************
 * boards/arm/stm32h7/stm32h750b-dk/src/stm32_sdmmc.c
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include <arch/board/board.h>

#include "stm32_gpio.h"
#include "stm32_sdmmc.h"

#include "stm32h750b-dk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32H7_SDMMC1
#  error SDMMC1 supported only (on-board eMMC)
#endif

#if CONFIG_MM_REGIONS > 1 && defined(CONFIG_STM32H7_SDMMC_IDMA)
#  error SDMMC1 with IDMA does not work with CONFIG_MM_REGIONS > 1
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct sdio_dev_s *g_sdio_dev;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support for the soldered eMMC.
 *   No card-detect GPIO — always report media present.
 *
 ****************************************************************************/

int stm32_sdio_initialize(void)
{
  int ret;

  /* Arm D4–D7 as well.  sdio_initialize() only configures D0–D3/CK/CMD.
   * Stock NuttX MMC path still runs 1-bit until a wider-bus fix lands.
   */

  stm32_configgpio(GPIO_SDMMC1_D4);
  stm32_configgpio(GPIO_SDMMC1_D5);
  stm32_configgpio(GPIO_SDMMC1_D6);
  stm32_configgpio(GPIO_SDMMC1_D7);

  finfo("Initializing SDIO slot %d (eMMC)\n", SDIO_SLOTNO);

  g_sdio_dev = sdio_initialize(SDIO_SLOTNO);
  if (!g_sdio_dev)
    {
      ferr("ERROR: Failed to initialize SDIO slot %d\n", SDIO_SLOTNO);
      return -ENODEV;
    }

  finfo("Bind SDIO to the MMC/SD driver, minor=%d\n", SDIO_MINOR);

  ret = mmcsd_slotinitialize(SDIO_MINOR, g_sdio_dev);
  if (ret != OK)
    {
      ferr("ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
      return ret;
    }

  /* Soldered eMMC — no CD pin */

  sdio_mediachange(g_sdio_dev, true);

  return OK;
}
