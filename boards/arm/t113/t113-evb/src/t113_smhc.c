/****************************************************************************
 * boards/arm/t113/t113-evb/src/t113_smhc.c
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

#include <debug.h>
#include <errno.h>

#include <nuttx/mmcsd.h>
#include <nuttx/sdio.h>

#include "t113_smhc.h"
#include "t113-evb.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_board_smhc_initialize
 *
 * Description:
 *   Register SMHC0 with the MMC/SD block-device upper half so that
 *   /dev/mmcsd0 appears at boot.  SMHC1 is reserved for Phase 2
 *   (RTL8723DS WiFi SDIO) and stays unregistered until that phase
 *   wires its own adapter.
 *
 ****************************************************************************/

int t113_board_smhc_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_T113_SMHC0
  struct sdio_dev_s *dev0 = t113_smhc_initialize(0);
  if (dev0 == NULL)
    {
      mcerr("ERROR: t113_smhc_initialize(0) failed\n");
      return -ENODEV;
    }

  ret = mmcsd_slotinitialize(0, dev0);
  if (ret < 0)
    {
      mcerr("ERROR: mmcsd_slotinitialize(0) failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_T113_SMHC1
  /* Phase 2: SMHC1 sdio_dev_s will be consumed by the rtw88/RTL8723DS
   * adapter rather than mmcsd_slotinitialize().  Left unregistered
   * here until that phase lands its own board hook.
   */
#endif

  return ret;
}
