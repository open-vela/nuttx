/****************************************************************************
 * boards/arm/stm32/alientek-m144z-m4/src/stm32_w25.c
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

#include <errno.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>

#include "stm32_spi.h"
#include "alientek-m144z-m4.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_w25initialize
 *
 * Description:
 *   Initialize the on-board W25Q128 NOR flash on SPI1 and register it as
 *   /dev/w25 (raw MTD character device).  When CONFIG_FS_LITTLEFS is
 *   selected the device is also mounted at /data.
 *
 * Input Parameters:
 *   minor - Minor device number (not used; kept for API symmetry).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_w25initialize(int minor)
{
  FAR struct spi_dev_s *spi;
  FAR struct mtd_dev_s *mtd;
  int ret;

  UNUSED(minor);

  /* Get the SPI1 interface */

  spi = stm32_spibus_initialize(1);
  if (spi == NULL)
    {
      syslog(LOG_ERR, "ERROR: stm32_spibus_initialize(1) failed\n");
      return -ENODEV;
    }

  /* Bind the SPI interface to the W25 driver */

  mtd = w25_initialize(spi);
  if (mtd == NULL)
    {
      syslog(LOG_ERR, "ERROR: w25_initialize() failed\n");
      return -ENODEV;
    }

  /* Register the MTD driver as /dev/w25 */

  ret = register_mtddriver("/dev/w25", mtd, 0755, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: register_mtddriver(/dev/w25) failed: %d\n",
             ret);
      return ret;
    }

#ifdef CONFIG_FS_LITTLEFS

  /* Mount LittleFS on the raw MTD node.  autoformat triggers lfs_format()
   * on first boot when the flash has no valid superblock.
   */

  ret = nx_mount("/dev/w25", "/data", "littlefs", 0, "autoformat");
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: nx_mount littlefs /dev/w25 -> /data failed: %d\n",
             ret);
    }
#endif

  return OK;
}
