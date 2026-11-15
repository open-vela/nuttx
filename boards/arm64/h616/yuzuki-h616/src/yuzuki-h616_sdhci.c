/****************************************************************************
 * boards/arm64/h616/yuzuki-h616/src/yuzuki-h616_sdhci.c
 *
 * Board-level wiring for the H616 eMMC (sdc2) SDIO controller.
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

#include "yuzuki-h616.h"
#include "h616_sdhci.h"

#ifdef CONFIG_H616_SDHCI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define H616_SDIO_SLOTNO   0  /* sdc2 = slot 0 */
#define H616_SDIO_MINOR    0  /* /dev/mmcsd0 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: yuzuki_h616_sdhci_initialize
 *
 * Description:
 *   Initialize the SDIO interface for the on-board eMMC (sdc2) and bind it
 *   to the MMC/SD driver.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 *
 ****************************************************************************/

int yuzuki_h616_sdhci_initialize(void)
{
  FAR struct sdio_dev_s *sdio_dev;
  int ret;

  _info("Initializing SDIO slot %d\n", H616_SDIO_SLOTNO);

  sdio_dev = sdio_initialize(H616_SDIO_SLOTNO);
  if (sdio_dev == NULL)
    {
      _err("ERROR: Failed to initialize SDIO slot %d\n", H616_SDIO_SLOTNO);
      return -ENODEV;
    }

  _info("Bind SDIO to the MMC/SD driver, minor=%d\n", H616_SDIO_MINOR);

  ret = mmcsd_slotinitialize(H616_SDIO_MINOR, sdio_dev);
  if (ret != OK)
    {
      _err("ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
      return ret;
    }

  /* eMMC is non-removable and always present */

  sdio_mediachange(sdio_dev, true);

  _info("Successfully bound SDIO to the MMC/SD driver\n");
  return OK;
}

#endif /* CONFIG_H616_SDHCI */
