/****************************************************************************
 * boards/xtensa/esp32s3/common/src/esp32s3_board_sdmmc.c
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

#include <sys/stat.h>
#include <unistd.h>
#include <syslog.h>
#include <debug.h>

#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

extern struct sdio_dev_s *sdio_initialize(int slotno);

#define ESP32S3_SDMMC_DEVPATH         "/dev/mmcsd1"
#define ESP32S3_SDMMC_PROBE_RETRIES   15
#define ESP32S3_SDMMC_PROBE_DELAY_US  500000
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_sdmmc_initialize
 *
 * Description:
 *   Configure the sdmmc subsystem.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_sdmmc_initialize(void)
{
  struct sdio_dev_s *sdio;
  struct stat st;
  int retry;
  int rv;

  sdio = sdio_initialize(1);
  if (!sdio)
    {
      syslog(LOG_ERR, "Failed to initialize SDIO slot\n");
      return -ENODEV;
    }

  rv = mmcsd_slotinitialize(1, sdio);
  if (rv < 0)
    {
      syslog(LOG_ERR, "Failed to bind SDIO port to SD slot: %d\n", rv);
      return rv;
    }

  /* This board has no separate card-detect GPIO; the controller card-
   * detect input is tied to the present state.  If the first identification
   * attempt fails there can be no later insertion edge to trigger another
   * probe.  Re-enable the existing media callback instead of initializing
   * another SDIO/MMCSD instance.
   */

  for (retry = 0; retry <= ESP32S3_SDMMC_PROBE_RETRIES; retry++)
    {
      if (stat(ESP32S3_SDMMC_DEVPATH, &st) == 0)
        {
          if (retry > 0)
            {
              syslog(LOG_INFO, "SD card ready after %d reprobes\n", retry);
            }

          return OK;
        }

      if (retry == ESP32S3_SDMMC_PROBE_RETRIES)
        {
          break;
        }

      usleep(ESP32S3_SDMMC_PROBE_DELAY_US);
      SDIO_CALLBACKENABLE(sdio, SDIOMEDIA_INSERTED);
    }

  syslog(LOG_ERR, "SD card probe failed after %d retries\n",
         ESP32S3_SDMMC_PROBE_RETRIES);
  return -ENODEV;
}
