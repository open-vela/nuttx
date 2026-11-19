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

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/mount.h>
#include <syslog.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

extern struct sdio_dev_s *sdio_initialize(int slotno);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BOARD_SDMMC_SLOTNO 0
#define BOARD_SDMMC_MINOR  0
#define BOARD_SDMMC_DEVPATH "/dev/mmcsd0"
#define BOARD_SDMMC_PARTPATH "/dev/mmcsd0p1"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t board_sdmmc_getle16(FAR const uint8_t *buffer)
{
  return (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
}

static uint32_t board_sdmmc_getle32(FAR const uint8_t *buffer)
{
  return (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) |
         ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 24);
}

static void board_sdmmc_warmup(void)
{
  FAR struct inode *inode;
  struct geometry geo;
  FAR uint8_t *buffer;
  uint8_t type = 0;
  uint32_t start = 0;
  uint32_t blocks = 0;
  uint16_t sig = 0;
  ssize_t nread;
  int rv;
  int i;

  rv = open_blockdriver(BOARD_SDMMC_DEVPATH, MS_RDONLY, &inode);
  if (rv < 0)
    {
      syslog(LOG_WARNING, "SDMMC warmup open failed: %d\n", rv);
      return;
    }

  syslog(LOG_INFO, "SDMMC warmup opened %s\n", BOARD_SDMMC_DEVPATH);

  rv = inode->u.i_bops->geometry(inode, &geo);
  if (rv < 0 || geo.geo_sectorsize < 512 || geo.geo_nsectors == 0)
    {
      syslog(LOG_WARNING, "SDMMC warmup geometry failed: %d\n", rv);
      close_blockdriver(inode);
      return;
    }

  syslog(LOG_INFO, "SDMMC warmup geo sectorsize=%d nsectors=%" PRIuOFF "\n",
         geo.geo_sectorsize, geo.geo_nsectors);

  buffer = kmm_malloc(geo.geo_sectorsize);
  if (buffer == NULL)
    {
      close_blockdriver(inode);
      return;
    }

  for (i = 0; i < 3; i++)
    {
      nread = inode->u.i_bops->read(inode, buffer, 0, 1);
      if (nread == 1)
        {
          sig = board_sdmmc_getle16(&buffer[510]);
          type = buffer[0x1c2];
          start = board_sdmmc_getle32(&buffer[0x1c6]);
          blocks = board_sdmmc_getle32(&buffer[0x1ca]);
          syslog(LOG_INFO,
                 "SDMMC warmup mbr try=%d sig=%04x type=%02x start=%" PRIu32
                 " blocks=%" PRIu32 "\n",
                 i, sig, type, start, blocks);
          if (sig == 0xaa55 && type != 0 && start != 0)
            {
              break;
            }
        }

      up_mdelay(20);
    }

  if (sig == 0xaa55 && start > 0 && start < geo.geo_nsectors)
    {
      syslog(LOG_INFO, "SDMMC warmup partition sector=%" PRIu32 "\n", start);
      inode->u.i_bops->read(inode, buffer, start, 1);

      if (type != 0 && blocks > 0 &&
          blocks <= geo.geo_nsectors - start)
        {
          rv = register_blockpartition(BOARD_SDMMC_PARTPATH, 0660,
                                       BOARD_SDMMC_DEVPATH, start, blocks);
          if (rv < 0 && rv != -EEXIST)
            {
              syslog(LOG_WARNING,
                     "SDMMC partition register %s failed: %d\n",
                     BOARD_SDMMC_PARTPATH, rv);
            }
          else
            {
              syslog(LOG_INFO,
                     "Registered SDMMC partition %s start=%" PRIu32
                     " blocks=%" PRIu32 "\n",
                     BOARD_SDMMC_PARTPATH, start, blocks);
            }
        }
      else
        {
          syslog(LOG_WARNING,
                 "SDMMC warmup partition invalid type=%02x blocks=%" PRIu32
                 " start=%" PRIu32 "\n",
                 type, blocks, start);
        }
    }
  else
    {
      syslog(LOG_WARNING,
             "SDMMC warmup no valid MBR sig=%04x start=%" PRIu32
             " nsectors=%" PRIuOFF "\n",
             sig, start, geo.geo_nsectors);
    }

  syslog(LOG_INFO,
         "SDMMC warmup sig=%04x type=%02x start=%" PRIu32
         " blocks=%" PRIu32 " total=%" PRIuOFF "\n",
         sig, type, start, blocks, geo.geo_nsectors);

  kmm_free(buffer);
  close_blockdriver(inode);
}

static void board_sdmmc_mount(void)
{
  int rv;

  if (mkdir("/data", 0755) < 0 && errno != EEXIST)
    {
      syslog(LOG_WARNING, "SDMMC mkdir /data failed: %d\n", errno);
    }

  rv = nx_mount(BOARD_SDMMC_PARTPATH, "/data", "vfat", 0, NULL);
  if (rv < 0)
    {
      syslog(LOG_WARNING, "SDMMC mount /data failed: %d\n", rv);
    }
  else
    {
      syslog(LOG_INFO, "SDMMC mounted %s at /data\n", BOARD_SDMMC_PARTPATH);
    }

  syslog(LOG_INFO, "SDMMC mount check done for /data\n");
}

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
  int rv;
  static bool initialized;

  if (initialized)
    {
      syslog(LOG_INFO, "SDMMC already initialized\n");
      return OK;
    }

  syslog(LOG_INFO, "SDMMC init slot=%d minor=%d cmd=%d clk=%d d0=%d\n",
         BOARD_SDMMC_SLOTNO, BOARD_SDMMC_MINOR, CONFIG_ESP32S3_SDMMC_CMD,
         CONFIG_ESP32S3_SDMMC_CLK, CONFIG_ESP32S3_SDMMC_D0);

  up_mdelay(500);

  sdio = sdio_initialize(BOARD_SDMMC_SLOTNO);
  if (!sdio)
    {
      syslog(LOG_ERR, "Failed to initialize SDIO slot\n");
      return -ENODEV;
    }

  syslog(LOG_INFO, "SDMMC slot %d initialized\n", BOARD_SDMMC_SLOTNO);

  rv = mmcsd_slotinitialize(BOARD_SDMMC_MINOR, sdio);
  if (rv < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SDMMC slot: %d\n", rv);
      return rv;
    }

  syslog(LOG_INFO, "SDMMC mmcsd slot register ok minor=%d\n",
         BOARD_SDMMC_MINOR);

  board_sdmmc_warmup();
  board_sdmmc_mount();

  initialized = true;

  return OK;
}
